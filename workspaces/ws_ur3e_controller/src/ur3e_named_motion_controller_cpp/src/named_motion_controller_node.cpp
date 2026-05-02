#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"
#include "ur_dashboard_msgs/srv/get_robot_mode.hpp"
#include "ur_dashboard_msgs/srv/get_safety_mode.hpp"
#include "ur_dashboard_msgs/srv/is_in_remote_control.hpp"
#include "ur_dashboard_msgs/srv/is_program_running.hpp"
#include "ur3e_controller_msgs/srv/execute_named_target.hpp"
#include "yaml-cpp/yaml.h"

namespace
{
using ExecuteNamedTarget = ur3e_controller_msgs::srv::ExecuteNamedTarget;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using GetRobotMode = ur_dashboard_msgs::srv::GetRobotMode;
using GetSafetyMode = ur_dashboard_msgs::srv::GetSafetyMode;
using IsInRemoteControl = ur_dashboard_msgs::srv::IsInRemoteControl;
using IsProgramRunning = ur_dashboard_msgs::srv::IsProgramRunning;
using namespace std::chrono_literals;

struct NamedTarget
{
  bool enabled{false};
  std::string reviewed_by;
  std::vector<double> positions_rad;
};

struct ModeConfig
{
  std::string planning_group{"ur_manipulator"};
  std::vector<std::string> joint_names;
  std::unordered_map<std::string, NamedTarget> targets;
  std::vector<std::string> required_active_controllers;
  double max_joint_delta_rad{0.10};
  double final_position_tolerance_rad{0.02};
  double final_state_timeout_sec{5.0};
  double planning_time_sec{3.0};
  int planning_attempts{3};
  double max_velocity_scaling{0.05};
  double max_acceleration_scaling{0.05};
  double min_speed_scaling{0.01};
  bool require_joint_state_stamp{true};
  bool require_remote_control{true};
  bool allow_reduced_safety_mode{false};
};

struct JointStateSnapshot
{
  sensor_msgs::msg::JointState msg;
  std::chrono::steady_clock::time_point received_at;
  std::size_t sequence{0};
};

std::string join(const std::vector<std::string> & values, const std::string & separator)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << separator;
    }
    stream << values[index];
  }
  return stream.str();
}

std::string format_double(const double value)
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed, std::ios::floatfield);
  stream.precision(6);
  stream << value;
  return stream.str();
}

std::vector<std::string> yaml_string_vector(const YAML::Node & node)
{
  std::vector<std::string> values;
  if (!node || !node.IsSequence()) {
    return values;
  }
  values.reserve(node.size());
  for (const auto & item : node) {
    values.push_back(item.as<std::string>());
  }
  return values;
}

std::vector<double> yaml_double_vector(const YAML::Node & node)
{
  std::vector<double> values;
  if (!node || !node.IsSequence()) {
    return values;
  }
  values.reserve(node.size());
  for (const auto & item : node) {
    values.push_back(item.as<double>());
  }
  return values;
}

double yaml_double_or(const YAML::Node & node, const char * key, const double fallback)
{
  return node[key] ? node[key].as<double>() : fallback;
}

int yaml_int_or(const YAML::Node & node, const char * key, const int fallback)
{
  return node[key] ? node[key].as<int>() : fallback;
}

bool yaml_bool_or(const YAML::Node & node, const char * key, const bool fallback)
{
  return node[key] ? node[key].as<bool>() : fallback;
}

std::string yaml_string_or(const YAML::Node & node, const char * key, const std::string & fallback)
{
  return node[key] ? node[key].as<std::string>() : fallback;
}

bool stamp_is_zero(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}
}  // namespace

class Ur3eNamedMotionController : public rclcpp::Node
{
public:
  Ur3eNamedMotionController()
  : Node("ur3e_named_motion_controller")
  {
    runtime_mode_ = this->declare_parameter<std::string>("runtime_mode", "sim");
    target_catalog_path_ = this->declare_parameter<std::string>("target_catalog", "");
    allow_execution_ = this->declare_parameter<bool>("allow_execution", false);
    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    joint_state_timeout_sec_ = this->declare_parameter<double>("joint_state_timeout_sec", 3.0);
    max_joint_state_age_sec_ = this->declare_parameter<double>("max_joint_state_age_sec", 1.0);
    speed_scaling_topic_ = this->declare_parameter<std::string>(
      "speed_scaling_topic", "/speed_scaling_state_broadcaster/speed_scaling");
    human_confirmation_token_ = this->declare_parameter<std::string>(
      "human_confirmation_token", "I_CONFIRM_REAL_ROBOT_MOTION");
    robot_mode_service_name_ = this->declare_parameter<std::string>(
      "robot_mode_service", "/dashboard_client/get_robot_mode");
    safety_mode_service_name_ = this->declare_parameter<std::string>(
      "safety_mode_service", "/dashboard_client/get_safety_mode");
    program_running_service_name_ = this->declare_parameter<std::string>(
      "program_running_service", "/dashboard_client/program_running");
    remote_control_service_name_ = this->declare_parameter<std::string>(
      "remote_control_service", "/dashboard_client/is_in_remote_control");
    list_controllers_service_name_ = this->declare_parameter<std::string>(
      "list_controllers_service", "/controller_manager/list_controllers");
    service_timeout_sec_ = this->declare_parameter<double>("service_timeout_sec", 3.0);

    state_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    real_gate_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions state_options;
    state_options.callback_group = state_callback_group_;
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        on_joint_state(msg);
      },
      state_options);
    speed_scaling_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      speed_scaling_topic_,
      10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        on_speed_scaling(msg);
      },
      state_options);

    robot_mode_client_ =
      this->create_client<GetRobotMode>(
      robot_mode_service_name_, rclcpp::ServicesQoS(), real_gate_callback_group_);
    safety_mode_client_ =
      this->create_client<GetSafetyMode>(
      safety_mode_service_name_, rclcpp::ServicesQoS(), real_gate_callback_group_);
    program_running_client_ = this->create_client<IsProgramRunning>(
      program_running_service_name_, rclcpp::ServicesQoS(), real_gate_callback_group_);
    remote_control_client_ = this->create_client<IsInRemoteControl>(
      remote_control_service_name_, rclcpp::ServicesQoS(), real_gate_callback_group_);
    list_controllers_client_ = this->create_client<ListControllers>(
      list_controllers_service_name_, rclcpp::ServicesQoS(), real_gate_callback_group_);

    load_catalog();

    execute_service_ = this->create_service<ExecuteNamedTarget>(
      "~/execute_named_target",
      [this](
        const std::shared_ptr<ExecuteNamedTarget::Request> request,
        std::shared_ptr<ExecuteNamedTarget::Response> response) {
        on_execute_named_target(request, response);
      },
      rclcpp::ServicesQoS(),
      service_callback_group_);

    RCLCPP_INFO(
      this->get_logger(),
      "UR3e named motion controller started. runtime_mode=%s allow_execution=%s service=%s",
      runtime_mode_.c_str(),
      allow_execution_ ? "true" : "false",
      execute_service_->get_service_name());
    RCLCPP_WARN(
      this->get_logger(),
      "V1 only executes one reviewed named joint target per request; it does not run Servo, waypoint queues, or recovery Dashboard commands.");
  }

  ~Ur3eNamedMotionController() override
  {
    move_group_.reset();
  }

private:
  struct BusyReset
  {
    explicit BusyReset(Ur3eNamedMotionController * owner_in)
    : owner(owner_in)
    {
    }

    ~BusyReset()
    {
      std::lock_guard<std::mutex> lock(owner->busy_mutex_);
      owner->busy_ = false;
    }

    Ur3eNamedMotionController * owner;
  };

  void load_catalog()
  {
    try {
      if (target_catalog_path_.empty()) {
        catalog_error_ = "target_catalog parameter is empty";
        RCLCPP_ERROR(this->get_logger(), "%s", catalog_error_.c_str());
        return;
      }

      const YAML::Node root = YAML::LoadFile(target_catalog_path_);
      const YAML::Node mode = root["runtime_modes"][runtime_mode_];
      if (!mode) {
        catalog_error_ = "runtime_mode '" + runtime_mode_ + "' not found in target catalog";
        RCLCPP_ERROR(this->get_logger(), "%s", catalog_error_.c_str());
        return;
      }

      mode_config_.planning_group =
        yaml_string_or(mode, "planning_group", mode_config_.planning_group);
      mode_config_.joint_names = yaml_string_vector(mode["joint_names"]);
      mode_config_.max_joint_delta_rad =
        yaml_double_or(mode, "max_joint_delta_rad", mode_config_.max_joint_delta_rad);
      mode_config_.final_position_tolerance_rad = yaml_double_or(
        mode, "final_position_tolerance_rad", mode_config_.final_position_tolerance_rad);
      mode_config_.final_state_timeout_sec =
        yaml_double_or(mode, "final_state_timeout_sec", mode_config_.final_state_timeout_sec);
      mode_config_.planning_time_sec =
        yaml_double_or(mode, "planning_time_sec", mode_config_.planning_time_sec);
      mode_config_.planning_attempts =
        yaml_int_or(mode, "planning_attempts", mode_config_.planning_attempts);
      mode_config_.max_velocity_scaling =
        yaml_double_or(mode, "max_velocity_scaling", mode_config_.max_velocity_scaling);
      mode_config_.max_acceleration_scaling =
        yaml_double_or(mode, "max_acceleration_scaling", mode_config_.max_acceleration_scaling);
      mode_config_.min_speed_scaling =
        yaml_double_or(mode, "min_speed_scaling", mode_config_.min_speed_scaling);
      mode_config_.require_joint_state_stamp = yaml_bool_or(
        mode, "require_joint_state_stamp", mode_config_.require_joint_state_stamp);
      mode_config_.require_remote_control =
        yaml_bool_or(mode, "require_remote_control", mode_config_.require_remote_control);
      mode_config_.allow_reduced_safety_mode = yaml_bool_or(
        mode, "allow_reduced_safety_mode", mode_config_.allow_reduced_safety_mode);
      mode_config_.required_active_controllers =
        yaml_string_vector(mode["required_active_controllers"]);

      const YAML::Node targets = mode["targets"];
      if (!targets || !targets.IsMap()) {
        catalog_error_ = "target catalog has no targets map for runtime_mode '" + runtime_mode_ +
          "'";
        RCLCPP_ERROR(this->get_logger(), "%s", catalog_error_.c_str());
        return;
      }

      for (const auto & item : targets) {
        const std::string name = item.first.as<std::string>();
        const YAML::Node target_node = item.second;
        NamedTarget target;
        target.enabled = yaml_bool_or(target_node, "enabled", true);
        target.reviewed_by = yaml_string_or(target_node, "reviewed_by", "");
        target.positions_rad = yaml_double_vector(target_node["positions_rad"]);
        if (target.positions_rad.size() != mode_config_.joint_names.size()) {
          catalog_error_ = "target '" + name + "' position count does not match joint_names";
          RCLCPP_ERROR(this->get_logger(), "%s", catalog_error_.c_str());
          return;
        }
        mode_config_.targets.emplace(name, std::move(target));
      }

      if (mode_config_.joint_names.empty()) {
        catalog_error_ = "joint_names is empty for runtime_mode '" + runtime_mode_ + "'";
        RCLCPP_ERROR(this->get_logger(), "%s", catalog_error_.c_str());
        return;
      }

      catalog_loaded_ = true;
      std::vector<std::string> target_names;
      target_names.reserve(mode_config_.targets.size());
      for (const auto & item : mode_config_.targets) {
        target_names.push_back(item.first);
      }
      std::sort(target_names.begin(), target_names.end());

      RCLCPP_INFO(
        this->get_logger(),
        "Loaded target catalog: path=%s runtime_mode=%s planning_group=%s targets=[%s]",
        target_catalog_path_.c_str(),
        runtime_mode_.c_str(),
        mode_config_.planning_group.c_str(),
        join(target_names, ", ").c_str());
    } catch (const YAML::Exception & error) {
      catalog_error_ = std::string("failed to parse target catalog: ") + error.what();
      RCLCPP_ERROR(this->get_logger(), "%s", catalog_error_.c_str());
    }
  }

  void on_execute_named_target(
    const std::shared_ptr<ExecuteNamedTarget::Request> request,
    std::shared_ptr<ExecuteNamedTarget::Response> response)
  {
    std::unique_lock<std::mutex> busy_lock(busy_mutex_);
    if (busy_) {
      reject(response, "rejected_busy", "controller is already processing one named target");
      return;
    }
    busy_ = true;
    busy_lock.unlock();
    BusyReset busy_reset(this);

    RCLCPP_INFO(
      this->get_logger(),
      "Received ExecuteNamedTarget request: target_name=%s execute=%s runtime_mode=%s",
      request->target_name.c_str(),
      request->execute ? "true" : "false",
      runtime_mode_.c_str());

    if (!catalog_loaded_) {
      reject(response, "rejected_catalog", catalog_error_);
      return;
    }

    const auto target_iter = mode_config_.targets.find(request->target_name);
    if (target_iter == mode_config_.targets.end()) {
      reject(
        response,
        "rejected_unknown_target",
        "target_name is not present in the reviewed catalog: " + request->target_name);
      return;
    }

    const NamedTarget & target = target_iter->second;
    if (!target.enabled) {
      reject(
        response,
        "rejected_disabled_target",
        "target is present but disabled; reviewed_by='" + target.reviewed_by + "'");
      return;
    }

    if (request->execute && !allow_execution_) {
      reject(
        response,
        "rejected_execution_disabled",
        "launch parameter execute/allow_execution is false, so this node accepts plan-only requests");
      return;
    }

    std::string current_state_message;
    const auto current_positions = get_current_positions(current_state_message);
    if (!current_positions.has_value()) {
      reject(response, "rejected_joint_state", current_state_message);
      return;
    }

    std::string delta_message;
    if (!passes_delta_gate(current_positions.value(), target, delta_message)) {
      reject(response, "rejected_delta", delta_message);
      return;
    }

    if (!ensure_move_group(response)) {
      return;
    }

    std::map<std::string, double> target_by_joint;
    for (std::size_t index = 0; index < mode_config_.joint_names.size(); ++index) {
      target_by_joint[mode_config_.joint_names[index]] = target.positions_rad[index];
    }

    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningTime(mode_config_.planning_time_sec);
    move_group_->setNumPlanningAttempts(mode_config_.planning_attempts);
    move_group_->setMaxVelocityScalingFactor(mode_config_.max_velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(mode_config_.max_acceleration_scaling);
    if (!move_group_->setJointValueTarget(target_by_joint)) {
      reject(
        response,
        "rejected_move_group_target",
        "MoveGroup rejected the joint target map before planning");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    if (!planned) {
      response->accepted = true;
      response->planned = false;
      response->executed = false;
      response->status = "planning_failed";
      response->message = "MoveGroup planning failed for target '" + request->target_name + "'";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "MoveGroup planned target '%s'. reviewed_by='%s'",
      request->target_name.c_str(),
      target.reviewed_by.c_str());

    if (!request->execute) {
      response->accepted = true;
      response->planned = true;
      response->executed = false;
      response->status = "planned";
      response->message =
        "plan-only request passed catalog, joint-state, delta, and MoveGroup planning gates";
      return;
    }

    if (runtime_mode_ == "real") {
      std::string real_gate_message;
      if (!passes_real_execution_gate(request->human_confirmation, real_gate_message)) {
        response->accepted = true;
        response->planned = true;
        response->executed = false;
        response->status = "rejected_real_gate";
        response->message = real_gate_message;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }
    }

    const bool executed = static_cast<bool>(move_group_->execute(plan));
    if (!executed) {
      response->accepted = true;
      response->planned = true;
      response->executed = false;
      response->status = "execution_failed";
      response->message = "MoveGroup execute() failed for target '" + request->target_name + "'";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    std::string final_gate_message;
    if (!wait_for_final_target(target, final_gate_message)) {
      response->accepted = true;
      response->planned = true;
      response->executed = true;
      response->status = "final_gate_failed";
      response->message = final_gate_message;
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    response->accepted = true;
    response->planned = true;
    response->executed = true;
    response->status = "executed";
    response->message =
      "target '" + request->target_name + "' executed and final-target gate passed";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void reject(
    const std::shared_ptr<ExecuteNamedTarget::Response> & response,
    const std::string & status,
    const std::string & message)
  {
    response->accepted = false;
    response->planned = false;
    response->executed = false;
    response->status = status;
    response->message = message;
    RCLCPP_WARN(this->get_logger(), "%s: %s", status.c_str(), message.c_str());
  }

  bool ensure_move_group(const std::shared_ptr<ExecuteNamedTarget::Response> & response)
  {
    if (move_group_) {
      return true;
    }

    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), mode_config_.planning_group);
      move_group_->setPlanningTime(mode_config_.planning_time_sec);
      move_group_->setNumPlanningAttempts(mode_config_.planning_attempts);
      move_group_->setMaxVelocityScalingFactor(mode_config_.max_velocity_scaling);
      move_group_->setMaxAccelerationScalingFactor(mode_config_.max_acceleration_scaling);
      RCLCPP_INFO(
        this->get_logger(),
        "MoveGroupInterface ready for planning_group=%s",
        mode_config_.planning_group.c_str());
      return true;
    } catch (const std::exception & error) {
      reject(
        response,
        "rejected_move_group",
        std::string("failed to create MoveGroupInterface: ") + error.what());
      return false;
    }
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_joint_state_ = JointStateSnapshot{*msg, std::chrono::steady_clock::now(),
      ++state_sequence_};
    state_cv_.notify_all();
  }

  void on_speed_scaling(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_speed_scaling_ = msg->data;
    last_speed_scaling_at_ = std::chrono::steady_clock::now();
  }

  std::optional<JointStateSnapshot> wait_for_joint_state()
  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    if (last_joint_state_.has_value()) {
      return last_joint_state_;
    }

    const auto timeout = std::chrono::duration<double>(joint_state_timeout_sec_);
    const bool received = state_cv_.wait_for(
      lock,
      std::chrono::duration_cast<std::chrono::milliseconds>(timeout),
      [this]() {
        return last_joint_state_.has_value();
      });

    if (!received) {
      return std::nullopt;
    }
    return last_joint_state_;
  }

  std::optional<std::vector<double>> get_current_positions(std::string & message)
  {
    const auto snapshot = wait_for_joint_state();
    if (!snapshot.has_value()) {
      message = "timed out waiting for any JointState on " + joint_state_topic_;
      return std::nullopt;
    }

    if (!validate_joint_state_freshness(snapshot.value(), message)) {
      return std::nullopt;
    }

    return ordered_positions(snapshot->msg, message);
  }

  bool validate_joint_state_freshness(
    const JointStateSnapshot & snapshot,
    std::string & message) const
  {
    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - snapshot.received_at).count();
    if (age > max_joint_state_age_sec_) {
      message =
        "latest JointState was received " + format_double(age) +
        "s ago, exceeding max_joint_state_age_sec=" +
        format_double(max_joint_state_age_sec_);
      return false;
    }

    if (mode_config_.require_joint_state_stamp && stamp_is_zero(snapshot.msg.header.stamp)) {
      message = "real-mode catalog requires non-zero JointState header stamp";
      return false;
    }

    return true;
  }

  std::optional<std::vector<double>> ordered_positions(
    const sensor_msgs::msg::JointState & msg,
    std::string & message) const
  {
    std::unordered_map<std::string, double> positions_by_name;
    for (std::size_t index = 0; index < msg.name.size(); ++index) {
      if (index < msg.position.size()) {
        positions_by_name[msg.name[index]] = msg.position[index];
      }
    }

    std::vector<double> positions;
    positions.reserve(mode_config_.joint_names.size());
    std::vector<std::string> missing;
    for (const auto & joint_name : mode_config_.joint_names) {
      const auto position = positions_by_name.find(joint_name);
      if (position == positions_by_name.end()) {
        missing.push_back(joint_name);
      } else {
        positions.push_back(position->second);
      }
    }

    if (!missing.empty()) {
      message = "JointState is missing required joints: " + join(missing, ", ");
      return std::nullopt;
    }

    return positions;
  }

  bool passes_delta_gate(
    const std::vector<double> & current_positions,
    const NamedTarget & target,
    std::string & message) const
  {
    double max_observed_delta = 0.0;
    std::string max_delta_joint;
    for (std::size_t index = 0; index < current_positions.size(); ++index) {
      const double delta = std::abs(target.positions_rad[index] - current_positions[index]);
      if (delta > max_observed_delta) {
        max_observed_delta = delta;
        max_delta_joint = mode_config_.joint_names[index];
      }
      if (delta > mode_config_.max_joint_delta_rad) {
        message =
          "delta gate failed at " + mode_config_.joint_names[index] +
          ": delta=" + format_double(delta) +
          " rad exceeds max_joint_delta_rad=" +
          format_double(mode_config_.max_joint_delta_rad);
        return false;
      }
    }

    message =
      "delta gate passed; max_delta=" + format_double(max_observed_delta) +
      " rad at " + max_delta_joint;
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    return true;
  }

  bool wait_for_final_target(const NamedTarget & target, std::string & message)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(mode_config_.final_state_timeout_sec);
    std::size_t observed_sequence = 0;
    std::string last_error = "no final JointState sample checked";

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      JointStateSnapshot snapshot;
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        state_cv_.wait_for(
          lock,
          100ms,
          [this, observed_sequence]() {
            return last_joint_state_.has_value() &&
                   last_joint_state_->sequence > observed_sequence;
          });
        if (!last_joint_state_.has_value()) {
          continue;
        }
        snapshot = last_joint_state_.value();
        observed_sequence = snapshot.sequence;
      }

      std::string positions_message;
      const auto positions = ordered_positions(snapshot.msg, positions_message);
      if (!positions.has_value()) {
        last_error = positions_message;
        continue;
      }

      bool pass = true;
      double max_error = 0.0;
      std::string max_error_joint;
      for (std::size_t index = 0; index < positions->size(); ++index) {
        const double error = std::abs(positions->at(index) - target.positions_rad[index]);
        if (error > max_error) {
          max_error = error;
          max_error_joint = mode_config_.joint_names[index];
        }
        if (error > mode_config_.final_position_tolerance_rad) {
          pass = false;
        }
      }

      last_error =
        "max final error=" + format_double(max_error) +
        " rad at " + max_error_joint +
        ", tolerance=" + format_double(mode_config_.final_position_tolerance_rad);
      if (pass) {
        message = "final-target gate passed; " + last_error;
        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        return true;
      }
    }

    message =
      "final-target gate timed out after " +
      format_double(mode_config_.final_state_timeout_sec) + "s; " + last_error;
    return false;
  }

  bool passes_real_execution_gate(
    const std::string & human_confirmation,
    std::string & message)
  {
    if (human_confirmation != human_confirmation_token_) {
      message =
        "real execution rejected: human_confirmation must be " +
        human_confirmation_token_ + " after现场确认";
      return false;
    }

    if (!check_robot_mode(message) ||
      !check_safety_mode(message) ||
      !check_program_running(message) ||
      !check_remote_control(message) ||
      !check_required_controllers(message) ||
      !check_speed_scaling(message))
    {
      return false;
    }

    message =
      "real execution gate passed: dashboard, controller, joint-state, and speed-scaling checks are acceptable";
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    return true;
  }

  bool check_robot_mode(std::string & message)
  {
    const auto response = call_service<GetRobotMode>(robot_mode_client_, robot_mode_service_name_);
    if (!response || !response->success) {
      message = "robot mode service failed";
      return false;
    }
    if (response->robot_mode.mode != ur_dashboard_msgs::msg::RobotMode::RUNNING) {
      message = "robot_mode is not RUNNING: " + response->answer;
      return false;
    }
    return true;
  }

  bool check_safety_mode(std::string & message)
  {
    const auto response =
      call_service<GetSafetyMode>(safety_mode_client_, safety_mode_service_name_);
    if (!response || !response->success) {
      message = "safety mode service failed";
      return false;
    }

    const auto mode = response->safety_mode.mode;
    if (mode == ur_dashboard_msgs::msg::SafetyMode::NORMAL) {
      return true;
    }

    // TODO(human): 如果现场 Reduced 模式是有意配置的低速安全策略，
    // 请先在目标 catalog 中把 allow_reduced_safety_mode 改为 true，并记录理由。
    if (mode == ur_dashboard_msgs::msg::SafetyMode::REDUCED &&
      mode_config_.allow_reduced_safety_mode)
    {
      return true;
    }

    message = "safety_mode is not acceptable for execution: " + response->answer;
    return false;
  }

  bool check_program_running(std::string & message)
  {
    const auto response =
      call_service<IsProgramRunning>(program_running_client_, program_running_service_name_);
    if (!response || !response->success) {
      message = "program_running service failed";
      return false;
    }
    if (!response->program_running) {
      message = "External Control program is not running: " + response->answer;
      return false;
    }
    return true;
  }

  bool check_remote_control(std::string & message)
  {
    if (!mode_config_.require_remote_control) {
      return true;
    }

    const auto response =
      call_service<IsInRemoteControl>(remote_control_client_, remote_control_service_name_);
    if (!response || !response->success) {
      message = "remote_control service failed";
      return false;
    }
    if (!response->remote_control) {
      message = "real execution requires Remote Control mode: " + response->answer;
      return false;
    }
    return true;
  }

  bool check_required_controllers(std::string & message)
  {
    if (mode_config_.required_active_controllers.empty()) {
      return true;
    }

    const auto response =
      call_service<ListControllers>(list_controllers_client_, list_controllers_service_name_);
    if (!response) {
      message = "list_controllers service failed";
      return false;
    }

    std::unordered_map<std::string, std::string> state_by_name;
    for (const auto & controller : response->controller) {
      state_by_name[controller.name] = controller.state;
    }

    for (const auto & controller_name : mode_config_.required_active_controllers) {
      const auto state = state_by_name.find(controller_name);
      if (state == state_by_name.end()) {
        message = "required controller is missing: " + controller_name;
        return false;
      }
      if (state->second != "active") {
        message =
          "required controller is not active: " + controller_name +
          " state=" + state->second;
        return false;
      }
    }

    return true;
  }

  bool check_speed_scaling(std::string & message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!last_speed_scaling_.has_value() || !last_speed_scaling_at_.has_value()) {
      message = "no speed scaling sample received on " + speed_scaling_topic_;
      return false;
    }

    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_speed_scaling_at_.value()).count();
    if (age > max_joint_state_age_sec_) {
      message =
        "speed scaling sample is stale: age=" + format_double(age) +
        "s max=" + format_double(max_joint_state_age_sec_) + "s";
      return false;
    }

    if (last_speed_scaling_.value() < mode_config_.min_speed_scaling) {
      message =
        "speed scaling below minimum: current=" + format_double(last_speed_scaling_.value()) +
        " min=" + format_double(mode_config_.min_speed_scaling);
      return false;
    }

    return true;
  }

  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call_service(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const std::string & service_name)
  {
    if (!client->wait_for_service(std::chrono::duration<double>(service_timeout_sec_))) {
      RCLCPP_ERROR(this->get_logger(), "Service unavailable: %s", service_name.c_str());
      return nullptr;
    }

    auto request = std::make_shared<typename ServiceT::Request>();
    auto future = client->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(service_timeout_sec_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (future.wait_for(50ms) == std::future_status::ready) {
        return future.get();
      }
    }

    RCLCPP_ERROR(this->get_logger(), "Service timeout: %s", service_name.c_str());
    return nullptr;
  }

  std::string runtime_mode_;
  std::string target_catalog_path_;
  bool allow_execution_{false};
  std::string joint_state_topic_;
  double joint_state_timeout_sec_{3.0};
  double max_joint_state_age_sec_{1.0};
  std::string speed_scaling_topic_;
  std::string human_confirmation_token_;
  std::string robot_mode_service_name_;
  std::string safety_mode_service_name_;
  std::string program_running_service_name_;
  std::string remote_control_service_name_;
  std::string list_controllers_service_name_;
  double service_timeout_sec_{3.0};

  bool catalog_loaded_{false};
  std::string catalog_error_;
  ModeConfig mode_config_;

  std::mutex busy_mutex_;
  bool busy_{false};

  std::mutex state_mutex_;
  std::condition_variable state_cv_;
  std::optional<JointStateSnapshot> last_joint_state_;
  std::size_t state_sequence_{0};
  std::optional<double> last_speed_scaling_;
  std::optional<std::chrono::steady_clock::time_point> last_speed_scaling_at_;

  rclcpp::CallbackGroup::SharedPtr state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr real_gate_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_scaling_sub_;
  rclcpp::Service<ExecuteNamedTarget>::SharedPtr execute_service_;

  rclcpp::Client<GetRobotMode>::SharedPtr robot_mode_client_;
  rclcpp::Client<GetSafetyMode>::SharedPtr safety_mode_client_;
  rclcpp::Client<IsProgramRunning>::SharedPtr program_running_client_;
  rclcpp::Client<IsInRemoteControl>::SharedPtr remote_control_client_;
  rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ur3eNamedMotionController>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
