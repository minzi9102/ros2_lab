#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ur_dashboard_msgs/srv/get_robot_mode.hpp>
#include <ur_dashboard_msgs/srv/get_safety_mode.hpp>
#include <ur_dashboard_msgs/srv/is_in_remote_control.hpp>
#include <ur_dashboard_msgs/srv/is_program_running.hpp>
#include <ur_msgs/srv/set_speed_slider_fraction.hpp>

#include "ur3e_handeye_motion/action/execute_plan.hpp"
#include "ur3e_handeye_motion/motion_safety.hpp"
#include "ur3e_handeye_motion/srv/plan_pose.hpp"

using namespace std::chrono_literals;

namespace ur3e_handeye_motion
{

class HandeyeMotionNode : public rclcpp::Node
{
public:
  using PlanPose = srv::PlanPose;
  using ExecutePlan = action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ServerGoalHandle<ExecutePlan>;
  using MoveGroup = moveit::planning_interface::MoveGroupInterface;

  HandeyeMotionNode()
  : Node("motion_server")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "ur_manipulator");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    end_effector_link_ = declare_parameter<std::string>("end_effector_link", "tool0");
    planning_time_sec_ = declare_parameter<double>("planning_time_sec", 3.0);
    planning_attempts_ = declare_parameter<int>("planning_attempts", 3);
    current_state_timeout_sec_ = declare_parameter<double>("current_state_timeout_sec", 2.0);
    ik_timeout_sec_ = declare_parameter<double>("ik_timeout_sec", 0.1);
    start_tolerance_rad_ = declare_parameter<double>("start_tolerance_rad", 0.01);
    position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.001);
    orientation_tolerance_rad_ = declare_parameter<double>(
      "orientation_tolerance_rad", M_PI / 180.0);
    speed_slider_fraction_ = declare_parameter<double>("speed_slider_fraction", 0.1);
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 1.0);
    joint_state_stale_sec_ = declare_parameter<double>("joint_state_stale_sec", 0.25);
    speed_scaling_stale_sec_ = declare_parameter<double>("speed_scaling_stale_sec", 1.0);

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&HandeyeMotionNode::on_joint_state, this, std::placeholders::_1),
      subscription_options);
    speed_scaling_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/speed_scaling_state_broadcaster/speed_scaling", 10,
      std::bind(&HandeyeMotionNode::on_speed_scaling, this, std::placeholders::_1),
      subscription_options);

    robot_mode_client_ = make_client<ur_dashboard_msgs::srv::GetRobotMode>(
      "/dashboard_client/get_robot_mode");
    safety_mode_client_ = make_client<ur_dashboard_msgs::srv::GetSafetyMode>(
      "/dashboard_client/get_safety_mode");
    program_running_client_ = make_client<ur_dashboard_msgs::srv::IsProgramRunning>(
      "/dashboard_client/program_running");
    remote_control_client_ = make_client<ur_dashboard_msgs::srv::IsInRemoteControl>(
      "/dashboard_client/is_in_remote_control");
    controller_client_ = make_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
    speed_slider_client_ = make_client<ur_msgs::srv::SetSpeedSliderFraction>(
      "/io_and_status_controller/set_speed_slider");

    const auto display_qos = rclcpp::QoS(1).transient_local().reliable();
    display_publisher_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", display_qos);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void initialize()
  {
    move_group_ = std::make_shared<MoveGroup>(shared_from_this(), planning_group_);
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setPlanningTime(planning_time_sec_);
    move_group_->setNumPlanningAttempts(planning_attempts_);

    plan_service_ = create_service<PlanPose>(
      "plan_pose",
      std::bind(
        &HandeyeMotionNode::plan_pose, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
    execute_action_ = rclcpp_action::create_server<ExecutePlan>(
      shared_from_this(), "execute_plan",
      std::bind(
        &HandeyeMotionNode::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&HandeyeMotionNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&HandeyeMotionNode::handle_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), callback_group_);

    readiness_timer_ = create_wall_timer(
      1s, std::bind(&HandeyeMotionNode::check_readiness, this), callback_group_);
    monitor_timer_ = create_wall_timer(
      200ms, std::bind(&HandeyeMotionNode::monitor_execution, this), callback_group_);
    RCLCPP_INFO(
      get_logger(),
      "Hand-eye motion API ready: frame=%s link=%s slider=%.0f%%",
      base_frame_.c_str(), end_effector_link_.c_str(), speed_slider_fraction_ * 100.0);
  }

private:
  struct GateReport
  {
    bool blocked{false};
    bool warned{false};
    std::string summary;
  };

  struct CachedPlan
  {
    uint64_t id{0};
    MoveGroup::Plan plan;
    geometry_msgs::msg::PoseStamped target;
  };

  template<typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr make_client(const std::string & name)
  {
    return create_client<ServiceT>(name, rclcpp::ServicesQoS(), callback_group_);
  }

  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request)
  {
    const auto timeout = std::chrono::duration<double>(service_timeout_sec_);
    if (!client->wait_for_service(timeout)) {
      return nullptr;
    }
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_joint_state_ = *msg;
    last_joint_state_time_ = now;
    joint_sample_times_.push_back(now);
    while (!joint_sample_times_.empty() && now - joint_sample_times_.front() > 3s) {
      joint_sample_times_.pop_front();
    }
  }

  void on_speed_scaling(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    speed_scaling_ = msg->data;
    last_speed_scaling_time_ = std::chrono::steady_clock::now();
  }

  bool set_speed_slider(std::string & reason)
  {
    auto request = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
    request->speed_slider_fraction = speed_slider_fraction_;
    const auto response = call<ur_msgs::srv::SetSpeedSliderFraction>(speed_slider_client_, request);
    if (!response || !response->success) {
      reason = "failed to set teach-pendant speed slider to 10%";
      return false;
    }
    return true;
  }

  GateReport check_gate(const bool nonblocking = false)
  {
    std::unique_lock<std::mutex> lock(gate_mutex_, std::defer_lock);
    if (nonblocking ? !lock.try_lock() : (lock.lock(), false)) {
      return {};
    }

    GateReport report;
    std::vector<std::string> lines;
    const auto add = [&report, &lines](const GateLevel level, const std::string & text) {
        report.blocked = report.blocked || level == GateLevel::block;
        report.warned = report.warned || level == GateLevel::warn;
        const char * label = level == GateLevel::block ? "BLOCK" :
          (level == GateLevel::warn ? "WARN" : "PASS");
        lines.emplace_back(std::string(label) + ":" + text);
      };

    const auto robot = call<ur_dashboard_msgs::srv::GetRobotMode>(
      robot_mode_client_, std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Request>());
    add(
      robot && robot->success ? robot_mode_level(robot->robot_mode.mode) : GateLevel::block,
      "robot_mode");

    const auto safety = call<ur_dashboard_msgs::srv::GetSafetyMode>(
      safety_mode_client_, std::make_shared<ur_dashboard_msgs::srv::GetSafetyMode::Request>());
    add(
      safety && safety->success ? safety_mode_level(safety->safety_mode.mode) : GateLevel::block,
      "safety_mode");

    const auto program = call<ur_dashboard_msgs::srv::IsProgramRunning>(
      program_running_client_,
      std::make_shared<ur_dashboard_msgs::srv::IsProgramRunning::Request>());
    add(
      program && program->success && program->program_running ? GateLevel::pass : GateLevel::block,
      "external_control");

    const auto remote = call<ur_dashboard_msgs::srv::IsInRemoteControl>(
      remote_control_client_,
      std::make_shared<ur_dashboard_msgs::srv::IsInRemoteControl::Request>());
    add(
      remote && remote->success && remote->remote_control ? GateLevel::pass : GateLevel::warn,
      "remote_control");

    const auto controllers = call<controller_manager_msgs::srv::ListControllers>(
      controller_client_,
      std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
    const auto controller_active = [&controllers](const std::string & name) {
        if (!controllers) {
          return false;
        }
        return std::any_of(
          controllers->controller.begin(), controllers->controller.end(),
          [&name](const auto & controller) {
            return controller.name == name && controller.state == "active";
          });
      };
    add(
      controller_active("joint_state_broadcaster") ? GateLevel::pass : GateLevel::block,
      "joint_state_broadcaster");
    add(
      controller_active("scaled_joint_trajectory_controller") ?
      GateLevel::pass : GateLevel::block,
      "scaled_joint_trajectory_controller");

    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      const auto now = std::chrono::steady_clock::now();
      const bool joint_fresh = latest_joint_state_.has_value() &&
        std::chrono::duration<double>(now - last_joint_state_time_).count() <=
        joint_state_stale_sec_;
      add(joint_fresh ? GateLevel::pass : GateLevel::block, "joint_states");
      if (joint_sample_times_.size() >= 2) {
        const double duration = std::chrono::duration<double>(
          joint_sample_times_.back() - joint_sample_times_.front()).count();
        const double rate = duration > 0.0 ? (joint_sample_times_.size() - 1) / duration : 0.0;
        add(rate >= 100.0 ? GateLevel::pass : GateLevel::warn, "joint_state_rate");
      } else {
        add(GateLevel::warn, "joint_state_rate");
      }

      const bool scaling_fresh = speed_scaling_.has_value() &&
        std::chrono::duration<double>(now - last_speed_scaling_time_).count() <=
        speed_scaling_stale_sec_;
      const bool scaling_positive = scaling_fresh && speed_scaling_.value() > 0.0;
      add(scaling_positive ? GateLevel::pass : GateLevel::block, "speed_scaling");
    }

    std::ostringstream stream;
    for (std::size_t index = 0; index < lines.size(); ++index) {
      if (index != 0) {
        stream << ", ";
      }
      stream << lines[index];
    }
    report.summary = stream.str();
    return report;
  }

  void check_readiness()
  {
    if (ready_.load()) {
      readiness_timer_->cancel();
      return;
    }
    std::string reason;
    if (!set_speed_slider(reason)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s", reason.c_str());
      return;
    }
    const auto gate = check_gate(true);
    if (gate.blocked || gate.summary.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for 8C gate: %s", gate.summary.c_str());
      return;
    }
    ready_.store(true);
    RCLCPP_INFO(get_logger(), "Startup 8C gate ready: %s", gate.summary.c_str());
  }

  void plan_pose(
    const PlanPose::Request::SharedPtr request,
    PlanPose::Response::SharedPtr response)
  {
    response->planned = false;
    if (request->target.header.frame_id != base_frame_) {
      response->message = "target.header.frame_id must be " + base_frame_;
      return;
    }
    if (!valid_pose(request->target.pose, response->message)) {
      return;
    }
    if (!ready_.load()) {
      response->message = "startup speed-slider and 8C gate are not ready";
      return;
    }

    std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock() || execution_active_.load()) {
      response->message = "another planning or execution operation is active";
      return;
    }
    {
      std::lock_guard<std::mutex> cache_lock(cache_mutex_);
      cached_plan_.reset();
      plan_slot_.clear();
    }
    const auto gate = check_gate();
    if (gate.blocked) {
      response->message = "8C gate blocked planning: " + gate.summary;
      return;
    }

    const auto current_state = move_group_->getCurrentState(current_state_timeout_sec_);
    if (!current_state) {
      response->message = "failed to read current robot state";
      return;
    }
    const auto * joint_group = current_state->getJointModelGroup(planning_group_);
    if (!joint_group) {
      response->message = "planning group does not exist: " + planning_group_;
      return;
    }

    moveit::core::RobotState target_state(*current_state);
    if (!target_state.setFromIK(
        joint_group, request->target.pose, end_effector_link_, ik_timeout_sec_))
    {
      response->message = "current-state-seeded IK failed";
      return;
    }
    move_group_->setStartState(*current_state);
    if (!move_group_->setJointValueTarget(target_state)) {
      response->message = "failed to lock the seeded IK joint target";
      return;
    }

    MoveGroup::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      response->message = "MoveIt planning failed";
      return;
    }
    if (plan.trajectory.joint_trajectory.points.empty()) {
      response->message = "MoveIt returned an empty trajectory";
      return;
    }

    response->trajectory = plan.trajectory;
    response->start_state = plan.start_state.joint_state;
    const auto & trajectory = plan.trajectory.joint_trajectory;
    const auto & endpoint = trajectory.points.back();
    if (trajectory.joint_names.size() != endpoint.positions.size()) {
      response->message = "MoveIt trajectory endpoint has inconsistent joint dimensions";
      return;
    }
    response->goal_state.header = trajectory.header;
    response->goal_state.name = trajectory.joint_names;
    response->goal_state.position = endpoint.positions;
    response->nominal_duration = endpoint.time_from_start;

    std::map<std::string, double> start_positions;
    for (std::size_t index = 0;
      index < response->start_state.name.size() && index < response->start_state.position.size();
      ++index)
    {
      start_positions[response->start_state.name[index]] = response->start_state.position[index];
    }
    response->joint_delta_rad.reserve(response->goal_state.name.size());
    for (std::size_t index = 0; index < response->goal_state.name.size(); ++index) {
      const auto found = start_positions.find(response->goal_state.name[index]);
      response->joint_delta_rad.push_back(
        found == start_positions.end() ? std::numeric_limits<double>::quiet_NaN() :
        endpoint.positions[index] - found->second);
    }

    std::lock_guard<std::mutex> cache_lock(cache_mutex_);
    const uint64_t id = plan_slot_.replace();
    cached_plan_ = CachedPlan{id, std::move(plan), request->target};
    response->planned = true;
    response->plan_id = id;
    response->message = gate.warned ?
      "planning succeeded with 8C warnings: " + gate.summary : "planning succeeded";
    publish_preview(cached_plan_->plan);
  }

  void publish_preview(const MoveGroup::Plan & plan)
  {
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = move_group_->getRobotModel()->getName();
    display.trajectory_start = plan.start_state;
    display.trajectory.push_back(plan.trajectory);
    display_publisher_->publish(display);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const ExecutePlan::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    if (execution_active_.load() || !cached_plan_ || goal->plan_id != plan_slot_.active_id()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    execution_active_.store(true);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecutePlan>)
  {
    if (!execution_active_.load()) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    cancel_requested_.store(true);
    if (move_group_) {
      move_group_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
  {
    std::optional<CachedPlan> selected;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      const uint64_t id = goal_handle->get_goal()->plan_id;
      if (cached_plan_ && plan_slot_.consume(id)) {
        selected = std::move(cached_plan_);
        cached_plan_.reset();
      }
    }
    if (!selected) {
      execution_active_.store(false);
      auto result = std::make_shared<ExecutePlan::Result>();
      result->message = "cached plan was no longer available";
      goal_handle->abort(result);
      return;
    }

    auto self = std::static_pointer_cast<HandeyeMotionNode>(shared_from_this());
    std::thread(
      [self, goal_handle, selected = std::move(*selected)]() mutable {
        self->execute_cached(goal_handle, std::move(selected));
      }).detach();
  }

  void execute_cached(
    const std::shared_ptr<GoalHandleExecutePlan> goal_handle,
    CachedPlan selected)
  {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    cancel_requested_.store(false);
    abort_requested_.store(false);
    auto result = std::make_shared<ExecutePlan::Result>();
    const auto finish = [this]() {
        monitoring_.store(false);
        execution_active_.store(false);
      };
    const auto abort = [&]() {
        finish();
        if (cancel_requested_.load() || goal_handle->is_canceling()) {
          goal_handle->canceled(result);
        } else {
          goal_handle->abort(result);
        }
      };

    publish_stage(goal_handle, "PRECHECK");
    std::string slider_reason;
    if (!set_speed_slider(slider_reason)) {
      result->message = slider_reason;
      abort();
      return;
    }
    const auto gate = check_gate();
    if (gate.blocked) {
      result->message = "8C gate blocked execution: " + gate.summary;
      abort();
      return;
    }

    const auto current_state = move_group_->getCurrentState(current_state_timeout_sec_);
    const auto * joint_group = current_state ?
      current_state->getJointModelGroup(planning_group_) : nullptr;
    if (!current_state || !joint_group) {
      result->message = "failed to read execution start state";
      abort();
      return;
    }
    std::vector<double> actual_joints;
    current_state->copyJointGroupPositions(joint_group, actual_joints);
    const auto expected = plan_start_positions(selected.plan, joint_group->getVariableNames());
    if (!joints_match(expected, actual_joints, start_tolerance_rad_)) {
      result->message = "robot state drifted from cached trajectory start; replan required";
      abort();
      return;
    }

    publish_stage(goal_handle, "EXECUTING");
    monitoring_.store(true);
    const bool execute_ok =
      move_group_->execute(selected.plan) == moveit::core::MoveItErrorCode::SUCCESS;
    monitoring_.store(false);
    if (cancel_requested_.load() || goal_handle->is_canceling()) {
      result->message = "execution canceled; replan required";
      abort();
      return;
    }
    if (abort_requested_.load()) {
      result->message = "8C gate blocked during execution; trajectory stopped";
      abort();
      return;
    }
    if (!execute_ok) {
      result->message = "cached trajectory execution failed; replan required";
      abort();
      return;
    }
    result->executed = true;

    publish_stage(goal_handle, "VERIFYING");
    try {
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_, end_effector_link_, tf2::TimePointZero, tf2::durationFromSec(2.0));
      result->actual_pose.header = transform.header;
      result->actual_pose.pose.position.x = transform.transform.translation.x;
      result->actual_pose.pose.position.y = transform.transform.translation.y;
      result->actual_pose.pose.position.z = transform.transform.translation.z;
      result->actual_pose.pose.orientation = transform.transform.rotation;
    } catch (const tf2::TransformException & error) {
      result->message = std::string("execution completed but TF verification failed: ") +
        error.what();
      abort();
      return;
    }

    const auto & actual = result->actual_pose.pose;
    const auto & target = selected.target.pose;
    result->position_error_m = std::hypot(
      std::hypot(actual.position.x - target.position.x, actual.position.y - target.position.y),
      actual.position.z - target.position.z);
    result->orientation_error_rad = orientation_error_rad(
      actual.orientation, target.orientation);
    result->within_tolerance = result->position_error_m <= position_tolerance_m_ &&
      result->orientation_error_rad <= orientation_tolerance_rad_;
    if (!result->within_tolerance) {
      result->message = "execution completed outside TF acceptance tolerance; no compensation sent";
      abort();
      return;
    }

    result->message = "cached trajectory executed and TF acceptance passed";
    finish();
    goal_handle->succeed(result);
  }

  std::vector<double> plan_start_positions(
    const MoveGroup::Plan & plan,
    const std::vector<std::string> & names) const
  {
    std::map<std::string, double> positions;
    const auto & state = plan.start_state.joint_state;
    for (std::size_t index = 0; index < state.name.size() && index < state.position.size();
      ++index)
    {
      positions[state.name[index]] = state.position[index];
    }
    std::vector<double> result;
    result.reserve(names.size());
    for (const auto & name : names) {
      const auto found = positions.find(name);
      if (found == positions.end()) {
        return {};
      }
      result.push_back(found->second);
    }
    return result;
  }

  void publish_stage(
    const std::shared_ptr<GoalHandleExecutePlan> & goal_handle,
    const std::string & stage)
  {
    auto feedback = std::make_shared<ExecutePlan::Feedback>();
    feedback->stage = stage;
    goal_handle->publish_feedback(feedback);
  }

  void monitor_execution()
  {
    if (!monitoring_.load()) {
      return;
    }
    if (cancel_requested_.load()) {
      move_group_->stop();
      return;
    }
    if (abort_requested_.load()) {
      return;
    }
    const auto gate = check_gate(true);
    if (gate.blocked && !gate.summary.empty()) {
      abort_requested_.store(true);
      RCLCPP_ERROR(get_logger(), "8C monitor stopped execution: %s", gate.summary.c_str());
      move_group_->stop();
    }
  }

  std::string planning_group_;
  std::string base_frame_;
  std::string end_effector_link_;
  double planning_time_sec_{3.0};
  int planning_attempts_{3};
  double current_state_timeout_sec_{2.0};
  double ik_timeout_sec_{0.1};
  double start_tolerance_rad_{0.01};
  double position_tolerance_m_{0.001};
  double orientation_tolerance_rad_{M_PI / 180.0};
  double speed_slider_fraction_{0.1};
  double service_timeout_sec_{1.0};
  double joint_state_stale_sec_{0.25};
  double speed_scaling_stale_sec_{1.0};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<MoveGroup> move_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher_;
  rclcpp::Service<PlanPose>::SharedPtr plan_service_;
  rclcpp_action::Server<ExecutePlan>::SharedPtr execute_action_;
  rclcpp::TimerBase::SharedPtr readiness_timer_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_scaling_subscription_;
  rclcpp::Client<ur_dashboard_msgs::srv::GetRobotMode>::SharedPtr robot_mode_client_;
  rclcpp::Client<ur_dashboard_msgs::srv::GetSafetyMode>::SharedPtr safety_mode_client_;
  rclcpp::Client<ur_dashboard_msgs::srv::IsProgramRunning>::SharedPtr program_running_client_;
  rclcpp::Client<ur_dashboard_msgs::srv::IsInRemoteControl>::SharedPtr remote_control_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controller_client_;
  rclcpp::Client<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr speed_slider_client_;

  std::mutex operation_mutex_;
  std::mutex cache_mutex_;
  std::mutex gate_mutex_;
  std::mutex state_mutex_;
  PlanSlot plan_slot_;
  std::optional<CachedPlan> cached_plan_;
  std::optional<sensor_msgs::msg::JointState> latest_joint_state_;
  std::optional<double> speed_scaling_;
  std::deque<std::chrono::steady_clock::time_point> joint_sample_times_;
  std::chrono::steady_clock::time_point last_joint_state_time_{};
  std::chrono::steady_clock::time_point last_speed_scaling_time_{};
  std::atomic_bool ready_{false};
  std::atomic_bool execution_active_{false};
  std::atomic_bool monitoring_{false};
  std::atomic_bool cancel_requested_{false};
  std::atomic_bool abort_requested_{false};
};

}  // namespace ur3e_handeye_motion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur3e_handeye_motion::HandeyeMotionNode>();
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
