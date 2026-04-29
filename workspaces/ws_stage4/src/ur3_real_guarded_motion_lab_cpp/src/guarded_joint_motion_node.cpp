#include <cstdlib>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{
constexpr const char * kConfirmationToken = "I_CONFIRM_REAL_ROBOT_MOTION";
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory =
  rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

struct JointTarget
{
  std::vector<std::string> joint_names;
  std::vector<double> positions_rad;
  std::string reviewed_by;
};

std::string format_vector(const std::vector<double> & values)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << values[index];
  }
  stream << "]";
  return stream.str();
}

std::string format_vector(const std::vector<std::string> & values)
{
  std::ostringstream stream;
  stream << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << values[index];
  }
  stream << "]";
  return stream.str();
}

bool load_target(
  const rclcpp::Node::SharedPtr & node,
  const std::string & target_prefix,
  JointTarget & target)
{
  target.joint_names = node->declare_parameter<std::vector<std::string>>(
    target_prefix + "_joint_names", std::vector<std::string>{});
  target.positions_rad = node->declare_parameter<std::vector<double>>(
    target_prefix + "_positions_rad", std::vector<double>{});
  target.reviewed_by = node->declare_parameter<std::string>(
    target_prefix + "_reviewed_by", "");

  if (target.joint_names.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "%s_joint_names is empty.",
      target_prefix.c_str());
    return false;
  }

  if (target.positions_rad.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "%s_positions_rad is empty.",
      target_prefix.c_str());
    return false;
  }

  if (target.joint_names.size() != target.positions_rad.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "%s target size mismatch: joint_names=%zu positions=%zu.",
      target_prefix.c_str(),
      target.joint_names.size(),
      target.positions_rad.size());
    return false;
  }

  return true;
}

bool report_delta_gate(
  const rclcpp::Node::SharedPtr & node,
  const JointTarget & home,
  const JointTarget & target,
  const double max_joint_delta_rad)
{
  if (home.joint_names != target.joint_names) {
    RCLCPP_ERROR(
      node->get_logger(),
      "home and target joint_names differ; refusing to compare delta.");
    RCLCPP_ERROR(
      node->get_logger(),
      "home_joint_names=%s",
      format_vector(home.joint_names).c_str());
    RCLCPP_ERROR(
      node->get_logger(),
      "target_joint_names=%s",
      format_vector(target.joint_names).c_str());
    return false;
  }

  std::vector<double> deltas;
  deltas.reserve(home.positions_rad.size());
  bool pass = true;
  for (std::size_t index = 0; index < home.positions_rad.size(); ++index) {
    const double delta = target.positions_rad[index] - home.positions_rad[index];
    deltas.push_back(delta);
    const bool joint_pass = std::abs(delta) <= max_joint_delta_rad;
    pass = pass && joint_pass;
    RCLCPP_INFO(
      node->get_logger(),
      "delta[%s]=%.6f rad (%s)",
      home.joint_names[index].c_str(),
      delta,
      joint_pass ? "pass" : "block");
  }

  RCLCPP_INFO(node->get_logger(), "delta_vector_rad=%s", format_vector(deltas).c_str());
  if (!pass) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Delta gate failed: at least one joint exceeds max_joint_delta_rad=%.6f.",
      max_joint_delta_rad);
    return false;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Delta gate passed: every joint is within max_joint_delta_rad=%.6f.",
    max_joint_delta_rad);
  return true;
}

builtin_interfaces::msg::Duration make_duration(const double seconds)
{
  builtin_interfaces::msg::Duration duration;
  const auto whole_seconds = static_cast<int32_t>(std::floor(seconds));
  const auto fractional_seconds = seconds - static_cast<double>(whole_seconds);
  duration.sec = whole_seconds;
  duration.nanosec = static_cast<uint32_t>(std::round(fractional_seconds * 1e9));
  if (duration.nanosec >= 1000000000U) {
    duration.sec += 1;
    duration.nanosec -= 1000000000U;
  }
  return duration;
}

std::optional<std::vector<double>> wait_for_current_positions(
  const rclcpp::Node::SharedPtr & node,
  const std::vector<std::string> & joint_names,
  const std::string & joint_state_topic,
  const double timeout_sec)
{
  std::unordered_map<std::string, double> positions_by_name;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic,
    rclcpp::SensorDataQoS(),
    [&positions_by_name](const sensor_msgs::msg::JointState::SharedPtr msg) {
      positions_by_name.clear();
      for (std::size_t index = 0; index < msg->name.size(); ++index) {
        if (index < msg->position.size()) {
          positions_by_name[msg->name[index]] = msg->position[index];
        }
      }
    });

  RCLCPP_INFO(
    node->get_logger(),
    "Waiting for current joint state on %s for up to %.1fs...",
    joint_state_topic.c_str(),
    timeout_sec);

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);

    std::vector<double> ordered_positions;
    ordered_positions.reserve(joint_names.size());
    bool complete = true;
    for (const auto & joint_name : joint_names) {
      const auto position = positions_by_name.find(joint_name);
      if (position == positions_by_name.end()) {
        complete = false;
        break;
      }
      ordered_positions.push_back(position->second);
    }

    if (complete) {
      RCLCPP_INFO(
        node->get_logger(),
        "current_positions_rad=%s",
        format_vector(ordered_positions).c_str());
      return ordered_positions;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  RCLCPP_ERROR(node->get_logger(), "Timed out waiting for complete current joint state.");
  return std::nullopt;
}

bool report_current_home_gate(
  const rclcpp::Node::SharedPtr & node,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & current_positions,
  const std::vector<double> & home_positions,
  const double max_joint_delta_rad)
{
  if (current_positions.size() != home_positions.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "current/home size mismatch: current=%zu home=%zu.",
      current_positions.size(),
      home_positions.size());
    return false;
  }

  std::vector<double> deltas;
  deltas.reserve(current_positions.size());
  bool pass = true;
  for (std::size_t index = 0; index < current_positions.size(); ++index) {
    const double delta = current_positions[index] - home_positions[index];
    deltas.push_back(delta);
    const bool joint_pass = std::abs(delta) <= max_joint_delta_rad;
    pass = pass && joint_pass;
    RCLCPP_INFO(
      node->get_logger(),
      "current_minus_home[%s]=%.6f rad (%s)",
      joint_names[index].c_str(),
      delta,
      joint_pass ? "pass" : "block");
  }

  RCLCPP_INFO(node->get_logger(), "current_minus_home_rad=%s", format_vector(deltas).c_str());
  if (!pass) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Current-home gate failed: robot moved too far from reviewed home.");
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Current-home gate passed.");
  return true;
}

FollowJointTrajectory::Goal build_single_point_goal(
  const JointTarget & target,
  const double duration_sec,
  const double goal_time_tolerance_sec)
{
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = target.positions_rad;
  point.velocities = std::vector<double>(target.positions_rad.size(), 0.0);
  point.time_from_start = make_duration(duration_sec);

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = target.joint_names;
  goal.trajectory.points.push_back(point);
  goal.goal_time_tolerance = make_duration(goal_time_tolerance_sec);
  return goal;
}

bool send_single_point_goal(
  const rclcpp::Node::SharedPtr & node,
  const std::string & action_name,
  const JointTarget & target,
  const double duration_sec,
  const double goal_time_tolerance_sec,
  const double action_server_timeout_sec)
{
  auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(node, action_name);
  RCLCPP_INFO(
    node->get_logger(),
    "Waiting for FollowJointTrajectory action server %s...",
    action_name.c_str());
  if (!action_client->wait_for_action_server(std::chrono::duration<double>(action_server_timeout_sec))) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Action server unavailable: %s.",
      action_name.c_str());
    return false;
  }

  const auto goal = build_single_point_goal(target, duration_sec, goal_time_tolerance_sec);
  RCLCPP_WARN(
    node->get_logger(),
    "Sending exactly one FollowJointTrajectory point to %s over %.2fs.",
    action_name.c_str(),
    duration_sec);

  auto goal_handle_future = action_client->async_send_goal(goal);
  const auto goal_response_code = rclcpp::spin_until_future_complete(
    node, goal_handle_future, std::chrono::duration<double>(action_server_timeout_sec));
  if (goal_response_code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for goal response.");
    return false;
  }

  const auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal rejected by controller.");
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "Goal accepted by controller.");

  auto result_future = action_client->async_get_result(goal_handle);
  const auto result_timeout = duration_sec + goal_time_tolerance_sec + 10.0;
  const auto result_code = rclcpp::spin_until_future_complete(
    node, result_future, std::chrono::duration<double>(result_timeout));
  if (result_code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for action result.");
    return false;
  }

  const GoalHandleFollowJointTrajectory::WrappedResult wrapped_result = result_future.get();
  if (!wrapped_result.result) {
    RCLCPP_ERROR(node->get_logger(), "Action result is empty.");
    return false;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Action result: status=%d error_code=%d error_string='%s'",
    static_cast<int>(wrapped_result.code),
    wrapped_result.result->error_code,
    wrapped_result.result->error_string.c_str());

  return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped_result.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL;
}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task8d_guarded_joint_motion");

  const bool execute = node->declare_parameter<bool>("execute", false);
  const bool require_confirmation = node->declare_parameter<bool>("require_confirmation", true);
  const std::string human_confirmation =
    node->declare_parameter<std::string>("human_confirmation", "");
  const std::string target_name = node->declare_parameter<std::string>("target_name", "ready");
  const double max_joint_delta_rad = node->declare_parameter<double>("max_joint_delta_rad", 0.10);
  const double min_trajectory_duration_sec =
    node->declare_parameter<double>("min_trajectory_duration_sec", 5.0);
  const double goal_time_tolerance_sec =
    node->declare_parameter<double>("goal_time_tolerance_sec", 2.0);
  const double action_server_timeout_sec =
    node->declare_parameter<double>("action_server_timeout_sec", 10.0);
  const double joint_state_timeout_sec =
    node->declare_parameter<double>("joint_state_timeout_sec", 3.0);
  const std::string action_name = node->declare_parameter<std::string>(
    "action_name", "/scaled_joint_trajectory_controller/follow_joint_trajectory");
  const std::string joint_state_topic =
    node->declare_parameter<std::string>("joint_state_topic", "/joint_states");

  RCLCPP_INFO(node->get_logger(), "Task 8D guarded motion scaffold started.");
  RCLCPP_INFO(node->get_logger(), "target_name=%s", target_name.c_str());
  RCLCPP_INFO(node->get_logger(), "execute=%s", execute ? "true" : "false");
  RCLCPP_INFO(node->get_logger(), "max_joint_delta_rad=%.3f", max_joint_delta_rad);
  RCLCPP_INFO(
    node->get_logger(),
    "min_trajectory_duration_sec=%.3f",
    min_trajectory_duration_sec);
  RCLCPP_INFO(node->get_logger(), "action_name=%s", action_name.c_str());

  JointTarget home;
  JointTarget ready;
  if (!load_target(node, "home", home) || !load_target(node, "ready", ready)) {
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const JointTarget * selected_target = nullptr;
  if (target_name == "home") {
    selected_target = &home;
  } else if (target_name == "ready") {
    selected_target = &ready;
  } else {
    RCLCPP_ERROR(
      node->get_logger(),
      "Unsupported target_name=%s. Expected home or ready.",
      target_name.c_str());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(node->get_logger(), "home_joint_names=%s", format_vector(home.joint_names).c_str());
  RCLCPP_INFO(node->get_logger(), "home_positions_rad=%s", format_vector(home.positions_rad).c_str());
  RCLCPP_INFO(node->get_logger(), "home_reviewed_by=%s", home.reviewed_by.c_str());
  RCLCPP_INFO(
    node->get_logger(),
    "%s_positions_rad=%s",
    target_name.c_str(),
    format_vector(selected_target->positions_rad).c_str());
  RCLCPP_INFO(
    node->get_logger(),
    "%s_reviewed_by=%s",
    target_name.c_str(),
    selected_target->reviewed_by.c_str());

  if (!report_delta_gate(node, home, *selected_target, max_joint_delta_rad)) {
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  if (!execute) {
    RCLCPP_WARN(
      node->get_logger(),
      "Dry-run only. No FollowJointTrajectory goal will be sent.");
    RCLCPP_WARN(
      node->get_logger(),
      "Dry-run passed target loading and delta gate. Execution remains disabled.");
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  }

  if (require_confirmation && human_confirmation != kConfirmationToken) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Execution rejected: human_confirmation must be %s after现场确认.",
      kConfirmationToken);
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const auto current_positions = wait_for_current_positions(
    node, home.joint_names, joint_state_topic, joint_state_timeout_sec);
  if (!current_positions.has_value()) {
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  if (!report_current_home_gate(
      node, home.joint_names, current_positions.value(), home.positions_rad, max_joint_delta_rad))
  {
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  if (!send_single_point_goal(
      node,
      action_name,
      *selected_target,
      min_trajectory_duration_sec,
      goal_time_tolerance_sec,
      action_server_timeout_sec))
  {
    RCLCPP_ERROR(node->get_logger(), "Task 8D guarded motion execution failed.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(node->get_logger(), "Task 8D guarded motion execution finished successfully.");

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
