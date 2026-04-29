#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr const char * kConfirmationToken = "I_CONFIRM_REAL_ROBOT_MOTION";

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

  RCLCPP_INFO(node->get_logger(), "Task 8D guarded motion scaffold started.");
  RCLCPP_INFO(node->get_logger(), "target_name=%s", target_name.c_str());
  RCLCPP_INFO(node->get_logger(), "execute=%s", execute ? "true" : "false");
  RCLCPP_INFO(node->get_logger(), "max_joint_delta_rad=%.3f", max_joint_delta_rad);

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

  RCLCPP_ERROR(
    node->get_logger(),
    "Execution intentionally not implemented yet. Complete Task 8C state gates and Task 8D target review first.");
  RCLCPP_ERROR(
    node->get_logger(),
    "TODO(human): 你需要先确认真机点位、速度、现场安全和执行许可。");

  rclcpp::shutdown();
  return EXIT_FAILURE;
}
