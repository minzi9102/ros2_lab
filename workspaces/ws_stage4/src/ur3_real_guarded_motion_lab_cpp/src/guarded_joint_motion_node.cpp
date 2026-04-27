#include <cstdlib>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr const char * kConfirmationToken = "I_CONFIRM_REAL_ROBOT_MOTION";
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

  if (!execute) {
    RCLCPP_WARN(
      node->get_logger(),
      "Dry-run only. No FollowJointTrajectory goal will be sent.");
    RCLCPP_WARN(
      node->get_logger(),
      "TODO(human): 填写并审核 home/ready 关节目标后，再进入真实动作实现。");
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
