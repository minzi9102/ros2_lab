#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class Ur3GoalPoseExecutorNode : public rclcpp::Node
{
public:
  Ur3GoalPoseExecutorNode()
  : Node("ur3_goal_pose_executor")
  {
    planning_group_ = this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    pose_reference_frame_ =
      this->declare_parameter<std::string>("pose_reference_frame", "base_link");
    end_effector_link_ = this->declare_parameter<std::string>("end_effector_link", "tool0");
    goal_topic_ = this->declare_parameter<std::string>("goal_topic", "/goal_pose");
    execute_plan_ = this->declare_parameter<bool>("execute_plan", true);
    target_height_ = this->declare_parameter<double>("target_height", 0.22);
    workspace_min_x_ = this->declare_parameter<double>("workspace_min_x", 0.10);
    workspace_max_x_ = this->declare_parameter<double>("workspace_max_x", 0.45);
    workspace_min_y_ = this->declare_parameter<double>("workspace_min_y", -0.25);
    workspace_max_y_ = this->declare_parameter<double>("workspace_max_y", 0.25);
    downward_roll_rad_ = this->declare_parameter<double>("downward_roll_rad", M_PI);
    downward_pitch_rad_ = this->declare_parameter<double>("downward_pitch_rad", 0.0);

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_,
      10,
      std::bind(&Ur3GoalPoseExecutorNode::on_goal_pose, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "goal_pose_executor_node started. topic=%s execute_plan=%s workspace_x=[%.2f, %.2f] workspace_y=[%.2f, %.2f]",
      goal_topic_.c_str(),
      execute_plan_ ? "true" : "false",
      workspace_min_x_,
      workspace_max_x_,
      workspace_min_y_,
      workspace_max_y_);
  }

private:
  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (busy_) {
      RCLCPP_WARN(this->get_logger(), "Planner is busy. Ignoring the new /goal_pose.");
      return;
    }

    busy_ = true;
    ensure_move_group();

    if (!is_frame_supported(msg->header.frame_id)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unsupported goal frame '%s'. Expected '%s' or an empty frame.",
        msg->header.frame_id.c_str(),
        pose_reference_frame_.c_str());
      busy_ = false;
      return;
    }

    if (!is_in_workspace(msg->pose.position.x, msg->pose.position.y)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Rejected goal outside workspace: x=%.3f y=%.3f",
        msg->pose.position.x,
        msg->pose.position.y);
      busy_ = false;
      return;
    }

    const double yaw = extract_yaw(msg->pose.orientation);
    const geometry_msgs::msg::Pose target_pose = map_goal_to_target_pose(*msg, yaw);

    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(target_pose, end_effector_link_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));

    RCLCPP_INFO(
      this->get_logger(),
      "Mapped goal_pose -> target pose. input=(%.3f, %.3f, yaw=%.3f) mapped=(%.3f, %.3f, %.3f) planned=%s",
      msg->pose.position.x,
      msg->pose.position.y,
      yaw,
      target_pose.position.x,
      target_pose.position.y,
      target_pose.position.z,
      planned ? "true" : "false");

    if (planned && execute_plan_) {
      move_group_->execute(plan);
    }

    // TODO(human): 请亲自评估当前工作区边界、目标高度和“朝下抓取”姿态是否适合你的桌面任务。
    busy_ = false;
  }

  void ensure_move_group()
  {
    if (move_group_) {
      return;
    }

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setPoseReferenceFrame(pose_reference_frame_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setPlanningTime(3.0);
    move_group_->setNumPlanningAttempts(3);
  }

  bool is_frame_supported(const std::string & frame_id) const
  {
    return frame_id.empty() || frame_id == pose_reference_frame_;
  }

  bool is_in_workspace(double x, double y) const
  {
    return x >= workspace_min_x_ && x <= workspace_max_x_ &&
           y >= workspace_min_y_ && y <= workspace_max_y_;
  }

  geometry_msgs::msg::Pose map_goal_to_target_pose(
    const geometry_msgs::msg::PoseStamped & input,
    double yaw) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = input.pose.position.x;
    pose.position.y = input.pose.position.y;
    pose.position.z = target_height_;

    tf2::Quaternion q;
    q.setRPY(downward_roll_rad_, downward_pitch_rad_, yaw);
    q.normalize();
    pose.orientation = tf2::toMsg(q);
    return pose;
  }

  double extract_yaw(const geometry_msgs::msg::Quaternion & q) const
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  std::string planning_group_;
  std::string pose_reference_frame_;
  std::string end_effector_link_;
  std::string goal_topic_;
  bool execute_plan_{true};
  double target_height_{0.22};
  double workspace_min_x_{0.10};
  double workspace_max_x_{0.45};
  double workspace_min_y_{-0.25};
  double workspace_max_y_{0.25};
  double downward_roll_rad_{M_PI};
  double downward_pitch_rad_{0.0};
  bool busy_{false};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ur3GoalPoseExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
