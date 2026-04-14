#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

class Ur3MoveGroupPlannerNode : public rclcpp::Node
{
public:
  Ur3MoveGroupPlannerNode()
  : Node("ur3_move_group_planner")
  {
    planning_group_ = this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    pose_reference_frame_ =
      this->declare_parameter<std::string>("pose_reference_frame", "base_link");
    end_effector_link_ = this->declare_parameter<std::string>("end_effector_link", "tool0");
    target_mode_ = this->declare_parameter<std::string>("target_mode", "joint");
    execute_plan_ = this->declare_parameter<bool>("execute_plan", false);
    planning_time_sec_ = this->declare_parameter<double>("planning_time_sec", 3.0);
    num_planning_attempts_ = this->declare_parameter<int>("num_planning_attempts", 3);
    joint_target_ = this->declare_parameter<std::vector<double>>(
      "joint_target",
      std::vector<double>{0.0, -1.57, 0.0, -1.57, 0.0, 0.0});
    pose_position_ = this->declare_parameter<std::vector<double>>(
      "pose_position",
      std::vector<double>{0.25, -0.20, 0.35});
    pose_orientation_xyzw_ = this->declare_parameter<std::vector<double>>(
      "pose_orientation_xyzw",
      std::vector<double>{0.7071, 0.0, 0.0, 0.7071});

    start_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&Ur3MoveGroupPlannerNode::run_once, this));

    RCLCPP_INFO(
      this->get_logger(),
      "move_group_planner_node started. group=%s target_mode=%s execute_plan=%s",
      planning_group_.c_str(),
      target_mode_.c_str(),
      execute_plan_ ? "true" : "false");
  }

private:
  void run_once()
  {
    if (started_) {
      return;
    }
    started_ = true;
    start_timer_->cancel();

    try {
      ensure_move_group();
      const bool target_ready = apply_requested_target();
      if (!target_ready) {
        request_shutdown("TODO(human) target logic not finished");
        return;
      }

      const bool flow_finished = run_plan_flow();
      if (!flow_finished) {
        request_shutdown("TODO(human) plan flow not finished");
        return;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Task 7B failed: %s", ex.what());
      request_shutdown("Planning scaffold failed");
    }
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
    move_group_->setPlanningTime(planning_time_sec_);
    move_group_->setNumPlanningAttempts(num_planning_attempts_);
  }

  bool apply_requested_target()
  {
    if (target_mode_ == "joint") {
      return apply_joint_target();
    }

    if (target_mode_ == "pose") {
      return apply_pose_target();
    }

    throw std::runtime_error("target_mode must be either 'joint' or 'pose'.");
  }

  bool apply_joint_target()
  {
    validate_joint_target();
    RCLCPP_INFO(
      this->get_logger(),
      "Joint target scaffold is ready. size=%zu values[0]=%.3f",
      joint_target_.size(),
      joint_target_.front());

    // TODO(human): 在这里调用 setJointValueTarget(joint_target_)。
    move_group_->setJointValueTarget(joint_target_);
    // TODO(human): 你需要判断“关节目标的尺寸、顺序、语义”是否和 ur_manipulator 一致。
    if (!move_group_->setJointValueTarget(joint_target_)) {
      RCLCPP_INFO(this->get_logger(), "Joint target is set successfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set joint target.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Joint target is set successfully.");
    return true;
  }

  bool apply_pose_target()
  {
    const geometry_msgs::msg::Pose pose = build_pose_target();
    RCLCPP_INFO(
      this->get_logger(),
      "Pose target scaffold is ready. position=(%.3f, %.3f, %.3f) link=%s frame=%s",
      pose.position.x,
      pose.position.y,
      pose.position.z,
      end_effector_link_.c_str(),
      pose_reference_frame_.c_str());

    // TODO(human): 在这里调用 setPoseTarget(pose, end_effector_link_)。
      move_group_->setPoseTarget(pose, end_effector_link_);
    // TODO(human): 你需要判断这个姿态是否可达，以及末端姿态语义是否合理。
      if (!move_group_->setPoseTarget(pose, end_effector_link_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set pose target.");
        return false;
      } else {
        RCLCPP_INFO(this->get_logger(), "Pose target is set successfully.");
      }
    return true;
  }

  bool run_plan_flow()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Planning flow scaffold is ready. execute_plan=%s",
      execute_plan_ ? "true" : "false");

    // TODO(human): 在这里创建 MoveGroupInterface::Plan，并调用 plan()。
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Planning failed.");
        return false;
      }
  
      RCLCPP_INFO(this->get_logger(), "Planning succeeded.");
  
      if (execute_plan_) {  
        // TODO(human): 在这里调用 execute()。
        const bool execute_success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!execute_success) {
          RCLCPP_ERROR(this->get_logger(), "Execution failed.");
          return false;
        }
        RCLCPP_INFO(this->get_logger(), "Execution succeeded.");
      }
    // TODO(human): 你需要自己决定 plan 失败时是直接退出、打印更多上下文，还是做一次保守重试。
    // TODO(human): 只有在 plan 成功时，才能根据 execute_plan_ 决定是否 execute()。 
    return true;
  }

  void validate_joint_target() const
  {
    if (joint_target_.size() != 6) {
      throw std::runtime_error("joint_target must contain exactly 6 values for UR3.");
    }
  }

  geometry_msgs::msg::Pose build_pose_target() const
  {
    if (pose_position_.size() != 3) {
      throw std::runtime_error("pose_position must contain exactly 3 values.");
    }

    if (pose_orientation_xyzw_.size() != 4) {
      throw std::runtime_error("pose_orientation_xyzw must contain exactly 4 values.");
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = pose_position_[0];
    pose.position.y = pose_position_[1];
    pose.position.z = pose_position_[2];
    pose.orientation.x = pose_orientation_xyzw_[0];
    pose.orientation.y = pose_orientation_xyzw_[1];
    pose.orientation.z = pose_orientation_xyzw_[2];
    pose.orientation.w = pose_orientation_xyzw_[3];
    return pose;
  }

  void request_shutdown(const std::string & reason)
  {
    if (shutdown_requested_) {
      return;
    }

    shutdown_requested_ = true;
    RCLCPP_INFO(this->get_logger(), "Shutting down node: %s", reason.c_str());
    rclcpp::shutdown();
  }

  std::string planning_group_;
  std::string pose_reference_frame_;
  std::string end_effector_link_;
  std::string target_mode_;
  bool execute_plan_{false};
  double planning_time_sec_{3.0};
  int num_planning_attempts_{3};
  std::vector<double> joint_target_;
  std::vector<double> pose_position_;
  std::vector<double> pose_orientation_xyzw_;

  bool started_{false};
  bool shutdown_requested_{false};

  rclcpp::TimerBase::SharedPtr start_timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ur3MoveGroupPlannerNode>();
  rclcpp::spin(node);
  return 0;
}
