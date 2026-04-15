#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

class Ur3SceneCartesianDemoNode : public rclcpp::Node
{
public:
  Ur3SceneCartesianDemoNode()
  : Node("ur3_scene_cartesian_demo")
  {
    planning_group_ = this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    pose_reference_frame_ =
      this->declare_parameter<std::string>("pose_reference_frame", "base_link");
    end_effector_link_ = this->declare_parameter<std::string>("end_effector_link", "tool0");
    demo_mode_ = this->declare_parameter<std::string>("demo_mode", "both");
    execute_plan_ = this->declare_parameter<bool>("execute_plan", false);
    table_object_id_ = this->declare_parameter<std::string>("table_object_id", "training_table");
    table_dimensions_ = this->declare_parameter<std::vector<double>>(
      "table_dimensions",
      std::vector<double>{0.70, 0.90, 0.05});
    table_position_ = this->declare_parameter<std::vector<double>>(
      "table_position",
      std::vector<double>{0.35, 0.0, -0.03});
    waypoint_a_position_ = this->declare_parameter<std::vector<double>>(
      "waypoint_a_position",
      std::vector<double>{0.25, -0.20, 0.22});
    waypoint_b_position_ = this->declare_parameter<std::vector<double>>(
      "waypoint_b_position",
      std::vector<double>{0.25, 0.00, 0.22});
    waypoint_c_position_ = this->declare_parameter<std::vector<double>>(
      "waypoint_c_position",
      std::vector<double>{0.25, 0.20, 0.22});
    common_orientation_xyzw_ = this->declare_parameter<std::vector<double>>(
      "common_orientation_xyzw",
      std::vector<double>{0.7071, 0.0, 0.0, 0.7071});
    eef_step_ = this->declare_parameter<double>("eef_step", 0.01);
    jump_threshold_ = this->declare_parameter<double>("jump_threshold", 0.0);
    min_cartesian_fraction_ = this->declare_parameter<double>("min_cartesian_fraction", 0.95);

    // 这里沿用 7B 的 one-shot 模式：节点先完整创建，再在定时器回调里做一次实验流程。
    // 这样可以避免对象尚未就绪时就去创建 MoveGroupInterface。
    start_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&Ur3SceneCartesianDemoNode::run_once, this));

    RCLCPP_INFO(
      this->get_logger(),
      "scene_cartesian_demo_node started. mode=%s execute_plan=%s",
      demo_mode_.c_str(),
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
      // 先准备规划客户端，再把桌面障碍物塞进 Planning Scene。
      // 这里影响的是“规划能不能通过碰撞检查”，不是底层 controller 的行为。
      ensure_move_group();
      add_table_collision_object();

      if (demo_mode_ == "pose" || demo_mode_ == "both") {
        // pose demo 只问一件事：能否找到“到达 A 点”的任意合法轨迹。
        run_pose_planning_demo();
      }

      if (demo_mode_ == "cartesian" || demo_mode_ == "both") {
        // cartesian demo 更严格：要求末端按笛卡尔路径依次经过多个 waypoint。
        run_cartesian_demo();
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Task 7C failed: %s", ex.what());
    }

    // 实验结束后把临时桌面移除，避免污染后续手动测试或下一次启动。
    remove_table_collision_object();
    request_shutdown("Task 7C scaffold finished");
  }

  void ensure_move_group()
  {
    if (move_group_) {
      return;
    }

    // MoveGroupInterface 是当前节点对 move_group 服务端的客户端包装。
    // 真正的规划仍发生在外部 move_group 节点里，这里负责设置目标与发起请求。
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setPoseReferenceFrame(pose_reference_frame_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setPlanningTime(3.0);
    move_group_->setNumPlanningAttempts(3);
  }

  void add_table_collision_object()
  {
    validate_vector(table_dimensions_, 3, "table_dimensions");
    validate_vector(table_position_, 3, "table_position");

    // CollisionObject 描述“场景里多了一个盒子障碍物”，它会参与后续碰撞检查。
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = pose_reference_frame_;
    table.id = table_object_id_;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {
      table_dimensions_[0],
      table_dimensions_[1],
      table_dimensions_[2]};

    geometry_msgs::msg::Pose pose;
    pose.position.x = table_position_[0];
    pose.position.y = table_position_[1];
    pose.position.z = table_position_[2];
    pose.orientation.w = 1.0;

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    // applyCollisionObject 会把桌面写入 Planning Scene，之后新的规划请求都会考虑它。
    planning_scene_interface_.applyCollisionObject(table);

    RCLCPP_INFO(
      this->get_logger(),
      "Added table collision object '%s' at (%.3f, %.3f, %.3f) size=(%.3f, %.3f, %.3f)",
      table_object_id_.c_str(),
      pose.position.x,
      pose.position.y,
      pose.position.z,
      table_dimensions_[0],
      table_dimensions_[1],
      table_dimensions_[2]);

    // TODO(human): 请亲自决定桌面的真实尺寸与位置，不要直接迷信这里的默认值。
  }

  void remove_table_collision_object()
  {
    planning_scene_interface_.removeCollisionObjects({table_object_id_});
  }

  void run_pose_planning_demo()
  {
    // pose demo 把 waypoint A 当作一个“目标位姿”，并不要求中间路径必须是直线。
    geometry_msgs::msg::Pose target_pose = make_pose(waypoint_a_position_);

    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(target_pose, end_effector_link_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success =
      static_cast<bool>(move_group_->plan(plan));

    RCLCPP_INFO(
      this->get_logger(),
      "Pose planning result: success=%s target=(%.3f, %.3f, %.3f)",
      success ? "true" : "false",
      target_pose.position.x,
      target_pose.position.y,
      target_pose.position.z);

    if (success && execute_plan_) {
      // 只有 execute_plan_=true 时，机器人当前状态才会真的更新到规划结果终点附近。
      move_group_->execute(plan);
    }
  }

  void run_cartesian_demo()
  {
    // 这里的 A/B/C 不是三个独立目标，而是“希望末端依次穿过的路径点”。
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(make_pose(waypoint_a_position_));
    waypoints.push_back(make_pose(waypoint_b_position_));
    waypoints.push_back(make_pose(waypoint_c_position_));

    moveit_msgs::msg::RobotTrajectory trajectory;
    // computeCartesianPath 从“当前真实机器人状态”出发做笛卡尔插值。
    // 如果 execute_plan_=false，前面的 pose planning 成功也不会把起点推进到 A。
    const double fraction =
      move_group_->computeCartesianPath(waypoints, eef_step_, trajectory);

    RCLCPP_INFO(
      this->get_logger(),
      "Cartesian path result: fraction=%.3f eef_step=%.3f jump_threshold=%.3f",
      fraction,
      eef_step_,
      jump_threshold_);

    // fraction 反映“整条笛卡尔路径有多少比例真的算出来了”，不是执行成功率。
    if (fraction < min_cartesian_fraction_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Cartesian fraction %.3f is below threshold %.3f",
        fraction,
        min_cartesian_fraction_);
    } else if (execute_plan_) {
      move_group_->execute(trajectory);
    }

    // TODO(human): 这里的 3 个抓取点位和 min_cartesian_fraction 只是保守默认值。
    // TODO(human): 你需要根据“桌面位置、末端姿态、可达性”亲自调整这些参数并解释原因。
  }

  geometry_msgs::msg::Pose make_pose(const std::vector<double> & position) const
  {
    validate_vector(position, 3, "waypoint_position");
    validate_vector(common_orientation_xyzw_, 4, "common_orientation_xyzw");

    // 这里仅负责把参数拼成 Pose，不在这一层判断“它是否可达”。
    geometry_msgs::msg::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.x = common_orientation_xyzw_[0];
    pose.orientation.y = common_orientation_xyzw_[1];
    pose.orientation.z = common_orientation_xyzw_[2];
    pose.orientation.w = common_orientation_xyzw_[3];
    return pose;
  }

  void validate_vector(
    const std::vector<double> & values,
    std::size_t expected_size,
    const std::string & label) const
  {
    // 提前在本地参数层做格式检查，避免把尺寸错误的数据带进规划调用里。
    if (values.size() != expected_size) {
      throw std::runtime_error(label + " must contain " + std::to_string(expected_size) + " values.");
    }
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
  std::string demo_mode_;
  bool execute_plan_{false};
  std::string table_object_id_;
  std::vector<double> table_dimensions_;
  std::vector<double> table_position_;
  std::vector<double> waypoint_a_position_;
  std::vector<double> waypoint_b_position_;
  std::vector<double> waypoint_c_position_;
  std::vector<double> common_orientation_xyzw_;
  double eef_step_{0.01};
  double jump_threshold_{0.0};
  double min_cartesian_fraction_{0.95};

  bool started_{false};
  bool shutdown_requested_{false};

  rclcpp::TimerBase::SharedPtr start_timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 节点本身不循环发命令，spin 的主要作用是等待定时器触发 one-shot 流程并处理回调。
  auto node = std::make_shared<Ur3SceneCartesianDemoNode>();
  rclcpp::spin(node);
  return 0;
}
