#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Task 7D 的最小目标：
// 1. 订阅 RViz 发出的 /goal_pose
// 2. 把二维点击语义映射成一个“可规划的末端目标位姿”
// 3. 先做本地输入过滤，再调用 MoveIt 自动规划与执行
class Ur3GoalPoseExecutorNode : public rclcpp::Node
{
public:
  Ur3GoalPoseExecutorNode()
  : Node("ur3_goal_pose_executor")
  {
    // 这里声明的都是 7D 的“任务语义参数”，不是底层 driver 参数。
    // 你后续主要会围绕这些值思考：哪些点击点应该被接受、末端应当飞到多高、姿态要不要保留 yaw。
    planning_group_ = this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    pose_reference_frame_ =
      this->declare_parameter<std::string>("pose_reference_frame", "base_link");
    end_effector_link_ = this->declare_parameter<std::string>("end_effector_link", "tool0");
    goal_topic_ = this->declare_parameter<std::string>("goal_topic", "/goal_pose");
    execute_plan_ = this->declare_parameter<bool>("execute_plan", true);
    target_height_ = this->declare_parameter<double>("target_height", 0.1519);
    workspace_min_x_ = this->declare_parameter<double>("workspace_min_x", 0.0);
    workspace_max_x_ = this->declare_parameter<double>("workspace_max_x", 0.6);
    workspace_min_y_ = this->declare_parameter<double>("workspace_min_y", -0.3);
    workspace_max_y_ = this->declare_parameter<double>("workspace_max_y", 0.3);
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
    // 这个 demo 故意做成串行 one-at-a-time：
    // 上一个请求还没结束时，不接受新的点击，避免多个规划/执行相互打架。
    if (busy_) {
      RCLCPP_WARN(this->get_logger(), "Planner is busy. Ignoring the new /goal_pose.");
      return;
    }

    busy_ = true;
    ensure_move_group();

    // /goal_pose 是从 RViz 发来的“人类交互输入”，不一定和当前规划参考坐标系一致。
    // 如果 frame 不对，先在本地拒绝，避免把语义不清楚的目标送进规划器。
    if (!is_frame_supported(msg->header.frame_id)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unsupported goal frame '%s'. Expected '%s' or an empty frame.",
        msg->header.frame_id.c_str(),
        pose_reference_frame_.c_str());
      busy_ = false;
      return;
    }

    // 这里只做最小的桌面任务过滤：x/y 必须落在我们事先定义的工作区里。
    // 这样可以把“明显不该规划的点击”挡在 MoveIt 外面。
    if (!is_in_workspace(msg->pose.position.x, msg->pose.position.y)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Rejected goal outside workspace: x=%.3f y=%.3f",
        msg->pose.position.x,
        msg->pose.position.y);
      busy_ = false;
      return;
    }

    // RViz 的 2D Goal Pose 只有平面上的位姿语义。
    // 所以这里先提取 yaw，再把输入点映射成一个固定高度、固定朝下的 3D 末端目标。
    const double yaw = extract_yaw(msg->pose.orientation);
    const geometry_msgs::msg::Pose target_pose = map_goal_to_target_pose(*msg, yaw);

    // MoveGroupInterface 是当前节点对外部 /move_group 的客户端封装。
    // 真正的规划在 move_group 服务端完成；这里负责清空旧目标、设置新目标、发起请求。
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

    // 7D 的默认行为是“规划成功就立即执行”，这样才能形成点击即动起来的最小 demo。
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

    // 延迟创建 MoveGroupInterface，避免节点对象还没完全构造好时就去加载 MoveIt 相关资源。
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setPoseReferenceFrame(pose_reference_frame_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setPlanningTime(3.0);
    move_group_->setNumPlanningAttempts(3);
  }

  bool is_frame_supported(const std::string & frame_id) const
  {
    // 空 frame 视为“接受默认参考系”，否则必须和当前任务假定的参考系一致。
    return frame_id.empty() || frame_id == pose_reference_frame_;
  }

  bool is_in_workspace(double x, double y) const
  {
    // 这里只看平面范围，因为 7D 的输入本来就是“桌面上的二维点击”。
    return x >= workspace_min_x_ && x <= workspace_max_x_ &&
           y >= workspace_min_y_ && y <= workspace_max_y_;
  }

  geometry_msgs::msg::Pose map_goal_to_target_pose(
    const geometry_msgs::msg::PoseStamped & input,
    double yaw) const
  {
    geometry_msgs::msg::Pose pose;
    // 平移部分保留点击得到的 x/y，但 z 不直接相信输入，而是替换成任务定义的安全高度。
    pose.position.x = input.pose.position.x;
    pose.position.y = input.pose.position.y;
    pose.position.z = target_height_;

    // 姿态部分采用“固定朝下 + 保留输入 yaw”的默认策略。
    // 这就是 7D 当前最值得你自己审查的语义映射。
    tf2::Quaternion q;
    q.setRPY(downward_roll_rad_, downward_pitch_rad_, yaw);
    q.normalize();
    pose.orientation = tf2::toMsg(q);
    return pose;
  }

  double extract_yaw(const geometry_msgs::msg::Quaternion & q) const
  {
    // 只取平面朝向，忽略输入里可能携带的 roll/pitch。
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  std::string planning_group_;
  std::string pose_reference_frame_;
  std::string end_effector_link_;
  std::string goal_topic_;
  bool execute_plan_{true};
  double target_height_{0.1519};
  double workspace_min_x_{0.0};
  double workspace_max_x_{0.6};
  double workspace_min_y_{-0.3};
  double workspace_max_y_{0.3};
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
