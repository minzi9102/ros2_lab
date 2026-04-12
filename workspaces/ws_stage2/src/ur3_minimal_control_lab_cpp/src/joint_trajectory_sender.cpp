#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class Ur3JointTrajectorySenderNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory =
    rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  Ur3JointTrajectorySenderNode()
  : Node("ur3_joint_trajectory_sender_cpp")
  {
    action_name_ = this->declare_parameter<std::string>(
      "action_name", "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    server_wait_timeout_sec_ = this->declare_parameter<double>("server_wait_timeout_sec", 10.0);
    start_delay_sec_ = this->declare_parameter<double>("start_delay_sec", 1.0);
    joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "joint_names",
      std::vector<std::string>{
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"});

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, action_name_);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&Ur3JointTrajectorySenderNode::on_joint_state, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(start_delay_sec_);
    start_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&Ur3JointTrajectorySenderNode::maybe_send_goal, this));

    RCLCPP_INFO(
      this->get_logger(),
      "joint_trajectory_sender_cpp started. action=%s topic=%s",
      action_name_.c_str(), joint_state_topic_.c_str());
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_positions_by_name_.clear();
    latest_positions_by_name_.reserve(msg->name.size());

    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      if (i < msg->position.size()) {
        latest_positions_by_name_[msg->name[i]] = msg->position[i];
      }
    }
  }

  void maybe_send_goal()
  {
    if (goal_sent_ || shutdown_requested_) {
      return;
    }

    std::vector<double> current_positions;
    try {
      current_positions = ordered_current_positions();
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for FollowJointTrajectory action server...");
    if (!action_client_->wait_for_action_server(std::chrono::duration<double>(server_wait_timeout_sec_))) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Action server unavailable: %s (waited %.1fs)",
        action_name_.c_str(), server_wait_timeout_sec_);
      request_shutdown("Action server unavailable");
      return;
    }

    FollowJointTrajectory::Goal goal_msg;
    try {
      goal_msg = build_demo_goal(current_positions);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      request_shutdown("Goal building failed");
      return;
    }

    goal_sent_ = true;
    start_timer_->cancel();

    try {
      send_goal_request(goal_msg);
    } catch (const std::logic_error & ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      request_shutdown("TODO(human) not finished");
    }
  }

  std::vector<double> ordered_current_positions() const
  {
    if (latest_positions_by_name_.empty()) {
      throw std::runtime_error("Waiting for current /joint_states before sending trajectory.");
    }

    std::vector<double> ordered_positions;
    ordered_positions.reserve(joint_names_.size());
    std::vector<std::string> missing_names;

    for (const auto & joint_name : joint_names_) {
      const auto it = latest_positions_by_name_.find(joint_name);
      if (it == latest_positions_by_name_.end()) {
        missing_names.push_back(joint_name);
        continue;
      }
      ordered_positions.push_back(it->second);
    }

    if (!missing_names.empty()) {
      std::string message = "Current /joint_states is missing joints:";
      for (const auto & name : missing_names) {
        message += " " + name;
      }
      throw std::runtime_error(message);
    }

    return ordered_positions;
  }

  FollowJointTrajectory::Goal build_demo_goal(const std::vector<double> & current_positions)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = joint_names_;

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
    points.push_back(make_point(current_positions, 1));

    auto middle_positions = current_positions;
    middle_positions[0] += 1.0;
    middle_positions[1] += 1.0;
    middle_positions[2] -= 1.0;
    points.push_back(make_point(middle_positions, 3));
    points.push_back(make_point(current_positions, 5));

    validate_trajectory_points(points);
    trajectory.points = points;

    FollowJointTrajectory::Goal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance = make_duration(1);
    return goal;
  }

  void send_goal_request(const FollowJointTrajectory::Goal & goal_msg)
  {
    // TODO(human): 请在这里亲自补写 C++ Action Client 的发送逻辑。
    // 你需要完成：
    // 1. 创建 SendGoalOptions
    // 2. 绑定 goal_response_callback / feedback_callback / result_callback
    // 3. 调用 action_client_->async_send_goal(goal_msg, options)
    // 建议先对照 5C 的 Python 版，再理解 rclcpp_action 的三类回调签名差异。
    (void)goal_msg;
    throw std::logic_error(
            "TODO(human): 请补写 send_goal_request()，完成 SendGoalOptions 绑定与 async_send_goal 调用。");
  }

  void on_goal_response(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(), "Goal rejected by controller.");
      request_shutdown("Goal rejected");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted, waiting for result...");
  }

  void on_feedback(
    GoalHandleFollowJointTrajectory::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Feedback received: desired_len=%zu actual_len=%zu",
      feedback->desired.positions.size(),
      feedback->actual.positions.size());
  }

  void on_result(const GoalHandleFollowJointTrajectory::WrappedResult & result)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Result received: status=%d error_code=%d error_string='%s'",
      static_cast<int>(result.code),
      result.result->error_code,
      result.result->error_string.c_str());
    request_shutdown("Goal finished");
  }

  trajectory_msgs::msg::JointTrajectoryPoint make_point(
    const std::vector<double> & positions,
    int sec) const
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities = std::vector<double>(joint_names_.size(), 0.0);
    point.time_from_start = make_duration(sec);
    return point;
  }

  builtin_interfaces::msg::Duration make_duration(int sec) const
  {
    builtin_interfaces::msg::Duration duration;
    duration.sec = sec;
    duration.nanosec = 0;
    return duration;
  }

  void validate_trajectory_points(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> & points) const
  {
    if (points.empty()) {
      throw std::runtime_error("trajectory.points is empty");
    }

    const auto expected_len = joint_names_.size();
    std::int64_t prev_time_ns = -1;

    for (std::size_t idx = 0; idx < points.size(); ++idx) {
      const auto & point = points[idx];
      if (point.positions.size() != expected_len) {
        throw std::runtime_error(
                "positions length mismatch at point[" + std::to_string(idx) + "]");
      }
      if (!point.velocities.empty() && point.velocities.size() != expected_len) {
        throw std::runtime_error(
                "velocities length mismatch at point[" + std::to_string(idx) + "]");
      }

      const std::int64_t time_ns =
        static_cast<std::int64_t>(point.time_from_start.sec) * 1000000000LL +
        static_cast<std::int64_t>(point.time_from_start.nanosec);
      if (time_ns <= prev_time_ns) {
        throw std::runtime_error(
                "time_from_start is not strictly increasing at point[" + std::to_string(idx) + "]");
      }
      prev_time_ns = time_ns;
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

  std::string action_name_;
  std::string joint_state_topic_;
  double server_wait_timeout_sec_{10.0};
  double start_delay_sec_{1.0};
  std::vector<std::string> joint_names_;

  bool goal_sent_{false};
  bool shutdown_requested_{false};
  std::unordered_map<std::string, double> latest_positions_by_name_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr start_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ur3JointTrajectorySenderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
