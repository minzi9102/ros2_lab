#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class Ur3StateMonitorNode : public rclcpp::Node
{
public:
  Ur3StateMonitorNode()
  : Node("ur3_state_monitor_node")
  {
    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    print_rate_hz_ = this->declare_parameter<double>("print_rate_hz", 1.0);
    expected_joints_ = this->declare_parameter<std::vector<std::string>>(
      "expected_joints",
      std::vector<std::string>{
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"});

    if (print_rate_hz_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "print_rate_hz <= 0.0, fallback to 1.0 Hz.");
      print_rate_hz_ = 1.0;
    }

    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&Ur3StateMonitorNode::on_joint_state, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / print_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&Ur3StateMonitorNode::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "UR3 state monitor started. topic=%s, print_rate_hz=%.2f",
      joint_state_topic_.c_str(),
      print_rate_hz_);
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_msg_ = msg;
    last_msg_time_ = this->now();
  }

  void on_timer()
  {
    if (!last_msg_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Waiting for %s ...", joint_state_topic_.c_str());
      return;
    }

    std::unordered_map<std::string, std::size_t> index_by_name;
    index_by_name.reserve(last_msg_->name.size());
    for (std::size_t i = 0; i < last_msg_->name.size(); ++i) {
      index_by_name[last_msg_->name[i]] = i;
    }

    constexpr double kRadToDeg = 180.0 / M_PI;
    double max_abs_velocity = 0.0;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "[UR3 monitor] age="
        << (this->now() - last_msg_time_).seconds()
        << "s";

    for (const auto & joint : expected_joints_) {
      const auto it = index_by_name.find(joint);
      if (it == index_by_name.end()) {
        oss << " | " << joint << "=N/A";
        continue;
      }

      const std::size_t idx = it->second;
      const bool has_pos = idx < last_msg_->position.size();
      const bool has_vel = idx < last_msg_->velocity.size();

      oss << " | " << joint << ":";
      if (has_pos) {
        oss << " pos=" << last_msg_->position[idx] << "rad("
            << (last_msg_->position[idx] * kRadToDeg) << "deg)";
      } else {
        oss << " pos=N/A";
      }

      if (has_vel) {
        const double vel = last_msg_->velocity[idx];
        max_abs_velocity = std::max(max_abs_velocity, std::abs(vel));
        oss << " vel=" << vel;
      } else {
        oss << " vel=N/A";
      }
    }

    // TODO(human): 根据手术场景定义速度/位姿等报警阈值，并在这里加入告警逻辑。
    oss << " | max_abs_velocity=" << max_abs_velocity << " rad/s";
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

  std::string joint_state_topic_;
  double print_rate_hz_;
  std::vector<std::string> expected_joints_;

  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::JointState::SharedPtr last_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ur3StateMonitorNode>());
  rclcpp::shutdown();
  return 0;
}

