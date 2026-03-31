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
    constexpr double kWarningVelocityThresholdRadS = 0.50;
    constexpr double kCriticalVelocityThresholdRadS = 1.00;
    constexpr double kWarningPositionThresholdRad = 2.50;
    constexpr double kCriticalPositionThresholdRad = 2.90;

    double max_abs_position = 0.0;
    double max_abs_velocity = 0.0;
    bool has_warning_alarm = false;
    bool has_critical_alarm = false;
    std::string first_warning_reason;
    std::string first_critical_reason;
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
        const double pos = last_msg_->position[idx];
        const double abs_pos = std::abs(pos);
        max_abs_position = std::max(max_abs_position, abs_pos);
        oss << " pos=" << pos << "rad("
            << (pos * kRadToDeg) << "deg)";
        if (abs_pos >= kCriticalPositionThresholdRad) {
          has_critical_alarm = true;
          if (first_critical_reason.empty()) {
            first_critical_reason = joint + " pos=" + std::to_string(pos) + "rad";
          }
        } else if (abs_pos >= kWarningPositionThresholdRad) {
          has_warning_alarm = true;
          if (first_warning_reason.empty()) {
            first_warning_reason = joint + " pos=" + std::to_string(pos) + "rad";
          }
        }
      } else {
        oss << " pos=N/A";
      }

      if (has_vel) {
        const double vel = last_msg_->velocity[idx];
        const double abs_vel = std::abs(vel);
        max_abs_velocity = std::max(max_abs_velocity, abs_vel);
        oss << " vel=" << vel;
        if (abs_vel >= kCriticalVelocityThresholdRadS) {
          has_critical_alarm = true;
          if (first_critical_reason.empty()) {
            first_critical_reason = joint + " vel=" + std::to_string(vel) + "rad/s";
          }
        } else if (abs_vel >= kWarningVelocityThresholdRadS) {
          has_warning_alarm = true;
          if (first_warning_reason.empty()) {
            first_warning_reason = joint + " vel=" + std::to_string(vel) + "rad/s";
          }
        }
      } else {
        oss << " vel=N/A";
      }
    }

    oss << " | max_abs_position=" << max_abs_position << " rad";
    oss << " | max_abs_velocity=" << max_abs_velocity << " rad/s";
    if (has_critical_alarm) {
      oss << " | alarm=CRITICAL(" << first_critical_reason << ")";
      RCLCPP_ERROR(this->get_logger(), "%s", oss.str().c_str());
      return;
    }
    if (has_warning_alarm) {
      oss << " | alarm=WARNING(" << first_warning_reason << ")";
      RCLCPP_WARN(this->get_logger(), "%s", oss.str().c_str());
      return;
    }
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
