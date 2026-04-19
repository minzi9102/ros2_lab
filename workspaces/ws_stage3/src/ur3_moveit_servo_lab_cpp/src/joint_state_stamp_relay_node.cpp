#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStateStampRelayNode : public rclcpp::Node
{
public:
  JointStateStampRelayNode()
  : Node("task7e_joint_state_stamp_relay")
  {
    source_topic_ = this->declare_parameter<std::string>("source_topic", "/joint_states");
    target_topic_ =
      this->declare_parameter<std::string>("target_topic", "/task7e/joint_states_fresh");
    publish_period_sec_ = this->declare_parameter<double>("publish_period_sec", 0.02);
    position_epsilon_ = this->declare_parameter<double>("position_epsilon", 1e-9);

    const auto qos = rclcpp::SensorDataQoS();
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      source_topic_,
      qos,
      std::bind(&JointStateStampRelayNode::on_joint_state, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(target_topic_, qos);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_period_sec_)),
      std::bind(&JointStateStampRelayNode::publish_latest, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Relaying joint states from %s to %s at %.3fs with fresh header stamps.",
      source_topic_.c_str(),
      target_topic_.c_str(),
      publish_period_sec_);
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::scoped_lock lock(mutex_);
    latest_msg_ = msg;
    if (!logged_source_msg_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received the first source JointState on %s.",
        source_topic_.c_str());
      logged_source_msg_ = true;
    }
  }

  void publish_latest()
  {
    sensor_msgs::msg::JointState::SharedPtr latest_msg;
    {
      std::scoped_lock lock(mutex_);
      latest_msg = latest_msg_;
    }

    if (!latest_msg) {
      return;
    }

    sensor_msgs::msg::JointState relay_msg = *latest_msg;
    relay_msg.header.stamp = this->now();
    // MoveIt's CurrentStateMonitor only wakes waiters when the numeric joint state changes.
    // For a stationary mock robot, forwarding identical positions with only a fresh stamp can
    // still time out Servo startup, so we introduce an experiment-local epsilon toggle here.
    if (!relay_msg.position.empty() && position_epsilon_ > 0.0) {
      relay_toggle_positive_ = !relay_toggle_positive_;
      const double signed_epsilon = relay_toggle_positive_ ? position_epsilon_ : -position_epsilon_;
      relay_msg.position.front() += signed_epsilon;
    }
    publisher_->publish(relay_msg);

    if (!logged_relay_msg_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing fresh-stamped JointState messages on %s.",
        target_topic_.c_str());
      logged_relay_msg_ = true;
    }
  }

  std::string source_topic_;
  std::string target_topic_;
  double publish_period_sec_{};
  double position_epsilon_{};
  bool logged_source_msg_{ false };
  bool logged_relay_msg_{ false };
  bool relay_toggle_positive_{ false };
  std::mutex mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateStampRelayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
