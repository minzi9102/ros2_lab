#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "moveit_msgs/msg/servo_status.hpp"
#include "rclcpp/rclcpp.hpp"

class Ur3ServoTwistCommanderNode : public rclcpp::Node
{
public:
  Ur3ServoTwistCommanderNode()
  : Node("ur3_servo_twist_commander")
  {
    command_topic_ =
      this->declare_parameter<std::string>("command_topic", "/servo_node/delta_twist_cmds");
    status_topic_ = this->declare_parameter<std::string>("status_topic", "/servo_node/status");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "tool0");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    start_delay_sec_ = this->declare_parameter<double>("start_delay_sec", 1.0);
    run_duration_sec_ = this->declare_parameter<double>("run_duration_sec", 1.5);
    halt_publish_count_ = this->declare_parameter<int>("halt_publish_count", 5);
    linear_x_ = this->declare_parameter<double>("linear_x", 0.02);
    linear_y_ = this->declare_parameter<double>("linear_y", 0.00);
    linear_z_ = this->declare_parameter<double>("linear_z", 0.00);
    angular_x_ = this->declare_parameter<double>("angular_x", 0.00);
    angular_y_ = this->declare_parameter<double>("angular_y", 0.00);
    angular_z_ = this->declare_parameter<double>("angular_z", 0.00);

    command_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(command_topic_, 10);
    status_sub_ = this->create_subscription<moveit_msgs::msg::ServoStatus>(
      status_topic_,
      10,
      std::bind(&Ur3ServoTwistCommanderNode::on_status, this, std::placeholders::_1));

    start_time_ = this->now() + rclcpp::Duration::from_seconds(start_delay_sec_);
    stop_time_ = start_time_ + rclcpp::Duration::from_seconds(run_duration_sec_);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&Ur3ServoTwistCommanderNode::on_publish_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "servo_twist_commander_node started. command_topic=%s status_topic=%s duration=%.2fs rate=%.1fHz",
      command_topic_.c_str(),
      status_topic_.c_str(),
      run_duration_sec_,
      publish_rate_hz_);

    // TODO(human): 请亲自调整速度上限、运行时长和停机策略，不要直接依赖这里的默认值。
  }

private:
  void on_publish_timer()
  {
    const rclcpp::Time now = this->now();

    if (now < start_time_) {
      return;
    }

    if (now <= stop_time_) {
      publish_twist(linear_x_, linear_y_, linear_z_, angular_x_, angular_y_, angular_z_);
      return;
    }

    if (halt_messages_sent_ < halt_publish_count_) {
      publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      ++halt_messages_sent_;
      return;
    }

    publish_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Servo command sequence finished, shutting down.");
    rclcpp::shutdown();
  }

  void publish_twist(
    double linear_x,
    double linear_y,
    double linear_z,
    double angular_x,
    double angular_y,
    double angular_z)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.twist.linear.x = linear_x;
    msg.twist.linear.y = linear_y;
    msg.twist.linear.z = linear_z;
    msg.twist.angular.x = angular_x;
    msg.twist.angular.y = angular_y;
    msg.twist.angular.z = angular_z;
    command_pub_->publish(msg);
  }

  void on_status(const moveit_msgs::msg::ServoStatus::SharedPtr msg) const
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Servo status: code=%d message='%s'",
      msg->code,
      msg->message.c_str());
  }

  std::string command_topic_;
  std::string status_topic_;
  std::string frame_id_;
  double publish_rate_hz_{50.0};
  double start_delay_sec_{1.0};
  double run_duration_sec_{1.5};
  int halt_publish_count_{5};
  double linear_x_{0.02};
  double linear_y_{0.0};
  double linear_z_{0.0};
  double angular_x_{0.0};
  double angular_y_{0.0};
  double angular_z_{0.0};
  int halt_messages_sent_{0};

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stop_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ur3ServoTwistCommanderNode>();
  rclcpp::spin(node);
  return 0;
}
