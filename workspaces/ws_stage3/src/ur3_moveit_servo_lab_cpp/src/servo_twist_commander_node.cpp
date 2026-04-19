#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "moveit_msgs/msg/servo_status.hpp"
#include "moveit_msgs/srv/servo_command_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class Ur3ServoTwistCommanderNode : public rclcpp::Node
{
public:
  Ur3ServoTwistCommanderNode()
  : Node("ur3_servo_twist_commander")
  {
    // 这些参数基本对应一次最小 Servo 实验的三个问题：
    // 1) 命令往哪里发；2) 何时允许开始动；3) 以多快的速度、持续多久、如何停下。
    command_topic_ =
      this->declare_parameter<std::string>("command_topic", "/servo_node/delta_twist_cmds");
    status_topic_ = this->declare_parameter<std::string>("status_topic", "/servo_node/status");
    command_type_service_ =
      this->declare_parameter<std::string>("command_type_service", "/servo_node/switch_command_type");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "tool0");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    start_delay_sec_ = this->declare_parameter<double>("start_delay_sec", 0.0);
    run_duration_sec_ = this->declare_parameter<double>("run_duration_sec", 2.5);
    servo_ready_timeout_sec_ = this->declare_parameter<double>("servo_ready_timeout_sec", 8.0);
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
    command_type_client_ =
      this->create_client<moveit_msgs::srv::ServoCommandType>(command_type_service_);
    start_motion_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/start_motion",
      std::bind(
        &Ur3ServoTwistCommanderNode::on_start_motion_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&Ur3ServoTwistCommanderNode::on_publish_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "servo_twist_commander_node started. command_topic=%s status_topic=%s command_type_service=%s start_motion_service=%s duration=%.2fs rate=%.1fHz",
      command_topic_.c_str(),
      status_topic_.c_str(),
      command_type_service_.c_str(),
      start_motion_service_->get_service_name(),
      run_duration_sec_,
      publish_rate_hz_);

    // TODO(human): 请亲自调整速度上限、运行时长和停机策略，不要直接依赖这里的默认值。
  }

private:
  enum class CommanderState
  {
    PreparingCommandType,
    WaitingServoReady,
    IdleReady,
    Running,
    Halting,
  };

  void on_publish_timer()
  {
    const rclcpp::Time now = this->now();

    // 这个定时器不会在 ready 后自动开始运动，而是只推进状态机。
    // 真正的运动开始由 ~/start_motion 服务触发。
    switch (state_) {
      case CommanderState::PreparingCommandType:
        if (prepare_twist_mode(now)) {
          state_ = CommanderState::WaitingServoReady;
        }
        return;
      case CommanderState::WaitingServoReady:
        if (wait_for_servo_ready(now)) {
          enter_idle_ready();
        }
        return;
      case CommanderState::IdleReady:
        return;
      case CommanderState::Running:
        if (now < start_time_) {
          return;
        }

        if (now <= stop_time_) {
          publish_twist(linear_x_, linear_y_, linear_z_, angular_x_, angular_y_, angular_z_);
          return;
        }

        state_ = CommanderState::Halting;
        halt_messages_sent_ = 0;
        RCLCPP_INFO(this->get_logger(), "Motion duration elapsed. Publishing halt commands.");
        [[fallthrough]];
      case CommanderState::Halting:
        if (halt_messages_sent_ < halt_publish_count_) {
          publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
          ++halt_messages_sent_;
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Servo command sequence finished. Commander returned to idle.");
        enter_idle_ready();
        return;
    }
  }

  bool wait_for_servo_ready(const rclcpp::Time & now)
  {
    // 在真正开始发运动命令前，先用零速度 warmup，等 Servo 至少发布一次 status。
    // 这样可以避免“命令序列已经跑完，但 Servo 还没进入可工作态”的空跑。
    publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    const double wait_sec = (now - servo_ready_wait_start_time_).seconds();
    if (!status_received_) {
      if (wait_sec >= servo_ready_timeout_sec_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Timed out waiting for Servo status after %.2fs. Servo never reached a ready state.",
          servo_ready_timeout_sec_);
        publish_timer_->cancel();
        rclcpp::shutdown();
        return false;
      }

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Waiting for Servo status before accepting motion requests... elapsed=%.2fs",
        wait_sec);
      return false;
    }

    return true;
  }

  bool prepare_twist_mode(const rclcpp::Time & now)
  {
    // MoveIt Servo 可以接受不同命令类型；这里显式切到 TWIST，
    // 避免 commander 已经开始发 TwistStamped，但 Servo 仍处在别的命令模式。
    if (!command_type_request_sent_) {
      if (!command_type_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Waiting for Servo command type service: %s",
          command_type_service_.c_str());
        return false;
      }

      auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
      request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
      auto future_and_request_id = command_type_client_->async_send_request(request);
      command_type_future_ = future_and_request_id.future.share();
      command_type_request_sent_ = true;

      RCLCPP_INFO(this->get_logger(), "Requested Servo TWIST command mode.");
      return false;
    }

    if (command_type_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return false;
    }

    const auto response = command_type_future_.get();
    if (!response->success) {
      RCLCPP_ERROR(this->get_logger(), "Servo rejected TWIST command mode request.");
      publish_timer_->cancel();
      rclcpp::shutdown();
      return false;
    }

    command_type_ready_ = true;
    servo_ready_wait_start_time_ = now;

    RCLCPP_INFO(
      this->get_logger(),
      "Servo accepted TWIST mode. Waiting for Servo status before accepting motion requests.");
    return true;
  }

  void on_start_motion_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (state_ != CommanderState::IdleReady) {
      response->success = false;
      response->message = reject_start_motion_message();
      RCLCPP_WARN(
        this->get_logger(),
        "Rejected start_motion request while commander state=%s: %s",
        state_name(state_),
        response->message.c_str());
      return;
    }

    halt_messages_sent_ = 0;
    start_time_ = this->now() + rclcpp::Duration::from_seconds(start_delay_sec_);
    stop_time_ = start_time_ + rclcpp::Duration::from_seconds(run_duration_sec_);
    state_ = CommanderState::Running;

    response->success = true;
    if (start_delay_sec_ > 0.0) {
      response->message =
        "motion accepted; waiting for configured start_delay before publishing twist";
    } else {
      response->message = "motion accepted; publishing twist commands immediately";
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Accepted start_motion request. Motion will run for %.2fs after a %.2fs start delay.",
      run_duration_sec_,
      start_delay_sec_);
  }

  void enter_idle_ready()
  {
    state_ = CommanderState::IdleReady;
    halt_messages_sent_ = 0;
    start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    stop_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    RCLCPP_INFO(
      this->get_logger(),
      "Commander is idle and ready. Call %s to start one motion cycle.",
      start_motion_service_->get_service_name());
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
    // frame_id 决定这组线速度/角速度是相对哪个坐标系解释的。
    // 7E 默认用 tool0，是为了先聚焦“连续速度控制链路”本身。
    msg.header.frame_id = frame_id_;
    msg.twist.linear.x = linear_x;
    msg.twist.linear.y = linear_y;
    msg.twist.linear.z = linear_z;
    msg.twist.angular.x = angular_x;
    msg.twist.angular.y = angular_y;
    msg.twist.angular.z = angular_z;
    command_pub_->publish(msg);
  }

  void on_status(const moveit_msgs::msg::ServoStatus::SharedPtr msg)
  {
    if (!status_received_) {
      status_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Received the first Servo status message.");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Servo status: code=%d message='%s'",
      msg->code,
      msg->message.c_str());
  }

  const char * state_name(CommanderState state) const
  {
    switch (state) {
      case CommanderState::PreparingCommandType:
        return "PreparingCommandType";
      case CommanderState::WaitingServoReady:
        return "WaitingServoReady";
      case CommanderState::IdleReady:
        return "IdleReady";
      case CommanderState::Running:
        return "Running";
      case CommanderState::Halting:
        return "Halting";
    }

    return "Unknown";
  }

  std::string reject_start_motion_message() const
  {
    switch (state_) {
      case CommanderState::PreparingCommandType:
        return "not ready yet: waiting for Servo TWIST mode";
      case CommanderState::WaitingServoReady:
        return "not ready yet: waiting for Servo status";
      case CommanderState::IdleReady:
        return "ready";
      case CommanderState::Running:
        return "already running";
      case CommanderState::Halting:
        return "halting";
    }

    return "unknown state";
  }

  std::string command_topic_;
  std::string status_topic_;
  std::string command_type_service_;
  std::string frame_id_;
  // 这组参数定义“发什么命令”。
  double publish_rate_hz_{50.0};
  double start_delay_sec_{0.0};
  double run_duration_sec_{2.5};
  double servo_ready_timeout_sec_{8.0};
  int halt_publish_count_{5};
  double linear_x_{0.02};
  double linear_y_{0.0};
  double linear_z_{0.0};
  double angular_x_{0.0};
  double angular_y_{0.0};
  double angular_z_{0.0};

  // 这组状态位描述 commander 当前处在哪个阶段。
  int halt_messages_sent_{0};
  bool command_type_request_sent_{false};
  bool command_type_ready_{false};
  bool status_received_{false};
  CommanderState state_{CommanderState::PreparingCommandType};

  // 这些时间戳把“一次短时 Servo 实验”切成等待 ready、开始运动、停止运动三个窗口。
  rclcpp::Time servo_ready_wait_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stop_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr status_sub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr command_type_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_motion_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture command_type_future_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ur3ServoTwistCommanderNode>();
  rclcpp::spin(node);
  return 0;
}
