#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ur3_real_guarded_motion_lab_cpp/planned_home_utils.hpp"

namespace
{

constexpr char kConfirmationToken[] = "I_CONFIRM_REAL_PEN_AIR_MOTION";

std::string json_escape(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return escaped;
}

}  // namespace

class PlannedHomeMotionNode : public rclcpp::Node
{
public:
  PlannedHomeMotionNode()
  : Node("planned_home_motion")
  {
    phase_ = declare_parameter<std::string>("phase", "pre_home");
    execute_ = declare_parameter<bool>("execute", false);
    human_confirmation_ = declare_parameter<std::string>("human_confirmation", "");
    reviewed_by_ = declare_parameter<std::string>("home_reviewed_by", "");
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "home_joint_names", std::vector<std::string>{});
    target_positions_ = declare_parameter<std::vector<double>>(
      "home_positions_rad", std::vector<double>{});
    planning_group_ = declare_parameter<std::string>("planning_group", "ur_manipulator");
    planning_time_sec_ = declare_parameter<double>("planning_time_sec", 5.0);
    planning_attempts_ = declare_parameter<int>("planning_attempts", 5);
    velocity_scaling_ = declare_parameter<double>("max_velocity_scaling", 0.10);
    acceleration_scaling_ = declare_parameter<double>("max_acceleration_scaling", 0.10);
    final_tolerance_rad_ = declare_parameter<double>("final_position_tolerance_rad", 0.02);
    current_state_timeout_sec_ = declare_parameter<double>("current_state_timeout_sec", 3.0);
    result_path_ = declare_parameter<std::string>("result_path", "");

    start_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&PlannedHomeMotionNode::start_worker_once, this));
  }

  ~PlannedHomeMotionNode() override
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  int exit_code() const
  {
    return exit_code_;
  }

private:
  void start_worker_once()
  {
    if (started_) {
      return;
    }
    started_ = true;
    start_timer_->cancel();
    worker_ = std::thread([this]() {run_once();});
  }

  void run_once()
  {
    if (!validate_parameters()) {
      finish(false, false, false, INFINITY, "invalid parameters");
      return;
    }
    if (execute_ && human_confirmation_ != kConfirmationToken) {
      finish(false, false, false, INFINITY, "real motion confirmation rejected");
      return;
    }

    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
      move_group_->setPlanningTime(planning_time_sec_);
      move_group_->setNumPlanningAttempts(planning_attempts_);
      move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
      move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

      const auto current_state = move_group_->getCurrentState(current_state_timeout_sec_);
      if (!current_state) {
        finish(false, false, false, INFINITY, "current robot state unavailable");
        return;
      }

      const auto initial_positions = ordered_positions(*current_state);
      if (ur3_real_guarded_motion_lab_cpp::joint_target_reached(
          initial_positions, target_positions_, final_tolerance_rad_))
      {
        finish(true, false, false, 0.0, "home already reached");
        return;
      }

      std::map<std::string, double> joint_target;
      for (std::size_t index = 0; index < joint_names_.size(); ++index) {
        joint_target.emplace(joint_names_[index], target_positions_[index]);
      }
      move_group_->setStartState(*current_state);
      if (!move_group_->setJointValueTarget(joint_target)) {
        finish(false, false, false, INFINITY, "MoveIt rejected home joint target");
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool planned =
        move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      if (!planned) {
        finish(false, false, false, INFINITY, "MoveIt home planning failed");
        return;
      }
      if (!execute_) {
        finish(true, true, false, INFINITY, "dry-run planning succeeded");
        return;
      }

      const bool executed =
        move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      if (!executed) {
        finish(false, true, false, INFINITY, "MoveIt home execution failed");
        return;
      }

      const auto final_state = move_group_->getCurrentState(current_state_timeout_sec_);
      if (!final_state) {
        finish(false, true, true, INFINITY, "final robot state unavailable");
        return;
      }
      const double final_error =
        ur3_real_guarded_motion_lab_cpp::max_abs_joint_error(
        ordered_positions(*final_state), target_positions_);
      if (final_error > final_tolerance_rad_) {
        finish(false, true, true, final_error, "final home tolerance failed");
        return;
      }
      finish(true, true, true, final_error, "home motion completed");
    } catch (const std::exception & exception) {
      finish(false, false, false, INFINITY, exception.what());
    }
  }

  bool validate_parameters()
  {
    const bool target_valid =
      ur3_real_guarded_motion_lab_cpp::valid_home_target(joint_names_, target_positions_);
    const bool scaling_valid =
      velocity_scaling_ > 0.0 && velocity_scaling_ <= 0.10 &&
      acceleration_scaling_ > 0.0 && acceleration_scaling_ <= 0.10;
    const bool timing_valid =
      planning_time_sec_ > 0.0 && planning_attempts_ > 0 &&
      current_state_timeout_sec_ > 0.0 && final_tolerance_rad_ > 0.0;
    if (!target_valid || reviewed_by_.empty() || !scaling_valid || !timing_valid) {
      RCLCPP_ERROR(
        get_logger(),
        "Rejected planned home parameters: target_valid=%s reviewed=%s scaling_valid=%s "
        "timing_valid=%s",
        target_valid ? "true" : "false", reviewed_by_.empty() ? "false" : "true",
        scaling_valid ? "true" : "false", timing_valid ? "true" : "false");
      return false;
    }
    return true;
  }

  std::vector<double> ordered_positions(const moveit::core::RobotState & state) const
  {
    std::vector<double> positions;
    positions.reserve(joint_names_.size());
    for (const auto & joint_name : joint_names_) {
      positions.push_back(state.getVariablePosition(joint_name));
    }
    return positions;
  }

  void finish(
    const bool success,
    const bool planned,
    const bool executed,
    const double max_error,
    const std::string & reason)
  {
    exit_code_ = success ? 0 : 1;
    RCLCPP_INFO(
      get_logger(),
      "Planned home result: phase=%s success=%s planned=%s executed=%s "
      "max_joint_error_rad=%.6f reason=%s",
      phase_.c_str(), success ? "true" : "false", planned ? "true" : "false",
      executed ? "true" : "false", max_error, reason.c_str());
    if (!result_path_.empty()) {
      std::ofstream result_file(result_path_, std::ios::trunc);
      result_file << std::setprecision(9)
                  << "{\n"
                  << "  \"phase\": \"" << json_escape(phase_) << "\",\n"
                  << "  \"success\": " << (success ? "true" : "false") << ",\n"
                  << "  \"planned\": " << (planned ? "true" : "false") << ",\n"
                  << "  \"executed\": " << (executed ? "true" : "false") << ",\n"
                  << "  \"max_joint_error_rad\": ";
      if (std::isfinite(max_error)) {
        result_file << max_error;
      } else {
        result_file << "null";
      }
      result_file << ",\n"
                  << "  \"reason\": \"" << json_escape(reason) << "\"\n"
                  << "}\n";
    }
    rclcpp::shutdown();
  }

  std::string phase_;
  bool execute_{false};
  std::string human_confirmation_;
  std::string reviewed_by_;
  std::vector<std::string> joint_names_;
  std::vector<double> target_positions_;
  std::string planning_group_;
  double planning_time_sec_{5.0};
  int planning_attempts_{5};
  double velocity_scaling_{0.10};
  double acceleration_scaling_{0.10};
  double final_tolerance_rad_{0.02};
  double current_state_timeout_sec_{3.0};
  std::string result_path_;
  bool started_{false};
  int exit_code_{1};
  rclcpp::TimerBase::SharedPtr start_timer_;
  std::thread worker_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannedHomeMotionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  return node->exit_code();
}
