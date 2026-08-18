#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <ur_dashboard_msgs/msg/robot_mode.hpp>
#include <ur_dashboard_msgs/msg/safety_mode.hpp>

namespace ur3e_handeye_motion
{

enum class GateLevel { pass, warn, block };

inline GateLevel robot_mode_level(const int8_t mode)
{
  using ur_dashboard_msgs::msg::RobotMode;
  if (mode == RobotMode::RUNNING) {
    return GateLevel::pass;
  }
  if (mode == RobotMode::POWER_ON || mode == RobotMode::IDLE) {
    return GateLevel::warn;
  }
  return GateLevel::block;
}

inline GateLevel safety_mode_level(const uint8_t mode)
{
  using ur_dashboard_msgs::msg::SafetyMode;
  if (mode == SafetyMode::NORMAL) {
    return GateLevel::pass;
  }
  if (mode == SafetyMode::REDUCED) {
    return GateLevel::warn;
  }
  return GateLevel::block;
}

inline bool valid_pose(const geometry_msgs::msg::Pose & pose, std::string & reason)
{
  const double values[] = {
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
  if (!std::all_of(std::begin(values), std::end(values), [](double value) {
      return std::isfinite(value);
    }))
  {
    reason = "target pose contains a non-finite value";
    return false;
  }

  const auto & q = pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (std::abs(norm - 1.0) > 1e-3) {
    reason = "target quaternion norm must be within 1e-3 of one";
    return false;
  }
  return true;
}

inline double orientation_error_rad(
  const geometry_msgs::msg::Quaternion & lhs,
  const geometry_msgs::msg::Quaternion & rhs)
{
  const double lhs_norm = std::sqrt(
    lhs.x * lhs.x + lhs.y * lhs.y + lhs.z * lhs.z + lhs.w * lhs.w);
  const double rhs_norm = std::sqrt(
    rhs.x * rhs.x + rhs.y * rhs.y + rhs.z * rhs.z + rhs.w * rhs.w);
  if (lhs_norm == 0.0 || rhs_norm == 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double dot = std::abs(
    (lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w) /
    (lhs_norm * rhs_norm));
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

inline bool joints_match(
  const std::vector<double> & expected,
  const std::vector<double> & actual,
  const double tolerance)
{
  if (expected.size() != actual.size()) {
    return false;
  }
  for (std::size_t index = 0; index < expected.size(); ++index) {
    if (!std::isfinite(actual[index]) || std::abs(expected[index] - actual[index]) > tolerance) {
      return false;
    }
  }
  return true;
}

class PlanSlot
{
public:
  uint64_t replace()
  {
    active_id_ = next_id_++;
    return active_id_;
  }

  bool consume(const uint64_t id)
  {
    if (id == 0 || id != active_id_) {
      return false;
    }
    active_id_ = 0;
    return true;
  }

  void clear() {active_id_ = 0;}

  uint64_t active_id() const {return active_id_;}

private:
  uint64_t next_id_{1};
  uint64_t active_id_{0};
};

}  // namespace ur3e_handeye_motion
