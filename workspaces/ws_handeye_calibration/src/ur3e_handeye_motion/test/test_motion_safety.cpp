#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "ur3e_handeye_motion/motion_safety.hpp"

using ur3e_handeye_motion::GateLevel;

TEST(MotionSafety, ValidatesPoseAndQuaternion)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  std::string reason;
  EXPECT_TRUE(ur3e_handeye_motion::valid_pose(pose, reason));

  pose.orientation.w = 0.5;
  EXPECT_FALSE(ur3e_handeye_motion::valid_pose(pose, reason));
  pose.orientation.w = 1.0;
  pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ur3e_handeye_motion::valid_pose(pose, reason));
}

TEST(MotionSafety, ComputesShortestQuaternionDistance)
{
  geometry_msgs::msg::Quaternion identity;
  identity.w = 1.0;
  geometry_msgs::msg::Quaternion same_rotation;
  same_rotation.w = -1.0;
  EXPECT_DOUBLE_EQ(ur3e_handeye_motion::orientation_error_rad(identity, same_rotation), 0.0);

  geometry_msgs::msg::Quaternion quarter_turn;
  quarter_turn.z = std::sqrt(0.5);
  quarter_turn.w = std::sqrt(0.5);
  EXPECT_NEAR(
    ur3e_handeye_motion::orientation_error_rad(identity, quarter_turn), M_PI_2, 1e-12);
}

TEST(MotionSafety, PreservesEightCGateSemantics)
{
  using ur_dashboard_msgs::msg::RobotMode;
  using ur_dashboard_msgs::msg::SafetyMode;
  EXPECT_EQ(ur3e_handeye_motion::robot_mode_level(RobotMode::RUNNING), GateLevel::pass);
  EXPECT_EQ(ur3e_handeye_motion::robot_mode_level(RobotMode::IDLE), GateLevel::warn);
  EXPECT_EQ(ur3e_handeye_motion::robot_mode_level(RobotMode::POWER_OFF), GateLevel::block);
  EXPECT_EQ(ur3e_handeye_motion::safety_mode_level(SafetyMode::NORMAL), GateLevel::pass);
  EXPECT_EQ(ur3e_handeye_motion::safety_mode_level(SafetyMode::REDUCED), GateLevel::warn);
  EXPECT_EQ(
    ur3e_handeye_motion::safety_mode_level(SafetyMode::PROTECTIVE_STOP), GateLevel::block);
}

TEST(MotionSafety, RejectsStaleStarts)
{
  const std::vector<double> expected{0.0, 1.0, 2.0};
  EXPECT_TRUE(ur3e_handeye_motion::joints_match(expected, {0.001, 0.999, 2.005}, 0.01));
  EXPECT_FALSE(ur3e_handeye_motion::joints_match(expected, {0.0, 1.0, 2.02}, 0.01));
  EXPECT_FALSE(ur3e_handeye_motion::joints_match(expected, {0.0, 1.0}, 0.01));
}

TEST(MotionSafety, ConsumesExactlyOnePlan)
{
  ur3e_handeye_motion::PlanSlot slot;
  const auto first = slot.replace();
  const auto second = slot.replace();
  EXPECT_NE(first, second);
  EXPECT_FALSE(slot.consume(first));
  EXPECT_TRUE(slot.consume(second));
  EXPECT_FALSE(slot.consume(second));

  const auto third = slot.replace();
  slot.clear();
  EXPECT_FALSE(slot.consume(third));
}
