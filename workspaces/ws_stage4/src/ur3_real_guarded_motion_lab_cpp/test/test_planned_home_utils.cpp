#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ur3_real_guarded_motion_lab_cpp/planned_home_utils.hpp"

TEST(PlannedHomeUtils, ValidatesReviewedSixJointShape)
{
  const std::vector<std::string> names{"a", "b", "c", "d", "e", "f"};
  const std::vector<double> positions{0.0, 1.0, 2.0, 3.0, 4.0, 5.0};

  EXPECT_TRUE(ur3_real_guarded_motion_lab_cpp::valid_home_target(names, positions));
  EXPECT_FALSE(
    ur3_real_guarded_motion_lab_cpp::valid_home_target(
      {"a", "b", "c", "d", "e", "e"}, positions));
  EXPECT_FALSE(
    ur3_real_guarded_motion_lab_cpp::valid_home_target(names, {0.0, 1.0}));
}

TEST(PlannedHomeUtils, ComputesMaximumJointError)
{
  EXPECT_NEAR(
    ur3_real_guarded_motion_lab_cpp::max_abs_joint_error(
      {0.0, 1.0, 2.0}, {0.01, 0.98, 2.005}),
    0.02,
    1e-12);
  EXPECT_TRUE(
    ur3_real_guarded_motion_lab_cpp::joint_target_reached(
      {0.0, 1.0}, {0.01, 0.99}, 0.02));
  EXPECT_FALSE(
    ur3_real_guarded_motion_lab_cpp::joint_target_reached(
      {0.0, 1.0}, {0.03, 0.99}, 0.02));
}
