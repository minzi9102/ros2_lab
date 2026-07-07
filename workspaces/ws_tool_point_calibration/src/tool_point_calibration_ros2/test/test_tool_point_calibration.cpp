#include <gtest/gtest.h>

#include <cmath>

#include "tool_point_calibration_ros2/tool_point_calibration.hpp"

namespace
{

Eigen::Matrix3d rotationFromRpy(double roll, double pitch, double yaw)
{
  return (
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
         .toRotationMatrix();
}

}  // namespace

TEST(ToolPointCalibration, RecoversKnownTcpAndTouchPoint)
{
  const Eigen::Vector3d expected_tcp(0.02, -0.015, 0.18);
  const Eigen::Vector3d expected_touch_point(0.45, -0.12, 0.31);

  tool_point_calibration_ros2::PoseVector poses;
  const Eigen::Matrix3d rotations[] = {
    rotationFromRpy(0.0, 0.0, 0.0),
    rotationFromRpy(0.35, -0.2, 0.1),
    rotationFromRpy(-0.4, 0.3, -0.25),
    rotationFromRpy(0.2, 0.45, 0.5),
  };

  for (const auto & rotation : rotations) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = rotation;
    pose.translation() = expected_touch_point - rotation * expected_tcp;
    poses.push_back(pose);
  }

  const auto result = tool_point_calibration_ros2::calibrateTcp(poses);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT((result.tcp_offset - expected_tcp).norm(), 1e-9);
  EXPECT_LT((result.touch_point - expected_touch_point).norm(), 1e-9);
  EXPECT_LT(result.average_residual, 1e-9);
}

TEST(ToolPointCalibration, RejectsTooFewSamples)
{
  const auto result = tool_point_calibration_ros2::calibrateTcp({});

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("At least 3"), std::string::npos);
}

TEST(ToolPointCalibration, RecoversKnownPaperPlane)
{
  const Eigen::Vector3d center(0.45, -0.02, 0.12);
  Eigen::Vector3d normal(0.1, -0.2, 1.0);
  normal.normalize();
  const Eigen::Vector3d x_axis = normal.cross(Eigen::Vector3d::UnitY()).normalized();
  const Eigen::Vector3d y_axis = normal.cross(x_axis).normalized();

  tool_point_calibration_ros2::PointVector points;
  points.push_back(center + 0.10 * x_axis + 0.05 * y_axis);
  points.push_back(center - 0.10 * x_axis + 0.05 * y_axis);
  points.push_back(center + 0.10 * x_axis - 0.05 * y_axis);
  points.push_back(center - 0.10 * x_axis - 0.05 * y_axis);

  const auto result = tool_point_calibration_ros2::calibratePlane(points);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT((result.center - center).norm(), 1e-9);
  EXPECT_LT((result.normal - normal).norm(), 1e-9);
  EXPECT_LT(result.average_residual, 1e-9);
  EXPECT_LT(result.max_residual, 1e-9);
}

TEST(ToolPointCalibration, RejectsTooFewPaperPoints)
{
  const auto result = tool_point_calibration_ros2::calibratePlane({});

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("At least 3"), std::string::npos);
}

TEST(ToolPointCalibration, ReportsPaperPlaneResiduals)
{
  tool_point_calibration_ros2::PointVector points;
  points.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  points.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  points.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
  points.push_back(Eigen::Vector3d(1.0, 1.0, 0.004));

  const auto result = tool_point_calibration_ros2::calibratePlane(points);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_TRUE(std::isfinite(result.average_residual));
  EXPECT_TRUE(std::isfinite(result.max_residual));
  EXPECT_GT(result.average_residual, 0.0);
  EXPECT_GT(result.max_residual, 0.0);
}
