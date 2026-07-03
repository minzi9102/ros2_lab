#ifndef TOOL_POINT_CALIBRATION_ROS2__TOOL_POINT_CALIBRATION_HPP_
#define TOOL_POINT_CALIBRATION_ROS2__TOOL_POINT_CALIBRATION_HPP_

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace tool_point_calibration_ros2
{

using PoseVector = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;

struct TcpCalibrationResult
{
  Eigen::Vector3d tcp_offset{Eigen::Vector3d::Zero()};
  Eigen::Vector3d touch_point{Eigen::Vector3d::Zero()};
  double average_residual{0.0};
  bool success{false};
  std::string message;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TcpCalibrationResult calibrateTcp(const PoseVector & tool_poses);

}  // namespace tool_point_calibration_ros2

#endif  // TOOL_POINT_CALIBRATION_ROS2__TOOL_POINT_CALIBRATION_HPP_
