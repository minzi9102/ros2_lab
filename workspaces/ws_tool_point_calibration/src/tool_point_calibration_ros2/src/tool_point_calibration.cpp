#include "tool_point_calibration_ros2/tool_point_calibration.hpp"

#include <cmath>
#include <sstream>

#include <Eigen/Dense>

namespace tool_point_calibration_ros2
{

TcpCalibrationResult calibrateTcp(const PoseVector & tool_poses)
{
  TcpCalibrationResult result;
  if (tool_poses.size() < 3) {
    result.message = "At least 3 tool poses are required for TCP calibration.";
    return result;
  }

  Eigen::MatrixXd a(3 * tool_poses.size(), 6);
  Eigen::VectorXd b(3 * tool_poses.size());

  for (std::size_t i = 0; i < tool_poses.size(); ++i) {
    const auto row = static_cast<Eigen::Index>(3 * i);
    a.block<3, 3>(row, 0) = tool_poses[i].rotation();
    a.block<3, 3>(row, 3) = -Eigen::Matrix3d::Identity();
    b.segment<3>(row) = -tool_poses[i].translation();
  }

  const Eigen::VectorXd x = a.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  result.tcp_offset = x.segment<3>(0);
  result.touch_point = x.segment<3>(3);

  double residual_sum = 0.0;
  for (const auto & pose : tool_poses) {
    residual_sum += (pose * result.tcp_offset - result.touch_point).norm();
  }

  result.average_residual = residual_sum / static_cast<double>(tool_poses.size());
  result.success = std::isfinite(result.average_residual);
  if (result.success) {
    result.message = "TCP calibration solved.";
  } else {
    result.message = "TCP calibration failed: non-finite residual.";
  }
  return result;
}

}  // namespace tool_point_calibration_ros2
