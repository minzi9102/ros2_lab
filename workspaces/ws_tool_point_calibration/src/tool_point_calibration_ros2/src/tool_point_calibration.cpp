#include "tool_point_calibration_ros2/tool_point_calibration.hpp"

#include <algorithm>
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

PlaneCalibrationResult calibratePlane(const PointVector & points)
{
  PlaneCalibrationResult result;
  if (points.size() < 3) {
    result.message = "At least 3 points are required for paper plane calibration.";
    return result;
  }

  for (const auto & point : points) {
    result.center += point;
  }
  result.center /= static_cast<double>(points.size());

  Eigen::MatrixXd centered(points.size(), 3);
  for (std::size_t i = 0; i < points.size(); ++i) {
    centered.row(static_cast<Eigen::Index>(i)) = points[i] - result.center;
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
  result.normal = svd.matrixV().col(2).normalized();
  if (result.normal.z() < 0.0) {
    result.normal = -result.normal;
  }

  double residual_sum = 0.0;
  for (const auto & point : points) {
    const double residual = std::abs(result.normal.dot(point - result.center));
    residual_sum += residual;
    result.max_residual = std::max(result.max_residual, residual);
  }

  result.average_residual = residual_sum / static_cast<double>(points.size());
  result.success =
    result.normal.allFinite() && std::isfinite(result.average_residual) &&
    std::isfinite(result.max_residual);
  if (result.success) {
    result.message = "Paper plane calibration solved.";
  } else {
    result.message = "Paper plane calibration failed: non-finite result.";
  }
  return result;
}

}  // namespace tool_point_calibration_ros2
