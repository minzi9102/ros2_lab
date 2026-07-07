#include <chrono>
#include <fstream>
#include <iostream>
#include <istream>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "tool_point_calibration_ros2/tool_point_calibration.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("console_paper_calibration");

  const auto base_frame = node->declare_parameter<std::string>("base_frame", "base");
  const auto tcp_frame = node->declare_parameter<std::string>("tcp_frame", "calibrated_tcp");
  const int num_samples = static_cast<int>(node->declare_parameter<int>("num_samples", 4));
  const auto lookup_timeout_sec = node->declare_parameter<double>("lookup_timeout_sec", 1.0);

  if (num_samples < 3) {
    RCLCPP_ERROR(node->get_logger(), "num_samples must be at least 3.");
    rclcpp::shutdown();
    return 1;
  }

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  RCLCPP_INFO(
    node->get_logger(), "Starting paper plane calibration from '%s' to '%s' with %d samples.",
    base_frame.c_str(), tcp_frame.c_str(), num_samples);
  RCLCPP_INFO(
    node->get_logger(),
    "Move the calibrated TCP to different points on the paper, then press Enter.");

  tool_point_calibration_ros2::PointVector points;
  points.reserve(static_cast<std::size_t>(num_samples));

  const auto timeout = tf2::durationFromSec(lookup_timeout_sec);
  std::ifstream tty("/dev/tty");
  std::istream & input = tty.is_open() ? tty : std::cin;
  std::string line;
  for (int count = 0; rclcpp::ok() && count < num_samples; ) {
    RCLCPP_INFO(node->get_logger(), "Point %d/%d: press Enter to capture.", count + 1, num_samples);
    std::getline(input, line);

    try {
      const auto transform = tf_buffer.lookupTransform(base_frame, tcp_frame, tf2::TimePointZero,
          timeout);
      const auto & translation = transform.transform.translation;
      const Eigen::Vector3d point(translation.x, translation.y, translation.z);
      points.push_back(point);
      RCLCPP_INFO_STREAM(node->get_logger(), "Captured point (m): [" << point.transpose() << "]");
      ++count;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(node->get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

  const auto result = tool_point_calibration_ros2::calibratePlane(points);
  if (!result.success) {
    RCLCPP_ERROR(node->get_logger(), "%s", result.message.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO_STREAM(
    node->get_logger(), "Paper center in " << base_frame << " (m): ["
                                           << result.center.transpose() << "]");
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Paper normal in " << base_frame << ": ["
                                           << result.normal.transpose() << "]");
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Plane equation: normal.dot(p - center) = 0");
  RCLCPP_INFO(node->get_logger(), "Average residual (m): %.9f", result.average_residual);
  RCLCPP_INFO(node->get_logger(), "Max residual (m): %.9f", result.max_residual);

  rclcpp::shutdown();
  return 0;
}
