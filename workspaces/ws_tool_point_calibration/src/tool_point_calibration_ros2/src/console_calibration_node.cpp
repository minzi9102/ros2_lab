#include <chrono>
#include <fstream>
#include <iostream>
#include <istream>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "tool_point_calibration_ros2/tool_point_calibration.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("console_tool_calibration");

  const auto base_frame = node->declare_parameter<std::string>("base_frame", "base");
  const auto tool0_frame = node->declare_parameter<std::string>("tool0_frame", "tool0");
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
    node->get_logger(), "Starting TCP calibration from '%s' to '%s' with %d samples.",
    base_frame.c_str(), tool0_frame.c_str(), num_samples);
  RCLCPP_INFO(
    node->get_logger(),
    "Move the tool tip to the same fixed point from different orientations, then press Enter.");

  tool_point_calibration_ros2::PoseVector observations;
  observations.reserve(static_cast<std::size_t>(num_samples));

  const auto timeout = tf2::durationFromSec(lookup_timeout_sec);
  std::ifstream tty("/dev/tty");
  std::istream & input = tty.is_open() ? tty : std::cin;
  std::string line;
  for (int count = 0; rclcpp::ok() && count < num_samples; ) {
    RCLCPP_INFO(node->get_logger(), "Pose %d/%d: press Enter to capture.", count + 1, num_samples);
    std::getline(input, line);

    try {
      const auto transform = tf_buffer.lookupTransform(base_frame, tool0_frame, tf2::TimePointZero,
          timeout);
      const Eigen::Isometry3d pose = tf2::transformToEigen(transform.transform);
      observations.push_back(pose);
      RCLCPP_INFO_STREAM(node->get_logger(), "Captured pose:\n" << pose.matrix());
      ++count;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(node->get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

  const auto result = tool_point_calibration_ros2::calibrateTcp(observations);
  if (!result.success) {
    RCLCPP_ERROR(node->get_logger(), "%s", result.message.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO_STREAM(
    node->get_logger(), "Calibrated TCP offset in " << tool0_frame << " (m): ["
                                                    << result.tcp_offset.transpose() << "]");
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Touch point in " << base_frame << " (m): ["
                                          << result.touch_point.transpose() << "]");
  RCLCPP_INFO(node->get_logger(), "Average residual (m): %.9f", result.average_residual);

  rclcpp::shutdown();
  return 0;
}
