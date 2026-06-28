#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

namespace ur3_real_guarded_motion_lab_cpp
{

inline bool valid_home_target(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions)
{
  if (joint_names.size() != 6 || positions.size() != joint_names.size()) {
    return false;
  }
  const std::unordered_set<std::string> unique_names(joint_names.begin(), joint_names.end());
  if (unique_names.size() != joint_names.size()) {
    return false;
  }
  return std::all_of(positions.begin(), positions.end(), [](const double value) {
             return std::isfinite(value);
           });
}

inline double max_abs_joint_error(
  const std::vector<double> & actual,
  const std::vector<double> & target)
{
  if (actual.size() != target.size() || actual.empty()) {
    return INFINITY;
  }
  double max_error = 0.0;
  for (std::size_t index = 0; index < actual.size(); ++index) {
    max_error = std::max(max_error, std::abs(actual[index] - target[index]));
  }
  return max_error;
}

inline bool joint_target_reached(
  const std::vector<double> & actual,
  const std::vector<double> & target,
  const double tolerance_rad)
{
  return tolerance_rad > 0.0 &&
         max_abs_joint_error(actual, target) <= tolerance_rad;
}

}  // namespace ur3_real_guarded_motion_lab_cpp
