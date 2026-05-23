#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <vector>

namespace ego_planner
{

inline double normalizeYawError(double target_yaw, double current_yaw)
{
  constexpr double kPi = 3.14159265358979323846;
  double err = target_yaw - current_yaw;
  while (err > kPi) err -= 2.0 * kPi;
  while (err < -kPi) err += 2.0 * kPi;
  return err;
}

inline bool firstDistinctWaypointYaw2d(
    const std::vector<Eigen::Vector3d> &wps,
    const Eigen::Vector3d &current_pos,
    double min_dist,
    double *yaw)
{
  if (yaw == nullptr)
    return false;

  for (const auto &wp : wps)
  {
    const Eigen::Vector2d delta = (wp - current_pos).head<2>();
    if (delta.norm() >= min_dist)
    {
      *yaw = std::atan2(delta.y(), delta.x());
      return true;
    }
  }
  return false;
}

inline bool needsYawAlignment(double target_yaw, double current_yaw, double threshold)
{
  return std::fabs(normalizeYawError(target_yaw, current_yaw)) > threshold;
}

}  // namespace ego_planner
