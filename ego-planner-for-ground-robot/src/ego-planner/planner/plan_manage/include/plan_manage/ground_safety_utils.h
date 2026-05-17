#pragma once

#include <cmath>
#include <functional>
#include <utility>
#include <vector>

#include <Eigen/Eigen>

namespace ego_planner
{

using Occupancy2dFn = std::function<int(const Eigen::Vector2d &)>;
using TimedPoint3d = std::pair<double, Eigen::Vector3d>;

inline bool hasFreeClearance2d(const Eigen::Vector2d &point,
                               const Occupancy2dFn &occupancy,
                               const double clearance_radius,
                               const double clearance_step)
{
  if (!occupancy || occupancy(point) != 0)
    return false;

  if (clearance_radius <= 1e-9)
    return true;

  const double step = clearance_step > 1e-9 ? clearance_step : clearance_radius;
  constexpr double kTwoPi = 2.0 * M_PI;
  constexpr double kAngleStep = M_PI / 8.0;
  for (double radius = step; radius <= clearance_radius + 1e-9; radius += step)
  {
    for (double angle = 0.0; angle < kTwoPi - 1e-9; angle += kAngleStep)
    {
      Eigen::Vector2d sample(point.x() + radius * std::cos(angle),
                             point.y() + radius * std::sin(angle));
      if (occupancy(sample) != 0)
        return false;
    }
  }

  return true;
}

inline bool projectToNearestFree2d(const Eigen::Vector3d &requested,
                                   const Occupancy2dFn &occupancy,
                                   Eigen::Vector3d *projected,
                                   const double max_search_radius,
                                   const double step,
                                   const double clearance_radius = 0.0,
                                   const double clearance_step = 0.0)
{
  if (!projected || !occupancy || max_search_radius < 0.0 || step <= 0.0)
    return false;

  const Eigen::Vector2d requested_xy(requested.x(), requested.y());
  if (hasFreeClearance2d(requested_xy, occupancy, clearance_radius, clearance_step))
  {
    *projected = requested;
    return true;
  }

  constexpr double kTwoPi = 2.0 * M_PI;
  constexpr double kAngleStep = M_PI / 8.0;
  for (double radius = step; radius <= max_search_radius + 1e-9; radius += step)
  {
    for (double angle = 0.0; angle < kTwoPi - 1e-9; angle += kAngleStep)
    {
      Eigen::Vector3d candidate(requested.x() + radius * std::cos(angle),
                                requested.y() + radius * std::sin(angle),
                                requested.z());
      const Eigen::Vector2d candidate_xy(candidate.x(), candidate.y());
      if (hasFreeClearance2d(candidate_xy, occupancy, clearance_radius, clearance_step))
      {
        *projected = candidate;
        return true;
      }
    }
  }

  return false;
}

inline bool isCollisionFreeWindow2d(const std::vector<TimedPoint3d> &points,
                                    const Occupancy2dFn &occupancy,
                                    const double t_start,
                                    const double t_end)
{
  if (!occupancy || t_end <= t_start)
    return false;

  bool has_window_sample = false;
  for (const auto &timed_point : points)
  {
    const double t = timed_point.first;
    if (t <= t_start || t > t_end)
      continue;

    has_window_sample = true;
    const Eigen::Vector3d &point = timed_point.second;
    const Eigen::Vector2d point_xy(point.x(), point.y());
    if (occupancy(point_xy) != 0)
      return false;
  }

  return has_window_sample;
}

inline bool isSafeToContinueDuringReplan2d(const std::vector<TimedPoint3d> &points,
                                           const Occupancy2dFn &occupancy,
                                           const double t_cur,
                                           const double braking_window)
{
  if (braking_window <= 1e-6)
    return false;

  return isCollisionFreeWindow2d(points, occupancy, t_cur, t_cur + braking_window);
}

inline bool computeControlledStopTarget2d(const Eigen::Vector3d &position,
                                          const Eigen::Vector3d &velocity,
                                          const double max_deceleration,
                                          Eigen::Vector3d *target)
{
  if (!target || max_deceleration <= 1e-6 || !position.allFinite() || !velocity.allFinite())
    return false;

  const Eigen::Vector2d velocity_xy(velocity.x(), velocity.y());
  const double speed = velocity_xy.norm();
  *target = position;
  if (speed <= 1e-6)
    return true;

  const double stop_distance = speed * speed / (2.0 * max_deceleration);
  const Eigen::Vector2d stop_xy = position.head<2>() + velocity_xy.normalized() * stop_distance;
  target->x() = stop_xy.x();
  target->y() = stop_xy.y();
  target->z() = position.z();
  return true;
}

inline bool isControlledStopPathFree2d(const Eigen::Vector3d &position,
                                       const Eigen::Vector3d &velocity,
                                       const double max_deceleration,
                                       const Occupancy2dFn &occupancy,
                                       const double sample_step)
{
  if (!occupancy || max_deceleration <= 1e-6 || sample_step <= 1e-6)
    return false;

  Eigen::Vector3d stop_target;
  if (!computeControlledStopTarget2d(position, velocity, max_deceleration, &stop_target))
    return false;

  const Eigen::Vector2d start_xy(position.x(), position.y());
  const Eigen::Vector2d end_xy(stop_target.x(), stop_target.y());
  const Eigen::Vector2d delta = end_xy - start_xy;
  const double distance = delta.norm();
  const int samples = std::max(1, static_cast<int>(std::ceil(distance / sample_step)));
  for (int i = 0; i <= samples; ++i)
  {
    const double ratio = static_cast<double>(i) / static_cast<double>(samples);
    const Eigen::Vector2d point = start_xy + ratio * delta;
    if (occupancy(point) != 0)
      return false;
  }

  return true;
}

}  // namespace ego_planner
