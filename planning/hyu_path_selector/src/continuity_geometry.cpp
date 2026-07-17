#include "hyu_path_selector/continuity_geometry.hpp"

#include <cstddef>
#include <cmath>
#include <limits>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace hyu_path_selector::continuity_geometry
{

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

bool isFresh(double event_time_sec, double now_sec, double timeout_sec)
{
  if (!std::isfinite(event_time_sec) || !std::isfinite(now_sec)) {
    return false;
  }
  const double age_sec = now_sec - event_time_sec;
  return age_sec >= 0.0 && age_sec <= timeout_sec;
}

bool isStampFresh(double event_time_sec, double now_sec, double timeout_sec)
{
  if (!std::isfinite(event_time_sec) || !std::isfinite(now_sec)) {
    return false;
  }
  const double age_sec = now_sec - event_time_sec;
  return age_sec >= -kMaxFutureHeaderStampSkewSec && age_sec <= timeout_sec;
}

bool finiteWaypoint(const eufs_msgs::msg::Waypoint & waypoint)
{
  return std::isfinite(waypoint.position.x) && std::isfinite(waypoint.position.y) &&
         std::isfinite(waypoint.position.z) && std::isfinite(waypoint.speed) &&
         std::isfinite(waypoint.suggested_steering) && std::isfinite(waypoint.x_m) &&
         std::isfinite(waypoint.y_m) && std::isfinite(waypoint.s_m) &&
         std::isfinite(waypoint.psi_rad) && std::isfinite(waypoint.kappa_radpm) &&
         std::isfinite(waypoint.vx_mps) && std::isfinite(waypoint.ax_mps2);
}

bool hasNonzeroSegment(const eufs_msgs::msg::WaypointArrayStamped & path)
{
  for (std::size_t index = 1U; index < path.waypoints.size(); ++index) {
    const auto & previous = path.waypoints[index - 1U];
    const auto & current = path.waypoints[index];
    if (std::hypot(current.x_m - previous.x_m, current.y_m - previous.y_m) >
      kGeometryEpsilon)
    {
      return true;
    }
  }
  return false;
}

double forwardLength(const eufs_msgs::msg::WaypointArrayStamped & path)
{
  double length_m = 0.0;
  for (std::size_t index = 1U; index < path.waypoints.size(); ++index) {
    const auto & previous = path.waypoints[index - 1U];
    const auto & current = path.waypoints[index];
    length_m += std::hypot(current.x_m - previous.x_m, current.y_m - previous.y_m);
  }
  return length_m;
}

std::optional<double> startHeading(const eufs_msgs::msg::WaypointArrayStamped & path)
{
  const auto & first = path.waypoints.front();
  for (std::size_t index = 1U; index < path.waypoints.size(); ++index) {
    const double dx = path.waypoints[index].x_m - first.x_m;
    const double dy = path.waypoints[index].y_m - first.y_m;
    if (std::hypot(dx, dy) > kGeometryEpsilon) {
      return std::atan2(dy, dx);
    }
  }
  return std::nullopt;
}

double wrappedAbsoluteDifference(double lhs, double rhs)
{
  return std::abs(std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs)));
}

TrimResult trimAtEgoNearestPoint(
  const eufs_msgs::msg::WaypointArrayStamped & path,
  const nav_msgs::msg::Odometry & odometry)
{
  if (path.waypoints.size() < 2U) {
    return {std::nullopt, 0U, ContinuityFailure::MalformedPath};
  }

  const double ego_x = odometry.pose.pose.position.x;
  const double ego_y = odometry.pose.pose.position.y;
  if (!std::isfinite(ego_x) || !std::isfinite(ego_y)) {
    return {std::nullopt, 0U, ContinuityFailure::MalformedOdometry};
  }

  std::size_t nearest_index = 0U;
  double nearest_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < path.waypoints.size(); ++index) {
    const auto & waypoint = path.waypoints[index];
    const double dx = waypoint.x_m - ego_x;
    const double dy = waypoint.y_m - ego_y;
    const double distance_squared = dx * dx + dy * dy;
    if (!std::isfinite(distance_squared)) {
      return {std::nullopt, 0U, ContinuityFailure::MalformedPath};
    }
    if (distance_squared < nearest_distance_squared) {
      nearest_distance_squared = distance_squared;
      nearest_index = index;
    }
  }

  if (path.waypoints.size() - nearest_index < 2U) {
    return {std::nullopt, nearest_index, ContinuityFailure::MalformedPath};
  }

  eufs_msgs::msg::WaypointArrayStamped trimmed;
  trimmed.header = path.header;
  trimmed.waypoints.assign(
    path.waypoints.begin() + static_cast<std::ptrdiff_t>(nearest_index),
    path.waypoints.end());
  if (!hasNonzeroSegment(trimmed)) {
    return {std::nullopt, nearest_index, ContinuityFailure::MalformedPath};
  }
  return {std::move(trimmed), nearest_index, ContinuityFailure::None};
}

nav_msgs::msg::Path toPath(const eufs_msgs::msg::WaypointArrayStamped & waypoints)
{
  nav_msgs::msg::Path path;
  path.header = waypoints.header;
  path.poses.reserve(waypoints.waypoints.size());
  for (const auto & waypoint : waypoints.waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = waypoints.header;
    pose.pose.position.x = waypoint.x_m;
    pose.pose.position.y = waypoint.y_m;
    pose.pose.orientation.z = std::sin(0.5 * waypoint.psi_rad);
    pose.pose.orientation.w = std::cos(0.5 * waypoint.psi_rad);
    path.poses.push_back(std::move(pose));
  }
  return path;
}

}
