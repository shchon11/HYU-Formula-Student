#pragma once

#include <optional>

#include "builtin_interfaces/msg/time.hpp"
#include "hyu_msgs/msg/waypoint.hpp"
#include "hyu_path_selector/continuity_check.hpp"

namespace hyu_path_selector::continuity_geometry
{

constexpr double kGeometryEpsilon = 1.0e-9;
constexpr double kMaxFutureHeaderStampSkewSec = 0.25;

double stampSeconds(const builtin_interfaces::msg::Time & stamp);
bool isFresh(double event_time_sec, double now_sec, double timeout_sec);
bool isStampFresh(double event_time_sec, double now_sec, double timeout_sec);
bool finiteWaypoint(const hyu_msgs::msg::Waypoint & waypoint);
bool hasNonzeroSegment(const hyu_msgs::msg::WaypointArrayStamped & path);
double forwardLength(const hyu_msgs::msg::WaypointArrayStamped & path);
std::optional<double> startHeading(const hyu_msgs::msg::WaypointArrayStamped & path);
double wrappedAbsoluteDifference(double lhs, double rhs);
TrimResult trimAtEgoNearestPoint(
  const hyu_msgs::msg::WaypointArrayStamped & path,
  const nav_msgs::msg::Odometry & odometry);
nav_msgs::msg::Path toPath(const hyu_msgs::msg::WaypointArrayStamped & waypoints);

}
