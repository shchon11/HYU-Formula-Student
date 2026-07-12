#include "global_planner/slam_centerline_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "global_planner/slam_boundary_ordering.hpp"

namespace global_planner
{
namespace
{

// Closest point to `query` on the polyline `poly` (projected onto each segment,
// not just the nearest vertex). Used to pair a boundary sample with the true
// opposite-boundary point instead of assuming both boundaries share an
// arc-length parameterisation — the latter mismatches whenever the two sides
// have unequal length or offset start points (i.e. any full-loop map).
PlannerPoint closestPointOnPolyline(
  const PlannerPoint & query, const std::vector<PlannerPoint> & poly)
{
  PlannerPoint best = poly.front();
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1U < poly.size(); ++i) {
    const PlannerPoint & a = poly[i];
    const PlannerPoint & b = poly[i + 1U];
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;
    double t = 0.0;
    if (len2 > 0.0) {
      t = std::clamp(((query.x - a.x) * vx + (query.y - a.y) * vy) / len2, 0.0, 1.0);
    }
    const PlannerPoint proj{a.x + t * vx, a.y + t * vy};
    const double dx = query.x - proj.x;
    const double dy = query.y - proj.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = proj;
    }
  }
  return best;
}

std::vector<PlannerPoint> finiteConePoints(
  const std::vector<eufs_msgs::msg::ConeWithCovariance> & cones)
{
  std::vector<PlannerPoint> points;
  points.reserve(cones.size());
  for (const auto & cone : cones) {
    const PlannerPoint point{cone.point.x, cone.point.y};
    if (isFinitePoint(point)) {
      points.push_back(point);
    }
  }
  return points;
}

bool validateWaypoints(
  const std::vector<PlannerWaypoint> & waypoints, double duplicate_tolerance, std::string & reason)
{
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const auto & waypoint = waypoints[i];
    if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) ||
      !std::isfinite(waypoint.s) || !std::isfinite(waypoint.psi) ||
      !std::isfinite(waypoint.kappa))
    {
      reason = "generated waypoint contains non-finite fields";
      return false;
    }
    if (i > 0U) {
      const PlannerPoint previous{waypoints[i - 1].x, waypoints[i - 1].y};
      const PlannerPoint current{waypoint.x, waypoint.y};
      if (!(waypoint.s > waypoints[i - 1].s)) {
        reason = "generated s_m is not strictly increasing";
        return false;
      }
      if (distance(previous, current) <= duplicate_tolerance) {
        reason = "generated adjacent duplicate waypoint";
        return false;
      }
    }
  }
  return true;
}

}  // namespace

bool buildCenterlineFromSlamMap(
  const eufs_msgs::msg::ConeArrayWithCovariance & cone_map,
  const nav_msgs::msg::Odometry & ego_odom,
  const SlamCenterlineConfig & config,
  std::vector<PlannerWaypoint> & waypoints,
  std::string & reason)
{
  if (!std::isfinite(config.waypoint_spacing_m) ||
    config.waypoint_spacing_m < kMinimumWaypointSpacingM)
  {
    reason = "waypoint spacing is below the supported minimum";
    return false;
  }
  if (!std::isfinite(config.duplicate_point_tolerance) ||
    config.duplicate_point_tolerance < 0.0)
  {
    reason = "duplicate point tolerance is invalid";
    return false;
  }

  const auto blue_points = finiteConePoints(cone_map.blue_cones);
  const auto yellow_points = finiteConePoints(cone_map.yellow_cones);
  if (blue_points.size() < static_cast<std::size_t>(config.min_cones_per_side) ||
    yellow_points.size() < static_cast<std::size_t>(config.min_cones_per_side))
  {
    reason = "insufficient blue/yellow cones";
    return false;
  }

  const PlannerPoint ego{ego_odom.pose.pose.position.x, ego_odom.pose.pose.position.y};
  if (!isFinitePoint(ego)) {
    reason = "ego odom position is non-finite";
    return false;
  }

  std::vector<PlannerPoint> ordered_blue;
  std::vector<PlannerPoint> ordered_yellow;
  if (!orderSlamBoundaries(
      blue_points, yellow_points, ego, config.max_boundary_gap_m,
      ordered_blue, ordered_yellow, reason))
  {
    return false;
  }

  const auto blue_arc = cumulativeArcLengths(ordered_blue);
  const auto yellow_arc = cumulativeArcLengths(ordered_yellow);
  if (blue_arc.back() <= config.duplicate_point_tolerance ||
    yellow_arc.back() <= config.duplicate_point_tolerance)
  {
    reason = "boundary arc length too short";
    return false;
  }

  // Sample evenly along the blue boundary and pair each sample with the closest
  // point on the yellow boundary. Nearest-point pairing (not shared arc-length
  // ratio) keeps the pairing local, so it stays correct on full-loop maps where
  // the two sides differ in length — e.g. loading a saved map in localization.
  const std::size_t pair_count = std::max<std::size_t>(
    2U, static_cast<std::size_t>(std::ceil(blue_arc.back() / config.waypoint_spacing_m)) + 1U);
  (void)yellow_arc;
  std::vector<PlannerPoint> centerline;
  centerline.reserve(pair_count + 1U);
  for (std::size_t i = 0; i < pair_count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(pair_count - 1U);
    const auto blue = interpolateAtArcLength(ordered_blue, blue_arc, ratio * blue_arc.back());
    const auto yellow = closestPointOnPolyline(blue, ordered_yellow);
    const double width = distance(blue, yellow);
    if (width < config.min_track_width_m || width > config.max_track_width_m) {
      std::ostringstream stream;
      stream << "track width " << width << " m outside ["
             << config.min_track_width_m << ", " << config.max_track_width_m << "]";
      reason = stream.str();
      return false;
    }
    const PlannerPoint midpoint = scale(add(blue, yellow), 0.5);
    if (centerline.empty() ||
      distance(centerline.back(), midpoint) > config.duplicate_point_tolerance)
    {
      centerline.push_back(midpoint);
    }
  }

  if (centerline.size() < 2U) {
    reason = "centerline has fewer than two unique points";
    return false;
  }
  if (distance(centerline.front(), centerline.back()) <= config.close_loop_distance_m &&
    distance(centerline.front(), centerline.back()) > config.duplicate_point_tolerance)
  {
    centerline.push_back(centerline.front());
  }

  const auto resampled = resampleBySpacing(
    centerline, config.waypoint_spacing_m, config.duplicate_point_tolerance);
  if (resampled.size() < 3U) {
    reason = "resampled centerline has fewer than three points";
    return false;
  }

  waypoints.clear();
  waypoints.reserve(resampled.size());
  for (const auto & point : resampled) {
    waypoints.push_back(PlannerWaypoint{point.x, point.y, 0.0, 0.0, 0.0});
  }
  computeWaypointGeometry(waypoints);
  return validateWaypoints(waypoints, config.duplicate_point_tolerance, reason);
}

}  // namespace global_planner
