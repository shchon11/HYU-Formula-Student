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

struct MonotonicProjection
{
  PlannerPoint point;
  double arc_s{0.0};
};

MonotonicProjection closestPointOnPolylineAfter(
  const PlannerPoint & query,
  const std::vector<PlannerPoint> & poly,
  const std::vector<double> & arc_lengths,
  double min_arc_s,
  double duplicate_tolerance,
  double forward_window)
{
  PlannerPoint best = poly.front();
  double best_arc_s = arc_lengths.empty() ? 0.0 : arc_lengths.front();
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1U < poly.size(); ++i) {
    const PlannerPoint & a = poly[i];
    const PlannerPoint & b = poly[i + 1U];
    const double segment_start_s = arc_lengths[i];
    const double segment_end_s = arc_lengths[i + 1U];
    if (segment_end_s + duplicate_tolerance < min_arc_s) {
      continue;  // wholly behind the monotonic frontier
    }
    // Bound the search to a forward window. Without this, on a CLOSED loop the
    // seam makes ordered_yellow.back() physically adjacent to ordered_yellow
    // .front() near the ego, so the first blue sample projects onto the far
    // end of the arc and pins every later pair to yellow.back() -> the width
    // blows past max_track_width and a geometrically valid loop is rejected.
    if (segment_start_s > min_arc_s + forward_window) {
      continue;
    }
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;
    double t = 0.0;
    if (len2 > 0.0) {
      t = std::clamp(((query.x - a.x) * vx + (query.y - a.y) * vy) / len2, 0.0, 1.0);
      if (segment_end_s > segment_start_s && min_arc_s > segment_start_s) {
        // min_t may land just past a segment end (within duplicate_tolerance);
        // std::min keeps std::clamp's lo <= hi (lo > hi is undefined behaviour).
        const double min_t = (min_arc_s - segment_start_s) / (segment_end_s - segment_start_s);
        t = std::clamp(t, std::min(min_t, 1.0), 1.0);
      }
    }
    const PlannerPoint proj{a.x + t * vx, a.y + t * vy};
    const double dx = query.x - proj.x;
    const double dy = query.y - proj.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = proj;
      best_arc_s = segment_start_s + t * (segment_end_s - segment_start_s);
    }
  }
  return MonotonicProjection{best, best_arc_s};
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

bool hasSameColorDuplicate(
  const std::vector<PlannerPoint> & points, const char * side, double threshold, std::string & reason)
{
  (void)side;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1U; j < points.size(); ++j) {
      const double separation = distance(points[i], points[j]);
      if (separation < threshold) {
        reason = "duplicate_ghost";
        return true;
      }
    }
  }
  return false;
}

bool hasInvalidNearestTrackWidth(
  const std::vector<PlannerPoint> & blue_points,
  const std::vector<PlannerPoint> & yellow_points,
  const SlamCenterlineConfig & config,
  std::string & reason)
{
  for (const auto * points : {&blue_points, &yellow_points}) {
    const auto & opposite = points == &blue_points ? yellow_points : blue_points;
    for (const auto & point : *points) {
      double best_width = std::numeric_limits<double>::max();
      for (const auto & candidate : opposite) {
        best_width = std::min(best_width, distance(point, candidate));
      }
      if (best_width < config.min_track_width_m || best_width > config.max_track_width_m) {
        reason = "invalid_width";
        return true;
      }
    }
  }
  return false;
}

}  // namespace

bool buildCenterlineFromSlamMap(
  const eufs_msgs::msg::ConeArrayWithCovariance & cone_map,
  const nav_msgs::msg::Odometry & ego_odom,
  const SlamCenterlineConfig & config,
  std::vector<PlannerWaypoint> & waypoints,
  std::string & reason)
{
  waypoints.clear();

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

  const double duplicate_ghost_threshold = std::max(
    config.duplicate_point_tolerance, config.min_track_width_m * 0.25);
  if (hasSameColorDuplicate(blue_points, "blue", duplicate_ghost_threshold, reason) ||
    hasSameColorDuplicate(yellow_points, "yellow", duplicate_ghost_threshold, reason))
  {
    return false;
  }
  if (hasInvalidNearestTrackWidth(blue_points, yellow_points, config, reason)) {
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

  // Pairing is locked to the opposite boundary's forward arc coordinate. This
  // prevents close branches from snapping backward across the track while still
  // allowing unequal boundary lengths.
  const std::size_t pair_count = std::max<std::size_t>(
    2U, static_cast<std::size_t>(std::ceil(blue_arc.back() / config.waypoint_spacing_m)) + 1U);
  // Forward search window for the monotonic pairing: large enough to absorb
  // the per-sample yellow advance and any blue/yellow phase offset, small
  // enough that a blue sample can never reach across the closed-loop seam.
  const double forward_window = std::max(5.0, 2.0 * config.max_track_width_m);
  double previous_yellow_s = 0.0;
  std::vector<PlannerPoint> centerline;
  centerline.reserve(pair_count + 1U);
  std::size_t out_of_range_pairs = 0U;
  for (std::size_t i = 0; i < pair_count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(pair_count - 1U);
    const auto blue = interpolateAtArcLength(ordered_blue, blue_arc, ratio * blue_arc.back());
    const auto yellow_projection = closestPointOnPolylineAfter(
      blue, ordered_yellow, yellow_arc, previous_yellow_s, config.duplicate_point_tolerance,
      forward_window);
    previous_yellow_s = yellow_projection.arc_s;
    const auto yellow = yellow_projection.point;
    const double width = distance(blue, yellow);
    if (width < config.min_track_width_m || width > config.max_track_width_m) {
      // A single mispaired/noisy sample (SLAM cone jitter, a local pairing skew)
      // must not discard the entire raceline. Skip this midpoint and let the
      // neighbours bridge it; only a WIDESPREAD width failure (a genuinely
      // wrong map, e.g. crossed boundaries) rejects the path below.
      ++out_of_range_pairs;
      continue;
    }
    const PlannerPoint midpoint = scale(add(blue, yellow), 0.5);
    if (centerline.empty() ||
      distance(centerline.back(), midpoint) > config.duplicate_point_tolerance)
    {
      centerline.push_back(midpoint);
    }
  }

  // Tolerate isolated bad pairs, but a map where a large fraction of samples
  // fail the width gate is genuinely wrong (crossed/duplicated boundaries) and
  // must still fail closed.
  if (out_of_range_pairs > pair_count / 4U) {
    reason = "invalid_width";
    return false;
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
