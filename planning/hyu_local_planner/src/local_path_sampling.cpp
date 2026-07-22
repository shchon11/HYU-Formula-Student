#include "local_path_builder_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace hyu_local_planner::internal
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

std::vector<double> cumulativeArc(const std::vector<Point2> & points)
{
  std::vector<double> arc(points.size(), 0.0);
  for (std::size_t index = 1; index < points.size(); ++index) {
    const double segment = distance(points[index - 1U], points[index]);
    if (!std::isfinite(segment) || segment <= kEpsilon) {
      return {};
    }
    arc[index] = arc[index - 1U] + segment;
  }
  return arc;
}

Point2 interpolate(
  const std::vector<Point2> & points, const std::vector<double> & arc, double target)
{
  if (target <= 0.0) {
    return points.front();
  }
  if (target >= arc.back()) {
    return points.back();
  }
  const auto upper = std::upper_bound(arc.begin(), arc.end(), target);
  const std::size_t index = static_cast<std::size_t>(upper - arc.begin());
  const double ratio = (target - arc[index - 1U]) / (arc[index] - arc[index - 1U]);
  return {
    points[index - 1U].x + ratio * (points[index].x - points[index - 1U].x),
    points[index - 1U].y + ratio * (points[index].y - points[index - 1U].y),
  };
}

std::vector<Point2> resample(
  const std::vector<Point2> & points, double spacing, double horizon, double duplicate_tolerance)
{
  const auto arc = cumulativeArc(points);
  if (arc.size() != points.size() || arc.empty() || !std::isfinite(arc.back())) {
    return {};
  }
  const double limit = std::min(arc.back(), horizon);
  std::vector<Point2> output;
  for (double target = 0.0; target <= limit + kEpsilon; target += spacing) {
    output.push_back(interpolate(points, arc, std::min(target, limit)));
  }
  const Point2 endpoint = interpolate(points, arc, limit);
  if (output.empty() || distance(output.back(), endpoint) > duplicate_tolerance) {
    output.push_back(endpoint);
  }
  return output;
}

}

double normalizeAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

std::vector<Point2> offsetBoundary(
  const std::vector<Point2> & ordered, BoundarySide side, double offset)
{
  std::vector<Point2> centerline;
  centerline.reserve(ordered.size());
  for (std::size_t index = 0; index < ordered.size(); ++index) {
    Point2 tangent;
    if (index == 0U) {
      tangent = {ordered[1U].x - ordered[0U].x, ordered[1U].y - ordered[0U].y};
    } else if (index + 1U == ordered.size()) {
      tangent = {ordered[index].x - ordered[index - 1U].x, ordered[index].y - ordered[index - 1U].y};
    } else {
      tangent = {
        ordered[index + 1U].x - ordered[index - 1U].x,
        ordered[index + 1U].y - ordered[index - 1U].y,
      };
    }
    const double norm = std::hypot(tangent.x, tangent.y);
    if (!std::isfinite(norm) || norm <= kEpsilon) {
      return {};
    }
    tangent.x /= norm;
    tangent.y /= norm;
    const Point2 normal = side == BoundarySide::kBlue ?
      Point2{tangent.y, -tangent.x} : Point2{-tangent.y, tangent.x};
    centerline.push_back({
      ordered[index].x + offset * normal.x,
      ordered[index].y + offset * normal.y,
    });
  }
  return centerline;
}

BuildResult finishPath(
  const std::vector<Point2> & raw_path, double horizon, double speed, PathKind kind,
  const PlannerConfig & config)
{
  BuildResult result;
  result.evaluated = true;
  result.kind = kind;
  if (raw_path.size() < 2U) {
    result.reason = "fewer than two ordered raw points";
    return result;
  }
  if (distance(raw_path.front(), {0.0, 0.0}) > config.max_start_distance_m) {
    result.reason = "path_start_too_far";
    return result;
  }
  // Straight-corridor missions must never publish a path that heads backward:
  // once the corridor cones fall behind the ego the path should simply end so
  // the car brakes, not fling a waypoint chain behind the car (dangerous near
  // the braking-zone cones). Curved missions keep u-turns, so this is gated.
  if (config.extend_straight_to_horizon && raw_path.back().x <= raw_path.front().x) {
    result.reason = "path_heads_backward";
    return result;
  }
  if (pathSelfIntersects(raw_path)) {
    result.reason = "ordered path self-intersects";
    return result;
  }
  const auto sampled = resample(
    raw_path, config.waypoint_spacing_m, horizon, config.duplicate_tolerance_m);
  if (sampled.size() < 5U) {
    result.reason = "fewer than five resampled points";
    return result;
  }

  const auto arc = cumulativeArc(sampled);
  if (arc.size() != sampled.size()) {
    result.reason = "non-increasing resampled geometry";
    return result;
  }
  result.waypoints.resize(sampled.size());
  for (std::size_t index = 0; index < sampled.size(); ++index) {
    const std::size_t from = index + 1U < sampled.size() ? index : index - 1U;
    const std::size_t to = index + 1U < sampled.size() ? index + 1U : index;
    result.waypoints[index] = {
      sampled[index].x,
      sampled[index].y,
      arc[index],
      std::atan2(sampled[to].y - sampled[from].y, sampled[to].x - sampled[from].x),
      0.0,
      speed,
    };
  }
  for (std::size_t index = 1; index < result.waypoints.size(); ++index) {
    const double delta_s = result.waypoints[index].s - result.waypoints[index - 1U].s;
    result.waypoints[index].kappa = normalizeAngle(
      result.waypoints[index].psi - result.waypoints[index - 1U].psi) / delta_s;
  }
  // Curvature speed profile: cap corner speed at sqrt(a_lat / |kappa|),
  // floored at min_speed, using the per-path speed as the straight-line cap.
  for (auto & waypoint : result.waypoints) {
    double v = speed;
    const double kappa = std::abs(waypoint.kappa);
    if (config.max_lateral_accel_mps2 > 0.0 && kappa > 1e-6) {
      v = std::min(v, std::sqrt(config.max_lateral_accel_mps2 / kappa));
    }
    waypoint.speed = std::max(config.min_speed_mps, v);
  }
  // Open-end stop taper (stop_at_path_end): walk the speed down to zero over
  // the final metres of the path so the controller rolls to rest at the
  // corridor end. Applied after -- and allowed below -- the min_speed floor:
  // the floor keeps the car moving mid-track, but reaching zero is the point
  // here. See PlannerConfig::stop_at_path_end for the frontier-vs-end argument.
  if (config.stop_at_path_end) {
    const double end_s = result.waypoints.back().s;
    for (auto & waypoint : result.waypoints) {
      const double remaining = std::max(
        0.0, end_s - waypoint.s - config.end_stop_margin_m);
      waypoint.speed = std::min(
        waypoint.speed, std::sqrt(2.0 * config.end_stop_decel_mps2 * remaining));
    }
  }
  for (std::size_t index = 0; index < result.waypoints.size(); ++index) {
    const auto & waypoint = result.waypoints[index];
    if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) ||
      !std::isfinite(waypoint.s) || !std::isfinite(waypoint.psi) ||
      !std::isfinite(waypoint.kappa) || !std::isfinite(waypoint.speed) ||
      (index > 0U && !(waypoint.s > result.waypoints[index - 1U].s)))
    {
      result.waypoints.clear();
      result.reason = "generated path is non-finite or non-increasing";
      return result;
    }
  }
  result.valid = true;
  result.reason.clear();
  return result;
}

}
