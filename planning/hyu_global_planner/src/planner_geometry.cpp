#include "hyu_global_planner/planner_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace hyu_global_planner
{

namespace
{
constexpr double kGeometryEpsilon = 1.0e-9;
}

double distance(const PlannerPoint & a, const PlannerPoint & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

PlannerPoint add(const PlannerPoint & a, const PlannerPoint & b)
{
  return PlannerPoint{a.x + b.x, a.y + b.y};
}

PlannerPoint subtract(const PlannerPoint & a, const PlannerPoint & b)
{
  return PlannerPoint{a.x - b.x, a.y - b.y};
}

PlannerPoint scale(const PlannerPoint & point, double factor)
{
  return PlannerPoint{point.x * factor, point.y * factor};
}

double dot(const PlannerPoint & a, const PlannerPoint & b)
{
  return a.x * b.x + a.y * b.y;
}

double cross2d(const PlannerPoint & a, const PlannerPoint & b)
{
  return a.x * b.y - a.y * b.x;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

bool isFinitePoint(const PlannerPoint & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

std::optional<double> median(std::vector<double> values)
{
  if (values.empty()) {
    return std::nullopt;
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2U == 1U) {
    return *middle;
  }
  const double upper = *middle;
  const auto lower_it = std::max_element(values.begin(), middle);
  return 0.5 * (*lower_it + upper);
}

std::vector<double> cumulativeArcLengths(const std::vector<PlannerPoint> & points)
{
  std::vector<double> arc_lengths(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    arc_lengths[i] = arc_lengths[i - 1] + distance(points[i - 1], points[i]);
  }
  return arc_lengths;
}

PlannerPoint interpolateAtArcLength(
  const std::vector<PlannerPoint> & points, const std::vector<double> & arc_lengths,
  double target_s)
{
  if (points.size() == 1U || target_s <= 0.0) {
    return points.front();
  }
  if (target_s >= arc_lengths.back()) {
    return points.back();
  }

  const auto upper = std::upper_bound(arc_lengths.begin(), arc_lengths.end(), target_s);
  const std::size_t i = static_cast<std::size_t>(upper - arc_lengths.begin());
  const double segment_length = arc_lengths[i] - arc_lengths[i - 1];
  if (segment_length <= kGeometryEpsilon) {
    return points[i];
  }
  const double ratio = (target_s - arc_lengths[i - 1]) / segment_length;
  return add(points[i - 1], scale(subtract(points[i], points[i - 1]), ratio));
}

std::vector<PlannerPoint> resampleBySpacing(
  const std::vector<PlannerPoint> & points, double spacing, double duplicate_tolerance)
{
  if (points.size() < 2U) {
    return points;
  }
  if (!std::isfinite(spacing) || spacing < kMinimumWaypointSpacingM ||
    !std::isfinite(duplicate_tolerance))
  {
    return {};
  }
  const double safe_duplicate_tolerance = std::max(0.0, duplicate_tolerance);
  const auto arc_lengths = cumulativeArcLengths(points);
  const double total_length = arc_lengths.back();
  if (!std::isfinite(total_length) || total_length <= safe_duplicate_tolerance) {
    return {};
  }

  std::vector<PlannerPoint> resampled;
  const double effective_spacing = std::max(spacing, safe_duplicate_tolerance);
  for (double s = 0.0; s < total_length; s += effective_spacing) {
    const auto point = interpolateAtArcLength(points, arc_lengths, s);
    if (resampled.empty() || distance(resampled.back(), point) > safe_duplicate_tolerance) {
      resampled.push_back(point);
    }
  }

  const auto last = points.back();
  if (resampled.empty() || distance(resampled.back(), last) > safe_duplicate_tolerance) {
    resampled.push_back(last);
  }
  return resampled;
}

std::vector<PlannerPoint> sampleNormalized(
  const std::vector<PlannerPoint> & points, std::size_t count)
{
  const auto arc_lengths = cumulativeArcLengths(points);
  std::vector<PlannerPoint> samples;
  samples.reserve(count);
  if (count <= 1U) {
    samples.push_back(points.front());
    return samples;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(count - 1U);
    samples.push_back(interpolateAtArcLength(points, arc_lengths, ratio * arc_lengths.back()));
  }
  return samples;
}

void computeWaypointGeometry(std::vector<PlannerWaypoint> & waypoints)
{
  if (waypoints.empty()) {
    return;
  }
  waypoints.front().s = 0.0;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    waypoints[i].s = waypoints[i - 1].s +
      distance({waypoints[i - 1].x, waypoints[i - 1].y}, {waypoints[i].x, waypoints[i].y});
  }

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    std::size_t from = i;
    std::size_t to = i + 1U;
    if (to >= waypoints.size()) {
      from = i - 1U;
      to = i;
    }
    waypoints[i].psi = std::atan2(
      waypoints[to].y - waypoints[from].y, waypoints[to].x - waypoints[from].x);
  }

  waypoints.front().kappa = 0.0;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    const double ds = waypoints[i].s - waypoints[i - 1].s;
    waypoints[i].kappa = ds <= kGeometryEpsilon ? 0.0 :
      normalizeAngle(waypoints[i].psi - waypoints[i - 1].psi) / ds;
  }
}

std::vector<VelocityProfilePoint> computeVelocityProfile(
  const std::vector<PlannerWaypoint> & waypoints, const VelocityProfileConfig & config)
{
  const std::size_t n = waypoints.size();
  std::vector<VelocityProfilePoint> profile(n);
  if (n == 0U) {
    return profile;
  }

  const double v_max = std::max(config.min_speed_mps, config.max_speed_mps);
  const double a_lat = std::max(0.0, config.max_lateral_accel_mps2);
  const double a_acc = std::max(0.0, config.max_accel_mps2);
  const double a_dec = std::max(0.0, config.max_decel_mps2);

  // 1. Curvature (friction-circle) speed limit per point.
  std::vector<double> v(n, v_max);
  for (std::size_t i = 0; i < n; ++i) {
    const double kappa = std::abs(waypoints[i].kappa);
    double v_curve = v_max;
    if (a_lat > 0.0 && kappa > kGeometryEpsilon) {
      v_curve = std::sqrt(a_lat / kappa);
    }
    v[i] = std::clamp(v_curve, config.min_speed_mps, v_max);
  }

  // 2. Forward pass: cap acceleration between points.
  for (std::size_t i = 1; i < n; ++i) {
    const double ds = std::max(0.0, waypoints[i].s - waypoints[i - 1].s);
    const double reachable = std::sqrt(v[i - 1] * v[i - 1] + 2.0 * a_acc * ds);
    v[i] = std::min(v[i], reachable);
  }

  // 3. Backward pass: cap braking so every corner speed is reachable.
  for (std::size_t i = n - 1; i-- > 0; ) {
    const double ds = std::max(0.0, waypoints[i + 1].s - waypoints[i].s);
    const double reachable = std::sqrt(v[i + 1] * v[i + 1] + 2.0 * a_dec * ds);
    v[i] = std::min(v[i], reachable);
  }

  // 4. Longitudinal accel toward the next point.
  for (std::size_t i = 0; i < n; ++i) {
    profile[i].vx = std::clamp(v[i], config.min_speed_mps, v_max);
  }
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double ds = std::max(kGeometryEpsilon, waypoints[i + 1].s - waypoints[i].s);
    profile[i].ax = (profile[i + 1].vx * profile[i + 1].vx - profile[i].vx * profile[i].vx) /
      (2.0 * ds);
  }
  if (n > 0U) {
    profile[n - 1].ax = 0.0;
  }
  return profile;
}

}  // namespace hyu_global_planner
