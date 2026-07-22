#include "hyu_path_selector/transition_blend.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace hyu_path_selector
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

struct Sample
{
  double x{0.0};
  double y{0.0};
  double v{0.0};
};

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

double smoothstep(double u)
{
  u = std::clamp(u, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}

std::vector<double> cumulativeArc(const hyu_msgs::msg::WaypointArrayStamped & path)
{
  std::vector<double> arc(path.waypoints.size(), 0.0);
  for (std::size_t index = 1U; index < path.waypoints.size(); ++index) {
    const auto & previous = path.waypoints[index - 1U];
    const auto & current = path.waypoints[index];
    const double segment = std::hypot(
      current.x_m - previous.x_m, current.y_m - previous.y_m);
    if (!std::isfinite(segment)) {
      return {};
    }
    arc[index] = arc[index - 1U] + segment;
  }
  return arc;
}

Sample sampleAt(
  const hyu_msgs::msg::WaypointArrayStamped & path,
  const std::vector<double> & arc,
  double target)
{
  const auto & waypoints = path.waypoints;
  if (target <= 0.0) {
    return {waypoints.front().x_m, waypoints.front().y_m, waypoints.front().vx_mps};
  }
  if (target >= arc.back()) {
    return {waypoints.back().x_m, waypoints.back().y_m, waypoints.back().vx_mps};
  }
  const auto upper = std::upper_bound(arc.begin(), arc.end(), target);
  const std::size_t index = static_cast<std::size_t>(upper - arc.begin());
  const double span = arc[index] - arc[index - 1U];
  const double ratio = span > kEpsilon ? (target - arc[index - 1U]) / span : 0.0;
  const auto & from = waypoints[index - 1U];
  const auto & to = waypoints[index];
  return {
    from.x_m + ratio * (to.x_m - from.x_m),
    from.y_m + ratio * (to.y_m - from.y_m),
    from.vx_mps + ratio * (to.vx_mps - from.vx_mps),
  };
}

}

std::optional<hyu_msgs::msg::WaypointArrayStamped> buildTransitionPath(
  const hyu_msgs::msg::WaypointArrayStamped & local_trimmed,
  const hyu_msgs::msg::WaypointArrayStamped & global_trimmed,
  double blend_progress_m,
  const TransitionBlendConfig & config)
{
  if (local_trimmed.waypoints.size() < 2U || global_trimmed.waypoints.size() < 2U) {
    return std::nullopt;
  }
  if (!std::isfinite(blend_progress_m) || blend_progress_m < 0.0 ||
    !(config.blend_length_m > 0.0) || !(config.waypoint_spacing_m > kEpsilon))
  {
    return std::nullopt;
  }

  const auto local_arc = cumulativeArc(local_trimmed);
  const auto global_arc = cumulativeArc(global_trimmed);
  if (local_arc.empty() || global_arc.empty() ||
    local_arc.back() <= kEpsilon || global_arc.back() <= kEpsilon)
  {
    return std::nullopt;
  }

  const double remaining = config.blend_length_m - blend_progress_m;
  // The blend window shrinks to what the frozen local path can still cover;
  // the weight ramp is compressed accordingly so it always reaches 1.
  const double window = std::min({remaining, local_arc.back(), global_arc.back()});
  if (window <= config.waypoint_spacing_m) {
    return std::nullopt;
  }

  const double u0 = std::clamp(blend_progress_m / config.blend_length_m, 0.0, 1.0);

  // Cross-faded blend samples over [0, window), seam handled by the tail.
  std::vector<Sample> blended;
  for (double s = 0.0; s < window - 0.5 * config.waypoint_spacing_m;
    s += config.waypoint_spacing_m)
  {
    const double u = u0 + (1.0 - u0) * (s / window);
    const double w = smoothstep(u);
    const Sample local_sample = sampleAt(local_trimmed, local_arc, s);
    const Sample global_sample = sampleAt(global_trimmed, global_arc, s);
    blended.push_back({
      local_sample.x + w * (global_sample.x - local_sample.x),
      local_sample.y + w * (global_sample.y - local_sample.y),
      local_sample.v + w * (global_sample.v - local_sample.v),
    });
  }
  if (blended.size() < 2U) {
    return std::nullopt;
  }

  // Global tail: seam point at `window`, then the original global waypoints
  // beyond it, untouched (the raceline speed profile is better than anything
  // we could recompute here).
  const Sample seam = sampleAt(global_trimmed, global_arc, window);
  std::size_t tail_begin = global_trimmed.waypoints.size();
  for (std::size_t index = 0U; index < global_trimmed.waypoints.size(); ++index) {
    if (global_arc[index] > window + 0.5 * config.waypoint_spacing_m) {
      tail_begin = index;
      break;
    }
  }

  hyu_msgs::msg::WaypointArrayStamped output;
  output.header = global_trimmed.header;
  output.waypoints.reserve(
    blended.size() + 1U + (global_trimmed.waypoints.size() - tail_begin));

  // Positions first (blend + seam), then psi/kappa by the same finite
  // differences hyu_local_planner uses, then its curvature speed profile.
  std::vector<Sample> segment = blended;
  segment.push_back(seam);
  double s_accum = 0.0;
  for (std::size_t index = 0U; index < segment.size(); ++index) {
    if (index > 0U) {
      const double step = std::hypot(
        segment[index].x - segment[index - 1U].x,
        segment[index].y - segment[index - 1U].y);
      if (!std::isfinite(step) || step <= kEpsilon) {
        return std::nullopt;
      }
      s_accum += step;
    }
    const std::size_t from = index + 1U < segment.size() ? index : index - 1U;
    const std::size_t to = index + 1U < segment.size() ? index + 1U : index;
    hyu_msgs::msg::Waypoint waypoint;
    waypoint.position.x = segment[index].x;
    waypoint.position.y = segment[index].y;
    waypoint.position.z = 0.0;
    waypoint.suggested_steering = 0.0;
    waypoint.x_m = segment[index].x;
    waypoint.y_m = segment[index].y;
    waypoint.s_m = s_accum;
    waypoint.psi_rad = std::atan2(
      segment[to].y - segment[from].y, segment[to].x - segment[from].x);
    waypoint.kappa_radpm = 0.0;
    waypoint.vx_mps = segment[index].v;
    waypoint.ax_mps2 = 0.0;
    output.waypoints.push_back(waypoint);
  }
  for (std::size_t index = 1U; index < output.waypoints.size(); ++index) {
    auto & waypoint = output.waypoints[index];
    const auto & previous = output.waypoints[index - 1U];
    waypoint.kappa_radpm = normalizeAngle(waypoint.psi_rad - previous.psi_rad) /
      (waypoint.s_m - previous.s_m);
  }
  for (auto & waypoint : output.waypoints) {
    double v = waypoint.vx_mps;
    const double kappa = std::abs(waypoint.kappa_radpm);
    if (config.max_lateral_accel_mps2 > 0.0 && kappa > 1.0e-6) {
      v = std::min(v, std::sqrt(config.max_lateral_accel_mps2 / kappa));
    }
    waypoint.vx_mps = std::max(config.min_speed_mps, v);
    waypoint.speed = waypoint.vx_mps;
  }

  const double seam_s = output.waypoints.back().s_m;
  for (std::size_t index = tail_begin; index < global_trimmed.waypoints.size(); ++index) {
    hyu_msgs::msg::Waypoint waypoint = global_trimmed.waypoints[index];
    waypoint.s_m = seam_s + (global_arc[index] - window);
    waypoint.position.x = waypoint.x_m;
    waypoint.position.y = waypoint.y_m;
    waypoint.speed = waypoint.vx_mps;
    output.waypoints.push_back(waypoint);
  }

  for (std::size_t index = 0U; index < output.waypoints.size(); ++index) {
    const auto & waypoint = output.waypoints[index];
    if (!std::isfinite(waypoint.x_m) || !std::isfinite(waypoint.y_m) ||
      !std::isfinite(waypoint.s_m) || !std::isfinite(waypoint.psi_rad) ||
      !std::isfinite(waypoint.kappa_radpm) || !std::isfinite(waypoint.vx_mps) ||
      (index > 0U && !(waypoint.s_m > output.waypoints[index - 1U].s_m)))
    {
      return std::nullopt;
    }
  }
  return output;
}

}
