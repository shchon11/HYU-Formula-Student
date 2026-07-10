#include "local_planner/local_path_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "local_path_builder_internal.hpp"

namespace local_planner
{
namespace
{

bool validConfig(const PlannerConfig & config)
{
  const double values[] = {
    config.roi_min_x,
    config.roi_max_x,
    config.roi_abs_y,
    config.endpoint_match_tolerance_m,
    config.min_track_width_m,
    config.max_track_width_m,
    config.duplicate_tolerance_m,
    config.min_forward_projection_m,
    config.max_traversal_gap_m,
    config.max_heading_change_rad,
    config.waypoint_spacing_m,
    config.max_start_distance_m,
    config.two_sided_horizon_m,
    config.fallback_horizon_m,
    config.fallback_offset_m,
    config.two_sided_speed_mps,
    config.fallback_speed_mps,
  };
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return config.roi_min_x < config.roi_max_x && config.roi_abs_y > 0.0 &&
         config.endpoint_match_tolerance_m >= 0.0 && config.min_track_width_m > 0.0 &&
         config.min_track_width_m <= config.max_track_width_m &&
         config.duplicate_tolerance_m >= 0.0 && config.min_forward_projection_m >= 0.0 &&
         config.max_traversal_gap_m > 0.0 && config.max_heading_change_rad >= 0.0 &&
         config.waypoint_spacing_m > config.duplicate_tolerance_m &&
         config.max_start_distance_m > 0.0 &&
         config.two_sided_horizon_m > 0.0 && config.fallback_horizon_m > 0.0 &&
         config.fallback_offset_m > 0.0 && config.two_sided_speed_mps > 0.0 &&
         config.fallback_speed_mps > 0.0;
}

}

BuildResult buildLocalPath(const ConeSet & cones, const PlannerConfig & config)
{
  BuildResult invalid;
  invalid.evaluated = true;
  if (!validConfig(config)) {
    invalid.reason = "planner configuration is invalid";
    return invalid;
  }
  if (cones.input_overflow || cones.blue.size() > kMaxBoundaryCones ||
    cones.yellow.size() > kMaxBoundaryCones)
  {
    invalid.reason = "cone input exceeds bounded planner capacity";
    return invalid;
  }

  const auto blue = internal::cropToRoi(cones.blue, config);
  const auto yellow = internal::cropToRoi(cones.yellow, config);
  BuildResult two_sided;
  if (blue.size() >= 2U && yellow.size() >= 2U) {
    const auto centerline = internal::boundaryMidpoints(blue, yellow, config);
    two_sided = internal::finishPath(
      centerline, config.two_sided_horizon_m, config.two_sided_speed_mps,
      PathKind::kTwoSided, config);
    if (two_sided.valid) {
      return two_sided;
    }
  }

  if (!config.allow_partial_boundary && two_sided.evaluated) {
    invalid.reason = two_sided.reason;
    return invalid;
  }

  const bool can_recover_one_sided = blue.size() >= 2U && yellow.size() >= 2U;
  const bool blue_only = blue.size() >= 3U &&
    (yellow.empty() || (can_recover_one_sided && yellow.size() >= 2U));
  const bool yellow_only = yellow.size() >= 3U &&
    (blue.empty() || (can_recover_one_sided && blue.size() >= 2U));
  if (!blue_only && !yellow_only) {
    invalid.reason = two_sided.evaluated ? two_sided.reason :
      "known boundary counts satisfy neither planning mode";
    return invalid;
  }

  const auto nearest = [](const std::vector<Point2> & points) {
      double value = std::numeric_limits<double>::infinity();
      for (const auto & point : points) {
        value = std::min(value, std::hypot(point.x, point.y));
      }
      return value;
    };
  const bool use_blue = blue_only && (!yellow_only || nearest(blue) <= nearest(yellow));
  const auto side = use_blue ? internal::BoundarySide::kBlue : internal::BoundarySide::kYellow;
  const auto sanitized_boundary = internal::deduplicate(
    use_blue ? blue : yellow, config.duplicate_tolerance_m);
  if (sanitized_boundary.size() < 3U) {
    invalid.reason = "one-sided boundary has fewer than three unique cones";
    return invalid;
  }
  const auto boundary = internal::forwardTraversal(sanitized_boundary, config);
  if (!config.allow_partial_boundary && boundary.size() != sanitized_boundary.size()) {
    invalid.reason = "one-sided traversal did not consume sanitized boundary";
    return invalid;
  }
  if (boundary.size() < 2U || pathSelfIntersects(boundary)) {
    invalid.reason = "one-sided traversal is invalid";
    return invalid;
  }
  const auto centerline = internal::offsetBoundary(boundary, side, config.fallback_offset_m);
  return internal::finishPath(
    centerline, config.fallback_horizon_m, config.fallback_speed_mps,
    use_blue ? PathKind::kBlueOnly : PathKind::kYellowOnly, config);
}

}
