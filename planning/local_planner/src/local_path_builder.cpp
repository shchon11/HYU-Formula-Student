#include "local_planner/local_path_builder.hpp"

#include <cmath>

#include <opencv2/core.hpp>

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

  const auto blue = internal::cropToRoi(cones.blue, config);
  const auto yellow = internal::cropToRoi(cones.yellow, config);
  if (blue.size() >= 2U && yellow.size() >= 2U) {
    try {
      const auto midpoints = internal::delaunayMidpoints(blue, yellow, config);
      const auto ordered = internal::forwardTraversal(midpoints, config);
      return internal::finishPath(
        ordered, config.two_sided_horizon_m, config.two_sided_speed_mps,
        PathKind::kTwoSided, config);
    } catch (const cv::Exception &) {
      invalid.reason = "OpenCV Delaunay construction failed";
      return invalid;
    }
  }

  const bool blue_only = blue.size() >= 3U && yellow.empty();
  const bool yellow_only = yellow.size() >= 3U && blue.empty();
  if (!blue_only && !yellow_only) {
    invalid.reason = "known boundary counts satisfy neither planning mode";
    return invalid;
  }

  const auto side = blue_only ? internal::BoundarySide::kBlue : internal::BoundarySide::kYellow;
  const auto sanitized_boundary = internal::deduplicate(
    blue_only ? blue : yellow, config.duplicate_tolerance_m);
  if (sanitized_boundary.size() < 3U) {
    invalid.reason = "one-sided boundary has fewer than three unique cones";
    return invalid;
  }
  const auto boundary = internal::forwardTraversal(sanitized_boundary, config);
  if (boundary.size() != sanitized_boundary.size()) {
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
    blue_only ? PathKind::kBlueOnly : PathKind::kYellowOnly, config);
}

}
