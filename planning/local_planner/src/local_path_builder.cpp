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
    config.max_u_turn_heading_change_rad,
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
         config.max_u_turn_heading_change_rad >= config.max_heading_change_rad &&
         config.waypoint_spacing_m > config.duplicate_tolerance_m &&
         config.max_start_distance_m > 0.0 &&
         config.two_sided_horizon_m > 0.0 && config.fallback_horizon_m > 0.0 &&
         config.fallback_offset_m > 0.0 && config.two_sided_speed_mps > 0.0 &&
         config.fallback_speed_mps > 0.0;
}

std::string traversalFailureReason(internal::TraversalFailure failure)
{
  switch (failure) {
    case internal::TraversalFailure::kNone:
      return "";
    case internal::TraversalFailure::kTopologyGap:
      return "local_topology_gap";
    case internal::TraversalFailure::kHeadingJump:
      return "local_heading_jump";
    case internal::TraversalFailure::kUTurnBranchAmbiguous:
      return "u_turn_branch_ambiguous";
    case internal::TraversalFailure::kFoldBack:
      return "local_fold_back";
  }
  return "local_topology_gap";
}

// Extend the final segment direction until the polyline reaches `target`
// arc length, so sparse fixes still yield enough geometry to resample.
std::vector<Point2> extendToLength(std::vector<Point2> points, double target)
{
  if (points.size() < 2U) {
    return points;
  }
  double length = 0.0;
  for (std::size_t i = 1U; i < points.size(); ++i) {
    length += internal::distance(points[i - 1U], points[i]);
  }
  if (length >= target) {
    return points;
  }
  const Point2 & tail = points[points.size() - 2U];
  const Point2 & head = points.back();
  const double segment = internal::distance(tail, head);
  if (segment <= 0.0) {
    return points;
  }
  const double extra = target - length;
  points.push_back(
    {head.x + (head.x - tail.x) / segment * extra,
      head.y + (head.y - tail.y) / segment * extra});
  return points;
}

// Sparse-map fallback: runs only when the regular two-sided (2+2) and
// one-sided (>=3) modes are unavailable AND allow_partial_boundary is set
// (slam_map default). A single width-gated blue/yellow pair — or a lone
// boundary side with two cones — is enough to creep forward instead of
// deadlocking with the car stranded beyond the mapped frontier.
BuildResult buildSparseFallback(
  const std::vector<Point2> & blue, const std::vector<Point2> & yellow,
  const PlannerConfig & config)
{
  BuildResult result;
  result.evaluated = true;
  result.reason = "sparse fallback: no usable cone pair or boundary side";

  // A) Width-gated blue<->yellow pairs -> centerline midpoints.
  std::vector<Point2> midpoints;
  for (const Point2 & b : blue) {
    for (const Point2 & y : yellow) {
      if (b.y <= y.y) {
        continue;
      }
      const double width = internal::distance(b, y);
      if (width >= config.min_track_width_m && width <= config.max_track_width_m) {
        midpoints.push_back({(b.x + y.x) * 0.5, (b.y + y.y) * 0.5});
      }
    }
  }
  midpoints = internal::deduplicate(std::move(midpoints), config.waypoint_spacing_m);
  if (!midpoints.empty()) {
    std::sort(
      midpoints.begin(), midpoints.end(),
      [](const Point2 & a, const Point2 & b) {
        return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);
      });
    std::vector<Point2> raw;
    raw.push_back({0.0, 0.0});  // anchor at the ego pose
    raw.insert(raw.end(), midpoints.begin(), midpoints.end());
    raw = extendToLength(std::move(raw), config.fallback_horizon_m);
    BuildResult sparse = internal::finishPath(
      raw, config.fallback_horizon_m, config.fallback_speed_mps,
      PathKind::kTwoSided, config);
    if (sparse.valid) {
      return sparse;
    }
    result.reason = sparse.reason;
  }

  // B) A lone boundary side with two cones (below the >=3 one-sided gate).
  const bool blue_usable = blue.size() >= 2U;
  const bool yellow_usable = yellow.size() >= 2U;
  if (blue_usable || yellow_usable) {
    const bool use_blue =
      blue_usable && (!yellow_usable || blue.size() >= yellow.size());
    const auto side =
      use_blue ? internal::BoundarySide::kBlue : internal::BoundarySide::kYellow;
    const auto points = internal::deduplicate(
      use_blue ? blue : yellow, config.duplicate_tolerance_m);
    if (points.size() >= 2U) {
      const auto ordered = internal::forwardTraversal(points, config);
      if (ordered.size() >= 2U && !pathSelfIntersects(ordered)) {
        auto centerline =
          internal::offsetBoundary(ordered, side, config.fallback_offset_m);
        centerline = extendToLength(std::move(centerline), config.fallback_horizon_m);
        BuildResult one_sided = internal::finishPath(
          centerline, config.fallback_horizon_m, config.fallback_speed_mps,
          use_blue ? PathKind::kBlueOnly : PathKind::kYellowOnly, config);
        if (one_sided.valid) {
          return one_sided;
        }
        result.reason = one_sided.reason;
      }
    }
  }

  result.valid = false;
  return result;
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
  const bool had_boundary_input = !cones.blue.empty() || !cones.yellow.empty();
  if (had_boundary_input && blue.empty() && yellow.empty()) {
    invalid.reason = "roi_no_boundary_cones";
    return invalid;
  }
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
    if (config.allow_partial_boundary) {
      const BuildResult sparse = buildSparseFallback(blue, yellow, config);
      if (sparse.valid) {
        return sparse;
      }
      invalid.reason = sparse.reason;
      return invalid;
    }
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
  const auto traversal = internal::forwardTraversalWithReason(sanitized_boundary, config);
  const auto & boundary = traversal.points;
  if (traversal.failure == internal::TraversalFailure::kUTurnBranchAmbiguous) {
    invalid.reason = traversalFailureReason(traversal.failure);
    return invalid;
  }
  if (!config.allow_partial_boundary && boundary.size() != sanitized_boundary.size()) {
    invalid.reason = traversalFailureReason(traversal.failure);
    if (invalid.reason.empty()) {
      invalid.reason = "local_topology_gap";
    }
    return invalid;
  }
  if (boundary.size() < 2U || pathSelfIntersects(boundary)) {
    invalid.reason = traversalFailureReason(traversal.failure);
    if (invalid.reason.empty()) {
      invalid.reason = "one-sided traversal is invalid";
    }
    return invalid;
  }
  const auto centerline = internal::offsetBoundary(boundary, side, config.fallback_offset_m);
  return internal::finishPath(
    centerline, config.fallback_horizon_m, config.fallback_speed_mps,
    use_blue ? PathKind::kBlueOnly : PathKind::kYellowOnly, config);
}

}
