#include "hyu_local_planner/local_path_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "local_path_builder_internal.hpp"

namespace hyu_local_planner
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
    config.unknown_absorb_lateral_m,
    config.unknown_geom_deadband_m,
    config.straight_extension_cap_m,
    config.end_stop_decel_mps2,
    config.end_stop_margin_m,
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
         config.fallback_speed_mps > 0.0 && config.unknown_absorb_lateral_m >= 0.0 &&
         config.unknown_geom_deadband_m >= 0.0 && config.straight_extension_cap_m >= 0.0 &&
         config.end_stop_decel_mps2 > 0.0 && config.end_stop_margin_m >= 0.0;
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

// Carry `points` forward until the polyline spans `target` arc length so a
// sparse fix still yields enough geometry to resample. STRAIGHT extension along
// the overall (first->last) heading -- no curvature/arc fitting. Using the
// overall heading (not the last segment's tangent) keeps a noisy final point
// from swinging the extension off-axis. Forward-only (dir_x > 0) so a centerline
// that points backward near the end of a run is not flung behind the car. No-op
// once the path already reaches `target`.
std::vector<Point2> extendToLength(std::vector<Point2> points, double target)
{
  if (points.size() < 2U) {
    return points;
  }
  double length = 0.0;
  for (std::size_t i = 1U; i < points.size(); ++i) {
    length += internal::distance(points[i - 1U], points[i]);
  }
  if (!(length < target)) {
    return points;
  }
  const Point2 & first = points.front();
  const Point2 & last = points.back();
  const double dir_x = last.x - first.x;
  const double dir_y = last.y - first.y;
  const double norm = std::hypot(dir_x, dir_y);
  if (!std::isfinite(norm) || norm <= 0.0 || dir_x <= 0.0) {
    return points;
  }
  const double extra = target - length;
  points.push_back({last.x + dir_x / norm * extra, last.y + dir_y / norm * extra});
  return points;
}

// Straight-corridor path (acceleration mission). Fits the corridor centerline
// through EVERY width-gated blue/yellow midpoint -- the ones behind the ego as
// well as ahead -- and returns a straight line from the ego forward to a bounded
// distance past the last cone. Two effects:
//   * No mid-run brake pulses. With slow perception the car keeps outrunning
//     the mapped frontier; a forward-only centerline collapses and the path
//     goes invalid (brake). Fitting through the cones the car has ALREADY
//     passed gives a stable straight line to project forward, so the path stays
//     valid across the frontier as long as behind cones remain in the ROI
//     (widen roi_min_x to raise that tolerance).
//   * Still stops. The line is only carried `straight_extension_cap_m` past the
//     furthest cone, so once the whole corridor falls that far behind the ego
//     the path shrinks to nothing, goes invalid, and the car brakes. The stop
//     point is last-cone + cap (minus the resample minimum), independent of how
//     wide roi_min_x is.
// Returns an empty path (-> invalid -> brake) when the corridor is degenerate
// or entirely behind the ego. SAFE ONLY on a genuinely straight track.
std::vector<Point2> straightCorridorPath(
  const std::vector<Point2> & blue, const std::vector<Point2> & yellow,
  const PlannerConfig & config)
{
  std::vector<Point2> midpoints;
  for (const Point2 & b : blue) {
    for (const Point2 & y : yellow) {
      const double width = internal::distance(b, y);
      if (b.y > y.y && width >= config.min_track_width_m && width <= config.max_track_width_m) {
        midpoints.push_back({0.5 * (b.x + y.x), 0.5 * (b.y + y.y)});
      }
    }
  }
  midpoints = internal::deduplicate(std::move(midpoints), config.waypoint_spacing_m);
  if (midpoints.size() < 2U) {
    return {};
  }
  // Least-squares line y = intercept + slope * x through the midpoints.
  const double n = static_cast<double>(midpoints.size());
  double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
  double x_max = -std::numeric_limits<double>::infinity();
  for (const Point2 & m : midpoints) {
    sum_x += m.x;
    sum_y += m.y;
    sum_xx += m.x * m.x;
    sum_xy += m.x * m.y;
    x_max = std::max(x_max, m.x);
  }
  const double denom = n * sum_xx - sum_x * sum_x;
  if (!std::isfinite(denom) || std::abs(denom) < 1.0e-9) {
    return {};  // corridor perpendicular to the car / all cones at one x -- reject
  }
  const double slope = (n * sum_xy - sum_x * sum_y) / denom;
  const double intercept = (sum_y - slope * sum_x) / n;
  const double x_end = x_max + config.straight_extension_cap_m;
  if (!(x_end > 0.0)) {
    return {};  // corridor entirely behind the ego -> no forward path -> stop
  }
  // Two endpoints define the straight line; finishPath resamples it. Start at
  // the ego so the path is anchored at the car.
  return {{0.0, intercept}, {x_end, intercept + slope * x_end}};
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
  const bool fold_orange_cones =
    config.use_orange_cones && !config.extend_straight_to_horizon;
  if (cones.input_overflow || cones.blue.size() > kMaxBoundaryCones ||
    cones.yellow.size() > kMaxBoundaryCones ||
    (config.use_unknown_cones && cones.unknown.size() > kMaxBoundaryCones) ||
    (fold_orange_cones &&
    (cones.orange.size() > kMaxBoundaryCones ||
    cones.big_orange.size() > kMaxBoundaryCones)))
  {
    invalid.reason = "cone input exceeds bounded planner capacity";
    return invalid;
  }

  auto blue = internal::cropToRoi(cones.blue, config);
  auto yellow = internal::cropToRoi(cones.yellow, config);
  bool had_boundary_input = !cones.blue.empty() || !cones.yellow.empty();
  // Colourless boundary evidence: unknown-colour cones and, when enabled for
  // the mission, the orange/big-orange start-finish cones. Both go through the
  // same conservative classification (absorb onto an unambiguous labelled
  // boundary line, else ego-side split outside the dead-band) so a lap-close
  // frame that sees only the orange gate still yields boundary cones instead
  // of "roi_no_boundary_cones" -> brake. The straight-corridor mission keeps
  // its own fold-in below.
  std::vector<Point2> colorless;
  if (config.use_unknown_cones && !cones.unknown.empty()) {
    const auto unknown = internal::cropToRoi(cones.unknown, config);
    colorless.insert(colorless.end(), unknown.begin(), unknown.end());
    had_boundary_input = had_boundary_input || !unknown.empty();
  }
  if (fold_orange_cones) {
    for (const std::vector<Point2> * source : {&cones.orange, &cones.big_orange}) {
      const auto cropped = internal::cropToRoi(*source, config);
      colorless.insert(colorless.end(), cropped.begin(), cropped.end());
      had_boundary_input = had_boundary_input || !cropped.empty();
    }
  }
  if (!colorless.empty()) {
    internal::classifyUnknownCones(blue, yellow, colorless, config);
  }
  // Straight-corridor missions fold in the big-orange start/finish gates. On an
  // acceleration track the gates sit ON the corridor line (y = +/- half width),
  // so splitting them by ego side gives the car boundary cones the instant it
  // sees the start gate (it can launch immediately instead of waiting for two
  // blue/yellow pairs) and carries the corridor across the finish gate at speed.
  if (config.extend_straight_to_horizon && !cones.big_orange.empty()) {
    for (const Point2 & cone : internal::cropToRoi(cones.big_orange, config)) {
      if (cone.y >= config.unknown_geom_deadband_m) {
        blue.push_back(cone);
      } else if (cone.y <= -config.unknown_geom_deadband_m) {
        yellow.push_back(cone);
      }
    }
    had_boundary_input = had_boundary_input || !cones.big_orange.empty();
  }
  if (had_boundary_input && blue.empty() && yellow.empty()) {
    invalid.reason = "roi_no_boundary_cones";
    return invalid;
  }
  // Straight-corridor missions (acceleration) prefer a line fitted through the
  // cones behind and ahead of the ego, bounded a little past the last cone: it
  // keeps a valid forward path when the car outruns the mapped frontier (no
  // mid-run brake pulses), and goes invalid -- so the car brakes and stops --
  // once the corridor falls far enough behind. See straightCorridorPath. It
  // needs at least two cone pairs to fit a line; when fewer are visible (the
  // single pair at the very start of the run, or a gap) we fall through to the
  // normal two-sided/one-sided/sparse logic so the car can still creep forward
  // and accumulate the map.
  if (config.extend_straight_to_horizon) {
    // Fold the orange braking-zone cones into the boundary too, so the path
    // stays continuous through the braking zone instead of collapsing onto the
    // blue/yellow behind the car (which reads as a deviation at the stop).
    // Speed: full while a corridor cone (blue/yellow/big-orange) is still at or
    // ahead of the ego; once the whole corridor is behind (only orange ahead)
    // the car is in the braking zone, so drop to the fallback speed and let it
    // decelerate to the stop. Using the furthest corridor cone (not "anything
    // strictly ahead") keeps full speed through a frontier outrun, where the
    // last mapped cone sits right at the ego.
    double corridor_max_x = -std::numeric_limits<double>::infinity();
    for (const Point2 & cone : blue) {
      corridor_max_x = std::max(corridor_max_x, cone.x);
    }
    for (const Point2 & cone : yellow) {
      corridor_max_x = std::max(corridor_max_x, cone.x);
    }
    const bool in_corridor = corridor_max_x > -config.waypoint_spacing_m;
    for (const Point2 & cone : internal::cropToRoi(cones.orange, config)) {
      if (cone.y >= config.unknown_geom_deadband_m) {
        blue.push_back(cone);
      } else if (cone.y <= -config.unknown_geom_deadband_m) {
        yellow.push_back(cone);
      }
    }
    const double speed =
      in_corridor ? config.two_sided_speed_mps : config.fallback_speed_mps;
    const auto fitted = straightCorridorPath(blue, yellow, config);
    if (fitted.size() >= 2U) {
      BuildResult straight = internal::finishPath(
        fitted, config.two_sided_horizon_m, speed, PathKind::kTwoSided, config);
      if (straight.valid) {
        straight.straight_fit = true;
        straight.straight_line_start = fitted.front();
        straight.straight_line_end = fitted.back();
        straight.straight_corridor_max_x = corridor_max_x;
        return straight;
      }
    }
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

std::optional<StraightLineLatch> latchStraightLine(
  const BuildResult & result, double ego_x, double ego_y, double ego_yaw)
{
  if (!result.straight_fit) {
    return std::nullopt;
  }
  const double cos_yaw = std::cos(ego_yaw);
  const double sin_yaw = std::sin(ego_yaw);
  const auto to_odom = [&](const Point2 & p) -> Point2 {
      return {
        ego_x + p.x * cos_yaw - p.y * sin_yaw,
        ego_y + p.x * sin_yaw + p.y * cos_yaw,
      };
    };
  const Point2 start = to_odom(result.straight_line_start);
  const Point2 end = to_odom(result.straight_line_end);
  const double length = internal::distance(start, end);
  if (!std::isfinite(length) || length <= 1.0e-9 ||
    !std::isfinite(start.x) || !std::isfinite(start.y))
  {
    return std::nullopt;
  }
  StraightLineLatch latch;
  latch.origin = start;
  latch.dir = {(end.x - start.x) / length, (end.y - start.y) / length};
  latch.path_end_s = length;
  // The fitted line is parameterised by ego-frame x (start sits at x=0, end at
  // x_end > 0), so the furthest corridor cone maps onto the line at the same
  // x fraction. -inf (fit ran on braking-zone orange only) stays -inf: the
  // whole held path is then braking-speed, matching the live fit.
  const double end_x = result.straight_line_end.x;
  latch.corridor_end_s =
    std::isfinite(result.straight_corridor_max_x) && end_x > 1.0e-9 ?
    length * result.straight_corridor_max_x / end_x :
    -std::numeric_limits<double>::infinity();
  return latch;
}

BuildResult buildHeldStraightPath(
  const StraightLineLatch & latch, double ego_x, double ego_y, double ego_yaw,
  const PlannerConfig & config)
{
  BuildResult invalid;
  invalid.evaluated = true;
  const double dir_norm = std::hypot(latch.dir.x, latch.dir.y);
  if (!std::isfinite(dir_norm) || std::abs(dir_norm - 1.0) > 1.0e-6 ||
    !std::isfinite(latch.path_end_s) || !std::isfinite(ego_x) ||
    !std::isfinite(ego_y) || !std::isfinite(ego_yaw))
  {
    invalid.reason = "held straight line latch is degenerate";
    return invalid;
  }
  const double s_ego =
    (ego_x - latch.origin.x) * latch.dir.x + (ego_y - latch.origin.y) * latch.dir.y;
  if (latch.path_end_s - s_ego <= 0.0) {
    invalid.reason = "held straight line is behind the ego";
    return invalid;
  }
  const double cos_yaw = std::cos(ego_yaw);
  const double sin_yaw = std::sin(ego_yaw);
  const auto to_ego = [&](double s) -> Point2 {
      const double odom_x = latch.origin.x + s * latch.dir.x - ego_x;
      const double odom_y = latch.origin.y + s * latch.dir.y - ego_y;
      return {
        odom_x * cos_yaw + odom_y * sin_yaw,
        -odom_x * sin_yaw + odom_y * cos_yaw,
      };
    };
  // Same speed rule as the live fit: full while the latched corridor end is
  // still at or ahead of the ego, braking speed once only the extension (the
  // braking zone) remains.
  const bool in_corridor = latch.corridor_end_s - s_ego > -config.waypoint_spacing_m;
  const double speed =
    in_corridor ? config.two_sided_speed_mps : config.fallback_speed_mps;
  // finishPath applies the start-distance gate (a pose jump off the line stays
  // a brake, not a swerve back), the no-backward gate (a flipped heading never
  // yields a path behind the car), and the minimum resampled length (so the
  // hold still ends -- and the car still stops -- at the latched line end).
  return internal::finishPath(
    {to_ego(s_ego), to_ego(latch.path_end_s)}, config.two_sided_horizon_m, speed,
    PathKind::kTwoSided, config);
}

}
