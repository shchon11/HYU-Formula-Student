#include "local_path_builder_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <tuple>

namespace local_planner
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

Point2 subtract(const Point2 & first, const Point2 & second)
{
  return {first.x - second.x, first.y - second.y};
}

double dot(const Point2 & first, const Point2 & second)
{
  return first.x * second.x + first.y * second.y;
}

bool finitePoint(const Point2 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool lexicographicPoint(const Point2 & first, const Point2 & second)
{
  return std::tie(first.x, first.y) < std::tie(second.x, second.y);
}

bool uTurnContinuationAllowed(std::size_t ordered_count, std::size_t point_count)
{
  return ordered_count >= 2U && point_count >= 4U;
}

struct TraversalCandidate
{
  std::size_t index{0U};
  double gap{0.0};
  double heading_change{0.0};
  Point2 unit;
};

bool betterNormalCandidate(
  const TraversalCandidate & candidate, const TraversalCandidate & best,
  const std::vector<Point2> & points)
{
  // Tie-break by |y| (proximity to the ego centreline), NOT signed y. Signed y
  // preferred the smaller-y candidate on every near-tie, i.e. the car's right,
  // which hugs the inside of a right-hand circle (stable) but walks the path
  // toward the OUTER wall on a left-hand circle. |y| is mirror-symmetric, so
  // both turn directions resolve ties identically.
  return std::make_tuple(
           candidate.gap, candidate.heading_change, points[candidate.index].x,
           std::abs(points[candidate.index].y), candidate.index) <
         std::make_tuple(
           best.gap, best.heading_change, points[best.index].x,
           std::abs(points[best.index].y), best.index);
}

bool betterUTurnCandidate(
  const TraversalCandidate & candidate, const TraversalCandidate & best,
  const std::vector<Point2> & points)
{
  // Mirror-symmetric tie-break: |y|, not signed y (see betterNormalCandidate).
  return std::make_tuple(
           candidate.heading_change, candidate.gap, points[candidate.index].x,
           std::abs(points[candidate.index].y), candidate.index) <
         std::make_tuple(
           best.heading_change, best.gap, points[best.index].x,
           std::abs(points[best.index].y), best.index);
}

bool closeUTurnBranch(
  const TraversalCandidate & first, const TraversalCandidate & second,
  const PlannerConfig & config)
{
  constexpr double kHeadingToleranceRad = 0.17453292519943295;
  return std::abs(first.gap - second.gap) <= config.waypoint_spacing_m &&
         std::abs(first.heading_change - second.heading_change) <= kHeadingToleranceRad;
}

}

namespace internal
{

double distance(const Point2 & first, const Point2 & second)
{
  return std::hypot(second.x - first.x, second.y - first.y);
}

std::vector<Point2> cropToRoi(const std::vector<Point2> & input, const PlannerConfig & config)
{
  std::vector<Point2> output;
  output.reserve(input.size());
  for (const auto & point : input) {
    if (finitePoint(point) && point.x >= config.roi_min_x && point.x <= config.roi_max_x &&
      std::abs(point.y) <= config.roi_abs_y)
    {
      output.push_back(point);
    }
  }
  std::sort(output.begin(), output.end(), lexicographicPoint);
  return output;
}

std::vector<Point2> deduplicate(
  std::vector<Point2> points, double tolerance, bool exact_only)
{
  std::sort(points.begin(), points.end(), lexicographicPoint);
  std::vector<Point2> output;
  output.reserve(points.size());
  for (const auto & point : points) {
    const double threshold = exact_only ? kEpsilon : tolerance;
    const bool duplicate = std::any_of(
      output.begin(), output.end(),
      [&point, threshold](const Point2 & retained) {
        return distance(point, retained) <= threshold;
      });
    if (!duplicate) {
      output.push_back(point);
    }
  }
  return output;
}

TraversalResult forwardTraversalWithReason(
  const std::vector<Point2> & points, const PlannerConfig & config)
{
  if (points.empty()) {
    return {};
  }
  std::optional<std::size_t> seed;
  // Key is (range, x, |y|) — mirror-symmetric. A signed-y final term used to
  // break exact ties toward the car's right; dropping it keeps left/right
  // circles symmetric (a genuine tie now keeps the first such point).
  std::tuple<double, double, double> seed_key;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto key = std::make_tuple(
      std::hypot(points[index].x, points[index].y), points[index].x,
      std::abs(points[index].y));
    if (!seed.has_value() || key < seed_key) {
      seed = index;
      seed_key = key;
    }
  }
  if (!seed.has_value()) {
    return {};
  }

  std::vector<bool> used(points.size(), false);
  used[*seed] = true;
  std::vector<Point2> ordered{points[*seed]};
  Point2 tangent{1.0, 0.0};
  // Fold-back guard state: a hairpin rounds with CONSECUTIVE sharp turns of
  // the SAME sign, while a chicane fold (the Z that bridges onto an adjacent
  // corridor leg and doubles back) needs two OPPOSITE-sign sharp turns within
  // a short arc. Track the last sharp turn's sign and the arc driven since.
  double last_sharp_sign = 0.0;
  double arc_since_sharp = -1.0;
  const double fold_guard_arc_m = 2.0 * config.max_traversal_gap_m;
  while (ordered.size() < points.size()) {
    std::optional<TraversalCandidate> normal;
    std::optional<TraversalCandidate> u_turn;
    bool u_turn_ambiguous = false;
    bool saw_connected = false;
    bool saw_heading_jump = false;
    for (std::size_t index = 0; index < points.size(); ++index) {
      if (used[index]) {
        continue;
      }
      const Point2 segment = subtract(points[index], ordered.back());
      const double gap = std::hypot(segment.x, segment.y);
      if (!std::isfinite(gap) || gap <= kEpsilon || gap > config.max_traversal_gap_m) {
        continue;
      }
      saw_connected = true;
      const double projection = dot(segment, tangent);
      const Point2 unit{segment.x / gap, segment.y / gap};
      const double heading_change = std::acos(std::clamp(dot(unit, tangent), -1.0, 1.0));
      if (!std::isfinite(heading_change)) {
        continue;
      }
      // Smooth forward continuation: enough forward projection AND a gentle
      // heading. Anything sharper falls through to the u-turn branch below.
      if (projection > config.min_forward_projection_m &&
        heading_change <= config.max_heading_change_rad)
      {
        const TraversalCandidate candidate{index, gap, heading_change, unit};
        if (!normal.has_value() || betterNormalCandidate(candidate, *normal, points)) {
          normal = candidate;
        }
        continue;
      }
      const bool sharp_forward = projection > config.min_forward_projection_m;
      // A sharp FORWARD turn is followed only when the map is trusted (SLAM /
      // partial mode). In strict live mode it is a heading stop: on a single
      // visible wall a sharp forward kink is more likely a perception artifact
      // than a real corner, so fail closed instead of driving through it.
      if (sharp_forward && !config.allow_partial_boundary) {
        saw_heading_jump = true;
        continue;
      }
      // u-turn continuation, bounded only by max_u_turn_heading_change_rad: a
      // hairpin's return-lane bridge projects BACKWARD onto the outbound
      // tangent, and the following return-lane cones sit in the 60-90 deg band
      // the gentle cap rejects. Handling both here (rather than gating on
      // projection sign, which made the u-turn cap unreachable) lets the
      // traversal round a hairpin. u-turn candidates are used only when NO
      // smooth forward candidate exists, so gentle tracks are unaffected.
      if (!uTurnContinuationAllowed(ordered.size(), points.size())) {
        if (sharp_forward) {
          saw_heading_jump = true;  // sharp forward turn with no u-turn fallback
        }
        continue;
      }
      if (heading_change > config.max_u_turn_heading_change_rad) {
        saw_heading_jump = true;
        continue;
      }
      const TraversalCandidate candidate{index, gap, heading_change, unit};
      if (!u_turn.has_value()) {
        u_turn = candidate;
      } else {
        if (closeUTurnBranch(candidate, *u_turn, config)) {
          u_turn_ambiguous = true;
        }
        if (betterUTurnCandidate(candidate, *u_turn, points)) {
          u_turn = candidate;
        }
      }
    }
    if (normal.has_value()) {
      used[normal->index] = true;
      ordered.push_back(points[normal->index]);
      tangent = normal->unit;
      if (arc_since_sharp >= 0.0) {
        arc_since_sharp += normal->gap;
      }
      continue;
    }
    if (u_turn_ambiguous) {
      return {ordered, TraversalFailure::kUTurnBranchAmbiguous};
    }
    if (u_turn.has_value()) {
      const double turn_sign = tangent.x * u_turn->unit.y - tangent.y * u_turn->unit.x;
      if (last_sharp_sign != 0.0 && turn_sign * last_sharp_sign < 0.0 &&
        arc_since_sharp >= 0.0 && arc_since_sharp <= fold_guard_arc_m)
      {
        // Opposite-sign sharp turn right after a sharp turn: this link folds
        // back across the corridor instead of rounding a hairpin. End the
        // chain here; the path simply stops where corridor knowledge ends.
        return {ordered, TraversalFailure::kFoldBack};
      }
      used[u_turn->index] = true;
      ordered.push_back(points[u_turn->index]);
      tangent = u_turn->unit;
      last_sharp_sign = turn_sign;
      arc_since_sharp = 0.0;
      continue;
    }
    return {
      ordered,
      saw_heading_jump && saw_connected ?
        TraversalFailure::kHeadingJump : TraversalFailure::kTopologyGap};
  }
  return {ordered, TraversalFailure::kNone};
}

std::vector<Point2> forwardTraversal(const std::vector<Point2> & points, const PlannerConfig & config)
{
  return forwardTraversalWithReason(points, config).points;
}

}
}
