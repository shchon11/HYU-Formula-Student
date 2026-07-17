#include "local_path_builder_internal.hpp"

#include <cmath>
#include <limits>
#include <optional>

namespace hyu_local_planner::internal
{
namespace
{

// A real track-width pair spans the EMPTY driving corridor; a pair that
// bridges to a parallel corridor (a peanut/hairpin waist) passes near the
// intervening walls. So the midpoint of a valid pair must be clear of every
// other cone. The two paired cones sit at width/2 from the midpoint, well
// outside this clearance, so they never trigger the reject themselves.
bool midpointClearOfCones(
  const Point2 & midpoint, double clearance,
  const std::vector<Point2> & blue, const std::vector<Point2> & yellow)
{
  const auto clear = [&midpoint, clearance](const std::vector<Point2> & cones) {
      for (const auto & cone : cones) {
        if (distance(midpoint, cone) < clearance) {
          return false;
        }
      }
      return true;
    };
  return clear(blue) && clear(yellow);
}

// Nearest opposite-colour cone a plausible track width away WHOSE corridor is
// clear (see midpointClearOfCones). Connects a cone across its own corridor,
// never across the gap to a parallel one.
std::optional<Point2> nearestCrossPair(
  const Point2 & from, const std::vector<Point2> & others,
  const std::vector<Point2> & blue, const std::vector<Point2> & yellow,
  const PlannerConfig & config)
{
  std::optional<Point2> best;
  double best_width = std::numeric_limits<double>::infinity();
  for (const auto & other : others) {
    const double width = distance(from, other);
    if (!std::isfinite(width) || width < config.min_track_width_m ||
      width > config.max_track_width_m || width >= best_width)
    {
      continue;
    }
    const Point2 midpoint{0.5 * (from.x + other.x), 0.5 * (from.y + other.y)};
    // Clearance stays below width/2 so the two paired cones never self-reject,
    // but above the offset of an intervening wall in the parallel corridor.
    if (!midpointClearOfCones(midpoint, 0.65 * 0.5 * width, blue, yellow)) {
      continue;
    }
    best_width = width;
    best = other;
  }
  return best;
}

}

// Two-sided centerline via width-gated cross pairing (a Delaunay-style
// midline, robust on folded tracks). Every blue cone is paired with its
// nearest yellow a track-width away and vice-versa; the midpoints are the
// centerline candidates. They are then chained forward from the ego with a
// tight link gap, so on a peanut/hairpin where the ROI box also contains the
// folded-back passage, only the corridor DIRECTLY AHEAD is followed — the
// old arc-length-ratio pairing bridged the two corridors and produced garbage.
std::vector<Point2> boundaryMidpoints(
  const std::vector<Point2> & blue, const std::vector<Point2> & yellow,
  const PlannerConfig & config)
{
  if (blue.empty() || yellow.empty()) {
    return {};
  }
  std::vector<Point2> midpoints;
  midpoints.reserve(blue.size() + yellow.size());
  std::size_t blue_paired = 0U;
  std::size_t yellow_paired = 0U;
  for (const auto & b : blue) {
    if (const auto y = nearestCrossPair(b, yellow, blue, yellow, config)) {
      midpoints.push_back({0.5 * (b.x + y->x), 0.5 * (b.y + y->y)});
      ++blue_paired;
    }
  }
  for (const auto & y : yellow) {
    if (const auto b = nearestCrossPair(y, blue, blue, yellow, config)) {
      midpoints.push_back({0.5 * (b->x + y.x), 0.5 * (b->y + y.y)});
      ++yellow_paired;
    }
  }
  // Strict mode (live perception): a cone with no cross-partner a track-width
  // away signals a gap/outlier in the boundary — refuse rather than plan
  // through it. slam_map mode tolerates outliers and plans the good corridor.
  if (!config.allow_partial_boundary &&
    (blue_paired != blue.size() || yellow_paired != yellow.size()))
  {
    return {};
  }
  midpoints = deduplicate(
    std::move(midpoints), std::max(config.duplicate_tolerance_m, 0.5 * config.waypoint_spacing_m));
  if (midpoints.size() < 2U) {
    return {};
  }

  // Chain the midpoints forward from the ego with a link gap below the
  // smallest inter-corridor spacing so a folded-back passage cannot join in.
  PlannerConfig chain = config;
  chain.max_traversal_gap_m =
    std::min(config.max_traversal_gap_m, config.centerline_max_link_gap_m);
  const auto ordered = forwardTraversal(midpoints, chain);
  if (ordered.size() < 2U) {
    return {};
  }
  return ordered;
}

}
