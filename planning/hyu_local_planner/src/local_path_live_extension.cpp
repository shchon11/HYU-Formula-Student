#include "hyu_local_planner/local_path_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>

#include "local_path_builder_internal.hpp"

namespace hyu_local_planner
{
namespace
{

constexpr std::size_t kMinWaypoints = 5U;

void setNote(std::string * note, const std::string & text)
{
  if (note) {
    *note = text;
  }
}

// Distance from `point` to the polyline through `path` (segment-wise).
double distanceToPath(const Point2 & point, const std::vector<PathWaypoint> & path)
{
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const Point2 a{path[i].x, path[i].y};
    if (i + 1U < path.size()) {
      const Point2 b{path[i + 1U].x, path[i + 1U].y};
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double len_sq = dx * dx + dy * dy;
      if (len_sq > 1.0e-12) {
        double t = ((point.x - a.x) * dx + (point.y - a.y) * dy) / len_sq;
        t = std::clamp(t, 0.0, 1.0);
        best = std::min(best, std::hypot(point.x - (a.x + t * dx), point.y - (a.y + t * dy)));
        continue;
      }
    }
    best = std::min(best, std::hypot(point.x - a.x, point.y - a.y));
  }
  return best;
}

// Index of the first waypoint at or after `from` whose heading differs from
// the heading `window` metres of arc earlier by more than `max_turn`, or
// npos when the path is smooth from `from` to its end.
std::size_t firstKink(
  const std::vector<PathWaypoint> & path, std::size_t from, double window, double max_turn)
{
  for (std::size_t i = std::max<std::size_t>(from, 1U); i < path.size(); ++i) {
    // Reference heading: the last waypoint at least `window` of arc behind i
    // (or the first waypoint when the path is shorter than the window).
    std::size_t j = i - 1U;
    while (j > 0U && path[i].s - path[j].s < window) {
      --j;
    }
    const double turn = std::abs(internal::normalizeAngle(path[i].psi - path[j].psi));
    if (turn > max_turn) {
      return i;
    }
  }
  return std::numeric_limits<std::size_t>::max();
}

// Cut `path` so that it ends before waypoint `kink`. Positions, arc lengths
// and headings of the surviving waypoints are unchanged; the curvature of
// the new final waypoint is recomputed from its predecessor.
void truncateAt(std::vector<PathWaypoint> & path, std::size_t kink)
{
  if (kink >= path.size()) {
    return;
  }
  path.resize(kink);
  if (path.size() >= 2U) {
    auto & last = path.back();
    const auto & prev = path[path.size() - 2U];
    const double delta_s = last.s - prev.s;
    last.psi = prev.psi;
    last.kappa = delta_s > 0.0 ? internal::normalizeAngle(last.psi - prev.psi) / delta_s : 0.0;
  }
}

}  // namespace

BuildResult reconcileLiveExtension(
  const BuildResult & map_only, const BuildResult & extended,
  const PlannerConfig & config, std::string * note)
{
  if (!extended.valid || extended.waypoints.size() < 2U) {
    setNote(note, "extended path invalid (" + extended.reason + "); map-only path kept");
    return map_only;
  }

  const double window = config.live_extension_turn_window_m;
  const double max_turn = config.live_extension_max_turn_rad;

  if (!map_only.valid || map_only.waypoints.empty()) {
    // No trusted reference: the extended path stands alone, but only up to
    // its first kink. A live outlier at the frontier must not steer the car
    // into a hook when the map cannot vouch for anything.
    BuildResult result = extended;
    const std::size_t kink = firstKink(result.waypoints, 1U, window, max_turn);
    if (kink != std::numeric_limits<std::size_t>::max()) {
      truncateAt(result.waypoints, kink);
    }
    if (result.waypoints.size() < kMinWaypoints) {
      BuildResult invalid;
      invalid.evaluated = true;
      invalid.valid = false;
      invalid.kind = PathKind::kNone;
      std::ostringstream reason;
      reason << "live_extension_rejected: no map path and the live path kinks within "
             << (result.waypoints.empty() ? 0.0 : result.waypoints.back().s) << " m";
      invalid.reason = reason.str();
      setNote(note, invalid.reason);
      return invalid;
    }
    std::ostringstream text;
    text << "no map path; live path "
         << (kink == std::numeric_limits<std::size_t>::max() ? "accepted whole" : "truncated at kink")
         << " (" << result.waypoints.back().s << " m)";
    setNote(note, text.str());
    return result;
  }

  const double map_length = map_only.waypoints.back().s;
  const double ext_length = extended.waypoints.back().s;
  if (ext_length <= map_length + 0.5 * config.waypoint_spacing_m) {
    setNote(note, "extended path adds no length; map-only path kept");
    return map_only;
  }

  // The mapped part of the path must be untouched: every map-only waypoint
  // has to lie on (near) the extended path.
  for (const auto & waypoint : map_only.waypoints) {
    const double deviation =
      distanceToPath({waypoint.x, waypoint.y}, extended.waypoints);
    if (deviation > config.live_extension_max_deviation_m) {
      std::ostringstream text;
      text << "extended path deviates " << deviation << " m from the map-only path at s="
           << waypoint.s << "; map-only path kept";
      setNote(note, text.str());
      return map_only;
    }
  }

  // Past the map frontier the extended path may only continue smoothly.
  BuildResult result = extended;
  std::size_t tail_start = 0U;
  while (tail_start < result.waypoints.size() &&
    result.waypoints[tail_start].s <= map_length - 0.5 * config.waypoint_spacing_m)
  {
    ++tail_start;
  }
  const std::size_t kink = firstKink(result.waypoints, tail_start, window, max_turn);
  if (kink != std::numeric_limits<std::size_t>::max()) {
    truncateAt(result.waypoints, kink);
  }
  if (result.waypoints.size() < kMinWaypoints ||
    result.waypoints.back().s <= map_length + 0.5 * config.waypoint_spacing_m)
  {
    std::ostringstream text;
    text << "extended tail kinks at s=" << extended.waypoints[std::min(kink, extended.waypoints.size() - 1U)].s
         << " (map path " << map_length << " m); map-only path kept";
    setNote(note, text.str());
    return map_only;
  }
  std::ostringstream text;
  text << "map path " << map_length << " m extended to " << result.waypoints.back().s << " m"
       << (kink == std::numeric_limits<std::size_t>::max() ? "" : " (tail truncated at kink)");
  setNote(note, text.str());
  return result;
}

BuildResult planWithLiveExtension(
  const ConeSet & map_only_cones, const ConeSet * extended,
  const PlannerConfig & config, std::string * note)
{
  BuildResult map_only = buildLocalPath(map_only_cones, config);
  if (extended == nullptr) {
    setNote(note, "");
    return map_only;
  }
  const BuildResult grown = buildLocalPath(*extended, config);
  return reconcileLiveExtension(map_only, grown, config, note);
}

}  // namespace hyu_local_planner
