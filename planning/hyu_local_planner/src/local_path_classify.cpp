#include "local_path_builder_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <tuple>
#include <utility>

namespace hyu_local_planner::internal
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

bool lexicographicPoint(const Point2 & first, const Point2 & second)
{
  return first.x < second.x || (first.x == second.x && first.y < second.y);
}

// Perpendicular distance from `point` to the local boundary line fitted from the
// two same-colour cones nearest `point`: the nearest must lie within `radius`
// (the cone has to be close to SOME part of that boundary), the second within
// twice that (a sparse boundary -- cones one traversal gap apart -- still
// defines a direction; a uncoloured cone one spacing past the last labelled
// cone of a straight then absorbs instead of being dropped, while on a bend
// the extrapolated chord misses it by its sagitta and it is still refused).
// Returns infinity when no such pair exists: the boundary direction is then
// unknown, so `point` cannot be judged against it.
double lateralToBoundary(
  const Point2 & point, const std::vector<Point2> & boundary, double radius)
{
  std::size_t nearest = boundary.size();
  std::size_t second = boundary.size();
  double nearest_d = std::numeric_limits<double>::infinity();
  double second_d = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < boundary.size(); ++index) {
    const double d = distance(point, boundary[index]);
    if (d > 2.0 * radius) {
      continue;
    }
    if (d < nearest_d) {
      second = nearest;
      second_d = nearest_d;
      nearest = index;
      nearest_d = d;
    } else if (d < second_d) {
      second = index;
      second_d = d;
    }
  }
  if (second == boundary.size() || nearest_d > radius) {
    return std::numeric_limits<double>::infinity();
  }
  const Point2 & anchor = boundary[nearest];
  const double tangent_x = boundary[second].x - anchor.x;
  const double tangent_y = boundary[second].y - anchor.y;
  const double norm = std::hypot(tangent_x, tangent_y);
  if (!std::isfinite(norm) || norm <= kEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  // |(point - anchor) x tangent| with tangent unit-normalised: the offset of the
  // cone from the supporting line of its two nearest same-colour neighbours.
  return std::abs((point.x - anchor.x) * tangent_y - (point.y - anchor.y) * tangent_x) / norm;
}

bool anyWithin(const Point2 & point, const std::vector<Point2> & cones, double radius)
{
  return std::any_of(
    cones.begin(), cones.end(),
    [&point, radius](const Point2 & cone) {return distance(point, cone) <= radius;});
}


// Absorb colourless cones onto the blue (left) / yellow (right) boundaries.
// Cones are taken nearest-first, and every cone that is classified joins the
// boundary it landed on before the next cone is judged, so a boundary GROWS
// outward through a run of uncoloured cones the way the traversal later walks
// it: a cone is absorbed when it lies within unknown_absorb_lateral_m of the
// local line through its two nearest same-colour cones (see
// lateralToBoundary) and of exactly one boundary -- on a straight or gentle
// bend the chord extrapolates onto the next cone, on a tight bend the
// sagitta refuses it. Where no boundary line explains the cone it is split
// by ego-frame side, but only within `split_max_range` of the ego (where
// left-of-car is left-of-track; infinite for trusted gate markers), dropping
// anything inside the centreline dead-band. Everything else is discarded so
// a colour drop-out never fabricates a boundary the geometry does not
// support.
struct ColorlessCone
{
  Point2 point;
  double split_max_range;
};

void classifyColorless(
  std::vector<Point2> & blue, std::vector<Point2> & yellow,
  std::vector<ColorlessCone> cones, const PlannerConfig & config)
{
  // Nearest-first, mirror-symmetric tie-break (range, x, |y|): the order in
  // which boundaries grow must not depend on input order or turn direction.
  std::sort(
    cones.begin(), cones.end(),
    [](const ColorlessCone & a, const ColorlessCone & b) {
      return std::make_tuple(std::hypot(a.point.x, a.point.y), a.point.x, std::abs(a.point.y)) <
             std::make_tuple(std::hypot(b.point.x, b.point.y), b.point.x, std::abs(b.point.y));
    });
  const double radius = config.max_traversal_gap_m;
  for (const ColorlessCone & entry : cones) {
    const Point2 & cone = entry.point;
    const bool blue_nearby = anyWithin(cone, blue, radius);
    const bool yellow_nearby = anyWithin(cone, yellow, radius);
    if (blue_nearby || yellow_nearby) {
      const bool blue_ok =
        lateralToBoundary(cone, blue, radius) <= config.unknown_absorb_lateral_m;
      const bool yellow_ok =
        lateralToBoundary(cone, yellow, radius) <= config.unknown_absorb_lateral_m;
      if (blue_ok && !yellow_ok) {
        blue.push_back(cone);
        continue;
      }
      if (yellow_ok && !blue_ok) {
        yellow.push_back(cone);
        continue;
      }
      if (blue_ok && yellow_ok) {
        // Both fitted boundary lines explain the cone: genuinely ambiguous -> drop.
        continue;
      }
      // Neither boundary line fits. This is dominated by a boundary too SPARSE to
      // define a line (needs >=2 same-colour cones): on the skidpad tight circle
      // the inner/outer cones fall outside the narrow camera FOV, arrive
      // uncoloured, and used to be dropped here -> a starved boundary the path
      // could not curve around. Fall through to the ego-frame side split so the
      // boundary still gets its lidar cones; the dead-band still drops cones too
      // central to call.
    }
    // Ego-side split. Only near the car: further out the ego frame's left/right
    // no longer follows the track, and a far uncoloured cone on the outside of
    // a bend would be handed to the wrong wall (see
    // PlannerConfig::unknown_geom_max_range_m).
    if (std::hypot(cone.x, cone.y) > entry.split_max_range) {
      continue;
    }
    if (cone.y >= config.unknown_geom_deadband_m) {
      blue.push_back(cone);
    } else if (cone.y <= -config.unknown_geom_deadband_m) {
      yellow.push_back(cone);
    }
  }
  // Restore the lexicographic ordering cropToRoi established before the append.
  std::sort(blue.begin(), blue.end(), lexicographicPoint);
  std::sort(yellow.begin(), yellow.end(), lexicographicPoint);
}

}

void classifyUnknownCones(
  std::vector<Point2> & blue, std::vector<Point2> & yellow,
  const std::vector<Point2> & unknown, const PlannerConfig & config)
{
  classifyColorlessCones(blue, yellow, unknown, {}, config);
}

void classifyColorlessCones(
  std::vector<Point2> & blue, std::vector<Point2> & yellow,
  const std::vector<Point2> & unknown, const std::vector<Point2> & markers,
  const PlannerConfig & config)
{
  if (unknown.empty() && markers.empty()) {
    return;
  }
  std::vector<ColorlessCone> cones;
  cones.reserve(unknown.size() + markers.size());
  for (const Point2 & cone : unknown) {
    cones.push_back({cone, config.unknown_geom_max_range_m});
  }
  for (const Point2 & cone : markers) {
    cones.push_back({cone, std::numeric_limits<double>::infinity()});
  }
  classifyColorless(blue, yellow, std::move(cones), config);
}

}
