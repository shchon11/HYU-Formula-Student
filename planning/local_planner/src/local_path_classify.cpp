#include "local_path_builder_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace local_planner::internal
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

bool lexicographicPoint(const Point2 & first, const Point2 & second)
{
  return first.x < second.x || (first.x == second.x && first.y < second.y);
}

// Perpendicular distance from `point` to the local boundary line fitted from the
// two same-colour cones nearest `point` within `radius`. Returns infinity when
// fewer than two boundary cones lie within radius: the boundary direction is
// then unknown, so `point` cannot be judged against it.
double lateralToBoundary(
  const Point2 & point, const std::vector<Point2> & boundary, double radius)
{
  std::size_t nearest = boundary.size();
  std::size_t second = boundary.size();
  double nearest_d = std::numeric_limits<double>::infinity();
  double second_d = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < boundary.size(); ++index) {
    const double d = distance(point, boundary[index]);
    if (d > radius) {
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
  if (second == boundary.size()) {
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

}

// Absorb unknown-colour cones onto the blue (left) / yellow (right) boundaries.
// Each cone is fitted against the ORIGINAL labelled sets so the result is
// independent of processing order. A cone is only taken when its side is
// unambiguous: it must lie within unknown_absorb_lateral_m of exactly one
// boundary's local line. Where no labelled boundary is within reach the cone is
// split by ego-frame side, dropping anything too near the centreline. Everything
// else is discarded so a colour drop-out never fabricates a boundary the
// geometry does not support.
void classifyUnknownCones(
  std::vector<Point2> & blue, std::vector<Point2> & yellow,
  const std::vector<Point2> & unknown, const PlannerConfig & config)
{
  if (unknown.empty()) {
    return;
  }
  const std::vector<Point2> blue_ref = blue;
  const std::vector<Point2> yellow_ref = yellow;
  const double radius = config.max_traversal_gap_m;
  for (const Point2 & cone : unknown) {
    const bool blue_nearby = anyWithin(cone, blue_ref, radius);
    const bool yellow_nearby = anyWithin(cone, yellow_ref, radius);
    if (blue_nearby || yellow_nearby) {
      const bool blue_ok =
        lateralToBoundary(cone, blue_ref, radius) <= config.unknown_absorb_lateral_m;
      const bool yellow_ok =
        lateralToBoundary(cone, yellow_ref, radius) <= config.unknown_absorb_lateral_m;
      if (blue_ok && !yellow_ok) {
        blue.push_back(cone);
      } else if (yellow_ok && !blue_ok) {
        yellow.push_back(cone);
      }
      // Both boundary lines explain the cone, or neither does: ambiguous -> drop.
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
