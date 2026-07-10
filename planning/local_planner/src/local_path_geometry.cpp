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

double cross(const Point2 & first, const Point2 & second, const Point2 & third)
{
  return (second.x - first.x) * (third.y - first.y) -
         (second.y - first.y) * (third.x - first.x);
}

bool finitePoint(const Point2 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool lexicographicPoint(const Point2 & first, const Point2 & second)
{
  return std::tie(first.x, first.y) < std::tie(second.x, second.y);
}

bool pointOnSegment(const Point2 & first, const Point2 & second, const Point2 & point)
{
  return std::abs(cross(first, second, point)) <= kEpsilon &&
         point.x >= std::min(first.x, second.x) - kEpsilon &&
         point.x <= std::max(first.x, second.x) + kEpsilon &&
         point.y >= std::min(first.y, second.y) - kEpsilon &&
         point.y <= std::max(first.y, second.y) + kEpsilon;
}

int orientation(const Point2 & first, const Point2 & second, const Point2 & third)
{
  const double value = cross(first, second, third);
  return value > kEpsilon ? 1 : value < -kEpsilon ? -1 : 0;
}

bool segmentsIntersect(
  const Point2 & first_a, const Point2 & first_b,
  const Point2 & second_a, const Point2 & second_b)
{
  const int first_orientation = orientation(first_a, first_b, second_a);
  const int second_orientation = orientation(first_a, first_b, second_b);
  const int third_orientation = orientation(second_a, second_b, first_a);
  const int fourth_orientation = orientation(second_a, second_b, first_b);
  if (first_orientation != second_orientation && third_orientation != fourth_orientation) {
    return true;
  }
  return (first_orientation == 0 && pointOnSegment(first_a, first_b, second_a)) ||
         (second_orientation == 0 && pointOnSegment(first_a, first_b, second_a)) ||
         (third_orientation == 0 && pointOnSegment(second_a, second_b, first_a)) ||
         (fourth_orientation == 0 && pointOnSegment(second_a, second_b, first_b));
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

std::vector<Point2> forwardTraversal(const std::vector<Point2> & points, const PlannerConfig & config)
{
  if (points.empty()) {
    return {};
  }
  std::optional<std::size_t> seed;
  std::tuple<double, double, double, double> seed_key;
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (points[index].x < 0.0) {
      continue;
    }
    const auto key = std::make_tuple(
      std::hypot(points[index].x, points[index].y), points[index].x,
      std::abs(points[index].y), points[index].y);
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
  while (ordered.size() < points.size()) {
    std::optional<std::size_t> next;
    std::tuple<double, double, double, double, std::size_t> next_key;
    Point2 next_tangent;
    for (std::size_t index = 0; index < points.size(); ++index) {
      if (used[index]) {
        continue;
      }
      const Point2 segment = subtract(points[index], ordered.back());
      const double gap = std::hypot(segment.x, segment.y);
      if (!std::isfinite(gap) || gap <= kEpsilon || gap > config.max_traversal_gap_m) {
        continue;
      }
      if (!(dot(segment, tangent) > config.min_forward_projection_m)) {
        continue;
      }
      const Point2 unit{segment.x / gap, segment.y / gap};
      const double heading_change = std::acos(std::clamp(dot(unit, tangent), -1.0, 1.0));
      if (!std::isfinite(heading_change) || heading_change > config.max_heading_change_rad) {
        continue;
      }
      const auto key = std::make_tuple(gap, heading_change, points[index].x, points[index].y, index);
      if (!next.has_value() || key < next_key) {
        next = index;
        next_key = key;
        next_tangent = unit;
      }
    }
    if (!next.has_value()) {
      break;
    }
    used[*next] = true;
    ordered.push_back(points[*next]);
    tangent = next_tangent;
  }
  return ordered;
}

}

bool pathSelfIntersects(const std::vector<Point2> & points)
{
  if (points.size() < 4U) {
    return false;
  }
  for (std::size_t first = 0; first + 1U < points.size(); ++first) {
    for (std::size_t second = first + 2U; second + 1U < points.size(); ++second) {
      if (segmentsIntersect(
          points[first], points[first + 1U], points[second], points[second + 1U]))
      {
        return true;
      }
    }
  }
  return false;
}

}
