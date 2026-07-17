#include "hyu_local_planner/local_path_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace hyu_local_planner
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

double cross(const Point2 & first, const Point2 & second, const Point2 & third)
{
  return (second.x - first.x) * (third.y - first.y) -
         (second.y - first.y) * (third.x - first.x);
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
         (second_orientation == 0 && pointOnSegment(first_a, first_b, second_b)) ||
         (third_orientation == 0 && pointOnSegment(second_a, second_b, first_a)) ||
         (fourth_orientation == 0 && pointOnSegment(second_a, second_b, first_b));
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
