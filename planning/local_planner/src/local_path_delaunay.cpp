#include "local_path_builder_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <tuple>

namespace local_planner::internal
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

double dot(const Point2 & first, const Point2 & second)
{
  return first.x * second.x + first.y * second.y;
}

Point2 subtract(const Point2 & first, const Point2 & second)
{
  return {first.x - second.x, first.y - second.y};
}

std::optional<double> median(std::vector<double> values)
{
  if (values.empty()) {
    return std::nullopt;
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2U == 1U) {
    return *middle;
  }
  const double upper = *middle;
  const auto lower = std::max_element(values.begin(), middle);
  return 0.5 * (*lower + upper);
}

std::vector<double> cumulativeArc(const std::vector<Point2> & points)
{
  std::vector<double> arc(points.size(), 0.0);
  for (std::size_t index = 1; index < points.size(); ++index) {
    const double segment = distance(points[index - 1U], points[index]);
    if (!std::isfinite(segment) || segment <= kEpsilon) {
      return {};
    }
    arc[index] = arc[index - 1U] + segment;
  }
  return arc;
}

Point2 sampleAtArc(
  const std::vector<Point2> & points, const std::vector<double> & arc, double target)
{
  if (target <= 0.0) {
    return points.front();
  }
  if (target >= arc.back()) {
    return points.back();
  }
  const auto upper = std::upper_bound(arc.begin(), arc.end(), target);
  const auto index = static_cast<std::size_t>(upper - arc.begin());
  const double ratio = (target - arc[index - 1U]) / (arc[index] - arc[index - 1U]);
  return {
    points[index - 1U].x + ratio * (points[index].x - points[index - 1U].x),
    points[index - 1U].y + ratio * (points[index].y - points[index - 1U].y),
  };
}

std::vector<Point2> normalizedSamples(
  const std::vector<Point2> & points, std::size_t count)
{
  const auto arc = cumulativeArc(points);
  if (arc.size() != points.size() || arc.empty() || count == 0U) {
    return {};
  }
  std::vector<Point2> samples;
  samples.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const double ratio = count == 1U ? 0.0 :
      static_cast<double>(index) / static_cast<double>(count - 1U);
    samples.push_back(sampleAtArc(points, arc, ratio * arc.back()));
  }
  return samples;
}

Point2 closestPointOnPolyline(const Point2 & query, const std::vector<Point2> & polyline)
{
  Point2 best = polyline.front();
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index + 1U < polyline.size(); ++index) {
    const Point2 segment = subtract(polyline[index + 1U], polyline[index]);
    const double length_squared = dot(segment, segment);
    const Point2 offset = subtract(query, polyline[index]);
    const double ratio = length_squared > kEpsilon ?
      std::clamp(dot(offset, segment) / length_squared, 0.0, 1.0) : 0.0;
    const Point2 projection{
      polyline[index].x + ratio * segment.x,
      polyline[index].y + ratio * segment.y,
    };
    const double candidate_distance = distance(query, projection);
    if (candidate_distance < best_distance) {
      best_distance = candidate_distance;
      best = projection;
    }
  }
  return best;
}

std::vector<Point2> orderBoundary(
  const std::vector<Point2> & input, const PlannerConfig & config)
{
  const auto points = deduplicate(input, config.duplicate_tolerance_m);
  if (points.size() < 2U) {
    return {};
  }

  std::size_t current = 0U;
  double start_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < points.size(); ++index) {
    const double candidate_distance = std::hypot(points[index].x, points[index].y);
    const auto key = std::make_tuple(
      candidate_distance, -points[index].x, std::abs(points[index].y), points[index].y);
    const auto best_key = std::make_tuple(
      start_distance, -points[current].x, std::abs(points[current].y), points[current].y);
    if (candidate_distance < start_distance || key < best_key) {
      start_distance = candidate_distance;
      current = index;
    }
  }

  std::vector<bool> used(points.size(), false);
  std::vector<Point2> ordered;
  ordered.reserve(points.size());
  ordered.push_back(points[current]);
  used[current] = true;
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
      const double forward_projection = dot(segment, tangent);
      if (!(forward_projection > config.min_forward_projection_m)) {
        continue;
      }
      const Point2 unit{segment.x / gap, segment.y / gap};
      const double heading_change = std::acos(std::clamp(dot(unit, tangent), -1.0, 1.0));
      if (!std::isfinite(heading_change) || heading_change > config.max_heading_change_rad) {
        continue;
      }
      const auto key = std::make_tuple(
        heading_change, gap, -points[index].x, std::abs(points[index].y), index);
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

bool sameDirection(
  const std::vector<Point2> & first, const std::vector<Point2> & second)
{
  const auto first_samples = normalizedSamples(first, std::min(first.size(), second.size()));
  const auto second_samples = normalizedSamples(second, first_samples.size());
  if (first_samples.size() < 2U || second_samples.size() != first_samples.size()) {
    return false;
  }
  std::vector<double> direction_dots;
  direction_dots.reserve(first_samples.size() - 1U);
  for (std::size_t index = 1; index < first_samples.size(); ++index) {
    const Point2 first_segment = subtract(first_samples[index], first_samples[index - 1U]);
    const Point2 second_segment = subtract(second_samples[index], second_samples[index - 1U]);
    const double first_length = std::hypot(first_segment.x, first_segment.y);
    const double second_length = std::hypot(second_segment.x, second_segment.y);
    if (first_length > kEpsilon && second_length > kEpsilon) {
      direction_dots.push_back(dot(first_segment, second_segment) / (first_length * second_length));
    }
  }
  const auto direction = median(direction_dots);
  return direction.has_value() && direction.value() > 0.0;
}

}

std::vector<Point2> boundaryMidpoints(
  const std::vector<Point2> & blue, const std::vector<Point2> & yellow,
  const PlannerConfig & config)
{
  const auto ordered_blue = orderBoundary(blue, config);
  const auto ordered_yellow = orderBoundary(yellow, config);
  if (ordered_blue.size() < 2U || ordered_yellow.size() < 2U ||
    !sameDirection(ordered_blue, ordered_yellow))
  {
    return {};
  }
  if (!config.allow_partial_boundary &&
    (ordered_blue.size() != deduplicate(blue, config.duplicate_tolerance_m).size() ||
    ordered_yellow.size() != deduplicate(yellow, config.duplicate_tolerance_m).size()))
  {
    return {};
  }

  const auto blue_arc = cumulativeArc(ordered_blue);
  if (blue_arc.size() != ordered_blue.size() || blue_arc.back() <= kEpsilon) {
    return {};
  }
  const std::size_t sample_count = std::max<std::size_t>(
    2U, static_cast<std::size_t>(std::ceil(blue_arc.back() / config.waypoint_spacing_m)) + 1U);

  std::vector<Point2> centerline;
  centerline.reserve(sample_count);
  for (std::size_t index = 0; index < sample_count; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(sample_count - 1U);
    const auto blue_sample = sampleAtArc(ordered_blue, blue_arc, ratio * blue_arc.back());
    const auto yellow_sample = closestPointOnPolyline(blue_sample, ordered_yellow);
    const double width = distance(blue_sample, yellow_sample);
    if (!std::isfinite(width) || width < config.min_track_width_m ||
      width > config.max_track_width_m)
    {
      return {};
    }
    const Point2 midpoint{
      0.5 * (blue_sample.x + yellow_sample.x),
      0.5 * (blue_sample.y + yellow_sample.y),
    };
    if (centerline.empty() || distance(centerline.back(), midpoint) > config.duplicate_tolerance_m) {
      centerline.push_back(midpoint);
    }
  }
  if (centerline.size() < 2U) {
    return {};
  }
  return centerline;
}

}
