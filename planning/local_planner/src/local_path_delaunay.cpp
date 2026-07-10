#include "local_path_builder_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <tuple>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace local_planner::internal
{
namespace
{

struct TaggedPoint
{
  Point2 point;
  BoundarySide side;
};

std::optional<std::size_t> nearestTaggedPoint(
  const Point2 & endpoint, const std::vector<TaggedPoint> & points, double tolerance)
{
  std::optional<std::size_t> best_index;
  std::tuple<double, double, double, int> best_key;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const double gap = distance(endpoint, points[index].point);
    if (gap > tolerance) {
      continue;
    }
    const auto key = std::make_tuple(
      gap, points[index].point.x, points[index].point.y,
      points[index].side == BoundarySide::kBlue ? 0 : 1);
    if (!best_index.has_value() || key < best_key) {
      best_index = index;
      best_key = key;
    }
  }
  return best_index;
}

}

std::vector<Point2> delaunayMidpoints(
  const std::vector<Point2> & blue, const std::vector<Point2> & yellow,
  const PlannerConfig & config)
{
  std::vector<TaggedPoint> tagged;
  tagged.reserve(blue.size() + yellow.size());
  for (const auto & point : blue) {
    tagged.push_back({point, BoundarySide::kBlue});
  }
  for (const auto & point : yellow) {
    tagged.push_back({point, BoundarySide::kYellow});
  }
  std::sort(
    tagged.begin(), tagged.end(),
    [](const TaggedPoint & first, const TaggedPoint & second) {
      return std::tie(first.point.x, first.point.y, first.side) <
             std::tie(second.point.x, second.point.y, second.side);
    });

  std::vector<Point2> insertion_points;
  insertion_points.reserve(tagged.size());
  for (const auto & item : tagged) {
    insertion_points.push_back(item.point);
  }
  insertion_points = deduplicate(std::move(insertion_points), 0.0, true);

  const int left = static_cast<int>(std::floor(config.roi_min_x)) - 1;
  const int top = static_cast<int>(std::floor(-config.roi_abs_y)) - 1;
  const int right = static_cast<int>(std::ceil(config.roi_max_x)) + 2;
  const int bottom = static_cast<int>(std::ceil(config.roi_abs_y)) + 2;
  cv::Subdiv2D subdivision(cv::Rect(left, top, right - left, bottom - top));
  for (const auto & point : insertion_points) {
    subdivision.insert(cv::Point2f(static_cast<float>(point.x), static_cast<float>(point.y)));
  }

  std::vector<cv::Vec4f> edges;
  subdivision.getEdgeList(edges);
  std::vector<Point2> midpoints;
  for (const auto & edge : edges) {
    const Point2 first{edge[0], edge[1]};
    const Point2 second{edge[2], edge[3]};
    const auto first_index = nearestTaggedPoint(first, tagged, config.endpoint_match_tolerance_m);
    const auto second_index = nearestTaggedPoint(second, tagged, config.endpoint_match_tolerance_m);
    if (!first_index.has_value() || !second_index.has_value() || *first_index == *second_index) {
      continue;
    }
    const auto & first_cone = tagged[*first_index];
    const auto & second_cone = tagged[*second_index];
    if (first_cone.side == second_cone.side) {
      continue;
    }
    const double width = distance(first_cone.point, second_cone.point);
    if (width < config.min_track_width_m || width > config.max_track_width_m) {
      continue;
    }
    midpoints.push_back({
      0.5 * (first_cone.point.x + second_cone.point.x),
      0.5 * (first_cone.point.y + second_cone.point.y),
    });
  }
  return deduplicate(std::move(midpoints), config.duplicate_tolerance_m);
}

}
