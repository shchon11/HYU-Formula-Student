#include "global_planner/slam_boundary_ordering.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>

namespace global_planner
{
namespace
{

constexpr double kGeometryEpsilon = 1.0e-9;

bool segmentsCross(
  const PlannerPoint & a, const PlannerPoint & b,
  const PlannerPoint & c, const PlannerPoint & d)
{
  const double ab_c = cross2d(subtract(b, a), subtract(c, a));
  const double ab_d = cross2d(subtract(b, a), subtract(d, a));
  const double cd_a = cross2d(subtract(d, c), subtract(a, c));
  const double cd_b = cross2d(subtract(d, c), subtract(b, c));
  return ab_c * ab_d < -kGeometryEpsilon && cd_a * cd_b < -kGeometryEpsilon;
}

std::size_t selfIntersectionCount(const std::vector<PlannerPoint> & points)
{
  std::size_t count = 0U;
  if (points.size() < 4U) {
    return count;
  }
  for (std::size_t i = 0; i + 1U < points.size(); ++i) {
    for (std::size_t j = i + 2U; j + 1U < points.size(); ++j) {
      if (segmentsCross(points[i], points[i + 1U], points[j], points[j + 1U])) {
        ++count;
      }
    }
  }
  return count;
}

double maxSegmentGap(const std::vector<PlannerPoint> & points)
{
  double max_gap = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    max_gap = std::max(max_gap, distance(points[i - 1U], points[i]));
  }
  return max_gap;
}

struct BoundaryOrderScore
{
  std::size_t self_intersections{0U};
  double max_gap{0.0};
  double loop_gap{0.0};
};

BoundaryOrderScore scoreBoundaryOrder(const std::vector<PlannerPoint> & points)
{
  BoundaryOrderScore score;
  score.self_intersections = selfIntersectionCount(points);
  score.max_gap = maxSegmentGap(points);
  if (points.size() >= 2U) {
    score.loop_gap = distance(points.front(), points.back());
  }
  return score;
}

bool betterBoundaryOrder(
  const BoundaryOrderScore & candidate,
  const BoundaryOrderScore & current)
{
  if (candidate.self_intersections != current.self_intersections) {
    return candidate.self_intersections < current.self_intersections;
  }
  if (std::abs(candidate.max_gap - current.max_gap) > kGeometryEpsilon) {
    return candidate.max_gap < current.max_gap;
  }
  return candidate.loop_gap < current.loop_gap;
}

double turnAngle(
  const PlannerPoint & previous,
  const PlannerPoint & current,
  const PlannerPoint & candidate)
{
  const auto incoming = subtract(current, previous);
  const auto outgoing = subtract(candidate, current);
  const double incoming_length = std::hypot(incoming.x, incoming.y);
  const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
  if (incoming_length <= kGeometryEpsilon || outgoing_length <= kGeometryEpsilon) {
    return 0.0;
  }
  const double cosine = std::clamp(
    dot(incoming, outgoing) / (incoming_length * outgoing_length), -1.0, 1.0);
  return std::acos(cosine);
}

bool headingAwareGraphOrder(
  const std::vector<PlannerPoint> & input, const PlannerPoint & ego,
  double max_gap, std::vector<PlannerPoint> & ordered, std::string & reason)
{
  ordered.clear();
  if (input.empty()) {
    reason = "no boundary points";
    return false;
  }

  std::vector<bool> used(input.size(), false);
  std::size_t current = 0U;
  double best_start_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < input.size(); ++i) {
    const double candidate_distance = distance(input[i], ego);
    if (candidate_distance < best_start_distance) {
      best_start_distance = candidate_distance;
      current = i;
    }
  }

  ordered.reserve(input.size());
  used[current] = true;
  ordered.push_back(input[current]);
  for (std::size_t count = 1; count < input.size(); ++count) {
    std::size_t next = input.size();
    double best_score = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < input.size(); ++i) {
      const double candidate_distance = distance(input[current], input[i]);
      if (used[i] || candidate_distance > max_gap) {
        continue;
      }
      double score = candidate_distance;
      if (ordered.size() >= 2U) {
        score += max_gap * turnAngle(ordered[ordered.size() - 2U], ordered.back(), input[i]);
      }
      if (score < best_score) {
        best_score = score;
        next = i;
      }
    }
    if (next == input.size()) {
      reason = "branch_jump";
      return false;
    }
    used[next] = true;
    ordered.push_back(input[next]);
    current = next;
  }
  return true;
}

bool orderBoundary(
  const std::vector<PlannerPoint> & input, const PlannerPoint & ego,
  double max_gap, std::vector<PlannerPoint> & ordered, std::string & reason)
{
  if (!headingAwareGraphOrder(input, ego, max_gap, ordered, reason)) {
    return false;
  }
  auto best_score = scoreBoundaryOrder(ordered);
  const auto input_score = scoreBoundaryOrder(input);
  if (input_score.max_gap <= max_gap && betterBoundaryOrder(input_score, best_score)) {
    ordered = input;
    best_score = input_score;
  }

  if (best_score.self_intersections > 0U) {
    reason = "self_intersection";
    return false;
  }
  if (best_score.max_gap > max_gap || best_score.loop_gap > max_gap) {
    reason = "branch_jump";
    return false;
  }
  return true;
}

bool alignBoundaryDirections(std::vector<PlannerPoint> & blue, std::vector<PlannerPoint> & yellow)
{
  const std::size_t sample_count = std::min(blue.size(), yellow.size());
  if (sample_count < 2U) {
    return false;
  }
  const auto blue_samples = sampleNormalized(blue, sample_count);
  const auto yellow_samples = sampleNormalized(yellow, sample_count);
  std::vector<double> direction_dots;
  direction_dots.reserve(sample_count - 1U);
  for (std::size_t i = 1; i < sample_count; ++i) {
    const auto blue_segment = subtract(blue_samples[i], blue_samples[i - 1]);
    const auto yellow_segment = subtract(yellow_samples[i], yellow_samples[i - 1]);
    const double blue_length = std::hypot(blue_segment.x, blue_segment.y);
    const double yellow_length = std::hypot(yellow_segment.x, yellow_segment.y);
    if (blue_length > kGeometryEpsilon && yellow_length > kGeometryEpsilon) {
      direction_dots.push_back(dot(blue_segment, yellow_segment) / (blue_length * yellow_length));
    }
  }
  const auto direction_dot_median = median(direction_dots);
  if (!direction_dot_median.has_value()) {
    return false;
  }
  if (direction_dot_median.value() < 0.0) {
    std::reverse(yellow.begin(), yellow.end());
  }
  return true;
}

std::optional<double> medianTrackSideSign(
  const std::vector<PlannerPoint> & blue, const std::vector<PlannerPoint> & yellow)
{
  const std::size_t sample_count = std::min(blue.size(), yellow.size());
  if (sample_count < 2U) {
    return std::nullopt;
  }
  const auto blue_samples = sampleNormalized(blue, sample_count);
  const auto yellow_samples = sampleNormalized(yellow, sample_count);
  std::vector<double> signs;
  signs.reserve(sample_count - 1U);
  for (std::size_t i = 1; i < sample_count; ++i) {
    const PlannerPoint previous_mid = scale(add(blue_samples[i - 1], yellow_samples[i - 1]), 0.5);
    const PlannerPoint current_mid = scale(add(blue_samples[i], yellow_samples[i]), 0.5);
    const PlannerPoint tangent = subtract(current_mid, previous_mid);
    const PlannerPoint side = subtract(yellow_samples[i], blue_samples[i]);
    const double tangent_length = std::hypot(tangent.x, tangent.y);
    const double side_length = std::hypot(side.x, side.y);
    if (tangent_length > kGeometryEpsilon && side_length > kGeometryEpsilon) {
      signs.push_back(cross2d(tangent, side) / (tangent_length * side_length));
    }
  }
  return median(signs);
}

bool orientTravelDirection(std::vector<PlannerPoint> & blue, std::vector<PlannerPoint> & yellow)
{
  auto sign = medianTrackSideSign(blue, yellow);
  if (!sign.has_value() || std::abs(sign.value()) <= kGeometryEpsilon) {
    return false;
  }
  if (sign.value() > 0.0) {
    std::reverse(blue.begin(), blue.end());
    std::reverse(yellow.begin(), yellow.end());
    sign = medianTrackSideSign(blue, yellow);
  }
  return sign.has_value() && sign.value() < -kGeometryEpsilon;
}

}

bool orderSlamBoundaries(
  const std::vector<PlannerPoint> & blue_points,
  const std::vector<PlannerPoint> & yellow_points,
  const PlannerPoint & ego,
  double max_gap,
  std::vector<PlannerPoint> & ordered_blue,
  std::vector<PlannerPoint> & ordered_yellow,
  std::string & reason)
{
  if (!orderBoundary(blue_points, ego, max_gap, ordered_blue, reason) ||
    !orderBoundary(yellow_points, ego, max_gap, ordered_yellow, reason))
  {
    return false;
  }
  if (!alignBoundaryDirections(ordered_blue, ordered_yellow)) {
    reason = "could not align boundary directions";
    return false;
  }
  if (!orientTravelDirection(ordered_blue, ordered_yellow)) {
    reason = "inconsistent blue/yellow travel direction";
    return false;
  }
  return true;
}

}
