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

bool nearestNeighborOrder(
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
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < input.size(); ++i) {
      if (!used[i] && distance(input[current], input[i]) < best_distance) {
        best_distance = distance(input[current], input[i]);
        next = i;
      }
    }
    if (next == input.size()) {
      reason = "nearest-neighbor ordering failed";
      return false;
    }
    if (best_distance > max_gap) {
      std::ostringstream stream;
      stream << "boundary gap " << best_distance << " m exceeds " << max_gap << " m";
      reason = stream.str();
      return false;
    }
    used[next] = true;
    ordered.push_back(input[next]);
    current = next;
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
  if (!nearestNeighborOrder(blue_points, ego, max_gap, ordered_blue, reason) ||
    !nearestNeighborOrder(yellow_points, ego, max_gap, ordered_yellow, reason))
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
