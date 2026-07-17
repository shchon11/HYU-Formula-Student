#include "test_utils/fault_injector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ksae_local/geometry.hpp"

namespace ksae_local::test
{
namespace
{

double checkedRatio(const double ratio)
{
  if (!isFinite(ratio)) {
    throw std::invalid_argument("fault ratio must be finite");
  }
  return clamp(ratio, 0.0, 1.0);
}

void appendNote(SyntheticScenario & scenario, const std::string & note)
{
  scenario.expected.notes.push_back(note);
}

std::vector<bool> exactSelection(
  const std::size_t total, const double ratio, const std::uint32_t seed)
{
  std::vector<std::size_t> indices(total);
  std::iota(indices.begin(), indices.end(), 0U);
  std::mt19937 generator(seed);
  std::shuffle(indices.begin(), indices.end(), generator);

  const std::size_t selected_count = static_cast<std::size_t>(
    std::llround(static_cast<double>(total) * checkedRatio(ratio)));
  std::vector<bool> selected(total, false);
  for (std::size_t index = 0U; index < selected_count; ++index) {
    selected[indices[index]] = true;
  }
  return selected;
}

double nearestSquaredDistance(
  const Point2D & point, const std::optional<std::vector<Point2D>> & candidates)
{
  if (!candidates.has_value() || candidates->empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double nearest = std::numeric_limits<double>::infinity();
  for (const Point2D & candidate : *candidates) {
    nearest = std::min(nearest, squaredDistance(point, candidate));
  }
  return nearest;
}

BoundarySide groundTruthSide(
  const ScenarioFrame & frame, const ConeObservation & observation,
  const SyntheticScenario & scenario)
{
  const Point2D position_odom =
    transformPoint(observation.position_sensor, frame.sensor_pose_odom);
  const double left_distance =
    nearestSquaredDistance(position_odom, scenario.ground_truth_left_boundary_odom);
  const double right_distance =
    nearestSquaredDistance(position_odom, scenario.ground_truth_right_boundary_odom);
  if (left_distance < right_distance) {
    return BoundarySide::LEFT;
  }
  if (right_distance < left_distance) {
    return BoundarySide::RIGHT;
  }
  return BoundarySide::UNKNOWN;
}

ConeObservation observationFromGroundTruth(
  const Point2D & point_odom, const ScenarioFrame & frame,
  const Covariance2D & covariance, const ConeColorProbability & color)
{
  return {
    inverseTransformPoint(point_odom, frame.sensor_pose_odom),
    covariance,
    color,
    frame.timestamp_s};
}

const std::vector<Point2D> & requireBoundary(
  const std::optional<std::vector<Point2D>> & boundary, const char * description)
{
  if (!boundary.has_value() || boundary->empty()) {
    throw std::invalid_argument(description);
  }
  return *boundary;
}

}  // namespace

SyntheticScenario FaultInjector::dropCones(
  const SyntheticScenario & source, const double ratio, const std::uint32_t seed)
{
  SyntheticScenario result = source;
  result.seed = seed;
  const std::vector<bool> dropped = exactSelection(observationCount(source), ratio, seed);

  std::size_t flat_index = 0U;
  for (std::size_t frame_index = 0U; frame_index < source.frames.size(); ++frame_index) {
    result.frames[frame_index].observations_sensor.clear();
    for (const ConeObservation & observation : source.frames[frame_index].observations_sensor) {
      if (!dropped[flat_index]) {
        result.frames[frame_index].observations_sensor.push_back(observation);
      }
      ++flat_index;
    }
  }
  appendNote(result, "An exact seeded fraction of observation samples was dropped.");
  refreshObservationInvariants(result);
  return result;
}

SyntheticScenario FaultInjector::removeSide(
  const SyntheticScenario & source, const BoundarySide side_to_remove)
{
  if (side_to_remove != BoundarySide::LEFT && side_to_remove != BoundarySide::RIGHT) {
    throw std::invalid_argument("removeSide accepts only LEFT or RIGHT");
  }

  SyntheticScenario result = source;
  for (std::size_t frame_index = 0U; frame_index < source.frames.size(); ++frame_index) {
    result.frames[frame_index].observations_sensor.clear();
    for (const ConeObservation & observation : source.frames[frame_index].observations_sensor) {
      if (groundTruthSide(source.frames[frame_index], observation, source) != side_to_remove) {
        result.frames[frame_index].observations_sensor.push_back(observation);
      }
    }
  }

  if (side_to_remove == BoundarySide::LEFT) {
    result.expected.total_unique_physical_cone_count =
      result.expected.ground_truth_right_cone_count;
    result.expected.expected_mode_hint = PlannerMode::RIGHT_ONLY;
    appendNote(result, "All observations associated with the ground-truth left side were removed.");
  } else {
    result.expected.total_unique_physical_cone_count =
      result.expected.ground_truth_left_cone_count;
    result.expected.expected_mode_hint = PlannerMode::LEFT_ONLY;
    appendNote(result, "All observations associated with the ground-truth right side were removed.");
  }
  refreshObservationInvariants(result);
  return result;
}

SyntheticScenario FaultInjector::swapBlueYellow(
  const SyntheticScenario & source, const double ratio, const std::uint32_t seed)
{
  SyntheticScenario result = source;
  result.seed = seed;
  const std::vector<bool> swapped = exactSelection(observationCount(source), ratio, seed);
  std::size_t flat_index = 0U;
  for (ScenarioFrame & frame : result.frames) {
    for (ConeObservation & observation : frame.observations_sensor) {
      if (swapped[flat_index]) {
        std::swap(
          observation.color_probability.blue,
          observation.color_probability.yellow);
      }
      ++flat_index;
    }
  }
  appendNote(result, "A seeded exact fraction of blue/yellow probabilities was swapped.");
  return result;
}

SyntheticScenario FaultInjector::convertAllToUnknown(const SyntheticScenario & source)
{
  SyntheticScenario result = source;
  const ConeColorProbability unknown{0.0, 0.0, 0.0, 0.0, 1.0};
  for (ScenarioFrame & frame : result.frames) {
    for (ConeObservation & observation : frame.observations_sensor) {
      observation.color_probability = unknown;
    }
  }
  result.expected.expected_mode_hint = PlannerMode::BOOTSTRAP;
  appendNote(result, "Every observed cone has unknown color probability one.");
  return result;
}

SyntheticScenario FaultInjector::addGaussianPositionNoise(
  const SyntheticScenario & source, const double standard_deviation_m,
  const std::uint32_t seed)
{
  if (!isFinite(standard_deviation_m) || standard_deviation_m < 0.0) {
    throw std::invalid_argument("noise standard deviation must be finite and nonnegative");
  }
  SyntheticScenario result = source;
  result.seed = seed;
  if (standard_deviation_m == 0.0) {
    return result;
  }

  std::mt19937 generator(seed);
  std::normal_distribution<double> noise(0.0, standard_deviation_m);
  for (ScenarioFrame & frame : result.frames) {
    for (ConeObservation & observation : frame.observations_sensor) {
      observation.position_sensor.x += noise(generator);
      observation.position_sensor.y += noise(generator);
    }
  }
  appendNote(result, "Seeded independent Gaussian sensor-frame position noise was added.");
  return result;
}

SyntheticScenario FaultInjector::addDuplicates(
  const SyntheticScenario & source, const std::size_t duplicates_per_observation,
  const double duplicate_noise_standard_deviation_m, const std::uint32_t seed)
{
  if (!isFinite(duplicate_noise_standard_deviation_m) ||
    duplicate_noise_standard_deviation_m < 0.0)
  {
    throw std::invalid_argument("duplicate noise must be finite and nonnegative");
  }
  SyntheticScenario result = source;
  result.seed = seed;
  std::mt19937 generator(seed);
  std::normal_distribution<double> unit_noise(0.0, 1.0);

  for (std::size_t frame_index = 0U; frame_index < source.frames.size(); ++frame_index) {
    auto & output = result.frames[frame_index].observations_sensor;
    const auto & original = source.frames[frame_index].observations_sensor;
    output.reserve(original.size() * (duplicates_per_observation + 1U));
    for (const ConeObservation & observation : original) {
      for (std::size_t duplicate = 0U; duplicate < duplicates_per_observation; ++duplicate) {
        ConeObservation copy = observation;
        if (duplicate_noise_standard_deviation_m > 0.0) {
          copy.position_sensor.x +=
            duplicate_noise_standard_deviation_m * unit_noise(generator);
          copy.position_sensor.y +=
            duplicate_noise_standard_deviation_m * unit_noise(generator);
        }
        output.push_back(copy);
      }
    }
  }
  appendNote(result, "Nearby duplicate samples were added without adding physical cones.");
  refreshObservationInvariants(result);
  return result;
}

SyntheticScenario FaultInjector::addFalsePositivesNearCenter(
  const SyntheticScenario & source, const std::size_t count_per_frame,
  double minimum_x_sensor_m, double maximum_x_sensor_m,
  const double lateral_half_width_m, const std::uint32_t seed)
{
  if (!isFinite(minimum_x_sensor_m) || !isFinite(maximum_x_sensor_m) ||
    !isFinite(lateral_half_width_m) || lateral_half_width_m < 0.0)
  {
    throw std::invalid_argument("false-positive bounds must be finite and nonnegative");
  }
  if (minimum_x_sensor_m > maximum_x_sensor_m) {
    std::swap(minimum_x_sensor_m, maximum_x_sensor_m);
  }

  SyntheticScenario result = source;
  result.seed = seed;
  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> longitudinal(minimum_x_sensor_m, maximum_x_sensor_m);
  std::uniform_real_distribution<double> lateral(-lateral_half_width_m, lateral_half_width_m);
  const ConeColorProbability unknown{0.0, 0.0, 0.0, 0.0, 1.0};

  for (ScenarioFrame & frame : result.frames) {
    for (std::size_t count = 0U; count < count_per_frame; ++count) {
      frame.observations_sensor.push_back({
        {longitudinal(generator), lateral(generator)},
        result.nominal_observation_covariance_sensor,
        unknown,
        frame.timestamp_s});
    }
  }
  appendNote(result, "Unknown-color false positives were added near the track center.");
  refreshObservationInvariants(result);
  return result;
}

SyntheticScenario FaultInjector::delayObservationTimestamps(
  const SyntheticScenario & source, const double delay_s)
{
  if (!isFinite(delay_s) || delay_s < 0.0) {
    throw std::invalid_argument("timestamp delay must be finite and nonnegative");
  }
  SyntheticScenario result = source;
  for (ScenarioFrame & frame : result.frames) {
    for (ConeObservation & observation : frame.observations_sensor) {
      observation.timestamp_s -= delay_s;
    }
  }
  appendNote(result, "Observation timestamps lag their corresponding frame timestamps.");
  return result;
}

SyntheticScenario FaultInjector::repeatSamePair(
  const SyntheticScenario & source, const std::size_t left_index,
  const std::size_t right_index)
{
  const auto & left = requireBoundary(
    source.ground_truth_left_boundary_odom, "same-pair fixture requires left ground truth");
  const auto & right = requireBoundary(
    source.ground_truth_right_boundary_odom, "same-pair fixture requires right ground truth");
  if (left_index >= left.size() || right_index >= right.size()) {
    throw std::out_of_range("same-pair ground-truth index is out of range");
  }

  SyntheticScenario result = source;
  for (ScenarioFrame & frame : result.frames) {
    frame.observations_sensor.clear();
    frame.observations_sensor.push_back(observationFromGroundTruth(
      left[left_index], frame, result.nominal_observation_covariance_sensor,
      result.nominal_left_color));
    frame.observations_sensor.push_back(observationFromGroundTruth(
      right[right_index], frame, result.nominal_observation_covariance_sensor,
      result.nominal_right_color));
  }
  result.expected.total_unique_physical_cone_count = 2U;
  result.expected.temporal_accumulation_possible = false;
  result.expected.spatial_information_expected_to_increase = false;
  result.expected.expected_mode_hint = PlannerMode::BOOTSTRAP;
  appendNote(result, "Repeated frames must not be interpreted as new spatial cone information.");
  refreshObservationInvariants(result);
  return result;
}

SyntheticScenario FaultInjector::oneNewPairPerFrame(const SyntheticScenario & source)
{
  const auto & left = requireBoundary(
    source.ground_truth_left_boundary_odom, "new-pair fixture requires left ground truth");
  const auto & right = requireBoundary(
    source.ground_truth_right_boundary_odom, "new-pair fixture requires right ground truth");
  const std::size_t pair_count = std::min(left.size(), right.size());

  SyntheticScenario result = source;
  for (std::size_t frame_index = 0U; frame_index < result.frames.size(); ++frame_index) {
    ScenarioFrame & frame = result.frames[frame_index];
    const std::size_t pair_index = frame_index % pair_count;
    frame.observations_sensor.clear();
    frame.observations_sensor.push_back(observationFromGroundTruth(
      left[pair_index], frame, result.nominal_observation_covariance_sensor,
      result.nominal_left_color));
    frame.observations_sensor.push_back(observationFromGroundTruth(
      right[pair_index], frame, result.nominal_observation_covariance_sensor,
      result.nominal_right_color));
  }
  result.expected.total_unique_physical_cone_count =
    2U * std::min(result.frames.size(), pair_count);
  result.expected.temporal_accumulation_possible = result.frames.size() > 1U;
  result.expected.spatial_information_expected_to_increase = result.frames.size() > 1U;
  result.expected.expected_mode_hint = PlannerMode::BOOTSTRAP;
  appendNote(result, "Each frame exposes a new physical pair until ground truth is exhausted.");
  refreshObservationInvariants(result);
  return result;
}

SyntheticScenario FaultInjector::dropObservationFrames(
  const SyntheticScenario & source, const std::size_t first_frame,
  const std::size_t frame_count)
{
  SyntheticScenario result = source;
  const std::size_t begin_frame = std::min(first_frame, result.frames.size());
  const std::size_t available_frames = result.frames.size() - begin_frame;
  const std::size_t end_frame = begin_frame + std::min(frame_count, available_frames);
  for (std::size_t index = begin_frame;
    index < end_frame; ++index)
  {
    result.frames[index].observations_sensor.clear();
  }
  result.expected.expected_mode_hint = PlannerMode::MEMORY_ONLY;
  appendNote(result, "A contiguous interval contains frames but no cone observations.");
  refreshObservationInvariants(result);
  return result;
}

}  // namespace ksae_local::test
