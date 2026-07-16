#include "test_utils/synthetic_scenario.hpp"

#include <algorithm>
#include <cmath>

#include "ksae_local/geometry.hpp"

namespace ksae_local::test
{
namespace
{

bool colorIsFinite(const ConeColorProbability & color) noexcept
{
  return isFinite(color.blue) && isFinite(color.yellow) && isFinite(color.orange) &&
         isFinite(color.big_orange) && isFinite(color.unknown);
}

bool pointsAreFinite(const std::optional<std::vector<Point2D>> & points) noexcept
{
  if (!points.has_value()) {
    return true;
  }
  return std::all_of(points->begin(), points->end(), [](const Point2D & point) {
    return isFinite(point);
  });
}

}  // namespace

std::size_t observationCount(const SyntheticScenario & scenario) noexcept
{
  std::size_t count = 0U;
  for (const ScenarioFrame & frame : scenario.frames) {
    count += frame.observations_sensor.size();
  }
  return count;
}

std::size_t maximumObservationCountPerFrame(const SyntheticScenario & scenario) noexcept
{
  std::size_t maximum = 0U;
  for (const ScenarioFrame & frame : scenario.frames) {
    maximum = std::max(maximum, frame.observations_sensor.size());
  }
  return maximum;
}

std::size_t emptyFrameCount(const SyntheticScenario & scenario) noexcept
{
  return static_cast<std::size_t>(std::count_if(
    scenario.frames.begin(), scenario.frames.end(), [](const ScenarioFrame & frame) {
      return frame.observations_sensor.empty();
    }));
}

bool scenarioContainsOnlyFiniteValues(const SyntheticScenario & scenario) noexcept
{
  if (!colorIsFinite(scenario.nominal_left_color) ||
    !colorIsFinite(scenario.nominal_right_color) ||
    !isFinite(scenario.nominal_observation_covariance_sensor) ||
    !pointsAreFinite(scenario.ground_truth_left_boundary_odom) ||
    !pointsAreFinite(scenario.ground_truth_right_boundary_odom) ||
    !pointsAreFinite(scenario.ground_truth_centerline_odom))
  {
    return false;
  }

  for (const ScenarioFrame & frame : scenario.frames) {
    if (!isFinite(frame.sensor_pose_odom) || !isFinite(frame.timestamp_s)) {
      return false;
    }
    for (const ConeObservation & observation : frame.observations_sensor) {
      if (!isFinite(observation.position_sensor) ||
        !isFinite(observation.covariance_sensor) ||
        !colorIsFinite(observation.color_probability) ||
        !isFinite(observation.timestamp_s))
      {
        return false;
      }
    }
  }
  return true;
}

void refreshObservationInvariants(SyntheticScenario & scenario) noexcept
{
  scenario.expected.maximum_observation_count_per_frame =
    maximumObservationCountPerFrame(scenario);
  scenario.expected.expected_empty_frame_count = emptyFrameCount(scenario);
}

}  // namespace ksae_local::test
