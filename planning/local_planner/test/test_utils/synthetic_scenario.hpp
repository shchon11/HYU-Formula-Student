#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ksae_local/types.hpp"

namespace ksae_local::test
{

inline constexpr std::uint32_t kDefaultScenarioSeed = 0x4B534145U;

/// One synchronized synthetic input frame.
struct ScenarioFrame
{
  /// Pose of the observation sensor expressed in the odom frame.
  Pose2D sensor_pose_odom;
  double timestamp_s{0.0};
  /// Observations expressed in the sensor frame at timestamp_s.
  std::vector<ConeObservation> observations_sensor;
};

/// Properties that a future algorithm may rely on without prescribing its output.
struct ScenarioInvariants
{
  std::size_t ground_truth_left_cone_count{0U};
  std::size_t ground_truth_right_cone_count{0U};
  std::size_t maximum_observation_count_per_frame{0U};
  std::size_t total_unique_physical_cone_count{0U};
  std::size_t expected_empty_frame_count{0U};
  bool temporal_accumulation_possible{false};
  bool spatial_information_expected_to_increase{false};
  PlannerMode expected_mode_hint{PlannerMode::BOOTSTRAP};
  std::vector<std::string> notes;
};

/// Deterministic, ROS-independent fixture for future planning algorithms.
struct SyntheticScenario
{
  std::string name;
  std::vector<ScenarioFrame> frames;
  ScenarioInvariants expected;
  std::optional<std::vector<Point2D>> ground_truth_left_boundary_odom;
  std::optional<std::vector<Point2D>> ground_truth_right_boundary_odom;
  std::optional<std::vector<Point2D>> ground_truth_centerline_odom;

  /// Metadata used by fault injectors when rebuilding observations.
  ConeColorProbability nominal_left_color;
  ConeColorProbability nominal_right_color;
  Covariance2D nominal_observation_covariance_sensor;
  std::uint32_t seed{kDefaultScenarioSeed};
};

std::size_t observationCount(const SyntheticScenario & scenario) noexcept;
std::size_t maximumObservationCountPerFrame(const SyntheticScenario & scenario) noexcept;
std::size_t emptyFrameCount(const SyntheticScenario & scenario) noexcept;

/// Check poses, timestamps, observations, covariance/color values, and optional truth.
bool scenarioContainsOnlyFiniteValues(const SyntheticScenario & scenario) noexcept;

/// Refresh observation-count invariants after a fault injection.
void refreshObservationInvariants(SyntheticScenario & scenario) noexcept;

}  // namespace ksae_local::test
