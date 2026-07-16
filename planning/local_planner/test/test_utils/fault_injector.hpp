#pragma once

#include <cstddef>
#include <cstdint>

#include "test_utils/synthetic_scenario.hpp"

namespace ksae_local::test
{

/// Deterministic transformations from an ideal scenario to faulty observations.
class FaultInjector
{
public:
  /// Drop exactly round(total_observations * ratio) observations.
  static SyntheticScenario dropCones(
    const SyntheticScenario & source, double ratio, std::uint32_t seed);

  static SyntheticScenario removeSide(
    const SyntheticScenario & source, BoundarySide side_to_remove);

  /// Swap blue/yellow probabilities on exactly round(total_observations * ratio) samples.
  static SyntheticScenario swapBlueYellow(
    const SyntheticScenario & source, double ratio, std::uint32_t seed);

  static SyntheticScenario convertAllToUnknown(const SyntheticScenario & source);

  static SyntheticScenario addGaussianPositionNoise(
    const SyntheticScenario & source, double standard_deviation_m, std::uint32_t seed);

  static SyntheticScenario addDuplicates(
    const SyntheticScenario & source, std::size_t duplicates_per_observation,
    double duplicate_noise_standard_deviation_m, std::uint32_t seed);

  /// Add unknown-color false positives in a narrow sensor-frame strip around y=0.
  static SyntheticScenario addFalsePositivesNearCenter(
    const SyntheticScenario & source, std::size_t count_per_frame,
    double minimum_x_sensor_m, double maximum_x_sensor_m,
    double lateral_half_width_m, std::uint32_t seed);

  /// Make observation timestamps older while retaining each frame timestamp.
  static SyntheticScenario delayObservationTimestamps(
    const SyntheticScenario & source, double delay_s);

  /// Observe the same physical left/right pair in every frame.
  static SyntheticScenario repeatSamePair(
    const SyntheticScenario & source, std::size_t left_index = 0U,
    std::size_t right_index = 0U);

  /// Observe a different physical left/right pair in each successive frame when available.
  static SyntheticScenario oneNewPairPerFrame(const SyntheticScenario & source);

  /// Remove all observations from a contiguous frame interval.
  static SyntheticScenario dropObservationFrames(
    const SyntheticScenario & source, std::size_t first_frame,
    std::size_t frame_count);
};

}  // namespace ksae_local::test
