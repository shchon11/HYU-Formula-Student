#include "test_utils/scenario_catalog.hpp"

#include <stdexcept>
#include <utility>

#include "test_utils/fault_injector.hpp"
#include "test_utils/scenario_builder.hpp"

namespace ksae_local::test
{
namespace
{

SyntheticScenario named(SyntheticScenario scenario, const std::string & name)
{
  scenario.name = name;
  return scenario;
}

}  // namespace

std::vector<SyntheticScenario> makeRequiredScenarios(const std::uint32_t seed)
{
  ScenarioBuilder builder;
  builder.setSeed(seed);

  const SyntheticScenario straight = builder.buildStraight("straight_full");
  const SyntheticScenario left_curve = builder.buildLeftCurve(20.0, "left_curve_full");
  const SyntheticScenario right_curve = builder.buildRightCurve(20.0, "right_curve_full");

  SyntheticScenario new_pairs = named(
    FaultInjector::oneNewPairPerFrame(straight),
    "one_pair_per_frame_new_cones");

  SyntheticScenario repeated_pair = named(
    FaultInjector::repeatSamePair(straight),
    "same_pair_repeated");

  SyntheticScenario left_missing = named(
    FaultInjector::removeSide(straight, BoundarySide::LEFT),
    "left_side_missing");

  SyntheticScenario right_missing = named(
    FaultInjector::removeSide(straight, BoundarySide::RIGHT),
    "right_side_missing");

  SyntheticScenario partial_swap = named(
    FaultInjector::swapBlueYellow(straight, 0.25, seed + 1U),
    "partial_color_swap");

  SyntheticScenario all_unknown = named(
    FaultInjector::convertAllToUnknown(straight),
    "all_unknown");

  SyntheticScenario false_positive = named(
    FaultInjector::addFalsePositivesNearCenter(
      straight, 1U, 2.0, 10.0, 0.15, seed + 2U),
    "false_positive_center");

  SyntheticScenario noisy = FaultInjector::addGaussianPositionNoise(
    straight, 0.05, seed + 3U);
  noisy = named(
    FaultInjector::addDuplicates(noisy, 1U, 0.01, seed + 4U),
    "noisy_duplicates");

  SyntheticScenario observation_dropout = named(
    FaultInjector::dropObservationFrames(straight, 2U, 3U),
    "observation_dropout");

  return {
    straight,
    left_curve,
    right_curve,
    std::move(new_pairs),
    std::move(repeated_pair),
    std::move(left_missing),
    std::move(right_missing),
    std::move(partial_swap),
    std::move(all_unknown),
    std::move(false_positive),
    std::move(noisy),
    std::move(observation_dropout)};
}

SyntheticScenario makeRequiredScenario(
  const std::string & name, const std::uint32_t seed)
{
  std::vector<SyntheticScenario> scenarios = makeRequiredScenarios(seed);
  for (SyntheticScenario & scenario : scenarios) {
    if (scenario.name == name) {
      return std::move(scenario);
    }
  }
  throw std::invalid_argument("unknown required scenario: " + name);
}

}  // namespace ksae_local::test
