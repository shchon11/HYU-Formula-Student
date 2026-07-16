#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "test_utils/synthetic_scenario.hpp"

namespace ksae_local::test
{

/// Build the twelve required stage-2 scenarios in stable catalog order.
std::vector<SyntheticScenario> makeRequiredScenarios(
  std::uint32_t seed = kDefaultScenarioSeed);

/// Build one required scenario by its documented name.
SyntheticScenario makeRequiredScenario(
  const std::string & name, std::uint32_t seed = kDefaultScenarioSeed);

}  // namespace ksae_local::test
