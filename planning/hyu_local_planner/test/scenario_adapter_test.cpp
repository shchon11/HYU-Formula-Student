#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

#include "test_utils/scenario_adapter.hpp"
#include "test_utils/scenario_builder.hpp"
#include "test_utils/scenario_catalog.hpp"

namespace hyu_local_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

using ksae_local::test::SyntheticScenario;
using ksae_local::test::makeRequiredScenarios;
using test::MockSlamMap;
using test::planScenario;

double headingStep(const PathWaypoint & a, const PathWaypoint & b)
{
  double step = std::abs(b.psi - a.psi);
  if (step > kPi) {
    step = 2.0 * kPi - step;
  }
  return step;
}

double maxHeadingStep(const std::vector<PathWaypoint> & waypoints)
{
  double worst = 0.0;
  for (std::size_t i = 1; i + 1U < waypoints.size(); ++i) {
    worst = std::max(worst, headingStep(waypoints[i - 1U], waypoints[i]));
  }
  return worst;
}

const char * kindName(PathKind kind)
{
  switch (kind) {
    case PathKind::kNone:
      return "kNone";
    case PathKind::kTwoSided:
      return "kTwoSided";
    case PathKind::kBlueOnly:
      return "kBlueOnly";
    case PathKind::kYellowOnly:
      return "kYellowOnly";
  }
  return "?";
}

struct Baseline
{
  const char * name;
  bool valid;
  PathKind kind;
  std::size_t waypoints;
  const char * reason;  // pinned only for invalid outcomes
};

// Stage-0 recorded baseline: the observed outcome of every catalog scenario
// against today's planner (slam-mode default config, full-scenario map, plan
// from the last frame pose). Descriptive, not normative — a deliberate
// behavior change updates this table in the same commit, an accidental one
// cannot.
//
// Notes from the recording run:
//  * same_pair_repeated goes invalid because the single observed pair has
//    fallen behind the ROI by the last frame — the map holds nothing ahead.
//  * partial_color_swap degrades to kBlueOnly: swapped cones break clean
//    two-sided pairing, the blue-side fallback survives.
constexpr Baseline kBaseline[] = {
  {"straight_full", true, PathKind::kTwoSided, 19U, ""},
  {"left_curve_full", true, PathKind::kTwoSided, 19U, ""},
  {"right_curve_full", true, PathKind::kTwoSided, 19U, ""},
  {"one_pair_per_frame_new_cones", true, PathKind::kTwoSided, 19U, ""},
  {"same_pair_repeated", false, PathKind::kNone, 0U, "roi_no_boundary_cones"},
  {"left_side_missing", true, PathKind::kYellowOnly, 17U, ""},
  {"right_side_missing", true, PathKind::kBlueOnly, 17U, ""},
  {"partial_color_swap", true, PathKind::kBlueOnly, 17U, ""},
  {"all_unknown", true, PathKind::kTwoSided, 19U, ""},
  {"false_positive_center", true, PathKind::kTwoSided, 19U, ""},
  {"noisy_duplicates", true, PathKind::kTwoSided, 19U, ""},
  {"observation_dropout", true, PathKind::kTwoSided, 19U, ""},
};

TEST(ScenarioCatalogBaseline, OutcomesMatchRecordedBaseline)
{
  const auto scenarios = makeRequiredScenarios();
  ASSERT_EQ(scenarios.size(), std::size(kBaseline));
  for (std::size_t i = 0; i < scenarios.size(); ++i) {
    const auto & scenario = scenarios[i];
    const Baseline & expected = kBaseline[i];
    ASSERT_EQ(scenario.name, std::string(expected.name)) << "catalog order changed";
    const BuildResult result = planScenario(scenario);
    EXPECT_EQ(result.valid, expected.valid)
      << scenario.name << " reason=\"" << result.reason << "\"";
    if (!expected.valid) {
      EXPECT_EQ(result.reason, std::string(expected.reason)) << scenario.name;
      continue;
    }
    EXPECT_EQ(result.kind, expected.kind)
      << scenario.name << " got " << kindName(result.kind);
    EXPECT_EQ(result.waypoints.size(), expected.waypoints) << scenario.name;
  }
}

// The mock map must merge per-frame re-observations back into one landmark
// per observed physical cone. straight_full has noise-free observations, so
// an exact-position dedupe over all frames is an independent oracle for the
// unique observed-cone count.
TEST(MockSlamMap, MergesReobservationsIntoUniquePhysicalCones)
{
  const SyntheticScenario straight =
    ksae_local::test::makeRequiredScenario("straight_full");
  ASSERT_TRUE(straight.expected.temporal_accumulation_possible);

  std::vector<ksae_local::Point2D> unique_odom;
  std::size_t observation_count = 0U;
  for (const auto & frame : straight.frames) {
    for (const auto & observation : frame.observations_sensor) {
      ++observation_count;
      const auto odom =
        ksae_local::transformPoint(observation.position_sensor, frame.sensor_pose_odom);
      bool seen = false;
      for (const auto & existing : unique_odom) {
        if (ksae_local::distance(existing, odom) < 1.0e-6) {
          seen = true;
          break;
        }
      }
      if (!seen) {
        unique_odom.push_back(odom);
      }
    }
  }

  MockSlamMap map;
  map.ingestScenario(straight);
  EXPECT_EQ(map.cones().size(), unique_odom.size());
  // The scenario must actually re-observe cones or this test is vacuous.
  EXPECT_GT(observation_count, map.cones().size());
  // Every observed cone is a real track cone, never an extra fabrication.
  EXPECT_LE(map.cones().size(), straight.expected.total_unique_physical_cone_count);
}

// Whatever the outcome table says, a valid path from any catalog scenario must
// be geometrically sane: finite fields, monotonic arc length, positive speeds,
// and no fold-back (the boa invariant).
TEST(ScenarioCatalogBaseline, ValidPathsAreSaneAndFinite)
{
  std::size_t valid_count = 0U;
  for (const auto & scenario : makeRequiredScenarios()) {
    const BuildResult result = planScenario(scenario);
    if (!result.valid) {
      continue;
    }
    ++valid_count;
    ASSERT_GE(result.waypoints.size(), 2U) << scenario.name;
    double previous_s = -1.0;
    for (const auto & waypoint : result.waypoints) {
      ASSERT_TRUE(
        std::isfinite(waypoint.x) && std::isfinite(waypoint.y) &&
        std::isfinite(waypoint.s) && std::isfinite(waypoint.psi) &&
        std::isfinite(waypoint.kappa) && std::isfinite(waypoint.speed))
        << scenario.name;
      ASSERT_GT(waypoint.s, previous_s) << scenario.name;
      ASSERT_GT(waypoint.speed, 0.0) << scenario.name;
      previous_s = waypoint.s;
    }
    EXPECT_LT(maxHeadingStep(result.waypoints), 0.5 * kPi) << scenario.name;
  }
  EXPECT_GT(valid_count, 0U);
}

// Same scenario, two independent map builds: byte-identical plans. The whole
// fixture chain (seeded builder, fault injectors, adapter) must be
// deterministic or later golden gates are meaningless.
TEST(ScenarioCatalogBaseline, AdapterIsDeterministic)
{
  for (const char * name : {"straight_full", "noisy_duplicates", "partial_color_swap"}) {
    const SyntheticScenario scenario = ksae_local::test::makeRequiredScenario(name);
    const BuildResult first = planScenario(scenario);
    const BuildResult second = planScenario(scenario);
    ASSERT_EQ(first.valid, second.valid) << name;
    ASSERT_EQ(first.kind, second.kind) << name;
    ASSERT_EQ(first.waypoints.size(), second.waypoints.size()) << name;
    for (std::size_t i = 0; i < first.waypoints.size(); ++i) {
      EXPECT_EQ(first.waypoints[i].x, second.waypoints[i].x) << name;
      EXPECT_EQ(first.waypoints[i].y, second.waypoints[i].y) << name;
      EXPECT_EQ(first.waypoints[i].s, second.waypoints[i].s) << name;
      EXPECT_EQ(first.waypoints[i].psi, second.waypoints[i].psi) << name;
      EXPECT_EQ(first.waypoints[i].kappa, second.waypoints[i].kappa) << name;
      EXPECT_EQ(first.waypoints[i].speed, second.waypoints[i].speed) << name;
    }
  }
}

}  // namespace
}  // namespace hyu_local_planner
