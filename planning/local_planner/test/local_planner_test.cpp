#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "local_planner/local_path_builder.hpp"

namespace local_planner
{
namespace
{

ConeSet straightTwoSided()
{
  ConeSet cones;
  for (double x : {0.0, 2.0, 4.0, 6.0, 8.0}) {
    cones.blue.push_back({x, 2.0});
    cones.yellow.push_back({x, -2.0});
  }
  return cones;
}

ConeSet simulatorLikeTwoBlueThreeYellow()
{
  ConeSet cones;
  cones.blue = {{0.0, 2.0}, {4.0, 2.0}};
  cones.yellow = {{0.0, -2.0}, {2.0, -2.0}, {4.0, -2.0}};
  cones.orange = {{1.0, 0.0}, {3.0, 0.0}};
  cones.big_orange = {{2.0, 0.0}};
  cones.unknown = {{1.5, 0.2}, {2.5, -0.2}};
  return cones;
}

bool expectStrictFinitePath(const BuildResult & result, double speed)
{
  if (!result.evaluated || !result.valid || result.waypoints.size() < 5U) {
    ADD_FAILURE() << "planner did not evaluate a valid five-point path: " << result.reason;
    return false;
  }
  for (std::size_t index = 0; index < result.waypoints.size(); ++index) {
    const auto & waypoint = result.waypoints[index];
    EXPECT_TRUE(std::isfinite(waypoint.x));
    EXPECT_TRUE(std::isfinite(waypoint.y));
    EXPECT_TRUE(std::isfinite(waypoint.s));
    EXPECT_TRUE(std::isfinite(waypoint.psi));
    EXPECT_TRUE(std::isfinite(waypoint.kappa));
    EXPECT_DOUBLE_EQ(waypoint.speed, speed);
    if (index > 0U) {
      EXPECT_GT(waypoint.s, result.waypoints[index - 1U].s);
    }
  }
  return true;
}

void expectInvalidEmptyPath(const BuildResult & result, const char * invalid_message = "")
{
  ASSERT_TRUE(result.evaluated);
  EXPECT_FALSE(result.valid) << invalid_message;
  EXPECT_TRUE(result.waypoints.empty());
}

TEST(LocalPathBuilder, TwoSidedUsesDeterministicBoundaryTraversalAndDedupe)
{
  auto cones = straightTwoSided();
  cones.blue.push_back({4.01, 2.0});
  const auto first = buildLocalPath(cones);
  const auto second = buildLocalPath(cones);
  if (!expectStrictFinitePath(first, 3.0)) {
    return;
  }
  ASSERT_TRUE(second.valid) << second.reason;
  ASSERT_EQ(first.waypoints.size(), second.waypoints.size());
  for (std::size_t index = 0; index < first.waypoints.size(); ++index) {
    EXPECT_DOUBLE_EQ(first.waypoints[index].x, second.waypoints[index].x);
    EXPECT_DOUBLE_EQ(first.waypoints[index].y, second.waypoints[index].y);
  }
  EXPECT_EQ(first.kind, PathKind::kTwoSided);
}

TEST(LocalPathBuilder, SimulatorLikeTwoBlueThreeYellowIgnoresNonBoundaryNoise)
{
  const auto result = buildLocalPath(simulatorLikeTwoBlueThreeYellow());

  if (!expectStrictFinitePath(result, 3.0)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
}

TEST(LocalPathBuilder, TwoSidedPathPairsOrderedMapBoundariesThroughCurve)
{
  ConeSet cones;
  cones.blue = {{0.0, 2.0}, {2.0, 2.2}, {4.0, 2.8}, {6.0, 3.8}, {8.0, 5.0}};
  cones.yellow = {{0.0, -2.0}, {2.0, -2.2}, {4.0, -1.8}, {6.0, -0.2}, {8.0, 1.0}};

  PlannerConfig config;
  config.allow_partial_boundary = true;
  const auto result = buildLocalPath(cones, config);

  if (!expectStrictFinitePath(result, 3.0)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  EXPECT_NEAR(result.waypoints.front().x, 0.0, 0.2);
  EXPECT_NEAR(result.waypoints.front().y, 0.0, 0.2);
  for (std::size_t index = 1; index < result.waypoints.size(); ++index) {
    EXPECT_GE(result.waypoints[index].x, result.waypoints[index - 1U].x);
  }
}

TEST(LocalPathBuilder, SparseMapUsesNearestUsableBoundaryAtReducedSpeed)
{
  ConeSet cones;
  cones.blue = {{6.0, 2.8}, {9.0, 3.0}};
  cones.yellow = {{0.7, -2.6}, {6.0, -1.8}, {9.0, -1.7}};

  PlannerConfig config;
  config.allow_partial_boundary = true;
  const auto result = buildLocalPath(cones, config);

  if (!expectStrictFinitePath(result, 1.5)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kYellowOnly);
  EXPECT_LT(result.waypoints.front().x, 2.0);
}

TEST(LocalPathBuilder, MapModeCanUseConnectedPrefixBeforeUnmappedGap)
{
  ConeSet cones;
  cones.yellow = {{0.0, -1.5}, {2.0, -1.5}, {4.0, -1.5}, {12.0, -1.5}};
  PlannerConfig config;
  config.allow_partial_boundary = true;

  const auto result = buildLocalPath(cones, config);

  if (!expectStrictFinitePath(result, 1.5)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kYellowOnly);
  EXPECT_LE(result.waypoints.back().x, 8.0);
}

TEST(LocalPathBuilder, StrictModeRejectsTruncatedTwoSidedBoundary)
{
  ConeSet cones;
  cones.blue = {{0.0, 2.0}, {2.0, 2.0}, {4.0, 2.0}, {6.0, 2.0}};
  cones.yellow = {{0.0, -2.0}, {2.0, -2.0}, {4.0, -2.0}, {12.0, -2.0}};

  expectInvalidEmptyPath(buildLocalPath(cones));
}

TEST(LocalPathBuilder, MapModeSeedsOneSidedTraversalAtNearestBehindCone)
{
  ConeSet cones;
  cones.yellow = {{-0.8, -1.5}, {2.0, -1.5}, {4.0, -1.5}, {6.0, -1.5}};
  PlannerConfig config;
  config.allow_partial_boundary = true;

  const auto result = buildLocalPath(cones, config);

  if (!expectStrictFinitePath(result, 1.5)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kYellowOnly);
  EXPECT_LT(result.waypoints.front().x, 0.0);
  EXPECT_LT(std::hypot(result.waypoints.front().x, result.waypoints.front().y), 4.0);
}

TEST(LocalPathBuilder, BlueOnlyOffsetsRightAtFallbackSpeed)
{
  ConeSet cones;
  cones.blue = {{0.0, 1.5}, {2.0, 1.5}, {4.0, 1.5}, {6.0, 1.5}};
  const auto result = buildLocalPath(cones);
  if (!expectStrictFinitePath(result, 1.5)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kBlueOnly);
  EXPECT_NEAR(result.waypoints.front().y, 0.0, 1.0e-9);
  EXPECT_LE(result.waypoints.back().s, 8.0);
}

TEST(LocalPathBuilder, YellowOnlyOffsetsLeftAtFallbackSpeed)
{
  ConeSet cones;
  cones.yellow = {{0.0, -1.5}, {2.0, -1.5}, {4.0, -1.5}, {6.0, -1.5}};
  const auto result = buildLocalPath(cones);
  if (!expectStrictFinitePath(result, 1.5)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kYellowOnly);
  EXPECT_NEAR(result.waypoints.front().y, 0.0, 1.0e-9);
  EXPECT_LE(result.waypoints.back().s, 8.0);
}

TEST(LocalPathBuilder, OneSidedGapCannotReturnValidTruncatedPrefix)
{
  ConeSet cones;
  cones.blue = {{0.0, 1.5}, {2.0, 1.5}, {4.0, 1.5}, {12.0, 1.5}};

  const auto result = buildLocalPath(cones);

  expectInvalidEmptyPath(result, "one-sided traversal must not publish the 0,2,4 prefix");
}

TEST(LocalPathBuilder, OneSidedHeadingCannotReturnValidTruncatedPrefix)
{
  ConeSet cones;
  cones.blue = {{0.0, 1.5}, {2.0, 1.5}, {4.0, 1.5}, {5.0, 4.5}};

  const auto result = buildLocalPath(cones);

  expectInvalidEmptyPath(result, "one-sided traversal must not publish before a heading stop");
}

TEST(LocalPathBuilder, NonBoundaryColorsAreIgnoredButCannotFormPath)
{
  auto known = straightTwoSided();
  known.orange = {{1.0, 0.0}, {2.0, 0.0}};
  known.big_orange = {{3.0, 0.0}};
  known.unknown = {{4.0, 0.0}};
  const auto with_noise = buildLocalPath(known);
  if (!expectStrictFinitePath(with_noise, 3.0)) {
    return;
  }

  ConeSet noise_only;
  noise_only.orange = known.orange;
  noise_only.big_orange = known.big_orange;
  noise_only.unknown = known.unknown;
  const auto invalid = buildLocalPath(noise_only);
  ASSERT_TRUE(invalid.evaluated);
  EXPECT_FALSE(invalid.valid);
}

TEST(LocalPathBuilder, OutOfRoiAndNonFiniteConesAreIgnored)
{
  auto usable = straightTwoSided();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  usable.blue.insert(usable.blue.end(), {{nan, 2.0}, {infinity, 2.0}, {21.0, 2.0}, {4.0, 8.1}});
  usable.yellow.insert(
    usable.yellow.end(), {{0.0, nan}, {0.0, infinity}, {-1.1, -2.0}, {4.0, -8.1}});

  const auto result = buildLocalPath(usable);
  if (!expectStrictFinitePath(result, 3.0)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kTwoSided);

  ConeSet unusable;
  unusable.blue = {{nan, 1.5}, {21.0, 1.5}, {0.0, 8.1}};
  unusable.yellow = {{0.0, nan}, {infinity, -1.5}};
  const auto invalid = buildLocalPath(unusable);
  expectInvalidEmptyPath(invalid);
}

TEST(LocalPathBuilder, MixedSparseAndUnknownOnlyFailClosed)
{
  ConeSet mixed;
  mixed.blue = {{0.0, 1.5}, {2.0, 1.5}, {4.0, 1.5}};
  mixed.yellow = {{0.0, -1.5}};
  mixed.unknown = {{1.0, 0.0}, {3.0, 0.0}, {5.0, 0.0}};
  const auto mixed_result = buildLocalPath(mixed);
  ASSERT_TRUE(mixed_result.evaluated);
  EXPECT_FALSE(mixed_result.valid);

  ConeSet unknown_only;
  unknown_only.unknown = mixed.unknown;
  const auto unknown_result = buildLocalPath(unknown_only);
  ASSERT_TRUE(unknown_result.evaluated);
  EXPECT_FALSE(unknown_result.valid);
}

TEST(LocalPathBuilder, PartialBoundarySinglePairBuildsSparsePath)
{
  PlannerConfig config;
  config.allow_partial_boundary = true;
  ConeSet sparse;
  sparse.blue = {{4.0, 1.8}};
  sparse.yellow = {{4.0, -1.8}};

  const auto result = buildLocalPath(sparse, config);
  ASSERT_TRUE(result.evaluated);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  ASSERT_GE(result.waypoints.size(), 5U);
  EXPECT_LE(std::hypot(result.waypoints.front().x, result.waypoints.front().y), 0.5);
  for (const auto & waypoint : result.waypoints) {
    EXPECT_DOUBLE_EQ(waypoint.speed, config.fallback_speed_mps);
  }
}

TEST(LocalPathBuilder, PartialBoundaryTwoConeSideOffsetsPath)
{
  PlannerConfig config;
  config.allow_partial_boundary = true;
  ConeSet sparse;
  sparse.blue = {{2.0, 1.6}, {6.0, 1.7}};

  const auto result = buildLocalPath(sparse, config);
  ASSERT_TRUE(result.evaluated);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kBlueOnly);
  ASSERT_GE(result.waypoints.size(), 5U);
  // Blue is the left boundary: the offset centerline must sit to its right.
  for (const auto & waypoint : result.waypoints) {
    EXPECT_LT(waypoint.y, 1.6);
    EXPECT_DOUBLE_EQ(waypoint.speed, config.fallback_speed_mps);
  }
}

TEST(LocalPathBuilder, StrictModeStillFailsClosedOnSparseInputs)
{
  ConeSet pair_only;
  pair_only.blue = {{4.0, 1.8}};
  pair_only.yellow = {{4.0, -1.8}};
  expectInvalidEmptyPath(buildLocalPath(pair_only));

  ConeSet two_blue;
  two_blue.blue = {{2.0, 1.6}, {6.0, 1.7}};
  expectInvalidEmptyPath(buildLocalPath(two_blue));
}

TEST(LocalPathBuilder, EmptyAndNoiseOnlyInputsFailClosed)
{
  const auto empty = buildLocalPath(ConeSet{});
  expectInvalidEmptyPath(empty);

  ConeSet noise_only;
  noise_only.orange = {{1.0, 0.0}, {2.0, 0.0}};
  noise_only.big_orange = {{3.0, 0.0}};
  noise_only.unknown = {{4.0, 0.0}};
  const auto noise = buildLocalPath(noise_only);
  expectInvalidEmptyPath(noise);
}

}
}
