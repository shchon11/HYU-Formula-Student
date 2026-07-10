#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "local_planner/input_policy.hpp"
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

HeaderMetadata header(const std::string & frame, std::int32_t sec, std::uint32_t nanosec, double age)
{
  return HeaderMetadata{frame, sec, nanosec, age};
}

OdomMetadata odom(double age = 0.0)
{
  return OdomMetadata{header("map", 10, 0, age), "base_footprint", {1.0, 2.0}, 0.25};
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

TEST(LocalPathBuilder, TwoSidedDelaunayUsesDeterministicTraversalAndDedupe)
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

  ASSERT_TRUE(result.evaluated);
  EXPECT_FALSE(result.valid) << "one-sided traversal must not publish the 0,2,4 prefix";
  EXPECT_TRUE(result.waypoints.empty());
}

TEST(LocalPathBuilder, OneSidedHeadingCannotReturnValidTruncatedPrefix)
{
  ConeSet cones;
  cones.blue = {{0.0, 1.5}, {2.0, 1.5}, {4.0, 1.5}, {5.0, 4.5}};

  const auto result = buildLocalPath(cones);

  ASSERT_TRUE(result.evaluated);
  EXPECT_FALSE(result.valid) << "one-sided traversal must not publish before a heading stop";
  EXPECT_TRUE(result.waypoints.empty());
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

TEST(LocalPathBuilder, ExcessiveGapHeadingAndSelfIntersectionFailClosed)
{
  ConeSet gap;
  gap.blue = {{0.0, 1.5}, {1.0, 1.5}, {8.0, 1.5}};
  const auto gap_result = buildLocalPath(gap);
  ASSERT_TRUE(gap_result.evaluated);
  EXPECT_FALSE(gap_result.valid);

  ConeSet heading;
  heading.yellow = {{0.0, -1.5}, {1.0, -1.5}, {1.1, 2.0}};
  const auto heading_result = buildLocalPath(heading);
  ASSERT_TRUE(heading_result.evaluated);
  EXPECT_FALSE(heading_result.valid);

  EXPECT_TRUE(pathSelfIntersects({{0.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {2.0, 0.0}}));
}

TEST(InputPolicy, LiveInputRejectsWrongFramesZeroStampsSkewAndStaleness)
{
  const auto valid = validateLiveInput(header("base_footprint", 10, 50000000U, 0.1), odom());
  ASSERT_TRUE(valid.evaluated);
  EXPECT_TRUE(valid.valid) << valid.reason;

  EXPECT_FALSE(validateLiveInput(header("camera", 10, 0, 0.0), odom()).valid);
  EXPECT_FALSE(validateLiveInput(header("base_footprint", 0, 0, 0.0), odom()).valid);
  EXPECT_FALSE(
    validateLiveInput(header("base_footprint", 10, 110000001U, 0.0), odom()).valid);
  EXPECT_FALSE(
    validateLiveInput(header("base_footprint", 10, 0, 0.51), odom()).valid);
  auto wrong_odom = odom();
  wrong_odom.header.frame_id = "odom";
  EXPECT_FALSE(validateLiveInput(header("base_footprint", 10, 0, 0.0), wrong_odom).valid);
}

TEST(InputPolicy, SlamMapUsesLatchedMapAndFreshOdomWithoutAutoSwitch)
{
  const auto latched_map = header("map", 1, 1, 50.0);
  const auto valid = validateSlamMapInput(latched_map, odom(0.1));
  ASSERT_TRUE(valid.evaluated);
  EXPECT_TRUE(valid.valid) << valid.reason;
  EXPECT_TRUE(acceptsSource(SourceMode::kSlamMap, SourceMode::kSlamMap));
  EXPECT_FALSE(acceptsSource(SourceMode::kSlamMap, SourceMode::kLiveCones));
  EXPECT_FALSE(acceptsSource(SourceMode::kLiveCones, SourceMode::kSlamMap));
  EXPECT_THROW(parseSourceMode("automatic"), std::invalid_argument);
  EXPECT_FALSE(validateSlamMapInput(header("base_footprint", 1, 1, 0.0), odom()).valid);
  EXPECT_FALSE(validateSlamMapInput(header("map", 0, 0, 0.0), odom()).valid);
  EXPECT_FALSE(validateSlamMapInput(latched_map, odom(0.51)).valid);
}

}
}
