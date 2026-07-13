#include <cstddef>
#include <cmath>

#include <gtest/gtest.h>

#include "local_planner/local_path_builder.hpp"

namespace local_planner
{

TEST(LocalPathBuilderRobustness, OversizedBoundaryInputFailsClosed)
{
  ConeSet cones;
  for (std::size_t index = 0; index <= kMaxBoundaryCones; ++index) {
    cones.blue.push_back({0.01 * static_cast<double>(index), 1.5});
  }

  const auto result = buildLocalPath(cones);

  ASSERT_TRUE(result.evaluated);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.waypoints.empty());
  EXPECT_EQ(result.reason, "cone input exceeds bounded planner capacity");
}

TEST(LocalPathBuilderRobustness, ExcessiveGapHeadingAndSelfIntersectionFailClosed)
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

TEST(LocalPathBuilderRobustness, UTurnOneSidedBoundaryProducesContinuousLocalPath)
{
  ConeSet cones;
  cones.blue = {{0.0, 1.5}, {2.0, 1.5}, {2.0, 3.5}, {0.0, 3.5}};

  const auto result = buildLocalPath(cones);

  ASSERT_TRUE(result.evaluated);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kBlueOnly);
  ASSERT_GE(result.waypoints.size(), 5U);
  double max_curvature = 0.0;
  for (const auto & waypoint : result.waypoints) {
    EXPECT_TRUE(std::isfinite(waypoint.x));
    EXPECT_TRUE(std::isfinite(waypoint.y));
    EXPECT_TRUE(std::isfinite(waypoint.psi));
    EXPECT_TRUE(std::isfinite(waypoint.kappa));
    max_curvature = std::max(max_curvature, std::abs(waypoint.kappa));
  }
  EXPECT_GT(max_curvature, 0.35);
  EXPECT_TRUE(result.reason.empty());
}

TEST(LocalPathBuilderRobustness, CloseReturnBranchDoesNotPullCenterlineAcrossLane)
{
  ConeSet cones;
  cones.blue = {
    {0.0, 2.0}, {2.0, 2.0}, {4.0, 2.0}, {6.0, 2.0}, {8.0, 2.0},
    {8.0, 6.0}, {6.0, 6.0}, {4.0, 6.0}, {2.0, 6.0}, {0.0, 6.0}};
  cones.yellow = {
    {0.0, -2.0}, {2.0, -2.0}, {4.0, -2.0}, {6.0, -2.0}, {8.0, -2.0},
    {8.0, 4.0}, {6.0, 4.0}, {4.0, 4.0}, {2.0, 4.0}, {0.0, 4.0}};

  const auto result = buildLocalPath(cones);

  ASSERT_TRUE(result.evaluated);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  ASSERT_GE(result.waypoints.size(), 5U);
  EXPECT_NEAR(result.waypoints.front().y, 0.0, 0.3);
  for (const auto & waypoint : result.waypoints) {
    if (waypoint.s <= 6.0) {
      EXPECT_NEAR(waypoint.y, 0.0, 0.6);
    }
  }
}

TEST(LocalPathBuilderRobustness, SparseFallbackRejectsSameSideOppositeLanePairs)
{
  PlannerConfig config;
  config.allow_partial_boundary = true;
  ConeSet cones;
  cones.blue = {{1.0, 2.0}};
  cones.yellow = {{1.0, -2.0}, {1.0, 4.0}};

  const auto result = buildLocalPath(cones, config);

  ASSERT_TRUE(result.evaluated);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  ASSERT_GE(result.waypoints.size(), 5U);
  for (const auto & waypoint : result.waypoints) {
    EXPECT_NEAR(waypoint.y, 0.0, 0.6);
  }
}

TEST(LocalPathBuilderRobustness, TopologyGapReportsLocalTopologyReason)
{
  ConeSet cones;
  cones.blue = {{0.0, 1.5}, {2.0, 1.5}, {9.0, 1.5}};

  const auto result = buildLocalPath(cones);

  ASSERT_TRUE(result.evaluated);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.waypoints.empty());
  EXPECT_EQ(result.reason, "local_topology_gap");
}

TEST(LocalPathBuilderRobustness, MalformedBranchJumpReportsLocalTopologyGap)
{
  ConeSet cones;
  cones.blue = {{0.0, 1.5}, {2.0, 1.5}, {2.0, 3.5}, {9.0, 3.5}};

  const auto result = buildLocalPath(cones);

  ASSERT_TRUE(result.evaluated);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.waypoints.empty());
  EXPECT_EQ(result.reason, "local_topology_gap");
}

TEST(LocalPathBuilderRobustness, CloseUTurnBranchAmbiguityReportsReason)
{
  ConeSet cones;
  cones.blue = {{0.0, 1.5}, {2.0, 1.5}, {2.0, -0.5}, {2.0, 3.5}, {0.0, 3.5}};

  const auto result = buildLocalPath(cones);

  ASSERT_TRUE(result.evaluated);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.waypoints.empty());
  EXPECT_EQ(result.reason, "u_turn_branch_ambiguous");
}

}
