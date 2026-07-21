#include <gtest/gtest.h>

#include "hyu_global_planner/slam_centerline_builder.hpp"
#include "slam_test_fixtures.hpp"

namespace hyu_global_planner::test
{

// The self-repair passes (duplicate merge, ghost drop, walk rescue) are all
// bounded to stray minorities — a map that is defective at its core must still
// fail closed with a reason, never publish invented geometry.
TEST(MapGeometryFailClosed, TooFewConesReturnsReasonAndNoWaypoints)
{
  hyu_msgs::msg::ConeArrayWithCovariance map;
  hyu_msgs::msg::ConeWithCovariance cone;
  cone.point.x = 0.0;
  cone.point.y = 0.0;
  map.blue_cones.push_back(cone);
  map.yellow_cones.push_back(cone);
  std::vector<PlannerWaypoint> waypoints{{1.0, 2.0, 3.0, 4.0, 5.0}};
  std::string reason;

  EXPECT_FALSE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason));

  EXPECT_TRUE(waypoints.empty());
  EXPECT_EQ(reason, "insufficient blue/yellow cones");
}

// A side that collapses to fewer than min_cones_per_side after the duplicate
// merge is a degenerate map, not a repairable one: merging must not talk the
// gate below it out of failing closed.
TEST(MapGeometryFailClosed, DuplicateMergeStillFailsClosedWhenTooLittleRemains)
{
  hyu_msgs::msg::ConeArrayWithCovariance map;
  hyu_msgs::msg::ConeWithCovariance cone;
  // Three blue "cones" that are really one physical cone split three ways.
  for (double dx : {0.0, 0.2, 0.4}) {
    cone.point.x = dx;
    cone.point.y = 0.0;
    map.blue_cones.push_back(cone);
  }
  for (double x : {0.0, 4.0, 8.0}) {
    cone.point.x = x;
    cone.point.y = 3.0;
    map.yellow_cones.push_back(cone);
  }
  std::vector<PlannerWaypoint> waypoints{{1.0, 2.0, 3.0, 4.0, 5.0}};
  std::string reason;

  EXPECT_FALSE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason));

  EXPECT_TRUE(waypoints.empty());
  EXPECT_EQ(reason, "insufficient blue/yellow cones");
}

}  // namespace hyu_global_planner::test
