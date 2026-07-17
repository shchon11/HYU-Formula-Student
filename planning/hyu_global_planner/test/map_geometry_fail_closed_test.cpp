#include <gtest/gtest.h>

#include "hyu_global_planner/slam_centerline_builder.hpp"
#include "slam_test_fixtures.hpp"

namespace hyu_global_planner::test
{

TEST(MapGeometryFailClosed, DuplicateGhostFixtureReturnsReasonAndNoWaypoints)
{
  const auto map = loadConeMapCsv("hyu_localization/map/map_20260713_004055.csv");
  std::vector<PlannerWaypoint> waypoints{{1.0, 2.0, 3.0, 4.0, 5.0}};
  std::string reason;

  EXPECT_FALSE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason));

  EXPECT_TRUE(waypoints.empty());
  EXPECT_EQ(reason, "duplicate_ghost");
}

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

}  // namespace hyu_global_planner::test
