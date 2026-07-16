#include <gtest/gtest.h>

#include "global_planner/slam_centerline_builder.hpp"
#include "slam_test_fixtures.hpp"

namespace global_planner::test
{

TEST(BadMapDuplicate, FailsClosedWithExactReason)
{
  const auto map = loadConeMapCsv("eufs_graph_slam/map/map_20260713_004055.csv");
  std::vector<PlannerWaypoint> waypoints{{1.0, 2.0, 3.0, 4.0, 5.0}};
  std::string reason;

  EXPECT_FALSE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason));

  EXPECT_TRUE(waypoints.empty());
  EXPECT_EQ(reason, "duplicate_ghost");
}

}  // namespace global_planner::test
