#include <gtest/gtest.h>

#include "hyu_global_planner/slam_centerline_builder.hpp"
#include "slam_test_fixtures.hpp"

namespace hyu_global_planner::test
{

// This live map carries split-landmark duplicates (same-colour pairs under
// 0.5 m). The builder used to fail the whole map closed as "duplicate_ghost";
// it now collapses each pair to its midpoint and builds the lap — a couple of
// bad landmarks cost those landmarks, not the mission's global path.
TEST(BadMapDuplicate, DuplicatePairsAreMergedAndThePathStillBuilds)
{
  const auto map = loadConeMapCsv("localization/map/map_20260713_004055.csv");
  std::vector<PlannerWaypoint> waypoints;
  std::string reason;

  ASSERT_TRUE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason))
    << reason;

  ASSERT_GE(waypoints.size(), 3U);
  // The repaired path must still respect the corridor: every waypoint keeps a
  // sane distance to both boundaries (same bound the shipped-track tests use).
  for (const auto & waypoint : waypoints) {
    EXPECT_GT(waypoint.d_left, 0.5);
    EXPECT_GT(waypoint.d_right, 0.5);
    EXPECT_LT(waypoint.d_left, fixtureConfig().max_track_width_m);
    EXPECT_LT(waypoint.d_right, fixtureConfig().max_track_width_m);
  }
}

}  // namespace hyu_global_planner::test
