#include <gtest/gtest.h>

#include "hyu_global_planner/slam_boundary_ordering.hpp"
#include "slam_test_fixtures.hpp"

namespace hyu_global_planner::test
{

TEST(SlamBoundaryOrdering, OrdersSmallTrackAsClosedBoundaries)
{
  const auto map = loadConeMapCsv("eufs_sim/eufs_tracks/csv/small_track.csv");
  std::vector<PlannerPoint> ordered_blue;
  std::vector<PlannerPoint> ordered_yellow;
  std::string reason;

  ASSERT_TRUE(orderSlamBoundaries(
    bluePoints(map), yellowPoints(map), {0.0, 0.0}, 12.0,
    ordered_blue, ordered_yellow, reason)) << reason;

  ASSERT_EQ(ordered_blue.size(), map.blue_cones.size());
  ASSERT_EQ(ordered_yellow.size(), map.yellow_cones.size());
  EXPECT_LE(distance(ordered_blue.front(), ordered_blue.back()), 5.0);
  EXPECT_LE(distance(ordered_yellow.front(), ordered_yellow.back()), 5.0);
}

TEST(SlamBoundaryOrdering, HandlesPeanutCloseBranchesWithoutLargeBoundaryJump)
{
  const auto map = loadConeMapCsv("eufs_sim/eufs_tracks/csv/peanut.csv");
  std::vector<PlannerPoint> ordered_blue;
  std::vector<PlannerPoint> ordered_yellow;
  std::string reason;

  ASSERT_TRUE(orderSlamBoundaries(
    bluePoints(map), yellowPoints(map), {0.0, 0.0}, 12.0,
    ordered_blue, ordered_yellow, reason)) << reason;

  for (const auto * boundary : {&ordered_blue, &ordered_yellow}) {
    for (std::size_t i = 1; i < boundary->size(); ++i) {
      EXPECT_LE(distance((*boundary)[i - 1U], (*boundary)[i]), 12.0);
    }
    EXPECT_LE(distance(boundary->front(), boundary->back()), 5.0);
  }
}

}  // namespace hyu_global_planner::test
