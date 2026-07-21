#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "hyu_global_planner/slam_centerline_builder.hpp"
#include "slam_test_fixtures.hpp"

namespace hyu_global_planner::test
{

namespace
{
hyu_msgs::msg::ConeWithCovariance coneAt(double x, double y)
{
  hyu_msgs::msg::ConeWithCovariance cone;
  cone.point.x = x;
  cone.point.y = y;
  cone.point.z = 0.0;
  return cone;
}
}  // namespace

// A WIDESPREAD width failure (every cone off-band) must still fail closed:
// two concentric rings pulled far too wide are a genuinely wrong map
// (crossed/swapped boundaries), not the local outliers the gate now tolerates.
TEST(InvalidWidth, WidespreadWidthFailureFailsClosed)
{
  hyu_msgs::msg::ConeArrayWithCovariance map;
  constexpr int kCount = 24;
  constexpr double kPi = 3.14159265358979323846;
  for (int i = 0; i < kCount; ++i) {
    const double a = 2.0 * kPi * i / kCount;
    // Inner ring blue at r=5, outer yellow at r=20: every nearest width is
    // ~15 m, far past max_track_width_m (6.0).
    map.blue_cones.push_back(coneAt(5.0 * std::cos(a), 5.0 * std::sin(a)));
    map.yellow_cones.push_back(coneAt(20.0 * std::cos(a), 20.0 * std::sin(a)));
  }
  std::vector<PlannerWaypoint> waypoints{{1.0, 2.0, 3.0, 4.0, 5.0}};
  std::string reason;

  EXPECT_FALSE(
    buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason));
  EXPECT_TRUE(waypoints.empty());
  EXPECT_EQ(reason, "invalid_width");
}

// A map with only a FEW off-width cones must NOT be rejected by the width
// pre-gate. This fixture used to fail "loop_not_closed" only because its two
// rings closed from different retry seeds, leaving the seams half a lap apart
// for the pairing sweep; with the seams re-anchored at the map origin it
// builds — the handful of outliers costs nothing.
TEST(InvalidWidth, LocalOutliersDoNotCostThePath)
{
  const auto map = loadConeMapCsv("localization/map/map_20260713_002645.csv");
  std::vector<PlannerWaypoint> waypoints;
  std::string reason;

  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason)) << reason;
  ASSERT_GE(waypoints.size(), 3U);
  // This map carries a drift-narrowed pair (1.68 m blue<->yellow) the repair
  // deliberately keeps — real cones at a hairpin pinch measure similarly, so
  // only sub-footprint pairs are dropped. The corridor is honestly tight
  // there (d ~0.03 m at three samples), never inverted or invented.
  for (const auto & waypoint : waypoints) {
    EXPECT_GE(waypoint.d_left, 0.0);
    EXPECT_GE(waypoint.d_right, 0.0);
    EXPECT_LT(waypoint.d_left, fixtureConfig().max_track_width_m);
    EXPECT_LT(waypoint.d_right, fixtureConfig().max_track_width_m);
  }
}

}  // namespace hyu_global_planner::test
