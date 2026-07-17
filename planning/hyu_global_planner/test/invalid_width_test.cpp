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
eufs_msgs::msg::ConeWithCovariance coneAt(double x, double y)
{
  eufs_msgs::msg::ConeWithCovariance cone;
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
  eufs_msgs::msg::ConeArrayWithCovariance map;
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
// pre-gate — it flows to the centerline builder and fails (or succeeds) on
// its real geometry. This fixture's true defect is loop closure, which the
// brittle per-cone width gate used to mask as "invalid_width".
TEST(InvalidWidth, LocalOutliersDoNotMaskTheRealReason)
{
  const auto map = loadConeMapCsv("hyu_localization/map/map_20260713_002645.csv");
  std::vector<PlannerWaypoint> waypoints{{1.0, 2.0, 3.0, 4.0, 5.0}};
  std::string reason;

  EXPECT_FALSE(
    buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason));
  EXPECT_TRUE(waypoints.empty());
  // Fails closed for its real reason, no longer masked by a couple of
  // off-width cones.
  EXPECT_NE(reason, "invalid_width");
}

}  // namespace hyu_global_planner::test
