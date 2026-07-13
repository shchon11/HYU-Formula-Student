#include <gtest/gtest.h>

#include "global_planner/slam_centerline_builder.hpp"
#include "slam_test_fixtures.hpp"

namespace global_planner::test
{

class SlamCenterlineFixtureTest : public ::testing::TestWithParam<const char *>
{
};

TEST_P(SlamCenterlineFixtureTest, ShippedTrackBuildsClosedNonIntersectingCenterline)
{
  const auto relative_path = std::string(GetParam());
  const auto map = loadConeMapCsv(relative_path);
  const auto blue = bluePoints(map);
  const auto yellow = yellowPoints(map);
  std::vector<PlannerWaypoint> waypoints;
  std::string reason;

  ASSERT_TRUE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason))
    << relative_path << ": " << reason;

  ASSERT_GE(waypoints.size(), 3U);
  EXPECT_LE(
    distance({waypoints.front().x, waypoints.front().y}, {waypoints.back().x, waypoints.back().y}),
    fixtureConfig().close_loop_distance_m);
  EXPECT_FALSE(hasSelfIntersection(waypoints));

  for (const double width : fixturePairingWidths(map, fixtureConfig())) {
    EXPECT_GE(width, fixtureConfig().min_track_width_m) << relative_path;
    EXPECT_LE(width, fixtureConfig().max_track_width_m) << relative_path;
  }
}

INSTANTIATE_TEST_SUITE_P(
  ShippedCsvTracks,
  SlamCenterlineFixtureTest,
  ::testing::Values(
    "eufs_sim/eufs_tracks/csv/small_track.csv",
    "eufs_sim/eufs_tracks/csv/peanut.csv"));

// Regression for the arc-pinning bug: on a closed loop the ordered yellow's
// front (arc 0) and back (arc = full length) are physically adjacent at the
// seam next to the ego. Without a forward search window the first blue sample
// projects onto yellow.back(), pinning every later pair to the seam so the
// width blows past max_track_width and a geometrically perfect loop is
// rejected with "invalid_width". A clean concentric ring must build a
// centerline that hugs the mid-radius the whole way around.
TEST(SlamCenterlineBuilder, ClosedLoopWithSeamNearEgoBuildsValidCenterline)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kInner = 18.0;
  constexpr double kOuter = 22.0;
  constexpr int kCones = 40;
  eufs_msgs::msg::ConeArrayWithCovariance map;
  for (int i = 0; i < kCones; ++i) {
    const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kCones);
    eufs_msgs::msg::ConeWithCovariance blue;
    blue.point.x = kInner * std::cos(angle);
    blue.point.y = kInner * std::sin(angle);
    map.blue_cones.push_back(blue);
    eufs_msgs::msg::ConeWithCovariance yellow;
    yellow.point.x = kOuter * std::cos(angle);
    yellow.point.y = kOuter * std::sin(angle);
    map.yellow_cones.push_back(yellow);
  }

  // Ego on the mid-radius at angle 0 -> ordering seeds both rings at the seam
  // right beside the ego, the exact condition that triggered the pin.
  nav_msgs::msg::Odometry ego;
  ego.pose.pose.position.x = 20.0;
  ego.pose.pose.position.y = 0.0;
  ego.pose.pose.orientation.w = 1.0;

  std::vector<PlannerWaypoint> waypoints;
  std::string reason;
  ASSERT_TRUE(buildCenterlineFromSlamMap(map, ego, fixtureConfig(), waypoints, reason)) << reason;
  ASSERT_GE(waypoints.size(), 3U);
  for (const auto & waypoint : waypoints) {
    const double radius = std::hypot(waypoint.x, waypoint.y);
    EXPECT_NEAR(radius, 20.0, 1.0) << "centerline left the corridor (arc-pinning regression)";
  }
}

TEST(SlamCenterlineBuilder, WidthFixtureFailsClosedWithExplicitReason)
{
  const auto map = loadConeMapCsv("eufs_graph_slam/map/map_20260713_002645.csv");
  std::vector<PlannerWaypoint> waypoints{{1.0, 2.0, 3.0, 4.0, 5.0}};
  std::string reason;

  EXPECT_FALSE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason));

  EXPECT_TRUE(waypoints.empty());
  EXPECT_EQ(reason, "invalid_width");
}

}  // namespace global_planner::test
