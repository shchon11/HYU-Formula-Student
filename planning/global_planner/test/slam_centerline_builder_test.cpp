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

namespace
{

double maxAdjacentHeadingStepRad(const std::vector<PlannerWaypoint> & waypoints)
{
  constexpr double kPi = 3.14159265358979323846;
  double worst = 0.0;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    double step = std::abs(waypoints[i].psi - waypoints[i - 1].psi);
    if (step > kPi) {
      step = 2.0 * kPi - step;
    }
    worst = std::max(worst, step);
  }
  return worst;
}

}  // namespace

// Regression for the seam fold-back "Z": each ring is seeded at its own
// nearest-to-ego cone, so with a phase offset between the rings the yellow
// arc ends before the blue sweep does. The monotonic projection then pins the
// final pairs to yellow.back(), the centerline tail drifts sideways past the
// start, and the blind closing chord doubles back against travel.
TEST(SlamCenterlineBuilder, SeamPhaseOffsetLoopDoesNotFoldBackAtSeam)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kInner = 18.0;
  constexpr double kOuter = 22.0;
  constexpr int kCones = 40;
  constexpr double kYellowPhaseOffset = 0.75 * 2.0 * kPi / static_cast<double>(kCones);
  eufs_msgs::msg::ConeArrayWithCovariance map;
  for (int i = 0; i < kCones; ++i) {
    const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kCones);
    eufs_msgs::msg::ConeWithCovariance blue;
    blue.point.x = kInner * std::cos(angle);
    blue.point.y = kInner * std::sin(angle);
    map.blue_cones.push_back(blue);
    eufs_msgs::msg::ConeWithCovariance yellow;
    yellow.point.x = kOuter * std::cos(angle + kYellowPhaseOffset);
    yellow.point.y = kOuter * std::sin(angle + kYellowPhaseOffset);
    map.yellow_cones.push_back(yellow);
  }

  nav_msgs::msg::Odometry ego;
  ego.pose.pose.position.x = 20.0;
  ego.pose.pose.position.y = 0.0;
  ego.pose.pose.orientation.w = 1.0;

  std::vector<PlannerWaypoint> waypoints;
  std::string reason;
  ASSERT_TRUE(buildCenterlineFromSlamMap(map, ego, fixtureConfig(), waypoints, reason)) << reason;
  ASSERT_GE(waypoints.size(), 3U);
  EXPECT_LT(maxAdjacentHeadingStepRad(waypoints), 60.0 * kPi / 180.0)
    << "centerline folds back on itself at the seam";
  EXPECT_LE(
    distance({waypoints.front().x, waypoints.front().y}, {waypoints.back().x, waypoints.back().y}),
    fixtureConfig().close_loop_distance_m);
  for (const auto & waypoint : waypoints) {
    const double radius = std::hypot(waypoint.x, waypoint.y);
    EXPECT_NEAR(radius, 20.0, 1.0) << "centerline left the corridor near the seam";
  }
}

// Same defect captured from a live run (small_track SLAM map, 2026-07-13):
// the published global path retraced ~2 m at the seam with a 157 deg turn.
TEST(SlamCenterlineBuilder, LiveSlamMapDoesNotFoldBackAtSeam)
{
  constexpr double kPi = 3.14159265358979323846;
  const auto map = loadConeMapCsv("planning/global_planner/test/fixtures/seam_fold_cone_map.csv");

  // Ego where the car sat when this map converged; the seam forms here.
  nav_msgs::msg::Odometry ego;
  ego.pose.pose.position.x = 4.1;
  ego.pose.pose.position.y = 16.8;
  ego.pose.pose.orientation.w = 1.0;

  std::vector<PlannerWaypoint> waypoints;
  std::string reason;
  ASSERT_TRUE(buildCenterlineFromSlamMap(map, ego, fixtureConfig(), waypoints, reason)) << reason;
  ASSERT_GE(waypoints.size(), 3U);
  EXPECT_LT(maxAdjacentHeadingStepRad(waypoints), 60.0 * kPi / 180.0)
    << "centerline folds back on itself at the seam";
  EXPECT_LE(
    distance({waypoints.front().x, waypoints.front().y}, {waypoints.back().x, waypoints.back().y}),
    fixtureConfig().close_loop_distance_m);
  EXPECT_FALSE(hasSelfIntersection(waypoints));
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

// FS marks the start/finish line with big orange cones, and the blue/yellow
// boundaries both break across it (5.9 m blue / 5.3 m yellow on small_track).
// The car starts ON that line, so the boundary walk seeds beside the gate: it
// runs away around the lap and ends on the far side, putting the ring seam on
// the gate gap instead of on ordinary cone spacing. With the markers dropped
// the seam exceeds close_loop_distance_m, the ring is never closed, and the
// global path is published with a visible break across the start/finish.
// Folding the markers into the boundary they flank halves that gap.
TEST(SlamCenterlineBuilder, StartFinishGateIsBridgedFromCarStart)
{
  const std::string track = "eufs_sim/eufs_tracks/csv/small_track.csv";
  const auto map = loadConeMapCsv(track);
  ASSERT_EQ(map.big_orange_cones.size(), 4U) << "small_track should ship a 4-cone start/finish gate";

  std::vector<PlannerWaypoint> waypoints;
  std::string reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoAtCarStart(track), fixtureConfig(), waypoints, reason))
    << reason;

  const double seam = distance(
    {waypoints.front().x, waypoints.front().y}, {waypoints.back().x, waypoints.back().y});
  EXPECT_LE(seam, fixtureConfig().close_loop_distance_m)
    << "ring seam " << seam << " m sits on the start/finish gate, so the loop never closes and "
    << "the global path is published broken there";
  EXPECT_FALSE(hasSelfIntersection(waypoints));

  // The gate must carry real centerline samples, not be spanned by a blind chord.
  PlannerPoint gate{0.0, 0.0};
  for (const auto & cone : map.big_orange_cones) {
    gate.x += cone.point.x / static_cast<double>(map.big_orange_cones.size());
    gate.y += cone.point.y / static_cast<double>(map.big_orange_cones.size());
  }
  double nearest_sample = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : waypoints) {
    nearest_sample = std::min(nearest_sample, distance(gate, {waypoint.x, waypoint.y}));
  }
  EXPECT_LE(nearest_sample, fixtureConfig().waypoint_spacing_m)
    << "no centerline sample at the start/finish gate";
}

}  // namespace global_planner::test
