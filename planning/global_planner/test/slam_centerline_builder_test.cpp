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
  const auto map = loadConeMapCsv("hyu_localization/map/map_20260713_002645.csv");
  std::vector<PlannerWaypoint> waypoints{{1.0, 2.0, 3.0, 4.0, 5.0}};
  std::string reason;

  EXPECT_FALSE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason));

  EXPECT_TRUE(waypoints.empty());
  // Fails closed on its real defect (loop closure). The per-cone width gate
  // used to mask this: a couple of off-width cones no longer nuke the map,
  // so the true reason surfaces. See invalid_width_test for the widespread
  // width case that still fails as invalid_width.
  EXPECT_NE(reason, "invalid_width");
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

// The published global path must be a closed loop from EVERY ego position, not
// just the lucky ones. The ring seam lands wherever the boundary walk happened to
// be seeded, and the sweep can leave its two ends metres apart there (measured up
// to 19 m), so accepting whatever the ego seed produced made the SAME map close
// or break depending only on where the car was when it was rebuilt: ~4% of ego
// positions on small_track and ~20% on the seam_fold map published a path broken
// across the start/finish. That is the intermittent break seen live. The builder
// now rejects a seed whose ring does not close and retries from another.
TEST(SlamCenterlineBuilder, ClosesFromEveryEgoPositionOnShippedTracks)
{
  for (const char * track : {"eufs_sim/eufs_tracks/csv/small_track.csv",
      "eufs_sim/eufs_tracks/csv/peanut.csv",
      "planning/global_planner/test/fixtures/seam_fold_cone_map.csv"})
  {
    const auto map = loadConeMapCsv(track);
    std::size_t built = 0U;
    std::size_t open = 0U;
    for (const auto & seed : bluePoints(map)) {
      for (const double lateral : {0.0, 1.5}) {
        nav_msgs::msg::Odometry odom;
        odom.pose.pose.position.x = seed.x;
        odom.pose.pose.position.y = seed.y + lateral;
        odom.pose.pose.orientation.w = 1.0;

        std::vector<PlannerWaypoint> waypoints;
        std::string reason;
        if (!buildCenterlineFromSlamMap(map, odom, fixtureConfig(), waypoints, reason)) {
          ADD_FAILURE() << track << ": build failed at ego (" << odom.pose.pose.position.x
                        << ", " << odom.pose.pose.position.y << "): " << reason;
          continue;
        }
        ++built;
        if (distance(
            {waypoints.front().x, waypoints.front().y},
            {waypoints.back().x, waypoints.back().y}) > fixtureConfig().duplicate_point_tolerance)
        {
          ++open;
        }
      }
    }
    EXPECT_GT(built, 0U) << track;
    EXPECT_EQ(open, 0U) << track << ": " << open << " ego positions published an OPEN loop";
  }
}

// The published path must be a pure function of the cone map. The seed picks the
// cone the boundary walk starts at, which sets the ordering and pairing phase for
// the whole lap, so seeding at the moving car made the SAME frozen map rebuild
// into a different line -- measured up to 0.20 m apart. Every SLAM cone
// refinement bumps the map signature (hashed to the millimetre) and triggers a
// rebuild, and the car has moved by then, so the path stepped ~0.2 m sideways
// under the controller on an essentially unchanged map, and the car shook.
// Pin the invariant outright: the ego must not move the path at all.
TEST(SlamCenterlineBuilder, PathIsIndependentOfEgoPosition)
{
  const auto map = loadConeMapCsv("eufs_sim/eufs_tracks/csv/small_track.csv");
  std::string reason;
  std::vector<PlannerWaypoint> reference;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), reference, reason)) << reason;
  ASSERT_GE(reference.size(), 3U);

  for (const auto & seed : bluePoints(map)) {
    for (const double lateral : {0.0, 2.0}) {
      nav_msgs::msg::Odometry odom;
      odom.pose.pose.position.x = seed.x;
      odom.pose.pose.position.y = seed.y + lateral;
      odom.pose.pose.orientation.w = 1.0;

      std::vector<PlannerWaypoint> candidate;
      ASSERT_TRUE(
        buildCenterlineFromSlamMap(map, odom, fixtureConfig(), candidate, reason)) << reason;
      ASSERT_EQ(candidate.size(), reference.size())
        << "ego (" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y
        << ") changed the path";
      for (std::size_t i = 0; i < reference.size(); ++i) {
        ASSERT_EQ(candidate[i].x, reference[i].x) << "ego moved the path at waypoint " << i;
        ASSERT_EQ(candidate[i].y, reference[i].y) << "ego moved the path at waypoint " << i;
      }
    }
  }
}

// The start/finish gate's two cones are closer to EACH OTHER than the boundaries
// are to either of them: the gate spans the track (~4 m) while the blue/yellow
// lines fall back on both sides of it to leave room for the timing equipment
// (~4.8 m on autocross_kase2026). So folding one marker in and then letting it
// serve as a "nearest cone" when placing the next puts BOTH gate cones on the
// SAME side: the ordering walk jumps the track there, the pairing collapses, and
// over a quarter of the pairs leave the width band -- the whole map is rejected
// as invalid_width and no global path is published at all. Sides must be decided
// against the ORIGINAL boundaries. (small_track's gate sits only 2.7 m from its
// boundaries, so it happened to survive this; the KASE tracks did not.)
TEST(SlamCenterlineBuilder, WideStartFinishGateFoldsToOppositeSides)
{
  constexpr double kPiLocal = 3.14159265358979323846;
  eufs_msgs::msg::ConeArrayWithCovariance map;
  constexpr std::size_t kCones = 40U;
  for (std::size_t i = 0; i < kCones; ++i) {
    const double angle = 2.0 * kPiLocal * static_cast<double>(i) / static_cast<double>(kCones);
    // Fall the boundaries back from the gate at angle 0, as a real gate does.
    if (std::abs(std::atan2(std::sin(angle), std::cos(angle))) * 20.0 < 5.0) {
      continue;
    }
    eufs_msgs::msg::ConeWithCovariance blue;
    blue.point.x = 22.0 * std::cos(angle);
    blue.point.y = 22.0 * std::sin(angle);
    map.blue_cones.push_back(blue);
    eufs_msgs::msg::ConeWithCovariance yellow;
    yellow.point.x = 18.0 * std::cos(angle);
    yellow.point.y = 18.0 * std::sin(angle);
    map.yellow_cones.push_back(yellow);
  }
  for (const double offset : {0.0, 0.5}) {      // the paired timing cones
    for (const double radius : {18.0, 22.0}) {  // across the track
      eufs_msgs::msg::ConeWithCovariance cone;
      cone.point.x = radius;
      cone.point.y = offset;
      map.big_orange_cones.push_back(cone);
    }
  }
  ASSERT_EQ(map.big_orange_cones.size(), 4U);

  std::vector<PlannerWaypoint> waypoints;
  std::string reason;
  ASSERT_TRUE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason))
    << "the gate folded to one side and took the map down: " << reason;

  // Both markers landed on their own side, so the centerline still runs down the
  // middle of the corridor instead of being dragged across it at the gate.
  for (const auto & waypoint : waypoints) {
    ASSERT_NEAR(std::hypot(waypoint.x, waypoint.y), 20.0, 1.5)
      << "centerline left the corridor: the gate folded to one side";
  }
}

}  // namespace global_planner::test
