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

// Concentric ring corridor (blue r=18 inner, yellow r=22 outer) — the same
// clean base geometry the seam-pinning regression test uses.
hyu_msgs::msg::ConeArrayWithCovariance ringMap()
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr int kCones = 40;
  hyu_msgs::msg::ConeArrayWithCovariance map;
  for (int i = 0; i < kCones; ++i) {
    const double a = 2.0 * kPi * i / kCones;
    map.blue_cones.push_back(coneAt(18.0 * std::cos(a), 18.0 * std::sin(a)));
    map.yellow_cones.push_back(coneAt(22.0 * std::cos(a), 22.0 * std::sin(a)));
  }
  return map;
}

void expectFullRingCenterline(
  const hyu_msgs::msg::ConeArrayWithCovariance & map, const char * what)
{
  nav_msgs::msg::Odometry ego;
  ego.pose.pose.position.x = 20.0;
  ego.pose.pose.orientation.w = 1.0;
  std::vector<PlannerWaypoint> waypoints;
  std::string reason;
  ASSERT_TRUE(buildCenterlineFromSlamMap(map, ego, fixtureConfig(), waypoints, reason))
    << what << ": " << reason;
  ASSERT_GE(waypoints.size(), 3U) << what;
  for (const auto & waypoint : waypoints) {
    EXPECT_NEAR(std::hypot(waypoint.x, waypoint.y), 20.0, 1.0)
      << what << ": centerline left the corridor";
  }
}

}  // namespace

// A split landmark (two same-colour cones 0.3 m apart) used to fail the whole
// map closed as duplicate_ghost. The pair is one physical cone; it must merge
// and the path must come out on the true mid-radius.
TEST(MapSelfRepair, SplitLandmarkPairIsMergedNotFatal)
{
  auto map = ringMap();
  const auto & first = map.blue_cones.front();
  map.blue_cones.push_back(coneAt(first.point.x + 0.3, first.point.y));
  expectFullRingCenterline(map, "split landmark");
}

// A ghost landmark far off the corridor (created while the pose estimate was
// transiently wrong) used to strand the ordering walk: it is gap-connected to
// its boundary, so the disconnection gate kept it, but no walk can thread it
// and return. It must be dropped, not fatal.
TEST(MapSelfRepair, OffCorridorGhostIsDroppedNotFatal)
{
  auto map = ringMap();
  // 11 m inside the blue ring: within max_boundary_gap of the boundary (so
  // not "disconnected"), but 10+ m from every yellow cone.
  map.blue_cones.push_back(coneAt(7.0, 0.0));
  expectFullRingCenterline(map, "off-corridor ghost");
}

// Both defects of the 2026-07-20 live map at once: an infield yellow ghost and
// same-colour strays the heading-aware walk bypasses and then cannot reach
// (observed reasons over the session: duplicate_ghost while the split pair
// lived, then branch_jump, then invalid_width from mismatched ring seams).
// The full lap must build and stay inside the cone corridor.
TEST(MapSelfRepair, LiveMapWithGhostAndStrandedConesBuildsFullLap)
{
  const auto map = loadConeMapCsv("localization/map/map_20260720_233754.csv");
  std::vector<PlannerWaypoint> waypoints;
  std::string reason;

  ASSERT_TRUE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason))
    << reason;

  // Full lap: this track is ~300 m of centerline at 0.5 m spacing.
  ASSERT_GE(waypoints.size(), 400U);
  double length = 0.0;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    length += std::hypot(
      waypoints[i].x - waypoints[i - 1].x, waypoints[i].y - waypoints[i - 1].y);
  }
  EXPECT_GT(length, 250.0);
  EXPECT_LT(length, 350.0);
  for (const auto & waypoint : waypoints) {
    EXPECT_GT(waypoint.d_left, 0.5);
    EXPECT_GT(waypoint.d_right, 0.5);
    EXPECT_LT(waypoint.d_left, fixtureConfig().max_track_width_m);
    EXPECT_LT(waypoint.d_right, fixtureConfig().max_track_width_m);
  }
}

// A lap-seam re-registration: one cone per boundary re-observed ~1.2 m off
// its original under the closed drift. Too far apart to merge as a split
// landmark, and the two ghosts mutually read as a plausible track width, so
// the corridor filter kept both — the walk folded the centerline and every
// seed failed with a heading reversal. The laterally misfitting copy must be
// dropped and the lap must build.
TEST(MapSelfRepair, DriftShiftedSeamDuplicatesAreDroppedNotFatal)
{
  auto map = ringMap();
  // Radial shifts: crowding the twin (under half the ring spacing) while
  // sitting clearly off the line through the neighbouring boundary cones.
  map.blue_cones.push_back(coneAt(19.2, 0.0));
  map.yellow_cones.push_back(coneAt(20.8, 0.0));
  expectFullRingCenterline(map, "drift-shifted seam duplicates");
}

// The live map that motivated the drift-duplicate repair: restored from
// map_20260722_030333, its final two rows are a blue/yellow pair re-registered
// ~2 m off their boundaries at the lap seam. The full 439 m lap must build.
TEST(MapSelfRepair, LiveMapWithDriftShiftedSeamDuplicatesBuildsFullLap)
{
  const auto map = loadConeMapCsv(
    "planning/hyu_global_planner/test/fixtures/drift_duplicate_cone_map.csv");
  std::vector<PlannerWaypoint> waypoints;
  std::string reason;

  ASSERT_TRUE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason))
    << reason;

  ASSERT_GE(waypoints.size(), 800U);
  double length = 0.0;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    length += std::hypot(
      waypoints[i].x - waypoints[i - 1].x, waypoints[i].y - waypoints[i - 1].y);
  }
  EXPECT_GT(length, 420.0);
  EXPECT_LT(length, 460.0);
  for (const auto & waypoint : waypoints) {
    EXPECT_GT(waypoint.d_left, 0.5);
    EXPECT_GT(waypoint.d_right, 0.5);
    EXPECT_LT(waypoint.d_left, fixtureConfig().max_track_width_m);
    EXPECT_LT(waypoint.d_right, fixtureConfig().max_track_width_m);
  }
}

// its_a_mess: the blue boundary is a dumbbell outline whose waist is two
// parallel rows 0.75 m apart with 1.0-1.5 m along-row spacing, so the greedy
// walk's nearest-cone choice zigzags between the rows. The ground truth only
// squeaked through the rescue tier; this live SLAM map (median 0.12 m noise,
// one unmapped bridge cone) failed every seed as branch_jump. The 2-opt ring
// repair phase must recover the full ~103 m lap.
TEST(MapSelfRepair, DumbbellWaistBoundaryIsRepairedNotFatal)
{
  const auto map = loadConeMapCsv(
    "planning/hyu_global_planner/test/fixtures/dumbbell_waist_cone_map.csv");
  std::vector<PlannerWaypoint> waypoints;
  std::string reason;

  ASSERT_TRUE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason))
    << reason;

  ASSERT_GE(waypoints.size(), 180U);
  double length = 0.0;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    length += std::hypot(
      waypoints[i].x - waypoints[i - 1].x, waypoints[i].y - waypoints[i - 1].y);
  }
  EXPECT_GT(length, 90.0);
  EXPECT_LT(length, 115.0);
  for (const auto & waypoint : waypoints) {
    EXPECT_GT(waypoint.d_left, 0.5);
    EXPECT_GT(waypoint.d_right, 0.5);
    EXPECT_LT(waypoint.d_left, fixtureConfig().max_track_width_m);
    EXPECT_LT(waypoint.d_right, fixtureConfig().max_track_width_m);
  }
}

// The ground-truth its_a_mess track built BEFORE the 2-opt repair existed (via
// the plain rescue tier), so it must keep building through the plain phase —
// the repair phase is ordered strictly after every plain attempt precisely so
// maps like this keep their pre-repair path.
TEST(MapSelfRepair, DumbbellWaistGroundTruthStillBuilds)
{
  const auto map = loadConeMapCsv("sim/eufs_sim/eufs_tracks/csv/its_a_mess.csv");
  std::vector<PlannerWaypoint> waypoints;
  std::string reason;

  ASSERT_TRUE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason))
    << reason;
  ASSERT_GE(waypoints.size(), 180U);
}

// The repairs must not soften the fail-closed contract for maps that are
// broken at their core: a boundary fragmented far beyond any stray minority
// still refuses to publish.
TEST(MapSelfRepair, FragmentedBoundaryStillFailsClosed)
{
  hyu_msgs::msg::ConeArrayWithCovariance map;
  // Two blue islands 40 m apart — no rescue budget may bridge or shed half a
  // boundary.
  for (double x : {0.0, 4.0, 8.0}) {
    map.blue_cones.push_back(coneAt(x, 0.0));
    map.blue_cones.push_back(coneAt(x + 40.0, 0.0));
    map.yellow_cones.push_back(coneAt(x, 3.0));
    map.yellow_cones.push_back(coneAt(x + 40.0, 3.0));
  }
  std::vector<PlannerWaypoint> waypoints{{1.0, 2.0, 3.0, 4.0, 5.0}};
  std::string reason;

  EXPECT_FALSE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason));
  EXPECT_TRUE(waypoints.empty());
}

}  // namespace hyu_global_planner::test
