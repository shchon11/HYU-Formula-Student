#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "global_planner/slam_centerline_builder.hpp"
#include "slam_test_fixtures.hpp"

// Raceline smoothing contract: strictly lower peak curvature than the pure
// centerline, hard clearance to both boundaries, closed loops stay closed,
// straights stay put, and iterations 0 reproduces the previous behavior
// exactly.

namespace global_planner::test
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// Ring track: two concentric cone circles, centerline radius 20 m, width 4 m.
eufs_msgs::msg::ConeArrayWithCovariance ringMap(std::size_t cones_per_side = 40U)
{
  eufs_msgs::msg::ConeArrayWithCovariance map;
  for (std::size_t i = 0; i < cones_per_side; ++i) {
    const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(cones_per_side);
    eufs_msgs::msg::ConeWithCovariance blue;
    blue.point.x = 22.0 * std::cos(angle);
    blue.point.y = 22.0 * std::sin(angle);
    map.blue_cones.push_back(blue);
    eufs_msgs::msg::ConeWithCovariance yellow;
    yellow.point.x = 18.0 * std::cos(angle);
    yellow.point.y = 18.0 * std::sin(angle);
    map.yellow_cones.push_back(yellow);
  }
  return map;
}

nav_msgs::msg::Odometry egoOnRing()
{
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 20.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  return odom;
}

double maxAbsCurvature(const std::vector<PlannerWaypoint> & waypoints)
{
  double worst = 0.0;
  for (const auto & waypoint : waypoints) {
    worst = std::max(worst, std::abs(waypoint.kappa));
  }
  return worst;
}

double minClearance(
  const std::vector<PlannerWaypoint> & waypoints, const std::vector<PlannerPoint> & boundary)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : waypoints) {
    best = std::min(
      best,
      distance(
        {waypoint.x, waypoint.y},
        closestPointOnFixturePolyline({waypoint.x, waypoint.y}, boundary)));
  }
  return best;
}

SlamCenterlineConfig racelineConfig(int iterations)
{
  SlamCenterlineConfig config = fixtureConfig();
  config.raceline_smoothing_iterations = iterations;
  config.raceline_margin_m = 1.2;
  config.raceline_alpha = 0.3;
  return config;
}

TEST(RacelineSmoothing, ZeroIterationsReproducesCenterlineExactly)
{
  const auto map = ringMap();
  std::vector<PlannerWaypoint> centerline;
  std::vector<PlannerWaypoint> untouched;
  std::string reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoOnRing(), fixtureConfig(), centerline, reason)) << reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoOnRing(), racelineConfig(0), untouched, reason)) << reason;
  ASSERT_EQ(centerline.size(), untouched.size());
  for (std::size_t i = 0; i < centerline.size(); ++i) {
    EXPECT_EQ(centerline[i].x, untouched[i].x);
    EXPECT_EQ(centerline[i].y, untouched[i].y);
  }
}

TEST(RacelineSmoothing, LowersPeakCurvatureOnClosedRing)
{
  const auto map = ringMap();
  std::vector<PlannerWaypoint> centerline;
  std::vector<PlannerWaypoint> raceline;
  std::string reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoOnRing(), fixtureConfig(), centerline, reason)) << reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoOnRing(), racelineConfig(400), raceline, reason)) << reason;

  // On a ring the raceline hugs the inside within the margin: measurably
  // lower curvature, not a reshuffle.
  EXPECT_LT(maxAbsCurvature(raceline), 0.98 * maxAbsCurvature(centerline));
}

TEST(RacelineSmoothing, KeepsMarginToBothBoundaries)
{
  const auto map = ringMap();
  std::vector<PlannerWaypoint> raceline;
  std::string reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoOnRing(), racelineConfig(400), raceline, reason)) << reason;

  std::vector<PlannerPoint> ordered_blue;
  std::vector<PlannerPoint> ordered_yellow;
  ASSERT_TRUE(
    orderSlamBoundaries(
      bluePoints(map), yellowPoints(map), {20.0, 0.0}, fixtureConfig().max_boundary_gap_m,
      ordered_blue, ordered_yellow, reason)) << reason;

  // Resampling can interpolate across the clamp by a hair; anything beyond a
  // few centimetres means the budget failed.
  constexpr double kTolerance = 0.05;
  EXPECT_GE(minClearance(raceline, ordered_blue), 1.2 - kTolerance);
  EXPECT_GE(minClearance(raceline, ordered_yellow), 1.2 - kTolerance);
}

TEST(RacelineSmoothing, ClosedLoopStaysClosedAndValid)
{
  const auto map = ringMap();
  std::vector<PlannerWaypoint> raceline;
  std::string reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoOnRing(), racelineConfig(400), raceline, reason)) << reason;
  ASSERT_GE(raceline.size(), 3U);
  EXPECT_LE(
    distance(
      {raceline.front().x, raceline.front().y},
      {raceline.back().x, raceline.back().y}),
    fixtureConfig().close_loop_distance_m);
  EXPECT_FALSE(hasSelfIntersection(raceline));
}

TEST(RacelineSmoothing, StraightSegmentsDoNotMove)
{
  // Open straight corridor: Laplacian of collinear points is zero, endpoints
  // are pinned, so smoothing must be a no-op.
  std::vector<PlannerPoint> centerline;
  for (int i = 0; i <= 40; ++i) {
    centerline.push_back({0.5 * static_cast<double>(i), 0.0});
  }
  const std::vector<PlannerPoint> reference = centerline;
  std::vector<PlannerPoint> left;
  std::vector<PlannerPoint> right;
  for (int i = 0; i <= 20; ++i) {
    left.push_back({static_cast<double>(i), 1.8});
    right.push_back({static_cast<double>(i), -1.8});
  }
  applyRacelineSmoothing(centerline, left, right, racelineConfig(400));
  ASSERT_EQ(centerline.size(), reference.size());
  for (std::size_t i = 0; i < centerline.size(); ++i) {
    EXPECT_NEAR(centerline[i].x, reference[i].x, 1.0e-9);
    EXPECT_NEAR(centerline[i].y, reference[i].y, 1.0e-9);
  }
}

TEST(RacelineSmoothing, IsDeterministic)
{
  const auto map = ringMap();
  std::vector<PlannerWaypoint> first;
  std::vector<PlannerWaypoint> second;
  std::string reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoOnRing(), racelineConfig(400), first, reason)) << reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoOnRing(), racelineConfig(400), second, reason)) << reason;
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_EQ(first[i].x, second[i].x);
    EXPECT_EQ(first[i].y, second[i].y);
  }
}

// Ego-independence of the ordering stage (multi-seed retry): on a known-good
// closed map the build must succeed from EVERY ego position around the lap,
// not just the ones whose nearest-cone seed happens to work. This is the
// regression for the observed "same frozen map, valid or branch_jump
// depending on where the car is" flapping.
TEST(OrderingEgoSweep, SeamFoldMapBuildsFromEveryEgoPosition)
{
  const auto map = loadConeMapCsv(
    "planning/global_planner/test/fixtures/seam_fold_cone_map.csv");
  const auto blues = bluePoints(map);
  std::size_t failures = 0U;
  for (const auto & seed : blues) {
    for (const double lateral : {0.0, 1.5}) {
      nav_msgs::msg::Odometry odom;
      odom.pose.pose.position.x = seed.x;
      odom.pose.pose.position.y = seed.y + lateral;
      odom.pose.pose.orientation.w = 1.0;
      // Both the pure centerline and the raceline must build from every ego
      // position: smoothing may never turn a valid map into an invalid path.
      for (const int iterations : {0, 400}) {
        std::vector<PlannerWaypoint> waypoints;
        std::string reason;
        if (!buildCenterlineFromSlamMap(map, odom, racelineConfig(iterations), waypoints, reason)) {
          ADD_FAILURE() << "build failed (raceline_iterations=" << iterations << ") at ego=("
                        << odom.pose.pose.position.x << "," << odom.pose.pose.position.y
                        << "): " << reason;
          ++failures;
        }
      }
    }
  }
  EXPECT_EQ(failures, 0U);
}

}  // namespace
}  // namespace global_planner::test
