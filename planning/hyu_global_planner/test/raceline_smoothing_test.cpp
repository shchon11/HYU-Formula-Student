#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "hyu_global_planner/slam_centerline_builder.hpp"
#include "slam_test_fixtures.hpp"

// Raceline smoothing contract: strictly lower peak curvature than the pure
// centerline, hard clearance to both boundaries, closed loops stay closed,
// straights stay put, and iterations 0 reproduces the previous behavior
// exactly.

namespace hyu_global_planner::test
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// Ring track: two concentric cone circles, centerline radius 20 m, width 4 m.
hyu_msgs::msg::ConeArrayWithCovariance ringMap(std::size_t cones_per_side = 40U)
{
  hyu_msgs::msg::ConeArrayWithCovariance map;
  for (std::size_t i = 0; i < cones_per_side; ++i) {
    const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(cones_per_side);
    hyu_msgs::msg::ConeWithCovariance blue;
    blue.point.x = 22.0 * std::cos(angle);
    blue.point.y = 22.0 * std::sin(angle);
    map.blue_cones.push_back(blue);
    hyu_msgs::msg::ConeWithCovariance yellow;
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
  applyMinimumCurvatureRaceline(centerline, left, right, racelineConfig(400));
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
    "planning/hyu_global_planner/test/fixtures/seam_fold_cone_map.csv");
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

// A 90 deg LEFT corner (radius 10 m) between two 20 m straights, in a 4 m wide
// corridor. `normal` is the left normal, so a positive lateral offset is toward
// the INSIDE of this corner and a negative one is toward the outside.
struct CornerTrack
{
  std::vector<PlannerPoint> centerline;
  std::vector<PlannerPoint> left;
  std::vector<PlannerPoint> right;
  std::vector<PlannerPoint> normal;
  std::size_t arc_begin{0};
  std::size_t arc_end{0};
};

CornerTrack cornerTrack()
{
  constexpr double kRadius = 10.0;
  constexpr double kHalfWidth = 2.0;
  CornerTrack track;
  const auto push = [&](PlannerPoint centre, PlannerPoint n) {
      track.centerline.push_back(centre);
      track.normal.push_back(n);
      track.left.push_back({centre.x + kHalfWidth * n.x, centre.y + kHalfWidth * n.y});
      track.right.push_back({centre.x - kHalfWidth * n.x, centre.y - kHalfWidth * n.y});
    };

  for (double x = -20.0; x < -1.0e-9; x += 0.5) {
    push({x, 0.0}, {0.0, 1.0});
  }
  track.arc_begin = track.centerline.size();
  for (double a = -kPi / 2.0; a <= 1.0e-9; a += 0.5 / kRadius) {
    push(
      {kRadius * std::cos(a), kRadius + kRadius * std::sin(a)},
      {-std::cos(a), -std::sin(a)});
  }
  track.arc_end = track.centerline.size() - 1U;
  for (double y = kRadius + 0.5; y <= kRadius + 20.0; y += 0.5) {
    push({kRadius, y}, {-1.0, 0.0});
  }
  return track;
}

// Menger curvature of the sampled line: 4*Area / (a*b*c).
double maxDiscreteCurvature(const std::vector<PlannerPoint> & points)
{
  double worst = 0.0;
  for (std::size_t i = 1U; i + 1U < points.size(); ++i) {
    const double a = distance(points[i - 1U], points[i]);
    const double b = distance(points[i], points[i + 1U]);
    const double c = distance(points[i - 1U], points[i + 1U]);
    const double cross = std::abs(
      (points[i].x - points[i - 1U].x) * (points[i + 1U].y - points[i - 1U].y) -
      (points[i].y - points[i - 1U].y) * (points[i + 1U].x - points[i - 1U].x));
    if (a * b * c > 0.0) {
      worst = std::max(worst, 2.0 * cross / (a * b * c));
    }
  }
  return worst;
}

// THE point of a minimum-curvature raceline, and what a Laplacian/curve
// shortening pass can never do: curvature is spent where it is cheapest, so the
// line runs WIDE into the corner, clips the apex on the inside, and tracks OUT
// again. A contraction pass instead pulls the whole corner inward, cutting
// entry and exit alike.
TEST(MinimumCurvatureRaceline, RunsWideInClipsApexAndTracksOut)
{
  const auto track = cornerTrack();
  std::vector<PlannerPoint> raceline = track.centerline;
  applyMinimumCurvatureRaceline(raceline, track.left, track.right, racelineConfig(600));
  ASSERT_EQ(raceline.size(), track.centerline.size());

  const auto lateral = [&](std::size_t i) {
      return (raceline[i].x - track.centerline[i].x) * track.normal[i].x +
             (raceline[i].y - track.centerline[i].y) * track.normal[i].y;
    };

  const std::size_t entry = track.arc_begin - 8U;             // ~4 m before turn-in
  const std::size_t apex = (track.arc_begin + track.arc_end) / 2U;
  const std::size_t exit = track.arc_end + 8U;                // ~4 m after track-out

  EXPECT_LT(lateral(entry), -0.2) << "entry should run WIDE (outside), got " << lateral(entry);
  EXPECT_GT(lateral(apex), 0.2) << "apex should clip the INSIDE, got " << lateral(apex);
  EXPECT_LT(lateral(exit), -0.2) << "exit should track OUT (outside), got " << lateral(exit);

  // And the whole point: a flatter corner than the centerline.
  EXPECT_LT(maxDiscreteCurvature(raceline), maxDiscreteCurvature(track.centerline));

  // Never outside the corridor: 2.0 m half width less the 1.2 m margin.
  for (std::size_t i = 0; i < raceline.size(); ++i) {
    EXPECT_LE(std::abs(lateral(i)), 0.8 + 1.0e-6) << "left the corridor at " << i;
  }
}

// The shipped track, raceline enabled end to end, seeded where the car really
// starts: still a valid closed non-intersecting path, and measurably flatter
// than the pure centerline it was turned on to improve.
TEST(MinimumCurvatureRaceline, ShippedTrackFlattensCorners)
{
  const std::string track = "eufs_sim/eufs_tracks/csv/small_track.csv";
  const auto map = loadConeMapCsv(track);
  std::vector<PlannerWaypoint> centerline;
  std::vector<PlannerWaypoint> raceline;
  std::string reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoAtCarStart(track), fixtureConfig(), centerline, reason))
    << reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoAtCarStart(track), racelineConfig(3000), raceline, reason))
    << reason;

  EXPECT_LT(maxAbsCurvature(raceline), 0.85 * maxAbsCurvature(centerline))
    << "raceline " << maxAbsCurvature(raceline) << " vs centerline "
    << maxAbsCurvature(centerline);
  EXPECT_FALSE(hasSelfIntersection(raceline));
  EXPECT_LE(
    distance({raceline.front().x, raceline.front().y}, {raceline.back().x, raceline.back().y}),
    fixtureConfig().close_loop_distance_m);
}

// A constant-radius ring is where a minimum-curvature line has exactly ONE right
// answer: the largest circle the corridor allows, because a bigger circle is
// flatter. Moving toward the centre of curvature makes the line SHARPER. This
// pins the radius directly, because a peak-curvature check cannot see it: the
// centerline is resampled off cone chords, so its discrete kappa is dominated by
// interpolation spikes and smoothing them away passes the check whichever way
// the line actually moved.
TEST(MinimumCurvatureRaceline, RingOptimisesOutwardToTheLargestCircle)
{
  const auto map = ringMap(120U);
  std::vector<PlannerWaypoint> raceline;
  std::string reason;
  ASSERT_TRUE(
    buildCenterlineFromSlamMap(map, egoOnRing(), racelineConfig(5000), raceline, reason)) << reason;

  double sum = 0.0;
  for (const auto & waypoint : raceline) {
    sum += std::hypot(waypoint.x, waypoint.y);
  }
  const double mean_radius = sum / static_cast<double>(raceline.size());

  // Centerline 20 m, cones at 18/22, margin 1.2 -> the corridor is [19.2, 20.8].
  EXPECT_GT(mean_radius, 20.0)
    << "raceline moved INWARD, i.e. SHARPER than the centerline: r=" << mean_radius;
  EXPECT_NEAR(mean_radius, 20.8, 0.3)
    << "expected the largest circle the corridor allows, got r=" << mean_radius;
}

}  // namespace
}  // namespace hyu_global_planner::test
