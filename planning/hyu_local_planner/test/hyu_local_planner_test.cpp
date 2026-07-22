#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "hyu_local_planner/local_path_builder.hpp"

namespace hyu_local_planner
{
namespace
{

ConeSet straightTwoSided()
{
  ConeSet cones;
  for (double x : {0.0, 2.0, 4.0, 6.0, 8.0}) {
    cones.blue.push_back({x, 2.0});
    cones.yellow.push_back({x, -2.0});
  }
  return cones;
}

ConeSet simulatorLikeTwoBlueThreeYellow()
{
  ConeSet cones;
  cones.blue = {{0.0, 2.0}, {4.0, 2.0}};
  cones.yellow = {{0.0, -2.0}, {2.0, -2.0}, {4.0, -2.0}};
  cones.orange = {{1.0, 0.0}, {3.0, 0.0}};
  cones.big_orange = {{2.0, 0.0}};
  cones.unknown = {{1.5, 0.2}, {2.5, -0.2}};
  return cones;
}

bool expectStrictFinitePath(const BuildResult & result, double speed)
{
  if (!result.evaluated || !result.valid || result.waypoints.size() < 5U) {
    ADD_FAILURE() << "planner did not evaluate a valid five-point path: " << result.reason;
    return false;
  }
  for (std::size_t index = 0; index < result.waypoints.size(); ++index) {
    const auto & waypoint = result.waypoints[index];
    EXPECT_TRUE(std::isfinite(waypoint.x));
    EXPECT_TRUE(std::isfinite(waypoint.y));
    EXPECT_TRUE(std::isfinite(waypoint.s));
    EXPECT_TRUE(std::isfinite(waypoint.psi));
    EXPECT_TRUE(std::isfinite(waypoint.kappa));
    // The per-path speed is the straight-line cap; the curvature profile may
    // lower it in corners, so it is an upper bound, not an exact value.
    EXPECT_LE(waypoint.speed, speed + 1.0e-9);
    EXPECT_GT(waypoint.speed, 0.0);
    if (index > 0U) {
      EXPECT_GT(waypoint.s, result.waypoints[index - 1U].s);
    }
  }
  return true;
}

void expectInvalidEmptyPath(const BuildResult & result, const char * invalid_message = "")
{
  ASSERT_TRUE(result.evaluated);
  EXPECT_FALSE(result.valid) << invalid_message;
  EXPECT_TRUE(result.waypoints.empty());
}

TEST(LocalPathBuilder, TwoSidedUsesDeterministicBoundaryTraversalAndDedupe)
{
  auto cones = straightTwoSided();
  cones.blue.push_back({4.01, 2.0});
  const auto first = buildLocalPath(cones);
  const auto second = buildLocalPath(cones);
  if (!expectStrictFinitePath(first, 3.0)) {
    return;
  }
  ASSERT_TRUE(second.valid) << second.reason;
  ASSERT_EQ(first.waypoints.size(), second.waypoints.size());
  for (std::size_t index = 0; index < first.waypoints.size(); ++index) {
    EXPECT_DOUBLE_EQ(first.waypoints[index].x, second.waypoints[index].x);
    EXPECT_DOUBLE_EQ(first.waypoints[index].y, second.waypoints[index].y);
  }
  EXPECT_EQ(first.kind, PathKind::kTwoSided);
}

TEST(LocalPathBuilder, SimulatorLikeTwoBlueThreeYellowIgnoresNonBoundaryNoise)
{
  const auto result = buildLocalPath(simulatorLikeTwoBlueThreeYellow());

  if (!expectStrictFinitePath(result, 3.0)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
}

TEST(LocalPathBuilder, TwoSidedPathPairsOrderedMapBoundariesThroughCurve)
{
  ConeSet cones;
  cones.blue = {{0.0, 2.0}, {2.0, 2.2}, {4.0, 2.8}, {6.0, 3.8}, {8.0, 5.0}};
  cones.yellow = {{0.0, -2.0}, {2.0, -2.2}, {4.0, -1.8}, {6.0, -0.2}, {8.0, 1.0}};

  PlannerConfig config;
  config.allow_partial_boundary = true;
  const auto result = buildLocalPath(cones, config);

  if (!expectStrictFinitePath(result, 3.0)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  EXPECT_NEAR(result.waypoints.front().x, 0.0, 0.2);
  EXPECT_NEAR(result.waypoints.front().y, 0.0, 0.2);
  for (std::size_t index = 1; index < result.waypoints.size(); ++index) {
    EXPECT_GE(result.waypoints[index].x, result.waypoints[index - 1U].x);
  }
}

TEST(LocalPathBuilder, SparseMapUsesNearestUsableBoundaryAtReducedSpeed)
{
  ConeSet cones;
  cones.blue = {{6.0, 2.8}, {9.0, 3.0}};
  cones.yellow = {{0.7, -2.6}, {6.0, -1.8}, {9.0, -1.7}};

  PlannerConfig config;
  config.allow_partial_boundary = true;
  const auto result = buildLocalPath(cones, config);

  if (!expectStrictFinitePath(result, 1.5)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kYellowOnly);
  EXPECT_LT(result.waypoints.front().x, 2.0);
}

// Regression: real peanut-track SLAM map cropped to the local ROI at the
// start/finish, where the ROI box also contains the folded-back return
// passage (a second blue wall at y~5-8 and yellow at y~7). The old
// arc-length-ratio pairing bridged the two corridors and produced no valid
// path (local_topology_gap), stalling the car. The width-gated cross pairing
// must follow only the corridor DIRECTLY AHEAD (near walls, y~0 / y~-3..-5),
// never drifting up to the folded-back wall.
TEST(LocalPathBuilder, PeanutFoldedPassageFollowsNearCorridorNotFarWall)
{
  ConeSet cones;
  cones.blue = {
    {-0.1, 1.7}, {0.3, 6.9}, {1.0, 5.5}, {1.6, 1.6}, {2.3, 4.2}, {3.6, 3.2},
    {3.6, 1.2}, {4.6, 3.0}, {5.6, 2.9}, {6.3, 0.7}, {7.2, 3.2}, {8.1, -0.2},
    {8.1, 0.5}, {8.6, 3.5}, {9.5, -1.0}, {10.2, -1.1}, {10.3, 4.7}, {11.7, -3.3},
    {11.9, 6.1}, {13.1, -3.3}, {13.4, 7.4}, {14.6, -2.9}, {14.8, 8.0}};
  cones.yellow = {
    {-0.6, -2.8}, {1.3, -3.0}, {3.0, -3.1}, {3.9, -2.8}, {4.9, 7.5}, {6.0, -3.8},
    {6.1, 7.4}, {7.2, 7.7}, {7.2, -4.4}, {8.3, -5.3}, {9.5, -6.3}, {11.2, -7.2},
    {12.4, -7.4}, {14.2, -7.6}};

  PlannerConfig config;                 // slam_map mode tolerates the outliers
  config.allow_partial_boundary = true;
  const auto result = buildLocalPath(cones, config);

  ASSERT_TRUE(result.valid) << "peanut fold stalled the planner: " << result.reason;
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  // The centerline must hug the near corridor, never climbing onto the
  // folded-back wall (which sits at y >= ~4).
  for (const auto & waypoint : result.waypoints) {
    EXPECT_LT(waypoint.y, 3.0) << "path drifted toward the folded-back passage";
  }
  // And it must make real forward progress, not stall at the ego.
  EXPECT_GT(result.waypoints.back().s, 5.0);
}

// Regression for the dead u-turn branch: a TIGHT two-sided hairpin. The
// centerline midpoints must chain around the ~180 deg apex, which requires
// the u-turn continuation (return-lane links project backward / sit in the
// 60-90 deg band). Before the fix the midpoint chain stranded at the apex.
TEST(LocalPathBuilder, TwoSidedHairpinRoundsTheApex)
{
  // Bottom leg (centerline y=0, +x) and top leg (centerline y=6, -x), 3 m
  // wide, joined by an apex near x=9.
  ConeSet cones;
  cones.blue = {  // inner wall (left of travel on both legs)
    {0.0, 1.5}, {2.0, 1.5}, {4.0, 1.5}, {6.0, 1.5}, {8.0, 1.5},
    {8.5, 3.0}, {8.0, 4.5}, {6.0, 4.5}, {4.0, 4.5}, {2.0, 4.5}, {0.0, 4.5}};
  cones.yellow = {  // outer wall
    {0.0, -1.5}, {2.0, -1.5}, {4.0, -1.5}, {6.0, -1.5}, {8.0, -1.5},
    {10.0, 0.0}, {11.0, 3.0}, {10.0, 6.0},
    {8.0, 7.5}, {6.0, 7.5}, {4.0, 7.5}, {2.0, 7.5}, {0.0, 7.5}};

  PlannerConfig config;
  config.allow_partial_boundary = true;
  config.two_sided_horizon_m = 30.0;   // long enough to include the return leg
  const auto result = buildLocalPath(cones, config);

  ASSERT_TRUE(result.valid) << "hairpin stranded the planner: " << result.reason;
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  // The path must round the apex into the return leg (centerline y ~ 6),
  // not stop on the outbound leg (centerline y ~ 0).
  double max_y = 0.0;
  for (const auto & waypoint : result.waypoints) {
    max_y = std::max(max_y, waypoint.y);
  }
  EXPECT_GT(max_y, 4.0) << "midpoint chain never rounded the hairpin apex";
}

TEST(LocalPathBuilder, MapModeCanUseConnectedPrefixBeforeUnmappedGap)
{
  ConeSet cones;
  cones.yellow = {{0.0, -1.5}, {2.0, -1.5}, {4.0, -1.5}, {12.0, -1.5}};
  PlannerConfig config;
  config.allow_partial_boundary = true;

  const auto result = buildLocalPath(cones, config);

  if (!expectStrictFinitePath(result, 1.5)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kYellowOnly);
  EXPECT_LE(result.waypoints.back().x, 8.0);
}

TEST(LocalPathBuilder, StrictModeRejectsTruncatedTwoSidedBoundary)
{
  ConeSet cones;
  cones.blue = {{0.0, 2.0}, {2.0, 2.0}, {4.0, 2.0}, {6.0, 2.0}};
  cones.yellow = {{0.0, -2.0}, {2.0, -2.0}, {4.0, -2.0}, {12.0, -2.0}};

  expectInvalidEmptyPath(buildLocalPath(cones));
}

TEST(LocalPathBuilder, MapModeSeedsOneSidedTraversalAtNearestBehindCone)
{
  ConeSet cones;
  cones.yellow = {{-0.8, -1.5}, {2.0, -1.5}, {4.0, -1.5}, {6.0, -1.5}};
  PlannerConfig config;
  config.allow_partial_boundary = true;

  const auto result = buildLocalPath(cones, config);

  if (!expectStrictFinitePath(result, 1.5)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kYellowOnly);
  EXPECT_LT(result.waypoints.front().x, 0.0);
  EXPECT_LT(std::hypot(result.waypoints.front().x, result.waypoints.front().y), 4.0);
}

TEST(LocalPathBuilder, BlueOnlyOffsetsRightAtFallbackSpeed)
{
  ConeSet cones;
  cones.blue = {{0.0, 1.5}, {2.0, 1.5}, {4.0, 1.5}, {6.0, 1.5}};
  const auto result = buildLocalPath(cones);
  if (!expectStrictFinitePath(result, 1.5)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kBlueOnly);
  EXPECT_NEAR(result.waypoints.front().y, 0.0, 1.0e-9);
  EXPECT_LE(result.waypoints.back().s, 8.0);
}

TEST(LocalPathBuilder, YellowOnlyOffsetsLeftAtFallbackSpeed)
{
  ConeSet cones;
  cones.yellow = {{0.0, -1.5}, {2.0, -1.5}, {4.0, -1.5}, {6.0, -1.5}};
  const auto result = buildLocalPath(cones);
  if (!expectStrictFinitePath(result, 1.5)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kYellowOnly);
  EXPECT_NEAR(result.waypoints.front().y, 0.0, 1.0e-9);
  EXPECT_LE(result.waypoints.back().s, 8.0);
}

TEST(LocalPathBuilder, OneSidedGapCannotReturnValidTruncatedPrefix)
{
  ConeSet cones;
  cones.blue = {{0.0, 1.5}, {2.0, 1.5}, {4.0, 1.5}, {12.0, 1.5}};

  const auto result = buildLocalPath(cones);

  expectInvalidEmptyPath(result, "one-sided traversal must not publish the 0,2,4 prefix");
}

TEST(LocalPathBuilder, OneSidedHeadingCannotReturnValidTruncatedPrefix)
{
  ConeSet cones;
  cones.blue = {{0.0, 1.5}, {2.0, 1.5}, {4.0, 1.5}, {5.0, 4.5}};

  const auto result = buildLocalPath(cones);

  expectInvalidEmptyPath(result, "one-sided traversal must not publish before a heading stop");
}

TEST(LocalPathBuilder, NonBoundaryColorsAreIgnoredButCannotFormPath)
{
  auto known = straightTwoSided();
  known.orange = {{1.0, 0.0}, {2.0, 0.0}};
  known.big_orange = {{3.0, 0.0}};
  known.unknown = {{4.0, 0.0}};
  const auto with_noise = buildLocalPath(known);
  if (!expectStrictFinitePath(with_noise, 3.0)) {
    return;
  }

  ConeSet noise_only;
  noise_only.orange = known.orange;
  noise_only.big_orange = known.big_orange;
  noise_only.unknown = known.unknown;
  const auto invalid = buildLocalPath(noise_only);
  ASSERT_TRUE(invalid.evaluated);
  EXPECT_FALSE(invalid.valid);
}

TEST(LocalPathBuilder, OutOfRoiAndNonFiniteConesAreIgnored)
{
  auto usable = straightTwoSided();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  usable.blue.insert(usable.blue.end(), {{nan, 2.0}, {infinity, 2.0}, {21.0, 2.0}, {4.0, 8.1}});
  usable.yellow.insert(
    usable.yellow.end(), {{0.0, nan}, {0.0, infinity}, {-1.1, -2.0}, {4.0, -8.1}});

  const auto result = buildLocalPath(usable);
  if (!expectStrictFinitePath(result, 3.0)) {
    return;
  }
  EXPECT_EQ(result.kind, PathKind::kTwoSided);

  ConeSet unusable;
  unusable.blue = {{nan, 1.5}, {21.0, 1.5}, {0.0, 8.1}};
  unusable.yellow = {{0.0, nan}, {infinity, -1.5}};
  const auto invalid = buildLocalPath(unusable);
  expectInvalidEmptyPath(invalid);
}

TEST(LocalPathBuilder, MixedSparseAndUnknownOnlyFailClosed)
{
  ConeSet mixed;
  mixed.blue = {{0.0, 1.5}, {2.0, 1.5}, {4.0, 1.5}};
  mixed.yellow = {{0.0, -1.5}};
  mixed.unknown = {{1.0, 0.0}, {3.0, 0.0}, {5.0, 0.0}};
  const auto mixed_result = buildLocalPath(mixed);
  ASSERT_TRUE(mixed_result.evaluated);
  EXPECT_FALSE(mixed_result.valid);

  ConeSet unknown_only;
  unknown_only.unknown = mixed.unknown;
  const auto unknown_result = buildLocalPath(unknown_only);
  ASSERT_TRUE(unknown_result.evaluated);
  EXPECT_FALSE(unknown_result.valid);
}

TEST(LocalPathBuilder, UnknownConeOnBlueLineEnablesOneSidedPath)
{
  // Two known blue cones sit below the >=3 one-sided gate; a colour drop-out on
  // the same boundary line is absorbed and lifts the side over the gate.
  ConeSet cones;
  cones.blue = {{0.0, 2.0}, {2.0, 2.0}};
  cones.unknown = {{4.0, 2.0}};

  const auto with_unknown = buildLocalPath(cones);
  ASSERT_TRUE(with_unknown.evaluated);
  ASSERT_TRUE(with_unknown.valid) << with_unknown.reason;
  EXPECT_EQ(with_unknown.kind, PathKind::kBlueOnly);

  PlannerConfig without_unknown;
  without_unknown.use_unknown_cones = false;
  const auto disabled = buildLocalPath(cones, without_unknown);
  ASSERT_TRUE(disabled.evaluated);
  EXPECT_FALSE(disabled.valid);
}

TEST(LocalPathBuilder, AllUnknownConesSplitByEgoSideFormTwoSidedPath)
{
  // A fully unclassified map (no labelled cones in reach) falls back to the
  // ego-frame left/right split and still yields a centred two-sided path.
  ConeSet cones;
  for (double x : {0.0, 2.0, 4.0, 6.0, 8.0}) {
    cones.unknown.push_back({x, 1.5});
    cones.unknown.push_back({x, -1.5});
  }

  const auto with_unknown = buildLocalPath(cones);
  if (!expectStrictFinitePath(with_unknown, 3.0)) {
    return;
  }
  EXPECT_EQ(with_unknown.kind, PathKind::kTwoSided);
  for (const auto & waypoint : with_unknown.waypoints) {
    EXPECT_NEAR(waypoint.y, 0.0, 0.25);
  }

  PlannerConfig without_unknown;
  without_unknown.use_unknown_cones = false;
  const auto disabled = buildLocalPath(cones, without_unknown);
  ASSERT_TRUE(disabled.evaluated);
  EXPECT_FALSE(disabled.valid);
}

TEST(LocalPathBuilder, CentrelineUnknownConeIsDropped)
{
  // A cone dead-centre between the boundaries is off both boundary lines, so it
  // must be discarded and leave the two-sided path bit-for-bit unchanged.
  const auto baseline = buildLocalPath(straightTwoSided());
  if (!expectStrictFinitePath(baseline, 3.0)) {
    return;
  }

  auto with_centre = straightTwoSided();
  with_centre.unknown = {{4.0, 0.0}};
  const auto result = buildLocalPath(with_centre);
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(baseline.waypoints.size(), result.waypoints.size());
  for (std::size_t index = 0; index < baseline.waypoints.size(); ++index) {
    EXPECT_DOUBLE_EQ(baseline.waypoints[index].x, result.waypoints[index].x);
    EXPECT_DOUBLE_EQ(baseline.waypoints[index].y, result.waypoints[index].y);
  }
}

TEST(LocalPathBuilder, StopAtPathEndTapersSpeedToZeroAtTheOpenEnd)
{
  // Open-ended track (dlc mission): the path speed must walk down to zero at
  // the corridor end following v = sqrt(2*a*max(0, remaining - margin)), below
  // the min_speed floor, while the cruise cap still rules far from the end.
  ConeSet cones;
  for (double x = 0.0; x <= 20.0; x += 2.0) {
    cones.blue.push_back({x, 1.5});
    cones.yellow.push_back({x, -1.5});
  }
  PlannerConfig config;
  config.two_sided_speed_mps = 6.0;
  config.stop_at_path_end = true;
  config.end_stop_decel_mps2 = 2.0;
  config.end_stop_margin_m = 1.0;

  const auto result = buildLocalPath(cones, config);
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(result.kind, PathKind::kTwoSided);
  const double end_s = result.waypoints.back().s;
  ASSERT_GT(end_s, 15.0);
  EXPECT_DOUBLE_EQ(result.waypoints.front().speed, 6.0);
  EXPECT_DOUBLE_EQ(result.waypoints.back().speed, 0.0);
  for (const auto & waypoint : result.waypoints) {
    const double remaining =
      std::max(0.0, end_s - waypoint.s - config.end_stop_margin_m);
    const double taper =
      std::sqrt(2.0 * config.end_stop_decel_mps2 * remaining);
    EXPECT_LE(waypoint.speed, std::min(6.0, taper) + 1.0e-9);
    EXPECT_GE(waypoint.speed, 0.0);
  }

  // Off by default: the same corridor carries cruise speed to the last
  // waypoint (the fail-safe brake owns the stop, as on closed-loop missions).
  PlannerConfig untapered_config = config;
  untapered_config.stop_at_path_end = false;
  const auto untapered = buildLocalPath(cones, untapered_config);
  ASSERT_TRUE(untapered.valid) << untapered.reason;
  EXPECT_DOUBLE_EQ(untapered.waypoints.back().speed, 6.0);
}

PlannerConfig straightCorridorConfig()
{
  PlannerConfig config;
  config.extend_straight_to_horizon = true;
  config.roi_min_x = -15.0;              // keep passed cones for the line fit
  config.straight_extension_cap_m = 5.0;
  return config;
}

TEST(LocalPathBuilder, StraightCorridorSurvivesFrontierOutrun)
{
  // The crux of the mid-run brake pulses: with slow perception the car outruns
  // the mapped frontier, so NO cones sit ahead of the ego -- only the ones it
  // has already passed. The line fit through those still yields a valid forward
  // path (carried a bounded distance ahead), so the path stays valid and the
  // car does not brake.
  ConeSet cones;
  for (double x : {-10.0, -5.0, 0.0}) {   // frontier is at the ego; nothing ahead
    cones.blue.push_back({x, 1.5});
    cones.yellow.push_back({x, -1.5});
  }
  const auto result = buildLocalPath(cones, straightCorridorConfig());
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  // Carried straight ahead, centred, past the last cone by roughly the cap.
  EXPECT_GT(result.waypoints.back().x, 3.0);
  for (const auto & waypoint : result.waypoints) {
    EXPECT_NEAR(waypoint.y, 0.0, 1.0e-6);
    EXPECT_GE(waypoint.x, -1.0e-9);        // never behind the ego
  }
}

TEST(LocalPathBuilder, StraightCorridorStartsFromSinglePair)
{
  // At the very start of the run only the first cone pair is in view -- too few
  // for the line fit. The mission must still yield a valid forward creep path
  // (via the sparse fallback) so the car can move off the line and accumulate
  // the map; otherwise it never starts.
  ConeSet cones;
  cones.blue = {{8.0, 1.5}};
  cones.yellow = {{8.0, -1.5}};
  PlannerConfig config = straightCorridorConfig();
  config.allow_partial_boundary = true;   // slam_map default for the mission
  const auto result = buildLocalPath(cones, config);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_GT(result.waypoints.back().x, 0.0);   // heads forward, not stuck
}

TEST(LocalPathBuilder, StraightCorridorLaunchesOnStartGate)
{
  // The big-orange start gate (two cones each side, sitting on the corridor
  // line) folds into the boundary, so the car launches at full speed off a
  // single blue/yellow pair instead of only creeping via the sparse fallback.
  ConeSet cones;
  cones.blue = {{8.0, 1.5}};
  cones.yellow = {{8.0, -1.5}};
  cones.big_orange = {{3.0, 1.5}, {2.5, 1.5}, {3.0, -1.5}, {2.5, -1.5}};
  PlannerConfig config = straightCorridorConfig();
  config.allow_partial_boundary = true;
  const auto result = buildLocalPath(cones, config);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kTwoSided);   // straight line fit, not a creep
  EXPECT_GT(result.waypoints.back().x, 3.0);
}

TEST(LocalPathBuilder, StraightCorridorFollowsOrangeBrakingZone)
{
  // Car has cleared the corridor: blue/yellow are behind, only the orange
  // braking-zone cones lie ahead. The path must continue FORWARD through the
  // orange (not deviate onto the cones behind) and at the reduced braking speed.
  ConeSet cones;
  cones.blue = {{-5.0, 1.5}, {-10.0, 1.5}};
  cones.yellow = {{-5.0, -1.5}, {-10.0, -1.5}};
  cones.orange = {{3.0, 1.5}, {8.0, 1.5}, {3.0, -1.5}, {8.0, -1.5}};
  PlannerConfig config = straightCorridorConfig();
  config.allow_partial_boundary = true;
  const auto result = buildLocalPath(cones, config);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_GT(result.waypoints.back().x, 0.0);   // forward through the orange
  for (const auto & waypoint : result.waypoints) {
    EXPECT_GE(waypoint.x, -1.0e-9);            // never behind the ego
    EXPECT_LE(waypoint.speed, config.fallback_speed_mps + 1.0e-9);  // braking speed
  }
}

TEST(LocalPathBuilder, StraightCorridorStopsPastCorridorEnd)
{
  // Once the whole corridor has fallen more than the cap behind the ego there
  // is no forward path -> invalid -> the car brakes and stops. This is what
  // ends the run (the corridor cones do not extend past the finish).
  ConeSet cones;
  for (double x : {-10.0, -8.0, -6.5}) {   // all cones > cap(5) behind the ego
    cones.blue.push_back({x, 1.5});
    cones.yellow.push_back({x, -1.5});
  }
  const auto result = buildLocalPath(cones, straightCorridorConfig());
  ASSERT_TRUE(result.evaluated);
  EXPECT_FALSE(result.valid) << "corridor is behind the ego; path must be invalid";
}

// Straight cones spanning the ego (ego-frame), frontier 10 m ahead: fit gives
// line y=0 with x_end = 10 + cap(5) = 15 and corridor end at x=10.
ConeSet straightLatchCones()
{
  ConeSet cones;
  for (double x : {-10.0, -5.0, 0.0, 5.0, 10.0}) {
    cones.blue.push_back({x, 1.5});
    cones.yellow.push_back({x, -1.5});
  }
  return cones;
}

TEST(LocalPathBuilder, StraightFitExposesLatchableLine)
{
  const auto result = buildLocalPath(straightLatchCones(), straightCorridorConfig());
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_TRUE(result.straight_fit);
  EXPECT_NEAR(result.straight_line_start.x, 0.0, 1.0e-9);
  EXPECT_NEAR(result.straight_line_start.y, 0.0, 1.0e-6);
  EXPECT_NEAR(result.straight_line_end.x, 15.0, 1.0e-6);
  EXPECT_NEAR(result.straight_line_end.y, 0.0, 1.0e-6);
  EXPECT_NEAR(result.straight_corridor_max_x, 10.0, 1.0e-9);

  // Generic (non-fit) results must never latch: the sparse single-pair creep
  // path carries no line.
  ConeSet pair_only;
  pair_only.blue = {{8.0, 1.5}};
  pair_only.yellow = {{8.0, -1.5}};
  PlannerConfig config = straightCorridorConfig();
  config.allow_partial_boundary = true;
  const auto sparse = buildLocalPath(pair_only, config);
  ASSERT_TRUE(sparse.valid) << sparse.reason;
  EXPECT_FALSE(sparse.straight_fit);
  EXPECT_FALSE(latchStraightLine(sparse, 0.0, 0.0, 0.0).has_value());
}

TEST(LocalPathBuilder, HeldStraightLineSurvivesTotalObservationDropout)
{
  // The unstable case: after a good fit the observations vanish entirely. The
  // latched line must keep producing the same straight path from the advanced
  // ego pose, with no cones at all.
  const PlannerConfig config = straightCorridorConfig();
  const auto fit = buildLocalPath(straightLatchCones(), config);
  ASSERT_TRUE(fit.valid) << fit.reason;
  const auto latch = latchStraightLine(fit, 0.0, 0.0, 0.0);
  ASSERT_TRUE(latch.has_value());

  // 5 m further down the run, slightly off-centre and askew.
  const auto held = buildHeldStraightPath(*latch, 5.0, 0.2, 0.05, config);
  ASSERT_TRUE(held.valid) << held.reason;
  EXPECT_EQ(held.kind, PathKind::kTwoSided);
  // The path tracks the latched odom line (y=0), not the ego centreline: in
  // the ego frame the line sits ~0.2 m to the right.
  for (const auto & waypoint : held.waypoints) {
    const double odom_y = 0.2 + waypoint.x * std::sin(0.05) + waypoint.y * std::cos(0.05);
    EXPECT_NEAR(odom_y, 0.0, 1.0e-6);
    EXPECT_DOUBLE_EQ(waypoint.speed, config.two_sided_speed_mps);  // corridor ahead
  }
  // Still ends at the latched end (odom x=15 -> ~10 m ahead of the ego).
  EXPECT_NEAR(held.waypoints.back().s, 10.0, 0.1);
}

TEST(LocalPathBuilder, HeldStraightLineTransformsAcrossYaw)
{
  // Latch computed while the run points along odom +y (ego yaw 90 deg): the
  // hold from a pose further along that line must still head straight ahead.
  const PlannerConfig config = straightCorridorConfig();
  const auto fit = buildLocalPath(straightLatchCones(), config);
  ASSERT_TRUE(fit.valid) << fit.reason;
  const double yaw = 1.57079632679489662;
  const auto latch = latchStraightLine(fit, 3.0, -2.0, yaw);
  ASSERT_TRUE(latch.has_value());
  EXPECT_NEAR(latch->dir.x, 0.0, 1.0e-9);
  EXPECT_NEAR(latch->dir.y, 1.0, 1.0e-9);

  const auto held = buildHeldStraightPath(*latch, 3.0, 3.0, yaw, config);
  ASSERT_TRUE(held.valid) << held.reason;
  for (const auto & waypoint : held.waypoints) {
    EXPECT_NEAR(waypoint.y, 0.0, 1.0e-6);   // dead ahead in the ego frame
    EXPECT_GE(waypoint.x, -1.0e-9);
  }
  EXPECT_NEAR(held.waypoints.back().x, 10.0, 0.1);  // latched end, 5 m consumed
}

TEST(LocalPathBuilder, HeldStraightLineBrakesPastCorridorAndEndsAtLatchedEnd)
{
  const PlannerConfig config = straightCorridorConfig();
  const auto fit = buildLocalPath(straightLatchCones(), config);
  ASSERT_TRUE(fit.valid) << fit.reason;
  const auto latch = latchStraightLine(fit, 0.0, 0.0, 0.0);
  ASSERT_TRUE(latch.has_value());

  // Past the latched corridor end (x=10) but before the latched line end
  // (x=15): still a valid forward path, at braking speed.
  const auto braking = buildHeldStraightPath(*latch, 12.0, 0.0, 0.0, config);
  ASSERT_TRUE(braking.valid) << braking.reason;
  for (const auto & waypoint : braking.waypoints) {
    EXPECT_DOUBLE_EQ(waypoint.speed, config.fallback_speed_mps);
  }

  // At/past the latched end there is no forward path left -> invalid -> the
  // car brakes to a stop exactly as it would have with a live fit.
  const auto past_end = buildHeldStraightPath(*latch, 16.0, 0.0, 0.0, config);
  ASSERT_TRUE(past_end.evaluated);
  EXPECT_FALSE(past_end.valid);
}

TEST(LocalPathBuilder, HeldStraightLineFailsClosedOnPoseJumpOrFlip)
{
  const PlannerConfig config = straightCorridorConfig();
  const auto fit = buildLocalPath(straightLatchCones(), config);
  ASSERT_TRUE(fit.valid) << fit.reason;
  const auto latch = latchStraightLine(fit, 0.0, 0.0, 0.0);
  ASSERT_TRUE(latch.has_value());

  // Lateral pose jump beyond max_start_distance_m: the hold must brake, not
  // steer the car back across the corridor cones.
  const auto jumped = buildHeldStraightPath(*latch, 5.0, 6.0, 0.0, config);
  ASSERT_TRUE(jumped.evaluated);
  EXPECT_FALSE(jumped.valid);

  // Heading flipped 180 deg: the held path would run backward in the ego
  // frame; the straight-mode no-backward gate must reject it.
  const auto flipped = buildHeldStraightPath(
    *latch, 5.0, 0.0, 3.14159265358979323846, config);
  ASSERT_TRUE(flipped.evaluated);
  EXPECT_FALSE(flipped.valid);
}

TEST(LocalPathBuilder, PartialBoundarySinglePairBuildsSparsePath)
{
  PlannerConfig config;
  config.allow_partial_boundary = true;
  ConeSet sparse;
  sparse.blue = {{4.0, 1.8}};
  sparse.yellow = {{4.0, -1.8}};

  const auto result = buildLocalPath(sparse, config);
  ASSERT_TRUE(result.evaluated);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  ASSERT_GE(result.waypoints.size(), 5U);
  EXPECT_LE(std::hypot(result.waypoints.front().x, result.waypoints.front().y), 0.5);
  for (const auto & waypoint : result.waypoints) {
    EXPECT_DOUBLE_EQ(waypoint.speed, config.fallback_speed_mps);
  }
}

TEST(LocalPathBuilder, PartialBoundaryTwoConeSideOffsetsPath)
{
  PlannerConfig config;
  config.allow_partial_boundary = true;
  ConeSet sparse;
  sparse.blue = {{2.0, 1.6}, {6.0, 1.7}};

  const auto result = buildLocalPath(sparse, config);
  ASSERT_TRUE(result.evaluated);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kBlueOnly);
  ASSERT_GE(result.waypoints.size(), 5U);
  // Blue is the left boundary: the offset centerline must sit to its right.
  for (const auto & waypoint : result.waypoints) {
    EXPECT_LT(waypoint.y, 1.6);
    EXPECT_DOUBLE_EQ(waypoint.speed, config.fallback_speed_mps);
  }
}

TEST(LocalPathBuilder, StrictModeStillFailsClosedOnSparseInputs)
{
  ConeSet pair_only;
  pair_only.blue = {{4.0, 1.8}};
  pair_only.yellow = {{4.0, -1.8}};
  expectInvalidEmptyPath(buildLocalPath(pair_only));

  ConeSet two_blue;
  two_blue.blue = {{2.0, 1.6}, {6.0, 1.7}};
  expectInvalidEmptyPath(buildLocalPath(two_blue));
}

TEST(LocalPathBuilder, EmptyAndNoiseOnlyInputsFailClosed)
{
  const auto empty = buildLocalPath(ConeSet{});
  expectInvalidEmptyPath(empty);

  ConeSet noise_only;
  noise_only.orange = {{1.0, 0.0}, {2.0, 0.0}};
  noise_only.big_orange = {{3.0, 0.0}};
  noise_only.unknown = {{4.0, 0.0}};
  const auto noise = buildLocalPath(noise_only);
  expectInvalidEmptyPath(noise);
}

}
}
