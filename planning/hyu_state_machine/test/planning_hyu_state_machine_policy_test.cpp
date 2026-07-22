#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "hyu_state_machine/planning_state_debug.hpp"
#include "hyu_state_machine/planning_hyu_state_machine_node.hpp"

namespace hyu_state_machine
{
namespace
{

hyu_msgs::msg::WaypointArrayStamped makePath(
  const std::vector<double> & s_values,
  const std::vector<std::pair<double, double>> & xy,
  const std::string & frame_id = "map")
{
  hyu_msgs::msg::WaypointArrayStamped msg;
  msg.header.frame_id = frame_id;
  for (std::size_t index = 0; index < s_values.size(); ++index) {
    hyu_msgs::msg::Waypoint waypoint;
    waypoint.s_m = s_values[index];
    waypoint.x_m = xy[index].first;
    waypoint.y_m = xy[index].second;
    waypoint.position.x = waypoint.x_m;
    waypoint.position.y = waypoint.y_m;
    msg.waypoints.push_back(waypoint);
  }
  return msg;
}

TEST(PlanningStatePolicy, GlobalEntryDoesNotRequireLap)
{
  EXPECT_TRUE(globalEntryAllowed({true, true, 0.25, 2.0, true}));
}

TEST(PlanningStatePolicy, OneLapHandoffUsesFinalStopSourceWhenConfigured)
{
  EXPECT_EQ(selectPathSource(PlanningState::LOCAL, 1, 1), PathSource::LOCAL);
  EXPECT_EQ(selectPathSource(PlanningState::GLOBAL, 0, 1), PathSource::GLOBAL_FULL);
  EXPECT_EQ(selectPathSource(PlanningState::GLOBAL, 1, 1), PathSource::GLOBAL_FINAL_STOP);
}

TEST(PlanningStatePolicy, RequiresContinuousFreshHandoffForHalfSecond)
{
  ContinuousHandoffGate gate;
  gate.observe(true, 10.0, 0.5);
  EXPECT_FALSE(gate.ready(10.49, 0.5, 0.5));
  gate.observe(true, 10.4, 0.5);
  EXPECT_TRUE(gate.ready(10.5, 0.5, 0.5));

  gate.refresh(11.0, 0.5);
  EXPECT_FALSE(gate.ready(11.0, 0.5, 0.5));
}

TEST(PlanningStatePolicy, FalseHandoffResetsDwell)
{
  ContinuousHandoffGate gate;
  gate.observe(true, 1.0, 0.5);
  gate.observe(false, 1.3, 0.5);
  gate.observe(true, 1.4, 0.5);
  EXPECT_FALSE(gate.ready(1.7, 0.5, 0.5));
  gate.observe(true, 1.8, 0.5);
  EXPECT_TRUE(gate.ready(1.9, 0.5, 0.5));
}

TEST(GlobalPathReadiness, ReadinessCanSkipGraphSlamLocalizationForCsvHandoffPolicy)
{
  GlobalPathReadiness readiness;
  const rclcpp::Time now(10, 0, RCL_ROS_TIME);
  auto path = makePath({0.0, 5.0, 10.0}, {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}});

  EXPECT_TRUE(readiness.onWaypoints(path, now));
  readiness.onValidity(true, now, 0.5);

  EXPECT_FALSE(readiness.ready(now, 0.5));
  EXPECT_TRUE(readiness.ready(now, 0.5, false));
  readiness.onGraphSlamStatus("localization");
  EXPECT_TRUE(readiness.ready(now, 0.5, true));
}

TEST(GlobalPathReadiness, ReadinessReasonNamesPolicyAndFreshnessFailures)
{
  GlobalPathReadiness readiness;
  const rclcpp::Time now(10, 0, RCL_ROS_TIME);
  const auto path = makePath(
    {0.0, 5.0, 10.0}, {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}});

  EXPECT_EQ(readiness.readinessReason(now, 0.5, true), "missing_global_path");

  EXPECT_TRUE(readiness.onWaypoints(path, now));
  readiness.onValidity(true, now, 0.5);
  EXPECT_EQ(readiness.readinessReason(now, 0.5, true), "graph_slam_not_localized");
  EXPECT_EQ(readiness.readinessReason(now, 0.5, false), "ready");

  readiness.refreshValidity(rclcpp::Time(11, 0, RCL_ROS_TIME), 0.5);
  EXPECT_EQ(
    readiness.readinessReason(rclcpp::Time(11, 0, RCL_ROS_TIME), 0.5, false),
    "stale_global_path_validity");
}

TEST(PlanningStateDebug, PublishesExplicitReadinessAndStopReasons)
{
  PlanningStateDebugSnapshot snapshot;
  snapshot.global_requires_graph_slam_localization = true;
  snapshot.global_readiness_reason = "stale_global_path_validity";
  snapshot.global_entry_reason = "stale_frenet_odom";
  snapshot.stop_request_reason = "stopline_detected";

  const auto debug = makePlanningStateDebugString(snapshot);

  EXPECT_NE(debug.find(" global_requires_graph_slam_localization=true"), std::string::npos);
  EXPECT_NE(debug.find(" global_readiness_reason=stale_global_path_validity"), std::string::npos);
  EXPECT_NE(debug.find(" global_entry_reason=stale_frenet_odom"), std::string::npos);
  EXPECT_NE(debug.find(" stop_request_reason=stopline_detected"), std::string::npos);
}

TEST(LapTrackingPolicy, ObservedMappingLocalizationCountsDiscoveryLapOnce)
{
  LapTrackingPolicy policy(1.0, 0.05);
  EXPECT_FALSE(policy.observeGraphSlamStatus("mapping"));
  EXPECT_TRUE(policy.observeGraphSlamStatus("localization"));
  EXPECT_FALSE(policy.observeGraphSlamStatus("mapping_converged"));
  EXPECT_FALSE(policy.observeGraphSlamStatus("localization"));
}

TEST(LapTrackingPolicy, StartupLocalizationDoesNotCount)
{
  LapTrackingPolicy policy(1.0, 0.05);
  EXPECT_FALSE(policy.observeGraphSlamStatus("localization"));
}

TEST(LapTrackingPolicy, DuplicateAndNonDuplicateClosureNormalizeLength)
{
  LapTrackingPolicy duplicate(1.0, 0.05);
  const auto duplicate_path = makePath(
    {0.0, 50.0, 100.0}, {{0.0, 0.0}, {50.0, 0.0}, {0.0, 0.0}});
  ASSERT_TRUE(duplicate.acceptPath(duplicate_path));
  EXPECT_DOUBLE_EQ(duplicate.pathLength(), 100.0);

  LapTrackingPolicy nonduplicate(1.0, 0.05);
  const auto nonduplicate_path = makePath(
    {0.0, 50.0, 99.5}, {{0.0, 0.0}, {50.0, 0.0}, {0.5, 0.0}});
  ASSERT_TRUE(nonduplicate.acceptPath(nonduplicate_path));
  EXPECT_DOUBLE_EQ(nonduplicate.pathLength(), 100.0);
}

TEST(LapTrackingPolicy, OpenOrMalformedPathDisablesLapCount)
{
  LapTrackingPolicy policy(1.0, 0.05);
  EXPECT_FALSE(policy.acceptPath(makePath(
      {0.0, 10.0, 20.0}, {{0.0, 0.0}, {10.0, 0.0}, {3.0, 0.0}})));
  EXPECT_FALSE(policy.acceptPath(makePath(
      {0.0, 10.0, 9.0}, {{0.0, 0.0}, {10.0, 0.0}, {0.0, 0.0}})));
  EXPECT_FALSE(policy.acceptPath(makePath(
      {0.0, 10.0, std::numeric_limits<double>::quiet_NaN()},
      {{0.0, 0.0}, {10.0, 0.0}, {0.0, 0.0}})));
  EXPECT_FALSE(policy.acceptPath(makePath(
      {0.0, 9.0, 18.0}, {{0.0, 0.0}, {9.0, 0.0}, {0.0, 0.0}})));
  EXPECT_FALSE(policy.pathValid());
}

TEST(LapTrackingPolicy, ForwardWrapCountsOnceWithCooldown)
{
  LapTrackingPolicy policy(1.0, 0.05);
  ASSERT_TRUE(policy.acceptPath(makePath(
      {0.0, 50.0, 100.0}, {{0.0, 0.0}, {50.0, 0.0}, {0.0, 0.0}})));

  EXPECT_FALSE(policy.observeFrenetSample(20.0, 1.0, 0.5, 2.0));
  EXPECT_FALSE(policy.observeFrenetSample(30.0, 1.1, 0.5, 2.0));
  EXPECT_TRUE(policy.armed());
  EXPECT_FALSE(policy.observeFrenetSample(96.0, 1.2, 0.5, 2.0));
  EXPECT_TRUE(policy.observeFrenetSample(4.0, 1.3, 0.5, 2.0));
  EXPECT_FALSE(policy.observeFrenetSample(96.0, 1.4, 0.5, 2.0));
  EXPECT_FALSE(policy.observeFrenetSample(4.0, 1.5, 0.5, 2.0));

  EXPECT_FALSE(policy.observeFrenetSample(20.0, 3.4, 0.5, 2.0));
  EXPECT_FALSE(policy.observeFrenetSample(30.0, 3.5, 0.5, 2.0));
  EXPECT_FALSE(policy.observeFrenetSample(96.0, 3.6, 0.5, 2.0));
  EXPECT_TRUE(policy.observeFrenetSample(4.0, 3.7, 0.5, 2.0));
}

namespace
{

OrangeGateLapTracker makeGateTracker()
{
  // cluster 2.0 m, gate width 2..7 m, arm 12 m, cooldown 10 s
  OrangeGateLapTracker tracker(2.0, 2.0, 7.0, 12.0, 10.0);
  // FS start line: two big-orange pairs, one per side, track width ~3 m,
  // gate segment along y at x = 0.
  EXPECT_TRUE(tracker.updateGate(
      {{{0.0, 1.5}}, {{0.6, 1.5}}, {{0.0, -1.5}}, {{0.6, -1.5}}}));
  return tracker;
}

// Walk the ego along y = 0 from x0 to x1 in plausible sub-2 m steps (larger
// jumps are rejected as pose snaps), 0.5 s apart, arriving at end_time.
// Returns whether any step counted a lap.
bool walk(OrangeGateLapTracker & tracker, double x0, double x1, double end_time)
{
  std::vector<double> xs;
  const double step = (x1 > x0) ? 1.5 : -1.5;
  for (double x = x0; (step > 0.0) ? (x < x1) : (x > x1); x += step) {
    xs.push_back(x);
  }
  xs.push_back(x1);
  bool counted = false;
  double t = end_time - 0.5 * static_cast<double>(xs.size() - 1U);
  for (const double x : xs) {
    counted = tracker.observeEgo(x, 0.0, t) || counted;
    t += 0.5;
  }
  return counted;
}

// Drive one full lap: out beyond the 12 m arm radius, then back across the
// gate in +x, final crossing just before end_time. Returns whether it counted.
// (The outbound pass back through the line is direction-mismatched and armed
// is false there, so it never counts — same as the real out-and-around lap.)
bool driveLap(OrangeGateLapTracker & tracker, double end_time)
{
  EXPECT_FALSE(walk(tracker, 1.0, -14.0, end_time - 5.0));
  return walk(tracker, -14.0, 1.0, end_time);
}

}

TEST(OrangeGateLapTracker, RequiresTwoSideClusters)
{
  OrangeGateLapTracker tracker(2.0, 2.0, 7.0, 12.0, 10.0);
  EXPECT_FALSE(tracker.updateGate({}));
  EXPECT_FALSE(tracker.updateGate({{{0.0, 1.5}}}));
  // One tight cluster only: no gate.
  EXPECT_FALSE(tracker.updateGate({{{0.0, 1.5}}, {{0.5, 1.4}}}));
  // Two clusters separated by a plausible track width: gate.
  EXPECT_TRUE(tracker.updateGate({{{0.0, 1.5}}, {{0.0, -1.5}}}));
  EXPECT_TRUE(tracker.gateValid());
  // A degraded later snapshot keeps the previous gate.
  EXPECT_TRUE(tracker.updateGate({}));
}

TEST(OrangeGateLapTracker, RunStartCrossingNeverCounts)
{
  auto tracker = makeGateTracker();
  // Spawn 13 m behind the line — beyond the 12 m arm radius, so the tracker
  // is armed before the car ever moves (small_track spawns outside the tuned
  // 3 m radius the same way). The departure crossing is the RACE START:
  // zero laps are complete there, so it must start the timer, not count.
  EXPECT_FALSE(tracker.observeEgo(-13.0, 0.0, 0.0));
  EXPECT_TRUE(tracker.armed());
  EXPECT_FALSE(walk(tracker, -13.0, 1.0, 10.0));
  EXPECT_EQ(tracker.countedLaps(), 0);
  EXPECT_FALSE(tracker.hasLapTime());
  // The crossing lands on the walk's second-to-last sample (t = 9.5).
  EXPECT_NEAR(tracker.currentLapElapsedSec(20.0), 10.5, 1.0e-9);
}

TEST(OrangeGateLapTracker, CountsArmedCrossingsAndTimesLaps)
{
  auto tracker = makeGateTracker();
  EXPECT_EQ(tracker.countedLaps(), 0);
  EXPECT_FALSE(walk(tracker, -2.0, 1.0, 1.0));       // race start, t=1

  EXPECT_TRUE(driveLap(tracker, 40.0));              // lap 1 at t=40
  ASSERT_TRUE(tracker.hasLapTime());
  EXPECT_EQ(tracker.countedLaps(), 1);
  EXPECT_NEAR(tracker.lastLapSec(), 39.0, 1.0e-9);
  EXPECT_NEAR(tracker.bestLapSec(), 39.0, 1.0e-9);

  EXPECT_TRUE(driveLap(tracker, 78.0));              // lap 2 at t=78
  EXPECT_EQ(tracker.countedLaps(), 2);
  EXPECT_NEAR(tracker.lastLapSec(), 38.0, 1.0e-9);
  EXPECT_NEAR(tracker.bestLapSec(), 38.0, 1.0e-9);
}

// The published lap count is max(gate, fallback), so a gate that never
// registers a crossing cannot wedge the count below the fallback's progress.
TEST(OrangeGateLapTracker, StalledGateDoesNotWedgeCombinedCount)
{
  auto tracker = makeGateTracker();
  // Ego drives near but never actually crosses the gate segment: no gate laps.
  EXPECT_FALSE(tracker.observeEgo(-30.0, 5.0, 0.0));
  EXPECT_FALSE(tracker.observeEgo(-3.0, 5.0, 1.0));
  EXPECT_FALSE(tracker.observeEgo(-3.0, 8.0, 2.0));
  EXPECT_EQ(tracker.countedLaps(), 0);
  // The node combines this with the frenet/discovery fallback via max, so with
  // gate=0 the published count tracks the fallback (verified here as a plain
  // max, matching recomputeLapCount).
  const int fallback = 3;
  EXPECT_EQ(std::max(tracker.countedLaps(), fallback), 3);
}

TEST(OrangeGateLapTracker, LineJitterAndReverseDoNotCount)
{
  auto tracker = makeGateTracker();
  EXPECT_FALSE(walk(tracker, -2.0, 1.0, 1.0));       // race start
  EXPECT_TRUE(driveLap(tracker, 41.0));              // lap 1, disarms
  // Jitter back and forth across the line without re-arming: nothing.
  EXPECT_FALSE(tracker.observeEgo(-1.0, 0.0, 43.0));
  EXPECT_FALSE(tracker.observeEgo(1.0, 0.0, 43.5));
  // Re-arm far past the line, then cross BACKWARD: direction mismatch.
  EXPECT_FALSE(walk(tracker, 1.0, 15.0, 60.0));
  EXPECT_FALSE(walk(tracker, 15.0, -1.0, 70.0));
  EXPECT_EQ(tracker.countedLaps(), 1);
}

TEST(OrangeGateLapTracker, PoseSnapAcrossTheLineDoesNotCross)
{
  auto tracker = makeGateTracker();
  EXPECT_FALSE(walk(tracker, -2.0, 1.0, 1.0));       // race start
  EXPECT_TRUE(driveLap(tracker, 41.0));              // real lap 1
  // Relocalisation snap from far behind to far past the line: the >2 m step
  // is a measurement discontinuity, the spanned segment never happened.
  EXPECT_FALSE(tracker.observeEgo(-14.0, 0.0, 60.0));
  EXPECT_FALSE(tracker.observeEgo(14.0, 0.0, 60.5));
  EXPECT_EQ(tracker.countedLaps(), 1);
}

TEST(OrangeGateLapTracker, RefittedGateSweepingOverEgoResetsInsteadOfCounting)
{
  auto tracker = makeGateTracker();
  // Young-map gate 13 m ahead of a stationary ego: armed by distance alone.
  EXPECT_FALSE(tracker.observeEgo(-13.0, 0.0, 0.0));
  EXPECT_FALSE(tracker.observeEgo(-12.9, 0.0, 0.5));
  EXPECT_TRUE(tracker.armed());
  // SLAM refits the gate to the ego's far side (way beyond the 0.75 m move
  // threshold): arming and crossing history reset with the new line, so the
  // sweep over the stationary car cannot read as a crossing.
  EXPECT_TRUE(tracker.updateGate({{{-20.0, 1.5}}, {{-20.0, -1.5}}}));
  EXPECT_FALSE(tracker.armed());
  EXPECT_FALSE(tracker.observeEgo(-12.8, 0.0, 1.0));
  EXPECT_EQ(tracker.countedLaps(), 0);
}

}
}
