#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "state_machine/planning_state_machine_node.hpp"

namespace state_machine
{
namespace
{

eufs_msgs::msg::WaypointArrayStamped makePath(
  const std::vector<double> & s_values,
  const std::vector<std::pair<double, double>> & xy,
  const std::string & frame_id = "map")
{
  eufs_msgs::msg::WaypointArrayStamped msg;
  msg.header.frame_id = frame_id;
  for (std::size_t index = 0; index < s_values.size(); ++index) {
    eufs_msgs::msg::Waypoint waypoint;
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

}
}
