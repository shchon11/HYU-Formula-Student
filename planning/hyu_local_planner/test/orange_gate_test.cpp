#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "hyu_local_planner/local_path_builder.hpp"

// use_orange_cones: orange/big-orange cones feed the boundaries through the
// unknown-cone classification outside the straight-corridor mission. The
// motivating failure is a closed lap on trackdrive: at the finish straight the
// ROI can hold nothing but the orange start/finish gate, the frame produced
// zero boundary cones, and the planner braked the car mid-track.

namespace hyu_local_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

PlannerConfig slamModeConfig(bool use_orange_cones)
{
  PlannerConfig config;
  config.allow_partial_boundary = true;  // slam_map source mode
  config.use_orange_cones = use_orange_cones;
  return config;
}

double maxHeadingStep(const std::vector<PathWaypoint> & waypoints)
{
  double worst = 0.0;
  for (std::size_t i = 1; i + 1U < waypoints.size(); ++i) {
    double step = std::abs(waypoints[i].psi - waypoints[i - 1U].psi);
    if (step > kPi) {
      step = 2.0 * kPi - step;
    }
    worst = std::max(worst, step);
  }
  return worst;
}

void expectSameWaypoints(const BuildResult & a, const BuildResult & b)
{
  ASSERT_EQ(a.valid, b.valid);
  ASSERT_EQ(a.kind, b.kind);
  ASSERT_EQ(a.waypoints.size(), b.waypoints.size());
  for (std::size_t i = 0; i < a.waypoints.size(); ++i) {
    EXPECT_EQ(a.waypoints[i].x, b.waypoints[i].x);
    EXPECT_EQ(a.waypoints[i].y, b.waypoints[i].y);
    EXPECT_EQ(a.waypoints[i].s, b.waypoints[i].s);
    EXPECT_EQ(a.waypoints[i].psi, b.waypoints[i].psi);
    EXPECT_EQ(a.waypoints[i].kappa, b.waypoints[i].kappa);
    EXPECT_EQ(a.waypoints[i].speed, b.waypoints[i].speed);
  }
}

// The reported failure, reproduced: every blue/yellow cone has fallen behind
// the ROI at lap close and only the big-orange gate is ahead.
ConeSet lapCloseFrame()
{
  ConeSet cones;
  for (const double x : {-8.0, -6.0, -4.0}) {
    cones.blue.push_back({x, 1.75});
    cones.yellow.push_back({x, -1.75});
  }
  cones.big_orange = {{3.0, 1.6}, {5.0, 1.6}, {3.0, -1.6}, {5.0, -1.6}};
  return cones;
}

TEST(OrangeGate, LapCloseFrameWasInvalidWithoutOrangeCones)
{
  const BuildResult result = buildLocalPath(lapCloseFrame(), slamModeConfig(false));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "roi_no_boundary_cones");
}

TEST(OrangeGate, LapCloseFramePlansThroughTheGate)
{
  const BuildResult result = buildLocalPath(lapCloseFrame(), slamModeConfig(true));
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  ASSERT_GE(result.waypoints.size(), 2U);
  // The path must run forward through the gate midline, not fold or veer.
  EXPECT_NEAR(result.waypoints.front().y, 0.0, 0.5);
  EXPECT_NEAR(result.waypoints.back().y, 0.0, 0.5);
  EXPECT_GT(result.waypoints.back().x, result.waypoints.front().x);
  EXPECT_LT(maxHeadingStep(result.waypoints), 0.5 * kPi);
}

TEST(OrangeGate, GateOnlyFramePlansTwoSided)
{
  ConeSet cones;
  cones.big_orange = {{2.0, 1.6}, {4.0, 1.6}, {2.0, -1.6}, {4.0, -1.6}};
  const BuildResult result = buildLocalPath(cones, slamModeConfig(true));
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  EXPECT_LT(maxHeadingStep(result.waypoints), 0.5 * kPi);
}

// Orange cones on the centreline (inside the dead-band, out of absorb reach of
// either boundary line) must change nothing: the classification drops them.
TEST(OrangeGate, CentredOrangeConesAreDropped)
{
  ConeSet cones;
  for (const double x : {1.0, 4.0, 7.0, 10.0}) {
    cones.blue.push_back({x, 1.75});
    cones.yellow.push_back({x, -1.75});
  }
  ConeSet with_centred = cones;
  with_centred.orange = {{3.0, 0.0}, {6.0, 0.2}, {9.0, -0.2}};

  const BuildResult reference = buildLocalPath(cones, slamModeConfig(true));
  const BuildResult centred = buildLocalPath(with_centred, slamModeConfig(true));
  ASSERT_TRUE(reference.valid) << reference.reason;
  expectSameWaypoints(reference, centred);
}

// Gate cones sitting on the labelled boundary lines absorb instead of
// perturbing the path: the frame stays valid and fold-free.
TEST(OrangeGate, GateConesAbsorbOntoLabelledBoundaries)
{
  ConeSet cones;
  for (const double x : {1.0, 4.0, 7.0, 10.0}) {
    cones.blue.push_back({x, 1.75});
    cones.yellow.push_back({x, -1.75});
  }
  cones.big_orange = {{5.5, 1.7}, {5.5, -1.7}};
  const BuildResult result = buildLocalPath(cones, slamModeConfig(true));
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.kind, PathKind::kTwoSided);
  EXPECT_LT(maxHeadingStep(result.waypoints), 0.5 * kPi);
}

// The acceleration mission keeps its own straight-corridor fold-in: with
// extend_straight_to_horizon the flag must be inert, bit for bit.
TEST(OrangeGate, StraightCorridorMissionIsUnaffected)
{
  ConeSet cones;
  for (const double x : {0.0, 3.0, 6.0, 9.0}) {
    cones.blue.push_back({x, 1.6});
    cones.yellow.push_back({x, -1.6});
  }
  cones.big_orange = {{12.0, 1.6}, {12.0, -1.6}};
  cones.orange = {{15.0, 1.6}, {15.0, -1.6}, {18.0, 1.6}, {18.0, -1.6}};

  PlannerConfig with_flag = slamModeConfig(true);
  with_flag.extend_straight_to_horizon = true;
  with_flag.roi_min_x = -15.0;
  PlannerConfig without_flag = with_flag;
  without_flag.use_orange_cones = false;

  const BuildResult a = buildLocalPath(cones, with_flag);
  const BuildResult b = buildLocalPath(cones, without_flag);
  ASSERT_TRUE(a.valid) << a.reason;
  expectSameWaypoints(a, b);
}

}  // namespace
}  // namespace hyu_local_planner
