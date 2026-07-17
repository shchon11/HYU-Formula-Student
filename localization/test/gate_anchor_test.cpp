// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "eufs_graph_slam/gate_anchor.hpp"

namespace
{

using eufs_graph_slam::GateMatchParams;
using eufs_graph_slam::GateMatchResult;
using eufs_graph_slam::GatePoint;
using eufs_graph_slam::GateSe2;
using eufs_graph_slam::clusterCentroids;
using eufs_graph_slam::matchGateConstellation;

GatePoint toBody(const GateSe2 & pose, const GatePoint & map_point)
{
  // Inverse of gate_anchor_detail::transform: map -> body.
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  const double dx = map_point[0] - pose.x;
  const double dy = map_point[1] - pose.y;
  return GatePoint{c * dx + s * dy, -s * dx + c * dy};
}

// A start-gate scene: two gate sides (two big cones each, 0.4 m apart,
// sides 4 m apart) plus a corridor of ordinary cones behind the gate.
struct Scene
{
  std::vector<GatePoint> gate;
  std::vector<GatePoint> track;

  std::vector<GatePoint> allLandmarks() const
  {
    std::vector<GatePoint> all = gate;
    all.insert(all.end(), track.begin(), track.end());
    return all;
  }
};

Scene makeScene()
{
  Scene scene;
  scene.gate = {
    {0.0, -2.0}, {0.4, -2.0},   // right side pair
    {0.0, 2.0}, {0.4, 2.0},     // left side pair
  };
  for (int i = 1; i <= 6; ++i) {
    scene.track.push_back(GatePoint{i * 3.0, -2.0});
    scene.track.push_back(GatePoint{i * 3.0, 2.0});
  }
  return scene;
}

// Observations of the whole scene from a true pose, expressed in body frame.
std::vector<GatePoint> observe(
  const Scene & scene, const GateSe2 & true_pose, bool gate_only)
{
  std::vector<GatePoint> obs;
  for (const GatePoint & p : scene.gate) {
    obs.push_back(toBody(true_pose, p));
  }
  if (!gate_only) {
    for (const GatePoint & p : scene.track) {
      obs.push_back(toBody(true_pose, p));
    }
  }
  return obs;
}

TEST(GateAnchorTest, ClusterCollapsesGateSidePairs)
{
  const Scene scene = makeScene();
  const auto sides = clusterCentroids(scene.gate, 1.5);
  ASSERT_EQ(sides.size(), 2U);
  EXPECT_NEAR(sides[0][0], 0.2, 1e-9);
  EXPECT_NEAR(sides[0][1], -2.0, 1e-9);
  EXPECT_NEAR(sides[1][1], 2.0, 1e-9);
}

TEST(GateAnchorTest, RecoversPoseUnderLargeDrift)
{
  // The pose the CAR is actually at; the SLAM estimate could be 30 m away —
  // the matcher never consumes the estimate, so drift size is irrelevant.
  const GateSe2 true_pose{-4.0, 0.5, 0.15};
  const Scene scene = makeScene();
  const auto all_obs = observe(scene, true_pose, false);
  const auto gate_obs = observe(scene, true_pose, true);

  GateMatchResult result;
  ASSERT_TRUE(
    matchGateConstellation(
      gate_obs, scene.gate, all_obs, scene.allLandmarks(),
      GateMatchParams{}, &result));
  EXPECT_NEAR(result.pose.x, true_pose.x, 1e-6);
  EXPECT_NEAR(result.pose.y, true_pose.y, 1e-6);
  EXPECT_NEAR(result.pose.theta, true_pose.theta, 1e-6);
  EXPECT_GE(result.inliers, static_cast<int>(all_obs.size()) - 1);
}

TEST(GateAnchorTest, InlierVerificationRejectsTheFlippedCandidate)
{
  // Gate-only observations make the 180-degree flip (left/right sides
  // swapped) fit the gate equally well; the surrounding track cones are what
  // disambiguate. With them observed, the returned pose must be the true
  // one, not the flip.
  const GateSe2 true_pose{-4.0, 0.0, 0.0};
  const Scene scene = makeScene();
  const auto all_obs = observe(scene, true_pose, false);
  const auto gate_obs = observe(scene, true_pose, true);

  GateMatchResult result;
  ASSERT_TRUE(
    matchGateConstellation(
      gate_obs, scene.gate, all_obs, scene.allLandmarks(),
      GateMatchParams{}, &result));
  EXPECT_NEAR(std::abs(result.pose.theta), 0.0, 1e-6);
}

TEST(GateAnchorTest, SeparationMismatchYieldsNoMatch)
{
  // Observed "gate" 2 m wide vs a mapped gate 4 m wide: no candidate.
  const Scene scene = makeScene();
  const std::vector<GatePoint> gate_obs = {{1.0, -1.0}, {1.0, 1.0}};

  GateMatchResult result;
  EXPECT_FALSE(
    matchGateConstellation(
      gate_obs, scene.gate, gate_obs, scene.allLandmarks(),
      GateMatchParams{}, &result));
}

TEST(GateAnchorTest, OneVisibleSideIsNotEnough)
{
  const GateSe2 true_pose{-4.0, 0.0, 0.0};
  const Scene scene = makeScene();
  // Only the right-side pair visible -> one cluster -> no baseline.
  const std::vector<GatePoint> gate_obs = {
    toBody(true_pose, scene.gate[0]), toBody(true_pose, scene.gate[1])};

  GateMatchResult result;
  EXPECT_FALSE(
    matchGateConstellation(
      gate_obs, scene.gate, gate_obs, scene.allLandmarks(),
      GateMatchParams{}, &result));
}

TEST(GateAnchorTest, InsufficientInliersRejects)
{
  // A gate match whose implied pose explains nothing else: track cones are
  // observed but the map has them elsewhere (e.g. matching a duplicated
  // drift-era gate). min_inliers must kill it.
  const GateSe2 true_pose{-4.0, 0.0, 0.0};
  const Scene scene = makeScene();
  const auto gate_obs = observe(scene, true_pose, true);

  // Observations include track cones that do NOT exist in the map.
  auto all_obs = gate_obs;
  for (int i = 1; i <= 6; ++i) {
    all_obs.push_back(toBody(true_pose, GatePoint{i * 3.0, -10.0}));
    all_obs.push_back(toBody(true_pose, GatePoint{i * 3.0, 10.0}));
  }

  GateMatchParams params;
  params.min_inliers = 6;  // gate alone (4 cones) must not satisfy this
  std::vector<GatePoint> gate_only_map = scene.gate;

  GateMatchResult result;
  EXPECT_FALSE(
    matchGateConstellation(
      gate_obs, gate_only_map, all_obs, gate_only_map, params, &result));
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
