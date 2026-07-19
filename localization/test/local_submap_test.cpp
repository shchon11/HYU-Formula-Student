// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "hyu_localization/local_submap.hpp"

namespace hyu_localization
{

namespace
{

constexpr double kHalfPi = 1.5707963267948966;

std::vector<SubmapPoint> onePoint(double x, double y, std::uint8_t color)
{
  return {SubmapPoint{x, y, color}};
}

}  // namespace

TEST(LocalSubmapSe2, ComposeInverseRoundTrip)
{
  const GateSe2 a{1.0, 2.0, 0.7};
  const GateSe2 b{-0.5, 0.3, -1.2};
  const GateSe2 ab = composeSe2(a, b);
  const GateSe2 back = composeSe2(inverseSe2(a), ab);
  EXPECT_NEAR(back.x, b.x, 1e-12);
  EXPECT_NEAR(back.y, b.y, 1e-12);
  EXPECT_NEAR(back.theta, b.theta, 1e-12);
}

TEST(LocalSubmap, ReExpressesOldFramesInReferenceBody)
{
  LocalConeSubmap submap;
  // Frame 1: car at origin facing +x, cone 5 m ahead -> map (5, 0).
  submap.addFrame(GateSe2{0.0, 0.0, 0.0}, onePoint(5.0, 0.0, 0U), 0.0);
  // Frame 2: car at (4, 0) facing +y.
  const GateSe2 reference{4.0, 0.0, kHalfPi};
  submap.addFrame(reference, onePoint(2.0, 1.0, 1U), 4.0);

  const std::vector<SubmapPoint> points = submap.pointsInFrame(reference);
  ASSERT_EQ(points.size(), 2U);
  // The frame-1 cone at map (5, 0) seen from (4, 0, +90deg): body frame
  // x-forward is map +y, body y-left is map -x -> (0, -1).
  const SubmapPoint & old_cone = points[0].color == 0U ? points[0] : points[1];
  EXPECT_NEAR(old_cone.x, 0.0, 1e-12);
  EXPECT_NEAR(old_cone.y, -1.0, 1e-12);
}

TEST(LocalSubmap, DedupesRepeatedSightingsOfOneCone)
{
  LocalConeSubmap submap;
  // The same physical cone (map ~(10, 0)) seen from three poses.
  submap.addFrame(GateSe2{0.0, 0.0, 0.0}, onePoint(10.0, 0.05, 0U), 0.0);
  submap.addFrame(GateSe2{2.0, 0.0, 0.0}, onePoint(8.0, -0.05, 0U), 2.0);
  submap.addFrame(GateSe2{4.0, 0.0, 0.0}, onePoint(6.0, 0.0, 0U), 4.0);

  const std::vector<SubmapPoint> points =
    submap.pointsInFrame(GateSe2{4.0, 0.0, 0.0});
  ASSERT_EQ(points.size(), 1U);
  EXPECT_NEAR(points[0].x, 6.0, 0.1);
  EXPECT_NEAR(points[0].y, 0.0, 0.1);
}

TEST(LocalSubmap, DifferentColorsNeverMerge)
{
  LocalConeSubmap submap;
  std::vector<SubmapPoint> frame = {
    SubmapPoint{5.0, 0.0, 0U},
    SubmapPoint{5.2, 0.0, 1U}};
  submap.addFrame(GateSe2{0.0, 0.0, 0.0}, frame, 0.0);
  EXPECT_EQ(submap.pointsInFrame(GateSe2{0.0, 0.0, 0.0}).size(), 2U);
}

TEST(LocalSubmap, PrunesBeyondTravelSpan)
{
  LocalSubmapParams params;
  params.span_m = 10.0;
  LocalConeSubmap submap(params);
  submap.addFrame(GateSe2{0.0, 0.0, 0.0}, onePoint(1.0, 0.0, 0U), 0.0);
  submap.addFrame(GateSe2{5.0, 0.0, 0.0}, onePoint(1.0, 0.0, 0U), 5.0);
  submap.addFrame(GateSe2{15.0, 0.0, 0.0}, onePoint(1.0, 0.0, 0U), 15.0);
  // Travel 15 - 0 > 10: the first frame must be gone.
  EXPECT_EQ(submap.frameCount(), 2U);
}

TEST(LocalSubmap, StationaryFramesReplaceInsteadOfFlooding)
{
  LocalConeSubmap submap;
  submap.addFrame(GateSe2{0.0, 0.0, 0.0}, onePoint(1.0, 0.0, 0U), 0.0);
  submap.addFrame(GateSe2{0.5, 0.0, 0.0}, onePoint(1.0, 0.0, 0U), 0.5);
  for (int i = 0; i < 100; ++i) {
    // No travel: each new frame replaces the newest, older history stays.
    submap.addFrame(GateSe2{0.51, 0.0, 0.0}, onePoint(1.0, 0.0, 1U), 0.51);
  }
  EXPECT_EQ(submap.frameCount(), 2U);
}

TEST(LocalSubmap, TravelResetClearsHistory)
{
  LocalConeSubmap submap;
  submap.addFrame(GateSe2{0.0, 0.0, 0.0}, onePoint(1.0, 0.0, 0U), 100.0);
  // Travel accounting restarted (relocalize): stale frames must not mix.
  submap.addFrame(GateSe2{1.0, 0.0, 0.0}, onePoint(1.0, 0.0, 0U), 0.0);
  EXPECT_EQ(submap.frameCount(), 1U);
}

TEST(SubmapScore, ColorAwareInlierCounting)
{
  // Two cones ahead; the map has a blue target at the blue cone, a yellow
  // target where the ORANGE cone is.
  const std::vector<SubmapPoint> body = {
    SubmapPoint{5.0, 1.0, 0U},   // blue
    SubmapPoint{5.0, -1.0, 2U},  // orange
  };
  const std::vector<SubmapPoint> targets = {
    SubmapPoint{5.0, 1.0, 0U},   // blue: compatible
    SubmapPoint{5.0, -1.0, 1U},  // yellow: NOT compatible with orange
  };
  const SubmapScore score =
    scoreSubmapPose(body, targets, GateSe2{0.0, 0.0, 0.0}, 0.5);
  EXPECT_EQ(score.inliers, 1);
}

TEST(SubmapScore, OrangeBigOrangeAndUnknownAreCompatible)
{
  const std::vector<SubmapPoint> body = {
    SubmapPoint{5.0, 1.0, 2U},   // orange obs vs big-orange target
    SubmapPoint{5.0, -1.0, 4U},  // unknown obs vs blue target
  };
  const std::vector<SubmapPoint> targets = {
    SubmapPoint{5.0, 1.0, 3U},
    SubmapPoint{5.0, -1.0, 0U},
  };
  const SubmapScore score =
    scoreSubmapPose(body, targets, GateSe2{0.0, 0.0, 0.0}, 0.5);
  EXPECT_EQ(score.inliers, 2);
}

TEST(SubmapScore, RecoversAppliedOffset)
{
  // A 12-cone lane constellation; score must peak at the true pose, not at
  // a 1-cone-spacing shift (the aliasing case a single frame cannot reject).
  std::vector<SubmapPoint> targets;
  for (int i = 0; i < 6; ++i) {
    targets.push_back(SubmapPoint{3.0 * i, 1.5, 0U});
    targets.push_back(SubmapPoint{3.0 * i, -1.5, 1U});
  }
  std::vector<SubmapPoint> body;
  for (const SubmapPoint & t : targets) {
    body.push_back(SubmapPoint{t.x - 4.0, t.y, t.color});  // car at (4, 0)
  }
  const SubmapScore at_truth =
    scoreSubmapPose(body, targets, GateSe2{4.0, 0.0, 0.0}, 0.5);
  const SubmapScore shifted =
    scoreSubmapPose(body, targets, GateSe2{7.0, 0.0, 0.0}, 0.5);
  EXPECT_EQ(at_truth.inliers, 12);
  // The shifted pose still aliases geometrically (regular spacing) but the
  // trailing cones fall off the constellation: strictly fewer inliers.
  EXPECT_LT(shifted.inliers, at_truth.inliers);
}

}  // namespace hyu_localization

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
