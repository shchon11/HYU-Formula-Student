// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <vector>

#include "hyu_localization/tentative_track_frontend.hpp"

namespace hyu_localization
{
namespace
{

constexpr std::uint8_t kBlue = 0U;
constexpr std::uint8_t kYellow = 1U;
constexpr std::uint8_t kUnknown = 4U;

FrontendObservation makeObservation(
  double x, double y, double sigma = 0.1, std::uint8_t color = kBlue,
  int pose_graph_id = 0)
{
  FrontendObservation observation;
  observation.map_point = Eigen::Vector2d(x, y);
  observation.map_covariance = Eigen::Matrix2d::Identity() * sigma * sigma;
  observation.keyframe_measurement = Eigen::Vector2d(x, y);
  observation.keyframe_covariance = observation.map_covariance;
  observation.pose_graph_id = pose_graph_id;
  observation.range_m = observation.map_point.norm();
  observation.color = color;
  return observation;
}

FrontendConfirmedLandmark makeLandmark(
  double x, double y, double sigma = 0.1, double last_seen = 0.0)
{
  FrontendConfirmedLandmark landmark;
  landmark.position = Eigen::Vector2d(x, y);
  landmark.covariance = Eigen::Matrix2d::Identity() * sigma * sigma;
  landmark.last_seen_traveled_m = last_seen;
  return landmark;
}

const auto kAlwaysVisible = [](const Eigen::Vector2d &) {return true;};
const auto kNeverVisible = [](const Eigen::Vector2d &) {return false;};

TEST(TentativeTrackFrontend, RepeatedObservationPromotesWithFullReplay)
{
  TentativeTrackFrontend frontend{FrontendParams{}};
  const std::vector<FrontendConfirmedLandmark> no_landmarks;

  FrontendFrameResult result;
  for (int frame = 0; frame < 3; ++frame) {
    result = frontend.processFrame(
      {makeObservation(5.0, 1.0, 0.1, kBlue, frame)},
      no_landmarks, 0.5 * frame, kAlwaysVisible);
  }

  ASSERT_EQ(result.promotions.size(), 1U);
  const FrontendPromotion & promotion = result.promotions.front();
  EXPECT_NEAR(promotion.position.x(), 5.0, 1e-9);
  EXPECT_NEAR(promotion.position.y(), 1.0, 1e-9);
  EXPECT_EQ(promotion.color, kBlue);
  // All three sightings replay into the graph: the delay loses nothing.
  ASSERT_EQ(promotion.observations.size(), 3U);
  EXPECT_EQ(promotion.observations[0].pose_graph_id, 0);
  EXPECT_EQ(promotion.observations[2].pose_graph_id, 2);
  EXPECT_TRUE(frontend.tracks().empty());
}

TEST(TentativeTrackFrontend, SingleBlipNeverReachesTheMapAndDies)
{
  TentativeTrackFrontend frontend{FrontendParams{}};
  const std::vector<FrontendConfirmedLandmark> no_landmarks;

  auto result = frontend.processFrame(
    {makeObservation(5.0, 1.0)}, no_landmarks, 0.0, kAlwaysVisible);
  EXPECT_EQ(result.new_tracks, 1U);
  EXPECT_TRUE(result.promotions.empty());

  // Blip gone; track misses while expected visible and dies.
  std::size_t killed = 0U;
  for (int frame = 0; frame < 4; ++frame) {
    result = frontend.processFrame({}, no_landmarks, 0.5 * frame, kAlwaysVisible);
    killed += result.killed_tracks;
  }
  EXPECT_EQ(killed, 1U);
  EXPECT_TRUE(frontend.tracks().empty());
}

TEST(TentativeTrackFrontend, ObservationNearConfirmedLandmarkMatchesIt)
{
  TentativeTrackFrontend frontend{FrontendParams{}};
  const std::vector<FrontendConfirmedLandmark> landmarks = {makeLandmark(5.0, 1.0)};

  const auto result = frontend.processFrame(
    {makeObservation(5.1, 1.05)}, landmarks, 10.0, kAlwaysVisible);

  ASSERT_EQ(result.confirmed_matches.size(), 1U);
  EXPECT_EQ(result.confirmed_matches.front().landmark_index, 0U);
  EXPECT_EQ(result.new_tracks, 0U);
  EXPECT_TRUE(frontend.tracks().empty());
}

TEST(TentativeTrackFrontend, OffsetTwinIsHeldNotPromoted)
{
  // The seam scenario: a lap-return observation stream offset ~1.6 m from
  // the mapped cone by drift. Legacy founding created the offset twin; the
  // frontend must accumulate a track but HOLD its promotion.
  FrontendParams params;
  TentativeTrackFrontend frontend{params};

  FrontendFrameResult result;
  std::size_t held = 0U;
  for (int frame = 0; frame < 6; ++frame) {
    const double traveled = 200.0 + 0.5 * frame;
    // True twin signature: the mapped cone went STALE (its returns now land
    // 1.6 m away and feed the track instead), and the drift inflation for
    // that gap is still too small for the offset to associate.
    const std::vector<FrontendConfirmedLandmark> landmarks = {
      makeLandmark(5.0, 1.0, 0.1, traveled - 7.0)};
    result = frontend.processFrame(
      {makeObservation(6.6, 1.0, 0.1, kBlue, frame)},
      landmarks, traveled, kAlwaysVisible);
    held += result.held_promotions;
  }

  EXPECT_TRUE(result.promotions.empty());
  EXPECT_GT(held, 0U);
  ASSERT_EQ(frontend.tracks().size(), 1U);
}

TEST(TentativeTrackFrontend, FreshNeighborInsideHoldRadiusStillPromotes)
{
  // Real adjacent cones spaced under the hold radius: the confirmed
  // neighbour keeps being observed (fresh) while the track accumulates —
  // both exist, so the track MUST promote (the autocross missing-teeth
  // regression: radius-only holds ate every sub-2 m spacing).
  TentativeTrackFrontend frontend{FrontendParams{}};

  FrontendFrameResult result;
  for (int frame = 0; frame < 4 && result.promotions.empty(); ++frame) {
    const double traveled = 100.0 + 0.5 * frame;
    const std::vector<FrontendConfirmedLandmark> landmarks = {
      makeLandmark(5.0, 1.0, 0.1, traveled)};  // co-observed: fresh
    result = frontend.processFrame(
      {makeObservation(6.6, 1.0, 0.1, kBlue, frame)},
      landmarks, traveled, kAlwaysVisible);
  }

  ASSERT_EQ(result.promotions.size(), 1U);
  EXPECT_NEAR(result.promotions.front().position.x(), 6.6, 0.05);
}

TEST(TentativeTrackFrontend, AmbiguousObservationIsDropped)
{
  TentativeTrackFrontend frontend{FrontendParams{}};
  // Two well-separated confirmed landmarks; long unseen distance inflates
  // both gates until an equidistant observation falls inside each.
  const std::vector<FrontendConfirmedLandmark> landmarks = {
    makeLandmark(5.0, 1.0, 0.3, 0.0), makeLandmark(5.0, 4.0, 0.3, 0.0)};

  const auto result = frontend.processFrame(
    {makeObservation(5.0, 2.5, 0.3)}, landmarks, 300.0, kAlwaysVisible);

  EXPECT_EQ(result.confirmed_matches.size(), 0U);
  ASSERT_EQ(result.ambiguous_observations.size(), 1U);
  // Ambiguity must not leak into a track founding either.
  EXPECT_EQ(result.new_tracks, 0U);
  EXPECT_TRUE(frontend.tracks().empty());
}

TEST(TentativeTrackFrontend, DuplicatePairIsNotAmbiguous)
{
  // Rivals close to each other are one physical cone mapped twice; the
  // observation must associate with the best instead of being refused
  // (refusing starves loop confirmation — legacy semantics preserved).
  TentativeTrackFrontend frontend{FrontendParams{}};
  const std::vector<FrontendConfirmedLandmark> landmarks = {
    makeLandmark(5.0, 1.0, 0.2, 0.0), makeLandmark(5.0, 1.8, 0.2, 0.0)};

  const auto result = frontend.processFrame(
    {makeObservation(5.0, 1.35, 0.2)}, landmarks, 100.0, kAlwaysVisible);

  EXPECT_TRUE(result.ambiguous_observations.empty());
  ASSERT_EQ(result.confirmed_matches.size(), 1U);
}

TEST(TentativeTrackFrontend, MutualExclusionKeepsAdjacentConesApart)
{
  // Two observations, one confirmed landmark: only one may claim it; the
  // other founds a track instead of collapsing into the same landmark.
  TentativeTrackFrontend frontend{FrontendParams{}};
  const std::vector<FrontendConfirmedLandmark> landmarks = {makeLandmark(5.0, 1.0)};

  const auto result = frontend.processFrame(
    {makeObservation(5.05, 1.0), makeObservation(5.9, 1.0)},
    landmarks, 10.0, kAlwaysVisible);

  ASSERT_EQ(result.confirmed_matches.size(), 1U);
  EXPECT_EQ(result.confirmed_matches.front().observation_index, 0U);
  EXPECT_EQ(result.new_tracks, 1U);
}

TEST(TentativeTrackFrontend, FarFoundingNeedsNearPassesToPromote)
{
  // Born at 12 m the heading-lever sigma dominates (0.035 * 12 = 0.42 m >
  // promotion cap), so far hits alone must not promote; near passes shrink
  // the fused sigma below the cap and promotion follows.
  FrontendParams params;
  TentativeTrackFrontend frontend{params};
  const std::vector<FrontendConfirmedLandmark> no_landmarks;

  FrontendFrameResult result;
  for (int frame = 0; frame < 3; ++frame) {
    result = frontend.processFrame(
      {makeObservation(12.0, 0.0, 0.1, kBlue, frame)},
      no_landmarks, 0.5 * frame, kAlwaysVisible);
    EXPECT_TRUE(result.promotions.empty());
  }
  ASSERT_EQ(frontend.tracks().size(), 1U);

  // Near passes: same cone seen from close range (small lever arm).
  for (int frame = 3; frame < 6 && result.promotions.empty(); ++frame) {
    FrontendObservation near = makeObservation(12.0, 0.0, 0.05, kBlue, frame);
    near.range_m = 3.0;
    result = frontend.processFrame({near}, no_landmarks, 0.5 * frame, kAlwaysVisible);
  }
  ASSERT_EQ(result.promotions.size(), 1U);
  EXPECT_NEAR(result.promotions.front().position.x(), 12.0, 0.05);
}

TEST(TentativeTrackFrontend, OutOfViewTracksAreFrozenNotKilled)
{
  TentativeTrackFrontend frontend{FrontendParams{}};
  const std::vector<FrontendConfirmedLandmark> no_landmarks;

  frontend.processFrame(
    {makeObservation(5.0, 1.0), makeObservation(5.0, 1.0)},
    no_landmarks, 0.0, kAlwaysVisible);
  ASSERT_FALSE(frontend.tracks().empty());

  // Many empty frames with the track outside the sensor view: the miss
  // budget must not tick (occlusions and FOV exits are not evidence of
  // absence).
  std::size_t killed = 0U;
  for (int frame = 0; frame < 10; ++frame) {
    const auto result =
      frontend.processFrame({}, no_landmarks, 0.5 * frame, kNeverVisible);
    killed += result.killed_tracks;
  }
  EXPECT_EQ(killed, 0U);
  EXPECT_FALSE(frontend.tracks().empty());
}

TEST(TentativeTrackFrontend, UnpromotedTrackAgesOutByTravel)
{
  FrontendParams params;
  params.promote_min_hits = 100;  // never promotable in this test
  TentativeTrackFrontend frontend{params};
  const std::vector<FrontendConfirmedLandmark> no_landmarks;

  frontend.processFrame(
    {makeObservation(5.0, 1.0)}, no_landmarks, 0.0, kNeverVisible);
  ASSERT_EQ(frontend.tracks().size(), 1U);

  // Keep re-observing (no misses) while driving far past the age cap: the
  // track must still die rather than absorb observations forever.
  std::size_t killed = 0U;
  for (int frame = 1; frame <= 60 && killed == 0U; ++frame) {
    const auto result = frontend.processFrame(
      {makeObservation(5.0, 1.0)}, no_landmarks, 1.0 * frame, kNeverVisible);
    killed += result.killed_tracks;
  }
  EXPECT_EQ(killed, 1U);
  EXPECT_TRUE(frontend.tracks().empty());
}

TEST(TentativeTrackFrontend, ColorMajorityVoteSurvivesMislabels)
{
  TentativeTrackFrontend frontend{FrontendParams{}};
  const std::vector<FrontendConfirmedLandmark> no_landmarks;

  FrontendFrameResult result;
  const std::uint8_t colors[] = {kBlue, kYellow, kBlue, kUnknown, kBlue};
  for (int frame = 0; frame < 5 && result.promotions.empty(); ++frame) {
    result = frontend.processFrame(
      {makeObservation(5.0, 1.0, 0.1, colors[frame], frame)},
      no_landmarks, 0.5 * frame, kAlwaysVisible);
  }
  ASSERT_EQ(result.promotions.size(), 1U);
  EXPECT_EQ(result.promotions.front().color, kBlue);
}

}  // namespace
}  // namespace hyu_localization
