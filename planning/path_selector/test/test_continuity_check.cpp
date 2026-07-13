#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "path_selector/continuity_check.hpp"

namespace path_selector
{
namespace
{

constexpr double kNowSec = 10.0;
constexpr double kReceiveSec = 9.8;

eufs_msgs::msg::WaypointArrayStamped makePath(
  const std::vector<std::pair<double, double>> & points,
  const std::string & frame = "map")
{
  eufs_msgs::msg::WaypointArrayStamped path;
  path.header.frame_id = frame;
  path.header.stamp.sec = 9;
  path.header.stamp.nanosec = 800000000U;
  for (std::size_t i = 0; i < points.size(); ++i) {
    eufs_msgs::msg::Waypoint waypoint;
    waypoint.position.x = points[i].first;
    waypoint.position.y = points[i].second;
    waypoint.x_m = points[i].first;
    waypoint.y_m = points[i].second;
    waypoint.s_m = static_cast<double>(i);
    if (i + 1U < points.size()) {
      waypoint.psi_rad = std::atan2(
        points[i + 1U].second - points[i].second,
        points[i + 1U].first - points[i].first);
    }
    path.waypoints.push_back(waypoint);
  }
  return path;
}

nav_msgs::msg::Odometry makeOdometry(double x = 0.0, double y = 0.0)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.header.stamp.sec = 9;
  odometry.header.stamp.nanosec = 800000000U;
  odometry.child_frame_id = "base_footprint";
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.orientation.w = 1.0;
  return odometry;
}

CandidateInput makeCandidate(const std::vector<std::pair<double, double>> & points)
{
  return CandidateInput{makePath(points), kReceiveSec, true, true, kReceiveSec};
}

ContinuityInputs makeContinuityInputs(
  const std::vector<std::pair<double, double>> & local,
  const std::vector<std::pair<double, double>> & global)
{
  return ContinuityInputs{
    makeCandidate(local),
    makeCandidate(global),
    OdometryInput{makeOdometry(), kReceiveSec},
    kNowSec,
    kNowSec};
}

TEST(ContinuityCheckTest, RejectsStaleMalformedOrWrongFramePath)
{
  const ContinuityCheck check;
  auto stale = makeCandidate({{0.0, 0.0}, {6.0, 0.0}});
  stale.path_receive_time_sec = 9.49;
  auto malformed = makeCandidate({{0.0, 0.0}, {6.0, 0.0}});
  malformed.path->waypoints.back().x_m = std::numeric_limits<double>::quiet_NaN();
  auto wrong_frame = makeCandidate({{0.0, 0.0}, {6.0, 0.0}});
  wrong_frame.path->header.frame_id = "base_footprint";

  EXPECT_EQ(
    check.validateCandidate(stale, kNowSec, kNowSec).failure,
    ContinuityFailure::StalePath);
  EXPECT_EQ(
    check.validateCandidate(malformed, kNowSec, kNowSec).failure,
    ContinuityFailure::MalformedPath);
  EXPECT_EQ(
    check.validateCandidate(wrong_frame, kNowSec, kNowSec).failure,
    ContinuityFailure::WrongPathFrame);
}

TEST(ContinuityCheckTest, AcceptsBoundedFutureHeaderStampsButRejectsLargerSkew)
{
  const ContinuityCheck check;
  auto path_with_observed_skew = makeCandidate({{0.0, 0.0}, {6.0, 0.0}});
  path_with_observed_skew.path->header.stamp.sec = 10;
  path_with_observed_skew.path->header.stamp.nanosec = 211000000U;
  auto path_beyond_tolerance = path_with_observed_skew;
  path_beyond_tolerance.path->header.stamp.nanosec = 250000001U;

  auto odom_with_observed_skew = OdometryInput{makeOdometry(), kReceiveSec};
  odom_with_observed_skew.odometry->header.stamp.sec = 10;
  odom_with_observed_skew.odometry->header.stamp.nanosec = 211000000U;
  auto odom_beyond_tolerance = odom_with_observed_skew;
  odom_beyond_tolerance.odometry->header.stamp.nanosec = 250000001U;

  EXPECT_TRUE(check.validateCandidate(path_with_observed_skew, kNowSec, kNowSec).ready);
  EXPECT_EQ(
    check.validateCandidate(path_beyond_tolerance, kNowSec, kNowSec).failure,
    ContinuityFailure::StalePath);
  EXPECT_TRUE(check.validateOdometry(odom_with_observed_skew, kNowSec, kNowSec).ready);
  EXPECT_EQ(
    check.validateOdometry(odom_beyond_tolerance, kNowSec, kNowSec).failure,
    ContinuityFailure::StaleOdometry);
}

TEST(ContinuityCheckTest, ContinuityReadyRequiresBoundedPositionHeadingAndLength)
{
  const ContinuityCheck check;
  const auto at_position_limit = check.evaluate(makeContinuityInputs(
      {{0.0, 0.0}, {5.0, 0.0}},
      {{1.5, 0.0}, {6.5, 0.0}}));
  const auto beyond_position_limit = check.evaluate(makeContinuityInputs(
      {{0.0, 0.0}, {5.0, 0.0}},
      {{1.51, 0.0}, {6.51, 0.0}}));
  const auto at_heading_limit = check.evaluate(makeContinuityInputs(
      {{0.0, 0.0}, {5.0, 0.0}},
      {{0.0, 0.0}, {5.0 * std::cos(0.52), 5.0 * std::sin(0.52)}}));
  const auto beyond_heading_limit = check.evaluate(makeContinuityInputs(
      {{0.0, 0.0}, {5.0, 0.0}},
      {{0.0, 0.0}, {5.0 * std::cos(0.53), 5.0 * std::sin(0.53)}}));
  const auto short_path = check.evaluate(makeContinuityInputs(
      {{0.0, 0.0}, {4.99, 0.0}},
      {{0.0, 0.0}, {5.0, 0.0}}));

  EXPECT_TRUE(at_position_limit.ready);
  EXPECT_FALSE(beyond_position_limit.ready);
  EXPECT_EQ(
    beyond_position_limit.failure,
    ContinuityFailure::ExcessiveStartSeparation);
  EXPECT_TRUE(at_heading_limit.ready);
  EXPECT_FALSE(beyond_heading_limit.ready);
  EXPECT_EQ(
    beyond_heading_limit.failure,
    ContinuityFailure::ExcessiveHeadingDifference);
  EXPECT_FALSE(short_path.ready);
  EXPECT_EQ(short_path.failure, ContinuityFailure::InsufficientCommonLength);
}

TEST(ContinuityCheckTest, UTurnPathsRemainHandoffReadyWhenStartsAndHeadingsMatch)
{
  const ContinuityCheck check;
  const auto result = check.evaluate(makeContinuityInputs(
      {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {-2.0, 2.0}},
      {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {-2.0, 2.0}}));

  EXPECT_TRUE(result.ready) << toString(result.failure);
  EXPECT_EQ(result.failure, ContinuityFailure::None);
  EXPECT_GE(result.common_forward_length_m, check.thresholds().minimum_common_length_m);
}

TEST(ContinuityCheckTest, FailureReasonsAreDistinctForClosedHandoffCases)
{
  const ContinuityCheck check;
  auto stale_path = makeCandidate({{0.0, 0.0}, {6.0, 0.0}});
  stale_path.path_receive_time_sec = 9.49;
  auto stale_odometry = OdometryInput{makeOdometry(), 9.49};

  EXPECT_STREQ(
    toString(check.validateCandidate(stale_path, kNowSec, kNowSec).failure),
    "stale_path");
  EXPECT_STREQ(
    toString(check.validateOdometry(stale_odometry, kNowSec, kNowSec).failure),
    "stale_odometry");
  EXPECT_STREQ(
    toString(check.evaluate(makeContinuityInputs(
        {{0.0, 0.0}, {5.0, 0.0}},
        {{1.51, 0.0}, {6.51, 0.0}})).failure),
    "excessive_start_separation");
  EXPECT_STREQ(
    toString(check.evaluate(makeContinuityInputs(
        {{0.0, 0.0}, {4.99, 0.0}},
        {{0.0, 0.0}, {5.0, 0.0}})).failure),
    "insufficient_common_length");
  EXPECT_STREQ(
    toString(check.evaluate(makeContinuityInputs(
        {{0.0, 0.0}, {5.0, 0.0}},
        {{0.0, 0.0}, {-5.0, 0.0}})).failure),
    "excessive_heading_difference");
}

TEST(ContinuityCheckTest, AttributesMissingGlobalPathAsGlobalCandidateFailure)
{
  const ContinuityCheck check;
  auto inputs = makeContinuityInputs(
    {{0.0, 0.0}, {6.0, 0.0}},
    {{0.0, 0.0}, {6.0, 0.0}});
  inputs.global.path = std::nullopt;

  const auto result = check.evaluate(inputs);

  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.failure, ContinuityFailure::MissingPath);
  EXPECT_EQ(result.local_candidate_failure, ContinuityFailure::None);
  EXPECT_EQ(result.global_candidate_failure, ContinuityFailure::MissingPath);
  EXPECT_EQ(result.odometry_failure, ContinuityFailure::NotImplemented);
}

TEST(ContinuityCheckTest, SourceChangeTrimsAtEgoNearestPoint)
{
  const ContinuityCheck check;
  const auto path = makePath({{-2.0, 0.0}, {0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}});

  const auto result = check.trimAtEgoNearestPoint(path, makeOdometry(1.0, 0.0));

  ASSERT_TRUE(result.success());
  EXPECT_EQ(result.original_start_index, 1U);
  ASSERT_EQ(result.path->waypoints.size(), 3U);
  EXPECT_DOUBLE_EQ(result.path->waypoints.front().x_m, 0.0);
}

}
}
