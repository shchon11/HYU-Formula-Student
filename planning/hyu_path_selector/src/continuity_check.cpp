#include "hyu_path_selector/continuity_check.hpp"
#include "hyu_path_selector/continuity_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hyu_path_selector
{
ContinuityCheck::ContinuityCheck(ContinuityThresholds thresholds)
: thresholds_(thresholds)
{
  if (!std::isfinite(thresholds_.timeout_sec) ||
    !std::isfinite(thresholds_.minimum_common_length_m) ||
    !std::isfinite(thresholds_.maximum_start_separation_m) ||
    !std::isfinite(thresholds_.maximum_heading_difference_rad) ||
    thresholds_.timeout_sec <= 0.0 || thresholds_.minimum_common_length_m <= 0.0 ||
    thresholds_.maximum_start_separation_m < 0.0 ||
    thresholds_.maximum_heading_difference_rad < 0.0)
  {
    throw std::invalid_argument("continuity thresholds must be finite and non-negative");
  }
}

CandidateValidation ContinuityCheck::validateCandidate(
  const CandidateInput & candidate,
  double receive_now_sec,
  double stamp_now_sec) const
{
  if (!candidate.path.has_value()) {
    return {false, ContinuityFailure::MissingPath};
  }
  if (!candidate.validity_received) {
    return {false, ContinuityFailure::MissingValidity};
  }
  if (!candidate.validity) {
    return {false, ContinuityFailure::ValidityFalse};
  }
  if (!continuity_geometry::isFresh(
      candidate.validity_receive_time_sec, receive_now_sec, thresholds_.timeout_sec))
  {
    return {false, ContinuityFailure::StaleValidity};
  }
  if (!continuity_geometry::isFresh(
      candidate.path_receive_time_sec, receive_now_sec, thresholds_.timeout_sec)) {
    return {false, ContinuityFailure::StalePath};
  }

  const auto & path = *candidate.path;
  if (path.header.frame_id != "map") {
    return {false, ContinuityFailure::WrongPathFrame};
  }
  const double stamp_sec = continuity_geometry::stampSeconds(path.header.stamp);
  if (stamp_sec <= 0.0 || !continuity_geometry::isStampFresh(
      stamp_sec, stamp_now_sec, thresholds_.timeout_sec)) {
    return {
      false,
      stamp_sec <= 0.0 ? ContinuityFailure::InvalidPathStamp : ContinuityFailure::StalePath};
  }
  if (path.waypoints.size() < 2U ||
    !std::all_of(path.waypoints.begin(), path.waypoints.end(), continuity_geometry::finiteWaypoint) ||
    !continuity_geometry::hasNonzeroSegment(path))
  {
    return {false, ContinuityFailure::MalformedPath};
  }
  return {true, ContinuityFailure::None};
}

CandidateValidation ContinuityCheck::validateOdometry(
  const OdometryInput & input,
  double receive_now_sec,
  double stamp_now_sec) const
{
  if (!input.odometry.has_value()) {
    return {false, ContinuityFailure::MissingOdometry};
  }
  if (!continuity_geometry::isFresh(
      input.receive_time_sec, receive_now_sec, thresholds_.timeout_sec)) {
    return {false, ContinuityFailure::StaleOdometry};
  }

  const auto & odometry = *input.odometry;
  if (odometry.header.frame_id != "map" || odometry.child_frame_id != "base_footprint") {
    return {false, ContinuityFailure::WrongOdometryFrame};
  }
  const double stamp_sec = continuity_geometry::stampSeconds(odometry.header.stamp);
  if (stamp_sec <= 0.0 || !continuity_geometry::isStampFresh(
      stamp_sec, stamp_now_sec, thresholds_.timeout_sec)) {
    return {
      false,
      stamp_sec <= 0.0 ?
      ContinuityFailure::InvalidOdometryStamp : ContinuityFailure::StaleOdometry};
  }

  const auto & position = odometry.pose.pose.position;
  const auto & orientation = odometry.pose.pose.orientation;
  const double orientation_norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z) || !std::isfinite(orientation_norm) ||
    orientation_norm <= continuity_geometry::kGeometryEpsilon)
  {
    return {false, ContinuityFailure::MalformedOdometry};
  }
  return {true, ContinuityFailure::None};
}

TrimResult ContinuityCheck::trimAtEgoNearestPoint(
  const hyu_msgs::msg::WaypointArrayStamped & path,
  const nav_msgs::msg::Odometry & odometry) const
{
  return continuity_geometry::trimAtEgoNearestPoint(path, odometry);
}

ContinuityResult ContinuityCheck::evaluate(const ContinuityInputs & inputs) const
{
  ContinuityResult result;
  const auto local_validation = validateCandidate(
    inputs.local, inputs.receive_now_sec, inputs.stamp_now_sec);
  result.local_candidate_failure = local_validation.failure;
  if (!local_validation.ready) {
    result.failure = local_validation.failure;
    return result;
  }
  const auto global_validation = validateCandidate(
    inputs.global, inputs.receive_now_sec, inputs.stamp_now_sec);
  result.global_candidate_failure = global_validation.failure;
  if (!global_validation.ready) {
    result.failure = global_validation.failure;
    return result;
  }
  const auto odometry_validation = validateOdometry(
    inputs.odometry, inputs.receive_now_sec, inputs.stamp_now_sec);
  result.odometry_failure = odometry_validation.failure;
  if (!odometry_validation.ready) {
    result.failure = odometry_validation.failure;
    return result;
  }

  const auto local_trimmed = trimAtEgoNearestPoint(*inputs.local.path, *inputs.odometry.odometry);
  result.local_trim_failure = local_trimmed.failure;
  if (!local_trimmed.success()) {
    result.failure = local_trimmed.failure;
    return result;
  }
  const auto global_trimmed = trimAtEgoNearestPoint(*inputs.global.path, *inputs.odometry.odometry);
  result.global_trim_failure = global_trimmed.failure;
  if (!global_trimmed.success()) {
    result.failure = global_trimmed.failure;
    return result;
  }

  result.local_forward_length_m = continuity_geometry::forwardLength(*local_trimmed.path);
  result.global_forward_length_m = continuity_geometry::forwardLength(*global_trimmed.path);
  result.common_forward_length_m = std::min(
    result.local_forward_length_m, result.global_forward_length_m);
  const auto & local_start = local_trimmed.path->waypoints.front();
  const auto & global_start = global_trimmed.path->waypoints.front();
  result.start_separation_m = std::hypot(
    local_start.x_m - global_start.x_m,
    local_start.y_m - global_start.y_m);

  const auto local_heading = continuity_geometry::startHeading(*local_trimmed.path);
  const auto global_heading = continuity_geometry::startHeading(*global_trimmed.path);
  if (!local_heading.has_value() || !global_heading.has_value()) {
    result.failure = ContinuityFailure::MalformedPath;
    return result;
  }
  result.heading_difference_rad = continuity_geometry::wrappedAbsoluteDifference(
    *local_heading, *global_heading);

  if (result.common_forward_length_m < thresholds_.minimum_common_length_m) {
    result.failure = ContinuityFailure::InsufficientCommonLength;
    return result;
  }
  if (result.start_separation_m > thresholds_.maximum_start_separation_m) {
    result.failure = ContinuityFailure::ExcessiveStartSeparation;
    return result;
  }
  if (result.heading_difference_rad > thresholds_.maximum_heading_difference_rad) {
    result.failure = ContinuityFailure::ExcessiveHeadingDifference;
    return result;
  }

  result.ready = true;
  result.failure = ContinuityFailure::None;
  return result;
}

nav_msgs::msg::Path ContinuityCheck::toPath(
  const hyu_msgs::msg::WaypointArrayStamped & waypoints) const
{
  return continuity_geometry::toPath(waypoints);
}

const ContinuityThresholds & ContinuityCheck::thresholds() const
{
  return thresholds_;
}

bool TrimResult::success() const
{
  return path.has_value() && failure == ContinuityFailure::None;
}

const char * toString(ContinuityFailure failure)
{
  switch (failure) {
    case ContinuityFailure::None:
      return "none";
    case ContinuityFailure::NotImplemented:
      return "not_implemented";
    case ContinuityFailure::MissingPath:
      return "missing_path";
    case ContinuityFailure::MissingValidity:
      return "missing_validity";
    case ContinuityFailure::ValidityFalse:
      return "validity_false";
    case ContinuityFailure::StaleValidity:
      return "stale_validity";
    case ContinuityFailure::StalePath:
      return "stale_path";
    case ContinuityFailure::WrongPathFrame:
      return "wrong_path_frame";
    case ContinuityFailure::InvalidPathStamp:
      return "invalid_path_stamp";
    case ContinuityFailure::MalformedPath:
      return "malformed_path";
    case ContinuityFailure::MissingOdometry:
      return "missing_odometry";
    case ContinuityFailure::StaleOdometry:
      return "stale_odometry";
    case ContinuityFailure::WrongOdometryFrame:
      return "wrong_odometry_frame";
    case ContinuityFailure::InvalidOdometryStamp:
      return "invalid_odometry_stamp";
    case ContinuityFailure::MalformedOdometry:
      return "malformed_odometry";
    case ContinuityFailure::InsufficientCommonLength:
      return "insufficient_common_length";
    case ContinuityFailure::ExcessiveStartSeparation:
      return "excessive_start_separation";
    case ContinuityFailure::ExcessiveHeadingDifference:
      return "excessive_heading_difference";
  }
  return "malformed_path";
}

}
