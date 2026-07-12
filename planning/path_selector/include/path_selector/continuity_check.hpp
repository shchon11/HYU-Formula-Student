#pragma once

#include <cstddef>
#include <optional>

#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace path_selector
{

enum class ContinuityFailure
{
  None,
  NotImplemented,
  MissingPath,
  MissingValidity,
  ValidityFalse,
  StaleValidity,
  StalePath,
  WrongPathFrame,
  InvalidPathStamp,
  MalformedPath,
  MissingOdometry,
  StaleOdometry,
  WrongOdometryFrame,
  InvalidOdometryStamp,
  MalformedOdometry,
  InsufficientCommonLength,
  ExcessiveStartSeparation,
  ExcessiveHeadingDifference,
};

struct CandidateInput
{
  std::optional<eufs_msgs::msg::WaypointArrayStamped> path;
  double path_receive_time_sec{0.0};
  bool validity_received{false};
  bool validity{false};
  double validity_receive_time_sec{0.0};
};

struct OdometryInput
{
  std::optional<nav_msgs::msg::Odometry> odometry;
  double receive_time_sec{0.0};
};

struct CandidateValidation
{
  bool ready{false};
  ContinuityFailure failure{ContinuityFailure::NotImplemented};
};

struct TrimResult
{
  std::optional<eufs_msgs::msg::WaypointArrayStamped> path;
  std::size_t original_start_index{0U};
  ContinuityFailure failure{ContinuityFailure::NotImplemented};

  bool success() const;
};

struct ContinuityInputs
{
  CandidateInput local;
  CandidateInput global;
  OdometryInput odometry;
  double receive_now_sec{0.0};
  double stamp_now_sec{0.0};
};

struct ContinuityResult
{
  bool ready{false};
  ContinuityFailure failure{ContinuityFailure::NotImplemented};
  double local_forward_length_m{0.0};
  double global_forward_length_m{0.0};
  double common_forward_length_m{0.0};
  double start_separation_m{0.0};
  double heading_difference_rad{0.0};
};

struct ContinuityThresholds
{
  double timeout_sec{0.5};
  double minimum_common_length_m{5.0};
  double maximum_start_separation_m{1.5};
  double maximum_heading_difference_rad{0.52};
};

class ContinuityCheck
{
public:
  explicit ContinuityCheck(ContinuityThresholds thresholds = {});

  CandidateValidation validateCandidate(
    const CandidateInput & candidate,
    double receive_now_sec,
    double stamp_now_sec) const;
  CandidateValidation validateOdometry(
    const OdometryInput & odometry,
    double receive_now_sec,
    double stamp_now_sec) const;
  TrimResult trimAtEgoNearestPoint(
    const eufs_msgs::msg::WaypointArrayStamped & path,
    const nav_msgs::msg::Odometry & odometry) const;
  ContinuityResult evaluate(const ContinuityInputs & inputs) const;
  nav_msgs::msg::Path toPath(
    const eufs_msgs::msg::WaypointArrayStamped & waypoints) const;

  const ContinuityThresholds & thresholds() const;

private:
  ContinuityThresholds thresholds_;
};

const char * toString(ContinuityFailure failure);

}
