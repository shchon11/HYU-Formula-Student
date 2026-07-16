#pragma once

#include "ksae_local/types.hpp"

namespace ksae_local
{

double squaredDistance(const Point2D & lhs, const Point2D & rhs) noexcept;
double distance(const Point2D & lhs, const Point2D & rhs) noexcept;
double normalizeAngle(double angle_rad) noexcept;

/// Transform a point from a source frame into a target frame.
///
/// `source_pose_in_target` is the pose of the source-frame origin expressed in the
/// target frame. Its yaw is the counter-clockwise rotation from source axes to target
/// axes. The returned point is therefore `R(yaw) * point_source + translation`.
Point2D transformPoint(
  const Point2D & point_source, const Pose2D & source_pose_in_target) noexcept;

/// Inverse of transformPoint: transform a target-frame point into the source frame.
Point2D inverseTransformPoint(
  const Point2D & point_target, const Pose2D & source_pose_in_target) noexcept;

/// Rotate a vector counter-clockwise. No translation is applied.
Point2D rotateVector(const Point2D & vector, double angle_rad) noexcept;

double dot(const Point2D & lhs, const Point2D & rhs) noexcept;
double cross(const Point2D & lhs, const Point2D & rhs) noexcept;
double norm(const Point2D & vector) noexcept;

/// Return a unit vector, or {0, 0} for a non-finite or near-zero input.
Point2D normalized(const Point2D & vector, double epsilon = 1.0e-12) noexcept;

bool isFinite(double value) noexcept;
bool isFinite(const Point2D & point) noexcept;
bool isFinite(const Pose2D & pose) noexcept;
bool isFinite(const Covariance2D & covariance) noexcept;

/// Clamp value to the inclusive interval. Reversed bounds are accepted and swapped.
/// NaN inputs remain NaN so invalid data is not silently converted into a valid value.
double clamp(double value, double lower, double upper) noexcept;

}  // namespace ksae_local
