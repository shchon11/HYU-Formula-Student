#include "ksae_local/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ksae_local
{
namespace
{
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;

double infinity() noexcept
{
  return std::numeric_limits<double>::infinity();
}
}  // namespace

double squaredDistance(const Point2D & lhs, const Point2D & rhs) noexcept
{
  if (!isFinite(lhs) || !isFinite(rhs)) {
    return infinity();
  }

  const long double dx = static_cast<long double>(lhs.x) - static_cast<long double>(rhs.x);
  const long double dy = static_cast<long double>(lhs.y) - static_cast<long double>(rhs.y);
  const long double squared = dx * dx + dy * dy;
  if (squared > static_cast<long double>(std::numeric_limits<double>::max())) {
    return infinity();
  }
  return static_cast<double>(squared);
}

double distance(const Point2D & lhs, const Point2D & rhs) noexcept
{
  if (!isFinite(lhs) || !isFinite(rhs)) {
    return infinity();
  }
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

double normalizeAngle(const double angle_rad) noexcept
{
  if (!isFinite(angle_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double wrapped = std::fmod(angle_rad, kTwoPi);
  if (wrapped >= kPi) {
    wrapped -= kTwoPi;
  } else if (wrapped < -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

Point2D transformPoint(
  const Point2D & point_source, const Pose2D & source_pose_in_target) noexcept
{
  const Point2D rotated = rotateVector(point_source, source_pose_in_target.yaw);
  return {rotated.x + source_pose_in_target.x, rotated.y + source_pose_in_target.y};
}

Point2D inverseTransformPoint(
  const Point2D & point_target, const Pose2D & source_pose_in_target) noexcept
{
  const Point2D translated{
    point_target.x - source_pose_in_target.x,
    point_target.y - source_pose_in_target.y};
  return rotateVector(translated, -source_pose_in_target.yaw);
}

Point2D rotateVector(const Point2D & vector, const double angle_rad) noexcept
{
  if (!isFinite(vector) || !isFinite(angle_rad)) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {nan, nan};
  }

  const double cosine = std::cos(angle_rad);
  const double sine = std::sin(angle_rad);
  return {
    cosine * vector.x - sine * vector.y,
    sine * vector.x + cosine * vector.y};
}

double dot(const Point2D & lhs, const Point2D & rhs) noexcept
{
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

double cross(const Point2D & lhs, const Point2D & rhs) noexcept
{
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

double norm(const Point2D & vector) noexcept
{
  if (!isFinite(vector)) {
    return infinity();
  }
  return std::hypot(vector.x, vector.y);
}

Point2D normalized(const Point2D & vector, const double epsilon) noexcept
{
  if (!isFinite(vector) || !isFinite(epsilon)) {
    return {};
  }

  const double length = norm(vector);
  if (length <= std::max(0.0, epsilon)) {
    return {};
  }
  return {vector.x / length, vector.y / length};
}

bool isFinite(const double value) noexcept
{
  return std::isfinite(value);
}

bool isFinite(const Point2D & point) noexcept
{
  return isFinite(point.x) && isFinite(point.y);
}

bool isFinite(const Pose2D & pose) noexcept
{
  return isFinite(pose.x) && isFinite(pose.y) && isFinite(pose.yaw);
}

bool isFinite(const Covariance2D & covariance) noexcept
{
  return isFinite(covariance.xx) && isFinite(covariance.xy) &&
         isFinite(covariance.yx) && isFinite(covariance.yy);
}

double clamp(const double value, double lower, double upper) noexcept
{
  if (std::isnan(value) || std::isnan(lower) || std::isnan(upper)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (lower > upper) {
    std::swap(lower, upper);
  }
  return std::max(lower, std::min(value, upper));
}

}  // namespace ksae_local
