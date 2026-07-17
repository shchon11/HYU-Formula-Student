#include "hyu_local_planner/input_policy.hpp"

#include <cmath>
#include <stdexcept>

namespace hyu_local_planner
{
namespace
{

bool nonzeroStamp(const HeaderMetadata & header)
{
  return header.stamp_sec >= 0 && (header.stamp_sec != 0 || header.stamp_nanosec != 0U);
}

bool fresh(const HeaderMetadata & header, double maximum_age)
{
  return std::isfinite(header.receive_age_sec) && header.receive_age_sec >= 0.0 &&
         header.receive_age_sec <= maximum_age;
}

double stampSeconds(const HeaderMetadata & header)
{
  return static_cast<double>(header.stamp_sec) +
         static_cast<double>(header.stamp_nanosec) * 1.0e-9;
}

ValidationResult invalid(const std::string & reason)
{
  return ValidationResult{true, false, reason};
}

ValidationResult validateOdom(const OdomMetadata & odom, double maximum_age)
{
  if (odom.header.frame_id != "map" || odom.child_frame_id != "base_footprint") {
    return invalid("odom frame contract failed");
  }
  if (!nonzeroStamp(odom.header)) {
    return invalid("odom stamp is zero or negative");
  }
  if (!fresh(odom.header, maximum_age)) {
    return invalid("odom receive age is stale");
  }
  if (!std::isfinite(odom.position.x) || !std::isfinite(odom.position.y) ||
    !std::isfinite(odom.yaw))
  {
    return invalid("odom pose is non-finite");
  }
  return ValidationResult{true, true, {}};
}

}

SourceMode parseSourceMode(const std::string & value)
{
  if (value == "live_cones") {
    return SourceMode::kLiveCones;
  }
  if (value == "slam_map") {
    return SourceMode::kSlamMap;
  }
  throw std::invalid_argument("source_mode must be live_cones or slam_map");
}

bool acceptsSource(SourceMode configured, SourceMode incoming)
{
  return configured == incoming;
}

ValidationResult validateLiveInput(
  const HeaderMetadata & cones, const OdomMetadata & odom,
  double max_stamp_skew_sec, double max_receive_age_sec)
{
  if (!std::isfinite(max_stamp_skew_sec) || max_stamp_skew_sec < 0.0 ||
    !std::isfinite(max_receive_age_sec) || max_receive_age_sec < 0.0)
  {
    return invalid("live input limits are invalid");
  }
  if (cones.frame_id != "base_footprint") {
    return invalid("cone frame must be base_footprint");
  }
  if (!nonzeroStamp(cones)) {
    return invalid("cone stamp is zero or negative");
  }
  if (!fresh(cones, max_receive_age_sec)) {
    return invalid("cone receive age is stale");
  }
  const auto odom_result = validateOdom(odom, max_receive_age_sec);
  if (!odom_result.valid) {
    return odom_result;
  }
  const double skew = std::abs(stampSeconds(cones) - stampSeconds(odom.header));
  if (!std::isfinite(skew) || skew > max_stamp_skew_sec) {
    return invalid("cone and odom stamps exceed maximum skew");
  }
  return ValidationResult{true, true, {}};
}

ValidationResult validateSlamMapInput(
  const HeaderMetadata & map, const OdomMetadata & odom,
  double max_receive_age_sec)
{
  if (!std::isfinite(max_receive_age_sec) || max_receive_age_sec < 0.0) {
    return invalid("slam map input limit is invalid");
  }
  if (map.frame_id != "map") {
    return invalid("slam map frame must be map");
  }
  if (!nonzeroStamp(map)) {
    return invalid("slam map stamp is zero or negative");
  }
  return validateOdom(odom, max_receive_age_sec);
}

}
