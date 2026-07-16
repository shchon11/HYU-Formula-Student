#include "pure_pursuit_controller/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace pure_pursuit_controller
{
namespace
{

bool isFiniteGeometry(const PathPoint & point)
{
  return std::isfinite(point.x_m) && std::isfinite(point.y_m);
}

bool isFinitePose(const EgoState & ego)
{
  return std::isfinite(ego.x_m) && std::isfinite(ego.y_m) && std::isfinite(ego.yaw_rad);
}

bool isFresh(double age_sec, double timeout_sec)
{
  return std::isfinite(age_sec) && age_sec >= 0.0 && age_sec <= timeout_sec;
}

bool isValidMapConfig(const ControllerConfig & config)
{
  return std::isfinite(config.map_lookahead_slope_s) && config.map_lookahead_slope_s >= 0.0 &&
         std::isfinite(config.map_lookahead_intercept_m) &&
         config.map_lookahead_intercept_m >= 0.0 &&
         std::isfinite(config.map_lookahead_min_m) && config.map_lookahead_min_m > 0.0 &&
         std::isfinite(config.map_lookahead_max_m) &&
         config.map_lookahead_max_m >= config.map_lookahead_min_m;
}

bool isValidConfig(const ControllerConfig & config)
{
  const bool base_valid =
         std::isfinite(config.wheelbase_m) && config.wheelbase_m > 0.0 &&
         std::isfinite(config.lookahead_m) && config.lookahead_m > 0.0 &&
         std::isfinite(config.max_steering_rad) && config.max_steering_rad > 0.0 &&
         std::isfinite(config.command_rate_hz) && config.command_rate_hz > 0.0 &&
         std::isfinite(config.input_timeout_sec) && config.input_timeout_sec >= 0.0 &&
         std::isfinite(config.max_speed_mps) && config.max_speed_mps >= 0.0 &&
         std::isfinite(config.longitudinal_kp) && config.longitudinal_kp >= 0.0 &&
         std::isfinite(config.min_acceleration_mps2) &&
         std::isfinite(config.max_acceleration_mps2) &&
         config.min_acceleration_mps2 <= config.max_acceleration_mps2 &&
         std::isfinite(config.brake_acceleration_mps2) &&
         config.brake_acceleration_mps2 < 0.0;
  if (!base_valid) {
    return false;
  }
  // MAP steering needs a sane adaptive-lookahead band on top of the base config.
  return config.steering_mode != SteeringMode::MAP || isValidMapConfig(config);
}

}

double plannedSpeed(const PathPoint & point)
{
  if (std::isfinite(point.vx_mps) && point.vx_mps > 0.0) {
    return point.vx_mps;
  }
  if (std::isfinite(point.speed_mps) && point.speed_mps > 0.0) {
    return point.speed_mps;
  }
  return 0.0;
}

double mapLookaheadDistance(const ControllerConfig & config, double planned_speed_mps)
{
  const double raw =
    config.map_lookahead_intercept_m + config.map_lookahead_slope_s * planned_speed_mps;
  // max() guards a mis-ordered band (min > max) so std::clamp stays defined even
  // if this public helper is called with a config computeControl would reject.
  const double lower = config.map_lookahead_min_m;
  const double upper = std::max(lower, config.map_lookahead_max_m);
  return std::clamp(raw, lower, upper);
}

DriveCommand brakeCommand(const ControllerConfig & config)
{
  // Fall back to the historical -5.0 m/s^2 if the configured value is not a
  // real deceleration, so an invalid config still brings the car to a stop.
  const double decel =
    (std::isfinite(config.brake_acceleration_mps2) && config.brake_acceleration_mps2 < 0.0) ?
    config.brake_acceleration_mps2 : -5.0;
  return DriveCommand{0.0, decel, 0.0};
}

std::chrono::nanoseconds commandPeriod(const ControllerConfig & config)
{
  if (!std::isfinite(config.command_rate_hz) || config.command_rate_hz <= 0.0) {
    throw std::invalid_argument("command_rate_hz must be finite and positive");
  }
  return std::chrono::nanoseconds{
    static_cast<std::chrono::nanoseconds::rep>(std::llround(1.0e9 / config.command_rate_hz))};
}

std::optional<double> yawFromQuaternion(double x, double y, double z, double w)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w)) {
    return std::nullopt;
  }
  const double norm_squared = x * x + y * y + z * z + w * w;
  if (!std::isfinite(norm_squared) || norm_squared <= 1.0e-12) {
    return std::nullopt;
  }
  const double sin_yaw = 2.0 * (w * z + x * y) / norm_squared;
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z) / norm_squared;
  const double yaw = std::atan2(sin_yaw, cos_yaw);
  if (!std::isfinite(yaw)) {
    return std::nullopt;
  }
  return yaw;
}

bool hasExpectedOdometryFrameIds(
  const std::string_view header_frame_id, const std::string_view child_frame_id)
{
  return header_frame_id == "map" && child_frame_id == "base_footprint";
}

std::optional<std::size_t> findNearestWaypoint(
  const std::vector<PathPoint> & path, const EgoState & ego)
{
  if (path.empty() || !isFinitePose(ego)) {
    return std::nullopt;
  }

  std::optional<std::size_t> nearest_index;
  double nearest_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < path.size(); ++index) {
    if (!isFiniteGeometry(path[index])) {
      return std::nullopt;
    }
    const double dx = path[index].x_m - ego.x_m;
    const double dy = path[index].y_m - ego.y_m;
    const double distance_squared = dx * dx + dy * dy;
    if (!std::isfinite(distance_squared)) {
      return std::nullopt;
    }
    if (distance_squared < nearest_distance_squared) {
      nearest_distance_squared = distance_squared;
      nearest_index = index;
    }
  }
  return nearest_index;
}

std::optional<TargetPoint> selectTarget(
  const std::vector<PathPoint> & path, const EgoState & ego, double lookahead_m)
{
  if (!std::isfinite(lookahead_m) || lookahead_m <= 0.0) {
    return std::nullopt;
  }
  const auto nearest_index = findNearestWaypoint(path, ego);
  if (!nearest_index.has_value()) {
    return std::nullopt;
  }

  std::optional<std::size_t> target_index;
  double cumulative_distance = 0.0;
  for (std::size_t index = nearest_index.value() + 1U; index < path.size(); ++index) {
    const double dx = path[index].x_m - path[index - 1U].x_m;
    const double dy = path[index].y_m - path[index - 1U].y_m;
    cumulative_distance += std::hypot(dx, dy);
    if (!std::isfinite(cumulative_distance)) {
      return std::nullopt;
    }
    if (cumulative_distance >= lookahead_m) {
      target_index = index;
      break;
    }
  }
  if (!target_index.has_value()) {
    target_index = path.size() - 1U;
  }

  const PathPoint & point = path[target_index.value()];
  const double dx = point.x_m - ego.x_m;
  const double dy = point.y_m - ego.y_m;
  const double cosine = std::cos(ego.yaw_rad);
  const double sine = std::sin(ego.yaw_rad);
  const double x_body = cosine * dx + sine * dy;
  const double y_body = -sine * dx + cosine * dy;
  if (!std::isfinite(x_body) || !std::isfinite(y_body) || x_body <= 0.0) {
    return std::nullopt;
  }
  return TargetPoint{point, target_index.value(), x_body, y_body};
}

ControlDecision computeControl(
  const ControllerInput & input, const ControllerConfig & config, const SteeringLookup * lut)
{
  if (!isValidConfig(config) || !input.path_received || !input.validity_received ||
    !input.stop_received || !input.odom_received || !input.path_frame_valid ||
    !input.odom_frame_valid || !input.selected_path_valid || input.stop_requested ||
    !isFresh(input.path_age_sec, config.input_timeout_sec) ||
    !isFresh(input.validity_age_sec, config.input_timeout_sec) ||
    !isFresh(input.stop_age_sec, config.input_timeout_sec) ||
    !isFresh(input.odom_age_sec, config.input_timeout_sec) || !input.ego.has_value() ||
    !isFinitePose(input.ego.value()))
  {
    return ControlDecision{brakeCommand(config), std::nullopt};
  }

  const EgoState & ego = input.ego.value();
  const bool map_mode = config.steering_mode == SteeringMode::MAP;
  // MAP steering is only as safe as its lookup table; fail to braking without one.
  if (map_mode && (lut == nullptr || !lut->valid())) {
    return ControlDecision{brakeCommand(config), std::nullopt};
  }

  // GEOMETRIC uses the fixed lookahead; MAP sizes it from the nearest waypoint's
  // planned speed (L_d = clamp(q + m*v, min, max)).
  double lookahead = config.lookahead_m;
  double planned_speed = 0.0;
  if (map_mode) {
    const auto nearest = findNearestWaypoint(input.path, ego);
    if (!nearest.has_value()) {
      return ControlDecision{brakeCommand(config), std::nullopt};
    }
    planned_speed = plannedSpeed(input.path[nearest.value()]);
    lookahead = mapLookaheadDistance(config, planned_speed);
  }

  const auto target = selectTarget(input.path, ego, lookahead);
  if (!target.has_value()) {
    return ControlDecision{brakeCommand(config), std::nullopt};
  }
  const double lookahead_squared =
    target->x_body_m * target->x_body_m + target->y_body_m * target->y_body_m;
  if (!std::isfinite(lookahead_squared) || lookahead_squared <= 0.0) {
    return ControlDecision{brakeCommand(config), std::nullopt};
  }

  // Longitudinal control is shared by both steering modes.
  const double target_speed =
    std::clamp(plannedSpeed(target->point), 0.0, config.max_speed_mps);
  const double current_speed = std::isfinite(ego.longitudinal_speed_mps) ?
    std::max(0.0, ego.longitudinal_speed_mps) : 0.0;
  const double acceleration = std::clamp(
    config.longitudinal_kp * (target_speed - current_speed),
    config.min_acceleration_mps2, config.max_acceleration_mps2);

  double steering = 0.0;
  if (map_mode) {
    // L1 guidance: the lateral acceleration needed to reach the lookahead point,
    // then the model+tyre table converts it to a steering angle. sin(eta) is the
    // lateral (left-positive) offset of the target over the chord distance L1.
    const double reference_speed =
      config.map_speed_source == MapSpeedSource::MEASURED ? current_speed : planned_speed;
    const double chord = std::sqrt(lookahead_squared);
    const double sin_eta = target->y_body_m / chord;
    const double lateral_accel = 2.0 * reference_speed * reference_speed * sin_eta / lookahead;
    const auto steer = lut->lookup(lateral_accel, reference_speed);
    if (!steer.has_value()) {
      return ControlDecision{brakeCommand(config), std::nullopt};
    }
    steering = std::clamp(steer.value(), -config.max_steering_rad, config.max_steering_rad);
  } else {
    steering = std::clamp(
      std::atan2(2.0 * config.wheelbase_m * target->y_body_m, lookahead_squared),
      -config.max_steering_rad, config.max_steering_rad);
  }
  return ControlDecision{DriveCommand{target_speed, acceleration, steering}, target};
}

DriveCommand computeCommand(
  const ControllerInput & input, const ControllerConfig & config, const SteeringLookup * lut)
{
  return computeControl(input, config, lut).command;
}

}
