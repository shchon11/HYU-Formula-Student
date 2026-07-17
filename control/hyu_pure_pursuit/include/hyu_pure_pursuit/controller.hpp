#ifndef HYU_PURE_PURSUIT__CONTROLLER_HPP_
#define HYU_PURE_PURSUIT__CONTROLLER_HPP_

#include <chrono>
#include <cstddef>
#include <optional>
#include <string_view>
#include <vector>

#include "hyu_pure_pursuit/steering_lookup.hpp"

namespace hyu_pure_pursuit
{

// GEOMETRIC is the kinematic pure-pursuit steering law delta = atan2(2*L*y, d^2).
// MAP (Model- and Acceleration-based Pursuit, ETH-PBL) instead sizes the
// lookahead with speed, computes the lateral acceleration the L1 guidance law
// demands, and reads the steering angle off a single-track + tyre model lookup
// table -- so it accounts for the speed-dependent understeer the kinematic law
// ignores. Longitudinal control is identical in both modes.
enum class SteeringMode
{
  GEOMETRIC,
  MAP
};

// Speed fed into the MAP lateral-acceleration law and the steering lookup.
// PLANNED (the reference behaviour) uses the path's target speed, making the
// steering a pure feed-forward that still commands sensibly near standstill;
// MEASURED uses the odometry speed, which is exact when tracking lags.
enum class MapSpeedSource
{
  PLANNED,
  MEASURED
};

struct ControllerConfig
{
  double wheelbase_m{1.58};
  double lookahead_m{3.5};
  double max_steering_rad{0.52};
  double command_rate_hz{20.0};
  double input_timeout_sec{0.5};
  double max_speed_mps{4.5};
  double longitudinal_kp{1.2};
  double min_acceleration_mps2{-8.0};
  double max_acceleration_mps2{2.5};
  // Deceleration commanded by the hard brake (invalid/stale path, stop request,
  // no reachable target). Separate from min_acceleration_mps2, which only
  // clamps in-path deceleration. Model the vehicle's real braking limit here.
  double brake_acceleration_mps2{-5.0};

  // Lateral steering law. GEOMETRIC keeps the original fixed-lookahead pure
  // pursuit; MAP requires a valid SteeringLookup passed to computeControl.
  SteeringMode steering_mode{SteeringMode::GEOMETRIC};

  // MAP adaptive lookahead: L_d = clamp(intercept + slope * v, min, max).
  double map_lookahead_slope_s{0.3};
  double map_lookahead_intercept_m{1.0};
  double map_lookahead_min_m{1.5};
  double map_lookahead_max_m{8.0};
  MapSpeedSource map_speed_source{MapSpeedSource::PLANNED};
};

struct PathPoint
{
  double x_m{0.0};
  double y_m{0.0};
  double vx_mps{0.0};
  double speed_mps{0.0};
};

struct EgoState
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double longitudinal_speed_mps{0.0};
};

struct TargetPoint
{
  PathPoint point;
  std::size_t original_index{0U};
  double x_body_m{0.0};
  double y_body_m{0.0};
};

struct DriveCommand
{
  double speed_mps{0.0};
  double acceleration_mps2{-5.0};
  double steering_angle_rad{0.0};
};

struct ControlDecision
{
  DriveCommand command;
  // Engaged only when the command tracks the path; empty on every brake fallback.
  std::optional<TargetPoint> target;
};

struct ControllerInput
{
  bool path_received{false};
  bool validity_received{false};
  bool stop_received{false};
  bool odom_received{false};
  bool path_frame_valid{false};
  bool odom_frame_valid{false};
  bool selected_path_valid{false};
  bool stop_requested{false};
  double path_age_sec{0.0};
  double validity_age_sec{0.0};
  double stop_age_sec{0.0};
  double odom_age_sec{0.0};
  std::vector<PathPoint> path;
  std::optional<EgoState> ego;
};

DriveCommand brakeCommand(const ControllerConfig & config = ControllerConfig{});

std::chrono::nanoseconds commandPeriod(const ControllerConfig & config);

std::optional<double> yawFromQuaternion(double x, double y, double z, double w);

bool hasExpectedOdometryFrameIds(
  std::string_view header_frame_id, std::string_view child_frame_id);

std::optional<std::size_t> findNearestWaypoint(
  const std::vector<PathPoint> & path, const EgoState & ego);

std::optional<TargetPoint> selectTarget(
  const std::vector<PathPoint> & path, const EgoState & ego, double lookahead_m);

// Planned longitudinal speed of a path point: vx_mps if it is finite and
// positive, else speed, else 0. Shared by the command speed and the MAP law.
double plannedSpeed(const PathPoint & point);

// MAP adaptive lookahead distance for a planned speed, clamped to [min, max].
double mapLookaheadDistance(const ControllerConfig & config, double planned_speed_mps);

// `lut` is used only in MAP steering mode; GEOMETRIC ignores it. In MAP mode a
// null or invalid lookup fails safe to braking.
ControlDecision computeControl(
  const ControllerInput & input, const ControllerConfig & config = ControllerConfig{},
  const SteeringLookup * lut = nullptr);

DriveCommand computeCommand(
  const ControllerInput & input, const ControllerConfig & config = ControllerConfig{},
  const SteeringLookup * lut = nullptr);

}

#endif
