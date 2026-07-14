#ifndef PURE_PURSUIT_CONTROLLER__CONTROLLER_HPP_
#define PURE_PURSUIT_CONTROLLER__CONTROLLER_HPP_

#include <chrono>
#include <cstddef>
#include <optional>
#include <string_view>
#include <vector>

namespace pure_pursuit_controller
{

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

DriveCommand computeCommand(
  const ControllerInput & input, const ControllerConfig & config = ControllerConfig{});

}

#endif
