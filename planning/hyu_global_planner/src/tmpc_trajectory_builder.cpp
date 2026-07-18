#include "hyu_global_planner/tmpc_trajectory_builder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace hyu_global_planner::tmpc
{
namespace
{

using TumTrajectory = hyu_tmpc_msgs::msg::TumTrajectory;

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kGeometryEpsilon = 1.0e-9;
constexpr double kValidationEpsilon = 1.0e-9;
constexpr double kFormulaEmergencyFinalSpeedLimitMps = 0.5;
constexpr double kFormulaAccelerationUtilizationLimit = 1.025;

void setError(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}

std::string trim(const std::string & input)
{
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1U);
}

bool parseFiniteDouble(const std::string & token, double * value)
{
  if (value == nullptr) {
    return false;
  }
  const std::string stripped = trim(token);
  if (stripped.empty()) {
    return false;
  }
  try {
    std::size_t consumed = 0U;
    const double parsed = std::stod(stripped, &consumed);
    if (consumed != stripped.size() || !std::isfinite(parsed)) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool validateGgvColumns(
  const std::vector<double> & speed,
  const std::vector<double> & ax,
  const std::vector<double> & ay,
  std::string * error)
{
  if (speed.empty()) {
    setError(error, "GGV table is empty.");
    return false;
  }
  if (speed.size() != ax.size() || speed.size() != ay.size()) {
    setError(error, "GGV speed, ax and ay arrays must have identical lengths.");
    return false;
  }
  for (std::size_t i = 0U; i < speed.size(); ++i) {
    if (!std::isfinite(speed[i]) || speed[i] < 0.0) {
      setError(error, "GGV speed values must be finite and non-negative.");
      return false;
    }
    if (!std::isfinite(ax[i]) || !(ax[i] > 0.0) ||
      !std::isfinite(ay[i]) || !(ay[i] > 0.0))
    {
      setError(error, "GGV ax and ay limits must be finite and greater than zero.");
      return false;
    }
    if (i > 0U && !(speed[i] > speed[i - 1U])) {
      setError(error, "GGV speed axis must be strictly increasing.");
      return false;
    }
  }
  return true;
}

double wrapTrackS(double s_m, double track_length_m)
{
  double wrapped = std::fmod(s_m, track_length_m);
  if (wrapped < 0.0) {
    wrapped += track_length_m;
  }
  // Avoid returning track_length due to floating-point roundoff.
  return wrapped >= track_length_m ? 0.0 : wrapped;
}

struct InterpolatedPoint
{
  double s_glob_m{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double psi_rad{0.0};
  double kappa_radpm{0.0};
  double v_mps{0.0};
  double d_left_m{0.0};
  double d_right_m{0.0};
};

bool validatePathSnapshot(
  const wpnt_publisher::PathSnapshot & path,
  std::string * error)
{
  const std::size_t count = path.s.size();
  if (count < 2U) {
    setError(error, "Global path needs at least two unique waypoints.");
    return false;
  }
  if (path.xs.size() != count || path.ys.size() != count || path.waypoints.size() != count) {
    setError(error, "Global path snapshot arrays have inconsistent lengths.");
    return false;
  }
  if (!std::isfinite(path.track_length) || !(path.track_length > 0.0)) {
    setError(error, "Closed-loop track length must be finite and greater than zero.");
    return false;
  }

  for (std::size_t i = 0U; i < count; ++i) {
    const auto & waypoint = path.waypoints[i];
    if (!std::isfinite(path.s[i]) || !std::isfinite(path.xs[i]) ||
      !std::isfinite(path.ys[i]) || !std::isfinite(waypoint.psi_rad) ||
      !std::isfinite(waypoint.kappa_radpm) || !std::isfinite(waypoint.vx_mps) ||
      waypoint.vx_mps < 0.0)
    {
      setError(error, "Global path contains a non-finite or negative trajectory value.");
      return false;
    }
    if (path.s[i] < 0.0 || !(path.s[i] < path.track_length)) {
      setError(error, "Every unique global waypoint s must lie in [0, track_length).");
      return false;
    }
    if (i > 0U && !(path.s[i] > path.s[i - 1U])) {
      setError(error, "Global waypoint s values must be strictly increasing.");
      return false;
    }
  }

  const double seam_distance =
    std::hypot(path.xs.front() - path.xs.back(), path.ys.front() - path.ys.back());
  if (!(seam_distance > kGeometryEpsilon)) {
    setError(error, "Global path has duplicate first/last geometry after snapshot normalization.");
    return false;
  }
  return true;
}

bool interpolatePeriodic(
  const wpnt_publisher::PathSnapshot & path,
  double query_s_m,
  HeadingConvention heading_convention,
  InterpolatedPoint * output,
  std::string * error)
{
  if (output == nullptr) {
    setError(error, "Internal interpolation output is null.");
    return false;
  }

  const double wrapped = wrapTrackS(query_s_m, path.track_length);
  const auto upper = std::upper_bound(path.s.begin(), path.s.end(), wrapped);

  std::size_t lower_index = 0U;
  std::size_t upper_index = 0U;
  double lower_s = 0.0;
  double upper_s = 0.0;
  double adjusted_query = wrapped;

  if (upper == path.s.begin() || upper == path.s.end()) {
    lower_index = path.s.size() - 1U;
    upper_index = 0U;
    lower_s = path.s.back();
    upper_s = path.s.front() + path.track_length;
    if (adjusted_query < path.s.front()) {
      adjusted_query += path.track_length;
    }
  } else {
    upper_index = static_cast<std::size_t>(upper - path.s.begin());
    lower_index = upper_index - 1U;
    lower_s = path.s[lower_index];
    upper_s = path.s[upper_index];
  }

  const double segment_length = upper_s - lower_s;
  if (!(segment_length > kGeometryEpsilon) || !std::isfinite(segment_length)) {
    setError(error, "Global path contains a zero or invalid interpolation interval.");
    return false;
  }
  const double alpha = std::clamp((adjusted_query - lower_s) / segment_length, 0.0, 1.0);
  const auto lerp = [alpha](double lower, double upper_value) {
      return lower + alpha * (upper_value - lower);
    };

  const auto & lower_waypoint = path.waypoints[lower_index];
  const auto & upper_waypoint = path.waypoints[upper_index];
  const double heading_delta =
    normalizeAngle(upper_waypoint.psi_rad - lower_waypoint.psi_rad);

  output->s_glob_m = wrapped;
  output->x_m = lerp(path.xs[lower_index], path.xs[upper_index]);
  output->y_m = lerp(path.ys[lower_index], path.ys[upper_index]);
  output->psi_rad = convertHeading(
    lower_waypoint.psi_rad + alpha * heading_delta, heading_convention);
  output->kappa_radpm = lerp(
    lower_waypoint.kappa_radpm, upper_waypoint.kappa_radpm);
  output->v_mps = lerp(lower_waypoint.vx_mps, upper_waypoint.vx_mps);
  // Boundary distances only interpolate between two KNOWN samples; a segment
  // touching an unknown (0) endpoint stays unknown rather than lerping toward
  // a fictitious zero-width track.
  const auto lerp_distance = [&lerp](double lower, double upper_value) {
      return lower > 0.0 && upper_value > 0.0 ? lerp(lower, upper_value) : 0.0;
    };
  output->d_left_m = lerp_distance(lower_waypoint.d_left_m, upper_waypoint.d_left_m);
  output->d_right_m = lerp_distance(lower_waypoint.d_right_m, upper_waypoint.d_right_m);

  if (!std::isfinite(output->x_m) || !std::isfinite(output->y_m) ||
    !std::isfinite(output->psi_rad) || !std::isfinite(output->kappa_radpm) ||
    !std::isfinite(output->v_mps) || output->v_mps < 0.0)
  {
    setError(error, "Periodic interpolation produced an invalid trajectory point.");
    return false;
  }
  return true;
}

void recalculateAcceleration(TumTrajectory * trajectory)
{
  for (std::size_t i = 0U; i + 1U < kTrajectoryPointCount; ++i) {
    const double ds = trajectory->s_loc_m[i + 1U] - trajectory->s_loc_m[i];
    trajectory->ax_mps2[i] =
      (trajectory->v_mps[i + 1U] * trajectory->v_mps[i + 1U] -
      trajectory->v_mps[i] * trajectory->v_mps[i]) / (2.0 * ds);
  }
  trajectory->ax_mps2.back() = trajectory->ax_mps2[kTrajectoryPointCount - 2U];
}

void fillLimits(
  const GgvTable & table, TumTrajectory * trajectory, bool * endpoint_clamped)
{
  for (std::size_t i = 0U; i < kTrajectoryPointCount; ++i) {
    bool sample_clamped = false;
    const GgvLimits limits = table.limitsAt(trajectory->v_mps[i], &sample_clamped);
    if (endpoint_clamped != nullptr) {
      *endpoint_clamped = *endpoint_clamped || sample_clamped;
    }
    trajectory->ax_lim_mps2[i] = limits.ax_mps2;
    trajectory->ay_lim_mps2[i] = limits.ay_mps2;
  }
}

double tubeFromBoundaryDistance(
  double distance_m, double fallback_tube_m, const BuilderConfig & config)
{
  if (!std::isfinite(distance_m) || !(distance_m > 0.0)) {
    return fallback_tube_m;
  }
  return std::clamp(
    distance_m - config.tube_margin_m,
    fallback_tube_m, std::max(fallback_tube_m, config.tube_max_m));
}

bool buildPerformance(
  const BuildInput & input,
  const BuilderConfig & config,
  const GgvTable & ggv_table,
  double horizon_m,
  TumTrajectory * trajectory,
  bool * endpoint_clamped,
  std::string * error)
{
  const auto & path = *input.path;
  const double span_m = config.start_behind_m + horizon_m;
  if (!(span_m < path.track_length - kGeometryEpsilon)) {
    setError(error, "Requested TMPC span is one full lap or longer.");
    return false;
  }

  TumTrajectory output;
  output.lap_cnt = input.lap_cnt;
  output.traj_cnt = input.traj_cnt;

  const double start_s = input.current_s_m - config.start_behind_m;
  for (std::size_t i = 0U; i < kTrajectoryPointCount; ++i) {
    const double ratio = static_cast<double>(i) /
      static_cast<double>(kTrajectoryPointCount - 1U);
    const double query_s = start_s + ratio * span_m;
    InterpolatedPoint point;
    if (!interpolatePeriodic(path, query_s, config.heading_convention, &point, error)) {
      return false;
    }

    output.s_glob_m[i] = point.s_glob_m;
    output.x_m[i] = point.x_m;
    output.y_m[i] = point.y_m;
    output.psi_rad[i] = point.psi_rad;
    output.kappa_radpm[i] = point.kappa_radpm;
    // Capped BEFORE recalculateAcceleration so the ax feed-forward stays
    // consistent with the capped speed profile.
    output.v_mps[i] = config.performance_speed_cap_mps > 0.0 ?
      std::min(point.v_mps, config.performance_speed_cap_mps) : point.v_mps;
    output.banking_rad[i] = config.banking_rad;
    output.tube_l_m[i] = tubeFromBoundaryDistance(point.d_left_m, config.tube_l_m, config);
    output.tube_r_m[i] = tubeFromBoundaryDistance(point.d_right_m, config.tube_r_m, config);

    if (i == 0U) {
      output.s_loc_m[i] = 0.0;
    } else {
      const double distance = std::hypot(
        output.x_m[i] - output.x_m[i - 1U],
        output.y_m[i] - output.y_m[i - 1U]);
      if (!std::isfinite(distance) || !(distance > config.minimum_adjacent_distance_m)) {
        setError(error, "Resampled trajectory geometry is duplicate or too closely spaced.");
        return false;
      }
      output.s_loc_m[i] = output.s_loc_m[i - 1U] + distance;
    }
  }

  recalculateAcceleration(&output);
  fillLimits(ggv_table, &output, endpoint_clamped);
  *trajectory = std::move(output);
  return true;
}

enum class EmergencyBuildStatus
{
  kSuccess,
  kTerminalSpeedNotReached,
  kInvalid,
};

EmergencyBuildStatus buildEmergency(
  const BuildInput & input,
  const BuilderConfig & config,
  const GgvTable & ggv_table,
  const TumTrajectory & performance,
  TumTrajectory * emergency,
  bool * endpoint_clamped,
  std::string * error)
{
  TumTrajectory output = performance;
  output.v_mps[0] = std::max(input.current_speed_mps, performance.v_mps[0]);

  const double drag_factor =
    0.5 * config.drag_coefficient * config.air_density_kgpm3 / config.vehicle_mass_kg;
  for (std::size_t i = 0U; i + 1U < kTrajectoryPointCount; ++i) {
    const double speed = output.v_mps[i];
    const double ds = output.s_loc_m[i + 1U] - output.s_loc_m[i];
    bool sample_clamped = false;
    const GgvLimits limits = ggv_table.limitsAt(speed, &sample_clamped);
    if (endpoint_clamped != nullptr) {
      *endpoint_clamped = *endpoint_clamped || sample_clamped;
    }
    const double lateral_utilization =
      std::abs(speed * speed * output.kappa_radpm[i] / limits.ay_mps2);
    const double remaining_utilization =
      config.emergency_decel_scale - lateral_utilization;
    if (!(remaining_utilization > 0.0)) {
      setError(error, "Emergency trajectory has no longitudinal budget after lateral demand.");
      return EmergencyBuildStatus::kInvalid;
    }

    if (speed <= config.emergency_final_speed_mps) {
      output.v_mps[i + 1U] = speed;
      continue;
    }

    const double drag_acceleration = drag_factor * speed * speed;
    const double full_braking_ax =
      -remaining_utilization * limits.ax_mps2 - drag_acceleration;
    const double next_speed_squared = speed * speed + 2.0 * full_braking_ax * ds;
    const double terminal_speed = std::min(speed, config.emergency_final_speed_mps);
    output.v_mps[i + 1U] = std::max(
      terminal_speed, std::sqrt(std::max(0.0, next_speed_squared)));
  }

  recalculateAcceleration(&output);
  fillLimits(ggv_table, &output, endpoint_clamped);
  *emergency = std::move(output);
  if (emergency->v_mps.back() > config.emergency_final_speed_mps + kValidationEpsilon) {
    setError(error, "Emergency trajectory does not reach the configured terminal speed.");
    return EmergencyBuildStatus::kTerminalSpeedNotReached;
  }
  return EmergencyBuildStatus::kSuccess;
}

template<typename ArrayT>
bool allFinite(const ArrayT & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

}  // namespace

double normalizeAngle(double angle_rad)
{
  if (!std::isfinite(angle_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double normalized = std::remainder(angle_rad, 2.0 * kPi);
  if (normalized <= -kPi) {
    normalized += 2.0 * kPi;
  }
  return normalized;
}

double convertHeading(double heading_rad, HeadingConvention convention)
{
  const double offset = convention == HeadingConvention::kRosEastZero ? 0.5 * kPi : 0.0;
  return normalizeAngle(heading_rad - offset);
}

std::optional<GgvTable> GgvTable::loadCsv(const std::string & path, std::string * error)
{
  std::ifstream stream(path);
  if (!stream.is_open()) {
    setError(error, "Unable to open GGV CSV: " + path);
    return std::nullopt;
  }

  std::vector<double> speed;
  std::vector<double> ax;
  std::vector<double> ay;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(stream, line)) {
    ++line_number;
    line = trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }

    std::vector<std::string> tokens;
    std::stringstream line_stream(line);
    std::string token;
    while (std::getline(line_stream, token, ',')) {
      tokens.push_back(token);
    }
    if (tokens.size() != 3U && tokens.size() != 4U) {
      setError(
        error, "GGV CSV line " + std::to_string(line_number) +
        " must contain three or four columns.");
      return std::nullopt;
    }

    std::array<double, 4U> values{};
    for (std::size_t i = 0U; i < tokens.size(); ++i) {
      if (!parseFiniteDouble(tokens[i], &values[i])) {
        setError(
          error, "GGV CSV line " + std::to_string(line_number) +
          " contains a non-numeric or non-finite value.");
        return std::nullopt;
      }
    }
    speed.push_back(values[0]);
    ax.push_back(tokens.size() == 4U ? std::min(values[1], values[3]) : values[1]);
    ay.push_back(values[2]);
  }
  return fromVectors(speed, ax, ay, error);
}

std::optional<GgvTable> GgvTable::fromVectors(
  const std::vector<double> & speed_mps,
  const std::vector<double> & ax_lim_mps2,
  const std::vector<double> & ay_lim_mps2,
  std::string * error)
{
  if (!validateGgvColumns(speed_mps, ax_lim_mps2, ay_lim_mps2, error)) {
    return std::nullopt;
  }
  GgvTable table;
  table.speed_mps_ = speed_mps;
  table.ax_lim_mps2_ = ax_lim_mps2;
  table.ay_lim_mps2_ = ay_lim_mps2;
  return table;
}

std::optional<GgvTable> GgvTable::fromVectors(
  const std::vector<double> & speed_mps,
  const std::vector<double> & ax_max_mps2,
  const std::vector<double> & ay_lim_mps2,
  const std::vector<double> & ax_decel_max_mps2,
  std::string * error)
{
  if (ax_max_mps2.size() != ax_decel_max_mps2.size()) {
    setError(error, "GGV acceleration and deceleration arrays must have identical lengths.");
    return std::nullopt;
  }
  std::vector<double> symmetric_ax;
  symmetric_ax.reserve(ax_max_mps2.size());
  for (std::size_t i = 0U; i < ax_max_mps2.size(); ++i) {
    if (!std::isfinite(ax_max_mps2[i]) || !(ax_max_mps2[i] > 0.0) ||
      !std::isfinite(ax_decel_max_mps2[i]) || !(ax_decel_max_mps2[i] > 0.0))
    {
      setError(
        error,
        "GGV acceleration and deceleration limits must each be finite and greater than zero.");
      return std::nullopt;
    }
    symmetric_ax.push_back(std::min(ax_max_mps2[i], ax_decel_max_mps2[i]));
  }
  return fromVectors(speed_mps, symmetric_ax, ay_lim_mps2, error);
}

bool GgvTable::valid(std::string * error) const
{
  return validateGgvColumns(speed_mps_, ax_lim_mps2_, ay_lim_mps2_, error);
}

GgvLimits GgvTable::limitsAt(double speed_mps, bool * clamped) const
{
  if (clamped != nullptr) {
    *clamped = false;
  }
  if (speed_mps_.empty() || !std::isfinite(speed_mps)) {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid};
  }
  if (speed_mps <= speed_mps_.front()) {
    if (clamped != nullptr) {
      *clamped = speed_mps < speed_mps_.front();
    }
    return {ax_lim_mps2_.front(), ay_lim_mps2_.front()};
  }
  if (speed_mps >= speed_mps_.back()) {
    if (clamped != nullptr) {
      *clamped = speed_mps > speed_mps_.back();
    }
    return {ax_lim_mps2_.back(), ay_lim_mps2_.back()};
  }

  const auto upper = std::upper_bound(speed_mps_.begin(), speed_mps_.end(), speed_mps);
  const std::size_t upper_index = static_cast<std::size_t>(upper - speed_mps_.begin());
  const std::size_t lower_index = upper_index - 1U;
  const double ratio = (speed_mps - speed_mps_[lower_index]) /
    (speed_mps_[upper_index] - speed_mps_[lower_index]);
  return {
    ax_lim_mps2_[lower_index] + ratio *
    (ax_lim_mps2_[upper_index] - ax_lim_mps2_[lower_index]),
    ay_lim_mps2_[lower_index] + ratio *
    (ay_lim_mps2_[upper_index] - ay_lim_mps2_[lower_index])};
}

TmpcTrajectoryBuilder::TmpcTrajectoryBuilder(BuilderConfig config, GgvTable ggv_table)
: config_(std::move(config)), ggv_table_(std::move(ggv_table))
{
}

bool TmpcTrajectoryBuilder::valid(std::string * error) const
{
  if (!std::isfinite(config_.foresight_time_sec) || config_.foresight_time_sec < 0.0 ||
    !std::isfinite(config_.min_horizon_m) || !(config_.min_horizon_m > 0.0) ||
    !std::isfinite(config_.max_horizon_m) ||
    config_.max_horizon_m < config_.min_horizon_m ||
    !std::isfinite(config_.start_behind_m) || config_.start_behind_m < 0.0)
  {
    setError(error, "TMPC horizon configuration is invalid.");
    return false;
  }
  if (!std::isfinite(config_.banking_rad) ||
    !std::isfinite(config_.tube_l_m) || config_.tube_l_m < 0.0 ||
    !std::isfinite(config_.tube_r_m) || config_.tube_r_m < 0.0 ||
    !std::isfinite(config_.tube_margin_m) || config_.tube_margin_m < 0.0 ||
    !std::isfinite(config_.tube_max_m) || config_.tube_max_m < 0.0)
  {
    setError(error, "TMPC banking and tube parameters must be finite and tubes non-negative.");
    return false;
  }
  if (!std::isfinite(config_.emergency_decel_scale) ||
    !(config_.emergency_decel_scale > 0.0) ||
    config_.emergency_decel_scale > config_.acceleration_utilization_limit ||
    !std::isfinite(config_.emergency_final_speed_mps) ||
    config_.emergency_final_speed_mps < 0.0 ||
    config_.emergency_final_speed_mps > kFormulaEmergencyFinalSpeedLimitMps)
  {
    setError(
      error,
      "TMPC emergency deceleration configuration is invalid; final speed must be <= 0.5 m/s.");
    return false;
  }
  if (!std::isfinite(config_.drag_coefficient) || config_.drag_coefficient < 0.0 ||
    !std::isfinite(config_.air_density_kgpm3) || config_.air_density_kgpm3 < 0.0 ||
    !std::isfinite(config_.vehicle_mass_kg) || !(config_.vehicle_mass_kg > 0.0))
  {
    setError(error, "TMPC vehicle drag/mass configuration is invalid.");
    return false;
  }
  if (!std::isfinite(config_.minimum_adjacent_distance_m) ||
    !(config_.minimum_adjacent_distance_m > 0.0) ||
    !std::isfinite(config_.acceleration_utilization_limit) ||
    !(config_.acceleration_utilization_limit > 0.0) ||
    config_.acceleration_utilization_limit > kFormulaAccelerationUtilizationLimit ||
    !std::isfinite(config_.acceleration_consistency_tolerance_mps2) ||
    config_.acceleration_consistency_tolerance_mps2 < 0.0)
  {
    setError(
      error,
      "TMPC validation tolerances are invalid; acceleration utilization must be <= 1.025.");
    return false;
  }
  return ggv_table_.valid(error);
}

BuildResult TmpcTrajectoryBuilder::build(const BuildInput & input) const
{
  BuildResult result;
  if (!valid(&result.error)) {
    return result;
  }
  if (input.path == nullptr) {
    result.error = "TMPC build input has no global path snapshot.";
    return result;
  }
  if (!std::isfinite(input.current_s_m) || !std::isfinite(input.current_speed_mps) ||
    input.current_speed_mps < 0.0)
  {
    result.error = "Current Frenet s and vehicle speed must be finite and speed non-negative.";
    return result;
  }
  if (input.traj_cnt == 0U) {
    result.error = "TMPC trajectory counter must be greater than zero.";
    return result;
  }
  if (!validatePathSnapshot(*input.path, &result.error)) {
    return result;
  }

  InterpolatedPoint current_path_point;
  if (!interpolatePeriodic(
      *input.path, input.current_s_m, config_.heading_convention,
      &current_path_point, &result.error))
  {
    return result;
  }
  const double reference_speed = std::max(input.current_speed_mps, current_path_point.v_mps);
  const double nominal_horizon = std::clamp(
    config_.foresight_time_sec * reference_speed,
    config_.min_horizon_m, config_.max_horizon_m);

  const auto attempt = [this, &input](
      double horizon, TumTrajectory * performance, TumTrajectory * emergency,
      bool * endpoint_clamped, std::string * error) -> EmergencyBuildStatus
    {
      if (!buildPerformance(
          input, config_, ggv_table_, horizon, performance, endpoint_clamped, error))
      {
        return EmergencyBuildStatus::kInvalid;
      }
      if (!validateTrajectory(*performance, false, error)) {
        return EmergencyBuildStatus::kInvalid;
      }
      const EmergencyBuildStatus emergency_status = buildEmergency(
        input, config_, ggv_table_, *performance, emergency, endpoint_clamped, error);
      if (emergency_status != EmergencyBuildStatus::kSuccess) {
        return emergency_status;
      }
      if (!validateTrajectory(*emergency, true, error)) {
        return EmergencyBuildStatus::kInvalid;
      }
      return EmergencyBuildStatus::kSuccess;
    };

  result.horizon_m = nominal_horizon;
  bool endpoint_clamped = false;
  EmergencyBuildStatus status = attempt(
    nominal_horizon, &result.performance, &result.emergency,
    &endpoint_clamped, &result.error);
  if (status == EmergencyBuildStatus::kTerminalSpeedNotReached &&
    nominal_horizon < config_.max_horizon_m - kGeometryEpsilon)
  {
    result.horizon_m = config_.max_horizon_m;
    result.used_max_horizon = true;
    result.error.clear();
    endpoint_clamped = false;
    status = attempt(
      config_.max_horizon_m, &result.performance, &result.emergency,
      &endpoint_clamped, &result.error);
  }
  if (status != EmergencyBuildStatus::kSuccess) {
    return result;
  }
  result.ggv_endpoint_clamped = endpoint_clamped;

  // A pair is atomic at the application layer: counters must be identical.
  if (result.performance.lap_cnt != result.emergency.lap_cnt ||
    result.performance.traj_cnt != result.emergency.traj_cnt)
  {
    result.error = "Performance and emergency trajectory counters differ.";
    return result;
  }
  result.success = true;
  result.error.clear();
  return result;
}

bool TmpcTrajectoryBuilder::validateTrajectory(
  const TumTrajectory & trajectory,
  bool require_emergency_stop,
  std::string * error) const
{
  if (trajectory.traj_cnt == 0U) {
    setError(error, "Trajectory counter must be greater than zero.");
    return false;
  }
  if (!allFinite(trajectory.s_loc_m) || !allFinite(trajectory.s_glob_m) ||
    !allFinite(trajectory.x_m) || !allFinite(trajectory.y_m) ||
    !allFinite(trajectory.psi_rad) || !allFinite(trajectory.kappa_radpm) ||
    !allFinite(trajectory.v_mps) || !allFinite(trajectory.ax_mps2) ||
    !allFinite(trajectory.banking_rad) || !allFinite(trajectory.ax_lim_mps2) ||
    !allFinite(trajectory.ay_lim_mps2) || !allFinite(trajectory.tube_l_m) ||
    !allFinite(trajectory.tube_r_m))
  {
    setError(error, "Trajectory contains NaN or Inf.");
    return false;
  }
  if (std::abs(trajectory.s_loc_m.front()) > kValidationEpsilon) {
    setError(error, "Trajectory local s must start at zero.");
    return false;
  }

  const double drag_factor =
    0.5 * config_.drag_coefficient * config_.air_density_kgpm3 / config_.vehicle_mass_kg;
  for (std::size_t i = 0U; i < kTrajectoryPointCount; ++i) {
    if (trajectory.v_mps[i] < 0.0 || !(trajectory.ax_lim_mps2[i] > 0.0) ||
      !(trajectory.ay_lim_mps2[i] > 0.0) || trajectory.tube_l_m[i] < 0.0 ||
      trajectory.tube_r_m[i] < 0.0)
    {
      setError(error, "Trajectory speed/tube values or acceleration limits are invalid.");
      return false;
    }
    if (i > 0U &&
      !(trajectory.s_loc_m[i] - trajectory.s_loc_m[i - 1U] >
      config_.minimum_adjacent_distance_m))
    {
      setError(error, "Trajectory local s is not strictly increasing with the minimum spacing.");
      return false;
    }

    const double speed_squared = trajectory.v_mps[i] * trajectory.v_mps[i];
    const double drag_acceleration_mps2 = drag_factor * speed_squared;
    const double lateral_acceleration_mps2 =
      speed_squared * trajectory.kappa_radpm[i];
    const double longitudinal_utilization =
      std::abs((drag_acceleration_mps2 + trajectory.ax_mps2[i]) /
      trajectory.ax_lim_mps2[i]);
    const double lateral_utilization =
      std::abs(lateral_acceleration_mps2 / trajectory.ay_lim_mps2[i]);
    const double utilization = longitudinal_utilization + lateral_utilization;
    if (!std::isfinite(utilization) ||
      utilization > config_.acceleration_utilization_limit + kValidationEpsilon)
    {
      std::ostringstream diagnostic;
      diagnostic << std::setprecision(10)
                 << "Trajectory violates the Formula L1 acceleration-utilization limit: "
                 << "trajectory=" << (require_emergency_stop ? "emergency" : "performance")
                 << " point=" << i
                 << " utilization=" << utilization
                 << " limit=" << config_.acceleration_utilization_limit
                 << " longitudinal_utilization=" << longitudinal_utilization
                 << " lateral_utilization=" << lateral_utilization
                 << " v_mps=" << trajectory.v_mps[i]
                 << " ax_mps2=" << trajectory.ax_mps2[i]
                 << " drag_ax_mps2=" << drag_acceleration_mps2
                 << " lateral_acceleration_mps2=" << lateral_acceleration_mps2
                 << " kappa_radpm=" << trajectory.kappa_radpm[i]
                 << " ax_lim_mps2=" << trajectory.ax_lim_mps2[i]
                 << " ay_lim_mps2=" << trajectory.ay_lim_mps2[i]
                 << " s_loc_m=" << trajectory.s_loc_m[i]
                 << " s_glob_m=" << trajectory.s_glob_m[i];
      setError(error, diagnostic.str());
      return false;
    }
  }

  for (std::size_t i = 0U; i + 1U < kTrajectoryPointCount; ++i) {
    const double ds = trajectory.s_loc_m[i + 1U] - trajectory.s_loc_m[i];
    const double expected_ax =
      (trajectory.v_mps[i + 1U] * trajectory.v_mps[i + 1U] -
      trajectory.v_mps[i] * trajectory.v_mps[i]) / (2.0 * ds);
    if (std::abs(expected_ax - trajectory.ax_mps2[i]) >
      config_.acceleration_consistency_tolerance_mps2)
    {
      setError(error, "Trajectory velocity and longitudinal acceleration are inconsistent.");
      return false;
    }
    if (require_emergency_stop &&
      trajectory.v_mps[i + 1U] > trajectory.v_mps[i] + kValidationEpsilon)
    {
      setError(error, "Emergency trajectory speed must be monotonically non-increasing.");
      return false;
    }
  }
  if (std::abs(
      trajectory.ax_mps2.back() - trajectory.ax_mps2[kTrajectoryPointCount - 2U]) >
    config_.acceleration_consistency_tolerance_mps2)
  {
    setError(error, "Final trajectory acceleration must repeat the final segment value.");
    return false;
  }
  if (require_emergency_stop &&
    trajectory.v_mps.back() > config_.emergency_final_speed_mps + kValidationEpsilon)
  {
    setError(error, "Emergency trajectory terminal speed exceeds its configured limit.");
    return false;
  }
  return true;
}

}  // namespace hyu_global_planner::tmpc
