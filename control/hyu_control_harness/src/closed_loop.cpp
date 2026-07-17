#include "hyu_control_harness/closed_loop.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>

#include "eufs_models/dynamic_bicycle.hpp"

namespace hyu_control_harness
{

namespace
{

namespace ppc = hyu_pure_pursuit;

// Half of the car's overall width plus a cone radius: closer than this to the
// cone line and the body is clipping cones even though the centre is inside.
constexpr double kCarHalfWidthM = 0.7;
// Centre of the car this far past the cone line is unambiguously off track.
constexpr double kOffTrackSlackM = 0.3;
constexpr double kMovingSpeedMps = 0.5;
constexpr double kStallSpeedMps = 0.05;
constexpr double kStallGraceS = 20.0;
constexpr double kStallTimeoutS = 8.0;
constexpr int kTargetLaps = 10;  // trackdrive mission length

struct TruthSample
{
  double t{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
};

struct MeasuredState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
};

hyu_local_planner::Point2 toEgoFrame(const MeasuredState & ego, const Vec2 & p)
{
  const double cosine = std::cos(ego.yaw);
  const double sine = std::sin(ego.yaw);
  const double dx = p.x - ego.x;
  const double dy = p.y - ego.y;
  return {cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

ppc::PathPoint toMapFrame(const MeasuredState & ego, const hyu_local_planner::PathWaypoint & wp)
{
  const double cosine = std::cos(ego.yaw);
  const double sine = std::sin(ego.yaw);
  ppc::PathPoint point;
  point.x_m = ego.x + cosine * wp.x - sine * wp.y;
  point.y_m = ego.y + sine * wp.x + cosine * wp.y;
  point.vx_mps = wp.speed;
  point.speed_mps = wp.speed;
  return point;
}

std::string formatDouble(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out.precision(6);
  out << value;
  return out.str();
}

}  // namespace

ppc::ControllerConfig trackdriveControllerConfig()
{
  ppc::ControllerConfig config;
  config.wheelbase_m = 1.58;
  config.max_steering_rad = 0.52;
  config.command_rate_hz = 20.0;
  config.input_timeout_sec = 0.5;
  config.max_speed_mps = 10.0;
  config.longitudinal_kp = 1.2;
  config.min_acceleration_mps2 = -8.0;
  config.max_acceleration_mps2 = 2.5;
  config.brake_acceleration_mps2 = -5.0;
  config.steering_mode = ppc::SteeringMode::MAP;
  config.map_lookahead_slope_s = 0.55;
  config.map_lookahead_intercept_m = 0.8;
  config.map_lookahead_min_m = 1.5;
  config.map_lookahead_max_m = 5.0;
  config.map_speed_source = ppc::MapSpeedSource::PLANNED;
  return config;
}

ppc::VehicleModel trackdriveLutModel()
{
  ppc::VehicleModel model;
  model.mass_kg = 225.0;
  model.yaw_inertia_kg_m2 = 31.27;
  model.cg_to_front_m = 0.869;
  model.cg_to_rear_m = 0.711;
  model.front_slip_lever_arm_m = 0.711;
  model.cg_height_m = 0.30;
  model.gravity_mps2 = 9.81;
  model.aero_downforce_coeff = 1.9;
  model.tire_model = ppc::TireModel::PACEJKA;
  model.pacejka_mu = 1.0;
  model.pacejka_b_front = 12.56;
  model.pacejka_c_front = 1.38;
  model.pacejka_d_front = 1.60;
  model.pacejka_e_front = -0.58;
  model.pacejka_b_rear = 12.56;
  model.pacejka_c_rear = 1.38;
  model.pacejka_d_rear = 1.60;
  model.pacejka_e_rear = -0.58;
  return model;
}

ppc::LutGrid trackdriveLutGrid()
{
  ppc::LutGrid grid;
  grid.steer_fine_end_rad = 0.1;
  grid.steer_max_rad = 0.52;
  grid.n_steer_fine = 30;
  grid.n_steer_coarse = 30;
  grid.vel_min_mps = 0.5;
  grid.vel_max_mps = 12.0;
  grid.n_vel = 46;
  grid.sim_dt_s = 0.01;
  grid.sim_duration_s = 2.0;
  grid.sim_substeps = 10;
  return grid;
}

hyu_local_planner::PlannerConfig trackdrivePlannerConfig()
{
  hyu_local_planner::PlannerConfig config;
  config.two_sided_speed_mps = 4.5;
  config.fallback_speed_mps = 3.0;
  config.use_orange_cones = true;
  // The node defaults this to true in slam_map mode (hyu_local_planner_node.cpp);
  // the harness feeds the planner a full ground-truth map, i.e. slam_map mode.
  config.allow_partial_boundary = true;
  return config;
}

std::string HarnessResult::toJson() const
{
  std::ostringstream out;
  out << "{";
  out << "\"dnf\":" << (dnf ? "true" : "false");
  out << ",\"dnf_reason\":\"" << dnf_reason << "\"";
  out << ",\"laps\":" << laps;
  out << ",\"lap_times_s\":[";
  for (std::size_t i = 0; i < lap_times_s.size(); ++i) {
    out << (i == 0U ? "" : ",") << formatDouble(lap_times_s[i]);
  }
  out << "]";
  out << ",\"best_lap_s\":" << formatDouble(best_lap_s);
  out << ",\"mean_lap_s\":" << formatDouble(mean_lap_s);
  out << ",\"cte_rmse_m\":" << formatDouble(cte_rmse_m);
  out << ",\"cte_max_m\":" << formatDouble(cte_max_m);
  out << ",\"violation_frac\":" << formatDouble(violation_frac);
  out << ",\"boundary_min_m\":" << formatDouble(boundary_min_m);
  out << ",\"steer_rate_mean_radps\":" << formatDouble(steer_rate_mean_radps);
  out << ",\"steer_rate_max_radps\":" << formatDouble(steer_rate_max_radps);
  out << ",\"steer_sat_frac\":" << formatDouble(steer_sat_frac);
  out << ",\"speed_rmse_mps\":" << formatDouble(speed_rmse_mps);
  out << ",\"planner_invalid_frac\":" << formatDouble(planner_invalid_frac);
  out << ",\"last_planner_reason\":\"" << last_planner_reason << "\"";
  out << ",\"distance_m\":" << formatDouble(distance_m);
  out << ",\"avg_speed_mps\":" << formatDouble(avg_speed_mps);
  out << ",\"sim_time_s\":" << formatDouble(sim_time_s);
  out << "}";
  return out.str();
}

HarnessResult runMapHarness(const HarnessConfig & config, const Track & track)
{
  HarnessResult result;

  const Centerline centerline = Centerline::build(track);
  if (!centerline.valid()) {
    result.dnf = true;
    result.dnf_reason = "bad_track";
    return result;
  }

  std::unique_ptr<eufs::models::DynamicBicycle> plant;
  try {
    plant = std::make_unique<eufs::models::DynamicBicycle>(config.plant_yaml);
  } catch (const std::exception & error) {
    result.dnf = true;
    result.dnf_reason = std::string("plant_yaml: ") + error.what();
    return result;
  }
  const auto & delta_range = plant->getParam().input_ranges.delta;
  const double max_steering_rate =
    (delta_range.max - delta_range.min) / config.steering_lock_time_s;

  const ppc::SteeringLookup lut = ppc::buildSteeringLookup(config.lut_model, config.lut_grid);
  if (config.controller.steering_mode == ppc::SteeringMode::MAP && !lut.valid()) {
    result.dnf = true;
    result.dnf_reason = "invalid_lut";
    return result;
  }

  std::mt19937 rng(config.seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  const bool with_noise = config.odom_noise_xy_m > 0.0 || config.odom_noise_yaw_rad > 0.0 ||
    config.odom_noise_v_mps > 0.0;

  eufs::models::State state{};
  state.x = track.start.x;
  state.y = track.start.y;
  state.yaw = track.start.yaw;
  eufs::models::Input act{};

  struct TimedCommand
  {
    double t{0.0};
    ppc::DriveCommand command;
  };
  std::deque<TimedCommand> command_queue;
  ppc::DriveCommand des;
  des.speed_mps = 0.0;
  des.acceleration_mps2 = 0.0;
  des.steering_angle_rad = 0.0;
  double last_cmd_arrival_s = 0.0;
  bool any_command = false;

  std::deque<TruthSample> truth_history;
  MeasuredState measured{state.x, state.y, state.yaw, 0.0};

  std::ofstream traj;
  if (!config.traj_csv.empty()) {
    traj.open(config.traj_csv);
    traj << "t,x,y,yaw,v,s,cte,half_width,delta_cmd,delta_act,acc_cmd,speed_cmd,path_valid\n";
  }

  const double dt = config.plant_dt_s;
  const double control_period_s = 1.0 / config.controller.command_rate_hz;
  const double odom_period_s = 1.0 / config.odom_rate_hz;
  double next_control_t = 0.0;
  double next_odom_t = 0.0;
  double next_traj_t = 0.0;

  double cte_sq_sum = 0.0;
  std::size_t cte_samples = 0;
  double speed_err_sq_sum = 0.0;
  std::size_t speed_samples = 0;
  double steer_rate_sum = 0.0;
  std::size_t steer_rate_samples = 0;
  std::size_t sat_cycles = 0;
  std::size_t control_cycles = 0;
  std::size_t invalid_cycles = 0;
  std::size_t violation_samples = 0;
  std::size_t moving_samples = 0;
  result.boundary_min_m = std::numeric_limits<double>::infinity();

  double progress_s = centerline.project({state.x, state.y}).s_m;
  double total_progress_m = 0.0;
  double lap_start_t = 0.0;
  int laps_done = 0;
  double stall_since_s = -1.0;
  bool last_path_valid = false;
  double last_speed_cmd = 0.0;

  double t = 0.0;
  while (t < config.sim_time_s) {
    // --- Localization chain: sample the (possibly delayed, noisy) pose ---
    if (t >= next_odom_t) {
      next_odom_t += odom_period_s;
      const double query_t = t - config.odom_latency_s;
      TruthSample sample{t, state.x, state.y, state.yaw, state.v_x};
      for (auto it = truth_history.rbegin(); it != truth_history.rend(); ++it) {
        if (it->t <= query_t) {
          sample = *it;
          break;
        }
      }
      if (!truth_history.empty() && truth_history.front().t > query_t) {
        sample = truth_history.front();
      }
      measured = {sample.x, sample.y, sample.yaw, sample.v};
      if (with_noise) {
        measured.x += config.odom_noise_xy_m * gauss(rng);
        measured.y += config.odom_noise_xy_m * gauss(rng);
        measured.yaw += config.odom_noise_yaw_rad * gauss(rng);
        measured.v += config.odom_noise_v_mps * gauss(rng);
      }
    }

    // --- Planner + controller cycle ---
    if (t >= next_control_t) {
      next_control_t += control_period_s;
      ++control_cycles;

      hyu_local_planner::ConeSet cones;
      for (const Vec2 & c : track.blue) {
        cones.blue.push_back(toEgoFrame(measured, c));
      }
      for (const Vec2 & c : track.yellow) {
        cones.yellow.push_back(toEgoFrame(measured, c));
      }
      for (const Vec2 & c : track.orange) {
        cones.orange.push_back(toEgoFrame(measured, c));
      }
      for (const Vec2 & c : track.big_orange) {
        cones.big_orange.push_back(toEgoFrame(measured, c));
      }

      const auto build = hyu_local_planner::buildLocalPath(cones, config.planner);
      if (!build.valid) {
        ++invalid_cycles;
        result.last_planner_reason = build.reason;
      }

      ppc::ControllerInput input;
      input.path_received = true;
      input.validity_received = true;
      input.stop_received = true;
      input.odom_received = true;
      input.path_frame_valid = true;
      input.odom_frame_valid = true;
      input.selected_path_valid = build.valid;
      input.stop_requested = false;
      input.path.reserve(build.waypoints.size());
      for (const auto & wp : build.waypoints) {
        input.path.push_back(toMapFrame(measured, wp));
      }
      input.ego = ppc::EgoState{measured.x, measured.y, measured.yaw, measured.v};

      const auto decision = ppc::computeControl(input, config.controller, &lut);
      command_queue.push_back({t, decision.command});
      last_cmd_arrival_s = t;
      any_command = true;
      last_path_valid = build.valid && decision.target.has_value();
      last_speed_cmd = decision.command.speed_mps;

      if (std::abs(decision.command.steering_angle_rad) >=
        config.controller.max_steering_rad - 1e-9)
      {
        ++sat_cycles;
      }
      if (decision.target.has_value()) {
        const double err = decision.command.speed_mps - std::hypot(state.v_x, state.v_y);
        speed_err_sq_sum += err * err;
        ++speed_samples;
      }
    }

    // --- Plant-side command chain: control delay, then rate-limited steering ---
    if (!command_queue.empty() && (t - command_queue.front().t) >= config.control_delay_s) {
      des = command_queue.front().command;
      command_queue.pop_front();
    }
    act.acc = (any_command && (t - last_cmd_arrival_s) < 1.0) ? des.acceleration_mps2 : -1.0;
    const double delta_err = des.steering_angle_rad - act.delta;
    const double prev_delta = act.delta;
    act.delta += (delta_err >= 0.0 ? 1.0 : -1.0) *
      std::min(max_steering_rate * dt, std::abs(delta_err));

    plant->updateState(state, act, dt);
    truth_history.push_back({t, state.x, state.y, state.yaw, state.v_x});
    while (!truth_history.empty() && truth_history.front().t < t - 2.0) {
      truth_history.pop_front();
    }

    // --- Metrics ---
    const double v = std::hypot(state.v_x, state.v_y);
    const auto projection = centerline.project({state.x, state.y});
    const double clearance = projection.half_width_m - projection.cte_m;
    result.distance_m += v * dt;

    steer_rate_sum += std::abs(act.delta - prev_delta) / dt;
    result.steer_rate_max_radps =
      std::max(result.steer_rate_max_radps, std::abs(act.delta - prev_delta) / dt);
    ++steer_rate_samples;

    if (v > kMovingSpeedMps) {
      cte_sq_sum += projection.cte_m * projection.cte_m;
      ++cte_samples;
      result.cte_max_m = std::max(result.cte_max_m, projection.cte_m);
      result.boundary_min_m = std::min(result.boundary_min_m, clearance);
      ++moving_samples;
      if (clearance < kCarHalfWidthM) {
        ++violation_samples;
      }
      if (clearance < -kOffTrackSlackM) {
        result.dnf = true;
        result.dnf_reason = "off_track";
      }
    }

    const double ds = centerline.wrappedDelta(progress_s, projection.s_m);
    progress_s = projection.s_m;
    if (v > kMovingSpeedMps) {
      total_progress_m += ds;
    }
    if (total_progress_m >= (laps_done + 1) * centerline.length()) {
      ++laps_done;
      result.lap_times_s.push_back(t - lap_start_t);
      lap_start_t = t;
      if (laps_done >= kTargetLaps) {
        t += dt;
        break;
      }
    }

    if (t > kStallGraceS && v < kStallSpeedMps) {
      if (stall_since_s < 0.0) {
        stall_since_s = t;
      } else if (t - stall_since_s > kStallTimeoutS) {
        result.dnf = true;
        result.dnf_reason = "stalled";
      }
    } else {
      stall_since_s = -1.0;
    }

    if (traj.is_open() && t >= next_traj_t) {
      next_traj_t += 0.01;
      traj << formatDouble(t) << ',' << formatDouble(state.x) << ',' << formatDouble(state.y)
           << ',' << formatDouble(state.yaw) << ',' << formatDouble(v) << ','
           << formatDouble(projection.s_m) << ',' << formatDouble(projection.cte_m) << ','
           << formatDouble(projection.half_width_m) << ','
           << formatDouble(des.steering_angle_rad) << ',' << formatDouble(act.delta) << ','
           << formatDouble(des.acceleration_mps2) << ',' << formatDouble(last_speed_cmd) << ','
           << (last_path_valid ? 1 : 0) << '\n';
    }

    if (result.dnf) {
      t += dt;
      break;
    }
    t += dt;
  }

  result.laps = laps_done;
  result.sim_time_s = t;
  if (!result.lap_times_s.empty()) {
    result.best_lap_s = *std::min_element(result.lap_times_s.begin(), result.lap_times_s.end());
    double sum = 0.0;
    for (const double lap : result.lap_times_s) {
      sum += lap;
    }
    result.mean_lap_s = sum / static_cast<double>(result.lap_times_s.size());
  }
  if (cte_samples > 0) {
    result.cte_rmse_m = std::sqrt(cte_sq_sum / static_cast<double>(cte_samples));
  }
  if (moving_samples > 0) {
    result.violation_frac =
      static_cast<double>(violation_samples) / static_cast<double>(moving_samples);
  }
  if (!std::isfinite(result.boundary_min_m)) {
    result.boundary_min_m = 0.0;
  }
  if (steer_rate_samples > 0) {
    result.steer_rate_mean_radps = steer_rate_sum / static_cast<double>(steer_rate_samples);
  }
  if (control_cycles > 0) {
    result.steer_sat_frac = static_cast<double>(sat_cycles) / static_cast<double>(control_cycles);
    result.planner_invalid_frac =
      static_cast<double>(invalid_cycles) / static_cast<double>(control_cycles);
  }
  if (speed_samples > 0) {
    result.speed_rmse_mps = std::sqrt(speed_err_sq_sum / static_cast<double>(speed_samples));
  }
  if (result.sim_time_s > 0.0) {
    result.avg_speed_mps = result.distance_m / result.sim_time_s;
  }
  return result;
}

HarnessResult runMapHarness(const HarnessConfig & config)
{
  const auto track = loadTrackCsv(config.track_csv);
  if (!track.has_value()) {
    HarnessResult result;
    result.dnf = true;
    result.dnf_reason = "track_csv_unreadable: " + config.track_csv;
    return result;
  }
  return runMapHarness(config, *track);
}

}  // namespace hyu_control_harness
