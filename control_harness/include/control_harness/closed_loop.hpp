#pragma once

#include <string>
#include <vector>

#include "control_harness/track.hpp"
#include "local_planner/local_path_builder.hpp"
#include "pure_pursuit_controller/controller.hpp"
#include "pure_pursuit_controller/steering_lookup.hpp"

namespace control_harness
{

// Closed-loop chain mirrored from the live stack:
//   cone map (csv, ground truth) -> ego-frame transform with the *measured* pose
//   -> local_planner::buildLocalPath -> map-frame path -> pure pursuit (MAP or
//   geometric steering) -> command queue (plant control delay) -> steering-rate
//   limit -> eufs DynamicBicycle plant.
// The measured pose models the localization chain: sampled at odom_rate_hz,
// delayed by odom_latency_s, with optional additive gaussian noise. With zero
// latency/noise MAP inverts the plant almost exactly and the loop is too
// optimistic to tune against -- keep the defaults non-ideal.
struct HarnessConfig
{
  std::string track_csv;
  std::string plant_yaml;
  double sim_time_s{300.0};
  double plant_dt_s{0.001};

  // Plant-side command chain (eufs_plugins.gazebo.xacro values).
  double control_delay_s{0.2};
  double steering_lock_time_s{1.0};

  // Measured-state (localization) chain.
  double odom_rate_hz{50.0};
  double odom_latency_s{0.05};
  double odom_noise_xy_m{0.0};
  double odom_noise_yaw_rad{0.0};
  double odom_noise_v_mps{0.0};
  unsigned int seed{1U};

  // Optional 100 Hz trajectory dump (t,x,y,yaw,v,s,cte,delta_cmd,delta_act,...).
  std::string traj_csv;

  pure_pursuit_controller::ControllerConfig controller;
  pure_pursuit_controller::VehicleModel lut_model;
  pure_pursuit_controller::LutGrid lut_grid;
  local_planner::PlannerConfig planner;
};

// Defaults mirroring the shipped trackdrive configs
// (pure_pursuit_controller.yaml / local_planner.yaml).
pure_pursuit_controller::ControllerConfig trackdriveControllerConfig();
pure_pursuit_controller::VehicleModel trackdriveLutModel();
pure_pursuit_controller::LutGrid trackdriveLutGrid();
local_planner::PlannerConfig trackdrivePlannerConfig();

struct HarnessResult
{
  bool dnf{false};
  std::string dnf_reason;

  int laps{0};
  std::vector<double> lap_times_s;
  double best_lap_s{0.0};
  double mean_lap_s{0.0};

  // Tracking quality, sampled at the plant rate while v > 0.5 m/s.
  double cte_rmse_m{0.0};
  double cte_max_m{0.0};
  // Fraction of moving samples where the car body (half width 0.7 m) reaches
  // past the cone line, and the worst clearance to the cone line (negative =
  // centre of the car beyond the cones).
  double violation_frac{0.0};
  double boundary_min_m{0.0};

  double steer_rate_mean_radps{0.0};
  double steer_rate_max_radps{0.0};
  double steer_sat_frac{0.0};
  double speed_rmse_mps{0.0};

  double planner_invalid_frac{0.0};
  std::string last_planner_reason;

  double distance_m{0.0};
  double avg_speed_mps{0.0};
  double sim_time_s{0.0};

  std::string toJson() const;
};

HarnessResult runMapHarness(const HarnessConfig & config, const Track & track);
HarnessResult runMapHarness(const HarnessConfig & config);

}  // namespace control_harness
