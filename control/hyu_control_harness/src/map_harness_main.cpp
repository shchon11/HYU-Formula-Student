#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include "hyu_control_harness/closed_loop.hpp"

namespace
{

constexpr const char * kUsage = R"(map_harness -- headless closed-loop pure-pursuit/MAP tuning run

Usage: map_harness track=<track.csv> plant_yaml=<configDry.yaml> [key=value ...]

Harness keys:
  sim_time_s control_delay_s steering_lock_time_s odom_rate_hz odom_latency_s
  odom_noise_xy_m odom_noise_yaw_rad odom_noise_v_mps seed traj_csv
Controller keys:
  steering_mode (map|geometric) max_speed_mps longitudinal_kp
  min_acceleration_mps2 max_acceleration_mps2 brake_acceleration_mps2
  map_lookahead_slope_s map_lookahead_intercept_m map_lookahead_min_m
  map_lookahead_max_m map_speed_source (planned|measured)
Planner keys:
  two_sided_speed_mps max_lateral_accel_mps2 min_speed_mps two_sided_horizon_m
  stop_at_path_end (0|1) end_stop_decel_mps2 end_stop_margin_m

Prints one JSON object with the run metrics to stdout.
)";

bool parseArg(const std::string & key, const std::string & value,
  hyu_control_harness::HarnessConfig & config)
{
  auto & controller = config.controller;
  auto & planner = config.planner;
  try {
    if (key == "track") {
      config.track_csv = value;
    } else if (key == "plant_yaml") {
      config.plant_yaml = value;
    } else if (key == "traj_csv") {
      config.traj_csv = value;
    } else if (key == "sim_time_s") {
      config.sim_time_s = std::stod(value);
    } else if (key == "control_delay_s") {
      config.control_delay_s = std::stod(value);
    } else if (key == "steering_lock_time_s") {
      config.steering_lock_time_s = std::stod(value);
    } else if (key == "odom_rate_hz") {
      config.odom_rate_hz = std::stod(value);
    } else if (key == "odom_latency_s") {
      config.odom_latency_s = std::stod(value);
    } else if (key == "odom_noise_xy_m") {
      config.odom_noise_xy_m = std::stod(value);
    } else if (key == "odom_noise_yaw_rad") {
      config.odom_noise_yaw_rad = std::stod(value);
    } else if (key == "odom_noise_v_mps") {
      config.odom_noise_v_mps = std::stod(value);
    } else if (key == "seed") {
      config.seed = static_cast<unsigned int>(std::stoul(value));
    } else if (key == "steering_mode") {
      if (value == "map") {
        controller.steering_mode = hyu_pure_pursuit::SteeringMode::MAP;
      } else if (value == "geometric") {
        controller.steering_mode = hyu_pure_pursuit::SteeringMode::GEOMETRIC;
      } else {
        return false;
      }
    } else if (key == "map_speed_source") {
      if (value == "planned") {
        controller.map_speed_source = hyu_pure_pursuit::MapSpeedSource::PLANNED;
      } else if (value == "measured") {
        controller.map_speed_source = hyu_pure_pursuit::MapSpeedSource::MEASURED;
      } else {
        return false;
      }
    } else if (key == "max_speed_mps") {
      controller.max_speed_mps = std::stod(value);
    } else if (key == "longitudinal_kp") {
      controller.longitudinal_kp = std::stod(value);
    } else if (key == "min_acceleration_mps2") {
      controller.min_acceleration_mps2 = std::stod(value);
    } else if (key == "max_acceleration_mps2") {
      controller.max_acceleration_mps2 = std::stod(value);
    } else if (key == "brake_acceleration_mps2") {
      controller.brake_acceleration_mps2 = std::stod(value);
    } else if (key == "map_lookahead_slope_s") {
      controller.map_lookahead_slope_s = std::stod(value);
    } else if (key == "map_lookahead_intercept_m") {
      controller.map_lookahead_intercept_m = std::stod(value);
    } else if (key == "map_lookahead_min_m") {
      controller.map_lookahead_min_m = std::stod(value);
    } else if (key == "map_lookahead_max_m") {
      controller.map_lookahead_max_m = std::stod(value);
    } else if (key == "two_sided_speed_mps") {
      planner.two_sided_speed_mps = std::stod(value);
    } else if (key == "max_lateral_accel_mps2") {
      planner.max_lateral_accel_mps2 = std::stod(value);
    } else if (key == "min_speed_mps") {
      planner.min_speed_mps = std::stod(value);
    } else if (key == "two_sided_horizon_m") {
      planner.two_sided_horizon_m = std::stod(value);
    } else if (key == "stop_at_path_end") {
      planner.stop_at_path_end = (value == "1" || value == "true");
    } else if (key == "end_stop_decel_mps2") {
      planner.end_stop_decel_mps2 = std::stod(value);
    } else if (key == "end_stop_margin_m") {
      planner.end_stop_margin_m = std::stod(value);
    } else {
      return false;
    }
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  hyu_control_harness::HarnessConfig config;
  config.controller = hyu_control_harness::trackdriveControllerConfig();
  config.lut_model = hyu_control_harness::trackdriveLutModel();
  config.lut_grid = hyu_control_harness::trackdriveLutGrid();
  config.planner = hyu_control_harness::trackdrivePlannerConfig();

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "-h" || arg == "--help") {
      std::cout << kUsage;
      return 0;
    }
    const auto eq = arg.find('=');
    if (eq == std::string::npos || !parseArg(arg.substr(0, eq), arg.substr(eq + 1), config)) {
      std::cerr << "unknown or malformed argument: " << arg << "\n" << kUsage;
      return 2;
    }
  }
  if (config.track_csv.empty() || config.plant_yaml.empty()) {
    std::cerr << "track= and plant_yaml= are required\n" << kUsage;
    return 2;
  }

  const auto result = hyu_control_harness::runMapHarness(config);
  std::cout << result.toJson() << std::endl;
  return result.dnf ? 1 : 0;
}
