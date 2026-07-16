#ifndef HYU_FORMULAR_CONTROL_CONFIG_HPP_
#define HYU_FORMULAR_CONTROL_CONFIG_HPP_

#include <string>

struct HyuFormulaControlConfig
{
  std::string vehicle_state_topic{"/tmpc/vehicle_state"};
  std::string performance_trajectory_topic{"/tmpc/trajectory_performance"};
  std::string emergency_trajectory_topic{"/tmpc/trajectory_emergency"};
  std::string output_topic{"/output"};

  double loop_rate_hz{100.0};
  double state_timeout_sec{0.5};
  double performance_trajectory_timeout_sec{0.5};
  double emergency_trajectory_timeout_sec{0.5};
  bool enable_emergency{false};
  bool publish_on_timeout{false};

  // eufs robot road-wheel limits (eufs_racecar/robots/eufs/configDry.yaml).
  double steering_angle_min_rad{-0.52};
  double steering_angle_max_rad{0.52};
  // TODO(tmpc): confirm against measured drivetrain force; ±6000 N is far
  // beyond the measured ax limit (11.772 m/s^2 * 225 kg ~= 2650 N).
  double drive_force_min_n{-6000.0};
  double drive_force_max_n{6000.0};
};

#endif  // HYU_FORMULAR_CONTROL_CONFIG_HPP_
