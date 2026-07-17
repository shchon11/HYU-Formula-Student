#ifndef HYU_FORMULAR_CONTROL_CONFIG_HPP_
#define HYU_FORMULAR_CONTROL_CONFIG_HPP_

#include <string>

struct HyuFormulaControlConfig
{
  std::string vehicle_state_topic{"/control/tmpc/vehicle_state"};
  std::string performance_trajectory_topic{"/planning/trajectory_performance"};
  std::string emergency_trajectory_topic{"/planning/trajectory_emergency"};
  std::string output_topic{"/output"};

  double loop_rate_hz{100.0};
  double state_timeout_sec{0.5};
  double performance_trajectory_timeout_sec{0.5};
  double emergency_trajectory_timeout_sec{0.5};
  bool enable_emergency{false};
  bool publish_on_timeout{false};

  // Actually-applied command feedback (the selector-owned /vehicle/cmd). The MPC's
  // ActualVehicleControl input must be what the PLANT received, not this
  // node's own last request: while Pure Pursuit owns /vehicle/cmd (LOCAL, fallback,
  // pre-takeover) the MPC's requests are never applied, and feeding them back
  // anyway poisons its disturbance/uncertainty learning -- the first commands
  // after a takeover then come out saturated in the wrong direction.
  std::string applied_command_topic{"/vehicle/cmd"};
  double applied_command_timeout_sec{0.5};
  // Mass for converting the applied acceleration back to the MPC's force
  // units; must match the generated model / output bridge (225 kg).
  double applied_command_mass_kg{225.0};

  // Divergence recovery: the RTI-SQP iterates once per cycle, so a diverged
  // solution (steering far beyond the actuator) never heals on its own -- it
  // keeps resurfacing at the next takeover. When the raw request exceeds
  // factor * actuator limit, reinitialize the MPC model so it re-converges
  // from a clean iterate at the current state. 3.0 (not 2.0): the generated
  // model's own internal steering saturation sits near 2x the road-wheel
  // limit (rides 1.04 rad on a 0.52 rad actuator), which is drivable after
  // the output clamp; only a runaway iterate (observed -2.89 rad) resets.
  double divergence_reset_factor{3.0};

  // eufs robot road-wheel limits (eufs_racecar/robots/eufs/configDry.yaml).
  double steering_angle_min_rad{-0.52};
  double steering_angle_max_rad{0.52};
  // TODO(tmpc): confirm against measured drivetrain force; ±6000 N is far
  // beyond the measured ax limit (11.772 m/s^2 * 225 kg ~= 2650 N).
  double drive_force_min_n{-6000.0};
  double drive_force_max_n{6000.0};
};

#endif  // HYU_FORMULAR_CONTROL_CONFIG_HPP_
