#ifndef TUM_MPC_OUTPUT_BRIDGE__COMMAND_CONVERSION_HPP_
#define TUM_MPC_OUTPUT_BRIDGE__COMMAND_CONVERSION_HPP_

#include <cstdint>

namespace hyu_tmpc_output_bridge
{

constexpr std::uint16_t kTubeMpcStatusOk = 2U;

struct ConversionConfig
{
  // Must equal the vehicle mass baked into the generated MPC (mvdc_mpc_data.cpp
  // vehiclemass_kg): the MPC plans force for that model, so dividing by the
  // same mass recovers the acceleration it intended.
  double conversion_mass_kg{225.0};
  // TODO(tmpc): confirm against measured drivetrain force; ±6000 N is far
  // beyond the measured ax limit (11.772 m/s^2 * 225 kg ~= 2650 N).
  double force_min_n{-6000.0};
  double force_max_n{6000.0};
  // eufs robot plant limits (eufs_racecar/robots/eufs/configDry.yaml).
  double steering_min_rad{-0.52};
  double steering_max_rad{0.52};
  double acceleration_min_mps2{-10.0};
  double acceleration_max_mps2{3.0};
  double safe_brake_mps2{-5.0};
  double output_timeout_sec{0.1};
  // Divergence guard: the generated model's own internal steering saturation
  // sits near 2x the road-wheel limit (rides 1.04 rad on a 0.52 rad actuator)
  // and is drivable after the clamp, but a solution this factor beyond the
  // actuator means the QP diverged (observed -2.89 rad at a handoff far
  // outside the tube). Clamping that would forward a "valid" full-lock command
  // and park the car; reject it instead so the selector falls back to Pure
  // Pursuit until the MPC recovers.
  double steering_reject_factor{3.0};
};

struct MpcCommand
{
  double steering_angle_rad{0.0};
  double long_force_n{0.0};
  std::uint16_t tube_mpc_status{0U};
};

struct AckermannCommand
{
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
  double steering_angle_rad{0.0};
};

enum class CommandState
{
  kValid,
  kNoInput,
  kStale,
  kInvalidStatus,
  kNonFiniteInput,
  kDivergedSteering,
  kInvalidConfig,
};

struct ConversionResult
{
  AckermannCommand command{};
  CommandState state{CommandState::kNoInput};

  bool valid() const {return state == CommandState::kValid;}
};

bool IsConversionConfigValid(const ConversionConfig & config);

ConversionResult BuildCommand(
  const MpcCommand & input,
  bool has_input,
  double input_age_sec,
  const ConversionConfig & config);

const char * CommandStateName(CommandState state);

}  // namespace hyu_tmpc_output_bridge

#endif  // TUM_MPC_OUTPUT_BRIDGE__COMMAND_CONVERSION_HPP_
