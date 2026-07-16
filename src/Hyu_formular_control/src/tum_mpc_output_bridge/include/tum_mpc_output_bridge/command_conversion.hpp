#ifndef TUM_MPC_OUTPUT_BRIDGE__COMMAND_CONVERSION_HPP_
#define TUM_MPC_OUTPUT_BRIDGE__COMMAND_CONVERSION_HPP_

#include <cstdint>

namespace tum_mpc_output_bridge
{

constexpr std::uint16_t kTubeMpcStatusOk = 2U;

struct ConversionConfig
{
  double conversion_mass_kg{225.0};
  double force_min_n{-6000.0};
  double force_max_n{6000.0};
  double steering_min_rad{-0.2};
  double steering_max_rad{0.2};
  double acceleration_min_mps2{-5.2};
  double acceleration_max_mps2{5.2};
  double safe_brake_mps2{-5.0};
  double output_timeout_sec{0.1};
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

}  // namespace tum_mpc_output_bridge

#endif  // TUM_MPC_OUTPUT_BRIDGE__COMMAND_CONVERSION_HPP_
