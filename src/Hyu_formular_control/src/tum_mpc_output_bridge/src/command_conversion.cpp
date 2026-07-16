#include "tum_mpc_output_bridge/command_conversion.hpp"

#include <algorithm>
#include <cmath>

namespace tum_mpc_output_bridge
{
namespace
{

double Clamp(double value, double lower, double upper)
{
  return std::min(std::max(value, lower), upper);
}

AckermannCommand SafeCommand(const ConversionConfig & config)
{
  AckermannCommand command;
  command.speed_mps = 0.0;
  command.acceleration_mps2 = config.safe_brake_mps2;
  command.steering_angle_rad = 0.0;
  return command;
}

ConversionResult SafeResult(const ConversionConfig & config, CommandState state)
{
  ConversionResult result;
  result.command = SafeCommand(config);
  result.state = state;
  return result;
}

}  // namespace

bool IsConversionConfigValid(const ConversionConfig & config)
{
  const bool all_finite =
    std::isfinite(config.conversion_mass_kg) &&
    std::isfinite(config.force_min_n) &&
    std::isfinite(config.force_max_n) &&
    std::isfinite(config.steering_min_rad) &&
    std::isfinite(config.steering_max_rad) &&
    std::isfinite(config.acceleration_min_mps2) &&
    std::isfinite(config.acceleration_max_mps2) &&
    std::isfinite(config.safe_brake_mps2) &&
    std::isfinite(config.output_timeout_sec);

  return all_finite &&
         config.conversion_mass_kg > 0.0 &&
         config.force_min_n <= config.force_max_n &&
         config.steering_min_rad <= 0.0 &&
         config.steering_max_rad >= 0.0 &&
         config.steering_min_rad <= config.steering_max_rad &&
         config.acceleration_min_mps2 <= config.acceleration_max_mps2 &&
         config.safe_brake_mps2 >= config.acceleration_min_mps2 &&
         config.safe_brake_mps2 <= config.acceleration_max_mps2 &&
         config.output_timeout_sec > 0.0;
}

ConversionResult BuildCommand(
  const MpcCommand & input,
  bool has_input,
  double input_age_sec,
  const ConversionConfig & config)
{
  if (!IsConversionConfigValid(config)) {
    ConversionResult result;
    result.command = AckermannCommand{};
    result.state = CommandState::kInvalidConfig;
    return result;
  }

  if (!has_input) {
    return SafeResult(config, CommandState::kNoInput);
  }

  if (!std::isfinite(input_age_sec) || input_age_sec < 0.0 ||
    input_age_sec > config.output_timeout_sec)
  {
    return SafeResult(config, CommandState::kStale);
  }

  if (input.tube_mpc_status != kTubeMpcStatusOk) {
    return SafeResult(config, CommandState::kInvalidStatus);
  }

  if (!std::isfinite(input.steering_angle_rad) || !std::isfinite(input.long_force_n)) {
    return SafeResult(config, CommandState::kNonFiniteInput);
  }

  const double force_n = Clamp(input.long_force_n, config.force_min_n, config.force_max_n);
  const double acceleration_mps2 = force_n / config.conversion_mass_kg;

  ConversionResult result;
  result.command.speed_mps = 0.0;
  result.command.acceleration_mps2 = Clamp(
    acceleration_mps2, config.acceleration_min_mps2, config.acceleration_max_mps2);
  result.command.steering_angle_rad = Clamp(
    input.steering_angle_rad, config.steering_min_rad, config.steering_max_rad);
  result.state = CommandState::kValid;
  return result;
}

const char * CommandStateName(CommandState state)
{
  switch (state) {
    case CommandState::kValid:
      return "valid";
    case CommandState::kNoInput:
      return "no_input";
    case CommandState::kStale:
      return "stale";
    case CommandState::kInvalidStatus:
      return "invalid_status";
    case CommandState::kNonFiniteInput:
      return "non_finite_input";
    case CommandState::kInvalidConfig:
      return "invalid_config";
    default:
      return "unknown";
  }
}

}  // namespace tum_mpc_output_bridge
