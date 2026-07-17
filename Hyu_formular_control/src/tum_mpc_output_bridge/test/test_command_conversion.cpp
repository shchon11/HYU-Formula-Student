#include <limits>

#include "gtest/gtest.h"
#include "tum_mpc_output_bridge/command_conversion.hpp"

namespace
{

using tum_mpc_output_bridge::BuildCommand;
using tum_mpc_output_bridge::CommandState;
using tum_mpc_output_bridge::ConversionConfig;
using tum_mpc_output_bridge::MpcCommand;

MpcCommand ValidInput(double steering_angle_rad = 0.1, double long_force_n = 1300.0)
{
  MpcCommand input;
  input.steering_angle_rad = steering_angle_rad;
  input.long_force_n = long_force_n;
  input.tube_mpc_status = tum_mpc_output_bridge::kTubeMpcStatusOk;
  return input;
}

TEST(CommandConversion, DividesForceByGeneratedModelMass)
{
  // 225 N on the 225 kg model baked into the generated MPC -> 1 m/s^2.
  const auto result = BuildCommand(ValidInput(0.1, 225.0), true, 0.01, ConversionConfig{});

  ASSERT_TRUE(result.valid());
  EXPECT_DOUBLE_EQ(result.command.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.command.acceleration_mps2, 1.0);
  EXPECT_DOUBLE_EQ(result.command.steering_angle_rad, 0.1);
}

TEST(CommandConversion, ClampsForceBeforeMassConversion)
{
  ConversionConfig config;
  config.conversion_mass_kg = 1000.0;
  config.acceleration_min_mps2 = -100.0;
  config.acceleration_max_mps2 = 100.0;
  config.safe_brake_mps2 = -5.0;

  EXPECT_DOUBLE_EQ(
    BuildCommand(ValidInput(0.0, 9000.0), true, 0.01, config).command.acceleration_mps2,
    6.0);
  EXPECT_DOUBLE_EQ(
    BuildCommand(ValidInput(0.0, -9000.0), true, 0.01, config).command.acceleration_mps2,
    -6.0);
}

TEST(CommandConversion, ClampsAccelerationAndSteering)
{
  ConversionConfig config;
  config.conversion_mass_kg = 100.0;

  const auto positive = BuildCommand(ValidInput(1.0, 6000.0), true, 0.01, config);
  const auto negative = BuildCommand(ValidInput(-1.0, -6000.0), true, 0.01, config);

  ASSERT_TRUE(positive.valid());
  ASSERT_TRUE(negative.valid());
  EXPECT_DOUBLE_EQ(positive.command.acceleration_mps2, 3.0);
  EXPECT_DOUBLE_EQ(negative.command.acceleration_mps2, -10.0);
  EXPECT_DOUBLE_EQ(positive.command.steering_angle_rad, 0.52);
  EXPECT_DOUBLE_EQ(negative.command.steering_angle_rad, -0.52);
}

TEST(CommandConversion, RejectsDivergedSteeringInsteadOfClampingIt)
{
  // Observed at a GLOBAL handoff far outside the tube: the QP returned
  // -2.89 rad on a +-0.52 rad actuator. Clamping would forward a confident
  // full-lock command; the guard must fail closed instead.
  const auto diverged = BuildCommand(ValidInput(-2.89, 500.0), true, 0.01, ConversionConfig{});
  EXPECT_EQ(diverged.state, CommandState::kDivergedSteering);
  EXPECT_FALSE(diverged.valid());
  EXPECT_DOUBLE_EQ(diverged.command.acceleration_mps2, -5.0);
  EXPECT_DOUBLE_EQ(diverged.command.steering_angle_rad, 0.0);

  // Just past the reject bound (3.0 * 0.52 = 1.56) fails; the model's own
  // internal saturation (~1.04 rad) stays clamped-drivable.
  EXPECT_EQ(
    BuildCommand(ValidInput(1.57, 500.0), true, 0.01, ConversionConfig{}).state,
    CommandState::kDivergedSteering);
  const auto saturated = BuildCommand(ValidInput(1.04, 500.0), true, 0.01, ConversionConfig{});
  ASSERT_TRUE(saturated.valid());
  EXPECT_DOUBLE_EQ(saturated.command.steering_angle_rad, 0.52);
}

TEST(CommandConversion, RejectsNonFiniteMpcInputs)
{
  auto nan_steering = ValidInput();
  nan_steering.steering_angle_rad = std::numeric_limits<double>::quiet_NaN();
  auto inf_force = ValidInput();
  inf_force.long_force_n = std::numeric_limits<double>::infinity();

  const auto nan_result = BuildCommand(nan_steering, true, 0.01, ConversionConfig{});
  const auto inf_result = BuildCommand(inf_force, true, 0.01, ConversionConfig{});

  EXPECT_EQ(nan_result.state, CommandState::kNonFiniteInput);
  EXPECT_EQ(inf_result.state, CommandState::kNonFiniteInput);
  EXPECT_FALSE(nan_result.valid());
  EXPECT_FALSE(inf_result.valid());
  EXPECT_DOUBLE_EQ(nan_result.command.acceleration_mps2, -5.0);
  EXPECT_DOUBLE_EQ(inf_result.command.acceleration_mps2, -5.0);
}

TEST(CommandConversion, RejectsNonOkTubeMpcStatus)
{
  auto input = ValidInput();
  input.tube_mpc_status = 0U;

  const auto result = BuildCommand(input, true, 0.01, ConversionConfig{});

  EXPECT_EQ(result.state, CommandState::kInvalidStatus);
  EXPECT_FALSE(result.valid());
  EXPECT_DOUBLE_EQ(result.command.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.command.acceleration_mps2, -5.0);
  EXPECT_DOUBLE_EQ(result.command.steering_angle_rad, 0.0);
}

TEST(CommandConversion, UsesSafeBrakeWithoutInputOrAfterTimeout)
{
  const auto no_input = BuildCommand(MpcCommand{}, false, 0.0, ConversionConfig{});
  const auto stale = BuildCommand(ValidInput(), true, 0.1001, ConversionConfig{});

  EXPECT_EQ(no_input.state, CommandState::kNoInput);
  EXPECT_EQ(stale.state, CommandState::kStale);
  EXPECT_FALSE(no_input.valid());
  EXPECT_FALSE(stale.valid());
  EXPECT_DOUBLE_EQ(no_input.command.acceleration_mps2, -5.0);
  EXPECT_DOUBLE_EQ(stale.command.acceleration_mps2, -5.0);
  EXPECT_DOUBLE_EQ(stale.command.steering_angle_rad, 0.0);
}

TEST(CommandConversion, RejectsInvalidConfiguration)
{
  ConversionConfig config;
  config.conversion_mass_kg = 0.0;

  const auto result = BuildCommand(ValidInput(), true, 0.01, config);

  EXPECT_EQ(result.state, CommandState::kInvalidConfig);
  EXPECT_DOUBLE_EQ(result.command.acceleration_mps2, 0.0);
}

}  // namespace
