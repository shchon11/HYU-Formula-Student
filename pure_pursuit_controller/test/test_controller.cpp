#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "gtest/gtest.h"
#include "pure_pursuit_controller/controller.hpp"

namespace pure_pursuit_controller
{
namespace
{

ControllerInput readyInput(const std::vector<PathPoint> & path)
{
  ControllerInput input;
  input.path_received = true;
  input.validity_received = true;
  input.stop_received = true;
  input.odom_received = true;
  input.path_frame_valid = true;
  input.odom_frame_valid = true;
  input.selected_path_valid = true;
  input.stop_requested = false;
  input.path = path;
  input.ego = EgoState{};
  return input;
}

std::vector<PathPoint> straightPath(double vx_mps = 3.0, double speed_mps = 3.0)
{
  return {
    PathPoint{0.0, 0.0, vx_mps, speed_mps},
    PathPoint{2.0, 0.0, vx_mps, speed_mps},
    PathPoint{4.0, 0.0, vx_mps, speed_mps},
  };
}

void expectExactBrake(const DriveCommand & command)
{
  EXPECT_DOUBLE_EQ(command.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(command.acceleration_mps2, -5.0);
  EXPECT_DOUBLE_EQ(command.steering_angle_rad, 0.0);
}

TEST(PurePursuitController, StraightPathProducesNearZeroSteer)
{
  const auto command = computeCommand(readyInput(straightPath()));

  EXPECT_NEAR(command.steering_angle_rad, 0.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(command.speed_mps, 3.0);
}

TEST(PurePursuitController, LeftAndRightTargetsProduceCorrectSteerSign)
{
  const std::vector<PathPoint> tied_nearest{
    PathPoint{-1.0, 0.0, 2.0, 2.0},
    PathPoint{1.0, 0.0, 2.0, 2.0},
    PathPoint{5.0, 0.0, 2.0, 2.0},
  };
  const auto nearest = findNearestWaypoint(tied_nearest, EgoState{});
  ASSERT_TRUE(nearest.has_value());
  EXPECT_EQ(nearest.value(), 0U);

  const std::vector<PathPoint> left_path{
    PathPoint{0.0, 0.0, 2.0, 2.0},
    PathPoint{2.0, 1.0, 2.0, 2.0},
    PathPoint{4.0, 2.0, 2.0, 2.0},
  };
  const std::vector<PathPoint> right_path{
    PathPoint{0.0, 0.0, 2.0, 2.0},
    PathPoint{2.0, -1.0, 2.0, 2.0},
    PathPoint{4.0, -2.0, 2.0, 2.0},
  };

  EXPECT_GT(computeCommand(readyInput(left_path)).steering_angle_rad, 0.0);
  EXPECT_LT(computeCommand(readyInput(right_path)).steering_angle_rad, 0.0);
}

TEST(PurePursuitController, SteeringAndAccelerationAreClamped)
{
  const std::vector<PathPoint> hard_left{
    PathPoint{0.0, 0.0, 100.0, 100.0},
    PathPoint{0.1, 4.0, 100.0, 100.0},
  };
  auto accelerating = readyInput(hard_left);
  const auto accelerate_command = computeCommand(accelerating);
  EXPECT_DOUBLE_EQ(accelerate_command.steering_angle_rad, 0.52);
  EXPECT_DOUBLE_EQ(accelerate_command.speed_mps, 4.5);
  EXPECT_DOUBLE_EQ(accelerate_command.acceleration_mps2, 2.5);

  auto braking = readyInput(hard_left);
  braking.ego->longitudinal_speed_mps = 100.0;
  const auto brake_command = computeCommand(braking);
  EXPECT_DOUBLE_EQ(brake_command.acceleration_mps2, -8.0);
}

TEST(PurePursuitController, VxPriorityFallsBackToSpeedThenZero)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(computeCommand(readyInput(straightPath(2.0, 3.0))).speed_mps, 2.0);
  EXPECT_DOUBLE_EQ(computeCommand(readyInput(straightPath(nan, 3.0))).speed_mps, 3.0);
  EXPECT_DOUBLE_EQ(computeCommand(readyInput(straightPath(-1.0, 4.0))).speed_mps, 4.0);
  EXPECT_DOUBLE_EQ(computeCommand(readyInput(straightPath(nan, nan))).speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(computeCommand(readyInput(straightPath(infinity, 3.0))).speed_mps, 3.0);
  EXPECT_DOUBLE_EQ(computeCommand(readyInput(straightPath(-infinity, 3.0))).speed_mps, 3.0);
  EXPECT_DOUBLE_EQ(computeCommand(readyInput(straightPath(nan, infinity))).speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(computeCommand(readyInput(straightPath(nan, -infinity))).speed_mps, 0.0);

  auto nonfinite_speed = readyInput(straightPath(1.0, 1.0));
  nonfinite_speed.ego->longitudinal_speed_mps = nan;
  EXPECT_DOUBLE_EQ(computeCommand(nonfinite_speed).acceleration_mps2, 1.2);
}

TEST(PurePursuitController, ConfiguredBrakeDecelerationIsUsed)
{
  // The hard brake honours brake_acceleration_mps2 (the acceleration mission
  // models a 3 m/s^2 vehicle), independent of the in-path min_acceleration.
  ControllerConfig soft_brake;
  soft_brake.brake_acceleration_mps2 = -3.0;
  auto invalid = readyInput(straightPath(2.0, 2.0));
  invalid.selected_path_valid = false;
  const auto braked = computeCommand(invalid, soft_brake);
  EXPECT_DOUBLE_EQ(braked.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(braked.acceleration_mps2, -3.0);

  // A config whose brake value is not a real deceleration is invalid, but the
  // car must still stop: the brake falls back to the historical -5.0 m/s^2.
  ControllerConfig bad;
  bad.brake_acceleration_mps2 = 1.0;
  const auto fallback = computeCommand(invalid, bad);
  EXPECT_DOUBLE_EQ(fallback.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(fallback.acceleration_mps2, -5.0);
}

TEST(PurePursuitController, InvalidStaleOrStopCommandsBrake)
{
  const ControllerConfig config;
  auto boundary = readyInput(straightPath(2.0, 2.0));
  boundary.path_age_sec = 0.5;
  boundary.validity_age_sec = 0.5;
  boundary.stop_age_sec = 0.5;
  boundary.odom_age_sec = 0.5;
  EXPECT_GT(computeCommand(boundary, config).speed_mps, 0.0);
  EXPECT_EQ(commandPeriod(config), std::chrono::milliseconds(50));

  auto invalid = boundary;
  invalid.selected_path_valid = false;
  expectExactBrake(computeCommand(invalid, config));

  auto stale_path = boundary;
  stale_path.path_age_sec = 0.500001;
  expectExactBrake(computeCommand(stale_path, config));
  stale_path.path_age_sec = 0.0;
  EXPECT_GT(computeCommand(stale_path, config).speed_mps, 0.0);

  auto stale_validity = boundary;
  stale_validity.validity_age_sec = 0.500001;
  expectExactBrake(computeCommand(stale_validity, config));
  stale_validity.validity_age_sec = 0.0;
  EXPECT_GT(computeCommand(stale_validity, config).speed_mps, 0.0);

  auto stale_odom = boundary;
  stale_odom.odom_age_sec = 0.500001;
  expectExactBrake(computeCommand(stale_odom, config));
  stale_odom.odom_age_sec = 0.0;
  EXPECT_GT(computeCommand(stale_odom, config).speed_mps, 0.0);

  auto missing_stop = boundary;
  missing_stop.stop_received = false;
  expectExactBrake(computeCommand(missing_stop, config));

  auto stale_stop = boundary;
  stale_stop.stop_age_sec = 0.500001;
  expectExactBrake(computeCommand(stale_stop, config));
  stale_stop.stop_age_sec = 0.0;
  EXPECT_GT(computeCommand(stale_stop, config).speed_mps, 0.0);

  auto stopped = boundary;
  stopped.stop_requested = true;
  expectExactBrake(computeCommand(stopped, config));

  auto malformed_path = boundary;
  malformed_path.path[1].x_m = std::numeric_limits<double>::infinity();
  expectExactBrake(computeCommand(malformed_path, config));

  auto malformed_odom = boundary;
  malformed_odom.ego->yaw_rad = std::numeric_limits<double>::quiet_NaN();
  expectExactBrake(computeCommand(malformed_odom, config));

  auto missing_path = boundary;
  missing_path.path_received = false;
  expectExactBrake(computeCommand(missing_path, config));

  auto missing_validity = boundary;
  missing_validity.validity_received = false;
  expectExactBrake(computeCommand(missing_validity, config));

  auto missing_odom = boundary;
  missing_odom.odom_received = false;
  expectExactBrake(computeCommand(missing_odom, config));

  auto nonfinite_path_age = boundary;
  nonfinite_path_age.path_age_sec = std::numeric_limits<double>::infinity();
  expectExactBrake(computeCommand(nonfinite_path_age, config));

  EXPECT_FALSE(yawFromQuaternion(0.0, 0.0, 0.0, 0.0).has_value());
  EXPECT_FALSE(yawFromQuaternion(0.0, 0.0, 0.0, std::numeric_limits<double>::infinity()).has_value());
  EXPECT_TRUE(yawFromQuaternion(0.0, 0.0, 0.0, 1.0).has_value());
}

TEST(PurePursuitController, OdomFrameContractRequiresMapAndBaseFootprint)
{
  auto wrong_child = readyInput(straightPath(2.0, 2.0));
  wrong_child.odom_frame_valid = hasExpectedOdometryFrameIds("map", "odom");

  EXPECT_FALSE(hasExpectedOdometryFrameIds("map", "odom"));
  expectExactBrake(computeCommand(wrong_child));

  auto valid_odom = readyInput(straightPath(2.0, 2.0));
  valid_odom.odom_frame_valid = hasExpectedOdometryFrameIds("map", "base_footprint");

  EXPECT_TRUE(hasExpectedOdometryFrameIds("map", "base_footprint"));
  EXPECT_GT(computeCommand(valid_odom).speed_mps, 0.0);
}

TEST(PurePursuitController, NoForwardTargetCommandsBrake)
{
  auto behind = readyInput({
    PathPoint{0.0, 0.0, 2.0, 2.0},
    PathPoint{1.0, 0.0, 2.0, 2.0},
  });
  behind.ego = EgoState{5.0, 0.0, 0.0, 0.0};
  expectExactBrake(computeCommand(behind));

  auto final_point_front = readyInput({
    PathPoint{0.0, 0.0, 2.0, 2.0},
    PathPoint{1.0, 0.0, 2.0, 2.0},
    PathPoint{2.0, 0.0, 2.0, 2.0},
  });
  EXPECT_DOUBLE_EQ(computeCommand(final_point_front).speed_mps, 2.0);

  auto empty = readyInput({});
  expectExactBrake(computeCommand(empty));

  auto zero_target = readyInput({
    PathPoint{0.0, 0.0, 2.0, 2.0},
    PathPoint{0.0, 0.0, 2.0, 2.0},
  });
  expectExactBrake(computeCommand(zero_target));
}

}
}
