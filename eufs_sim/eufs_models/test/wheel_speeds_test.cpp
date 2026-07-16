// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "eufs_models/dynamic_bicycle.hpp"
#include "eufs_models/noise.hpp"

namespace {

using eufs::models::DynamicBicycle;
using eufs::models::Input;
using eufs::models::Noise;
using eufs::models::State;

constexpr double kAxleWidth = 1.2;
constexpr double kTireRadius = 0.2525;
const double kRpmPerMps = 60.0 / (2.0 * M_PI * kTireRadius);

/// A config on disk, because Param only knows how to come from a yaml file.
/// peak_slip_ratio < 0 omits the key (exercises the 0.15 default). Aero
/// defaults to zero so the geometry tests see no drag-holding slip.
std::string writeVehicleConfig(const std::string &name, double peak_slip_ratio = -1.0,
                               double c_drag = 0.0, double c_down = 0.0) {
  const std::string path = std::string(std::tmpnam(nullptr)) + name + ".yaml";
  std::ofstream out(path);
  out << "inertia:\n  m: 225.0\n  g: 9.81\n  I_z: 31.27\n"
      << "kinematics:\n  l: 1.53\n  b_F: 0.765\n  b_R: 0.765\n"
      << "  w_front: 0.5\n  axle_width: " << kAxleWidth << "\n"
      << "tire:\n  tire_coefficient: 1.0\n  B: 12.56\n  C: -1.38\n  D: 0.8\n  E: -0.58\n"
      << "  radius: " << kTireRadius << "\n";
  if (peak_slip_ratio >= 0.0) {
    out << "  peak_slip_ratio: " << peak_slip_ratio << "\n";
  }
  out
      << "aero:\n  C_Down: " << c_down << "\n  C_drag: " << c_drag << "\n"
      << "suspension:\n  h_cg: 0.30\n  k_roll: 30000.0\n  k_pitch: 40000.0\n"
      << "  natural_freq_hz: 2.5\n  damping_ratio: 0.5\n"
      << "  load_transfer_to_tires: false\n"
      << "input_ranges:\n  acceleration:\n    max: 5.2\n    min: -5.2\n"
      << "  velocity:\n    max: 30\n    min: 0\n  steering:\n    max: 0.42\n    min: -0.42\n";
  return path;
}

std::string writeNoiseConfig(const std::string &name, double sigma, double quantum) {
  const std::string path = std::string(std::tmpnam(nullptr)) + name + ".yaml";
  std::ofstream out(path);
  out << "noise:\n"
      << "  positionNoise: [0.0, 0.0, 0.0]\n"
      << "  orientationNoise: [0.0, 0.0, 0.0]\n"
      << "  linearVelocityNoise: [0.0, 0.0, 0.0]\n"
      << "  angularVelocityNoise: [0.0, 0.0, 0.0]\n"
      << "  linearAccelerationNoise: [0.0, 0.0, 0.0]\n"
      << "  wheelSpeedNoise: [" << sigma << ", " << sigma << ", " << sigma << ", " << sigma
      << "]\n"
      << "  wheelSpeedQuantumRPM: " << quantum << "\n";
  return path;
}

State moving(double v_x, double r_z) {
  State state{};
  state.v_x = v_x;
  state.r_z = r_z;
  return state;
}

State accelerating(double v_x, double a_x) {
  State state{};
  state.v_x = v_x;
  state.a_x = a_x;
  return state;
}

double rearMean(const eufs_msgs::msg::WheelSpeeds &ws) {
  return 0.5 * (ws.lb_speed + ws.rb_speed);
}

TEST(WheelSpeeds, StraightLineAllFourReadTheCarSpeed) {
  DynamicBicycle model(writeVehicleConfig("straight"));

  const auto ws = model.getWheelSpeeds(moving(10.0, 0.0), Input{});

  EXPECT_FLOAT_EQ(ws.lb_speed, ws.rb_speed);
  EXPECT_FLOAT_EQ(ws.lf_speed, ws.rf_speed);
  EXPECT_FLOAT_EQ(ws.lf_speed, ws.lb_speed);
  EXPECT_NEAR(ws.lb_speed, 10.0 * kRpmPerMps, 1e-2);
}

// In a corner the two sides travel different radii; a left turn (positive
// yaw rate) makes the right/outer wheel faster by axle_width * r_z.
TEST(WheelSpeeds, CornerPutsTheDifferentialAcrossTheAxle) {
  DynamicBicycle model(writeVehicleConfig("corner"));

  const auto ws = model.getWheelSpeeds(moving(10.0, 1.0), Input{});

  EXPECT_GT(ws.rb_speed, ws.lb_speed);
  EXPECT_NEAR(ws.rb_speed - ws.lb_speed, kAxleWidth * 1.0 * kRpmPerMps, 1e-2);
  // The pair's mean is still the axle speed: wheel_odometry averages it.
  EXPECT_NEAR(0.5 * (ws.rb_speed + ws.lb_speed), 10.0 * kRpmPerMps, 1e-2);
}

// 4WD: the fronts are real encoders on steered wheels, so they read their
// contact-patch velocity projected onto the wheel's rolling direction.
TEST(WheelSpeeds, SteeredFrontsProjectOntoTheirRollingDirection) {
  DynamicBicycle model(writeVehicleConfig("fronts"));

  State state = moving(10.0, 1.0);
  state.v_y = 0.5;
  Input input{};
  input.delta = 0.3;
  const auto ws = model.getWheelSpeeds(state, input);

  const double half_track = 0.5 * kAxleWidth * 1.0;
  const double front_lateral = 0.5 + 1.53 * 0.5 * 1.0;  // v_y + l_F * r_z
  const double lf =
      ((10.0 - half_track) * std::cos(0.3) + front_lateral * std::sin(0.3)) * kRpmPerMps;
  const double rf =
      ((10.0 + half_track) * std::cos(0.3) + front_lateral * std::sin(0.3)) * kRpmPerMps;
  EXPECT_NEAR(ws.lf_speed, lf, 5e-2);
  EXPECT_NEAR(ws.rf_speed, rf, 5e-2);
  // The front differential is the track-width term through cos(delta).
  EXPECT_NEAR(ws.rf_speed - ws.lf_speed, kAxleWidth * 1.0 * std::cos(0.3) * kRpmPerMps,
              5e-2);
}

// Drive torque near the grip limit over-reads the encoders; braking
// under-reads them. That over-read is the failure mode wheel odometry meets
// on a real launch, so the sim has to produce it. Utilization comes from
// the ACHIEVED a_x the model wrote into the state, not the command.
TEST(WheelSpeeds, DriveSlipOverReadsAndBrakeSlipUnderReads) {
  DynamicBicycle model(writeVehicleConfig("slip"));

  const double coasting =
      rearMean(model.getWheelSpeeds(accelerating(10.0, 0.0), Input{}));
  const double driving =
      rearMean(model.getWheelSpeeds(accelerating(10.0, 4.0), Input{}));
  const double braking =
      rearMean(model.getWheelSpeeds(accelerating(10.0, -4.0), Input{}));

  EXPECT_GT(driving, coasting);
  EXPECT_LT(braking, coasting);

  // 4WD even-utilization: kappa = peak * a_x/(mu*g), applied to all four.
  const auto launch = model.getWheelSpeeds(accelerating(10.0, 5.2), Input{});
  const double expected = (10.0 + 0.15 * (5.2 / (0.8 * 9.81)) * 10.0) * kRpmPerMps;
  EXPECT_NEAR(rearMean(launch), expected, 5e-2);
  // Fronts slip with the same kappa (straight line: identical to rears).
  EXPECT_NEAR(launch.lf_speed, expected, 5e-2);
}

// Velocity command mode synthesizes acc = (v_des - v)/dt per physics step
// (huge, then clamped): slip keyed off input.acc would flicker at full
// scale. Keyed off the achieved a_x it must ignore the command entirely.
TEST(WheelSpeeds, SlipTracksAchievedDynamicsNotTheCommand) {
  DynamicBicycle model(writeVehicleConfig("velmode"));

  State state = accelerating(10.0, 1.0);
  Input input{};
  input.acc = 100.0;  // one 0.1 m/s velocity error at dt = 1 ms
  const auto ws = model.getWheelSpeeds(state, input);

  const double expected = (10.0 + 0.15 * (1.0 / (0.8 * 9.81)) * 10.0) * kRpmPerMps;
  const double full_scale = (10.0 + 0.15 * 10.0) * kRpmPerMps;
  EXPECT_NEAR(rearMean(ws), expected, 5e-2);
  EXPECT_LT(rearMean(ws), full_scale - 1.0);
}

// At constant speed the drive force holds off aero drag, so a small
// positive slip remains; downforce enlarges the grip it draws from.
TEST(WheelSpeeds, DragHoldsResidualSlipAtConstantSpeed) {
  DynamicBicycle model(writeVehicleConfig("drag", -1.0, 1.0, 2.0));

  const auto ws = model.getWheelSpeeds(accelerating(10.0, 0.0), Input{});

  const double drive = 1.0 * 100.0 / 225.0;                 // c_drag*v^2/m
  const double grip = 0.8 * (9.81 + 2.0 * 100.0 / 225.0);   // mu*(g + c_down*v^2/m)
  const double expected = (10.0 + 0.15 * (drive / grip) * 10.0) * kRpmPerMps;
  EXPECT_GT(rearMean(ws), 10.0 * kRpmPerMps);
  EXPECT_NEAR(rearMean(ws), expected, 5e-2);
}

TEST(WheelSpeeds, ZeroPeakSlipRatioDisablesSlip) {
  DynamicBicycle model(writeVehicleConfig("noslip", 0.0));

  const double launch =
      rearMean(model.getWheelSpeeds(accelerating(10.0, 5.2), Input{}));
  EXPECT_NEAR(launch, 10.0 * kRpmPerMps, 1e-2);
}

// The real chain is an AMK inverter reporting integer MOTOR rpm (int16
// N_act, 18-bit shaft encoder) divided by the gearbox: the wheel-side step
// is 1/ratio RPM, so quantized values are integer in the MOTOR frame.
TEST(WheelSpeedNoise, QuantizesToTheMotorLsbOverGearRatio) {
  const double ratio = 14.5;
  Noise noise(writeNoiseConfig("motorlsb", 0.0, 1.0 / ratio));

  eufs_msgs::msg::WheelSpeeds ws;
  ws.lb_speed = 378.16;
  const auto noisy = noise.applyNoiseToWheelSpeeds(ws);

  // Tolerance absorbs float32 storage and the 6-digit yaml round-trip of
  // 1/14.5; an unquantized value would sit at fraction .32, far outside it.
  const double motor_rpm = noisy.lb_speed * ratio;
  EXPECT_NEAR(motor_rpm, std::round(motor_rpm), 0.05);
  EXPECT_NEAR(noisy.lb_speed, 378.16, 0.5 / ratio);
}

TEST(WheelSpeedNoise, QuantizesToTheCanStep) {
  Noise noise(writeNoiseConfig("quant", 0.0, 1.0));

  eufs_msgs::msg::WheelSpeeds ws;
  ws.lb_speed = 123.4;
  ws.rb_speed = 123.6;
  const auto noisy = noise.applyNoiseToWheelSpeeds(ws);

  EXPECT_FLOAT_EQ(noisy.lb_speed, 123.0);
  EXPECT_FLOAT_EQ(noisy.rb_speed, 124.0);
}

TEST(WheelSpeedNoise, QuantumZeroPassesThrough) {
  Noise noise(writeNoiseConfig("noquant", 0.0, 0.0));

  eufs_msgs::msg::WheelSpeeds ws;
  ws.lb_speed = 123.456;
  const auto noisy = noise.applyNoiseToWheelSpeeds(ws);

  EXPECT_FLOAT_EQ(noisy.lb_speed, 123.456);
}

}  // namespace
