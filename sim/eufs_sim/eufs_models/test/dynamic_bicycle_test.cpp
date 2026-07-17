// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "eufs_models/dynamic_bicycle.hpp"

#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

namespace {

using eufs::models::DynamicBicycle;
using eufs::models::Input;
using eufs::models::State;

constexpr double kMass = 225.0;
constexpr double kGravity = 9.81;
constexpr double kWheelbase = 1.53;
constexpr double kHCg = 0.30;
constexpr double kWFront = 0.5;

/// A config on disk, because Param only knows how to come from a yaml file.
std::string writeConfig(const std::string &name, bool load_transfer_to_tires) {
  const std::string path = std::string(std::tmpnam(nullptr)) + name + ".yaml";
  std::ofstream out(path);
  out << "inertia:\n  m: " << kMass << "\n  g: " << kGravity << "\n  I_z: 31.27\n"
      << "kinematics:\n  l: " << kWheelbase << "\n  b_F: 0.765\n  b_R: 0.765\n"
      << "  w_front: " << kWFront << "\n  axle_width: 1.2\n"
      << "tire:\n  tire_coefficient: 1.0\n  B: 12.56\n  C: -1.38\n  D: 0.8\n  E: -0.58\n"
      << "  radius: 0.2525\n"
      << "aero:\n  C_Down: 0.0\n  C_drag: 1.0\n"
      << "suspension:\n  h_cg: " << kHCg << "\n  k_roll: 30000.0\n  k_pitch: 40000.0\n"
      << "  natural_freq_hz: 2.5\n  damping_ratio: 0.5\n"
      << "  load_transfer_to_tires: " << (load_transfer_to_tires ? "true" : "false") << "\n"
      << "input_ranges:\n  acceleration:\n    max: 5.2\n    min: -5.2\n"
      << "  velocity:\n    max: 30\n    min: 0\n  steering:\n    max: 0.42\n    min: -0.42\n";
  return path;
}

State rolling(double v_x) {
  State state{};
  state.v_x = v_x;
  return state;
}

Input command(double acc) {
  Input input{};
  input.acc = acc;
  input.vel = 0.0;
  input.delta = 0.0;
  return input;
}

// The default: a car whose tyres never notice it is braking. Kept because the
// controllers are tuned against it, so it has to stay bit-for-bit what it was.
TEST(DynamicBicycleLoadTransfer, OffIsAlwaysTheStaticSplit) {
  DynamicBicycle model(writeConfig("off", false));

  EXPECT_DOUBLE_EQ(model.getDynamicWeightFront(rolling(10.0), command(0.0)), kWFront);
  EXPECT_DOUBLE_EQ(model.getDynamicWeightFront(rolling(10.0), command(-5.0)), kWFront);
  EXPECT_DOUBLE_EQ(model.getDynamicWeightFront(rolling(10.0), command(5.0)), kWFront);
}

TEST(DynamicBicycleLoadTransfer, BrakingBuysFrontGripAndAccelerationSellsIt) {
  DynamicBicycle model(writeConfig("on", true));

  const double coasting = model.getDynamicWeightFront(rolling(10.0), command(0.0));
  const double braking = model.getDynamicWeightFront(rolling(10.0), command(-4.0));
  const double accelerating = model.getDynamicWeightFront(rolling(10.0), command(4.0));

  EXPECT_GT(braking, coasting);
  EXPECT_LT(accelerating, coasting);

  // The magnitude is not a free parameter: it is m*a_x*h_cg/L as a share of
  // weight, i.e. a_x*h_cg/(g*L). Drag makes the true a_x a little more negative
  // than the command, so check against the model's own Fx via the coasting
  // offset rather than restating the drag here.
  const double expected_shift = 4.0 * kHCg / (kGravity * kWheelbase);
  EXPECT_NEAR(braking - coasting, expected_shift, 1e-6);
  EXPECT_NEAR(coasting - accelerating, expected_shift, 1e-6);
}

// A tyre with no load on it makes no lateral force, and a bicycle model has no
// business simulating a wheelie.
TEST(DynamicBicycleLoadTransfer, NeverUnloadsAnAxleCompletely) {
  DynamicBicycle model(writeConfig("clamp", true));

  const double absurd_brake = model.getDynamicWeightFront(rolling(10.0), command(-1000.0));
  const double absurd_accel = model.getDynamicWeightFront(rolling(10.0), command(1000.0));

  EXPECT_LE(absurd_brake, 0.95);
  EXPECT_GE(absurd_brake, 0.05);
  EXPECT_LE(absurd_accel, 0.95);
  EXPECT_GE(absurd_accel, 0.05);
}

// Turning the coupling on must change how the car drives -- otherwise the flag
// is decoration and the whole feature is untested.
TEST(DynamicBicycleLoadTransfer, CouplingChangesTheYawResponseUnderBraking) {
  DynamicBicycle uncoupled(writeConfig("uncoupled", false));
  DynamicBicycle coupled(writeConfig("coupled", true));

  auto brake_and_steer = [](DynamicBicycle &model) {
    State state{};
    state.v_x = 12.0;
    Input input{};
    input.acc = -4.0;
    input.vel = 0.0;
    input.delta = 0.2;
    for (int i = 0; i < 200; ++i) {
      model.updateState(state, input, 0.001);
    }
    return state.r_z;
  };

  const double without = brake_and_steer(uncoupled);
  const double with = brake_and_steer(coupled);

  // Braking loads the front axle, the front tyres hold more, and the car turns
  // in harder for the same steering.
  EXPECT_GT(std::abs(with), std::abs(without));
}

}  // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
