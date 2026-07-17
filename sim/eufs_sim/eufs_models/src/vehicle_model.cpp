#include "eufs_models/vehicle_model.hpp"

#include <algorithm>
#include <cmath>

namespace eufs {
namespace models {

VehicleModel::VehicleModel(const std::string &yaml_file) : _param(yaml_file) {}

void VehicleModel::validateState(State &state) { state.v_x = std::max(0.0, state.v_x); }

void VehicleModel::validateInput(Input &input) {
  double max_acc = _param.input_ranges.acc.max;
  double min_acc = _param.input_ranges.acc.min;

  double max_vel = _param.input_ranges.vel.max;
  double min_vel = _param.input_ranges.vel.min;

  double max_delta = _param.input_ranges.delta.max;
  double min_delta = _param.input_ranges.delta.min;

  input.acc = std::fmin(std::fmax(input.acc, min_acc), max_acc);
  input.vel = std::fmin(std::fmax(input.vel, min_vel), max_vel);
  input.delta = std::fmin(std::fmax(input.delta, min_delta), max_delta);
}

double VehicleModel::getSlipAngle(const State &x, const Input &u, bool is_front) {
  double lever_arm_length = _param.kinematic.l * _param.kinematic.w_front;

  if (!is_front) {
    double v_x = std::max(1.0, x.v_x);
    return std::atan((x.v_y - lever_arm_length * x.r_z) /
                     (v_x - 0.5 * _param.kinematic.axle_width * x.r_z));
  }

  double v_x = std::max(1.0, x.v_x);
  return std::atan((x.v_y + lever_arm_length * x.r_z) /
                   (v_x - 0.5 * _param.kinematic.axle_width * x.r_z)) -
         u.delta;
}

hyu_msgs::msg::WheelSpeeds VehicleModel::getWheelSpeeds(const State &state, const Input &input) {
  float PI = 3.14159265;
  float wheel_circumference = 2 * PI * _param.tire.radius;

  hyu_msgs::msg::WheelSpeeds wheel_speeds;

  wheel_speeds.steering = input.delta;

  // All four wheels are real: the car is 4WD with one AMK DD5 per wheel, so
  // every corner reports the same encoder chain (18-bit shaft encoder ->
  // inverter AMK_ActualVelocity, int16 integer motor rpm per PDK_205481 ->
  // / gear ratio; quantization lives in noise.yaml). The 999 front
  // placeholder was the ADS-DV (2WD feed) contract and is gone with it.
  //
  // Each wheel reads its own contact-patch velocity, projected onto the
  // wheel's rolling direction: in a corner the outer side runs
  // axle_width * r_z faster (positive yaw rate = left turn = right outer),
  // and the steered fronts additionally see the front axle's lateral
  // velocity through sin(delta).
  double half_track_speed = 0.5 * _param.kinematic.axle_width * state.r_z;
  double front_lateral = state.v_y + _param.kinematic.l_F * state.r_z;
  double cos_delta = std::cos(input.delta);
  double sin_delta = std::sin(input.delta);

  // Longitudinal slip: encoders over-read when drive torque approaches the
  // grip, under-read when braking. Below the tyre's peak, kappa(utilization)
  // is near-linear: kappa = peak_slip_ratio * u.
  //
  // u comes from the ACHIEVED dynamics (state.a_x, which both models fill),
  // not from input.acc. The command is the wrong signal twice over: it does
  // not contain the force holding off drag (so slip at constant speed would
  // read zero, and braking slip would overstate by the drag share), and in
  // velocity command mode the plugin synthesizes acc = (v_des - v)/dt per
  // physics step - a deadbeat that saturates through transients and spikes
  // for one step at every /cmd setpoint jump. The achieved tyre force is
  // m*a_x plus the drag share, drawn from a grip that grows with downforce
  // (mu = Pacejka D). Known approximation: the steering-projection term
  // (sin(delta)*FyF) of the drive force is not visible from here and is
  // omitted - small-angle, understates slip in hard combined corners. This
  // mirrors wheel_odometry.py's compensation, which estimates kappa from
  // the measured (IMU) a_x.
  //
  // With torque going to all four wheels, the baseline assumption is even
  // utilization across the car (what torque vectoring aims at), so one
  // kappa applies to every wheel; per-wheel spin-up needs a wheel-spin
  // state these models do not carry, and kappa saturates at the peak
  // instead of diverging. The max(1, v_x) floor mirrors getSlipAngle above.
  double slip_speed = 0.0;
  double mu = _param.tire.D;
  double gravity = _param.inertia.g;
  double mass = _param.inertia.m;
  if (_param.tire.peak_slip_ratio > 0.0 && mu > 0.0 && gravity > 0.0 && mass > 0.0) {
    double v_sq = state.v_x * state.v_x;
    double drive_accel = state.a_x + _param.aero.c_drag * v_sq / mass;
    double grip_accel = mu * (gravity + _param.aero.c_down * v_sq / mass);
    double utilization = drive_accel / grip_accel;
    utilization = std::min(std::max(utilization, -1.0), 1.0);
    slip_speed = _param.tire.peak_slip_ratio * utilization * std::max(1.0, state.v_x);
  }

  wheel_speeds.lf_speed =
      (((state.v_x - half_track_speed) * cos_delta + front_lateral * sin_delta + slip_speed) /
       wheel_circumference) *
      60;
  wheel_speeds.rf_speed =
      (((state.v_x + half_track_speed) * cos_delta + front_lateral * sin_delta + slip_speed) /
       wheel_circumference) *
      60;
  wheel_speeds.lb_speed =
      ((state.v_x - half_track_speed + slip_speed) / wheel_circumference) * 60;
  wheel_speeds.rb_speed =
      ((state.v_x + half_track_speed + slip_speed) / wheel_circumference) * 60;

  return wheel_speeds;
}
}  // namespace models
}  // namespace eufs
