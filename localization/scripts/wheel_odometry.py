#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Wheel-encoder odometry for EUFS graph SLAM.

Integrates rear-wheel speeds (RPM) with the INS yaw rate into a drifting SE2
pose and publishes it as a CarState on ``/localization/wheel_odom`` — a
GNSS-independent odometry source for the graph SLAM motion input, so the
GNSS prior stays the only absolute channel (no correlated double injection).

Wiring — identical in the sim and on the car, which is the point:
  - ``/vehicle/wheel_speeds``     hyu_msgs/WheelSpeedsStamped, wheel speeds
    in RPM (all four are real: one AMK DD5 per wheel; this node uses the
    rears, which stay unsteered), steering in rad.
  - ``/sbg/ekf_rot_accel_body``   sbg_driver/SbgEkfRotAccel, the INS body-frame
    yaw rate + longitudinal acceleration, free of gravity and of the sensor
    biases the EKF has estimated. If it times out the node falls back to
    bicycle-model yaw from the steering angle (and slip compensation pauses).
  - ``/sbg/ekf_nav``              sbg_driver/SbgEkfNav, read ONLY for
    status.solution_mode. Below NAV_VELOCITY (mode 3) the EKF is free
    inertial: its attitude error ramps and g*sin(theta) leaks into the
    "gravity-free" acceleration. Consumed as truth, that leak reaches the
    CarState linear_acceleration, TMPC's LongAcc feedback integrates it, and
    the throttle winds up (2026-07-19 mode-2 runaway). So while
    solution_mode < min_rot_accel_solution_mode the rot_accel log is treated
    exactly like a stale one: encoder-differential yaw, slip compensation
    paused, acceleration zeroed. Unknown mode (nav never seen) gates nothing
    — a car whose nav log is off must not silently lose slip compensation.

This node used to read sensor_msgs/Imu on ``/imu/data``, which was a sim-only
fiction. The driver publishes the ROS standard messages only when
``output.ros_standard`` AND ``output.use_enu`` are both true, and our config
(sbg_device_uart_default.yaml) sets both to false — so ``/imu/data`` does not
exist on the car at all. Only gazebo was ever filling it. On the car this node
would have sat in the bicycle-model fallback forever, with slip compensation
silently off, and the warning blaming a "timeout" for a topic that was never
going to arrive.

Why rot_accel_body and not ``sbg/imu_data`` (which our config does publish):
the slip estimate below is driven by longitudinal acceleration, and
``imu_data.accel`` still has gravity in it. Braking pitches the car, and
g*sin(pitch) then leaks into the longitudinal axis — 0.5 m/s^2 at 3 deg —
biasing kappa in exactly the braking zones the compensation exists to correct.
rot_accel_body is the same signal with the EKF's gravity and bias estimates
already removed. The cost is that it needs ELLIPSE firmware new enough to know
the log (sbgECom >= 4.0); if it is missing, this node degrades to the steering
fallback rather than reporting anything wrong. Check ``ros2 topic hz`` on the
car before trusting the slip numbers.

Frame: the SBG body frame is X-forward/Y-right/Z-DOWN, so the reported yaw
rate is the NEGATIVE of the ROS FLU convention this node integrates in, while
the longitudinal axis needs no flip. Getting that sign wrong steers the
integrated heading backwards, which is why test_ins_yaw_rate_sign pins it.

Why the rear MEAN and not four wheels: the rear left/right split
(+/- half_track * yaw_rate) cancels exactly in the mean, while the fronts
carry a cos(steering) projection plus a lateral term. That is about STEERING,
not drive — with a motor at every corner there are no undriven wheels to
retreat to, so slip compensation is not a refinement here, it is the only
option available.

Slip: driven wheels over-read by kappa = peak_slip_ratio * a_x/(mu*g)
(saturating), which is a BIAS, not noise — integrating it uncorrected put
~0.076 m/m of drift through braking zones. The node estimates kappa from the
INS's low-passed longitudinal acceleration (the same traction-utilization
signal the tyre model uses), removes it from v, and widens its reported
sigma_v by a fraction of the correction so the graph downweights odometry
exactly when the estimate is working hardest.

KNOWN GAP, and it is the sim's side, not this node's: eufs_models'
getWheelSpeeds generates its slip from ``state.a_x``, which the vehicle model
fills with the body-frame velocity DERIVATIVE (v_x_dot = r_z*v_y + Fx/m), not
with the specific force an INS reports. So the sim's generator and this
estimator disagree by peak_slip_ratio * (r_z*v_y)/grip whenever the car is
both cornering and sliding — order 1 % of v in a hard corner. Fixing it means
subtracting the transport term in vehicle_model.cpp; until then, re-fit slip_*
from RTK parity on the car, not from sim parity in corners.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from hyu_msgs.msg import CarState, WheelSpeedsStamped

try:
    from sbg_driver.msg import SbgEkfNav, SbgEkfRotAccel
except ImportError as exc:  # pragma: no cover - depends on runtime environment
    raise SystemExit(
        "sbg_driver messages not found. Build/source the workspace that "
        "provides the sbg_driver package before running this node."
    ) from exc

RPM_TO_RAD_S = 2.0 * math.pi / 60.0


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        self.declare_parameter("wheel_speeds_topic", "/vehicle/wheel_speeds")
        self.declare_parameter("rot_accel_topic", "/sbg/ekf_rot_accel_body")
        self.declare_parameter("ekf_nav_topic", "/sbg/ekf_nav")
        # Below this solution_mode the EKF is free inertial and the rot_accel
        # log carries the gravity leak (module docstring); its consumption is
        # gated exactly like a stale log. 3 = NAV_VELOCITY.
        self.declare_parameter("min_rot_accel_solution_mode", 3)
        self.declare_parameter("output_topic", "/localization/wheel_odom")
        self.declare_parameter("wheel_radius", 0.2525)   # m (eufs configDry)
        self.declare_parameter("wheelbase", 1.58)        # m
        self.declare_parameter("track_width", 1.4)       # m (configDry axle_width)
        # Kinematic-vy arm: with a no-slip rear axle, a body origin this far
        # ahead of it sweeps sideways at w * arm (URDF midpoint would be
        # 0.79 m). DEFAULT 0 = disabled: an interleaved 3x3 trial on
        # small_track (2026-07-17) showed enabling it degrades SLAM ATE
        # (0.3->0.5/2.9/9.8) even though the geometry is right; until the
        # consumer-frame mismatch is understood, the honest model is vy=0.
        self.declare_parameter("rear_axle_to_base_m", 0.0)
        self.declare_parameter("use_ins_yaw_rate", True)
        self.declare_parameter("ins_timeout", 0.3)       # s -> steering fallback
        self.declare_parameter("max_dt", 0.5)            # s, reject stale steps
        # Per-sample noise the graph can turn into edge information later.
        # RE-MEASURED 2026-07-16 against the fidelity-pass sensor sim: the
        # random error on the rear mean is ~1 mm/s (0.05 RPM noise + 0.069 RPM
        # CAN quantum) and the 200 Hz gyro is ~7.4e-4 rad/s — the old
        # 0.05/0.02 were 50x/27x pessimistic and starved odometry of weight.
        # The slip BIAS is handled by compensation + dynamic widening below,
        # not by these static floors.
        self.declare_parameter("sigma_v", 0.01)          # m/s
        self.declare_parameter("sigma_w", 0.001)         # rad/s
        # Traction-slip compensation, mirroring eufs_models' ACHIEVED-dynamics
        # form: kappa = peak_slip_ratio * clamp(u, -1, 1) with
        # u = (a_x + C_drag*v^2/m) / (mu * (g + C_Down*v^2/m)) — the drag
        # share is real drive force at constant speed, and downforce grows
        # the grip. Defaults mirror robots/eufs/configDry.yaml; re-fit on the
        # real car from RTK parity.
        self.declare_parameter("slip_compensation", True)
        self.declare_parameter("slip_peak_ratio", 0.15)  # tyre peak slip ratio
        self.declare_parameter("slip_mu", 1.6)           # tyre D (configDry)
        self.declare_parameter("slip_g", 9.81)
        self.declare_parameter("slip_c_drag", 1.0)       # F_drag = C*v^2
        self.declare_parameter("slip_c_down", 1.9)       # F_down = C*v^2
        self.declare_parameter("slip_mass", 225.0)       # kg
        self.declare_parameter("slip_accel_lp_tau", 0.2)  # s, a_x low-pass
        # Fraction of the applied correction kept as extra sigma_v: the
        # estimate uses measured a_x against the model's commanded a, so
        # trust it, but not fully.
        self.declare_parameter("slip_residual_fraction", 0.3)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_footprint")

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.use_ins = bool(self.get_parameter("use_ins_yaw_rate").value)
        self.min_rot_accel_mode = int(
            self.get_parameter("min_rot_accel_solution_mode").value)
        self.ins_timeout = float(self.get_parameter("ins_timeout").value)
        self.max_dt = float(self.get_parameter("max_dt").value)
        self.sigma_v = float(self.get_parameter("sigma_v").value)
        self.sigma_w = float(self.get_parameter("sigma_w").value)
        self.slip_compensation = bool(self.get_parameter("slip_compensation").value)
        self.slip_peak_ratio = float(self.get_parameter("slip_peak_ratio").value)
        self.slip_mu = float(self.get_parameter("slip_mu").value)
        self.slip_g = float(self.get_parameter("slip_g").value)
        self.slip_c_drag = float(self.get_parameter("slip_c_drag").value)
        self.slip_c_down = float(self.get_parameter("slip_c_down").value)
        self.slip_mass = max(1.0, float(self.get_parameter("slip_mass").value))
        self.slip_lp_tau = float(self.get_parameter("slip_accel_lp_tau").value)
        self.slip_residual_fraction = float(
            self.get_parameter("slip_residual_fraction").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.rear_axle_to_base = float(
            self.get_parameter("rear_axle_to_base_m").value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_t = None
        self.last_v = 0.0
        self.last_w = 0.0
        self.ins_yaw_rate = None
        self.ins_accel_flu = None
        self.ins_stamp = None
        # Last reported solution_mode; None until ekf_nav is seen. Held (not
        # aged out) between nav messages: if the whole driver dies, rot_accel
        # staleness gates everything anyway, and if only the nav log is off
        # the mode was never going to update — fail open in both cases.
        self.nav_mode = None
        self.used_ins_fallback = False
        self.accel_x_lp = 0.0
        self.accel_lp_stamp = None
        self.warned_frame = False

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(
            CarState, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            WheelSpeedsStamped,
            str(self.get_parameter("wheel_speeds_topic").value),
            self.on_wheel_speeds,
            sensor_qos,
        )
        if self.use_ins:
            self.create_subscription(
                SbgEkfRotAccel,
                str(self.get_parameter("rot_accel_topic").value),
                self.on_rot_accel,
                sensor_qos,
            )
            self.create_subscription(
                SbgEkfNav,
                str(self.get_parameter("ekf_nav_topic").value),
                self.on_ekf_nav,
                sensor_qos,
            )

        self.get_logger().info(
            "wheel odometry: %s + %s -> %s (r=%.4f m, L=%.2f m)" % (
                self.get_parameter("wheel_speeds_topic").value,
                self.get_parameter("rot_accel_topic").value
                if self.use_ins else "steering yaw",
                self.get_parameter("output_topic").value,
                self.wheel_radius,
                self.wheelbase,
            )
        )

    def on_ekf_nav(self, msg):
        mode = int(msg.status.solution_mode)
        if self.nav_mode is not None and self.nav_mode != mode:
            if mode < self.min_rot_accel_mode <= self.nav_mode:
                self.get_logger().warn(
                    "solution_mode %d < %d: EKF is free inertial, gating "
                    "rot_accel consumption (encoder yaw, slip compensation "
                    "paused, acceleration zeroed)" % (
                        mode, self.min_rot_accel_mode)
                )
            elif self.nav_mode < self.min_rot_accel_mode <= mode:
                self.get_logger().info(
                    "solution_mode %d: rot_accel consumption restored" % mode)
        self.nav_mode = mode

    def _ins_ok(self, t):
        """rot_accel is consumable: fresh AND not free-inertial (if known)."""
        return (
            self.ins_stamp is not None
            and abs(t - self.ins_stamp) <= self.ins_timeout
            and not (
                self.nav_mode is not None
                and self.nav_mode < self.min_rot_accel_mode
            )
        )

    def on_rot_accel(self, msg):
        self._check_frame(msg.header.frame_id)
        # SBG body frame is X-forward/Y-right/Z-DOWN; this node integrates in
        # ROS FLU, so the yaw rate flips sign. X is forward in both, so the
        # longitudinal acceleration passes through untouched.
        self.ins_yaw_rate = -msg.rate.z
        # Raw (unfiltered) gravity-free body acceleration, NED body -> FLU:
        # x passes, y flips. Passed through on the CarState for downstream
        # consumers (TMPC vehicle state); the low-passed copy below stays
        # dedicated to the slip estimate.
        self.ins_accel_flu = (msg.acceleration.x, -msg.acceleration.y)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Low-pass a_x for the slip estimate: per-sample accel noise would
        # otherwise jitter kappa; tau ~0.2 s tracks real traction transients.
        if self.accel_lp_stamp is not None and stamp > self.accel_lp_stamp:
            dt = stamp - self.accel_lp_stamp
            alpha = dt / (self.slip_lp_tau + dt)
            self.accel_x_lp += alpha * (msg.acceleration.x - self.accel_x_lp)
        elif self.accel_lp_stamp is None:
            # First sample initializes the filter; a duplicated/out-of-order
            # stamp must NOT reset it to a raw sample (one unfiltered spike
            # goes straight into the slip correction otherwise).
            self.accel_x_lp = msg.acceleration.x
        self.accel_lp_stamp = max(self.accel_lp_stamp or stamp, stamp)
        self.ins_stamp = stamp

    def _check_frame(self, frame_id):
        """
        Warn once if the driver looks like it is in ENU mode.

        sbg_driver 3.3.2 applies its NED->ENU *navigation*-frame swap to this
        *body*-frame log, so with output.use_enu:=true the message carries
        longitudinal and lateral EXCHANGED and this node would integrate
        lateral acceleration as traction utilization. The frame_id is the only
        thing on the wire that distinguishes the two, and it is a free-form
        config string, so this is a smoke alarm rather than a guarantee.
        """
        if self.warned_frame or not frame_id:
            return
        if not frame_id.endswith("_ned"):
            self.get_logger().warn(
                "%s reports frame_id '%s': expected an NED body frame. If the "
                "driver is running output.use_enu:=true, its rot_accel body "
                "axes are swapped and the slip estimate is reading lateral "
                "acceleration. Prefer use_enu:=false." % (
                    self.get_parameter("rot_accel_topic").value, frame_id)
            )
        self.warned_frame = True

    def yaw_rate(self, t, v, steering, diff_w=None):
        """INS yaw rate when fresh, else rear-encoder differential, else bicycle."""
        if self.use_ins and self.ins_yaw_rate is not None and self._ins_ok(t):
            if self.used_ins_fallback:
                self.get_logger().info("INS yaw rate restored")
                self.used_ins_fallback = False
            return self.ins_yaw_rate

        if self.use_ins and not self.used_ins_fallback:
            mode_gated = (
                self.nav_mode is not None
                and self.nav_mode < self.min_rot_accel_mode
            )
            if not mode_gated:
                # On the car, "never arrived" is as likely as "stopped
                # arriving": the log needs sbgECom >= 4.0 firmware and
                # log_ekf_rot_accel_body switched on in the driver config.
                # (The mode-gated case already announced itself in
                # on_ekf_nav; repeating a firmware hint there would mislead.)
                self.get_logger().warn(
                    "%s yaw rate unavailable; falling back to rear-encoder "
                    "differential yaw (slip compensation paused). If it never "
                    "arrives, check the driver's log_ekf_rot_accel_body and "
                    "the ELLIPSE firmware." % (
                        self.get_parameter("rot_accel_topic").value)
                )
            self.used_ins_fallback = True
        # Every wheel has its own AMK encoder, so the rear pair's speed
        # difference observes yaw directly — unlike the bicycle model it needs
        # no steering geometry and stays honest under understeer. The bicycle
        # model remains only as the last resort for a dead rear encoder.
        if diff_w is not None:
            return diff_w
        return v * math.tan(steering) / self.wheelbase

    def on_wheel_speeds(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        # Validate BEFORE committing last_t: rewinding it on a rejected
        # out-of-order sample makes the NEXT sample integrate an inflated dt
        # (v*dt of phantom travel in one publish — with interleaved stamp
        # sources this repeats and multiplies total distance).
        if dt <= 0.0:
            return
        self.last_t = t
        if dt > self.max_dt:
            # Resync after a gap: skip integration for this interval.
            return

        # Rear axle mean; the simulator (and real ros_can) report RPM.
        # The differential split cancels in the mean; common-mode slip does not.
        rear_rpm = 0.5 * (msg.speeds.lb_speed + msg.speeds.rb_speed)
        v = rear_rpm * RPM_TO_RAD_S * self.wheel_radius
        # Per-wheel encoders: the rear pair's split observes yaw rate without
        # any steering geometry (fallback tier between INS and bicycle model).
        diff_w = (
            (msg.speeds.rb_speed - msg.speeds.lb_speed)
            * RPM_TO_RAD_S * self.wheel_radius / self.track_width
            if self.track_width > 1e-3 else None
        )

        slip_correction = 0.0
        if self.slip_compensation and self._ins_ok(t):
            v_sq = v * v
            drive_accel = self.accel_x_lp + self.slip_c_drag * v_sq / self.slip_mass
            grip_accel = self.slip_mu * (
                self.slip_g + self.slip_c_down * v_sq / self.slip_mass)
            utilization = max(-1.0, min(1.0, drive_accel / grip_accel))
            kappa = self.slip_peak_ratio * utilization
            # v_meas = v_true + kappa * max(1, v_true); one fixed-point step
            # (kappa <= 0.15 makes the second iteration sub-mm/s).
            slip_correction = kappa * max(1.0, abs(v - kappa * max(1.0, abs(v))))
            v -= slip_correction

        w = self.yaw_rate(t, v, msg.speeds.steering, diff_w)

        # Kinematic lateral velocity of the body origin: with a no-slip rear
        # axle, a point rear_axle_to_base ahead of it sweeps sideways at
        # w * arm. This is the dominant vy term the twist consumers (SLAM
        # motion, lidar deskew, TMPC state) need in corners; true tyre-slip
        # vy is unobservable from wheel spin and stays unmodelled.
        vy = w * self.rear_axle_to_base

        # Midpoint heading: advancing with the start-of-interval yaw lags
        # truth by 0.5*w*dt every step, a systematic outward bias on every
        # arc that the graph cannot average out (shows up as track-width
        # error in the map).
        mid_yaw = self.yaw + 0.5 * w * dt
        cos_y = math.cos(mid_yaw)
        sin_y = math.sin(mid_yaw)
        self.x += (v * cos_y - vy * sin_y) * dt
        self.y += (v * sin_y + vy * cos_y) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
        self.last_v = v
        self.last_w = w

        out = CarState()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        out.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        out.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        # Per-sample noise levels; consumers derive relative-motion
        # information from these diagonals (same contract as the SBG bridge).
        # sigma_v widens by a fraction of any slip correction applied — the
        # graph consumes this (use_odom_covariance) and downweights odometry
        # exactly in the traction regimes where the estimate works hardest.
        sigma_v_eff = self.sigma_v + \
            self.slip_residual_fraction * abs(slip_correction)
        out.pose.covariance[0] = sigma_v_eff * sigma_v_eff
        out.pose.covariance[7] = sigma_v_eff * sigma_v_eff
        out.pose.covariance[35] = self.sigma_w * self.sigma_w
        out.twist.twist.linear.x = v
        out.twist.twist.linear.y = vy
        out.twist.twist.angular.z = w
        # Body acceleration from the INS log when fresh AND trustworthy;
        # zeros in fallback, matching the "sensor absent" semantics of the
        # yaw-rate fallback. The mode gate matters most right here: this
        # field feeds TMPC's LongAcc feedback, and a free-inertial gravity
        # leak passed through it winds the throttle up (mode-2 runaway).
        if self.ins_accel_flu is not None and self._ins_ok(t):
            out.linear_acceleration.x = self.ins_accel_flu[0]
            out.linear_acceleration.y = self.ins_accel_flu[1]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
