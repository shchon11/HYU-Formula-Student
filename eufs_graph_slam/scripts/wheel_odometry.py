#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Wheel-encoder odometry for EUFS graph SLAM.

Integrates rear-wheel speeds (RPM) with the INS yaw rate into a drifting SE2
pose and publishes it as a CarState on ``/wheel_odometry/car_state`` — a
GNSS-independent odometry source for the graph SLAM motion input, so the
GNSS prior stays the only absolute channel (no correlated double injection).

Wiring — identical in the sim and on the car, which is the point:
  - ``/ros_can/wheel_speeds``     eufs_msgs/WheelSpeedsStamped, wheel speeds
    in RPM (all four are real: one AMK DD5 per wheel; this node uses the
    rears, which stay unsteered), steering in rad.
  - ``/sbg/ekf_rot_accel_body``   sbg_driver/SbgEkfRotAccel, the INS body-frame
    yaw rate + longitudinal acceleration, free of gravity and of the sensor
    biases the EKF has estimated. If it times out the node falls back to
    bicycle-model yaw from the steering angle (and slip compensation pauses).

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

from eufs_msgs.msg import CarState, WheelSpeedsStamped

try:
    from sbg_driver.msg import SbgEkfRotAccel
except ImportError as exc:  # pragma: no cover - depends on runtime environment
    raise SystemExit(
        "sbg_driver messages not found. Build/source the workspace that "
        "provides the sbg_driver package before running this node."
    ) from exc

RPM_TO_RAD_S = 2.0 * math.pi / 60.0


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        self.declare_parameter("wheel_speeds_topic", "/ros_can/wheel_speeds")
        self.declare_parameter("rot_accel_topic", "/sbg/ekf_rot_accel_body")
        self.declare_parameter("output_topic", "/wheel_odometry/car_state")
        self.declare_parameter("wheel_radius", 0.2525)   # m (eufs configDry)
        self.declare_parameter("wheelbase", 1.58)        # m
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

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_t = None
        self.last_v = 0.0
        self.last_w = 0.0
        self.ins_yaw_rate = None
        self.ins_stamp = None
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

    def on_rot_accel(self, msg):
        self._check_frame(msg.header.frame_id)
        # SBG body frame is X-forward/Y-right/Z-DOWN; this node integrates in
        # ROS FLU, so the yaw rate flips sign. X is forward in both, so the
        # longitudinal acceleration passes through untouched.
        self.ins_yaw_rate = -msg.rate.z
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Low-pass a_x for the slip estimate: per-sample accel noise would
        # otherwise jitter kappa; tau ~0.2 s tracks real traction transients.
        if self.accel_lp_stamp is not None and stamp > self.accel_lp_stamp:
            dt = stamp - self.accel_lp_stamp
            alpha = dt / (self.slip_lp_tau + dt)
            self.accel_x_lp += alpha * (msg.acceleration.x - self.accel_x_lp)
        else:
            self.accel_x_lp = msg.acceleration.x
        self.accel_lp_stamp = stamp
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

    def yaw_rate(self, t, v, steering):
        """INS yaw rate when fresh, else kinematic bicycle fallback."""
        if (
            self.use_ins
            and self.ins_yaw_rate is not None
            and self.ins_stamp is not None
            and abs(t - self.ins_stamp) <= self.ins_timeout
        ):
            if self.used_ins_fallback:
                self.get_logger().info("INS yaw rate restored")
                self.used_ins_fallback = False
            return self.ins_yaw_rate

        if self.use_ins and not self.used_ins_fallback:
            # On the car, "never arrived" is as likely as "stopped arriving":
            # the log needs sbgECom >= 4.0 firmware and log_ekf_rot_accel_body
            # switched on in the driver config.
            self.get_logger().warn(
                "%s yaw rate unavailable; falling back to bicycle-model yaw "
                "(slip compensation paused). If it never arrives, check the "
                "driver's log_ekf_rot_accel_body and the ELLIPSE firmware." % (
                    self.get_parameter("rot_accel_topic").value)
            )
            self.used_ins_fallback = True
        return v * math.tan(steering) / self.wheelbase

    def on_wheel_speeds(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > self.max_dt:
            return

        # Rear axle mean; the simulator (and real ros_can) report RPM.
        # The differential split cancels in the mean; common-mode slip does not.
        rear_rpm = 0.5 * (msg.speeds.lb_speed + msg.speeds.rb_speed)
        v = rear_rpm * RPM_TO_RAD_S * self.wheel_radius

        slip_correction = 0.0
        if (
            self.slip_compensation
            and self.ins_stamp is not None
            and abs(t - self.ins_stamp) <= self.ins_timeout
        ):
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

        w = self.yaw_rate(t, v, msg.speeds.steering)

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
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
        out.twist.twist.angular.z = w
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
