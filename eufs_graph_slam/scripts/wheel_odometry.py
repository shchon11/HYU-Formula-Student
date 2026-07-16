#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Wheel-encoder odometry for EUFS graph SLAM.

Integrates rear-wheel speeds (RPM) with an IMU yaw rate into a drifting SE2
pose and publishes it as a CarState on ``/wheel_odometry/car_state`` — a
GNSS-independent odometry source for the graph SLAM motion input, so the
GNSS prior stays the only absolute channel (no correlated double injection).

Simulator wiring (default parameters):
  - ``/ros_can/wheel_speeds``  eufs_msgs/WheelSpeedsStamped, wheel speeds in
    RPM (all four are real on the 4WD car; this node uses the rears, which
    stay unsteered), steering in rad.
  - ``/imu/data``              sensor_msgs/Imu yaw rate + longitudinal accel.
    If the IMU times out the node falls back to bicycle-model yaw from the
    steering angle (and slip compensation pauses).

Why the rear MEAN and not four wheels: the rear differential split
(+/- half_track * yaw_rate) cancels exactly in the mean, while the fronts
carry a cos(steering) projection plus a lateral term. Preferring undriven
wheels to dodge traction slip is a real-car strategy only — the sim applies
one common-mode slip term to all four wheels, so it cannot be validated here.

Slip: driven wheels over-read by kappa = peak_slip_ratio * a_x/(mu*g)
(saturating), which is a BIAS, not noise — integrating it uncorrected put
~0.076 m/m of drift through braking zones. The node estimates kappa from the
IMU's low-passed longitudinal acceleration (the same traction-utilization
signal the tyre model uses), removes it from v, and widens its reported
sigma_v by a fraction of the correction so the graph downweights odometry
exactly when the estimate is working hardest.

Real-car wiring: point ``wheel_speeds_topic``/``imu_topic`` at the ros_can
encoder feed and the SBG IMU; the message contracts are identical, so no code
change is needed. Re-fit slip_* parameters from RTK parity once real logs
exist.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from eufs_msgs.msg import CarState, WheelSpeedsStamped
from sensor_msgs.msg import Imu

RPM_TO_RAD_S = 2.0 * math.pi / 60.0


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        self.declare_parameter("wheel_speeds_topic", "/ros_can/wheel_speeds")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("output_topic", "/wheel_odometry/car_state")
        self.declare_parameter("wheel_radius", 0.2525)   # m (eufs configDry)
        self.declare_parameter("wheelbase", 1.58)        # m
        self.declare_parameter("use_imu_yaw_rate", True)
        self.declare_parameter("imu_timeout", 0.3)       # s -> steering fallback
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
        # Traction-slip compensation (mirrors eufs_models: slip_speed =
        # peak_slip_ratio * clamp(a_x/(mu*g), -1, 1) * max(1, v)).
        self.declare_parameter("slip_compensation", True)
        self.declare_parameter("slip_peak_ratio", 0.15)  # tyre peak slip ratio
        self.declare_parameter("slip_mu", 1.6)           # tyre D (configDry)
        self.declare_parameter("slip_g", 9.81)
        self.declare_parameter("slip_accel_lp_tau", 0.2)  # s, a_x low-pass
        # Fraction of the applied correction kept as extra sigma_v: the
        # estimate uses measured a_x against the model's commanded a, so
        # trust it, but not fully.
        self.declare_parameter("slip_residual_fraction", 0.3)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_footprint")

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.use_imu = bool(self.get_parameter("use_imu_yaw_rate").value)
        self.imu_timeout = float(self.get_parameter("imu_timeout").value)
        self.max_dt = float(self.get_parameter("max_dt").value)
        self.sigma_v = float(self.get_parameter("sigma_v").value)
        self.sigma_w = float(self.get_parameter("sigma_w").value)
        self.slip_compensation = bool(self.get_parameter("slip_compensation").value)
        self.slip_peak_ratio = float(self.get_parameter("slip_peak_ratio").value)
        self.slip_mu_g = (float(self.get_parameter("slip_mu").value)
                          * float(self.get_parameter("slip_g").value))
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
        self.imu_yaw_rate = None
        self.imu_stamp = None
        self.used_imu_fallback = False
        self.accel_x_lp = 0.0
        self.accel_lp_stamp = None

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(
            CarState, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            WheelSpeedsStamped,
            str(self.get_parameter("wheel_speeds_topic").value),
            self.on_wheel_speeds,
            sensor_qos,
        )
        if self.use_imu:
            self.create_subscription(
                Imu,
                str(self.get_parameter("imu_topic").value),
                self.on_imu,
                sensor_qos,
            )

        self.get_logger().info(
            "wheel odometry: %s + %s -> %s (r=%.4f m, L=%.2f m)" % (
                self.get_parameter("wheel_speeds_topic").value,
                self.get_parameter("imu_topic").value if self.use_imu else "steering yaw",
                self.get_parameter("output_topic").value,
                self.wheel_radius,
                self.wheelbase,
            )
        )

    def on_imu(self, msg):
        self.imu_yaw_rate = msg.angular_velocity.z
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Low-pass a_x for the slip estimate: per-sample accel noise would
        # otherwise jitter kappa; tau ~0.2 s tracks real traction transients.
        if self.accel_lp_stamp is not None and stamp > self.accel_lp_stamp:
            dt = stamp - self.accel_lp_stamp
            alpha = dt / (self.slip_lp_tau + dt)
            self.accel_x_lp += alpha * (msg.linear_acceleration.x - self.accel_x_lp)
        else:
            self.accel_x_lp = msg.linear_acceleration.x
        self.accel_lp_stamp = stamp
        self.imu_stamp = stamp

    def yaw_rate(self, t, v, steering):
        """IMU yaw rate when fresh, else kinematic bicycle fallback."""
        if (
            self.use_imu
            and self.imu_yaw_rate is not None
            and self.imu_stamp is not None
            and abs(t - self.imu_stamp) <= self.imu_timeout
        ):
            if self.used_imu_fallback:
                self.get_logger().info("IMU yaw rate restored")
                self.used_imu_fallback = False
            return self.imu_yaw_rate

        if self.use_imu and not self.used_imu_fallback:
            self.get_logger().warn(
                "IMU yaw rate unavailable; falling back to bicycle-model yaw")
            self.used_imu_fallback = True
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
            and self.imu_stamp is not None
            and abs(t - self.imu_stamp) <= self.imu_timeout
        ):
            utilization = max(-1.0, min(1.0, self.accel_x_lp / self.slip_mu_g))
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
