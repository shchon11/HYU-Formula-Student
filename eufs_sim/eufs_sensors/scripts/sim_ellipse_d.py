#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Simulated SBG Ellipse-D GNSS/INS for the EUFS simulator.

Consumes the simulator ground truth (``/ground_truth/state``) and re-emits it
as the SBG driver's EKF outputs (``/sbg/ekf_nav`` + ``/sbg/ekf_euler``), so the
real-hardware pipeline (sbg_odometry_bridge -> graph SLAM GNSS prior) runs
unmodified against the simulator.

Error model constants come from the Ellipse-N/D datasheet (MK068EN v1.1,
1-sigma, land application):

  horizontal position   RTK 0.01 m | single point 1.2 m (RTK float ~0.3 m typ.)
  velocity              RTK 0.03 m/s | single point 0.05 m/s
  heading (dual ant.)   0.2 deg for baseline > 2 m; 0.5 deg for > 0.5 m —
                        tune ``sigma_heading`` to the car's actual baseline
  roll/pitch            0.05 deg (INS); degrades to 0.1-0.4 deg in AHRS mode
  gyro in-run bias      7 deg/h, angular random walk 0.18 deg/sqrt(h)
  accel in-run bias     14 ug (negligible next to gravity leakage)
  internal GNSS update  5 Hz (EKF/INS output itself runs much faster)
  long GNSS outage      0.5 % of travelled distance (land, WITH odometer
                        aiding, spec'd up to 10 min)

Two independent degradation axes are modelled:

1. ``solution_mode`` (SbgEkfStatus ladder) with error-state propagation:
     mode 4 NAV_POSITION  errors decay with ``reacquire_tau`` -> truth + tier
                          noise. Re-entry reproduces the re-acquisition pull-in.
     mode 3 NAV_VELOCITY  velocity stays aided; position integrates a small
                          residual velocity bias + random walk.
     mode <=2             free-inertial: the dominant error is gravity leakage
                          through the attitude error, a = g*sin(theta), with
                          theta ramping at the gyro in-run bias (7 deg/h) from
                          an initial 0.05 deg -> position diverges ~t^2..t^3
                          (~0.3 m @ 8 s, ~4 m @ 30 s). The 14 ug accel bias is
                          negligible next to this. With ``odometer_aided:=true``
                          (future wheel encoder) drift is instead the
                          datasheet's 0.5 % of travelled distance.
     mode <=1             heading additionally ramps at the gyro bias.

2. ``correction_type`` within mode 4 — the most common real degradation is RTK
   falling back to float/single while solution_mode STAYS 4. Position sigma
   switches 0.01 / 0.3 / 1.2 m (first-order Gauss-Markov error, tau
   ``gnss_error_tau``, so single-point error wanders slowly instead of white
   jitter). The bridge forwards the reported accuracy, and the SLAM prior gate
   (gnss_prior_max_position_sigma, default 0.5 m) then drops single-point
   anchors automatically.

Mode control (topics override and disable the schedules):
  * ``mode_schedule``, e.g. ``"20:3,30:4"`` (t_sec:mode, relative to first GT)
  * ``correction_schedule``, e.g. ``"40:single,50:rtk_fixed"``
  * ``/sim_ins/set_solution_mode``  (std_msgs/UInt8)
  * ``/sim_ins/set_correction_type`` (std_msgs/String: rtk_fixed|rtk_float|single)

Geodesy: local ENU ground truth is inverse-projected about a configurable datum
(default: Hanyang University) with the same WGS84 tangent-plane model the
bridge uses. Output convention is NED by default (driver ``use_enu: false``).
Run with ``use_sim_time:=true`` alongside the simulator; all internal timing is
driven by ground-truth message stamps.
"""

import math
import random
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from eufs_msgs.msg import CarState
from std_msgs.msg import String, UInt8

try:
    from sbg_driver.msg import SbgEkfEuler, SbgEkfNav, SbgEkfStatus
except ImportError as exc:  # pragma: no cover - depends on runtime environment
    raise SystemExit(
        "sbg_driver messages not found. Build/source the workspace that "
        "provides the sbg_driver package before running this node."
    ) from exc

# WGS84 ellipsoid (must match sbg_odometry_bridge so projections round-trip).
_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3
_GRAVITY = 9.80665

_MODE_NAMES = {
    0: "UNINITIALIZED",
    1: "VERTICAL_GYRO",
    2: "AHRS",
    3: "NAV_VELOCITY",
    4: "NAV_POSITION",
}
_CORRECTION_TYPES = ("rtk_fixed", "rtk_float", "single")


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SimEllipseD(Node):
    def __init__(self):
        super().__init__("sim_ellipse_d")

        # Topics.
        self.ground_truth_topic = self.declare_parameter(
            "ground_truth_topic", "/ground_truth/state"
        ).value
        self.ekf_nav_topic = self.declare_parameter(
            "ekf_nav_topic", "/sbg/ekf_nav"
        ).value
        self.ekf_euler_topic = self.declare_parameter(
            "ekf_euler_topic", "/sbg/ekf_euler"
        ).value

        # Output convention: mirror the driver's output.use_enu (default NED).
        self.use_enu = self.declare_parameter("use_enu", False).value

        # Datum for the inverse ENU->geodetic projection (default: Hanyang Univ).
        self.datum_lat = self.declare_parameter("datum_latitude", 37.5572).value
        self.datum_lon = self.declare_parameter("datum_longitude", 127.0455).value
        self.datum_alt = self.declare_parameter("datum_altitude", 50.0).value

        self.ekf_rate = self.declare_parameter("ekf_rate", 25.0).value

        # --- datasheet-derived 1-sigma constants -------------------------
        self.sigma_pos = {
            "rtk_fixed": self.declare_parameter("sigma_pos_rtk_fixed", 0.01).value,
            "rtk_float": self.declare_parameter("sigma_pos_rtk_float", 0.30).value,
            "single": self.declare_parameter("sigma_pos_single", 1.2).value,
        }
        # Correlation time of the GNSS position error (Gauss-Markov), so
        # single-point error wanders slowly instead of white 1.2 m jitter.
        self.gnss_error_tau = self.declare_parameter("gnss_error_tau", 30.0).value
        self.sigma_vel = self.declare_parameter("sigma_vel", 0.03).value
        self.sigma_vel_single = self.declare_parameter("sigma_vel_single", 0.05).value
        # Dual-antenna heading: 0.2 deg assumes a >2 m antenna baseline; use
        # ~0.0087 (0.5 deg) for a short (~0.5 m) Formula Student baseline.
        self.sigma_heading = self.declare_parameter("sigma_heading", 0.0035).value
        self.sigma_attitude = self.declare_parameter("sigma_attitude", 0.00087).value
        # Free-inertial divergence model (mode <= 2 without odometer):
        # gravity leakage through the attitude error, ramped by the gyro bias.
        self.attitude_error_deg = self.declare_parameter(
            "attitude_error_deg", 0.05
        ).value
        self.gyro_bias_dph = self.declare_parameter("gyro_bias_dph", 7.0).value
        self.gyro_arw_dpsh = self.declare_parameter("gyro_arw_dpsh", 0.18).value
        # Mode 3: residual velocity bias integrated into position + random walk.
        self.mode3_vel_bias = self.declare_parameter("mode3_vel_bias", 0.03).value
        self.mode3_pos_rw = self.declare_parameter("mode3_pos_rw", 0.02).value
        # Future wheel encoder: datasheet land outage = 0.5 % travelled distance.
        self.odometer_aided = self.declare_parameter("odometer_aided", False).value
        self.odometer_drift_ratio = self.declare_parameter(
            "odometer_drift_ratio", 0.005
        ).value
        # Error decay time constant when (re-)entering mode 4 [s].
        self.reacquire_tau = self.declare_parameter("reacquire_tau", 0.5).value
        self.max_dt = self.declare_parameter("max_dt", 0.5).value

        # Mode / correction control.
        self.mode = int(self.declare_parameter("initial_mode", 4).value)
        self.correction = self.declare_parameter(
            "correction_type", "rtk_fixed"
        ).value
        if self.correction not in _CORRECTION_TYPES:
            raise ValueError(f"correction_type must be one of {_CORRECTION_TYPES}")
        self.mode_schedule = self._parse_schedule(
            self.declare_parameter("mode_schedule", "").value, int
        )
        self.correction_schedule = self._parse_schedule(
            self.declare_parameter("correction_schedule", "").value, str
        )
        self.mode_schedule_index = 0
        self.correction_schedule_index = 0
        self.manual_mode = False
        self.manual_correction = False

        self.rng = random.Random(self.declare_parameter("seed", 42).value)

        # Error state (ENU): mode-drift position/velocity/heading errors, the
        # Gauss-Markov GNSS measurement error, and per-episode directions.
        self.pos_err = [0.0, 0.0]
        self.vel_err = [0.0, 0.0]
        self.heading_err = 0.0
        self.gm_err = [0.0, 0.0]
        self.vel_bias_vec = [0.0, 0.0]
        self.accel_dir = [1.0, 0.0]
        self.drift_dir = [1.0, 0.0]
        self.degraded_since = None  # stamp when mode dropped below 3

        self.first_stamp = None
        self.last_stamp = None
        self.last_pub_stamp = None

        # Datum radii for the inverse projection.
        lat0 = math.radians(self.datum_lat)
        sin_lat = math.sin(lat0)
        denom = 1.0 - _WGS84_E2 * sin_lat * sin_lat
        self._meridional_radius = _WGS84_A * (1.0 - _WGS84_E2) / math.pow(denom, 1.5)
        self._prime_vertical_radius = _WGS84_A / math.sqrt(denom)
        self._cos_lat0 = math.cos(lat0)

        self.nav_pub = self.create_publisher(SbgEkfNav, self.ekf_nav_topic, 10)
        self.euler_pub = self.create_publisher(SbgEkfEuler, self.ekf_euler_topic, 10)
        self.gt_sub = self.create_subscription(
            CarState,
            self.ground_truth_topic,
            self.on_ground_truth,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.mode_sub = self.create_subscription(
            UInt8, "/sim_ins/set_solution_mode", self.on_set_mode, 10
        )
        self.correction_sub = self.create_subscription(
            String, "/sim_ins/set_correction_type", self.on_set_correction, 10
        )

        self.get_logger().info(
            f"Simulated Ellipse-D: {self.ground_truth_topic} -> "
            f"{self.ekf_nav_topic}+{self.ekf_euler_topic} at {self.ekf_rate} Hz "
            f"({'ENU' if self.use_enu else 'NED'}), mode={self.mode}, "
            f"correction={self.correction}"
            + (", odometer-aided" if self.odometer_aided else "")
        )

    # --- mode / correction control ---------------------------------------

    @staticmethod
    def _parse_schedule(text, value_type):
        schedule = []
        for token in text.split(","):
            token = token.strip()
            if not token:
                continue
            t_str, value_str = token.split(":")
            schedule.append((float(t_str), value_type(value_str)))
        return sorted(schedule, key=lambda item: item[0])

    def on_set_mode(self, msg):
        self.manual_mode = True
        self._set_mode(int(msg.data), source="topic")

    def on_set_correction(self, msg):
        self.manual_correction = True
        self._set_correction(msg.data.strip(), source="topic")

    def _random_unit(self):
        angle = self.rng.uniform(-math.pi, math.pi)
        return [math.cos(angle), math.sin(angle)]

    def _set_mode(self, mode, source):
        mode = max(0, min(4, mode))
        if mode == self.mode:
            return
        previous = self.mode
        self.mode = mode
        # Re-randomize the episode drift directions so each dropout differs.
        if mode == 3:
            u = self._random_unit()
            self.vel_bias_vec = [self.mode3_vel_bias * u[0], self.mode3_vel_bias * u[1]]
            self.drift_dir = self._random_unit()
            self.degraded_since = None
        elif mode <= 2:
            self.accel_dir = self._random_unit()
            self.drift_dir = self._random_unit()
            self.degraded_since = self.last_stamp
        else:
            self.degraded_since = None
        self.get_logger().info(
            f"solution_mode {previous}({_MODE_NAMES[previous]}) -> "
            f"{mode}({_MODE_NAMES[mode]}) [{source}]"
        )

    def _set_correction(self, correction, source):
        if correction not in _CORRECTION_TYPES:
            self.get_logger().warn(f"unknown correction type '{correction}'")
            return
        if correction == self.correction:
            return
        previous = self.correction
        self.correction = correction
        self.get_logger().info(f"correction {previous} -> {correction} [{source}]")

    def _apply_schedules(self, elapsed):
        if not self.manual_mode:
            while (
                self.mode_schedule_index < len(self.mode_schedule)
                and elapsed >= self.mode_schedule[self.mode_schedule_index][0]
            ):
                self._set_mode(
                    self.mode_schedule[self.mode_schedule_index][1], source="schedule"
                )
                self.mode_schedule_index += 1
        if not self.manual_correction:
            while (
                self.correction_schedule_index < len(self.correction_schedule)
                and elapsed
                >= self.correction_schedule[self.correction_schedule_index][0]
            ):
                self._set_correction(
                    self.correction_schedule[self.correction_schedule_index][1],
                    source="schedule",
                )
                self.correction_schedule_index += 1

    # --- error-state propagation -------------------------------------------

    def _propagate_errors(self, dt, stamp, speed):
        if dt <= 0.0 or dt > self.max_dt:
            return
        sq = math.sqrt(dt)

        # GNSS measurement error: first-order Gauss-Markov at the current
        # correction tier's steady-state sigma.
        sigma_tier = self.sigma_pos[self.correction]
        alpha = math.exp(-dt / self.gnss_error_tau)
        scale = sigma_tier * math.sqrt(max(0.0, 1.0 - alpha * alpha))
        self.gm_err[0] = self.gm_err[0] * alpha + self.rng.gauss(0.0, scale)
        self.gm_err[1] = self.gm_err[1] * alpha + self.rng.gauss(0.0, scale)

        if self.mode >= 4:
            decay = math.exp(-dt / self.reacquire_tau)
            self.pos_err = [e * decay for e in self.pos_err]
            self.vel_err = [e * decay for e in self.vel_err]
            self.heading_err *= decay
            return

        if self.mode == 3:
            decay = math.exp(-dt / self.reacquire_tau)
            self.vel_err = [e * decay for e in self.vel_err]
            if self.odometer_aided:
                step = self.odometer_drift_ratio * speed * dt
                self.pos_err[0] += self.drift_dir[0] * step
                self.pos_err[1] += self.drift_dir[1] * step
            else:
                self.pos_err[0] += self.vel_bias_vec[0] * dt + self.rng.gauss(
                    0.0, self.mode3_pos_rw * sq
                )
                self.pos_err[1] += self.vel_bias_vec[1] * dt + self.rng.gauss(
                    0.0, self.mode3_pos_rw * sq
                )
            return

        # mode <= 2: dead reckoning.
        if self.odometer_aided and self.mode == 2:
            # Odometer speed + dual-antenna heading: datasheet 0.5 %/distance.
            step = self.odometer_drift_ratio * speed * dt
            self.pos_err[0] += self.drift_dir[0] * step
            self.pos_err[1] += self.drift_dir[1] * step
        else:
            # Free inertial: gravity leaks through the attitude error, which
            # ramps at the gyro in-run bias. (The 14 ug accel bias is
            # negligible in comparison.)
            t_out = (
                stamp - self.degraded_since if self.degraded_since is not None else 0.0
            )
            theta = math.radians(self.attitude_error_deg) + math.radians(
                self.gyro_bias_dph / 3600.0
            ) * t_out
            accel = _GRAVITY * math.sin(theta)
            self.vel_err[0] += self.accel_dir[0] * accel * dt
            self.vel_err[1] += self.accel_dir[1] * accel * dt
            self.pos_err[0] += self.vel_err[0] * dt
            self.pos_err[1] += self.vel_err[1] * dt
        if self.mode <= 1:
            # Heading no longer aided: gyro-bias ramp + angular random walk
            # (0.18 deg/sqrt(h) = deg/60 per sqrt(s)).
            self.heading_err += math.radians(self.gyro_bias_dph / 3600.0) * dt
            self.heading_err += self.rng.gauss(
                0.0, math.radians(self.gyro_arw_dpsh) / 60.0 * sq
            )

    # --- publishing ----------------------------------------------------------

    def on_ground_truth(self, msg):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.first_stamp is None:
            self.first_stamp = t
            self.last_stamp = t
        self._apply_schedules(t - self.first_stamp)
        vx, vy = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        self._propagate_errors(t - self.last_stamp, t, math.hypot(vx, vy))
        self.last_stamp = t

        # Decimate to the EKF output rate.
        if (
            self.last_pub_stamp is not None
            and (t - self.last_pub_stamp) < (1.0 / self.ekf_rate) - 1e-6
        ):
            return
        self.last_pub_stamp = t

        # Truth (ENU local frame + body twist).
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        )
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        ve_true = vx * math.cos(yaw) - vy * math.sin(yaw)
        vn_true = vx * math.sin(yaw) + vy * math.cos(yaw)

        # INS estimate = truth + mode drift + GNSS Gauss-Markov error.
        sigma_vel = (
            self.sigma_vel_single if self.correction == "single" else self.sigma_vel
        )
        east = msg.pose.pose.position.x + self.pos_err[0] + self.gm_err[0]
        north = msg.pose.pose.position.y + self.pos_err[1] + self.gm_err[1]
        ve = ve_true + self.vel_err[0] + self.rng.gauss(0.0, sigma_vel)
        vn = vn_true + self.vel_err[1] + self.rng.gauss(0.0, sigma_vel)
        yaw_out = _normalize_angle(
            yaw + self.heading_err + self.rng.gauss(0.0, self.sigma_heading)
        )
        roll_out = roll + self.rng.gauss(0.0, self.sigma_attitude)
        pitch_out = pitch + self.rng.gauss(0.0, self.sigma_attitude)

        # Inverse ENU -> geodetic about the datum.
        lat = self.datum_lat + math.degrees(north / self._meridional_radius)
        lon = self.datum_lon + math.degrees(
            east / (self._prime_vertical_radius * self._cos_lat0)
        )
        alt = self.datum_alt + msg.pose.pose.position.z

        # Reported (EKF self-estimated) accuracies: tier sigma + drift growth.
        sigma_tier = self.sigma_pos[self.correction]
        pos_acc_e = sigma_tier + abs(self.pos_err[0])
        pos_acc_n = sigma_tier + abs(self.pos_err[1])
        vel_acc_e = sigma_vel + abs(self.vel_err[0])
        vel_acc_n = sigma_vel + abs(self.vel_err[1])
        heading_acc = self.sigma_heading + abs(self.heading_err)

        status = SbgEkfStatus()
        status.solution_mode = self.mode
        status.attitude_valid = self.mode >= 1
        status.heading_valid = self.mode >= 2
        status.velocity_valid = self.mode >= 3
        status.position_valid = self.mode >= 3 and math.hypot(*self.pos_err) < 10.0
        status.gps1_pos_used = self.mode >= 4
        status.gps1_vel_used = self.mode >= 3
        status.gps1_hdt_used = self.mode >= 2
        status.odo_used = bool(self.odometer_aided) and self.mode >= 2

        time_stamp = int((t * 1e6) % 2**32)

        nav = SbgEkfNav()
        nav.header.stamp = stamp
        nav.header.frame_id = "imu_link" if self.use_enu else "imu_link_ned"
        nav.time_stamp = time_stamp
        nav.latitude = lat
        nav.longitude = lon
        nav.altitude = alt
        nav.undulation = 0.0
        nav.status = status
        if self.use_enu:
            nav.velocity.x, nav.velocity.y, nav.velocity.z = ve, vn, 0.0
            nav.position_accuracy.x = pos_acc_e
            nav.position_accuracy.y = pos_acc_n
            nav.velocity_accuracy.x = vel_acc_e
            nav.velocity_accuracy.y = vel_acc_n
        else:  # NED: x=North, y=East, z=Down
            nav.velocity.x, nav.velocity.y, nav.velocity.z = vn, ve, 0.0
            nav.position_accuracy.x = pos_acc_n
            nav.position_accuracy.y = pos_acc_e
            nav.velocity_accuracy.x = vel_acc_n
            nav.velocity_accuracy.y = vel_acc_e
        nav.position_accuracy.z = 2.0 * sigma_tier
        nav.velocity_accuracy.z = self.sigma_vel

        euler = SbgEkfEuler()
        euler.header.stamp = stamp
        euler.header.frame_id = nav.header.frame_id
        euler.time_stamp = time_stamp
        euler.status = status
        if self.use_enu:
            euler.angle.x, euler.angle.y = roll_out, pitch_out
            euler.angle.z = yaw_out
        else:
            # Flat-track ENU->NED euler mapping.
            euler.angle.x, euler.angle.y = roll_out, -pitch_out
            euler.angle.z = _normalize_angle(0.5 * math.pi - yaw_out)
        euler.accuracy.x = euler.accuracy.y = self.sigma_attitude
        euler.accuracy.z = heading_acc

        self.nav_pub.publish(nav)
        self.euler_pub.publish(euler)


def main():
    rclpy.init(args=sys.argv)
    node = SimEllipseD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
