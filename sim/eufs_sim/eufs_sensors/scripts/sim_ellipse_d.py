#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Simulated SBG Ellipse-D GNSS/INS for the EUFS simulator.

Consumes the simulator ground truth (``/ground_truth/state``) and re-emits it
as the SBG driver's EKF outputs (``/sbg/ekf_nav`` + ``/sbg/ekf_euler`` +
``/sbg/ekf_rot_accel_body``), so the real-hardware pipeline (sbg_odometry_bridge
-> graph SLAM GNSS prior, and the controller's feasibility gate) runs unmodified
against the simulator.

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

Two properties of a real INS output that the first version of this node got
wrong, both load-bearing for anything that consumes the stream:

* **Every output error is time-correlated, none of it is white.** An EKF is a
  low-pass filter; its velocity/heading/attitude output errors wander over
  seconds, they do not redraw at 200 Hz. White errors would let a downstream
  integrator (sbg_odometry_bridge integrates velocity into the relative
  odometry pose) average them away sqrt(N)-style — the sim odometry drift was
  structurally optimistic. Velocity, heading, attitude and altitude errors are
  now first-order Gauss-Markov states (``vel_error_tau``,
  ``heading_error_tau``, ``attitude_error_tau``; altitude shares
  ``gnss_error_tau``), like the horizontal GNSS error always was. The
  correlation times are NOT datasheet numbers — the datasheet only gives
  steady-state sigmas — they are exposed as parameters so the Allan/parity
  harness can pin them against real bench logs.

* **The reported accuracy is what the unit believes, not what is true.** It
  used to be ``sigma_tier + |realised error|`` — an oracle no real EKF has.
  A real unit reports its covariance: a deterministic function of the aiding
  history (which modes, for how long), propagated from the same nominal
  constants, *independent of the realised error draw*. This node now
  propagates that model (``pos/vel/heading`` accuracy states mirroring every
  branch of the error propagation, with nominal magnitudes and no randomness)
  and reports it. Consequences worth having: the reported sigma is identical
  across seeds while the realised error differs, reported and realised can
  disagree in both directions (confidently wrong / conservatively right), and
  the SLAM prior gate finally gets exercised the way the real unit will
  exercise it. ``position_valid`` is likewise gated on the *believed* accuracy.

``/sbg/ekf_rot_accel_body`` (driver ``log_ekf_rot_accel_body``, sbgECom >= 4.0)
carries the INS body-frame rotation rate and acceleration "free from gravity and
sensor bias/scale errors" — what a controller wants for a friction-limit
feasibility gate, since ``sbg/imu_data.accel`` still has gravity in it and leaks
g*sin(pitch) into the longitudinal channel exactly when the car is braking
hardest. Two things this node has to get right, both silent if wrong:

* **Ground truth's ``linear_acceleration`` is NOT this quantity.** The vehicle
  model sets ``a_x = x_dot.v_x``, the body-frame velocity *derivative*, which
  carries the transport terms: ``v_x_dot = r*v_y + Fx/m`` and
  ``v_y_dot = Fy/m - r*v_x`` (eufs_models/src/dynamic_bicycle.cpp). The specific
  force an INS reports is the ``F/m`` part, so the transport terms have to come
  back out::

      a_x = gt.linear_acceleration.x - r_z * v_y
      a_y = gt.linear_acceleration.y + r_z * v_x

  This is not a detail. In a steady-state corner ``v_y_dot -> 0`` while the real
  lateral acceleration is the whole ``r*v_x``, so forwarding ground truth
  unconverted reports ~0 g of lateral load in precisely the sustained-cornering
  case a feasibility gate exists to catch. test_rot_accel_body_reports_
  centripetal_load pins it.

* **Frame.** Ground truth body is ROS FLU (REP-103); the SBG body frame is
  X-forward/Y-right/Z-down. Hence y and z flip on the way out.

Output error is dominated by gravity leakage through the attitude error, g*
``att_gm`` — the same correlated attitude state the euler output already
reports, so the two channels cannot disagree — plus the datasheet's 14 ug
in-run accel bias and 7 deg/h gyro bias as Gauss-Markov states. Deliberately
not modelled: accelerometer velocity random walk (the EKF output is filtered,
and the bandwidth is not a datasheet number) — pin it with the parity harness
before trusting the high-frequency content.

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

Transport latency IS modelled: messages are stamped at measurement time but
released ``latency_mean`` (+ truncated-gaussian ``latency_jitter``) later, in
FIFO order, the way a serial link delivers them. The defaults (8 +/- 2 ms) are
representative of the common RS-232/422 hookup (a sbgECom frame at 115200+
baud plus OS scheduling); an ethernet install runs ~1 ms. These are
engineering estimates, NOT datasheet values — when the bench exists, measure
arrival-vs-time_stamp with scripts/sensor_parity_report.py and pin them. The
release clock is the ground-truth stream itself, so the worst extra
granularity is one GT period.

Known gaps, recorded so they are chosen and not discovered: no antenna lever
arm, and no 5 Hz measurement-update sawtooth inside mode 4 (the GM error is
smooth). The rot_accel output is reported at the IMU origin, like the real
unit's — a consumer comparing it against a CoG-referenced model still owes
itself the ``r_z_dot * l_x`` lever-arm term, which this node cannot supply
because it does not know where the IMU is mounted.
"""

import collections
import math
import random
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from hyu_msgs.msg import CarState
from std_msgs.msg import String, UInt8

try:
    from sbg_driver.msg import SbgEkfEuler, SbgEkfNav, SbgEkfRotAccel, SbgEkfStatus
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

# Believed position sigma [m] above which the unit stops calling its own
# position valid (mirrors the realised-error threshold this replaced).
_POSITION_VALID_SIGMA = 10.0


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
        self.rot_accel_topic = self.declare_parameter(
            "rot_accel_body_topic", "/sbg/ekf_rot_accel_body"
        ).value
        # Mirrors driver output.log_ekf_rot_accel_body != 0. Off on the real
        # unit by default, and it needs ELLIPSE firmware new enough to know the
        # log at all (sbgECom >= 4.0) — check `ros2 topic hz` on the car before
        # trusting a sim-tuned consumer.
        self.publish_rot_accel = self.declare_parameter(
            "publish_rot_accel_body", True
        ).value

        # Output convention: mirror the driver's output.use_enu (default NED).
        self.use_enu = self.declare_parameter("use_enu", False).value

        # Datum for the inverse ENU->geodetic projection (default: Hanyang Univ).
        self.datum_lat = self.declare_parameter("datum_latitude", 37.5572).value
        self.datum_lon = self.declare_parameter("datum_longitude", 127.0455).value
        self.datum_alt = self.declare_parameter("datum_altitude", 50.0).value

        # Real Ellipse-D EKF/INS output runs at up to 200 Hz (datasheet); the
        # internal GNSS update stays at 5 Hz regardless of this output rate.
        # Effective rate is min(ekf_rate, /ground_truth/state rate) in SIM time;
        # wall-clock rate additionally scales with Gazebo's real-time factor.
        self.ekf_rate = self.declare_parameter("ekf_rate", 200.0).value
        # Each sbgECom log carries its own output divider, so rot_accel_body is
        # decimated independently of ekf_nav/ekf_euler (the recommended car
        # config runs nav at 25 Hz and rot_accel at 100 Hz).
        self.rot_accel_rate = self.declare_parameter("rot_accel_rate", 100.0).value

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
        # Correlation times of the INS output errors. The datasheet gives the
        # steady-state sigmas above but says nothing about correlation; these
        # defaults are engineering estimates (EKF bandwidth ~ the 5 Hz GNSS
        # aiding for velocity, slower for the angle channels) exposed as
        # parameters so bench parity runs can pin them. They must be > 0; the
        # whole point is that the errors are NOT white at 200 Hz.
        self.vel_error_tau = self.declare_parameter("vel_error_tau", 2.0).value
        self.heading_error_tau = self.declare_parameter(
            "heading_error_tau", 10.0
        ).value
        self.attitude_error_tau = self.declare_parameter(
            "attitude_error_tau", 10.0
        ).value
        # Free-inertial divergence model (mode <= 2 without odometer):
        # gravity leakage through the attitude error, ramped by the gyro bias.
        self.attitude_error_deg = self.declare_parameter(
            "attitude_error_deg", 0.05
        ).value
        self.gyro_bias_dph = self.declare_parameter("gyro_bias_dph", 7.0).value
        self.gyro_arw_dpsh = self.declare_parameter("gyro_arw_dpsh", 0.18).value
        # Residual in-run biases left on the rot_accel_body output after the EKF
        # has estimated them out (datasheet 1-sigma: 14 ug accel, 7 deg/h gyro,
        # the latter shared with the free-inertial ramp above). Both are an
        # order of magnitude below the g*attitude_error leakage that dominates
        # this channel; they are here so the budget is complete, not because
        # they move the answer. The correlation time is an engineering estimate
        # (in-run bias stability is quoted without one) — parity-harness bait.
        self.accel_bias_ug = self.declare_parameter("accel_bias_ug", 14.0).value
        self.bias_error_tau = self.declare_parameter("bias_error_tau", 60.0).value
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
        # Transport latency (serial/ethernet + driver): stamp stays the
        # measurement time, arrival is delayed. Estimates, not datasheet —
        # see the module docstring. 0 = publish immediately.
        self.latency_mean = self.declare_parameter("latency_mean", 0.008).value
        self.latency_jitter = self.declare_parameter("latency_jitter", 0.002).value

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

        # Realised error state (ENU): mode-drift position/velocity/heading
        # errors, per-episode directions, and the Gauss-Markov output errors
        # (GNSS position, velocity, heading, attitude, altitude).
        self.pos_err = [0.0, 0.0]
        self.vel_err = [0.0, 0.0]
        self.heading_err = 0.0
        self.gm_err = [0.0, 0.0]
        self.vel_gm = [0.0, 0.0]
        self.heading_gm = 0.0
        self.att_gm = [0.0, 0.0]  # roll, pitch
        self.alt_gm = 0.0
        self.vel_bias_vec = [0.0, 0.0]
        self.accel_dir = [1.0, 0.0]
        self.drift_dir = [1.0, 0.0]
        self.degraded_since = None  # stamp when mode dropped below 3
        # rot_accel_body channel: residual sensor biases (body axes) and the
        # free-inertial attitude-error magnitude, kept in sync with the
        # velocity divergence so the accel output degrades with it instead of
        # staying suspiciously clean through a dropout. Its direction in the
        # body plane is a per-episode draw, like accel_dir is in ENU.
        self.accel_bias_gm = [0.0, 0.0, 0.0]
        self.gyro_bias_gm = [0.0, 0.0, 0.0]
        self.att_ramp = 0.0
        self.accel_body_dir = [1.0, 0.0]

        # Believed (reported) accuracy state: the EKF covariance surrogate.
        # Each channel mirrors the realised propagation branch-for-branch with
        # NOMINAL magnitudes and no randomness: `lin` accumulates bias-like
        # growth [unit], `var` accumulates random-walk variance [unit^2].
        # Reported 1-sigma = sqrt(tier^2 + lin^2 + var). Never touched by the
        # realised draws — that independence is the point.
        self.acc_pos_lin = 0.0
        self.acc_pos_var = 0.0
        self.acc_vel_lin = 0.0
        self.acc_vel_var = 0.0
        self.acc_hdg_lin = 0.0
        self.acc_hdg_var = 0.0

        self.first_stamp = None
        self.last_stamp = None
        self.last_pub_stamp = None
        self.last_rot_pub_stamp = None

        # Messages waiting out their transport latency: (due_time, ((pub, msg),
        # ...)) in FIFO order, the way a serial link delivers them. Released
        # against the ground-truth stream's clock in on_ground_truth. Each log
        # is its own sbgECom frame, so each draws its own latency and a later
        # frame can sit behind an earlier one — head-of-line blocking is what
        # the real link does too.
        self._outbox = collections.deque()

        # Datum radii for the inverse projection.
        lat0 = math.radians(self.datum_lat)
        sin_lat = math.sin(lat0)
        denom = 1.0 - _WGS84_E2 * sin_lat * sin_lat
        self._meridional_radius = _WGS84_A * (1.0 - _WGS84_E2) / math.pow(denom, 1.5)
        self._prime_vertical_radius = _WGS84_A / math.sqrt(denom)
        self._cos_lat0 = math.cos(lat0)

        self.nav_pub = self.create_publisher(SbgEkfNav, self.ekf_nav_topic, 10)
        self.euler_pub = self.create_publisher(SbgEkfEuler, self.ekf_euler_topic, 10)
        self.rot_accel_pub = (
            self.create_publisher(SbgEkfRotAccel, self.rot_accel_topic, 10)
            if self.publish_rot_accel
            else None
        )
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
            + (
                f"(+{self.rot_accel_topic} at {self.rot_accel_rate} Hz) "
                if self.publish_rot_accel
                else ""
            )
            + f"({'ENU' if self.use_enu else 'NED'}), mode={self.mode}, "
            f"correction={self.correction}"
            + (", odometer-aided" if self.odometer_aided else "")
        )
        if self.publish_rot_accel and self.use_enu:
            # Faithfully reproduced, not inherited by accident: see
            # _to_output_frame. Warn rather than quietly emit swapped axes.
            self.get_logger().warn(
                "use_enu:=true — sbg_driver 3.3.2 applies the NED->ENU "
                "NAVIGATION-frame swap to the BODY-frame rot_accel log, so "
                f"{self.rot_accel_topic} carries longitudinal and lateral "
                "exchanged. This node reproduces that so sim matches the car. "
                "Prefer use_enu:=false and flip y/z in the consumer."
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
            self.accel_body_dir = self._random_unit()
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

    def _gm_step(self, value, sigma, tau, dt, pull_in=False):
        """
        One first-order Gauss-Markov step toward steady-state 1-sigma.

        With ``pull_in``, a value left out-of-family by a sigma downgrade
        (e.g. the RTK refix after riding at single-point error) is dragged in
        over ``reacquire_tau`` instead of the slow wander tau — a refix snaps
        in seconds, it does not take a 30 s correlation time to matter.
        """
        if pull_in and abs(value) > 3.0 * sigma:
            value *= math.exp(-dt / self.reacquire_tau)
        alpha = math.exp(-dt / tau)
        scale = sigma * math.sqrt(max(0.0, 1.0 - alpha * alpha))
        return value * alpha + self.rng.gauss(0.0, scale)

    def _propagate_errors(self, dt, stamp, speed):
        if dt <= 0.0 or dt > self.max_dt:
            return
        sq = math.sqrt(dt)

        # Correlated output errors: GNSS position (per axis), velocity,
        # heading, attitude, altitude. Steady-state sigma follows the current
        # correction tier where the tier owns the channel.
        sigma_tier = self.sigma_pos[self.correction]
        sigma_vel = (
            self.sigma_vel_single if self.correction == "single" else self.sigma_vel
        )
        for i in (0, 1):
            self.gm_err[i] = self._gm_step(
                self.gm_err[i], sigma_tier, self.gnss_error_tau, dt, pull_in=True
            )
            self.vel_gm[i] = self._gm_step(
                self.vel_gm[i], sigma_vel, self.vel_error_tau, dt, pull_in=True
            )
            self.att_gm[i] = self._gm_step(
                self.att_gm[i], self.sigma_attitude, self.attitude_error_tau, dt
            )
        self.heading_gm = self._gm_step(
            self.heading_gm, self.sigma_heading, self.heading_error_tau, dt
        )
        # Residual sensor biases on the rot_accel_body channel (body axes).
        sigma_ab = self.accel_bias_ug * 1e-6 * _GRAVITY
        sigma_gb = math.radians(self.gyro_bias_dph / 3600.0)
        for i in (0, 1, 2):
            self.accel_bias_gm[i] = self._gm_step(
                self.accel_bias_gm[i], sigma_ab, self.bias_error_tau, dt
            )
            self.gyro_bias_gm[i] = self._gm_step(
                self.gyro_bias_gm[i], sigma_gb, self.bias_error_tau, dt
            )
        # Datasheet gives no vertical tier table; 2x the horizontal sigma is
        # the convention this node has always reported for z — now it is also
        # what the altitude error actually does.
        self.alt_gm = self._gm_step(
            self.alt_gm, 2.0 * sigma_tier, self.gnss_error_tau, dt, pull_in=True
        )

        # Mode-drift errors (realised) and believed accuracy (nominal), branch
        # for branch. The believed side uses the same constants with no
        # randomness and no knowledge of the realised draws.
        if self.mode >= 4:
            decay = math.exp(-dt / self.reacquire_tau)
            self.pos_err = [e * decay for e in self.pos_err]
            self.vel_err = [e * decay for e in self.vel_err]
            self.heading_err *= decay
            self.acc_pos_lin *= decay
            self.acc_pos_var *= decay * decay
            self.acc_vel_lin *= decay
            self.acc_vel_var *= decay * decay
            self.acc_hdg_lin *= decay
            self.acc_hdg_var *= decay * decay
            self.att_ramp = 0.0
            return

        if self.mode == 3:
            # Velocity is still aided, so attitude is still observable: the
            # accel channel stays at its nominal g*att_gm leakage.
            self.att_ramp = 0.0
            decay = math.exp(-dt / self.reacquire_tau)
            self.vel_err = [e * decay for e in self.vel_err]
            self.acc_vel_lin *= decay
            self.acc_vel_var *= decay * decay
            if self.odometer_aided:
                step = self.odometer_drift_ratio * speed * dt
                self.pos_err[0] += self.drift_dir[0] * step
                self.pos_err[1] += self.drift_dir[1] * step
                self.acc_pos_lin += step
            else:
                self.pos_err[0] += self.vel_bias_vec[0] * dt + self.rng.gauss(
                    0.0, self.mode3_pos_rw * sq
                )
                self.pos_err[1] += self.vel_bias_vec[1] * dt + self.rng.gauss(
                    0.0, self.mode3_pos_rw * sq
                )
                self.acc_pos_lin += self.mode3_vel_bias * dt
                self.acc_pos_var += self.mode3_pos_rw * self.mode3_pos_rw * dt
            return

        # mode <= 2: dead reckoning.
        if self.odometer_aided and self.mode == 2:
            # Odometer speed + dual-antenna heading: datasheet 0.5 %/distance.
            self.att_ramp = 0.0
            step = self.odometer_drift_ratio * speed * dt
            self.pos_err[0] += self.drift_dir[0] * step
            self.pos_err[1] += self.drift_dir[1] * step
            self.acc_pos_lin += step
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
            # Same theta the rot_accel_body output leaks gravity through: the
            # accel channel degrades with the dropout, it does not stay clean.
            self.att_ramp = theta
            accel = _GRAVITY * math.sin(theta)
            self.vel_err[0] += self.accel_dir[0] * accel * dt
            self.vel_err[1] += self.accel_dir[1] * accel * dt
            self.pos_err[0] += self.vel_err[0] * dt
            self.pos_err[1] += self.vel_err[1] * dt
            self.acc_vel_lin += accel * dt
            self.acc_pos_lin += self.acc_vel_lin * dt
        if self.mode <= 1:
            # Heading no longer aided: gyro-bias ramp + angular random walk
            # (0.18 deg/sqrt(h) = deg/60 per sqrt(s)).
            ramp = math.radians(self.gyro_bias_dph / 3600.0) * dt
            arw = math.radians(self.gyro_arw_dpsh) / 60.0
            self.heading_err += ramp
            self.heading_err += self.rng.gauss(0.0, arw * sq)
            self.acc_hdg_lin += ramp
            self.acc_hdg_var += arw * arw * dt

    # --- publishing ----------------------------------------------------------

    def _to_output_frame(self, vec_rfd):
        """
        Device body frame (X fwd, Y right, Z down) -> the driver's output.

        With ``use_enu:=false`` the driver passes the body log through
        untouched, which is what we want. With ``use_enu:=true`` it applies its
        NED->ENU *navigation*-frame conversion (x=v[1], y=v[0], z=-v[2]) to
        this *body*-frame log, exchanging longitudinal and lateral — see
        sbg_ros2_driver message_wrapper.cpp createSbgEkfRotAccelMessage, which
        both rot_accel topics share while only the NED one wants that swap.
        Reproduced deliberately: this node's contract is to emit what the car's
        stack emits, so a consumer that would break on the car breaks here too.
        """
        if self.use_enu:
            return [vec_rfd[1], vec_rfd[0], -vec_rfd[2]]
        return list(vec_rfd)

    def _flush_outbox(self, now):
        """Release queued messages whose transport latency has elapsed."""
        while self._outbox and self._outbox[0][0] <= now:
            _, payload = self._outbox.popleft()
            for publisher, msg in payload:
                publisher.publish(msg)

    def _send(self, t, payload):
        """Publish now, or queue the frame until its transport latency is up."""
        if self.latency_mean <= 0.0:
            for publisher, msg in payload:
                publisher.publish(msg)
            return
        due = t + max(0.0, self.rng.gauss(self.latency_mean, self.latency_jitter))
        self._outbox.append((due, payload))

    def _build_rot_accel(self, msg, stamp, time_stamp):
        """
        Ground-truth body kinematics -> SbgEkfRotAccel (gravity-free, body).

        ``CarState.linear_acceleration`` is the body-frame velocity derivative,
        so the transport terms carried by v_x_dot/v_y_dot have to be removed to
        recover the specific force the INS reports; without that a steady-state
        corner reads ~0 lateral g. See the module docstring.
        """
        v_x, v_y = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        r_x, r_y, r_z = (
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        )
        a_x = msg.linear_acceleration.x - r_z * v_y
        a_y = msg.linear_acceleration.y + r_z * v_x
        a_z = msg.linear_acceleration.z

        # Gravity leakage through the attitude error dominates this channel:
        # the EKF removes gravity with the attitude it believes, so a roll/pitch
        # error of theta leaves g*sin(theta) behind. att_gm is the same state
        # the euler output reports, so the two channels agree by construction;
        # att_ramp adds the free-inertial growth. The sign convention below is a
        # choice, not a derivation — both errors are zero-mean and symmetric, so
        # only their magnitude and correlation carry information.
        leak = _GRAVITY * math.sin(self.att_ramp)
        a_x += -_GRAVITY * self.att_gm[1] + self.accel_body_dir[0] * leak
        a_y += _GRAVITY * self.att_gm[0] + self.accel_body_dir[1] * leak
        a_x += self.accel_bias_gm[0]
        a_y += self.accel_bias_gm[1]
        a_z += self.accel_bias_gm[2]

        # Ground truth body is ROS FLU; the SBG body frame is X fwd/Y right/Z
        # down, so the lateral and vertical axes flip.
        accel_rfd = [a_x, -a_y, -a_z]
        rate_rfd = [
            r_x + self.gyro_bias_gm[0],
            -(r_y + self.gyro_bias_gm[1]),
            -(r_z + self.gyro_bias_gm[2]),
        ]

        rot = SbgEkfRotAccel()
        rot.header.stamp = stamp
        rot.header.frame_id = "imu_link" if self.use_enu else "imu_link_ned"
        rot.time_stamp = time_stamp
        out_accel = self._to_output_frame(accel_rfd)
        out_rate = self._to_output_frame(rate_rfd)
        rot.acceleration.x, rot.acceleration.y, rot.acceleration.z = out_accel
        rot.rate.x, rot.rate.y, rot.rate.z = out_rate
        return rot

    def on_ground_truth(self, msg):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.first_stamp is None:
            self.first_stamp = t
            self.last_stamp = t
        self._flush_outbox(t)
        self._apply_schedules(t - self.first_stamp)
        vx, vy = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        self._propagate_errors(t - self.last_stamp, t, math.hypot(vx, vy))
        self.last_stamp = t

        time_stamp = int((t * 1e6) % 2**32)

        # rot_accel_body carries its own sbgECom output divider, so it is
        # decimated independently of the nav/euler pair below.
        if self.rot_accel_pub is not None and (
            self.last_rot_pub_stamp is None
            or (t - self.last_rot_pub_stamp) >= (1.0 / self.rot_accel_rate) - 1e-6
        ):
            self.last_rot_pub_stamp = t
            self._send(
                t,
                ((self.rot_accel_pub, self._build_rot_accel(msg, stamp, time_stamp)),),
            )

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

        # INS estimate = truth + mode drift + correlated output errors. No
        # white noise anywhere: at 200 Hz a real EKF's output error moves on
        # its correlation time, not per sample.
        east = msg.pose.pose.position.x + self.pos_err[0] + self.gm_err[0]
        north = msg.pose.pose.position.y + self.pos_err[1] + self.gm_err[1]
        ve = ve_true + self.vel_err[0] + self.vel_gm[0]
        vn = vn_true + self.vel_err[1] + self.vel_gm[1]
        yaw_out = _normalize_angle(yaw + self.heading_err + self.heading_gm)
        roll_out = roll + self.att_gm[0]
        pitch_out = pitch + self.att_gm[1]

        # Inverse ENU -> geodetic about the datum.
        lat = self.datum_lat + math.degrees(north / self._meridional_radius)
        lon = self.datum_lon + math.degrees(
            east / (self._prime_vertical_radius * self._cos_lat0)
        )
        alt = self.datum_alt + msg.pose.pose.position.z + self.alt_gm

        # Reported (believed) accuracies: deterministic covariance surrogate,
        # independent of the realised draws above. Identical across seeds.
        sigma_tier = self.sigma_pos[self.correction]
        sigma_vel = (
            self.sigma_vel_single if self.correction == "single" else self.sigma_vel
        )
        pos_acc = math.sqrt(
            sigma_tier * sigma_tier
            + self.acc_pos_lin * self.acc_pos_lin
            + self.acc_pos_var
        )
        vel_acc = math.sqrt(
            sigma_vel * sigma_vel
            + self.acc_vel_lin * self.acc_vel_lin
            + self.acc_vel_var
        )
        heading_acc = math.sqrt(
            self.sigma_heading * self.sigma_heading
            + self.acc_hdg_lin * self.acc_hdg_lin
            + self.acc_hdg_var
        )

        status = SbgEkfStatus()
        status.solution_mode = self.mode
        status.attitude_valid = self.mode >= 1
        status.heading_valid = self.mode >= 2
        status.velocity_valid = self.mode >= 3
        status.position_valid = self.mode >= 3 and pos_acc < _POSITION_VALID_SIGMA
        status.gps1_pos_used = self.mode >= 4
        status.gps1_vel_used = self.mode >= 3
        status.gps1_hdt_used = self.mode >= 2
        status.odo_used = bool(self.odometer_aided) and self.mode >= 2

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
        else:  # NED: x=North, y=East, z=Down
            nav.velocity.x, nav.velocity.y, nav.velocity.z = vn, ve, 0.0
        # The believed accuracy model is isotropic (a real EKF's is close to
        # it on a land vehicle); publish the same sigma on both axes.
        nav.position_accuracy.x = pos_acc
        nav.position_accuracy.y = pos_acc
        nav.position_accuracy.z = 2.0 * sigma_tier
        nav.velocity_accuracy.x = vel_acc
        nav.velocity_accuracy.y = vel_acc
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

        self._send(t, ((self.nav_pub, nav), (self.euler_pub, euler)))


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
