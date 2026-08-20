#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
# SIZE_OK: This integration only preserves topic/config compatibility; splitting
# the legacy monolithic SBG odometry bridge is outside this focused scope.
"""
SBG Ellipse-D GNSS/INS -> odometry + global-anchor bridge for EUFS graph SLAM.

The Ellipse-D is a GNSS/INS whose internal EKF fuses the built-in IMU with
(RTK) GNSS and dual-antenna heading. Its solution quality is reported by
SbgEkfStatus.solution_mode:

  0 UNINITIALIZED  nothing valid
  1 VERTICAL_GYRO  roll/pitch only; heading + nav drift freely
  2 AHRS           full orientation (heading) valid; position/velocity drift
  3 NAV_VELOCITY   orientation + velocity valid; position integrated (drifts)
  4 NAV_POSITION   nominal; absolute GNSS-anchored position (position_valid)

Rather than hard-gating on a valid fix (which would drop the SLAM motion input
the instant RTK is lost), this node *degrades gracefully* across those modes and
splits the GNSS solution into two decoupled roles so losing the global anchor
never stops the odometry:

  * ``/odometry_integration/car_state`` (hyu_msgs/CarState) -- **relative
    odometry**. Pose is integrated here on ONE integrator whose motion source
    is picked per tick from a ladder, so it is jump-free even when RTK
    re-acquisition makes the absolute position step or the source switches.
    graph_slam_node differences this between keyframes. The ladder:

      A  EKF nav velocity + EKF heading      mode >= 3, velocity_valid
      C  raw GNSS (RTK fix deltas + Doppler) + HDT/gyro heading   [rtk grade]
      B  wheel speed x heading               /vehicle/wheel_speeds live
      C  raw GNSS Doppler only + HDT/gyro heading                 [doppler grade]
      -  hold (frozen pose, huge sigma) for at most ``held_max_sec``, then
         FAULT: publication stops and SLAM coasts on its keyframe snapshot.

    Rung C exists because of what the vehicle actually did on 2026-08-01: the
    Ellipse EKF re-initialised mid-run three times (mode 4 -> 0/1 for ~30 s
    each, car driving 50-70 m in circles) while the raw receiver kept
    reporting RTK_INT position (1 cm), Doppler velocity and dual-antenna
    heading (0.4 deg) at 5 Hz the whole time. The original ladder was keyed
    on solution_mode alone, so mode 1 meant "fault", the motion input went
    silent, and on re-entry the pose was 50-70 m behind the car. Rung C reads
    /sbg/gps_pos, /sbg/gps_vel, /sbg/gps_hdt and /sbg/imu_data directly, so an
    EKF reset is invisible to SLAM as long as the receiver itself is healthy.
    The dual-antenna heading has an installation-dependent offset from the
    vehicle heading (antenna order); it is learned online from the EKF while
    the EKF is healthy (``hdt_yaw_offset_deg`` NaN) or pinned by parameter.
    Between HDT epochs (and through short HDT gaps) the heading is carried by
    the raw gyro; its sigma grows with gyro-only time.

    Rung B (wheels) needs a wheel-speed source; the vehicle has none wired
    today (no CAN bridge), so on the car the ladder is effectively A -> C ->
    hold. Mode 2 (AHRS) still has a valid heading by definition, so with
    wheels present the SAME integrator falls back to wheel-speed dead
    reckoning (sigma widened to ``odom_sigma_mode2``). Cutting the motion
    input is what used to kill SLAM: the pose froze while the car kept
    moving, and the re-entry jump exceeded the cone association gate.
    Publication stops only when every rung is dead. The first message after
    a blind gap (fault, or a hold that timed out) carries a huge sigma so the
    graph does not believe a confident tiny delta across a gap during which
    the car may have moved.
  * ``/localization/gnss_odom`` (nav_msgs/Odometry) -- raw ekf_nav absolute ENU
    position with a mode-tiered covariance (tight only at mode 4 / RTK, huge
    otherwise). **Nothing consumes it.** It was meant to enter the graph as a
    unary GPS prior gated on that covariance, so the anchor would fade as GNSS
    degraded; that was dropped deliberately and graph_slam_node has no GNSS
    code at all (no EdgeSE2XYPrior, no lat/lon, no datum). SLAM therefore runs
    on RELATIVE odometry alone and only cone landmarks bound its drift.
    Published anyway because it costs nothing and is the obvious debugging and
    re-entry point, but do not read this topic's existence as an anchor.
  * ``/sbg_bridge/status`` (diagnostic_msgs/DiagnosticArray) -- health so the
    autonomy stack knows the current degradation tier.

Both channels live in the same local ENU frame (datum = first valid fix, or a
fixed ``datum_*`` for reproducibility). The datum never leaves this node --
graph_slam neither reads it nor georeferences the maps it saves -- so it
matters only to the (unconsumed) absolute channel above.

Startup is NOT gated on mode 4. A fix georeferences the origin when one is
there; otherwise ``allow_dr_start`` (default true) starts on wheels or raw
GNSS with a provisional (0,0) origin and holds the absolute channel until a
fix arrives, because SLAM differences CarState and does not care where zero
is. A DR/raw start needs a heading source: the EKF heading (mode >= 2) or the
dual-antenna heading with a known offset. Mode 1 with neither is refused on
purpose: heading drifts freely there, and integrating speed under it walks the
pose confidently in the wrong direction.
Heading
(NED 0=North CW / ENU 0=East CCW) is converted to ROS ENU yaw; set
``frame_convention`` to match the driver's ``output.use_enu`` (default false =
"ned"). Run with ``use_sim_time:=false`` on real hardware.
"""

import collections
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from hyu_msgs.msg import CarState, WheelSpeedsStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Optional fixed screen-corner HUD (same plugin the ATE overlay uses:
# apt install ros-humble-rviz-2d-overlay-plugins).
try:
    from rviz_2d_overlay_msgs.msg import OverlayText
except ImportError:  # pragma: no cover - depends on runtime environment
    OverlayText = None

try:
    from sbg_driver.msg import (
        SbgEkfEuler, SbgEkfNav, SbgGpsHdt, SbgGpsPos, SbgGpsVel, SbgImuData,
    )
except ImportError as exc:  # pragma: no cover - depends on runtime environment
    raise SystemExit(
        "sbg_driver messages not found. Source the workspace that provides the "
        "sbg_driver package before running this node."
    ) from exc

# WGS84 ellipsoid.
_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3

_HUGE_SIGMA = 1.0e3  # 1-sigma [m] that makes a covariance-gated prior negligible

# ekf_euler age beyond which the published yaw sigma is inflated: heading is
# sampled asynchronously, so if euler stops the last yaw would otherwise be
# republished forever with its old (tight) sigma.
_HEADING_STALE_S = 0.5

_RPM_TO_RAD_S = 2.0 * math.pi / 60.0

# Raw GNSS status enums (sbg_driver SbgGpsPos/SbgGpsVel/SbgGpsHdt).
_GPS_SOL_COMPUTED = 0
_GPS_POS_TYPE_RTK_FLOAT = 6
_GPS_HDT_STATUS_MASK = 0x3F  # bit 6 is the BASELINE_VALID flag, not a status


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SbgOdometryBridge(Node):
    def __init__(self):
        super().__init__("sbg_odometry_bridge")

        # Topics.
        self.car_state_topic = self.declare_parameter(
            "car_state_topic", "/localization/ins_odom"
        ).value
        self.gnss_odom_topic = self.declare_parameter(
            "gnss_odom_topic", "/localization/gnss_odom"
        ).value
        self.health_topic = self.declare_parameter(
            "health_topic", "/sbg_bridge/status"
        ).value
        # Default off: the 2-sigma fix disc reads like a floating pie chart in
        # RViz. Re-enable with publish_markers:=true when debugging GNSS.
        self.publish_markers = self.declare_parameter("publish_markers", False).value
        self.marker_topic = self.declare_parameter(
            "marker_topic", "/localization/debug/gnss_markers"
        ).value
        self.publish_overlay = self.declare_parameter("publish_overlay", True).value
        self.overlay_topic = self.declare_parameter(
            "overlay_topic", "/localization/debug/gnss_overlay"
        ).value
        self.ekf_nav_topic = self.declare_parameter(
            "ekf_nav_topic", "/sbg/ekf_nav"
        ).value
        self.ekf_euler_topic = self.declare_parameter(
            "ekf_euler_topic", "/sbg/ekf_euler"
        ).value

        # Frames.
        self.world_frame = self.declare_parameter("world_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value

        # Convention of the SBG driver output: "enu" or "ned". Must match the
        # driver's `output.use_enu` param (use_enu:false -> "ned", the default).
        self.convention = (
            self.declare_parameter("frame_convention", "ned").value.strip().lower()
        )
        if self.convention not in ("enu", "ned"):
            raise ValueError("frame_convention must be 'enu' or 'ned'")

        # Datum: NaN means "use first valid fix".
        self.datum_lat = self.declare_parameter("datum_latitude", float("nan")).value
        self.datum_lon = self.declare_parameter("datum_longitude", float("nan")).value

        # Degradation policy (see module docstring for the solution_mode ladder).
        # Wait for one absolute fix (mode >= start_min_solution_mode) before
        # emitting anything, so the SLAM origin is georeferenced.
        # Allow a dead-reckoning-only start when no absolute fix is available
        # yet (covered boot / cold GNSS). Motion output begins on a provisional
        # origin; the georeferenced anchor waits for a real fix. Without this
        # the bridge never publishes and the whole state chain sits silent.
        self.allow_dr_start = self.declare_parameter("allow_dr_start", True).value
        self.start_min_solution_mode = self.declare_parameter(
            "start_min_solution_mode", 4
        ).value
        # Keep publishing odometry down to this mode; below it we fault (stop).
        # Default 3 (NAV_VELOCITY): in mode 2 (AHRS) the position is held while
        # the car may still be moving, so publishing would feed the graph
        # confident zero-motion edges that warp the map. Stopping publication
        # lets the SLAM side coast on its keyframe snapshot instead.
        self.min_odom_solution_mode = self.declare_parameter(
            "min_odom_solution_mode", 3
        ).value
        # The absolute /localization/gnss_odom prior is only trustworthy at/above this mode;
        # below it we still publish but with a huge covariance so the SLAM
        # prior gate drops it.
        self.absolute_min_solution_mode = self.declare_parameter(
            "absolute_min_solution_mode", 4
        ).value
        # Reject integration steps with a non-positive or too-large dt [s].
        self.max_dt = self.declare_parameter("max_dt", 0.5).value
        # How long a frozen ("held") pose may keep going out with a huge sigma
        # before the bridge faults instead. Holding bridges a momentary
        # velocity dropout without a fault; holding for long is worse than a
        # fault, because ego_odom keeps flowing with a pose that no longer
        # moves and the downstream stale-odometry watchdogs (pure pursuit
        # brakes at 0.5 s of silence) never trip while the car keeps going.
        self.held_max_sec = self.declare_parameter("held_max_sec", 0.5).value

        # Refix holdoff. When the solution recovers (mode returns to 4, or the
        # correction tier improves e.g. single -> RTK), the INS's *believed*
        # sigma snaps to the new tier instantly while its *realized* position
        # error pulls in over seconds — a window of confident-but-wrong
        # absolute fixes. Fed to the graph at the tier sigma, those priors
        # yank the whole pose chain toward the not-yet-converged fix and the
        # car localizes off the track (2026-07-19 fault-injection runs). For
        # gnss_refix_holdoff_sec after any tier improvement, the published
        # anchor sigma is floored at gnss_refix_holdoff_sigma so graph_slam's
        # covariance gate (gnss_prior_max_position_sigma, 3.0 m) drops the
        # priors naturally — no extra protocol between the nodes. The floor
        # must stay above that gate or the "held" priors get admitted anyway,
        # merely downweighted. The very first anchor after georeferencing is
        # exempt: the datum is taken from that fix, so its error is absorbed
        # into the map frame and there is no graph to yank yet.
        self.refix_holdoff_sec = self.declare_parameter(
            "gnss_refix_holdoff_sec", 3.0
        ).value
        self.refix_holdoff_sigma = self.declare_parameter(
            "gnss_refix_holdoff_sigma", 5.0
        ).value

        # 1-sigma fallbacks / tier inflation [m], [rad].
        self.default_position_sigma = self.declare_parameter(
            "default_position_sigma", 0.05
        ).value
        self.default_heading_sigma = self.declare_parameter(
            "default_heading_sigma", 0.02
        ).value
        # Odometry (relative) 1-sigma per tier used for the CarState covariance.
        # mode2 is the wheel+AHRS dead-reckoning tier. Measured in the
        # fault-injection harness (small_track, 40 s AHRS outage): DR error
        # ~0.7 % of distance — mode-3 grade — so it shares mode 3's sigma. Do
        # NOT soften it "to be safe": at 0.5 the graph went limp during the
        # outage and mapped a warped lap segment (ATE 3.3 m, 35 false
        # landmarks); at 0.2 the same run held together (ATE 0.28 m, clean
        # map). The unmodelled slip bias rides in this number too — this path
        # applies no traction-slip compensation.
        self.odom_sigma_mode4 = self.declare_parameter("odom_sigma_mode4", 0.05).value
        self.odom_sigma_mode3 = self.declare_parameter("odom_sigma_mode3", 0.20).value
        self.odom_sigma_mode2 = self.declare_parameter("odom_sigma_mode2", 0.20).value
        # Per-edge trust floor from reported velocity accuracy (its 1 s
        # displacement error): degrades motion-edge stiffness with the
        # CORRECTION grade, which the mode tier alone cannot see.
        self.vel_accuracy_horizon_sec = self.declare_parameter(
            "vel_accuracy_horizon_sec", 1.0).value

        # Wheel+AHRS dead-reckoning fallback (mode 2). Heading is valid in
        # AHRS mode by definition; wheel speeds come from the CAN bus and do
        # not care about GNSS. Together they keep the relative odometry
        # flowing so the cone-anchored SLAM pose never free-runs.
        self.ahrs_fallback_enable = self.declare_parameter(
            "ahrs_fallback_enable", True
        ).value
        self.wheel_speeds_topic = self.declare_parameter(
            "wheel_speeds_topic", "/vehicle/wheel_speeds"
        ).value
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.2525).value
        # Wheel-speed / heading staleness beyond which dead reckoning is not
        # attempted and the bridge faults (stops publishing) as before.
        self.wheel_speed_timeout = self.declare_parameter(
            "wheel_speed_timeout", 0.3
        ).value

        # Raw-GNSS fallback (rung C, see the module docstring). Reads the
        # receiver outputs the EKF is built from, so an EKF re-initialisation
        # (observed 3x on 2026-08-01, ~30 s each, mode 4 -> 0/1) does not take
        # the motion input down with it.
        self.raw_gnss_fallback_enable = self.declare_parameter(
            "raw_gnss_fallback_enable", True
        ).value
        self.gps_pos_topic = self.declare_parameter(
            "gps_pos_topic", "/sbg/gps_pos").value
        self.gps_vel_topic = self.declare_parameter(
            "gps_vel_topic", "/sbg/gps_vel").value
        self.gps_hdt_topic = self.declare_parameter(
            "gps_hdt_topic", "/sbg/gps_hdt").value
        self.imu_topic = self.declare_parameter("imu_topic", "/sbg/imu_data").value
        # Receiver epochs come at 5 Hz; allow one missed epoch.
        self.raw_gnss_timeout = self.declare_parameter(
            "raw_gnss_timeout", 0.45).value
        # gps_pos types at/above this are "rtk grade": their epoch-to-epoch
        # deltas correct the Doppler dead reckoning (RTK_FLOAT=6, RTK_INT=7).
        # Below it (single / PSRDIFF, metre-scale multipath excursions) only
        # the Doppler velocity is integrated.
        self.raw_fix_min_type_rtk = self.declare_parameter(
            "raw_fix_min_type_rtk", _GPS_POS_TYPE_RTK_FLOAT).value
        # Doppler velocity accuracy above which the raw rung is refused.
        self.raw_vel_max_accuracy = self.declare_parameter(
            "raw_vel_max_accuracy", 0.5).value
        # Dual-antenna heading -> vehicle heading offset [deg]. NaN = learn it
        # online while the EKF is healthy (mode 4, heading_valid, gps1_hdt_used):
        # the receiver reports the primary->secondary baseline direction, which
        # is 180 deg off the vehicle heading with the current rear-secondary
        # installation. Pin it explicitly to allow a raw-GNSS start before the
        # EKF has ever aligned.
        self.hdt_yaw_offset_deg = self.declare_parameter(
            "hdt_yaw_offset_deg", float("nan")).value
        # HDT samples with a reported accuracy above this are ignored [deg].
        self.hdt_max_accuracy_deg = self.declare_parameter(
            "hdt_max_accuracy_deg", 2.0).value
        # Yaw sigma growth while the heading is carried by the raw gyro alone
        # [rad/s]. Measured on the 2026-08-01 outages: gyro-only heading error
        # 2-6 deg over ~30 s of continuous turning (roll/pitch coupling, no
        # bias estimate), i.e. ~0.1-0.2 deg/s.
        self.gyro_yaw_drift_rate = self.declare_parameter(
            "gyro_yaw_drift_rate", 0.004).value
        # Beyond this long without an absolute heading update (EKF or HDT) the
        # gyro-carried heading is refused and the raw/wheel rungs go dark.
        self.gyro_only_heading_timeout = self.declare_parameter(
            "gyro_only_heading_timeout", 10.0).value
        self.imu_timeout = self.declare_parameter("imu_timeout", 0.3).value
        # Relative-odometry sigma for the raw rung, by grade.
        self.odom_sigma_raw_rtk = self.declare_parameter(
            "odom_sigma_raw_rtk", 0.10).value
        self.odom_sigma_raw_doppler = self.declare_parameter(
            "odom_sigma_raw_doppler", 0.20).value
        # Primary GNSS antenna position in the vehicle body frame (x forward,
        # y left) [m]: gps_pos/gps_vel are the ANTENNA's, ekf_nav is at the
        # IMU. Defaults follow the flashed settings (leverArmPrimary 0.18 m
        # ahead). Only rotation-induced motion is compensated (v_ant = v_base
        # + w x l); the constant offset cancels in deltas.
        self.antenna_lever_arm_x = self.declare_parameter(
            "antenna_lever_arm_x", 0.18).value
        self.antenna_lever_arm_y = self.declare_parameter(
            "antenna_lever_arm_y", 0.0).value

        # Zero-velocity update (ZUPT). At a true standstill the EKF output the
        # bridge integrates is NOT trustworthy: measured on the real unit, the
        # nav velocity reads 2-8 cm/s of residual noise (integrated, that is
        # ~2 m of phantom travel over a 90 s stop) and the heading free-drifts
        # and jumps up to ~9 deg because the dual-antenna heading is unused
        # (gps1_hdt_used ~1 %) and the velocity course is undefined at rest --
        # all while solution_mode stays 4 and every valid flag stays true. An
        # Ackermann car cannot translate or yaw with the wheels stopped, so when
        # the WHEEL SPEED says stopped we freeze both the integrated position
        # and the heading, killing the phantom motion/rotation at the source.
        # Gate on wheel speed, NOT the nav velocity -- the nav velocity is the
        # very signal that lies at rest; the encoder reads a true zero. With no
        # fresh wheel speed the gate is simply inactive (no worse than before)
        # until the CAN wheel-speed driver is wired. Hysteresis (enter < exit)
        # stops chattering on creep at the threshold.
        self.stationary_enter_speed = self.declare_parameter(
            "stationary_enter_speed", 0.1
        ).value
        self.stationary_exit_speed = self.declare_parameter(
            "stationary_exit_speed", 0.3
        ).value

        # Local tangent-plane origin (radians) and curvature radii, set from the
        # datum or the first valid fix.
        self._origin_lat = None
        self._origin_lon = None
        self._meridional_radius = None
        self._prime_vertical_radius = None
        if math.isfinite(self.datum_lat) and math.isfinite(self.datum_lon):
            self._set_origin(self.datum_lat, self.datum_lon)

        # Integrated (jump-free) odometry pose in ENU: (x, y, yaw). Set once the
        # first valid absolute fix arrives. Velocity is integrated with the
        # trapezoid rule (previous sample kept) so curved driving does not
        # accumulate the rectangle-rule lag (~a*dt/2 per second).
        self._odom_xy = None
        self._odom_yaw = 0.0
        self._last_int_t = None
        self._last_vel_enu = None
        self._started = False
        # True once the origin datum is fixed from a real absolute fix; the
        # /localization/gnss_odom anchor only publishes when georeferenced (a DR-only start
        # runs un-georeferenced until a fix arrives).
        self._georeferenced = False
        # Stamp of the last nav message that reached the publishers, for the
        # monotonicity gate below.
        self._last_pub_nav_t = None
        # Stamp of the last CarState actually published, the hold-start stamp
        # (None while not holding), and whether a blind gap (fault or timed-out
        # hold) has happened since the last publish.
        self._last_car_state_t = None
        self._held_since = None
        self._blind_gap = False
        # Last published body forward speed, carried through a hold instead of
        # a zero (a zero winds the speed loop up; see _publish_car_state).
        self._last_body_vx = 0.0

        # Latest heading solution.
        self._yaw = None
        self._yaw_rate_estimate = 0.0
        self._yaw_sigma = self.default_heading_sigma
        self._yaw_stamp = None
        self._heading_valid = False

        # Fallback heading (ENU) for the rungs that do not have the EKF's:
        # tracks the EKF yaw while that is valid, is corrected by the
        # dual-antenna HDT (+ learned offset) when it is not, and is carried
        # by the raw gyro in between. _yaw_dr_abs_t is the stamp of the last
        # ABSOLUTE update (EKF or HDT); the sigma grows from _yaw_dr_abs_sigma
        # with the gyro-only time since.
        self._yaw_dr = None
        self._yaw_dr_stamp = None
        self._yaw_dr_abs_t = None
        self._yaw_dr_abs_sigma = self.default_heading_sigma
        # HDT -> vehicle-heading offset [rad] as a running unit vector (None
        # until learned or pinned), and the last accepted HDT sample.
        self._hdt_offset = (
            math.radians(self.hdt_yaw_offset_deg)
            if math.isfinite(self.hdt_yaw_offset_deg) else None
        )
        self._hdt_offset_vec = None
        self._hdt_offset_n = 0
        self._hdt_yaw_enu_raw = None
        self._hdt_stamp = None
        self._hdt_sigma = None
        # Raw gyro yaw rate (ENU sign, rad/s) and stamp.
        self._gyro_wz = 0.0
        self._gyro_stamp = None
        # Device-clock -> ROS-stamp anchor from the latest imu_data (device
        # time_stamp [us], ROS header stamp [s]). The receiver epochs (gps_pos /
        # gps_vel / gps_hdt) are emitted ~90 ms after the EKF/IMU frame that
        # carries the SAME device time_stamp (measured 2026-08-01: +90 ms p50,
        # +113 ms p90), so their header stamps describe the receipt, not the
        # instant; at 35 deg/s that is 3-4 deg of heading. Their time_stamp
        # is the instant -- map it through this anchor.
        self._dev_anchor = None
        # Raw Doppler velocity (ENU, antenna) and its epoch stamp / accuracy.
        self._raw_vel_enu = None
        self._raw_vel_stamp = None
        self._raw_vel_acc = None
        # Raw fix chain for the rtk-grade correction: last rtk-grade fix
        # (stamp, ENU base point with the lever arm removed) and, while the
        # rung is integrating, the pure Doppler-DR position sampled at that
        # fix's time (_raw_chain_ref). The pure-DR track (_raw_dr_pos with a
        # short history) is never corrected, so the DR displacement over an
        # epoch interval can be read off by interpolation, independent of the
        # order in which the 5 Hz epoch and the 25 Hz tick with the same stamp
        # happen to arrive.
        self._raw_fix_stamp = None
        self._raw_fix_enu = None
        self._raw_fix_rtk = False
        self._raw_chain_ref = None
        self._raw_dr_pos = [0.0, 0.0]
        self._raw_dr_hist = collections.deque()
        # Whether the last published tick ran on the raw rung (bookkeeping for
        # the fix chain: the chain only counts while the raw rung integrates).
        self._raw_active = False
        # Last nav gps1_hdt_used flag (offset learning gate).
        self._nav_hdt_used = False
        self._nav_mode = 0

        # Latest wheel-speed sample (rear-axle mean [m/s], stamp [s]) for the
        # dead-reckoning fallback.
        self._wheel_speed = None
        self._wheel_stamp = None
        # ZUPT latch: True while the wheels report a standstill (hysteresis).
        self._stationary = False

        # Refix-holdoff state: correction tier of the last anchored message
        # (None while unanchored), whether an anchor was ever published (the
        # first-anchor exemption), and the holdoff expiry stamp.
        self._anchor_tier = None
        self._anchor_ever = False
        self._holdoff_until = None

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.car_state_pub = self.create_publisher(CarState, self.car_state_topic, 10)
        self.gnss_odom_pub = self.create_publisher(Odometry, self.gnss_odom_topic, 10)
        self.health_pub = self.create_publisher(DiagnosticArray, self.health_topic, 10)
        self.marker_pub = (
            self.create_publisher(MarkerArray, self.marker_topic, 10)
            if self.publish_markers
            else None
        )
        self._last_marker_t = None
        # Latched so RViz shows the latest state immediately on (re)connect.
        self.overlay_pub = (
            self.create_publisher(
                OverlayText,
                self.overlay_topic,
                QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
            )
            if (self.publish_overlay and OverlayText is not None)
            else None
        )
        self._last_board_t = None
        self.euler_sub = self.create_subscription(
            SbgEkfEuler, self.ekf_euler_topic, self.on_euler, sensor_qos
        )
        self.nav_sub = self.create_subscription(
            SbgEkfNav, self.ekf_nav_topic, self.on_nav, sensor_qos
        )
        self.wheel_sub = (
            self.create_subscription(
                WheelSpeedsStamped,
                self.wheel_speeds_topic,
                self.on_wheel_speeds,
                sensor_qos,
            )
            if self.ahrs_fallback_enable
            else None
        )
        self.gps_pos_sub = self.gps_vel_sub = self.gps_hdt_sub = None
        self.imu_sub = None
        if self.raw_gnss_fallback_enable:
            self.gps_pos_sub = self.create_subscription(
                SbgGpsPos, self.gps_pos_topic, self.on_gps_pos, sensor_qos)
            self.gps_vel_sub = self.create_subscription(
                SbgGpsVel, self.gps_vel_topic, self.on_gps_vel, sensor_qos)
            self.gps_hdt_sub = self.create_subscription(
                SbgGpsHdt, self.gps_hdt_topic, self.on_gps_hdt, sensor_qos)
            self.imu_sub = self.create_subscription(
                SbgImuData, self.imu_topic, self.on_imu, sensor_qos)

        self.get_logger().info(
            f"SBG odometry bridge: {self.ekf_nav_topic}+{self.ekf_euler_topic} -> "
            f"{self.car_state_topic} (odom) + {self.gnss_odom_topic} (anchor) "
            f"[convention={self.convention}, waiting for mode>="
            f"{self.start_min_solution_mode} fix, raw_gnss_fallback="
            f"{'on' if self.raw_gnss_fallback_enable else 'off'}, wheels="
            f"{'on' if self.ahrs_fallback_enable else 'off'}]"
        )

    # --- geodesy ---------------------------------------------------------

    def _set_origin(self, lat_deg, lon_deg):
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        sin_lat = math.sin(lat)
        denom = 1.0 - _WGS84_E2 * sin_lat * sin_lat
        self._origin_lat = lat
        self._origin_lon = lon
        self._meridional_radius = _WGS84_A * (1.0 - _WGS84_E2) / math.pow(denom, 1.5)
        self._prime_vertical_radius = _WGS84_A / math.sqrt(denom)

    def _project_enu(self, lat_deg, lon_deg):
        """Geodetic (degrees) -> local ENU metres about the datum."""
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        east = self._prime_vertical_radius * math.cos(self._origin_lat) * (
            lon - self._origin_lon
        )
        north = self._meridional_radius * (lat - self._origin_lat)
        return east, north

    def _yaw_to_enu(self, yaw):
        """SBG heading -> ROS ENU yaw (0 at East, CCW)."""
        if self.convention == "ned":
            return _normalize_angle(0.5 * math.pi - yaw)
        return _normalize_angle(yaw)

    def _vel_enu(self, velocity):
        """ekf_nav velocity -> world ENU (east, north)."""
        if self.convention == "ned":
            return velocity.y, velocity.x
        return velocity.x, velocity.y

    def _rate_to_enu(self, wz_body):
        """Body yaw rate on the wire -> ENU yaw rate (CCW positive)."""
        return -wz_body if self.convention == "ned" else wz_body

    @staticmethod
    def _stamp_sec(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    # --- callbacks -------------------------------------------------------

    def on_euler(self, msg):
        prev_yaw = self._yaw
        prev_stamp = self._yaw_stamp
        self._yaw = self._yaw_to_enu(msg.angle.z)
        acc = msg.accuracy.z
        self._yaw_sigma = acc if acc > 0.0 else self.default_heading_sigma
        self._yaw_stamp = self._stamp_sec(msg.header)
        self._heading_valid = bool(msg.status.heading_valid)
        # Yaw rate for the twist output, differentiated from the heading
        # stream (light low-pass: per-sample diff noise at 200 Hz would
        # jitter the controllers' feedforward).
        if prev_yaw is not None and prev_stamp is not None:
            dt = self._yaw_stamp - prev_stamp
            if 0.0 < dt < 0.5:
                raw = (self._yaw - prev_yaw + math.pi) % (2.0 * math.pi) - math.pi
                alpha = dt / (0.05 + dt)
                self._yaw_rate_estimate += alpha * (raw / dt - self._yaw_rate_estimate)
        # The fallback heading follows the EKF while the EKF's is valid, so a
        # later switch to HDT/gyro carry starts from where the EKF left off.
        # Mode >= 2 (AHRS) is required as well as the flag: below it the EKF
        # heading is a free-running integral restarted at zero.
        if self._heading_valid and self._nav_mode >= 2:
            self._yaw_dr = self._yaw
            self._yaw_dr_stamp = self._yaw_stamp
            self._yaw_dr_abs_t = self._yaw_stamp
            self._yaw_dr_abs_sigma = self._yaw_sigma

    def on_wheel_speeds(self, msg):
        # Rear-axle mean: the differential split cancels exactly in the mean
        # (same rationale as wheel_odometry.py). RPM on the wire.
        rear_rpm = 0.5 * (msg.speeds.lb_speed + msg.speeds.rb_speed)
        self._wheel_speed = rear_rpm * _RPM_TO_RAD_S * self.wheel_radius
        self._wheel_stamp = self._stamp_sec(msg.header)

    def _raw_epoch_time(self, msg):
        """ROS-time instant a receiver epoch describes (device stamp), or the
        header stamp when no anchor / an implausible mapping."""
        t_hdr = self._stamp_sec(msg.header)
        anchor = self._dev_anchor
        if anchor is None:
            return t_hdr
        d_us = (int(msg.time_stamp) - int(anchor[0])) & 0xFFFFFFFF
        if d_us >= 0x80000000:  # negative modulo 2^32
            d_us -= 0x100000000
        t_dev = anchor[1] + d_us * 1e-6
        if abs(t_dev - t_hdr) > 0.5:
            return t_hdr
        return t_dev

    def on_imu(self, msg):
        """Raw gyro: carries the fallback heading between absolute updates."""
        t = self._stamp_sec(msg.header)
        self._dev_anchor = (int(msg.time_stamp), t)
        wz = self._rate_to_enu(msg.gyro.z)
        prev_t = self._gyro_stamp
        self._gyro_wz = wz
        self._gyro_stamp = t
        if self._yaw_dr is None or self._yaw_dr_stamp is None:
            return
        # Only propagate when the EKF heading is NOT the live source (else the
        # next euler overwrites anyway and double-counting is impossible, but
        # keep the state clean); propagate from the last fallback stamp so an
        # HDT/EKF update in between is not double-integrated.
        if self._ekf_heading_live(t):
            return
        dt = t - self._yaw_dr_stamp
        if prev_t is not None and 0.0 < dt <= self.max_dt:
            self._yaw_dr = _normalize_angle(self._yaw_dr + wz * dt)
            self._yaw_dr_stamp = t

    def on_gps_vel(self, msg):
        """Raw Doppler velocity of the primary antenna (5 Hz)."""
        st = msg.status
        if st.vel_status != _GPS_SOL_COMPUTED or st.vel_type == 0:
            self._raw_vel_stamp = None
            return
        acc = max(msg.velocity_accuracy.x, msg.velocity_accuracy.y)
        if acc <= 0.0 or acc > self.raw_vel_max_accuracy:
            self._raw_vel_stamp = None
            return
        self._raw_vel_enu = self._vel_enu(msg.velocity)
        self._raw_vel_acc = acc
        self._raw_vel_stamp = self._raw_epoch_time(msg)

    def on_gps_pos(self, msg):
        """Raw receiver fix (5 Hz): rtk-grade epochs correct the raw rung."""
        t = self._raw_epoch_time(msg)
        good = (
            msg.status.status == _GPS_SOL_COMPUTED
            and msg.status.type >= self.raw_fix_min_type_rtk
        )
        if not good:
            self._raw_fix_stamp = None
            self._raw_fix_enu = None
            self._raw_fix_rtk = False
            self._raw_chain_ref = None
            return
        if self._origin_lat is None:
            # No datum yet (raw start before any EKF fix): any origin serves
            # the deltas; a later EKF georeference keeps this one.
            self._set_origin(msg.latitude, msg.longitude)
        east, north = self._project_enu(msg.latitude, msg.longitude)
        # Remove the antenna lever arm so the fix chain measures the base
        # point the odometry integrates (needs the fallback heading).
        yaw, _ = self._heading_source(t)
        if yaw is not None:
            lx, ly = self.antenna_lever_arm_x, self.antenna_lever_arm_y
            east -= math.cos(yaw) * lx - math.sin(yaw) * ly
            north -= math.sin(yaw) * lx + math.cos(yaw) * ly
        ref = self._raw_chain_ref
        dr_now = self._raw_dr_at(t) if self._raw_active else None
        if (
            ref is not None and dr_now is not None and self._odom_xy is not None
            and 0.0 < t - ref[0] <= self.raw_gnss_timeout
        ):
            # Replace the Doppler DR displacement over [ref, t] with the RTK
            # epoch delta; the residual is a few cm at most (Doppler held
            # constant across a 0.2 s epoch), so this stays jump-free.
            de = (east - ref[1]) - (dr_now[0] - ref[3][0])
            dn = (north - ref[2]) - (dr_now[1] - ref[3][1])
            self._odom_xy[0] += de
            self._odom_xy[1] += dn
        self._raw_fix_stamp = t
        self._raw_fix_enu = (east, north)
        self._raw_fix_rtk = True
        # The chain only starts from an epoch taken while the raw rung is
        # integrating: a reference from before the rung would make the first
        # correction span the source transition (double-counting the EKF's
        # part of the epoch interval).
        self._raw_chain_ref = (t, east, north, dr_now) if dr_now is not None else None

    def on_gps_hdt(self, msg):
        """Dual-antenna heading (5 Hz): absolute fallback-heading source."""
        if (msg.status & _GPS_HDT_STATUS_MASK) != _GPS_SOL_COMPUTED:
            return
        acc_deg = msg.true_heading_acc
        if acc_deg <= 0.0 or acc_deg > self.hdt_max_accuracy_deg:
            return
        t = self._raw_epoch_time(msg)
        # true_heading is in degrees on the wire, same convention as euler.
        raw_enu = self._yaw_to_enu(math.radians(msg.true_heading))
        self._hdt_yaw_enu_raw = raw_enu
        self._hdt_stamp = t
        self._hdt_sigma = math.radians(acc_deg)
        if self._ekf_heading_live(t):
            # Learn the installation offset while the EKF is fusing the HDT
            # (mode 4, hdt_used): circular running mean of (ekf - hdt).
            if (self._nav_mode >= 4 and self._nav_hdt_used
                    and not math.isfinite(self.hdt_yaw_offset_deg)):
                d = _normalize_angle(self._yaw - raw_enu)
                vec = self._hdt_offset_vec or [0.0, 0.0]
                vec[0] += math.cos(d)
                vec[1] += math.sin(d)
                self._hdt_offset_vec = vec
                self._hdt_offset_n += 1
                if self._hdt_offset_n >= 5:
                    learned = math.atan2(vec[1], vec[0])
                    if self._hdt_offset is None:
                        self.get_logger().info(
                            f"HDT->vehicle heading offset learned: "
                            f"{math.degrees(learned):+.1f} deg "
                            f"(n={self._hdt_offset_n})")
                    self._hdt_offset = learned
            return
        if self._hdt_offset is None:
            return
        # EKF heading is not live: the HDT (+offset) IS the heading.
        self._yaw_dr = _normalize_angle(raw_enu + self._hdt_offset)
        self._yaw_dr_stamp = t
        self._yaw_dr_abs_t = t
        self._yaw_dr_abs_sigma = self._hdt_sigma

    # --- source selection --------------------------------------------------

    def _ekf_heading_live(self, t):
        return (
            self._heading_valid
            and self._nav_mode >= 2
            and self._yaw_stamp is not None
            and abs(t - self._yaw_stamp) <= _HEADING_STALE_S
        )

    def _heading_source(self, t):
        """
        (yaw_enu, sigma) of the best heading at t, or (None, None).

        EKF heading when valid and fresh; else the fallback heading (HDT +
        offset, gyro-carried) while its last absolute update is recent enough
        and something is actually carrying it (fresh gyro or fresh HDT).
        """
        if self._ekf_heading_live(t):
            return self._yaw, self._yaw_sigma
        if self._yaw_dr is None or self._yaw_dr_abs_t is None:
            return None, None
        # Receiver epochs are stamped independently of the EKF ticks and may
        # land a few ms AFTER the nav stamp being served: a slightly negative
        # age is "fresh", not "from the future".
        abs_age = max(0.0, t - self._yaw_dr_abs_t)
        if t - self._yaw_dr_abs_t < -1.0 or abs_age > self.gyro_only_heading_timeout:
            return None, None
        gyro_fresh = (
            self._gyro_stamp is not None
            and abs(t - self._gyro_stamp) <= self.imu_timeout
        )
        hdt_fresh = (
            self._hdt_stamp is not None
            and abs(t - self._hdt_stamp) <= self.raw_gnss_timeout
        )
        if not (gyro_fresh or hdt_fresh):
            return None, None
        sigma = self._yaw_dr_abs_sigma + self.gyro_yaw_drift_rate * abs_age
        return self._yaw_dr, sigma

    def _wheels_fresh(self, t):
        return (
            self.ahrs_fallback_enable
            and self._wheel_stamp is not None
            and abs(t - self._wheel_stamp) <= self.wheel_speed_timeout
        )

    def _can_dead_reckon(self, t, mode):
        """
        Check that wheel + heading dead reckoning has live inputs.

        The heading is the EKF's when it is valid (mode >= 2 by definition)
        or the HDT/gyro fallback otherwise; with neither, wheels alone are
        refused: integrating speed under a freely drifting heading walks the
        pose confidently in the wrong direction.
        """
        del mode  # the heading validity, not the mode number, is the gate
        if not self._wheels_fresh(t):
            return False
        yaw, _ = self._heading_source(t)
        return yaw is not None

    def _raw_grade(self, t):
        """'rtk' | 'doppler' | None: raw-GNSS rung availability at t."""
        if not self.raw_gnss_fallback_enable:
            return None
        if self._raw_vel_stamp is None or abs(t - self._raw_vel_stamp) > self.raw_gnss_timeout:
            return None
        yaw, _ = self._heading_source(t)
        if yaw is None:
            return None
        if (
            self._raw_fix_rtk and self._raw_fix_stamp is not None
            and abs(t - self._raw_fix_stamp) <= self.raw_gnss_timeout
        ):
            return "rtk"
        return "doppler"

    def _update_stationary(self, t):
        """
        Latch/unlatch the ZUPT standstill state from the wheel speed.

        Wheel speed is the only signal that does not lie at rest (the encoder
        reads a true zero while the INS velocity reads noise). With no fresh
        wheel sample the gate stays off, so the bridge behaves exactly as before
        until the CAN wheel-speed driver exists. Hysteresis: enter below
        ``stationary_enter_speed``, leave only above ``stationary_exit_speed``.
        """
        fresh = (
            self._wheel_stamp is not None
            and abs(t - self._wheel_stamp) <= self.wheel_speed_timeout
        )
        if not fresh:
            self._stationary = False
            return
        ws = abs(self._wheel_speed)
        if self._stationary:
            if ws > self.stationary_exit_speed:
                self._stationary = False
        elif ws < self.stationary_enter_speed:
            self._stationary = True

    def _raw_base_velocity(self, yaw):
        """Doppler antenna velocity -> base-point velocity (ENU), lever arm removed."""
        ve, vn = self._raw_vel_enu
        lx, ly = self.antenna_lever_arm_x, self.antenna_lever_arm_y
        if lx == 0.0 and ly == 0.0:
            return ve, vn
        # v_ant = v_base + w x l  ->  in body: (−w*ly, w*lx); rotate to ENU.
        w = self._gyro_wz if (
            self._gyro_stamp is not None) else self._yaw_rate_estimate
        bx, by = -w * ly, w * lx
        c, s = math.cos(yaw), math.sin(yaw)
        return ve - (c * bx - s * by), vn - (s * bx + c * by)

    def on_nav(self, msg):
        mode = msg.status.solution_mode
        stamp = msg.header
        t = stamp.stamp.sec + stamp.stamp.nanosec * 1e-9
        self._nav_mode = mode
        self._nav_hdt_used = bool(msg.status.gps1_hdt_used)

        if self._yaw is None and self._yaw_dr is None:
            self._warn_throttle("no SBG heading (ekf_euler / gps_hdt) received yet")
            return

        # A backward (or duplicate) stamp makes graph_slam_node's
        # recordRawOdometry clear its whole interpolation buffer, orphaning
        # the cone frames waiting in it — drop the message here instead.
        if self._last_pub_nav_t is not None and t <= self._last_pub_nav_t:
            self._warn_throttle(
                f"non-monotonic nav stamp ({t:.3f} <= {self._last_pub_nav_t:.3f}), "
                "dropping"
            )
            return

        # Startup gate: georeference the origin from the first good absolute fix.
        if not self._started:
            start_yaw, _ = self._heading_source(t)
            have_absolute = (
                msg.status.position_valid and mode >= self.start_min_solution_mode
                and start_yaw is not None
            )
            if have_absolute:
                if self._origin_lat is None:
                    self._set_origin(msg.latitude, msg.longitude)
                east0, north0 = self._project_enu(msg.latitude, msg.longitude)
                self._odom_xy = [east0, north0]
                self._odom_yaw = start_yaw
                self._last_int_t = t
                self._started = True
                self._georeferenced = True
                self.get_logger().info(
                    f"Started (RTK): origin datum "
                    f"({self._origin_lat*180/math.pi:.7f}, "
                    f"{self._origin_lon*180/math.pi:.7f}), ENU=(0,0)"
                )
            elif self.allow_dr_start and start_yaw is not None and (
                self._can_dead_reckon(t, mode) or self._raw_grade(t) is not None
            ):
                # DR-only start: no absolute fix yet (covered boot, cold GNSS,
                # or an EKF that has not aligned), but a heading and a speed
                # source (wheels or raw GNSS) are live. SLAM consumes CarState
                # as deltas, so a provisional (0,0) origin is harmless — start
                # feeding motion NOW instead of leaving the whole stack with
                # no state input. The /localization/gnss_odom anchor stays gated until a
                # real fix georeferences the origin (below).
                self._odom_xy = [0.0, 0.0]
                self._odom_yaw = start_yaw
                self._last_int_t = t
                self._started = True
                self._georeferenced = False
                src = "wheels" if self._can_dead_reckon(t, mode) else "raw GNSS"
                self.get_logger().warn(
                    f"Started (DR-only, {src}): no absolute fix yet; motion "
                    "output on a provisional origin, GNSS anchor held until a "
                    "fix arrives"
                )
            else:
                self._warn_throttle(
                    f"waiting to start (mode={mode}, "
                    f"position_valid={msg.status.position_valid}, "
                    f"dr={self._can_dead_reckon(t, mode)}, "
                    f"raw={self._raw_grade(t)})"
                )
                self._publish_health(mode, msg.status, anchored=False, started=False)
                self._publish_status_board(
                    t, mode, None, anchored=False, fault=False, started=False
                )
                return
        elif not self._georeferenced and (
            msg.status.position_valid and mode >= self.start_min_solution_mode
        ):
            # A real fix arrived after a DR-only start: georeference the datum
            # so /localization/gnss_odom can begin. The absolute origin is taken here, and
            # the CarState motion frame stays as it was — SLAM's own map frame
            # is provisional either way, and its GNSS prior suppress/rearm
            # machinery reconciles the resulting offset.
            if self._origin_lat is None:
                self._set_origin(msg.latitude, msg.longitude)
            self._georeferenced = True
            self.get_logger().info(
                f"Georeferenced after DR start: origin datum "
                f"({self._origin_lat*180/math.pi:.7f}, "
                f"{self._origin_lon*180/math.pi:.7f})"
            )

        # Below the minimum odometry mode the ENU nav solution is unusable.
        # Degrade to a fallback rung on the SAME integrator (so the position
        # stays continuous through mode flapping); fault — stop feeding SLAM
        # — only when every rung is dead.
        below_min_odom = mode < self.min_odom_solution_mode
        raw_grade = self._raw_grade(t)
        wheels_ok = self._can_dead_reckon(t, mode)
        if below_min_odom and not wheels_ok and raw_grade is None:
            self._fault_tick(t, stamp, mode, msg.status,
                             f"solution_mode={mode}: below min_odom, no fallback")
            return

        # --- relative odometry: integrate (jump-free) --------------------
        # ENU position needs a georeferenced datum; a DR-only start has none
        # yet (and its anchor is gated off below), so skip the projection.
        east, north = (
            self._project_enu(msg.latitude, msg.longitude)
            if self._georeferenced else (0.0, 0.0)
        )
        dt = t - self._last_int_t if self._last_int_t is not None else 0.0
        self._last_int_t = t
        self._update_stationary(t)
        velocity_ok = (
            bool(msg.status.velocity_valid) and mode >= 3 and not below_min_odom
        )
        dt_ok = 0.0 < dt <= self.max_dt
        dead_reckoning = False
        raw_used = None
        held = False
        body_vx = None
        body_vy = None
        yaw_src, yaw_src_sigma = self._heading_source(t)
        if self._stationary:
            # ZUPT: the wheels report a standstill. Freeze BOTH position and
            # heading -- an Ackermann car cannot translate or yaw with the
            # wheels stopped, so the nav-velocity noise (integrated -> phantom
            # travel) and the unaided heading drift/jumps are pure error. The
            # pose is held identical between frames, so the keyframe delta stays
            # ~0 and graph_slam's frozen-input gate skips the phantom keyframes.
            # Heading is deliberately NOT refreshed from the heading source
            # below, and the body twist is published as zero (the car is
            # genuinely stopped, unlike the dead-reckoning branch).
            self._last_vel_enu = None
            body_vx = body_vy = 0.0
        elif velocity_ok and dt_ok:
            ve, vn = self._vel_enu(msg.velocity)
            prev = self._last_vel_enu or (ve, vn)
            self._odom_xy[0] += 0.5 * (prev[0] + ve) * dt
            self._odom_xy[1] += 0.5 * (prev[1] + vn) * dt
            self._last_vel_enu = (ve, vn)
        elif raw_grade == "rtk" and dt_ok:
            raw_used = self._raw_tick(t, dt, yaw_src, raw_grade)
            body_vx, body_vy = self._body_twist_from_enu(
                self._raw_base_velocity(yaw_src), yaw_src)
        elif wheels_ok and dt_ok:
            # Wheel + heading dead reckoning: covers mode 2 and momentary
            # velocity dropouts at mode >= 3. Euler at the EKF rate is
            # sub-mm accurate per step; drop the stale ENU velocity so
            # re-entry does not trapezoid across the gap.
            self._odom_xy[0] += self._wheel_speed * math.cos(yaw_src) * dt
            self._odom_xy[1] += self._wheel_speed * math.sin(yaw_src) * dt
            self._last_vel_enu = None
            dead_reckoning = True
            body_vx, body_vy = self._wheel_speed, 0.0
        elif raw_grade == "doppler" and dt_ok:
            raw_used = self._raw_tick(t, dt, yaw_src, raw_grade)
            body_vx, body_vy = self._body_twist_from_enu(
                self._raw_base_velocity(yaw_src), yaw_src)
        else:
            # Hold position (gap / no fallback) - only heading stays fresh.
            # The frozen pose must not reach the graph with the tier sigma:
            # the car may still be moving, and confident zero-motion edges
            # are exactly what warps the map. A hold is time-limited: past
            # held_max_sec it becomes a fault so downstream watchdogs see the
            # silence instead of a frozen pose that keeps flowing.
            self._last_vel_enu = None
            held = True
            # Any hold is a (short) blind gap: the pose did not move while
            # the car may have; the first message after it must be free.
            self._blind_gap = True
            if self._held_since is None:
                self._held_since = t
            elif t - self._held_since > self.held_max_sec:
                self._fault_tick(t, stamp, mode, msg.status,
                                 f"held for {t - self._held_since:.1f} s: no motion source")
                return
        if not held:
            self._held_since = None
        self._raw_active = raw_used is not None
        if raw_used is None:
            # Break the rtk fix chain when the raw rung is not integrating:
            # the pure-DR track no longer covers the epoch interval.
            self._raw_chain_ref = None
            self._raw_dr_hist.clear()
        if not self._stationary and yaw_src is not None:
            self._odom_yaw = yaw_src

        # Heading staleness: the DR path hard-gates on heading age, but the
        # velocity path would republish an arbitrarily old yaw otherwise.
        if yaw_src is None:
            yaw_sigma = _HUGE_SIGMA
        elif self._ekf_heading_live(t):
            yaw_sigma = self._yaw_sigma
        else:
            yaw_sigma = yaw_src_sigma

        if held:
            odom_sigma = _HUGE_SIGMA
        elif dead_reckoning:
            odom_sigma = max(self.odom_sigma_mode2, self._odom_sigma_for_mode(mode))
        elif raw_used == "rtk":
            odom_sigma = self.odom_sigma_raw_rtk
        elif raw_used == "doppler":
            odom_sigma = self.odom_sigma_raw_doppler
        else:
            odom_sigma = self._odom_sigma_for_mode(mode)
        # Correction-grade honesty: solution mode alone is too coarse — at
        # mode 4 with SINGLE corrections the realized position error is a
        # metres-scale GM excursion while the mode tier still claims 0.05 m.
        # SLAM then trusts stiff wrong motion, and the map pays (2026-07-18
        # F1 autocross: 56 ghosts from exactly this). The reported velocity
        # accuracy tracks the correction grade, so scale the per-edge trust
        # with its 1 s displacement error. On the raw rung the receiver's
        # Doppler accuracy plays the same role.
        # Only the source that was integrated may inflate the sigma: the
        # EKF's velocity_accuracy is 750 m/s (an "invalid" marker) while it
        # is re-initialising, and applying that to a wheel or raw tick would
        # publish a free edge for a rung that is in fact tracking.
        if raw_used is not None:
            vel_acc = self._raw_vel_acc or 0.0
        elif not (held or dead_reckoning or self._stationary):
            vel_acc = max(msg.velocity_accuracy.x, msg.velocity_accuracy.y)
        else:
            vel_acc = 0.0
        if vel_acc > 0.0:
            odom_sigma = max(odom_sigma, vel_acc * self.vel_accuracy_horizon_sec)
        # Blind gap: the first message after a fault (or a timed-out hold)
        # must not claim a confident tiny delta across a gap during which the
        # car may have moved. graph_slam mints a keyframe on it (dt >=
        # keyframe_max_dt) and its edge should be free.
        if self._blind_gap and not held:
            odom_sigma = _HUGE_SIGMA
            self._blind_gap = False
        self._last_pub_nav_t = t
        # Dead reckoning has no ENU velocity, but the twist must NOT read as
        # v=0: ego_odom passes this twist through to the controllers, and a
        # zero there makes the speed loop wind full throttle for the whole
        # outage (2026-07-19 injection run: cmd saturated +2.5, 8 -> 14.5 m/s
        # in 10 s). The wheel speed / raw velocity that drives the integrator
        # IS the body velocity — publish it. A hold carries the last speed.
        if held:
            body_vx, body_vy = self._last_body_vx, 0.0
        self._publish_car_state(
            stamp, self._odom_xy, self._odom_yaw, odom_sigma, yaw_sigma,
            body_vx=body_vx, body_vy=body_vy,
            stationary=self._stationary, held=held,
            # Without a live EKF heading its differentiated yaw rate is
            # garbage (mode <= 1 restarts the integral); use the raw gyro.
            yaw_rate=(
                self._gyro_wz
                if (not self._ekf_heading_live(t) and self._gyro_stamp is not None
                    and abs(t - self._gyro_stamp) <= self.imu_timeout)
                else None),
        )
        self._last_car_state_t = t

        # --- global anchor: raw absolute ENU + mode-tiered covariance ----
        # Never anchor before the origin datum is georeferenced: a DR-only
        # start has a provisional (0,0) origin, so an ENU position projected
        # against a not-yet-set datum would be meaningless.
        anchored = (
            self._georeferenced
            and msg.status.position_valid
            and mode >= self.absolute_min_solution_mode
        )
        holdoff = False
        if anchored:
            # The driver reports 0.0 accuracy for "not computed"; that must
            # not map to the tightest tier.
            acc_e, acc_n = msg.position_accuracy.x, msg.position_accuracy.y
            pos_sigma_e = acc_e if acc_e > 0.0 else _HUGE_SIGMA
            pos_sigma_n = acc_n if acc_n > 0.0 else _HUGE_SIGMA
            if self.convention == "ned":
                pos_sigma_e, pos_sigma_n = pos_sigma_n, pos_sigma_e

            # Refix holdoff (see the parameter block): a tier improvement —
            # re-anchoring after a mode dip, or single/float -> RTK while
            # anchored — starts the window. The tier is tracked on the RAW
            # accuracies so an expiring holdoff cannot retrigger itself.
            tier = self._correction_tier(max(pos_sigma_e, pos_sigma_n))
            prev_tier = self._anchor_tier
            improved = (
                tier > prev_tier
                if prev_tier is not None
                else self._anchor_ever
            )
            if improved:
                self._holdoff_until = t + self.refix_holdoff_sec
                self.get_logger().info(
                    f"GNSS refix (tier {prev_tier} -> {tier}): anchor sigma "
                    f"held at >= {self.refix_holdoff_sigma:.1f} m for "
                    f"{self.refix_holdoff_sec:.1f} s while the fix pulls in"
                )
            self._anchor_tier = tier
            self._anchor_ever = True
            if self._holdoff_until is not None:
                if t < self._holdoff_until:
                    holdoff = True
                    pos_sigma_e = max(pos_sigma_e, self.refix_holdoff_sigma)
                    pos_sigma_n = max(pos_sigma_n, self.refix_holdoff_sigma)
                else:
                    self._holdoff_until = None
        else:
            pos_sigma_e = pos_sigma_n = _HUGE_SIGMA
            self._anchor_tier = None
        # Before georeferencing (DR-only start) the ENU position is a
        # provisional (0,0), not a real fix — suppress the anchor entirely
        # rather than publish a meaningless huge-sigma point.
        if self._georeferenced:
            self._publish_gnss_odom(stamp, east, north, self._odom_yaw,
                                    pos_sigma_e, pos_sigma_n, yaw_sigma)

        self._publish_health(
            mode, msg.status, anchored=anchored, started=True,
            dead_reckoning=dead_reckoning, stationary=self._stationary,
            raw_grade=raw_used, held=held,
        )
        self._publish_markers(
            stamp, east, north, max(pos_sigma_e, pos_sigma_n), mode,
            anchored=anchored, fault=False,
        )
        self._publish_status_board(
            t, mode, max(pos_sigma_e, pos_sigma_n),
            anchored=anchored, fault=False, started=True,
            dead_reckoning=dead_reckoning, holdoff=holdoff,
            stationary=self._stationary, raw_grade=raw_used, held=held,
        )

    def _raw_tick(self, t, dt, yaw, grade):
        """Advance the integrator one tick on the raw rung (Doppler DR)."""
        ve, vn = self._raw_base_velocity(yaw)
        de, dn = ve * dt, vn * dt
        self._odom_xy[0] += de
        self._odom_xy[1] += dn
        self._last_vel_enu = None
        # Pure-DR track for the rtk epoch correction (see _raw_chain_ref).
        if not self._raw_active:
            self._raw_dr_hist.clear()
            self._raw_dr_pos = [0.0, 0.0]
            self._raw_dr_hist.append((t - dt, 0.0, 0.0))
        self._raw_dr_pos[0] += de
        self._raw_dr_pos[1] += dn
        self._raw_dr_hist.append((t, self._raw_dr_pos[0], self._raw_dr_pos[1]))
        while len(self._raw_dr_hist) > 2 and self._raw_dr_hist[0][0] < t - 2.0:
            self._raw_dr_hist.popleft()
        return grade

    def _raw_dr_at(self, t):
        """Pure Doppler-DR position at t (interpolated; short extrapolation)."""
        hist = self._raw_dr_hist
        if not hist or t < hist[0][0] - 1e-6:
            return None
        if t >= hist[-1][0]:
            # Epoch stamped after the last tick (by up to one tick): carry
            # forward with the current Doppler velocity, which is exactly what
            # the next tick will integrate over that span.
            gap = t - hist[-1][0]
            if gap > self.max_dt or self._raw_vel_enu is None:
                return None
            ve, vn = self._raw_base_velocity(self._odom_yaw)
            return (hist[-1][1] + ve * gap, hist[-1][2] + vn * gap)
        # Binary-free linear scan from the back: the epoch is always recent.
        for i in range(len(hist) - 1, 0, -1):
            t0, x0, y0 = hist[i - 1]
            t1, x1, y1 = hist[i]
            if t0 <= t <= t1:
                if t1 - t0 <= 0.0:
                    return (x1, y1)
                a = (t - t0) / (t1 - t0)
                return (x0 + a * (x1 - x0), y0 + a * (y1 - y0))
        return None

    @staticmethod
    def _body_twist_from_enu(vel_enu, yaw):
        ve, vn = vel_enu
        c, s = math.cos(-yaw), math.sin(-yaw)
        return c * ve - s * vn, s * ve + c * vn

    def _fault_tick(self, t, stamp, mode, status, reason):
        """No usable motion source: hold output, keep the clock, report."""
        self._warn_throttle(f"{reason}: holding output (FAULT)")
        self._publish_health(mode, status, anchored=False, started=True, fault=True)
        if self._odom_xy is not None:
            self._publish_markers(
                stamp, self._odom_xy[0], self._odom_xy[1],
                _HUGE_SIGMA, mode, anchored=False, fault=True,
            )
        self._publish_status_board(
            t, mode, None, anchored=False, fault=True, started=True
        )
        # Keep the integration clock ticking so re-entry does not step
        # across the whole outage in one dt (max_dt would drop it anyway).
        self._last_int_t = t
        self._last_vel_enu = None
        self._raw_active = False
        self._raw_chain_ref = None
        self._raw_dr_hist.clear()
        # _held_since is deliberately kept: a hold that timed out stays a
        # fault until a real motion source publishes again.
        self._blind_gap = True

    # --- publishing ------------------------------------------------------

    @staticmethod
    def _correction_tier(sigma):
        """Reported-accuracy tier: 2 RTK fixed / 1 float / 0 single-point."""
        if sigma <= 0.05:
            return 2
        if sigma <= 0.5:
            return 1
        return 0

    def _odom_sigma_for_mode(self, mode):
        if mode >= 4:
            return self.odom_sigma_mode4
        if mode == 3:
            return self.odom_sigma_mode3
        return self.odom_sigma_mode2

    def _publish_car_state(self, header, xy, yaw, trans_sigma, yaw_sigma,
                           body_vx=None, body_vy=None, stationary=False,
                           held=False, yaw_rate=None):
        state = CarState()
        state.header = header
        state.header.frame_id = self.world_frame
        state.child_frame_id = self.base_frame
        state.pose.pose.position.x = xy[0]
        state.pose.pose.position.y = xy[1]
        state.pose.pose.orientation.z = math.sin(0.5 * yaw)
        state.pose.pose.orientation.w = math.cos(0.5 * yaw)
        state.pose.covariance[0] = trans_sigma * trans_sigma
        state.pose.covariance[7] = trans_sigma * trans_sigma
        state.pose.covariance[35] = yaw_sigma * yaw_sigma
        # Body twist: consumers downstream of SLAM (pure pursuit / TMPC
        # speed loops, frenet odom) read velocity from this message. An
        # empty twist reads as v=0 — the controller's speed loop runs open
        # and launches the car off track (2026-07-18 full-pipeline autopsy).
        # body_vx overrides the ENU-derived twist for the dead-reckoning
        # tiers, where the ENU nav velocity is exactly what is unavailable.
        if body_vx is not None:
            state.twist.twist.linear.x = body_vx
            state.twist.twist.linear.y = body_vy if body_vy is not None else 0.0
        else:
            cos_y = math.cos(-yaw)
            sin_y = math.sin(-yaw)
            ve, vn = self._last_vel_enu or (0.0, 0.0)
            state.twist.twist.linear.x = cos_y * ve - sin_y * vn
            state.twist.twist.linear.y = sin_y * ve + cos_y * vn
        # At a ZUPT standstill the car is genuinely stopped: the differentiated
        # yaw rate is just the unaided heading drift/jumps, so report zero
        # rotation too (not only zero linear velocity).
        if stationary:
            state.twist.twist.angular.z = 0.0
        elif yaw_rate is not None:
            state.twist.twist.angular.z = yaw_rate
        else:
            state.twist.twist.angular.z = self._yaw_rate_estimate
        # Twist trust: a held pose carries a stale speed — say so. Nothing
        # reads this yet, but it is the honest contract for a consumer that
        # wants to age the speed feedback (ego_odom passes the twist through).
        twist_var = _HUGE_SIGMA * _HUGE_SIGMA if held else 0.0
        state.twist.covariance[0] = twist_var
        state.twist.covariance[7] = twist_var
        state.twist.covariance[35] = twist_var
        self._last_body_vx = state.twist.twist.linear.x
        self.car_state_pub.publish(state)

    def _publish_gnss_odom(
        self, header, east, north, yaw, sigma_e, sigma_n, yaw_sigma
    ):
        odom = Odometry()
        odom.header = header
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.orientation.z = math.sin(0.5 * yaw)
        odom.pose.pose.orientation.w = math.cos(0.5 * yaw)
        odom.pose.covariance[0] = sigma_e * sigma_e
        odom.pose.covariance[7] = sigma_n * sigma_n
        odom.pose.covariance[35] = yaw_sigma * yaw_sigma
        self.gnss_odom_pub.publish(odom)

    @staticmethod
    def _state_style(mode, sigma, anchored, fault, started, dead_reckoning=False,
                     stationary=False, raw_grade=None, held=False):
        """(rgb, state word) shared by the map markers and the HUD board."""
        if not started:
            return (0.55, 0.55, 0.55), "WAITING FOR FIX"
        if fault:
            return (0.9, 0.1, 0.1), "FAULT"
        if stationary:
            return (0.2, 0.6, 0.95), "STATIONARY (ZUPT)"
        if held:
            return (0.9, 0.3, 0.1), "HOLD (no motion src)"
        if raw_grade == "rtk":
            return (0.95, 0.6, 0.1), "RAW GNSS DR (RTK)"
        if raw_grade == "doppler":
            return (0.95, 0.5, 0.1), "RAW GNSS DR (Doppler)"
        if dead_reckoning:
            return (0.95, 0.6, 0.1), "DEAD RECKONING"
        if not anchored:
            return (0.55, 0.55, 0.55), "NO ANCHOR (INS)"
        if sigma <= 0.05:
            return (0.1, 0.8, 0.2), "RTK FIXED"
        if sigma <= 0.5:
            return (0.9, 0.8, 0.1), "RTK FLOAT"
        return (0.95, 0.5, 0.1), "SINGLE POINT"

    def _publish_status_board(
        self, t, mode, sigma, anchored, fault, started, dead_reckoning=False,
        holdoff=False, stationary=False, raw_grade=None, held=False,
    ):
        """Render a fixed HUD below the ATE overlay via rviz_2d_overlay."""
        if self.overlay_pub is None:
            return
        if self._last_board_t is not None and (t - self._last_board_t) < 0.2:
            return
        self._last_board_t = t

        rgb, state = self._state_style(
            mode, sigma, anchored, fault, started, dead_reckoning, stationary,
            raw_grade, held)
        mode_names = {0: "UNINITIALIZED", 1: "VERT_GYRO", 2: "AHRS",
                      3: "NAV_VELOCITY", 4: "NAV_POSITION"}
        if holdoff:
            prior = "HOLDOFF (refix)"
        elif anchored and sigma is not None:
            prior = "ACTIVE" if sigma <= 0.5 else "DROPPED (sigma gate)"
        else:
            prior = "-"
        show_sigma = anchored and sigma is not None
        lines = [
            f"GNSS  {state}",
            f"mode  {mode} {mode_names.get(mode, '?')}",
            f"sigma {sigma:.3f} m" if show_sigma else "sigma -",
            f"prior {prior}",
        ]

        ov = OverlayText()
        ov.action = OverlayText.ADD
        # Sits just below the stack HUD board (left-aligned at x12, y56, height
        # 24 + 19*9 = 195 -> bottom ~251, published by hyu_planning_bringup
        # stack_hud.py). Keep positions in sync when moving either.
        ov.width = 220
        ov.height = 96
        ov.horizontal_distance = 12
        ov.vertical_distance = 256
        ov.horizontal_alignment = OverlayText.LEFT
        ov.vertical_alignment = OverlayText.TOP
        ov.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.55)
        ov.fg_color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)
        ov.line_width = 2
        ov.text_size = 13.0
        ov.font = "DejaVu Sans Mono"
        ov.text = "\n".join(lines)
        self.overlay_pub.publish(ov)

    def _publish_markers(self, header, east, north, sigma, mode, anchored, fault):
        """Render RViz GNSS state as a fix disc plus status text."""
        if self.marker_pub is None:
            return
        t = header.stamp.sec + header.stamp.nanosec * 1e-9
        if self._last_marker_t is not None and (t - self._last_marker_t) < 0.2:
            return
        self._last_marker_t = t

        rgb, _ = self._state_style(mode, sigma, anchored, fault, started=True)

        markers = MarkerArray()

        disc = Marker()
        disc.header.stamp = header.stamp
        disc.header.frame_id = self.world_frame
        disc.ns = "gnss"
        disc.id = 0
        disc.type = Marker.CYLINDER
        disc.action = Marker.ADD
        disc.pose.position.x = east
        disc.pose.position.y = north
        disc.pose.position.z = 0.05
        disc.pose.orientation.w = 1.0
        # Disc diameter = 2-sigma circle, clamped so RTK stays visible and a
        # dropped anchor does not draw a kilometre-wide disc.
        diameter = min(max(2.0 * sigma, 0.3), 5.0) if anchored else 0.6
        disc.scale.x = disc.scale.y = diameter
        disc.scale.z = 0.1
        disc.color.r, disc.color.g, disc.color.b = rgb
        disc.color.a = 0.6
        disc.lifetime.sec = 2
        markers.markers.append(disc)

        self.marker_pub.publish(markers)

    def _publish_health(self, mode, status, anchored, started,
                        dead_reckoning=False, stationary=False, raw_grade=None,
                        held=False, fault=False):
        names = {0: "UNINITIALIZED", 1: "VERTICAL_GYRO", 2: "AHRS",
                 3: "NAV_VELOCITY", 4: "NAV_POSITION"}
        if not started:
            level, text = DiagnosticStatus.WARN, "waiting for absolute fix"
        elif fault:
            level, text = DiagnosticStatus.ERROR, "fault: no usable motion source"
        elif stationary:
            level, text = DiagnosticStatus.OK, "stationary: ZUPT hold (pose frozen)"
        elif held:
            level, text = DiagnosticStatus.WARN, "hold: no motion source (pose frozen, sigma huge)"
        elif raw_grade is not None:
            level, text = (
                DiagnosticStatus.WARN,
                f"degraded: raw GNSS dead reckoning ({raw_grade}), INS EKF unusable",
            )
        elif dead_reckoning:
            level, text = (
                DiagnosticStatus.WARN,
                "degraded: wheel+heading dead reckoning, no GNSS",
            )
        elif anchored:
            level, text = DiagnosticStatus.OK, "GNSS-anchored (mode 4)"
        elif mode >= self.min_odom_solution_mode:
            level, text = DiagnosticStatus.WARN, "degraded: odometry only, no anchor"
        else:
            level, text = DiagnosticStatus.ERROR, "fault: below min odometry mode"

        st = DiagnosticStatus()
        st.level = level
        st.name = "sbg_odometry_bridge"
        st.message = f"{names.get(mode, mode)}: {text}"
        st.values = [
            KeyValue(key="solution_mode", value=str(mode)),
            KeyValue(key="position_valid", value=str(bool(status.position_valid))),
            KeyValue(key="velocity_valid", value=str(bool(status.velocity_valid))),
            KeyValue(key="heading_valid", value=str(bool(status.heading_valid))),
            KeyValue(key="anchored", value=str(bool(anchored))),
            KeyValue(key="motion_source", value=(
                "none" if not started else
                "fault" if fault else
                "zupt" if stationary else
                "hold" if held else
                f"raw_gnss_{raw_grade}" if raw_grade else
                "wheels" if dead_reckoning else
                "ekf" if mode >= self.min_odom_solution_mode else "fault")),
            KeyValue(key="hdt_offset_deg", value=(
                f"{math.degrees(self._hdt_offset):.1f}"
                if self._hdt_offset is not None else "unknown")),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.health_pub.publish(arr)

    def _warn_throttle(self, message):
        self.get_logger().warn(message, throttle_duration_sec=2.0)


def main():
    rclpy.init(args=sys.argv)
    node = SbgOdometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
