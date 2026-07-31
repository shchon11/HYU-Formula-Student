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
    odometry**. Pose is the ENU velocity+heading *integrated* here, so it is
    jump-free even when RTK re-acquisition makes the absolute position step.
    graph_slam_node differences this between keyframes. Below
    ``min_odom_solution_mode`` (default 3) the ENU velocity is unusable, but
    mode 2 (AHRS) still has a valid heading by definition — so the SAME
    integrator falls back to wheel-speed dead reckoning (rear mean x AHRS
    heading, sigma widened to ``odom_sigma_mode2``) instead of stopping.
    Cutting the motion input here is what used to kill SLAM: the pose froze
    while the car kept moving, and the re-entry jump exceeded the cone
    association gate, after which nothing could pull the pose back. One
    integrator, source-switched per tick, keeps the position continuous
    through arbitrary mode flapping. Publication stops only when dead
    reckoning is impossible too (mode <= 1: heading drifts freely; or stale
    wheel speeds / heading).
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
there; otherwise ``allow_dr_start`` (default true) starts on wheels + AHRS
heading with a provisional (0,0) origin and holds the absolute channel until a
fix arrives, because SLAM differences CarState and does not care where zero
is. Mode 1 (VERTICAL_GYRO) is the floor and is refused on purpose: heading
drifts freely there, and integrating wheel speed under it walks the pose
confidently in the wrong direction. A bag recorded without a fix therefore
never starts this node -- that is the design working, not a gate to relax.
Heading
(NED 0=North CW / ENU 0=East CCW) is converted to ROS ENU yaw; set
``frame_convention`` to match the driver's ``output.use_enu`` (default false =
"ned"). Run with ``use_sim_time:=false`` on real hardware.
"""

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
    from sbg_driver.msg import SbgEkfEuler, SbgEkfNav
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

        # Latest heading solution.
        self._yaw = None
        self._yaw_rate_estimate = 0.0
        self._yaw_sigma = self.default_heading_sigma
        self._yaw_stamp = None
        self._heading_valid = False

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

        self.get_logger().info(
            f"SBG odometry bridge: {self.ekf_nav_topic}+{self.ekf_euler_topic} -> "
            f"{self.car_state_topic} (odom) + {self.gnss_odom_topic} (anchor) "
            f"[convention={self.convention}, waiting for mode>="
            f"{self.start_min_solution_mode} fix]"
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

    # --- callbacks -------------------------------------------------------

    def on_euler(self, msg):
        prev_yaw = self._yaw
        prev_stamp = self._yaw_stamp
        self._yaw = self._yaw_to_enu(msg.angle.z)
        acc = msg.accuracy.z
        self._yaw_sigma = acc if acc > 0.0 else self.default_heading_sigma
        self._yaw_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
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

    def on_wheel_speeds(self, msg):
        # Rear-axle mean: the differential split cancels exactly in the mean
        # (same rationale as wheel_odometry.py). RPM on the wire.
        rear_rpm = 0.5 * (msg.speeds.lb_speed + msg.speeds.rb_speed)
        self._wheel_speed = rear_rpm * _RPM_TO_RAD_S * self.wheel_radius
        self._wheel_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _can_dead_reckon(self, t, mode):
        """
        Check that wheel+AHRS dead reckoning has live inputs.

        Mode 2 is the floor: below it (VERTICAL_GYRO) the heading itself
        drifts freely and integrating wheel speed under it would confidently
        walk the pose in a wrong direction.
        """
        return (
            self.ahrs_fallback_enable
            and mode >= 2
            and self._heading_valid
            and self._yaw_stamp is not None
            and abs(t - self._yaw_stamp) <= self.wheel_speed_timeout
            and self._wheel_stamp is not None
            and abs(t - self._wheel_stamp) <= self.wheel_speed_timeout
        )

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

    def on_nav(self, msg):
        mode = msg.status.solution_mode
        stamp = msg.header
        t = stamp.stamp.sec + stamp.stamp.nanosec * 1e-9

        if self._yaw is None:
            self._warn_throttle("no SBG heading (ekf_euler) received yet")
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
            have_absolute = (
                msg.status.position_valid and mode >= self.start_min_solution_mode
            )
            if have_absolute:
                if self._origin_lat is None:
                    self._set_origin(msg.latitude, msg.longitude)
                east0, north0 = self._project_enu(msg.latitude, msg.longitude)
                self._odom_xy = [east0, north0]
                self._odom_yaw = self._yaw
                self._last_int_t = t
                self._started = True
                self._georeferenced = True
                self.get_logger().info(
                    f"Started (RTK): origin datum "
                    f"({self._origin_lat*180/math.pi:.7f}, "
                    f"{self._origin_lon*180/math.pi:.7f}), ENU=(0,0)"
                )
            elif self.allow_dr_start and self._can_dead_reckon(t, mode):
                # DR-only start: no absolute fix yet (covered boot, cold GNSS),
                # but heading + wheels are live. SLAM consumes CarState as
                # deltas, so a provisional (0,0) origin is harmless — start
                # feeding motion NOW instead of leaving the whole stack with
                # no state input. The /localization/gnss_odom anchor stays gated until a
                # real fix georeferences the origin (below).
                self._odom_xy = [0.0, 0.0]
                self._odom_yaw = self._yaw
                self._last_int_t = t
                self._started = True
                self._georeferenced = False
                self.get_logger().warn(
                    "Started (DR-only): no absolute fix yet; motion output on "
                    "a provisional origin, GNSS anchor held until a fix arrives"
                )
            else:
                self._warn_throttle(
                    f"waiting to start (mode={mode}, "
                    f"position_valid={msg.status.position_valid}, "
                    f"dr={self._can_dead_reckon(t, mode)})"
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
        # Degrade to wheel+AHRS dead reckoning on the SAME integrator (so the
        # position stays continuous through mode flapping); fault — stop
        # feeding SLAM — only when dead reckoning is impossible too.
        below_min_odom = mode < self.min_odom_solution_mode
        if below_min_odom and not self._can_dead_reckon(t, mode):
            self._warn_throttle(f"solution_mode={mode}: below min_odom, holding output")
            self._publish_health(mode, msg.status, anchored=False, started=True)
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
        dead_reckoning = False
        held = False
        if self._stationary:
            # ZUPT: the wheels report a standstill. Freeze BOTH position and
            # heading -- an Ackermann car cannot translate or yaw with the
            # wheels stopped, so the nav-velocity noise (integrated -> phantom
            # travel) and the unaided heading drift/jumps are pure error. The
            # pose is held identical between frames, so the keyframe delta stays
            # ~0 and graph_slam's skip-tiny gate drops the phantom keyframes on
            # its own. Heading is deliberately NOT refreshed from self._yaw
            # below, and the body twist is published as zero (the car is
            # genuinely stopped, unlike the dead-reckoning branch).
            self._last_vel_enu = None
        elif velocity_ok and 0.0 < dt <= self.max_dt:
            ve, vn = self._vel_enu(msg.velocity)
            prev = self._last_vel_enu or (ve, vn)
            self._odom_xy[0] += 0.5 * (prev[0] + ve) * dt
            self._odom_xy[1] += 0.5 * (prev[1] + vn) * dt
            self._last_vel_enu = (ve, vn)
        elif self._can_dead_reckon(t, mode) and 0.0 < dt <= self.max_dt:
            # Wheel+AHRS dead reckoning: covers mode 2 and momentary
            # velocity dropouts at mode >= 3. Euler at the EKF rate is
            # sub-mm accurate per step; drop the stale ENU velocity so
            # re-entry does not trapezoid across the gap.
            self._odom_xy[0] += self._wheel_speed * math.cos(self._yaw) * dt
            self._odom_xy[1] += self._wheel_speed * math.sin(self._yaw) * dt
            self._last_vel_enu = None
            dead_reckoning = True
        else:
            # Hold position (gap / no fallback) - only heading stays fresh.
            # The frozen pose must not reach the graph with the tier sigma:
            # the car may still be moving, and confident zero-motion edges
            # are exactly what warps the map.
            self._last_vel_enu = None
            held = True
        if not self._stationary:
            self._odom_yaw = self._yaw

        # Heading staleness: the DR path hard-gates on euler age, but the
        # velocity path would republish an arbitrarily old yaw otherwise.
        yaw_sigma = self._yaw_sigma
        if t - self._yaw_stamp > _HEADING_STALE_S:
            yaw_sigma = _HUGE_SIGMA

        if held:
            odom_sigma = _HUGE_SIGMA
        elif dead_reckoning:
            odom_sigma = max(self.odom_sigma_mode2, self._odom_sigma_for_mode(mode))
        else:
            odom_sigma = self._odom_sigma_for_mode(mode)
        # Correction-grade honesty: solution mode alone is too coarse — at
        # mode 4 with SINGLE corrections the realized position error is a
        # metres-scale GM excursion while the mode tier still claims 0.05 m.
        # SLAM then trusts stiff wrong motion, and the map pays (2026-07-18
        # F1 autocross: 56 ghosts from exactly this). The reported velocity
        # accuracy tracks the correction grade, so scale the per-edge trust
        # with its 1 s displacement error.
        vel_acc = max(msg.velocity_accuracy.x, msg.velocity_accuracy.y)
        if vel_acc > 0.0:
            odom_sigma = max(odom_sigma, vel_acc * self.vel_accuracy_horizon_sec)
        self._last_pub_nav_t = t
        # Dead reckoning has no ENU velocity, but the twist must NOT read as
        # v=0: ego_odom passes this twist through to the controllers, and a
        # zero there makes the speed loop wind full throttle for the whole
        # outage (2026-07-19 injection run: cmd saturated +2.5, 8 -> 14.5 m/s
        # in 10 s). The wheel speed that drives the integrator IS the body
        # forward velocity — publish it.
        self._publish_car_state(
            stamp, self._odom_xy, self._odom_yaw, odom_sigma, yaw_sigma,
            body_vx=0.0 if self._stationary
            else (self._wheel_speed if dead_reckoning else None),
            stationary=self._stationary,
        )

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
        )
        self._publish_markers(
            stamp, east, north, max(pos_sigma_e, pos_sigma_n), mode,
            anchored=anchored, fault=False,
        )
        self._publish_status_board(
            t, mode, max(pos_sigma_e, pos_sigma_n),
            anchored=anchored, fault=False, started=True,
            dead_reckoning=dead_reckoning, holdoff=holdoff,
            stationary=self._stationary,
        )

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
                           body_vx=None, stationary=False):
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
        # tier, where the ENU velocity is exactly what is unavailable.
        if body_vx is not None:
            state.twist.twist.linear.x = body_vx
            state.twist.twist.linear.y = 0.0
        else:
            cos_y = math.cos(-yaw)
            sin_y = math.sin(-yaw)
            ve, vn = self._last_vel_enu or (0.0, 0.0)
            state.twist.twist.linear.x = cos_y * ve - sin_y * vn
            state.twist.twist.linear.y = sin_y * ve + cos_y * vn
        # At a ZUPT standstill the car is genuinely stopped: the differentiated
        # yaw rate is just the unaided heading drift/jumps, so report zero
        # rotation too (not only zero linear velocity).
        state.twist.twist.angular.z = 0.0 if stationary else self._yaw_rate_estimate
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
                     stationary=False):
        """(rgb, state word) shared by the map markers and the HUD board."""
        if not started:
            return (0.55, 0.55, 0.55), "WAITING FOR FIX"
        if fault:
            return (0.9, 0.1, 0.1), "FAULT"
        if stationary:
            return (0.2, 0.6, 0.95), "STATIONARY (ZUPT)"
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
        holdoff=False, stationary=False,
    ):
        """Render a fixed HUD below the ATE overlay via rviz_2d_overlay."""
        if self.overlay_pub is None:
            return
        if self._last_board_t is not None and (t - self._last_board_t) < 0.2:
            return
        self._last_board_t = t

        rgb, state = self._state_style(
            mode, sigma, anchored, fault, started, dead_reckoning, stationary)
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
                        dead_reckoning=False, stationary=False):
        names = {0: "UNINITIALIZED", 1: "VERTICAL_GYRO", 2: "AHRS",
                 3: "NAV_VELOCITY", 4: "NAV_POSITION"}
        if not started:
            level, text = DiagnosticStatus.WARN, "waiting for absolute fix"
        elif stationary:
            level, text = DiagnosticStatus.OK, "stationary: ZUPT hold (pose frozen)"
        elif anchored:
            level, text = DiagnosticStatus.OK, "GNSS-anchored (mode 4)"
        elif dead_reckoning:
            level, text = (
                DiagnosticStatus.WARN,
                "degraded: wheel+AHRS dead reckoning, no GNSS",
            )
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
