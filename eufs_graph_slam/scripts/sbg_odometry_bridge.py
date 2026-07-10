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

  * ``/odometry_integration/car_state`` (eufs_msgs/CarState) -- **relative
    odometry**. Pose is the ENU velocity+heading *integrated* here, so it is
    jump-free even when RTK re-acquisition makes the absolute position step.
    graph_slam_node differences this between keyframes. Published only while
    the solution is at least ``min_odom_solution_mode`` (default 3: velocity
    valid). In mode 2 (AHRS) publication stops — the held position would look
    like confident zero motion to the graph while the car may be moving.
  * ``/gnss/odom`` (nav_msgs/Odometry) -- **global anchor**. Pose is the raw
    ekf_nav absolute ENU position with a mode-tiered covariance (tight only at
    mode 4 / RTK, huge otherwise). graph_slam_node adds this as a unary GPS
    prior (EdgeSE2XYPrior) gated on the covariance, so the anchor fades out
    smoothly as GNSS degrades instead of being switched off.
  * ``/sbg_bridge/status`` (diagnostic_msgs/DiagnosticArray) -- health so the
    autonomy stack knows the current degradation tier.

Both channels live in the same local ENU frame (datum = first valid fix, or a
fixed ``datum_*`` for reproducibility), and the node waits for one valid
absolute fix before starting so the SLAM origin is georeferenced. Heading
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
from eufs_msgs.msg import CarState
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


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SbgOdometryBridge(Node):
    def __init__(self):
        super().__init__("sbg_odometry_bridge")

        # Topics.
        self.car_state_topic = self.declare_parameter(
            "car_state_topic", "/odometry_integration/car_state"
        ).value
        self.gnss_odom_topic = self.declare_parameter(
            "gnss_odom_topic", "/gnss/odom"
        ).value
        self.health_topic = self.declare_parameter(
            "health_topic", "/sbg_bridge/status"
        ).value
        # Default off: the 2-sigma fix disc reads like a floating pie chart in
        # RViz. Re-enable with publish_markers:=true when debugging GNSS.
        self.publish_markers = self.declare_parameter("publish_markers", False).value
        self.marker_topic = self.declare_parameter(
            "marker_topic", "/gnss/markers"
        ).value
        self.publish_overlay = self.declare_parameter("publish_overlay", True).value
        self.overlay_topic = self.declare_parameter(
            "overlay_topic", "/gnss/overlay"
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
        # The absolute /gnss/odom prior is only trustworthy at/above this mode;
        # below it we still publish but with a huge covariance so the SLAM
        # prior gate drops it.
        self.absolute_min_solution_mode = self.declare_parameter(
            "absolute_min_solution_mode", 4
        ).value
        # Reject integration steps with a non-positive or too-large dt [s].
        self.max_dt = self.declare_parameter("max_dt", 0.5).value

        # 1-sigma fallbacks / tier inflation [m], [rad].
        self.default_position_sigma = self.declare_parameter(
            "default_position_sigma", 0.05
        ).value
        self.default_heading_sigma = self.declare_parameter(
            "default_heading_sigma", 0.02
        ).value
        # Odometry (relative) 1-sigma per tier used for the CarState covariance.
        self.odom_sigma_mode4 = self.declare_parameter("odom_sigma_mode4", 0.05).value
        self.odom_sigma_mode3 = self.declare_parameter("odom_sigma_mode3", 0.20).value
        self.odom_sigma_mode2 = self.declare_parameter("odom_sigma_mode2", 10.0).value

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

        # Latest heading solution.
        self._yaw = None
        self._yaw_sigma = self.default_heading_sigma
        self._yaw_stamp = None
        self._heading_valid = False

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
        self._yaw = self._yaw_to_enu(msg.angle.z)
        acc = msg.accuracy.z
        self._yaw_sigma = acc if acc > 0.0 else self.default_heading_sigma
        self._yaw_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._heading_valid = bool(msg.status.heading_valid)

    def on_nav(self, msg):
        mode = msg.status.solution_mode
        stamp = msg.header
        t = stamp.stamp.sec + stamp.stamp.nanosec * 1e-9

        if self._yaw is None:
            self._warn_throttle("no SBG heading (ekf_euler) received yet")
            return

        # Startup gate: georeference the origin from the first good absolute fix.
        if not self._started:
            if not (msg.status.position_valid and mode >= self.start_min_solution_mode):
                self._warn_throttle(
                    f"waiting for absolute fix to start "
                    f"(mode={mode}, position_valid={msg.status.position_valid})"
                )
                self._publish_health(mode, msg.status, anchored=False, started=False)
                self._publish_status_board(
                    t, mode, None, anchored=False, fault=False, started=False
                )
                return
            if self._origin_lat is None:
                self._set_origin(msg.latitude, msg.longitude)
            east0, north0 = self._project_enu(msg.latitude, msg.longitude)
            self._odom_xy = [east0, north0]
            self._odom_yaw = self._yaw
            self._last_int_t = t
            self._started = True
            self.get_logger().info(
                f"Started: origin datum ({self._origin_lat*180/math.pi:.7f}, "
                f"{self._origin_lon*180/math.pi:.7f}), ENU=(0,0)"
            )

        # Fault below the minimum odometry mode: stop feeding SLAM.
        if mode < self.min_odom_solution_mode:
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
            return

        # --- relative odometry: integrate (jump-free) --------------------
        east, north = self._project_enu(msg.latitude, msg.longitude)
        dt = t - self._last_int_t if self._last_int_t is not None else 0.0
        self._last_int_t = t
        velocity_ok = bool(msg.status.velocity_valid) and mode >= 3
        if velocity_ok and 0.0 < dt <= self.max_dt:
            ve, vn = self._vel_enu(msg.velocity)
            prev = self._last_vel_enu or (ve, vn)
            self._odom_xy[0] += 0.5 * (prev[0] + ve) * dt
            self._odom_xy[1] += 0.5 * (prev[1] + vn) * dt
            self._last_vel_enu = (ve, vn)
        else:
            # Hold position (mode 2 / gap) - only heading stays fresh; drop
            # the stale velocity so re-entry does not trapezoid across a gap.
            self._last_vel_enu = None
        self._odom_yaw = self._yaw

        odom_sigma = self._odom_sigma_for_mode(mode)
        self._publish_car_state(stamp, self._odom_xy, self._odom_yaw, odom_sigma)

        # --- global anchor: raw absolute ENU + mode-tiered covariance ----
        anchored = (
            msg.status.position_valid and mode >= self.absolute_min_solution_mode
        )
        if anchored:
            pos_sigma_e = msg.position_accuracy.x or self.default_position_sigma
            pos_sigma_n = msg.position_accuracy.y or self.default_position_sigma
            if self.convention == "ned":
                pos_sigma_e, pos_sigma_n = pos_sigma_n, pos_sigma_e
        else:
            pos_sigma_e = pos_sigma_n = _HUGE_SIGMA
        self._publish_gnss_odom(stamp, east, north, self._odom_yaw,
                                pos_sigma_e, pos_sigma_n)

        self._publish_health(mode, msg.status, anchored=anchored, started=True)
        self._publish_markers(
            stamp, east, north, max(pos_sigma_e, pos_sigma_n), mode,
            anchored=anchored, fault=False,
        )
        self._publish_status_board(
            t, mode, max(pos_sigma_e, pos_sigma_n),
            anchored=anchored, fault=False, started=True,
        )

    # --- publishing ------------------------------------------------------

    def _odom_sigma_for_mode(self, mode):
        if mode >= 4:
            return self.odom_sigma_mode4
        if mode == 3:
            return self.odom_sigma_mode3
        return self.odom_sigma_mode2

    def _publish_car_state(self, header, xy, yaw, trans_sigma):
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
        state.pose.covariance[35] = self._yaw_sigma * self._yaw_sigma
        self.car_state_pub.publish(state)

    def _publish_gnss_odom(self, header, east, north, yaw, sigma_e, sigma_n):
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
        odom.pose.covariance[35] = self._yaw_sigma * self._yaw_sigma
        self.gnss_odom_pub.publish(odom)

    @staticmethod
    def _state_style(mode, sigma, anchored, fault, started):
        """(rgb, state word) shared by the map markers and the HUD board."""
        if not started:
            return (0.55, 0.55, 0.55), "WAITING FOR FIX"
        if fault:
            return (0.9, 0.1, 0.1), "FAULT"
        if not anchored:
            return (0.55, 0.55, 0.55), "NO ANCHOR (INS)"
        if sigma <= 0.05:
            return (0.1, 0.8, 0.2), "RTK FIXED"
        if sigma <= 0.5:
            return (0.9, 0.8, 0.1), "RTK FLOAT"
        return (0.95, 0.5, 0.1), "SINGLE POINT"

    def _publish_status_board(self, t, mode, sigma, anchored, fault, started):
        """Render a fixed HUD below the ATE overlay via rviz_2d_overlay."""
        if self.overlay_pub is None:
            return
        if self._last_board_t is not None and (t - self._last_board_t) < 0.2:
            return
        self._last_board_t = t

        rgb, state = self._state_style(mode, sigma, anchored, fault, started)
        mode_names = {0: "UNINITIALIZED", 1: "VERT_GYRO", 2: "AHRS",
                      3: "NAV_VELOCITY", 4: "NAV_POSITION"}
        if anchored and sigma is not None:
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
        # HUD stack (left edge, 8px gaps): CTE box (12,12,h110) -> SLAM status
        # box (12,130,h34) -> this GNSS box (12,172). Keep the three publishers
        # in sync when moving any of them.
        ov.width = 220
        ov.height = 96
        ov.horizontal_distance = 12
        ov.vertical_distance = 172
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

    def _publish_health(self, mode, status, anchored, started):
        names = {0: "UNINITIALIZED", 1: "VERTICAL_GYRO", 2: "AHRS",
                 3: "NAV_VELOCITY", 4: "NAV_POSITION"}
        if not started:
            level, text = DiagnosticStatus.WARN, "waiting for absolute fix"
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
