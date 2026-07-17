#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Path-tracking cross-track-error (CTE) monitor for RViz.

The planned raceline (/global_waypoints) is the d=0 reference. The ego pose's
Frenet lateral offset d — published by frenet_odom_node as
pose.pose.position.y on /planning/frenet_odom — is treated as the tracking
error and evaluated ATE-style (running RMSE of d, max |d|). GT-based ATE was
removed: it needs ground truth, which does not exist on the real car.

Publishes:
- /planning/cte_overlay (OverlayText): fixed top-left HUD with the current
  signed d, running RMSE and max |d|. Greys out to a "waiting" card while the
  global path is invalid (frenet_odom_node stops publishing then).
- /planning/cte and /planning/cte_rmse (Float32): the same numbers for
  rqt_plot.
- /graph_slam/status_overlay (OverlayText): SLAM lifecycle box (unchanged).

Stats reset each time the global path becomes valid again — a new raceline is
a new evaluation window.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ColorRGBA, Float32, String

# The fixed screen-corner HUD needs the rviz_2d_overlay plugin
# (apt install ros-humble-rviz-2d-overlay-plugins).
try:
    from rviz_2d_overlay_msgs.msg import OverlayText
    HAVE_OVERLAY = True
except ImportError:
    HAVE_OVERLAY = False


class CTEMonitor(Node):
    # Node/executable name stays "ate_monitor" so existing launch arguments
    # (graph_slam.launch.py ate_monitor:=..., hyu_planning_bringup
    # graph_slam_ate_monitor:=...) keep working unchanged.
    def __init__(self, args):
        super().__init__("ate_monitor")
        self.stale_timeout = args.stale_timeout

        # Running stats over the signed Frenet d (reset on path re-validation).
        self.sum_sq = 0.0
        self.count = 0
        self.max_abs = 0.0
        self.latest = None            # (d, s, rmse)
        self.last_rx_monotonic = None
        self.path_valid = False

        self.create_subscription(Odometry, args.frenet_odom, self.on_frenet, 50)

        # Planner heartbeat: reliable volatile (matches planner_node's QoS).
        self.create_subscription(
            Bool, args.path_valid_topic, self.on_path_valid, 10)

        # SLAM lifecycle (mapping / mapping_converged / localization); the
        # node latches it, so subscribe transient_local to get the last state.
        self.slam_status = "unknown"
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, args.status_topic, self.on_status, status_qos)

        self.cte_pub = self.create_publisher(Float32, "/planning/cte", 10)
        self.cte_rmse_pub = self.create_publisher(
            Float32, "/planning/cte_rmse", 10)

        # Latched (transient-local) so a late-joining RViz immediately gets
        # the last HUD state — and so the transient-local overlay displays in
        # the shipped RViz config actually match this publisher's QoS.
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        if HAVE_OVERLAY:
            self.overlay_pub = self.create_publisher(
                OverlayText, "/planning/cte_overlay", latched)
            self.status_overlay_pub = self.create_publisher(
                OverlayText, "/graph_slam/status_overlay", latched)
        else:
            self.overlay_pub = None
            self.status_overlay_pub = None
            self.get_logger().warn(
                "rviz_2d_overlay_plugins not found; install "
                "ros-humble-rviz-2d-overlay-plugins for the CTE HUD on "
                "/planning/cte_overlay.")

        self.create_timer(0.1, self.publish_overlays)  # 10 Hz refresh

        self.get_logger().info(
            f"CTE monitor: frenet d from '{args.frenet_odom}' vs d=0 raceline; "
            f"validity from '{args.path_valid_topic}'")

    def on_status(self, msg):
        self.slam_status = msg.data

    def on_path_valid(self, msg):
        if msg.data and not self.path_valid:
            # New / re-validated raceline: start a fresh evaluation window.
            self.sum_sq = 0.0
            self.count = 0
            self.max_abs = 0.0
            self.latest = None
        self.path_valid = msg.data

    def on_frenet(self, msg):
        d = float(msg.pose.pose.position.y)
        s = float(msg.pose.pose.position.x)
        if not math.isfinite(d):
            return
        self.sum_sq += d * d
        self.count += 1
        self.max_abs = max(self.max_abs, abs(d))
        rmse = math.sqrt(self.sum_sq / self.count)
        self.latest = (d, s, rmse)
        self.last_rx_monotonic = time.monotonic()
        self.cte_pub.publish(Float32(data=d))
        self.cte_rmse_pub.publish(Float32(data=rmse))

    def publish_overlays(self):
        self.publish_status_overlay()

        if self.overlay_pub is None:
            return

        stale = (
            self.last_rx_monotonic is None or
            time.monotonic() - self.last_rx_monotonic > self.stale_timeout)

        if stale or self.latest is None:
            reason = "path invalid" if not self.path_valid else "no frenet odom"
            self.publish_overlay(
                f"PATH CTE\nwaiting...\n({reason})", (0.7, 0.7, 0.7))
            return

        d, s, rmse = self.latest
        # Green when on the line, red once |d| reaches 1 m.
        c = min(1.0, abs(d) / 1.0)
        label = (
            f"PATH CTE\nd    {d:+.2f} m\nrmse {rmse:.2f} m\n"
            f"max  {self.max_abs:.2f} m"
        )
        self.publish_overlay(label, (c, 1.0 - c, 0.3))

    def publish_overlay(self, text, rgb):
        """Fixed top-left HUD box (12,12) — the top of the HUD stack."""
        try:
            ov = OverlayText()
            ov.action = OverlayText.ADD
            ov.width = 220
            ov.height = 110
            ov.horizontal_distance = 12
            ov.vertical_distance = 12
            ov.horizontal_alignment = OverlayText.LEFT
            ov.vertical_alignment = OverlayText.TOP
            ov.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.55)
            ov.fg_color = ColorRGBA(
                r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=1.0)
            ov.line_width = 2
            ov.text_size = 13.0
            ov.font = "DejaVu Sans Mono"
            ov.text = text
            self.overlay_pub.publish(ov)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"OverlayText publish failed ({exc})")
            self.overlay_pub = None

    STATUS_STYLE = {
        # label, (r, g, b)
        "mapping": ("MAPPING", (1.0, 0.85, 0.2)),
        "mapping_converged": ("MAPPING (converged)", (0.2, 0.9, 1.0)),
        "localization": ("LOCALIZATION", (0.3, 1.0, 0.3)),
        "unknown": ("SLAM: waiting...", (0.7, 0.7, 0.7)),
    }

    def publish_status_overlay(self):
        """SLAM lifecycle box just below the CTE box (does not overlap it)."""
        if self.status_overlay_pub is None:
            return
        label, (r, g, b) = self.STATUS_STYLE.get(
            self.slam_status, (self.slam_status, (0.7, 0.7, 0.7)))
        try:
            ov = OverlayText()
            ov.action = OverlayText.ADD
            ov.width = 220
            ov.height = 34
            ov.horizontal_distance = 12
            ov.vertical_distance = 130
            ov.horizontal_alignment = OverlayText.LEFT
            ov.vertical_alignment = OverlayText.TOP
            ov.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.55)
            ov.fg_color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
            ov.line_width = 2
            ov.text_size = 13.0
            ov.font = "DejaVu Sans Mono"
            ov.text = label
            self.status_overlay_pub.publish(ov)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"status OverlayText publish failed ({exc})")
            self.status_overlay_pub = None


def build_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frenet-odom", default="/planning/frenet_odom")
    parser.add_argument("--status-topic", default="/localization/status")
    parser.add_argument(
        "--path-valid-topic", default="/planning/global_path_valid")
    parser.add_argument("--stale-timeout", type=float, default=1.0)
    return parser


def main():
    parser = build_arg_parser()
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    args = parser.parse_args(argv[1:])

    rclpy.init()
    node = CTEMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
