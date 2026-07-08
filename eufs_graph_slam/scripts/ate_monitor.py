#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""Live SLAM-vs-ground-truth error monitor for RViz.

Compares the graph SLAM pose (`/graph_slam/odom`) against ground truth
(`/ground_truth/state`), interpolating ground truth to each SLAM stamp, and
publishes:

- `/graph_slam/ate_markers` (MarkerArray): a text label that follows the car
  showing the instantaneous position error and the running ATE (RMSE), plus a
  line connecting the SLAM and ground-truth positions.
- `/graph_slam/pose_error` and `/graph_slam/ate` (Float32): the same numbers as
  plain topics for rqt_plot.

Add a MarkerArray display on `/graph_slam/ate_markers` in RViz (fixed frame
`map`) to see it live.
"""

import argparse
import bisect
import math
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy

from eufs_msgs.msg import CarState
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Float32
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

# Optional: a fixed screen-corner HUD needs the rviz_2d_overlay plugin
# (apt install ros-humble-rviz-2d-overlay-plugins). Without it we fall back to
# a text marker that follows the car.
try:
    from rviz_2d_overlay_msgs.msg import OverlayText
    HAVE_OVERLAY = True
except ImportError:
    HAVE_OVERLAY = False


def stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class ATEMonitor(Node):
    def __init__(self, args):
        super().__init__("ate_monitor")
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)]
        )
        self.map_frame = args.map_frame
        self.text_z = args.text_height

        self.gt = deque(maxlen=4000)   # (t, x, y)
        self.gt_t = deque(maxlen=4000)
        self.sum_sq = 0.0
        self.count = 0
        self.max_err = 0.0
        self.latest = None             # (sx, sy, gx, gy, err, rmse)

        best_effort = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, args.slam_odom, self.on_slam, 50)
        self.create_subscription(CarState, args.gt_topic, self.on_gt, best_effort)

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/graph_slam/ate_markers", latched)
        self.err_pub = self.create_publisher(Float32, "/graph_slam/pose_error", 10)
        self.ate_pub = self.create_publisher(Float32, "/graph_slam/ate", 10)

        if HAVE_OVERLAY:
            self.overlay_pub = self.create_publisher(
                OverlayText, "/graph_slam/ate_overlay", latched)
        else:
            self.overlay_pub = None
            self.get_logger().warn(
                "rviz_2d_overlay_plugins not found; ATE shows as a marker above "
                "the car. Install ros-humble-rviz-2d-overlay-plugins for a fixed "
                "top-left HUD on /graph_slam/ate_overlay.")

        self.create_timer(0.1, self.publish_markers)  # 10 Hz refresh

    def on_gt(self, msg):
        t = stamp_sec(msg.header)
        if self.gt_t and t <= self.gt_t[-1]:
            return
        self.gt_t.append(t)
        self.gt.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def gt_at(self, t):
        """Interpolate ground truth to time t; None if out of range."""
        if len(self.gt_t) < 2 or t < self.gt_t[0] or t > self.gt_t[-1]:
            return None
        i = bisect.bisect_left(self.gt_t, t)
        if i == 0:
            return self.gt[0]
        t0, t1 = self.gt_t[i - 1], self.gt_t[i]
        (x0, y0), (x1, y1) = self.gt[i - 1], self.gt[i]
        if t1 <= t0:
            return (x1, y1)
        a = (t - t0) / (t1 - t0)
        return (x0 + a * (x1 - x0), y0 + a * (y1 - y0))

    def on_slam(self, msg):
        t = stamp_sec(msg.header)
        gt = self.gt_at(t)
        if gt is None:
            return
        sx, sy = msg.pose.pose.position.x, msg.pose.pose.position.y
        err = math.hypot(sx - gt[0], sy - gt[1])
        self.sum_sq += err * err
        self.count += 1
        self.max_err = max(self.max_err, err)
        rmse = math.sqrt(self.sum_sq / self.count)
        self.latest = (sx, sy, gt[0], gt[1], err, rmse)
        self.err_pub.publish(Float32(data=float(err)))
        self.ate_pub.publish(Float32(data=float(rmse)))

    def publish_markers(self):
        if self.latest is None:
            return
        sx, sy, gx, gy, err, rmse = self.latest
        now = self.get_clock().now().to_msg()
        # Green when accurate, red when the error grows.
        c = min(1.0, err / 1.0)
        label = (
            f"SLAM vs GT\nerr  {err:.2f} m\nATE  {rmse:.2f} m\nmax  {self.max_err:.2f} m"
        )

        if self.overlay_pub is not None:
            self.publish_overlay(label, c)

        arr = MarkerArray()

        # Text marker above the car only as a fallback when there is no overlay.
        if self.overlay_pub is None:
            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = now
            text.ns = "ate"
            text.id = 0
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=sx, y=sy, z=self.text_z)
            text.pose.orientation.w = 1.0
            text.scale = Vector3(x=0.0, y=0.0, z=0.6)
            text.color = ColorRGBA(r=float(c), g=float(1.0 - c), b=0.2, a=1.0)
            text.text = label
            arr.markers.append(text)

        line = Marker()
        line.header.frame_id = self.map_frame
        line.header.stamp = now
        line.ns = "ate"
        line.id = 1
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.1
        line.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.9)
        line.points = [Point(x=sx, y=sy, z=0.0), Point(x=gx, y=gy, z=0.0)]
        arr.markers.append(line)

        self.marker_pub.publish(arr)

    def publish_overlay(self, text, err_frac):
        """Fixed top-left HUD via rviz_2d_overlay_plugins."""
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
                r=float(err_frac), g=float(1.0 - err_frac), b=0.3, a=1.0)
            ov.line_width = 2
            ov.text_size = 13.0
            ov.font = "DejaVu Sans Mono"
            ov.text = text
            self.overlay_pub.publish(ov)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"OverlayText publish failed ({exc}); falling back to car marker")
            self.overlay_pub = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--slam-odom", default="/graph_slam/odom")
    parser.add_argument("--gt-topic", default="/ground_truth/state")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--text-height", type=float, default=2.0)
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    args = parser.parse_args(argv[1:])

    rclpy.init()
    node = ATEMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
