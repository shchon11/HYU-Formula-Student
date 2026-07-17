#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Pure-pursuit test driver for EUFS graph SLAM experiments.

Builds a centerline from a track CSV (blue/yellow cone pairs), sets the
TRACK_DRIVE mission, then follows the centerline using ground-truth state.
This is a test harness: it deliberately uses ground truth so SLAM quality
does not affect the driven trajectory.
"""

import argparse
import csv
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import CanState, CarState
from eufs_msgs.srv import SetCanState


def load_centerline(csv_path):
    blues, yellows, start, start_dir = [], [], (0.0, 0.0), 0.0
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            tag = row["tag"].strip()
            x, y = float(row["x"]), float(row["y"])
            if tag == "blue":
                blues.append((x, y))
            elif tag == "yellow":
                yellows.append((x, y))
            elif tag == "car_start":
                start = (x, y)
                start_dir = float(row.get("direction", 0.0) or 0.0)

    # The CSV is track-absolute, but /ground_truth/state reports the car
    # spawn-RELATIVE (0,0,0 at car_start). Following absolute waypoints with
    # a relative pose drives a loop translated/rotated by the whole spawn
    # offset — transform everything into the spawn frame instead.
    cos_d, sin_d = math.cos(-start_dir), math.sin(-start_dir)

    def to_spawn_frame(p):
        dx, dy = p[0] - start[0], p[1] - start[1]
        return (cos_d * dx - sin_d * dy, sin_d * dx + cos_d * dy)

    blues = [to_spawn_frame(p) for p in blues]
    yellows = [to_spawn_frame(p) for p in yellows]
    start = (0.0, 0.0)

    if not blues or not yellows:
        raise RuntimeError("track csv has no blue/yellow cones")

    # Midpoint of each blue cone and its nearest yellow cone.
    midpoints = []
    for bx, by in blues:
        yx, yy = min(yellows, key=lambda c: (c[0] - bx) ** 2 + (c[1] - by) ** 2)
        if math.hypot(yx - bx, yy - by) < 8.0:
            midpoints.append(((bx + yx) / 2.0, (by + yy) / 2.0))

    # Greedy nearest-neighbour chaining from the start position.
    remaining = midpoints[:]
    chain = []
    cur = start
    while remaining:
        nxt = min(remaining, key=lambda p: (p[0] - cur[0]) ** 2 + (p[1] - cur[1]) ** 2)
        remaining.remove(nxt)
        chain.append(nxt)
        cur = nxt

    # Resample the closed loop at fixed spacing.
    pts = []
    spacing = 0.5
    n = len(chain)
    for i in range(n):
        x0, y0 = chain[i]
        x1, y1 = chain[(i + 1) % n]
        seg = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(seg / spacing))
        for s in range(steps):
            t = s / steps
            pts.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))

    # The greedy chain picks an arbitrary loop direction. Race direction is
    # defined by the spawn heading (+x in the spawn frame after the
    # transform above): if the path tangent at the nearest point opposes it,
    # the whole loop is reversed — driving backwards flips the blue/yellow
    # context every consumer assumes.
    nearest = min(range(len(pts)), key=lambda i: pts[i][0] ** 2 + pts[i][1] ** 2)
    after = pts[(nearest + 1) % len(pts)]
    if after[0] - pts[nearest][0] < 0.0:
        pts.reverse()
    return pts


def curvature(pts, i):
    p0 = pts[(i - 2) % len(pts)]
    p1 = pts[i % len(pts)]
    p2 = pts[(i + 2) % len(pts)]
    a = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    b = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    c = math.hypot(p2[0] - p0[0], p2[1] - p0[1])
    area2 = abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]))
    if a * b * c < 1e-6:
        return 0.0
    return 2.0 * area2 / (a * b * c)


class DriveTrack(Node):
    WHEELBASE = 1.58
    MAX_STEER = 0.52

    def __init__(self, args):
        super().__init__("drive_track")
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)]
        )
        self.path = load_centerline(args.csv)
        self.lookahead = args.lookahead
        self.v_max = args.speed
        self.duration = args.duration
        self.pose = None
        self.speed = 0.0
        self.t_start = None
        self.done = False

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, "/cmd", 1)
        self.state_sub = self.create_subscription(
            CarState,
            "/ground_truth/state",
            self.on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.timer = self.create_timer(0.05, self.on_timer)

        self.mission_client = self.create_client(SetCanState, "/ros_can/set_mission")
        self.mission_sent = False
        self.get_logger().info(
            f"loaded centerline with {len(self.path)} points; waiting for sim"
        )

    def on_state(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def send_mission(self):
        if not self.mission_client.service_is_ready():
            return
        req = SetCanState.Request()
        req.ami_state = CanState.AMI_TRACK_DRIVE
        self.mission_client.call_async(req)
        self.mission_sent = True
        self.get_logger().info("mission TRACK_DRIVE requested")

    def on_timer(self):
        if not self.mission_sent:
            self.send_mission()
            return
        if self.pose is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_start is None:
            self.t_start = now
        if now - self.t_start >= self.duration:
            cmd = AckermannDriveStamped()
            cmd.drive.acceleration = -5.0
            self.cmd_pub.publish(cmd)
            if self.speed < 0.2:
                self.done = True
            return

        x, y, yaw = self.pose
        # Nearest path point, then advance by the lookahead distance.
        n = len(self.path)
        idx = min(range(n), key=lambda i: (self.path[i][0] - x) ** 2 + (self.path[i][1] - y) ** 2)
        dist = 0.0
        j = idx
        while dist < self.lookahead:
            j2 = (j + 1) % n
            dist += math.hypot(
                self.path[j2][0] - self.path[j][0], self.path[j2][1] - self.path[j][1]
            )
            j = j2
        tx, ty = self.path[j]

        dx, dy = tx - x, ty - y
        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        ld = math.hypot(dx, dy)
        steer = math.atan2(2.0 * self.WHEELBASE * math.sin(alpha), max(ld, 0.5))
        steer = max(-self.MAX_STEER, min(self.MAX_STEER, steer))

        # Slow down for upcoming curvature.
        kappa = max(curvature(self.path, j), curvature(self.path, (j + 6) % n))
        v_ref = self.v_max
        if kappa > 1e-3:
            v_ref = min(v_ref, math.sqrt(4.0 / kappa))
        v_ref = max(2.0, v_ref)

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.steering_angle = steer
        cmd.drive.speed = v_ref
        cmd.drive.acceleration = max(-8.0, min(2.5, 1.2 * (v_ref - self.speed)))
        self.cmd_pub.publish(cmd)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--speed", type=float, default=4.5)
    parser.add_argument("--lookahead", type=float, default=3.5)
    args = parser.parse_args()

    rclpy.init()
    node = DriveTrack(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.get_logger().info("drive finished")
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
