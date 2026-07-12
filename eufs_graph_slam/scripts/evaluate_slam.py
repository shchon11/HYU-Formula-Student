#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Graph SLAM evaluation harness.

Records SLAM odometry, raw (input) odometry, and ground truth for a fixed
sim-time window, then reports:
- trajectory ATE (RMSE / max) for SLAM output and for raw odometry input
- landmark map quality against the track CSV: matches, position RMSE,
  false positives, duplicates, color accuracy

Writes a JSON report and prints a summary.
"""

import argparse
import csv
import json
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import CarState, ConeArrayWithCovariance
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def load_gt_cones(csv_path):
    """GT cones in the car-start frame (the frame SLAM and GT state use)."""
    cones = []
    start = (0.0, 0.0, 0.0)
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            tag = row["tag"].strip()
            if tag == "car_start":
                start = (float(row["x"]), float(row["y"]), float(row["direction"]))
            elif tag in ("blue", "yellow", "orange", "big_orange"):
                cones.append((tag, float(row["x"]), float(row["y"])))

    x0, y0, yaw0 = start
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    return [
        (tag, c * (x - x0) - s * (y - y0), s * (x - x0) + c * (y - y0))
        for tag, x, y in cones
    ]


def interpolate(traj, t):
    """Linear interpolation of (t, x, y) samples; None outside the range."""
    if not traj or t < traj[0][0] or t > traj[-1][0]:
        return None
    lo, hi = 0, len(traj) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if traj[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, x0, y0 = traj[lo]
    t1, x1, y1 = traj[hi]
    if t1 <= t0:
        return (x0, y0)
    a = (t - t0) / (t1 - t0)
    return (x0 + a * (x1 - x0), y0 + a * (y1 - y0))


def ate(est, gt):
    errs = []
    for t, x, y in est:
        p = interpolate(gt, t)
        if p is None:
            continue
        errs.append(math.hypot(x - p[0], y - p[1]))
    if not errs:
        return {"rmse": None, "max": None, "mean": None, "n": 0}
    return {
        "rmse": math.sqrt(sum(e * e for e in errs) / len(errs)),
        "max": max(errs),
        "mean": sum(errs) / len(errs),
        "n": len(errs),
    }


def map_metrics(est_cones, gt_cones, match_dist):
    """est_cones: list of (color, x, y). gt_cones likewise."""
    result = {
        "estimated": len(est_cones),
        "gt_total": len(gt_cones),
        "matched": 0,
        "duplicates": 0,
        "false_positives": 0,
        "color_correct": 0,
        "position_rmse": None,
    }
    if not est_cones:
        return result

    color_counts = {}
    for color, _, _ in est_cones:
        color_counts[color] = color_counts.get(color, 0) + 1
    result["est_by_color"] = color_counts

    gt_hit = [0] * len(gt_cones)
    sq_errs = []
    for color, x, y in est_cones:
        best_i, best_d = -1, match_dist
        for i, (gc, gx, gy) in enumerate(gt_cones):
            d = math.hypot(gx - x, gy - y)
            if d < best_d:
                best_d, best_i = d, i
        if best_i < 0:
            result["false_positives"] += 1
            continue
        if gt_hit[best_i]:
            result["duplicates"] += 1
        else:
            result["matched"] += 1
            sq_errs.append(best_d * best_d)
        gt_hit[best_i] += 1
        gt_color = gt_cones[best_i][0]
        if color == gt_color or (color == "orange" and gt_color == "big_orange"):
            result["color_correct"] += 1

    if sq_errs:
        result["position_rmse"] = math.sqrt(sum(sq_errs) / len(sq_errs))
    return result


class Evaluator(Node):
    def __init__(self, args):
        super().__init__("slam_evaluator")
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)]
        )
        self.args = args
        self.gt = []
        self.slam = []
        self.raw = []
        self.map_msg = None
        self.done = False

        best_effort = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(CarState, args.gt_topic, self.on_gt, best_effort)
        self.create_subscription(Odometry, args.slam_odom, self.on_slam, 50)
        self.create_subscription(CarState, args.raw_odom, self.on_raw, best_effort)
        self.create_subscription(
            ConeArrayWithCovariance, args.map_topic, self.on_map, latched
        )
        # Instrumentation: is the driver actually commanding the car?
        self.cmd_count = 0
        self.last_cmd = None
        self.create_subscription(AckermannDriveStamped, "/cmd", self.on_cmd, 10)

    def on_cmd(self, msg):
        self.cmd_count += 1
        self.last_cmd = (msg.drive.speed, msg.drive.acceleration, msg.drive.steering_angle)

    def on_gt(self, msg):
        t = stamp_sec(msg.header)
        self.gt.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))
        if (
            len(self.gt) > 2
            and self.gt[-1][0] - self.gt[0][0] >= self.args.duration
        ):
            self.done = True

    def on_slam(self, msg):
        self.slam.append(
            (stamp_sec(msg.header), msg.pose.pose.position.x, msg.pose.pose.position.y)
        )

    def on_raw(self, msg):
        self.raw.append(
            (stamp_sec(msg.header), msg.pose.pose.position.x, msg.pose.pose.position.y)
        )

    def on_map(self, msg):
        self.map_msg = msg

    def report(self):
        est_cones = []
        if self.map_msg is not None:
            for attr, color in (
                ("blue_cones", "blue"),
                ("yellow_cones", "yellow"),
                ("orange_cones", "orange"),
                ("big_orange_cones", "big_orange"),
                ("unknown_color_cones", "unknown"),
            ):
                for cone in getattr(self.map_msg, attr):
                    est_cones.append((color, cone.point.x, cone.point.y))

        gt_cones = load_gt_cones(self.args.csv)

        # Did the car actually drive? Path length + bounding box of the GT and
        # raw (drifted) trajectories tell mapping/driving faults apart.
        def traj_stats(traj):
            if len(traj) < 2:
                return {"path_len": 0.0, "bbox": [0.0, 0.0], "displacement": 0.0}
            length = sum(
                math.hypot(traj[i][1] - traj[i - 1][1], traj[i][2] - traj[i - 1][2])
                for i in range(1, len(traj))
            )
            xs = [p[1] for p in traj]
            ys = [p[2] for p in traj]
            disp = math.hypot(traj[-1][1] - traj[0][1], traj[-1][2] - traj[0][2])
            return {
                "path_len": length,
                "bbox": [max(xs) - min(xs), max(ys) - min(ys)],
                "displacement": disp,
            }

        report = {
            "duration": self.args.duration,
            "samples": {
                "gt": len(self.gt),
                "slam": len(self.slam),
                "raw": len(self.raw),
            },
            "gt_traj": traj_stats(self.gt),
            "raw_traj": traj_stats(self.raw),
            "gt_start": [self.gt[0][1], self.gt[0][2]] if self.gt else None,
            "cmd_count": self.cmd_count,
            "last_cmd": self.last_cmd,
            "ate_slam": ate(self.slam, self.gt),
            "ate_raw_odom": ate(self.raw, self.gt),
            "map": map_metrics(est_cones, gt_cones, self.args.match_dist),
        }
        return report


def build_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, help="track csv with GT cones")
    parser.add_argument("--duration", type=float, default=120.0, help="sim-time window")
    parser.add_argument("--output", required=True, help="output json path")
    parser.add_argument("--gt-topic", default="/ground_truth/state")
    parser.add_argument("--slam-odom", default="/localization/ego_odom")
    parser.add_argument("--raw-odom", default="/drift_odom/car_state")
    parser.add_argument("--map-topic", default="/localization/cone_map")
    parser.add_argument("--match-dist", type=float, default=1.0)
    return parser


def main():
    args = build_arg_parser().parse_args()

    rclpy.init()
    node = Evaluator(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass

    report = node.report()
    with open(args.output, "w") as f:
        json.dump(report, f, indent=2)

    def fmt(v):
        return "n/a" if v is None else f"{v:.3f}"

    a_s, a_r, m = report["ate_slam"], report["ate_raw_odom"], report["map"]
    print("==== graph SLAM evaluation ====")
    print(f"samples gt/slam/raw: {report['samples']['gt']}/"
          f"{report['samples']['slam']}/{report['samples']['raw']}")
    print(f"ATE slam: rmse={fmt(a_s['rmse'])} max={fmt(a_s['max'])} n={a_s['n']}")
    print(f"ATE raw : rmse={fmt(a_r['rmse'])} max={fmt(a_r['max'])} n={a_r['n']}")
    print(f"map: est={m['estimated']} matched={m['matched']}/{m['gt_total']} "
          f"rmse={fmt(m['position_rmse'])} dup={m['duplicates']} "
          f"fp={m['false_positives']} color_ok={m['color_correct']}")
    print(f"report written to {args.output}")

    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
