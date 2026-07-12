#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Drifting-odometry generator for EUFS graph SLAM experiments.

The simulator's noise model perturbs the absolute pose with iid Gaussian
noise, which does not reproduce real odometry drift. This node integrates
the ground-truth body-frame twist with a velocity/yaw-rate bias plus noise,
producing a CarState whose pose drifts like real wheel odometry. Point
graph SLAM at the output topic to test drift correction.
"""

import argparse
import math
import random
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from eufs_msgs.msg import CarState


class DriftOdom(Node):
    def __init__(self, args):
        super().__init__("drift_odom")
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)]
        )
        self.rng = random.Random(args.seed)
        self.v_bias = args.v_bias
        self.w_bias = args.w_bias
        self.sigma_v = args.sigma_v
        self.sigma_w = args.sigma_w

        self.pose = None  # (x, y, yaw)
        self.last_t = None

        self.pub = self.create_publisher(CarState, args.output, 10)
        self.sub = self.create_subscription(
            CarState,
            args.input,
            self.on_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.get_logger().info(
            f"drift odometry {args.input} -> {args.output} "
            f"(v_bias={self.v_bias} w_bias={self.w_bias} "
            f"sigma_v={self.sigma_v} sigma_w={self.sigma_w})"
        )

    def on_state(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.pose is None:
            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            self.last_t = t
            return

        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 0.5:
            return

        # Corrupt body-frame twist with bias + white noise, then integrate.
        v_x = msg.twist.twist.linear.x * (1.0 + self.v_bias) + self.rng.gauss(
            0.0, self.sigma_v
        )
        v_y = msg.twist.twist.linear.y
        w = msg.twist.twist.angular.z * (1.0 + self.w_bias) + self.rng.gauss(
            0.0, self.sigma_w
        )

        x, y, yaw = self.pose
        x += (v_x * math.cos(yaw) - v_y * math.sin(yaw)) * dt
        y += (v_x * math.sin(yaw) + v_y * math.cos(yaw)) * dt
        yaw = math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt))
        self.pose = (x, y, yaw)

        out = CarState()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = math.sin(0.5 * yaw)
        out.pose.pose.orientation.w = math.cos(0.5 * yaw)
        # Per-sample twist noise levels; downstream consumers can derive
        # relative-motion covariance from these diagonals and dt.
        out.pose.covariance[0] = self.sigma_v * self.sigma_v
        out.pose.covariance[7] = self.sigma_v * self.sigma_v
        out.pose.covariance[35] = self.sigma_w * self.sigma_w
        out.twist = msg.twist
        self.pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="/ground_truth/state")
    parser.add_argument("--output", default="/drift_odom/car_state")
    parser.add_argument("--v-bias", type=float, default=0.02)
    parser.add_argument("--w-bias", type=float, default=0.01)
    parser.add_argument("--sigma-v", type=float, default=0.05)
    parser.add_argument("--sigma-w", type=float, default=0.01)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    rclpy.init()
    node = DriftOdom(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
