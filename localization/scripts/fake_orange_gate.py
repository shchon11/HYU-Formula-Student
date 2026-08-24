#!/usr/bin/env python3
"""fake_orange_gate — relabel the cones around given map-frame points as big-orange.

BAG-REPLAY AID ONLY. graph_slam closes the lap (seam registration -> map freeze ->
localization) only with an orange start/finish gate in view; a test track laid out with
blue/yellow cones alone never leaves mapping. This relay sits between perception and the
consumers and re-colours every cone that lands within `radius` of one of `gate_xy`
(pairs of x,y in the INS/map frame, i.e. the frame of `pose_topic`) so that one physical
pair plays the gate. Everything else passes through untouched (positions, covariances,
stamps), and the perception RViz markers (<input>/viz) are forwarded as <output>/viz. Run perception on another topic and put this in front of it:

    bagplay <bag> output_cones_topic:=/perception/cones_raw
    ros2 run hyu_localization fake_orange_gate --ros-args -p use_sim_time:=true \
        -p gate_xy:="[-2.06, 0.47, -0.77, 2.68]" -p radius:=0.7

`gate_xy` is a flat list x1,y1,x2,y2[,x3,y3...]. Relabelled cones get `orange_sigma` (0.26 m,
lidar grade) as covariance so graph_slam accepts them as gate evidence (it demotes orange
above csm_orange_max_obs_sigma=0.45 to unknown; uncoloured LiDAR cones carry 0.48).
The pose used to place a cone frame is
the `pose_topic` sample nearest its header stamp (CarState or Odometry). Frames without
a pose within `pose_max_age` are forwarded unchanged.
"""
import bisect
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from hyu_msgs.msg import CarState, ConeArrayWithCovariance
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FakeOrangeGate(Node):
    def __init__(self):
        super().__init__("fake_orange_gate")
        self.input_topic = self.declare_parameter("input_topic", "/perception/cones_raw").value
        self.output_topic = self.declare_parameter("output_topic", "/perception/cones").value
        self.pose_topic = self.declare_parameter("pose_topic", "/localization/ins_odom").value
        self.pose_type = self.declare_parameter("pose_type", "car_state").value  # car_state | odometry
        gate = list(self.declare_parameter("gate_xy", [0.0, 0.0]).value)
        if len(gate) < 2 or len(gate) % 2:
            raise ValueError("gate_xy must be a flat list x1,y1[,x2,y2...]")
        self.gate = [(float(gate[i]), float(gate[i + 1])) for i in range(0, len(gate), 2)]
        self.radius = float(self.declare_parameter("radius", 0.7).value)
        self.color = self.declare_parameter("color", "big_orange").value  # big_orange | orange
        self.pose_max_age = float(self.declare_parameter("pose_max_age", 0.5).value)
        # graph_slam only lets an orange observation certify the gate when its sigma is
        # below csm_orange_max_obs_sigma (0.45 m); uncoloured LiDAR cones carry 0.48 m, so
        # a relabelled cone keeps its position but gets this (lidar-grade) sigma. <= 0 keeps
        # the original covariance.
        self.orange_sigma = float(self.declare_parameter("orange_sigma", 0.26).value)
        self.poses = []  # (stamp, x, y, yaw), sorted
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(ConeArrayWithCovariance, self.output_topic, qos)
        self.create_subscription(ConeArrayWithCovariance, self.input_topic, self.on_cones, qos)
        # perception publishes its RViz markers on <output_cones_topic>/viz; forward them under the
        # relayed name too so the "Live Cones" display keeps working without reconfiguring RViz.
        self.viz_pub = self.create_publisher(MarkerArray, self.output_topic.rstrip("/") + "/viz", qos)
        self.create_subscription(MarkerArray, self.input_topic.rstrip("/") + "/viz", self.viz_pub.publish, qos)
        if self.pose_type == "odometry":
            self.create_subscription(Odometry, self.pose_topic, self.on_pose, 50)
        else:
            self.create_subscription(CarState, self.pose_topic, self.on_pose, 50)
        self.relabelled = 0
        self.frames = 0
        self.get_logger().info(
            f"fake gate: {len(self.gate)} point(s) {self.gate} r={self.radius} m -> {self.color}; "
            f"{self.input_topic} -> {self.output_topic}, pose from {self.pose_topic} ({self.pose_type}), "
            f"orange_sigma={self.orange_sigma}")
        self.create_timer(5.0, self.report)

    def on_pose(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        if self.poses and stamp < self.poses[-1][0] - 1.0:
            self.poses.clear()  # bag looped / restarted: time went backwards
        self.poses.append((stamp, p.position.x, p.position.y, yaw_of(p.orientation)))
        if len(self.poses) > 400:
            del self.poses[:100]

    def pose_at(self, stamp):
        if not self.poses:
            return None
        keys = [p[0] for p in self.poses]
        k = bisect.bisect_left(keys, stamp)
        cands = [self.poses[i] for i in (k - 1, k) if 0 <= i < len(self.poses)]
        best = min(cands, key=lambda p: abs(p[0] - stamp))
        return best if abs(best[0] - stamp) <= self.pose_max_age else None

    def on_cones(self, msg):
        self.frames += 1
        pose = self.pose_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if pose is None:
            self.pub.publish(msg)
            return
        _, ex, ey, eyaw = pose
        c, s = math.cos(eyaw), math.sin(eyaw)
        out = ConeArrayWithCovariance()
        out.header = msg.header
        sink = out.big_orange_cones if self.color == "big_orange" else out.orange_cones
        r2 = self.radius * self.radius

        def is_gate(cone):
            mx = ex + c * cone.point.x - s * cone.point.y
            my = ey + s * cone.point.x + c * cone.point.y
            return any((mx - gx) ** 2 + (my - gy) ** 2 <= r2 for gx, gy in self.gate)

        for src, dst in ((msg.blue_cones, out.blue_cones), (msg.yellow_cones, out.yellow_cones),
                         (msg.orange_cones, out.orange_cones), (msg.big_orange_cones, out.big_orange_cones),
                         (msg.unknown_color_cones, out.unknown_color_cones)):
            for cone in src:
                if dst is not sink and is_gate(cone):
                    if self.orange_sigma > 0.0:
                        v = self.orange_sigma * self.orange_sigma
                        cone.covariance = [v, 0.0, 0.0, v]
                    sink.append(cone)
                    self.relabelled += 1
                else:
                    dst.append(cone)
        self.pub.publish(out)

    def report(self):
        self.get_logger().info(f"frames {self.frames}, cones relabelled so far {self.relabelled}")


def main():
    rclpy.init()
    node = FakeOrangeGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
