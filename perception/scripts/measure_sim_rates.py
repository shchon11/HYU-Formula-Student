"""The sensors' rate in SIM TIME, read off their own stamps.

`ros2 topic hz` measures wall clock. For a simulator that is the wrong clock:
perception, SLAM and planning all run on sim time, so what they experience is
the interval between message STAMPS, not between arrivals. If the sim runs at
RTF 0.83, a sensor firing every 33 ms of sim time shows up as 25 Hz on a wall
clock and is still, to everything that consumes it, a 30 Hz sensor.

This reads consecutive stamps and reports 1/median(delta), which needs no RTF
estimate at all.
"""
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from hyu_msgs.msg import BoundingBoxes, ConeArrayWithCovariance

QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                 reliability=ReliabilityPolicy.BEST_EFFORT)
TOPICS = [
    ("/zed/left/image_rect_color", Image),
    ("/zed/right/image_rect_color", Image),
    ("/velodyne_points", PointCloud2),
    ("/yolo_bounding_boxes", BoundingBoxes),
    ("/cones", ConeArrayWithCovariance),
]


def stamp_of(msg):
    header = getattr(msg, "image_header", None)
    if header is not None and (header.stamp.sec or header.stamp.nanosec):
        return header.stamp
    return msg.header.stamp


class Probe(Node):
    def __init__(self):
        super().__init__("sim_rate_probe")
        self.stamps = {name: [] for name, _ in TOPICS}
        self.arrivals = {name: [] for name, _ in TOPICS}
        for name, kind in TOPICS:
            self.create_subscription(
                kind, name, lambda m, n=name: self._on(n, m), QOS)

    def _on(self, name, msg):
        s = stamp_of(msg)
        self.stamps[name].append(int(s.sec) * 1_000_000_000 + int(s.nanosec))
        self.arrivals[name].append(self.get_clock().now().nanoseconds)


rclpy.init()
node = Probe()
node.set_parameters([rclpy.parameter.Parameter(
    "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
import time
end = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 25)
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)

print(f"\n{'topic':<32} {'n':>4} {'SIM Hz':>8} {'jitter':>9}")
for name, _ in TOPICS:
    s = sorted(set(node.stamps[name]))
    if len(s) < 5:
        print(f"{name:<32} {len(s):4d}   (too few)")
        continue
    deltas = [(b - a) / 1e9 for a, b in zip(s, s[1:]) if b > a]
    if not deltas:
        continue
    median = statistics.median(deltas)
    spread = statistics.pstdev(deltas) if len(deltas) > 1 else 0.0
    print(f"{name:<32} {len(s):4d} {1.0 / median:8.2f} {1000 * spread:8.1f}ms")
node.destroy_node()
rclpy.try_shutdown()
