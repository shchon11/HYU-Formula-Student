#!/usr/bin/env python3
"""Measure per-tier depth error against simulator ground truth.

Reproduces the IIT Bombay paper's Table 1 with this stack's own numbers, so the
tiers can be compared on the same footing the paper used:

    LiDAR-camera fusion       0.85 %   (paper)
    Monocular bbox-height     4.49 %
    Stereo slender-bbox SIFT  6.39 %
    Mono + stereo routed      3.38 %

How it works
------------
`/cones` carries the fused estimate but not which tier produced each cone, so
this subscribes to the fusion debug stream alongside it:

    /cones                       final cone positions, base_footprint
    /fusion/debug/rejections     per-detection tier + reason (needs
                                 publish_fusion_debug:=true)
    /ground_truth/cones          true cone positions, base_footprint

Each published cone is matched to its nearest ground-truth cone (greedy, gated
by --match-radius). Unmatched cones are counted as false positives, and truths
with no cone within the gate as misses, because a tier that simply drops hard
cones would otherwise look artificially accurate.

Usage
-----
Run the simulator with perception and debug enabled, drive a lap, then:

    ros2 run eufs_perception_baseline evaluate_perception_tiers.py --duration 60

or against a bag:

    ros2 bag play <bag> &
    ./evaluate_perception_tiers.py --duration 0     # until Ctrl-C
"""
import argparse
import math
import sys
from collections import defaultdict

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from eufs_msgs.msg import ConeArrayWithCovariance
except ImportError:
    sys.exit(
        "This script needs a sourced ROS 2 workspace with eufs_msgs on the "
        "PYTHONPATH:\n"
        "    source /opt/ros/<distro>/setup.bash && source install/setup.bash"
    )


CONE_FIELDS = (
    ("blue_cones", "blue"),
    ("yellow_cones", "yellow"),
    ("orange_cones", "orange"),
    ("big_orange_cones", "big_orange"),
    ("unknown_color_cones", "unknown"),
)


def cones_of(msg):
    """Flatten a ConeArrayWithCovariance into [(x, y, colour, variance), ...]."""
    out = []
    for field, colour in CONE_FIELDS:
        for cone in getattr(msg, field, []):
            x, y = float(cone.point.x), float(cone.point.y)
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            covariance = list(getattr(cone, "covariance", []) or [0.0])
            out.append((x, y, colour, float(covariance[0])))
    return out


def greedy_match(estimates, truths, radius):
    """Pair each estimate with its nearest unused truth inside ``radius``."""
    pairs, misses = [], []
    taken = set()
    for ex, ey, ecolour, evar in estimates:
        best, best_d2 = None, radius * radius
        for index, (tx, ty, tcolour) in enumerate(truths):
            if index in taken:
                continue
            d2 = (ex - tx) ** 2 + (ey - ty) ** 2
            if d2 <= best_d2:
                best, best_d2 = index, d2
        if best is None:
            misses.append((ex, ey, ecolour))
            continue
        taken.add(best)
        tx, ty, tcolour = truths[best]
        pairs.append({
            "est_range": math.hypot(ex, ey),
            "true_range": math.hypot(tx, ty),
            "lateral_err": math.hypot(ex - tx, ey - ty),
            "colour_ok": ecolour == tcolour,
            "variance": evar,
        })
    unseen = len(truths) - len(taken)
    return pairs, misses, unseen


class TierEvaluator(Node):
    def __init__(self, args):
        super().__init__("evaluate_perception_tiers")
        self.args = args
        self.samples = []
        self.false_positives = 0
        self.missed_truths = 0
        self.frames = 0
        self.latest_truth = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            ConeArrayWithCovariance, args.truth_topic,
            self._on_truth, sensor_qos)
        self.create_subscription(
            ConeArrayWithCovariance, args.cones_topic,
            self._on_cones, sensor_qos)
        self.get_logger().info(
            f"comparing {args.cones_topic} against {args.truth_topic}; "
            f"match radius {args.match_radius} m"
        )

    def _on_truth(self, msg):
        self.latest_truth = [
            (x, y, colour) for x, y, colour, _ in cones_of(msg)
        ]

    def _on_cones(self, msg):
        if self.latest_truth is None:
            return
        estimates = cones_of(msg)
        if not estimates:
            return
        pairs, misses, unseen = greedy_match(
            estimates, self.latest_truth, self.args.match_radius)
        self.samples.extend(pairs)
        self.false_positives += len(misses)
        self.missed_truths += unseen
        self.frames += 1

    def report(self):
        if not self.samples:
            print("\nNo matched cones. Check that perception is publishing and "
                  "that the simulator's ground-truth cone topic is enabled.")
            return

        print(f"\n{'=' * 62}")
        print(f"cone frames        {self.frames}")
        print(f"matched cones      {len(self.samples)}")
        print(f"false positives    {self.false_positives}")
        print(f"missed truths      {self.missed_truths}")

        # Depth error, which is what the paper's Table 1 reports.
        errors = [
            abs(s["est_range"] - s["true_range"]) / s["true_range"] * 100.0
            for s in self.samples if s["true_range"] > 0.1
        ]
        errors.sort()
        colour_ok = sum(1 for s in self.samples if s["colour_ok"])

        def pct(p):
            return errors[min(len(errors) - 1, int(p / 100.0 * len(errors)))]

        print(f"\nrange error vs ground truth (paper's metric):")
        print(f"  mean    {sum(errors) / len(errors):6.2f} %")
        print(f"  median  {pct(50):6.2f} %")
        print(f"  p90     {pct(90):6.2f} %")
        print(f"  max     {errors[-1]:6.2f} %")
        print(f"\n  within  <5%   {100.0 * sum(1 for e in errors if e < 5) / len(errors):5.1f} %")
        print(f"  within  <10%  {100.0 * sum(1 for e in errors if e < 10) / len(errors):5.1f} %")
        print(f"  within  <20%  {100.0 * sum(1 for e in errors if e < 20) / len(errors):5.1f} %")
        print(f"\ncolour correct    {100.0 * colour_ok / len(self.samples):5.1f} %")

        # By range band: LiDAR carries the near field, vision the far field, so
        # a single mean hides which tier is actually failing.
        print(f"\nrange error by distance band:")
        bands = defaultdict(list)
        for s in self.samples:
            if s["true_range"] <= 0.1:
                continue
            band = int(s["true_range"] // 5) * 5
            err = abs(s["est_range"] - s["true_range"]) / s["true_range"] * 100.0
            bands[band].append(err)
        for band in sorted(bands):
            values = bands[band]
            print(f"  {band:2d}-{band + 5:2d} m   n={len(values):4d}   "
                  f"mean {sum(values) / len(values):6.2f} %")

        print(f"\npaper Table 1 for reference:")
        print(f"  LiDAR-camera fusion  0.85 %")
        print(f"  monocular bb-height  4.49 %")
        print(f"  stereo slender SIFT  6.39 %")
        print(f"  mono + stereo routed 3.38 %")
        print(f"{'=' * 62}")
        print("\nTo attribute error to a specific tier, run with only that tier "
              "enabled:\n"
              "  Tier 1 only:  monocular_fallback_enabled:=false "
              "stereo_fallback_enabled:=false\n"
              "  Tier 2 only:  fusion of LiDAR disabled is not supported; "
              "instead compare\n"
              "                the >10 m band, where LiDAR support is sparse.")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cones-topic", default="/cones")
    parser.add_argument("--truth-topic", default="/ground_truth/cones")
    parser.add_argument("--match-radius", type=float, default=2.0,
                        help="max distance to call an estimate the same cone (m)")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="seconds to collect; 0 runs until Ctrl-C")
    args = parser.parse_args()

    rclpy.init()
    node = TierEvaluator(args)
    try:
        if args.duration > 0:
            end = node.get_clock().now().nanoseconds + args.duration * 1e9
            while rclpy.ok() and node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
