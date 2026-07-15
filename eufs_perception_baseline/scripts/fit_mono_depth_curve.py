#!/usr/bin/env python3
"""Fit the Tier-2 monocular depth curve to the camera actually in use.

Tier 2 estimates depth from the bounding-box height alone:

    D = c * h_n ^ e,    h_n = bbox_height_px / image_height_px

``c`` and ``e`` are camera- and cone-specific.  The IIT Bombay paper reports
c=0.498, e=-0.954 for their ZED 2i and their cones, and states the fit must be
redone whenever camera placement or optics change.  Carrying their constants to
a different rig silently biases every monocular cone -- on this simulator's ZED
they overestimate depth by roughly 50%.

Two ways to use this:

  --analytic
      Print the exact pinhole curve implied by the camera intrinsics and the
      cone height.  For an undistorted (simulated) camera this is the ground
      truth: h_px = fy*H_cone/D, so e = -1 exactly and c = fy*H_cone/H_img.

  --live
      Fit c and e empirically by pairing the detector's own boxes with the true
      cone depths, against a running simulator.  This is the one to use: the
      analytic curve assumes the detector's box is EXACT, and it is not -- this
      one runs ~17% short past 6 m, which bends the exponent away from -1 and is
      the same reason the paper carries -0.954.

      Re-fit per camera AND per detector weight. On the real car there is no
      ground truth: use Tier-1 LiDAR depth as the reference instead.

Examples
--------
    # What should the constants be for this simulator's ZED?
    ./fit_mono_depth_curve.py --analytic --cone-height 0.450

    # Fit against a running simulator (drive it while this collects).
    ./fit_mono_depth_curve.py --live --duration 60
"""
import argparse
import math
import sys


def analytic_curve(fx_px, image_height_px, cone_height_m):
    """Exact pinhole curve: a rigid cone's pixel height is inversely linear in depth."""
    coefficient = fx_px * cone_height_m / image_height_px
    return coefficient, -1.0


def fx_from_fov(width_px, horizontal_fov_rad):
    return (width_px / 2.0) / math.tan(horizontal_fov_rad / 2.0)


def fit_power_law(normalized_heights, depths):
    """Least-squares fit of D = c * h^e, solved linearly in log space."""
    try:
        import numpy as np
    except ImportError:
        sys.exit("numpy is required to fit; pip install numpy")

    h = np.asarray(normalized_heights, dtype=float)
    d = np.asarray(depths, dtype=float)
    keep = (h > 0) & (d > 0) & np.isfinite(h) & np.isfinite(d)
    h, d = h[keep], d[keep]
    if h.size < 2:
        sys.exit(f"need at least 2 valid samples, got {h.size}")

    # log D = log c + e * log h  ->  ordinary least squares on [log h, 1]
    design = np.column_stack([np.log(h), np.ones(h.size)])
    (exponent, log_c), *_ = np.linalg.lstsq(design, np.log(d), rcond=None)
    coefficient = float(np.exp(log_c))

    predicted = coefficient * h ** exponent
    relative_error = np.abs(predicted - d) / d
    return coefficient, float(exponent), h.size, relative_error


def report_fit(coefficient, exponent, count, relative_error):
    import numpy as np

    print(f"\nfitted on {count} cone observations")
    print(f"  monocular_depth_coefficient: {coefficient:.4f}")
    print(f"  monocular_depth_exponent:    {exponent:.4f}")
    print("\nresidual error vs ground truth:")
    print(f"  mean   {100 * float(np.mean(relative_error)):5.2f}%")
    print(f"  median {100 * float(np.median(relative_error)):5.2f}%")
    print(f"  p90    {100 * float(np.percentile(relative_error, 90)):5.2f}%")
    print(f"  max    {100 * float(np.max(relative_error)):5.2f}%")
    print("\nThe paper reports 4.49% mean error for its own fitted curve.")


def collect_live(args):
    """Pair every detected box with the TRUE depth of the cone it is on.

    The procedure, and the reason for each step:

    - Truth comes from ``/ground_truth/track``, not ``/ground_truth/cones``:
      the latter is FOV/range-filtered by the plugin, and fitting a curve on a
      filtered sample fits the filter as much as the camera.
    - Truth is map-frame, so it is carried to the camera's optical frame at the
      BBOX stamp. The detector's boxes trail sim time by its own latency, and
      pairing them against the newest truth would fit how far the car drove.
    - Association is done in PIXELS, not in metres. Projected cones are metres
      apart on the image at the ranges that matter, so the match is unambiguous
      -- where a metric match would need a depth we do not have yet, which is
      the thing being fitted.
    - ``D`` is the truth cone's optical **Z**, because that is what the node
      back-projects the box centre at.
    """
    try:
        import numpy as np
        import rclpy
        import tf2_ros
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from eufs_msgs.msg import BoundingBoxes, ConeArrayWithCovariance
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import CameraInfo
    except ImportError as exc:
        sys.exit(f"--live needs a sourced ROS 2 workspace with eufs_msgs: {exc}")

    CONE_FIELDS = ("blue_cones", "yellow_cones", "orange_cones",
                   "big_orange_cones", "unknown_color_cones")

    def stamp_ns(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    class Collector(Node):
        def __init__(self):
            super().__init__("fit_mono_depth_curve")
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.samples = []
            self.truth = None
            self.odom = []
            self.camera_info = None
            self.skipped_no_tf = 0
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.create_subscription(ConeArrayWithCovariance, args.truth_topic,
                                     self._on_truth, qos)
            self.create_subscription(Odometry, args.odom_topic, self._on_odom, qos)
            self.create_subscription(CameraInfo, args.camera_info_topic,
                                     self._on_info, qos)
            self.create_subscription(BoundingBoxes, args.bbox_topic,
                                     self._on_bboxes, qos)

        def _on_info(self, msg):
            self.camera_info = msg

        def _on_odom(self, msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.odom.append((stamp_ns(msg.header),
                              (msg.pose.pose.position.x,
                               msg.pose.pose.position.y, yaw)))
            del self.odom[:-4000]

        def _on_truth(self, msg):
            cones = []
            for field in CONE_FIELDS:
                for cone in getattr(msg, field, []):
                    cones.append((cone.point.x, cone.point.y))
            self.truth = (stamp_ns(msg.header), msg.header.frame_id, cones)

        def _nearest_odom(self, target_ns):
            best, best_delta = None, None
            for stamp, pose in self.odom:
                delta = abs(stamp - target_ns)
                if best_delta is None or delta < best_delta:
                    best, best_delta = pose, delta
            return best, best_delta

        def _on_bboxes(self, msg):
            if self.camera_info is None or self.truth is None:
                return
            header = msg.image_header if (msg.image_header.stamp.sec
                                          or msg.image_header.stamp.nanosec) \
                else msg.header
            target = stamp_ns(header)
            truth_stamp, truth_frame, cones = self.truth
            if abs(truth_stamp - target) > args.max_stamp_skew_sec * 1e9:
                return
            pose, delta = self._nearest_odom(target)
            if pose is None or delta > args.max_stamp_skew_sec * 1e9:
                return

            # map -> base_footprint, then base -> camera optical via TF.
            car_x, car_y, car_yaw = pose
            cos_y, sin_y = math.cos(-car_yaw), math.sin(-car_yaw)
            try:
                tf = self.tf_buffer.lookup_transform(
                    args.camera_frame, args.output_frame, header.stamp)
            except tf2_ros.TransformException:
                self.skipped_no_tf += 1
                return
            t, q = tf.transform.translation, tf.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            rotation = np.asarray([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ])
            translation = np.asarray([t.x, t.y, t.z])

            matrix = np.asarray(self.camera_info.p, dtype=float)
            matrix = (matrix.reshape(3, 4)[:, :3] if matrix.size == 12
                      else np.asarray(self.camera_info.k, dtype=float).reshape(3, 3))
            fx, fy = matrix[0, 0], matrix[1, 1]
            cx, cy = matrix[0, 2], matrix[1, 2]

            projected = []
            for map_x, map_y in cones:
                dx, dy = map_x - car_x, map_y - car_y
                base = np.asarray([dx * cos_y - dy * sin_y,
                                   dx * sin_y + dy * cos_y, 0.0])
                cam = rotation.dot(base) + translation
                if cam[2] <= args.min_depth_m:
                    continue
                projected.append((fx * cam[0] / cam[2] + cx,
                                  fy * cam[1] / cam[2] + cy, float(cam[2])))
            if not projected:
                return

            for box in msg.bounding_boxes:
                height_px = float(box.ymax) - float(box.ymin)
                if height_px <= 0.0:
                    continue
                u = 0.5 * (float(box.xmin) + float(box.xmax))
                # The box is the cone's full extent; its centre sits above the
                # cone's base, which is what the truth point is. Match on the
                # BOTTOM edge, which is the base, and on u.
                v = float(box.ymax)
                best, best_d2 = None, args.match_radius_px ** 2
                for px, py, depth in projected:
                    d2 = (u - px) ** 2 + (v - py) ** 2
                    if d2 <= best_d2:
                        best, best_d2 = depth, d2
                if best is None:
                    continue
                self.samples.append(
                    (height_px / float(self.camera_info.height), best))

    rclpy.init()
    node = Collector()
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds <= 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        end = node.get_clock().now().nanoseconds + args.duration * 1e9
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    if node.skipped_no_tf:
        print(f"note: {node.skipped_no_tf} frames had no camera TF at their stamp")
    samples = node.samples
    node.destroy_node()
    rclpy.try_shutdown()
    if not samples:
        sys.exit("no box/cone pairs collected -- is the sim running and driving?")
    return [h for h, _ in samples], [d for _, d in samples]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--analytic", action="store_true",
                        help="compute the exact curve from camera intrinsics")
    parser.add_argument("--cone-height", type=float, default=0.450,
                        help="cone height in metres (simulator default: 0.450)")
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--hfov", type=float, default=1.91986,
                        help="horizontal FOV in radians (simulator ZED: 1.91986)")
    parser.add_argument("--fx", type=float, default=None,
                        help="focal length in pixels; overrides --hfov")
    parser.add_argument("--live", action="store_true",
                        help="fit against a running simulator")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="seconds of sim time to collect for --live")
    parser.add_argument("--bbox-topic", default="/yolo_bounding_boxes")
    parser.add_argument("--truth-topic", default="/ground_truth/track",
                        help="the UNFILTERED full track; /ground_truth/cones is "
                             "FOV-filtered by the plugin and fitting on it fits "
                             "the filter as much as the camera")
    parser.add_argument("--odom-topic", default="/ground_truth/odom")
    parser.add_argument("--camera-info-topic", default="/zed/left/camera_info")
    parser.add_argument("--camera-frame", default="zed_left_camera_optical_frame")
    parser.add_argument("--output-frame", default="base_footprint")
    parser.add_argument("--match-radius-px", type=float, default=40.0,
                        help="max pixel distance to call a box and a projected "
                             "cone the same cone")
    parser.add_argument("--max-stamp-skew-sec", type=float, default=0.05)
    parser.add_argument("--min-depth-m", type=float, default=0.5)
    args = parser.parse_args()

    fx = args.fx if args.fx else fx_from_fov(args.image_width, args.hfov)

    if args.live:
        heights, depths = collect_live(args)
        report_fit(*fit_power_law(heights, depths))
        return

    if not args.analytic:
        parser.error("pass --analytic, or --live to fit against a running sim")

    coefficient, exponent = analytic_curve(fx, args.image_height, args.cone_height)
    print(f"camera:  {args.image_width}x{args.image_height}, fx = {fx:.2f} px")
    print(f"cone:    {args.cone_height} m tall")
    print("\nexact pinhole curve for this camera:")
    print(f"  monocular_depth_coefficient: {coefficient:.4f}")
    print(f"  monocular_depth_exponent:    {exponent:.1f}")
    print("\n  (the exponent is -1 exactly because a simulated camera has no")
    print("   lens distortion; a real lens bends it slightly, which is what the")
    print("   paper's -0.954 absorbs. Fit with --bag on real hardware.)")

    print(f"\n{'true D':>8} {'bbox h':>8} {'this curve':>12} {'paper curve':>12}")
    for depth in (2.0, 3.0, 5.0, 10.0, 15.0, 20.0):
        h_px = fx * args.cone_height / depth
        h_n = h_px / args.image_height
        ours = coefficient * h_n ** exponent
        paper = 0.498 * h_n ** -0.954
        print(f"{depth:7.1f}m {h_px:7.1f}px {ours:11.2f}m {paper:11.2f}m")


if __name__ == "__main__":
    main()
