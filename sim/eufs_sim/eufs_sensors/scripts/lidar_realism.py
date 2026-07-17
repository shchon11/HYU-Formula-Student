#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
LiDAR realism bridge: gazebo's ideal scan -> the real Puck driver's output.

Sits between the gazebo ray sensor (``/velodyne_points_ideal``) and the
stack's LiDAR topic (``/velodyne_points``) and closes three sim-to-real gaps
the render itself cannot:

1. **Field parity.** The real pipeline reads velodyne_pointcloud's layout
   ``x,y,z,intensity(f32) ring(u16) time(f32)``, point_step 22. Gazebo emits
   only xyz+intensity, so any code touching ``ring``/``time`` (deskewing,
   ring-wise ground removal) would run on real bags and silently not in sim.
   ``ring`` is recovered from each beam's elevation (VLP-16: ring 0 = -15 deg,
   2 deg spacing), ``time`` from its azimuth and the sweep model below.
   Intensity is passed through untouched: gazebo's value carries no material
   model, and inventing a plausible one would be worse than an obviously
   flat signal. Do not train or threshold on sim intensity.

2. **Motion distortion.** Gazebo renders the whole revolution at ONE sim
   instant; the real Puck sweeps it over 1/hz while the car moves — at
   6 m/s a rev smears 60 cm across the seam, which is a *sensor* effect that
   exists before any pose estimate. Each point is re-expressed in the sensor
   frame at its own measurement time using the ground-truth twist
   (constant-twist, planar approximation over the 100 ms rev):
   ``p' = R(-w*tau) @ (p - v_sensor*tau)``. A perfect deskewer applying the
   true motion recovers the ideal cloud exactly — that is the contract that
   makes the perception/SLAM deskew path testable in sim at all.

3. **Received-power dropout.** Gazebo returns every beam out to the 100 m
   range cap; a real Puck does not. Two fades, same smooth-band mechanics:
   grazing ground (``snr ~ cos(incidence)/R^2`` with flat-ground incidence
   from the beam's world elevation, knee at ``max_ground_range``) and a
   reflectivity-scaled object ceiling (``max_object_range``, the 100 m
   datasheet range derated by sqrt(rho_object/rho_spec) for ordinary
   non-retro surfaces). At the defaults nothing inside ~25 m is touched —
   cones keep >10x snr margin — only the far fiction dies.

   Deliberately NO per-band (black stripe) cone dropout: the received-power
   physics puts a rho~0.05 stripe's ceiling at ~100*sqrt(0.05/0.8) = 25 m,
   outside the 10-12 m perception envelope, so a stripe model would change
   nothing where it matters while pretending a precision no one has
   measured. When the car exists, sensor_parity_report.py's points-per-cone
   curve against a real bag is the evidence that would justify one. Rain,
   dust and retro-stripe saturation stay un-modelled for the same reason.

Conventions (all parameters, all must match the real driver before comparing
bags across domains):

* ``spin``: ``cw`` — a VLP-16 rotates clockwise seen from above (+z), i.e.
  azimuth decreasing in the REP-103 sensor frame. Sweep runs +pi -> -pi, so
  the seam sits at the rear, which is also where gazebo's sample grid wraps.
* ``stamp_at``: ``end`` — the humble velodyne_driver default
  (``timestamp_first_packet:=false``) stamps a scan at its LAST packet, so
  per-point ``time`` offsets are NEGATIVE (-T..0] and the message stamp is
  the freshest geometry. That matches gazebo's stamp being the render
  instant, and keeps stereo/LiDAR stamp-pairing semantics unchanged. Set
  ``start`` if the real driver runs timestamp_first_packet:=true.
* The scan period T is 1/``spin_hz`` and must match the VLP-16R macro's
  ``hz`` (both default 10).

The sensor's mount pose (lever arm for the twist, height and pitch for the
ground-incidence proxy) is read from TF (``base_frame`` -> cloud frame_id,
published by robot_state_publisher from the same URDF that placed the
sensor); the ``sensor_*`` parameters are only the fallback if TF never
arrives. Ground-truth twist comes from ``/ground_truth/state`` (body frame).
If it goes stale the node logs and publishes undistorted — degrading to the
old behaviour, never blocking the topic.

Verification: tools in the session scratchpad drive the module functions
directly (deskew inverse recovery, ring mapping, dropout margins); e2e, run
the sim and check ``ros2 topic echo --field fields /velodyne_points`` shows
the 6-field layout and that a moving car's cloud shears near the seam.
"""

import math
import sys

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from hyu_msgs.msg import CarState
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

# Real velodyne_pointcloud (humble) PointXYZIRT wire layout: 22-byte points.
_OUT_DTYPE = np.dtype(
    {
        "names": ["x", "y", "z", "intensity", "ring", "time"],
        "formats": [np.float32, np.float32, np.float32, np.float32, np.uint16, np.float32],
        "offsets": [0, 4, 8, 12, 16, 18],
        "itemsize": 22,
    }
)

_OUT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
    PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1),
]

# VLP-16 vertical grid: ring 0 at -15 deg, 2 deg spacing, bottom-up (the
# velodyne_pointcloud "ring" convention, ordered by angle not firing order).
_RING_MIN_DEG = -15.0
_RING_STEP_DEG = 2.0
_NUM_RINGS = 16


def sweep_progress(azimuth, spin):
    """
    Fraction of the revolution completed when this azimuth is measured.

    cw: +pi -> -pi (VLP-16 seen from above); ccw: -pi -> +pi.
    """
    if spin == "cw":
        return (math.pi - azimuth) / (2.0 * math.pi)
    return (azimuth + math.pi) / (2.0 * math.pi)


def ring_from_elevation(elev_rad):
    """Nearest VLP-16 ring for a beam elevation (bottom-up numbering)."""
    idx = np.rint((np.degrees(elev_rad) - _RING_MIN_DEG) / _RING_STEP_DEG)
    return np.clip(idx, 0, _NUM_RINGS - 1).astype(np.uint16)


def distort(xyz, tau, v_sensor_x, v_sensor_y, omega):
    """
    Express each point in the sensor frame at its own measurement time.

    The scene is true at tau=0 (the message stamp). At tau the sensor has
    moved v*tau and yawed omega*tau (constant twist, planar), so the point
    it reports is p' = R(-omega*tau) @ (p - v*tau).
    """
    theta = omega * tau
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    px = xyz[:, 0] - v_sensor_x * tau
    py = xyz[:, 1] - v_sensor_y * tau
    out = np.empty_like(xyz)
    out[:, 0] = cos_t * px + sin_t * py
    out[:, 1] = -sin_t * px + cos_t * py
    out[:, 2] = xyz[:, 2]
    return out


def keep_mask(rng, elev_world, sensor_height, max_ground_range,
              max_object_range, uniform):
    """
    Received-power dropout: snr ~ cos(incidence)/R^2 vs two thresholds.

    Ground: flat-ground incidence (cos = sin|elev|) for below-horizon
    beams, threshold calibrated so flat asphalt stops returning around
    max_ground_range. Objects: a near-normal ceiling at max_object_range
    (datasheet range derated for ordinary reflectivity). Cones inside the
    perception envelope clear both by >10x — see the module docstring for
    why there is deliberately no per-band stripe model.
    """
    r_sq = np.maximum(rng * rng, 1e-6)
    cos_inc = np.where(elev_world < 0.0, np.sin(np.abs(elev_world)), 1.0)
    ground_margin = (cos_inc / r_sq) / (sensor_height / float(max_ground_range) ** 3)
    object_margin = (1.0 / r_sq) / (1.0 / float(max_object_range) ** 2)
    # Smooth band per fade: full keep above 1.3*thr, gone below 0.7*thr.
    p_keep = (
        np.clip((ground_margin - 0.7) / 0.6, 0.0, 1.0)
        * np.clip((object_margin - 0.7) / 0.6, 0.0, 1.0)
    )
    return uniform < p_keep


class LidarRealism(Node):
    def __init__(self):
        super().__init__("lidar_realism")

        self.input_topic = self.declare_parameter(
            "input_topic", "/velodyne_points_ideal"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/velodyne_points"
        ).value
        self.ground_truth_topic = self.declare_parameter(
            "ground_truth_topic", "/ground_truth/state"
        ).value
        self.distortion = self.declare_parameter("distortion", True).value
        self.dropout = self.declare_parameter("dropout", True).value
        self.spin = self.declare_parameter("spin", "cw").value
        self.stamp_at = self.declare_parameter("stamp_at", "end").value
        self.spin_hz = self.declare_parameter("spin_hz", 10.0).value
        self.max_ground_range = self.declare_parameter("max_ground_range", 28.0).value
        # 100 m datasheet range derated by sqrt(rho/rho_spec) ~ sqrt(0.5/0.8)
        # for ordinary (non-retro) surfaces. Estimate, not measurement.
        self.max_object_range = self.declare_parameter("max_object_range", 80.0).value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.twist_timeout = self.declare_parameter("twist_timeout", 0.5).value
        # Fallbacks if TF never resolves (eufs robot values).
        self.sensor_x = self.declare_parameter("sensor_x", 1.6).value
        self.sensor_y = self.declare_parameter("sensor_y", 0.0).value
        self.sensor_height = self.declare_parameter("sensor_height", 0.6).value
        self.sensor_pitch = self.declare_parameter("sensor_pitch", 0.0175).value
        if self.spin not in ("cw", "ccw"):
            raise ValueError("spin must be cw or ccw")
        if self.stamp_at not in ("end", "start"):
            raise ValueError("stamp_at must be end or start")

        self.rng = np.random.default_rng(self.declare_parameter("seed", 42).value)
        self._tf_resolved = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.twist = None  # (stamp_sec, vx, vy, omega) body frame
        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.on_cloud,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.gt_sub = self.create_subscription(
            CarState,
            self.ground_truth_topic,
            self.on_ground_truth,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.get_logger().info(
            f"lidar realism: {self.input_topic} -> {self.output_topic} "
            f"(distortion={self.distortion}, dropout={self.dropout}, "
            f"spin={self.spin}@{self.spin_hz} Hz, stamp_at={self.stamp_at})"
        )

    # ------------------------------------------------------------------ TF

    def _resolve_mount(self, cloud_frame):
        """Cache the sensor pose in base_frame from the URDF's own TF."""
        if self._tf_resolved:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, cloud_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return  # keep fallbacks until TF shows up; re-try next cloud
        t = tf.transform.translation
        q = tf.transform.rotation
        self.sensor_x, self.sensor_y = t.x, t.y
        # base_footprint sits on the ground plane, so z IS the height.
        self.sensor_height = t.z
        self.sensor_pitch = math.asin(
            max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        )
        self._tf_resolved = True
        self.get_logger().info(
            f"mount from TF {self.base_frame}->{cloud_frame}: "
            f"x={self.sensor_x:.3f} y={self.sensor_y:.3f} "
            f"h={self.sensor_height:.3f} pitch={math.degrees(self.sensor_pitch):.2f} deg"
        )

    # ------------------------------------------------------------- inputs

    def on_ground_truth(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.twist = (
            stamp,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        )

    def _current_twist(self, cloud_stamp):
        if self.twist is None:
            return None
        stamp, vx, vy, omega = self.twist
        if abs(cloud_stamp - stamp) > self.twist_timeout:
            return None
        return vx, vy, omega

    # -------------------------------------------------------------- cloud

    def on_cloud(self, msg):
        self._resolve_mount(msg.header.frame_id)

        names = {f.name for f in msg.fields}
        wanted = ("x", "y", "z", "intensity") if "intensity" in names else ("x", "y", "z")
        try:
            structured = point_cloud2.read_points(
                msg, field_names=wanted, skip_nans=True
            )
        except Exception:  # pragma: no cover - malformed cloud
            self.get_logger().warn("unreadable input cloud; dropping frame")
            return
        arr = np.asarray(structured)
        n = arr.size
        if n == 0:
            self._publish(msg, np.empty((0, 3)), np.empty(0), np.empty(0), np.empty(0))
            return
        xyz = np.column_stack(
            [arr["x"], arr["y"], arr["z"]]
        ).astype(np.float64)
        intensity = (
            arr["intensity"].astype(np.float32)
            if "intensity" in wanted
            else np.zeros(n, dtype=np.float32)
        )
        # skip_nans passes +-inf (no-return beams); drop them explicitly.
        finite = np.isfinite(xyz).all(axis=1)
        if not finite.all():
            xyz, intensity = xyz[finite], intensity[finite]
            n = xyz.shape[0]

        # Beam geometry of the ideal measurement (pre-distortion: ring and
        # timing are properties of the beam, not of where the car moved).
        planar = np.hypot(xyz[:, 0], xyz[:, 1])
        rng = np.sqrt(planar * planar + xyz[:, 2] * xyz[:, 2])
        azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
        elev = np.arctan2(xyz[:, 2], planar)

        progress = sweep_progress(azimuth, self.spin)
        period = 1.0 / self.spin_hz
        tau = (progress - (1.0 if self.stamp_at == "end" else 0.0)) * period
        ring = ring_from_elevation(elev)

        keep = np.ones(n, dtype=bool)
        if self.dropout:
            # World-frame elevation: the mount pitch tilts every beam.
            elev_world = elev - self.sensor_pitch
            keep = keep_mask(
                rng,
                elev_world,
                max(self.sensor_height, 0.05),
                self.max_ground_range,
                self.max_object_range,
                self.rng.random(n),
            )

        distorted = False
        if self.distortion:
            cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            twist = self._current_twist(cloud_stamp)
            if twist is None:
                self.get_logger().warn(
                    "no fresh ground-truth twist; publishing undistorted",
                    throttle_duration_sec=5.0,
                )
            else:
                vx, vy, omega = twist
                # Body twist at the sensor's lever arm (sensor yaw is 0).
                v_sx = vx - omega * self.sensor_y
                v_sy = vy + omega * self.sensor_x
                xyz = distort(xyz, tau, v_sx, v_sy, omega)
                distorted = True

        # The time field is the deskew protocol: downstream inverts the
        # distortion whenever it is populated. Publishing real tau on an
        # UNDISTORTED cloud makes the consumer inject the full v*tau smear
        # it was built to remove — zero it so the inverse degenerates to
        # the identity.
        publish_tau = tau[keep] if distorted else np.zeros(
            int(np.count_nonzero(keep)), dtype=tau.dtype)
        self._publish(msg, xyz[keep], intensity[keep], ring[keep], publish_tau)

    def _publish(self, msg, xyz, intensity, ring, tau):
        n = xyz.shape[0]
        out = np.zeros(n, dtype=_OUT_DTYPE)
        if n:
            out["x"] = xyz[:, 0].astype(np.float32)
            out["y"] = xyz[:, 1].astype(np.float32)
            out["z"] = xyz[:, 2].astype(np.float32)
            out["intensity"] = intensity
            out["ring"] = ring
            out["time"] = tau.astype(np.float32)

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = n
        cloud.fields = _OUT_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = _OUT_DTYPE.itemsize
        cloud.row_step = _OUT_DTYPE.itemsize * n
        cloud.is_dense = True
        cloud.data = out.tobytes()
        self.pub.publish(cloud)


def main():
    rclpy.init(args=sys.argv)
    node = LidarRealism()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
