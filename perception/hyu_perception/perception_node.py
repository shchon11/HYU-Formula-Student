"""Cone perception: a LiDAR backbone, extended by bounded, cross-checked vision.

The design, in one paragraph
---------------------------
LiDAR carries the **position**. A camera detection that lands on LiDAR gives it a
**colour**; LiDAR the camera never confirmed still publishes, as
``unknown_color``. A detection with **no** LiDAR support falls back to a
monocular bbox-height estimate, but **only inside the range where that estimate
is trustworthy**, and it is cross-checked against stereo before it is believed.
Everything else is dropped. What this buys is latency and robustness, and both
come from the structure rather than from tuning.

Three decisions carry it
------------------------
**The camera drives the output.** Every bbox frame publishes ``/perception/cones``. There is
no one-to-one bbox/LiDAR pairing: that tied the output rate to the *worse* of the
two sensors, and was measured to hold ``/perception/cones`` at ~0.6x the LiDAR rate even
with the node otherwise idle at 28% CPU. Nothing here waits.

**Heavy work runs at the rate of the thing it depends on.** Clustering is a
property of the cloud, so it happens once per cloud (~10 Hz) and is cached; the
bbox callback motion-compensates that cache to its own stamp. Doing it per output
frame would re-cluster the same points at the camera's rate for no new
information.

**Vision is bounded, and the bound is measured.** ``monocular_min_bbox_height_px``
is not a tuning knob: past ~9 m this detector's bbox *centre* collapses from
1-2.6 px of error to 17-32 px, because that is where a cone crosses ~9 px at the
network's ``imgsz:640`` input. Stereo cannot rescue that -- it fixes depth, not
bearing -- so past the bound the cone is dropped.

The priority order is the design
--------------------------------
LiDAR position always beats a vision position::

    cluster + detection   -> coloured cone, LiDAR position   (the good case)
    cluster alone         -> unknown_color cone              (still published)
    detection + raw LiDAR -> coloured cone, LiDAR position    (sparse: the cone
                             was there but too thin to cluster)
    detection alone       -> monocular, inside the bound, cross-checked by ZNCC

Provenance is published because ``/perception/cones`` cannot carry it, and error measured
against ground truth is meaningless if it cannot be attributed to the thing that
produced it.
"""

import math
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from hyu_msgs.msg import (
    BoundingBoxes,
    CarState,
    ConeArrayWithCovariance,
    ConeWithCovariance,
)

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover - exercised only on older distros
    from hyu_perception import point_cloud2_compat as point_cloud2

import tf2_ros

from hyu_perception.fusion_core import (
    bearing_aligned_covariance,
    camera_point_from_depth,
    monocular_depth_from_bbox,
    monocular_relative_depth_sigma,
    remove_ground_polar_grid,
    remove_ground_ransac,
    stereo_relative_depth_sigma,
    zncc_disparity,
)
from hyu_perception.ros_image_utils import image_message_to_numpy

CONE_COLORS = ("blue", "yellow", "orange", "big_orange", "unknown")

# What produced a cone. Published on the provenance stream; also the key for the
# covariance model, since only the vision provenances carry an error ellipse.
PROV_CLUSTER = "cluster_camera"      # LiDAR cluster + a detection's colour
PROV_CLUSTER_ONLY = "cluster_only"   # LiDAR cluster the camera never confirmed
PROV_SPARSE = "sparse"               # raw LiDAR inside a box, too thin to cluster
# Never published. The bbox-height curve's estimate exists only to centre the
# ZNCC disparity search; a cone carrying this provenance is dropped if stereo
# cannot measure it. See _stereo_cone.
PROV_MONOCULAR = "monocular"         # bbox height only -- INTERNAL CANDIDATE
PROV_MONO_ZNCC = "monocular_zncc"    # bbox bearing, stereo disparity depth


@dataclass(frozen=True)
class Detection:
    color: str
    probability: float
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    @property
    def height_px(self) -> float:
        return self.ymax - self.ymin

    @property
    def width_px(self) -> float:
        return self.xmax - self.xmin

    @property
    def center(self) -> Tuple[float, float]:
        return (0.5 * (self.xmin + self.xmax), 0.5 * (self.ymin + self.ymax))


@dataclass
class ClusterFrame:
    """Everything the bbox callback needs from one LiDAR cloud.

    Points are kept in the **LiDAR frame**, not base: the bbox callback carries
    them to its own stamp, and doing that from the original frame is exact,
    where doing it from an already-transformed copy is a correction on top of a
    correction. ``cluster_indices`` indexes into ``points_lidar`` so the raw
    points, the clusters and the sparse leftovers all share one array.
    """

    stamp: object
    frame_id: str
    points_lidar: np.ndarray = field(
        default_factory=lambda: np.empty((0, 3), dtype=np.float64))
    cluster_indices: List[np.ndarray] = field(default_factory=list)


@dataclass
class Scene:
    """The cloud, resolved into this bbox frame's instant."""

    points_base: np.ndarray
    pixels: np.ndarray           # (N, 2); NaN where the point is behind the camera
    cluster_indices: List[np.ndarray]


@dataclass
class Cone:
    x: float
    y: float
    color: str
    provenance: str
    range_m: float
    # Set only by the vision provenances. LiDAR measures range directly and its
    # error is near-isotropic at cone ranges, so it uses a measured constant.
    relative_depth_sigma: Optional[float] = None
    sigma_u_px: Optional[float] = None
    fx_px: Optional[float] = None


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        self._lock = threading.Lock()
        self._cluster_frame: Optional[ClusterFrame] = None
        self._bbox_msg: Optional[BoundingBoxes] = None
        self._camera_info: Optional[CameraInfo] = None
        # Stereo halves waiting for their partner, keyed by exact stamp, plus
        # the newest complete pair. See _offer_half for why "newest of each"
        # is not the same thing as "a pair".
        self._pending_left: "OrderedDict[int, Image]" = OrderedDict()
        self._pending_right: "OrderedDict[int, Image]" = OrderedDict()
        self._pair: Optional[Tuple[Image, Image]] = None
        # Recent complete pairs keyed by stamp. The ZNCC depth belongs to the
        # PAIR's instant; measuring on the newest pair but attributing it to
        # the bbox's instant smears v * gap (up to 1.3 m at 13 m/s) into the
        # cone. The detector ran on a frame of this same stream, so an
        # exact-stamp pair for the bbox usually exists — select it.
        self._pairs: dict = {}
        self._pair_buffer_size = 8
        self._latencies: List[float] = []
        self._stage_ms: Dict[str, List[float]] = {}
        # Cleared every cloud; see _to_gray for why id() is the key.
        self._gray_cache: Dict[int, np.ndarray] = {}
        self._last_latency_log_ns: Optional[int] = None
        self._warned: Dict[str, float] = {}
        self._zncc_stats = [0, 0]            # [attempted, confirmed]

        self.tf_buffer = tf2_ros.Buffer()
        # NOT spin_thread=True. On Humble that flag does the opposite of what
        # its name promises: the "dedicated" thread runs
        #     executor.add_node(self.node)   <- the WHOLE node, not just /tf
        # so a second SingleThreadedExecutor takes ownership of every callback
        # this node has, main()'s rclpy.spin() then finds the node already
        # owned and spins nothing, and cloud processing ends up sharing ONE
        # thread with the /tf subscription again. Measured 2026-08-01: the
        # cloud callback saturated that thread (272% CPU across the process)
        # and perception's TF buffer FROZE -- 'latest data' stuck at one
        # timestamp for minutes while /tf itself ran at 49 Hz and an
        # independent listener read map->base_footprint 0.04 s fresh. Static
        # transforms kept working (latched on /tf_static, received once), so
        # ROI and ground removal looked healthy while every projection into
        # the image failed on an extrapolation error and the whole run
        # published uncoloured cones.
        #
        # The listener puts its own subscriptions in a ReentrantCallbackGroup;
        # main() spins a MultiThreadedExecutor so that group runs on a
        # different thread from the cone callback, which is what keeps the
        # lookup timeouts below from deadlocking.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST,
                         depth=self.sync_queue_size,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.cones_pub = self.create_publisher(
            ConeArrayWithCovariance, self.output_cones_topic, 10)
        self.cones_viz_pub = self.create_publisher(
            MarkerArray, self.output_cones_topic.rstrip("/") + "/viz", 10)
        self.provenance_pub = self.create_publisher(
            MarkerArray, self.debug_prefix + "/cone_provenance", 10)
        # Segmented LiDAR cloud for RViz: what the backbone kept (ROI and
        # non-ground, intensity preserved -> colour by intensity) and what it
        # removed (ground / outside the ROI / on the car -> draw grey). Built
        # only while something is subscribed.
        self.cloud_kept_pub = self.cloud_removed_pub = None
        if self.publish_segmented_cloud:
            cloud_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                   reliability=ReliabilityPolicy.BEST_EFFORT)
            self.cloud_kept_pub = self.create_publisher(
                PointCloud2, self.debug_prefix + "/cloud_kept", cloud_qos)
            self.cloud_removed_pub = self.create_publisher(
                PointCloud2, self.debug_prefix + "/cloud_removed", cloud_qos)

        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback, qos)
        self.create_subscription(
            BoundingBoxes, self.bbox_topic, self._bbox_callback, qos)
        if self.zncc_enabled:
            self.create_subscription(
                Image, self.left_image_topic, self._left_image_callback, qos)
            self.create_subscription(
                Image, self.right_image_topic, self._right_image_callback, qos)
        # Ego twist for de-skew and the speed-proportional covariance term.
        # Two sources, freshest-preferred: the primary (wheel odometry) when
        # it is flowing, else the fallback (the SBG bridge's fused odometry,
        # which carries a body twist through every one of its rungs). The car
        # has no wheel encoder wired today, so without the fallback the
        # primary is silent and every cloud goes out skewed.
        self._twist = None
        self._twist_primary = None
        self._twist_fallback = None
        self._twist_source = None
        self._deskew_time_mode = None
        if self.deskew_enabled:
            self.create_subscription(
                CarState, self.deskew_twist_topic,
                lambda msg: self._twist_callback(msg, primary=True), qos)
            if (self.deskew_twist_fallback_topic
                    and self.deskew_twist_fallback_topic != self.deskew_twist_topic):
                self.create_subscription(
                    CarState, self.deskew_twist_fallback_topic,
                    lambda msg: self._twist_callback(msg, primary=False), qos)
        # Last: the output path, so the first cloud can find a bbox to colour by.
        self.create_subscription(
            PointCloud2, self.pointcloud_topic, self._cloud_callback, qos)

        self.get_logger().info(
            f"Perception: LiDAR backbone {self.pointcloud_topic}, colour and "
            f"vision fallback {self.bbox_topic}, vision depth by ZNCC stereo "
            f"{'on' if self.zncc_enabled else 'OFF -- no vision cones at all'} "
            f"(monocular depth is never published); de-skew twist "
            f"{self.deskew_twist_topic}"
            + (f" -> {self.deskew_twist_fallback_topic} when stale"
               if self.deskew_twist_fallback_topic else "")
            + f"; publishing {self.output_cones_topic} on every LiDAR cloud")

    # ---------------------------------------------------------------- params

    def _declare_parameters(self) -> None:
        self.declare_parameter("pointcloud_topic", "/sensors/lidar/points")
        self.declare_parameter("bbox_topic", "/perception/bounding_boxes")
        self.declare_parameter("camera_info_topic", "/sensors/zed/left/color/rect/camera_info")
        self.declare_parameter("left_image_topic", "/sensors/zed/left/color/rect/image")
        self.declare_parameter("right_image_topic", "/sensors/zed/right/color/rect/image")
        self.declare_parameter("output_cones_topic", "/perception/cones")
        self.declare_parameter("output_frame", "base_footprint")
        self.declare_parameter("camera_frame", "zed_left_camera_optical_frame")
        self.declare_parameter("debug_prefix", "/perception/debug")
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("latency_log_period_sec", 5.0)

        self.declare_parameter("max_cluster_age_sec", 0.20)
        self.declare_parameter("max_image_age_sec", 0.10)
        self.declare_parameter("max_pair_skew_sec", 0.005)
        self.declare_parameter("motion_compensation_frame", "map")
        self.declare_parameter("tf_timeout_sec", 0.0)
        # Largest cloud->image gap projected WITHOUT motion compensation
        # instead of publishing the frame uncoloured. DEFAULT 0 = never.
        # Tried at 0.03 s and measured worse: that is the mis-association
        # boundary this file's own projection docstring identifies (33 ms =
        # 0.21 m = ~20 px against a ~30 px cone at 15 m), and a MIS-coloured
        # cone is worse than an uncoloured one because it breaks the
        # blue/yellow structure association depends on. Map baseline over three
        # runs each: 0.196/0.297/0.528 m with this off, 0.460/0.741/0.753 m
        # with it at 0.03. It also buys nothing — the tf_timeout fix alone
        # restores colour just as well (unknown 21-24 % either way). Kept as a
        # knob only so the tested-and-rejected shortcut stays on the record.
        self.declare_parameter("max_uncompensated_gap_sec", 0.0)

        self.declare_parameter("projection_model", "eufs_bbox")
        self.declare_parameter("bbox_coordinates", "PIXELS")
        self.declare_parameter("min_bbox_probability", 0.0)
        self.declare_parameter("min_project_depth", 0.1)

        # --- LiDAR backbone -------------------------------------------------
        # De-skew: the realism bridge expresses every LiDAR point at its own
        # measurement time (up to 100 ms before the end-stamp) and publishes
        # the offset in the 'time' field. Undoing it needs the ego twist —
        # from odometry, never ground truth, because the real car has no GT.
        # Uncorrected, cones bias along the travel direction by v*|tau|
        # (~0.3 m at 6 m/s ahead, 2x that to the left under cw spin).
        # RViz: the deskewed cloud split into kept (ROI + non-ground, with
        # intensity) and removed (ground / ROI / self-mask) point clouds on
        # <debug_prefix>/cloud_kept and /cloud_removed, in output_frame.
        self.declare_parameter("publish_segmented_cloud", True)
        self.declare_parameter("deskew_enabled", True)
        self.declare_parameter(
            "deskew_twist_topic", "/localization/wheel_odom")
        # Second twist source, used whenever the primary has not been heard
        # from within deskew_twist_timeout. "" disables it.
        self.declare_parameter(
            "deskew_twist_fallback_topic", "/localization/ins_odom")
        self.declare_parameter("deskew_twist_timeout", 0.5)
        # Per-point time for the de-skew on RoboSense clouds (the sim's 'time'
        # field is exact by construction and is always used as is):
        #   timestamp  the driver's per-point host-clock stamp, as is;
        #   azimuth    from the sweep geometry: the unit spins at a very
        #              uniform 600 rpm, so tau = -(degrees still to sweep
        #              until the frame seam) / 360 * period, with the seam and
        #              the period read off the frame's own stamps. Immune to
        #              the host-stamp jitter measured 2026-08-01 (2-3 ms rms,
        #              occasional ~30 ms UDP-batching steps = 8 cm at 2.7 m/s);
        #   auto       azimuth whenever the sweep model fits the stamps
        #              (residual < 15 ms rms, period 50-200 ms), else stamps.
        self.declare_parameter("deskew_time_source", "auto")
        self.declare_parameter("roi_min_x", 0.5)
        self.declare_parameter("roi_max_x", 30.0)
        self.declare_parameter("roi_abs_y", 15.0)
        self.declare_parameter("roi_min_z", -0.2)
        self.declare_parameter("roi_max_z", 1.5)
        self.declare_parameter("ground_min_z", 0.05)
        # "polar_grid" (local floor per polar cell, default) or "ransac"
        # (one global plane, the pre-2026-08-17 method); see perception.yaml.
        self.declare_parameter("ground_method", "polar_grid")
        self.declare_parameter("ground_cell_range_m", 1.0)
        self.declare_parameter("ground_cell_angle_deg", 5.0)
        self.declare_parameter("ground_height_margin_m", 0.05)
        self.declare_parameter("ground_height_slope_per_m", 0.004)
        self.declare_parameter("ground_ransac_enabled", True)
        self.declare_parameter("ground_ransac_distance_threshold", 0.03)
        self.declare_parameter("ground_ransac_max_iterations", 200)
        self.declare_parameter("ground_ransac_max_tilt_degrees", 20.0)
        self.declare_parameter("ground_ransac_min_inliers", 20)
        self.declare_parameter("ground_ransac_seed", 7)
        self.declare_parameter("self_mask_enabled", True)
        self.declare_parameter("self_mask_min_x", -1.5)
        self.declare_parameter("self_mask_max_x", 1.8)
        self.declare_parameter("self_mask_abs_y", 0.8)
        self.declare_parameter("self_mask_min_z", -0.4)
        self.declare_parameter("self_mask_max_z", 1.4)
        self.declare_parameter("cluster_eps", 0.35)
        self.declare_parameter("cluster_min_points", 3)
        self.declare_parameter("cluster_min_height", 0.02)
        self.declare_parameter("cluster_max_height", 0.80)
        self.declare_parameter("cluster_max_width", 0.90)
        # Gate-pair splitter (see _split_pair_cluster): big-orange gate pairs
        # stand 0.4 m apart and chain into one DBSCAN cluster at eps 0.35.
        self.declare_parameter("pair_split_enabled", True)
        self.declare_parameter("pair_split_min_width", 0.45)
        self.declare_parameter("pair_split_min_separation", 0.30)

        # --- colouring ------------------------------------------------------
        self.declare_parameter("bbox_match_margin_px", 4.0)
        self.declare_parameter("bbox_match_margin_ratio", 0.10)
        self.declare_parameter("min_cluster_support_px", 1)

        # --- sparse: LiDAR that never clustered -----------------------------
        # A far cone gets 1-2 LiDAR returns: real support, too thin for DBSCAN.
        # This was measured as the MOST accurate source in the stack (0.59 %
        # range error), so it is worth recovering rather than dropping to
        # vision. The checks below are what stop it from fusing a fence post.
        self.declare_parameter("sparse_enabled", True)
        self.declare_parameter("sparse_near_range_m", 8.0)
        self.declare_parameter("sparse_far_range_m", 15.0)
        self.declare_parameter("sparse_max_range_m", 0.0)
        self.declare_parameter("sparse_near_min_points", 3)
        self.declare_parameter("sparse_mid_min_points", 2)
        self.declare_parameter("sparse_far_min_points", 2)
        self.declare_parameter("sparse_max_width_m", 0.90)
        self.declare_parameter("sparse_max_depth_span_m", 1.0)
        self.declare_parameter("sparse_max_depth_span_ratio", 0.35)
        self.declare_parameter("sparse_far_min_probability", 0.5)
        self.declare_parameter("sparse_far_min_bbox_width_px", 4.0)
        self.declare_parameter("sparse_far_min_bbox_height_px", 4.0)

        # --- monocular ------------------------------------------------------
        self.declare_parameter("monocular_enabled", True)
        self.declare_parameter("monocular_min_bbox_height_px", 16.0)
        self.declare_parameter("monocular_min_depth_m", 0.5)
        self.declare_parameter("monocular_max_depth_m", 30.0)
        self.declare_parameter("monocular_depth_coefficient", 0.5575)
        self.declare_parameter("monocular_depth_exponent", -0.7555)
        self.declare_parameter("standard_cone_height_m", 0.450)
        self.declare_parameter("big_cone_height_m", 0.5255)
        self.declare_parameter("vision_dedup_radius_m", 0.5)

        # --- ZNCC stereo cross-check ----------------------------------------
        # Replaces per-crop SIFT, which cost 150-300 ms and 200 % CPU to solve a
        # problem that does not exist: on a rectified pair the scale differs by
        # ~1 %, the rotation is zero, and the match is on the same row.
        self.declare_parameter("zncc_enabled", True)
        self.declare_parameter("stereo_baseline_m", 0.12)
        self.declare_parameter("zncc_min_score", 0.5)
        # Window around the prior. Asymmetric because the detector's box runs
        # short past 6 m, so the true disparity sits ABOVE a box-derived prior.
        self.declare_parameter("zncc_search_low_ratio", 0.70)
        self.declare_parameter("zncc_search_high_ratio", 1.45)
        # zncc_replaces_depth and zncc_reject_unconfirmed are GONE. Both existed
        # to choose between publishing a stereo depth and publishing the
        # monocular curve's depth, and the curve's depth is no longer publishable
        # at all: it sizes the disparity search and is discarded. Measured
        # confirmation rate when the choice was still live: 95.9 % (5017/5231).
        self.declare_parameter("sigma_d_px", 0.25)

        # --- covariance ------------------------------------------------------
        self.declare_parameter("lidar_variance_x", 0.04)
        self.declare_parameter("lidar_variance_y", 0.04)
        self.declare_parameter("lidar_only_variance_x", 0.20)
        self.declare_parameter("lidar_only_variance_y", 0.20)
        # Bearing-aligned, not isotropic. See _sparse_covariance.
        self.declare_parameter("cluster_range_bias_m", 0.0)
        self.declare_parameter("sparse_sigma_lat_m", 0.10)
        self.declare_parameter("sparse_sigma_lon_m", 0.25)
        self.declare_parameter("range_variance_scale", 0.0005)
        # Speed-proportional timing sigma. The dominant residual on a LiDAR
        # cluster is time alignment through the (now honestly drifting)
        # odometry TF, not sensor geometry — so it grows with speed while
        # the fixed variance above (calibrated at ~6 m/s) does not. sigma =
        # this * speed; at 0.02 s and 13 m/s that is ~0.26 m, added in
        # quadrature to keep the covariance honest where use_cone_covariance
        # gating needs it.
        self.declare_parameter("lidar_timing_sigma_per_mps", 0.02)
        self.declare_parameter("min_variance", 1.0e-4)
        self.declare_parameter("monocular_sigma_u_px", 10.0)
        self.declare_parameter("sigma_h_px", 4.0)
        # Range-independent floor on a vision cone's LATERAL sigma. See
        # _vision_covariance: the measured error has no range trend and the
        # pixel term does, so the pixel term alone is over-confident near.
        self.declare_parameter("vision_lateral_floor_m", 0.25)

    def _load_parameters(self) -> None:
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        for name in (
            "pointcloud_topic", "bbox_topic", "camera_info_topic",
            "deskew_twist_topic", "deskew_twist_fallback_topic", "ground_method",
            "deskew_time_source",
            "left_image_topic", "right_image_topic", "output_cones_topic",
            "output_frame", "camera_frame",
        ):
            setattr(self, name, str(g(name)))
        self.deskew_time_source = self.deskew_time_source.strip().lower()
        if self.deskew_time_source not in ("auto", "timestamp", "azimuth"):
            raise ValueError(
                "deskew_time_source must be auto, timestamp or azimuth "
                f"(got {self.deskew_time_source!r})")
        self.debug_prefix = str(g("debug_prefix")).rstrip("/")
        self.motion_compensation_frame = str(g("motion_compensation_frame")).strip()
        self.max_uncompensated_gap_sec = float(g("max_uncompensated_gap_sec"))
        self.projection_model = str(g("projection_model"))
        self.bbox_coordinates = str(g("bbox_coordinates")).upper()
        for name in ("deskew_enabled", "publish_segmented_cloud",
                     "publish_debug", "ground_ransac_enabled", "self_mask_enabled",
                     "pair_split_enabled",
                     "sparse_enabled", "monocular_enabled", "zncc_enabled"):
            setattr(self, name, bool(g(name)))
        for name in ("sync_queue_size", "ground_ransac_max_iterations",
                     "ground_ransac_min_inliers", "ground_ransac_seed",
                     "cluster_min_points", "min_cluster_support_px",
                     "sparse_near_min_points", "sparse_mid_min_points",
                     "sparse_far_min_points"):
            setattr(self, name, int(g(name)))
        for name in (
            "latency_log_period_sec", "max_cluster_age_sec", "max_image_age_sec",
            "max_pair_skew_sec",
            "tf_timeout_sec", "min_bbox_probability", "min_project_depth",
            "roi_min_x", "roi_max_x", "roi_abs_y", "roi_min_z", "roi_max_z",
            "ground_min_z", "ground_ransac_distance_threshold",
            "ground_cell_range_m", "ground_cell_angle_deg",
            "ground_height_margin_m", "ground_height_slope_per_m",
            "ground_ransac_max_tilt_degrees", "self_mask_min_x",
            "self_mask_max_x", "self_mask_abs_y", "self_mask_min_z",
            "self_mask_max_z", "cluster_eps", "cluster_min_height",
            "cluster_max_height", "cluster_max_width", "pair_split_min_width",
            "pair_split_min_separation", "bbox_match_margin_px",
            "bbox_match_margin_ratio", "sparse_max_range_m",
            "sparse_near_range_m",
            "sparse_far_range_m", "sparse_max_width_m",
            "sparse_max_depth_span_m", "sparse_max_depth_span_ratio",
            "sparse_far_min_probability", "sparse_far_min_bbox_width_px",
            "sparse_far_min_bbox_height_px", "monocular_min_bbox_height_px",
            "deskew_twist_timeout",
            "monocular_min_depth_m", "monocular_max_depth_m",
            "monocular_depth_coefficient", "monocular_depth_exponent",
            "standard_cone_height_m", "big_cone_height_m",
            "vision_dedup_radius_m", "stereo_baseline_m", "zncc_min_score",
            "zncc_search_low_ratio", "zncc_search_high_ratio", "sigma_d_px",
            "lidar_variance_x", "lidar_variance_y", "lidar_only_variance_x",
            "lidar_only_variance_y", "cluster_range_bias_m",
            "sparse_sigma_lat_m", "sparse_sigma_lon_m",
            "range_variance_scale", "min_variance", "monocular_sigma_u_px",
            "sigma_h_px", "vision_lateral_floor_m", "lidar_timing_sigma_per_mps",
        ):
            setattr(self, name, float(g(name)))

    def _validate_parameters(self) -> None:
        if self.projection_model not in ("eufs_bbox", "pinhole"):
            raise ValueError(
                "projection_model must be 'eufs_bbox' or 'pinhole'; they differ "
                "by an axis permutation applied in both directions")
        if self.bbox_coordinates not in ("PIXELS", "PERCENTAGE"):
            raise ValueError("bbox_coordinates must be PIXELS or PERCENTAGE")
        if self.roi_min_x >= self.roi_max_x or self.roi_min_z >= self.roi_max_z:
            raise ValueError("roi bounds must be ordered min < max")
        if not 0.0 <= self.cluster_min_height < self.cluster_max_height:
            raise ValueError("cluster height bounds must satisfy 0 <= min < max")
        if self.cluster_max_width <= 0.0 or self.cluster_eps <= 0.0:
            raise ValueError("cluster_max_width and cluster_eps must be positive")
        if self.cluster_min_points < 1:
            raise ValueError("cluster_min_points must be at least 1")
        if self.monocular_min_bbox_height_px < 0.0:
            raise ValueError("monocular_min_bbox_height_px must not be negative")
        if not 0.0 < self.monocular_min_depth_m < self.monocular_max_depth_m:
            raise ValueError("monocular depth bounds must satisfy 0 < min < max")
        if self.monocular_depth_coefficient <= 0.0:
            raise ValueError("monocular_depth_coefficient must be positive")
        if self.monocular_depth_exponent >= 0.0:
            raise ValueError(
                "monocular_depth_exponent must be negative: depth falls as the "
                "box grows")
        if not 0.0 < self.zncc_search_low_ratio < self.zncc_search_high_ratio:
            raise ValueError(
                "zncc search ratios must satisfy 0 < low < high")
        if self.sparse_near_range_m >= self.sparse_far_range_m:
            raise ValueError("sparse_near_range_m must be below sparse_far_range_m")
        # A zero pixel sigma claims a perfect measurement, which would hand SLAM
        # an infinitely trusted cone. Reject it here rather than let
        # min_variance quietly paper over it once per cone.
        for name in ("monocular_sigma_u_px", "sigma_h_px", "sigma_d_px",
                     "stereo_baseline_m", "lidar_variance_x", "lidar_variance_y",
                     "lidar_only_variance_x", "lidar_only_variance_y",
                     "sparse_sigma_lat_m", "sparse_sigma_lon_m", "min_variance",
                     "standard_cone_height_m", "big_cone_height_m",
                     "max_cluster_age_sec"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.range_variance_scale < 0.0:
            raise ValueError("range_variance_scale must not be negative")

    # ------------------------------------------------------------- callbacks

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        with self._lock:
            self._camera_info = msg

    def _left_image_callback(self, msg: Image) -> None:
        self._offer_half(msg, self._pending_left, self._pending_right, left=True)

    def _right_image_callback(self, msg: Image) -> None:
        self._offer_half(msg, self._pending_right, self._pending_left, left=False)

    def _offer_half(self, msg, mine, theirs, left: bool) -> None:
        """Pair the stereo halves by exact stamp, not by "whatever arrived last".

        Both halves come out of one multicamera sensor, so a true pair is
        stamp-identical and this match is exact rather than a tolerance. Keeping
        the newest of each independently looked equivalent and was not: no single
        Python process can deserialise two 2.7 MB streams at 30 Hz, so the halves
        arrive at different effective rates and the two "newest" images are
        routinely different instants. Triangulating those measures the car's
        motion and reports it as depth.

        Dropping frames is fine here. We only need a pair per LiDAR cloud, and
        pairs survive at well over 10 Hz even when each half loses a third.
        """
        key = self._stamp_key(msg.header.stamp)
        with self._lock:
            match = theirs.pop(key, None)
            if match is not None:
                self._pair = (msg, match) if left else (match, msg)
                self._pairs[key] = self._pair
                while len(self._pairs) > 10:
                    self._pairs.pop(next(iter(self._pairs)))
                theirs.clear()
                mine.clear()
                return
            mine[key] = msg
            # Unmatched halves are frames whose counterpart the process dropped.
            # Bound the buffer; the stale ones will never find a partner.
            while len(mine) > self._pair_buffer_size:
                del mine[next(iter(mine))]

    @staticmethod
    def _stamp_key(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _cloud_callback(self, msg: PointCloud2) -> None:
        """The output path. One cloud in, one /perception/cones frame out.

        The LiDAR is the backbone: it is what actually measures where a cone is,
        so it sets the rate. Running the output off the camera instead meant
        republishing the same clusters two extra times per cloud, dressed in a
        new stamp each time, which invites a consumer to read three independent
        measurements where there was one.
        """
        if not msg.header.frame_id:
            self._warn_throttled(
                "cloud_frame", "LiDAR cloud has an empty frame_id; it cannot be "
                "transformed into the output frame")
            return
        t = self._stage_timer()
        self._gray_cache.clear()
        points, point_times, intensity = self._decode_cloud(msg)
        t("deserialise")
        frame = ClusterFrame(stamp=msg.header.stamp, frame_id=msg.header.frame_id)
        if points.shape[0]:
            # ROI and ground removal want a level ground plane, so they run in
            # the output frame; the points are kept in the LiDAR frame so the
            # bbox callback can compensate them from their true origin.
            lidar_to_base = self._lookup_transform(
                self.output_frame, msg.header.frame_id, msg.header.stamp)
            if lidar_to_base is None:
                self._warn_throttled(
                    "cloud_tf", f"No TF {msg.header.frame_id} -> "
                    f"{self.output_frame} at the cloud stamp; skipping cloud")
                return
            points = self._deskew_points(points, point_times, msg, lidar_to_base)
            t("deskew")
            points_base = self._transform_points(points, lidar_to_base)
            t("tf+transform")
            keep = self._roi_mask(points_base) & self._non_ground_mask(points_base)
            t("roi+ground")
            self._publish_segmented_cloud(msg.header.stamp, points_base, keep, intensity)
            frame.points_lidar = points[keep]
            frame.cluster_indices = self._cluster(points_base[keep])
            t("cluster")
        with self._lock:
            self._cluster_frame = frame
            camera_info = self._camera_info
            bbox_msg = self._bbox_msg
            pair = self._pair
            pairs = dict(self._pairs)
        left, right = pair if pair is not None else (None, None)

        stamp = frame.stamp
        if camera_info is None:
            self._warn_throttled(
                "no_camera_info", f"Waiting for {self.camera_info_topic}; "
                f"cannot project without intrinsics")
            return
        camera_matrix = self._camera_matrix(camera_info)
        if not np.all(np.isfinite(camera_matrix)):
            self._warn_throttled(
                "bad_camera_info",
                f"{self.camera_info_topic} carries no usable K or P")
            return
        if bbox_msg is None:
            self._warn_throttled(
                "no_bbox", f"Waiting for {self.bbox_topic}; publishing "
                f"uncoloured clusters until it arrives")
            detections, image_stamp = [], stamp
        else:
            image_stamp = self._bbox_stamp(bbox_msg)
            detections = self._extract_detections(bbox_msg, camera_info)
            # Prefer the stereo pair captured at the bbox's OWN instant: the
            # disparity then measures that frame's geometry, not the car's
            # motion since it. Falls back to the newest pair (still gated by
            # max_image_age downstream) when the exact pair was dropped.
            exact_pair = pairs.get(self._stamp_key(image_stamp))
            if exact_pair is not None:
                left, right = exact_pair
            # A stale bbox must degrade exactly like a missing one: keep the
            # LiDAR backbone and publish uncoloured. Letting _scene_at
            # return None here used to drop the cluster loop entirely — a
            # YOLO stall silently removed ALL accurate cones for its whole
            # duration instead of just the colours.
            age = abs(self._stamp_sec(image_stamp) - self._stamp_sec(stamp))
            if age > self.max_cluster_age_sec:
                self._warn_throttled(
                    "cluster_age",
                    f"Newest bbox frame is {age:.2f}s from the cloud stamp "
                    f"(limit {self.max_cluster_age_sec}s); publishing "
                    f"uncoloured cones")
                detections, image_stamp = [], stamp
        t("detections")

        scene = self._scene_at(frame, image_stamp, camera_matrix)
        if (scene is None and frame.points_lidar.shape[0]
                and self._stamp_ns(image_stamp) != self._stamp_ns(stamp)):
            # A failed cloud->image-stamp TF must degrade exactly like a
            # stale bbox: keep the LiDAR backbone and drop only the colours.
            # At the cloud's own stamp the projection needs no
            # motion-compensation frame (equal stamps take the plain
            # lookup), so this succeeds whenever the cloud could be placed
            # at all -- which the callback already required. Without this,
            # any frame whose bbox stamp missed the LiDAR tick lost every
            # cluster and published vision-only cones: gate pairs merged
            # 4->2 and per-landmark covariance flapped 27x at 14% of frames
            # (measured 2026-07-20, odom frame absent).
            detections, image_stamp = [], stamp
            scene = self._scene_at(frame, image_stamp, camera_matrix)
        t("scene(tf+project)")
        cones = self._build_cones(detections, scene, camera_info, camera_matrix,
                                  image_stamp, left, right)
        t("build_cones(+zncc)")
        # The vision-only provenances measured their cone in an image from
        # `image_stamp`, so their x/y belong to that instant, not to the cloud's.
        # The array goes out under one stamp; carrying them forward is what makes
        # that stamp true for every cone in it rather than just for the clusters.
        cones = self._carry_vision_cones(cones, image_stamp, stamp)
        # Provenance goes out FIRST. A consumer keys it by the /perception/cones stamp, so
        # publishing it afterwards leaves that consumer holding cones it cannot
        # attribute for one frame -- which reads as "no provenance" rather than
        # as a race.
        if self.publish_debug:
            self.provenance_pub.publish(self._provenance_markers(cones, stamp))
        self._publish(cones, stamp)
        t("publish")
        self._commit_stages(t)

    def _stage_timer(self):
        """Wall-clock stopwatch for the cloud callback's stages.

        Wall, not CPU: the number this has to explain is the 70.8 ms median a
        consumer actually waits, and a CPU profile cannot see a TF lookup or a
        lock blocking. perf_counter is monotonic and ~50 ns, so ~8 calls a frame
        at 10 Hz costs nothing measurable.
        """
        marks = []
        start = [time.perf_counter()]

        def mark(name: str) -> None:
            now = time.perf_counter()
            marks.append((name, (now - start[0]) * 1000.0))
            start[0] = now

        mark.marks = marks
        return mark

    def _commit_stages(self, timer) -> None:
        for name, ms in timer.marks:
            self._stage_ms.setdefault(name, []).append(ms)

    def _log_stages(self) -> None:
        """Where the callback's wall time went, per stage, over the last window."""
        if not self._stage_ms:
            return
        parts, total = [], 0.0
        for name, values in self._stage_ms.items():
            values.sort()
            median = values[len(values) // 2]
            p90 = values[min(len(values) - 1, int(0.9 * len(values)))]
            total += median
            parts.append(f"{name} {median:.1f}/{p90:.1f}")
        n = max(len(v) for v in self._stage_ms.values())
        self.get_logger().info(
            f"/perception/cones stages median/p90 ms over {n} clouds: "
            + "  ".join(parts) + f"   [sum of medians {total:.1f} ms]")
        self._stage_ms.clear()

    def _bbox_callback(self, msg: BoundingBoxes) -> None:
        """Cache the newest detections. The cloud callback does the publishing.

        Colour does not move, so a bbox up to one camera period old still
        describes the same cone. Holding it costs nothing and lets the output
        stay on the LiDAR's clock.
        """
        with self._lock:
            self._bbox_msg = msg

    def _carry_vision_cones(self, cones: List[Cone], from_stamp, to_stamp):
        """Move the vision-only cones from the image's instant to the cloud's.

        Cluster cones are already at ``to_stamp`` by construction: the scene put
        them there. Only the monocular and ZNCC provenances came out of the
        image alone, and they carry its stamp. Returns the cones unchanged if
        the gap is negligible or TF cannot span it, which is honest: a cone at a
        stamp 30 ms stale beats no cone.
        """
        vision = [c for c in cones
                  if c.provenance in (PROV_MONOCULAR, PROV_MONO_ZNCC)]
        if not vision:
            return cones
        gap = abs(self._stamp_sec(to_stamp) - self._stamp_sec(from_stamp))
        if gap < 1e-4:
            return cones
        carry = self._lookup_transform_between(
            self.output_frame, to_stamp, self.output_frame, from_stamp)
        if carry is None:
            self._warn_throttled(
                "vision_carry_tf",
                f"No TF to carry vision-only cones {gap * 1000:.0f} ms to the "
                f"cloud stamp; they go out at the image's instant")
            return cones
        points = np.array([[c.x, c.y, 0.0] for c in vision], dtype=np.float64)
        moved = self._transform_points(points, carry)
        for cone, (x, y, _) in zip(vision, moved):
            cone.x, cone.y = float(x), float(y)
            cone.range_m = float(math.hypot(x, y))
        return cones

    # -------------------------------------------------------------- backbone

    def _pointcloud_to_xyz_time(self, msg: PointCloud2):
        """Decode xyz (+ per-point time offset), vectorised; see _decode_cloud."""
        xyz, times, _ = self._decode_cloud(msg)
        return xyz, times

    def _decode_cloud(self, msg: PointCloud2):
        """Decode (xyz, per-point time offset or None, intensity or None).

        One read_points call over ONE field set, so every returned array is
        aligned (skip_nans drops the same rows from all of them). Two
        per-point time contracts are understood:
          * ``time``       -- the sim realism bridge: offset from the header
                              stamp, in (-period, 0] under end stamping;
          * ``timestamp``  -- the RoboSense driver (rslidar.yaml: host clock,
                              header = LAST point): ABSOLUTE seconds per
                              point, so the offset is timestamp - header.
        Until 2026-08-17 only 'time' was read, so on the real LiDAR the
        de-skew silently never ran (times=None) -- every outing's clouds went
        out skewed. Ideal/legacy clouds have neither and return times=None.
        Vectorised: this used to be a per-point Python loop.
        """
        empty = np.empty((0, 3), dtype=np.float64)
        header_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for fields in (("x", "y", "z", "intensity", "time"),
                       ("x", "y", "z", "intensity", "timestamp"),
                       ("x", "y", "z", "time"),
                       ("x", "y", "z", "timestamp"),
                       ("x", "y", "z", "intensity"),
                       ("x", "y", "z")):
            try:
                structured = point_cloud2.read_points(
                    msg, field_names=fields, skip_nans=True)
            except Exception:
                continue  # a field in this set is absent -> next contract
            array = np.asarray(structured)
            if array.size == 0:
                return empty, None, None
            if not array.dtype.names:  # pragma: no cover - plain xyz array
                return np.asarray(array, dtype=np.float64).reshape(-1, 3), None, None
            xyz = np.column_stack(
                [array["x"], array["y"], array["z"]]).astype(np.float64)
            intensity = (np.asarray(array["intensity"], dtype=np.float32)
                         if "intensity" in fields else None)
            times = None
            if "time" in fields:
                times = np.asarray(array["time"], dtype=np.float64)
            elif "timestamp" in fields:
                times = np.asarray(array["timestamp"], dtype=np.float64) - header_sec
                # Guard the contract: with a lidar-clock/host-clock mismatch
                # the offsets are seconds off, and "correcting" with them
                # would smear the cloud far worse than leaving it skewed.
                if times.size and (np.abs(times).max() > 1.0):
                    self._warn_throttled(
                        "cloud_timestamp",
                        "per-point 'timestamp' is not within 1 s of the header "
                        f"stamp (offsets {times.min():.2f}..{times.max():.2f} s); "
                        "ignoring it for de-skew (check rslidar use_lidar_clock)")
                    times = None
                if times is not None and self.deskew_time_source != "timestamp":
                    times = self._azimuth_times(xyz, times)
            return xyz, times, intensity

    # RoboSense sweep geometry: azimuth grows CLOCKWISE from the lidar +x
    # (x = d cos(az), y = -d sin(az)); a frame runs from the seam
    # (split_angle) once around to the seam, the header is the last point.
    _AZ_MODEL_MIN_POINTS = 200
    _AZ_MODEL_MIN_BINS = 180
    _AZ_MODEL_MAX_RESID_SEC = 0.005
    _AZ_MODEL_PERIOD_SEC = (0.05, 0.20)

    def _azimuth_times(self, xyz: np.ndarray, tau_ts: np.ndarray) -> np.ndarray:
        """Per-point offsets from the sweep geometry instead of the host stamps.

        tau = -(degrees still to sweep until the seam) / 360 * period. Host
        stamps are only ever LATE (UDP batching stamps a burst of packets at
        its end), so the earliest stamp in each 1-degree azimuth bin is a
        clean estimate of that bin's true time. The seam is the bin boundary
        where those bin minima drop by a whole period (the frame wraps
        there), the period the least-squares slope of bin minimum vs
        degrees-to-seam, and the fit residual the model check: a frame that
        does not spin clockwise from +x at a steady rate keeps the stamps.
        Points straddling the seam are snapped to the stamp's revolution.
        Tiny clouds keep the stamps too. Mode changes are logged.
        """
        n = tau_ts.size
        if n < self._AZ_MODEL_MIN_POINTS:
            return tau_ts
        az = np.degrees(np.arctan2(-xyz[:, 1], xyz[:, 0])) % 360.0
        bins = np.minimum(az.astype(np.int64), 359)
        bin_min = np.full(360, np.inf)
        np.minimum.at(bin_min, bins, tau_ts)
        filled = np.flatnonzero(np.isfinite(bin_min))
        fits = False
        refined = tau_ts
        if filled.size >= self._AZ_MODEL_MIN_BINS:
            vals = bin_min[filled]
            # cyclic differences between consecutive filled bins; the frame
            # wraps where the stamp drops by ~period
            drop = np.roll(vals, -1) - vals
            j = int(np.argmin(drop))
            seam = float(filled[(j + 1) % filled.size])  # start angle of the first bin after the seam
            u_bins = (seam - (filled + 0.5)) % 360.0     # bin centres, degrees to seam
            um, tm = u_bins.mean(), vals.mean()
            du = u_bins - um
            var = float(np.dot(du, du))
            if var > 0.0:
                slope = float(np.dot(du, vals - tm)) / var  # s/deg, < 0 for cw
                period = -slope * 360.0
                resid = vals - (slope * du + tm)
                rms = float(np.sqrt(np.mean(resid * resid)))
                lo, hi = self._AZ_MODEL_PERIOD_SEC
                if (lo <= period <= hi and rms <= self._AZ_MODEL_MAX_RESID_SEC
                        and -drop[j] > 0.5 * period):
                    u = (seam - az) % 360.0
                    refined = -u / 360.0 * period
                    # The driver splits frames on the BLOCK azimuth while a
                    # point's geometric azimuth (channel offset, laser origin
                    # offset) can fall a fraction of a degree on the other
                    # side of the seam: the model would then put it a whole
                    # period off. The stamps, late by at most tens of ms, say
                    # which revolution cycle a point belongs to -- snap to it.
                    refined -= np.rint((refined - tau_ts) / period) * period
                    np.clip(refined, -period, 0.0, out=refined)
                    fits = True
        mode = "azimuth" if fits else "timestamp"
        if mode != self._deskew_time_mode:
            self._deskew_time_mode = mode
            if fits:
                self.get_logger().info(
                    "de-skew point times: sweep model (clockwise from +x, "
                    f"period {period * 1e3:.1f} ms, seam {seam:.0f} deg, stamp "
                    f"residual {rms * 1e3:.2f} ms rms)")
            else:
                level = (self.get_logger().warning
                         if self.deskew_time_source == "azimuth"
                         else self.get_logger().info)
                level("de-skew point times: driver stamps (sweep model "
                      "does not fit this cloud)")
        return refined

    _SEG_FIELDS = None  # built lazily: [x, y, z, intensity] float32

    def _publish_segmented_cloud(self, stamp, points_base, keep, intensity) -> None:
        """Publish the kept / removed halves of the (deskewed) cloud in
        output_frame. Cheap when nobody listens; ~1 ms per cloud otherwise
        (numpy pack, no per-point Python)."""
        pubs = (self.cloud_kept_pub, self.cloud_removed_pub)
        if pubs[0] is None:
            return
        wanted = [pub.get_subscription_count() > 0 for pub in pubs]
        if not any(wanted):
            return
        if PerceptionNode._SEG_FIELDS is None:
            from sensor_msgs.msg import PointField
            PerceptionNode._SEG_FIELDS = [
                PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(("x", "y", "z", "intensity"))]
        n = points_base.shape[0]
        inten = (intensity if intensity is not None and intensity.shape[0] == n
                 else np.zeros(n, dtype=np.float32))
        packed = np.empty((n, 4), dtype=np.float32)
        packed[:, :3] = points_base
        packed[:, 3] = inten
        for pub, want, mask in zip(pubs, wanted, (keep, ~keep)):
            if not want:
                continue
            sel = packed[mask]
            cloud = PointCloud2()
            cloud.header.stamp = stamp
            cloud.header.frame_id = self.output_frame
            cloud.height = 1
            cloud.width = int(sel.shape[0])
            cloud.fields = PerceptionNode._SEG_FIELDS
            cloud.is_bigendian = False
            cloud.point_step = 16
            cloud.row_step = 16 * cloud.width
            cloud.is_dense = True
            cloud.data = sel.tobytes()
            pub.publish(cloud)

    def _twist_callback(self, msg: CarState, primary: bool = True) -> None:
        tw = msg.twist.twist
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        twist = (tw.linear.x, tw.linear.y, tw.angular.z, stamp)
        if primary:
            self._twist_primary = twist
        else:
            self._twist_fallback = twist
            # The fallback only speaks while the primary is silent: a primary
            # sample within the timeout of THIS stamp keeps precedence.
            prim = self._twist_primary
            if prim is not None and abs(stamp - prim[3]) <= self.deskew_twist_timeout:
                return
        self._twist = twist
        source = "primary" if primary else "fallback"
        if source != self._twist_source:
            self._twist_source = source
            topic = (self.deskew_twist_topic if primary
                     else self.deskew_twist_fallback_topic)
            self.get_logger().info(
                f"de-skew twist source: {source} ({topic})")

    def _deskew_points(
        self,
        points: np.ndarray,
        times,
        msg: PointCloud2,
        lidar_to_base,
    ) -> np.ndarray:
        """Undo the LiDAR's motion distortion, point by point.

        Each point was measured at its own instant tau in (-period, 0] before
        the header stamp, from a sensor that was then at a different pose:
        p' = R(-w*tau) @ (p - v_sensor*tau) under a constant body twist. The
        inverse, p = R(w*tau) @ p' + v_sensor*tau, restores the whole cloud to
        the stamp instant -- the contract every downstream stage (single
        transform to base, bbox pairing, projection) assumes, and the one the
        sim bridge's contract test pins (a correct deskew recovers the ideal
        cloud).

        The twist is a BASE quantity (body x forward, y left, yaw rate about
        base z) and the sensor may sit at any yaw/pitch/roll -- the 2026-08
        nose unit was mounted at yaw -83 deg, and applying the base-oriented
        velocity straight onto lidar-frame coordinates (the old "sensor yaw
        ~ 0" shortcut) pushed the points sideways: 20-40 cm of residual at
        2.7 m/s, worse than no deskew when driving straight. So: rotate the
        cloud into base ORIENTATION (rotation only), deskew there with the
        sensor velocity v_sensor = v + w x r (r = mount lever arm), rotate
        back. The caller's lidar->base transform stays the one place that
        moves the cloud into the output frame. The twist comes from odometry,
        never ground truth: this code path is identical on the real car.
        """
        if not self.deskew_enabled or times is None or points.shape[0] == 0:
            return points
        twist = self._twist
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if twist is None or abs(stamp - twist[3]) > self.deskew_twist_timeout:
            self._warn_throttled(
                "deskew_twist",
                "no fresh odometry twist for de-skew; publishing skewed "
                "cloud (cone bias grows with speed)")
            return points
        vx, vy, wz = twist[0], twist[1], twist[2]
        if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3:
            return points
        # lidar_to_base: 4x4 sensor pose in the output frame. Its translation
        # is the mount lever arm, its rotation the mount attitude.
        rot = np.asarray(lidar_to_base[:3, :3], dtype=np.float64)
        mount_x = float(lidar_to_base[0, 3])
        mount_y = float(lidar_to_base[1, 3])
        v_sx = vx - wz * mount_y
        v_sy = vy + wz * mount_x
        q = points @ rot.T  # lidar frame -> base orientation (no translation)
        theta = wz * times
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        qx, qy = q[:, 0], q[:, 1]
        out = q.copy()
        out[:, 0] = cos_t * qx - sin_t * qy + v_sx * times
        out[:, 1] = sin_t * qx + cos_t * qy + v_sy * times
        return out @ rot  # back to the lidar frame

    def _roi_mask(self, points_base: np.ndarray) -> np.ndarray:
        x, y, z = points_base[:, 0], points_base[:, 1], points_base[:, 2]
        mask = (
            np.isfinite(points_base).all(axis=1)
            & (x >= self.roi_min_x) & (x <= self.roi_max_x)
            & (np.abs(y) <= self.roi_abs_y)
            & (z >= self.roi_min_z) & (z <= self.roi_max_z)
        )
        if self.self_mask_enabled:
            on_self = (
                (x >= self.self_mask_min_x) & (x <= self.self_mask_max_x)
                & (np.abs(y) <= self.self_mask_abs_y)
                & (z >= self.self_mask_min_z) & (z <= self.self_mask_max_z)
            )
            mask &= ~on_self
        return mask

    def _non_ground_mask(self, points_base: np.ndarray) -> np.ndarray:
        """Drop the ground, failing SAFE to a flat cut.

        Default is the local polar-grid floor (remove_ground_polar_grid):
        real ground is not one plane and a global RANSAC plane kept 12-48 %
        ground in the 2026-08-01 clouds. ``ground_method: ransac`` restores
        the single-plane method. A missed plane must not delete the cones,
        so an unusable RANSAC result falls back to a height cut rather than
        rejecting everything.
        """
        if points_base.shape[0] == 0:
            return points_base[:, 2] >= self.ground_min_z
        if self.ground_method == "polar_grid":
            return remove_ground_polar_grid(
                points_base,
                cell_range_m=self.ground_cell_range_m,
                cell_angle_deg=self.ground_cell_angle_deg,
                height_margin_m=self.ground_height_margin_m,
                height_slope_per_m=self.ground_height_slope_per_m,
                max_range_m=max(self.roi_max_x, self.roi_abs_y) * 1.5,
            )
        if not self.ground_ransac_enabled:
            return points_base[:, 2] >= self.ground_min_z
        result = remove_ground_ransac(
            points_base,
            distance_threshold=self.ground_ransac_distance_threshold,
            max_iterations=self.ground_ransac_max_iterations,
            max_tilt_degrees=self.ground_ransac_max_tilt_degrees,
            min_inliers=self.ground_ransac_min_inliers,
            seed=self.ground_ransac_seed,
        )
        if result.plane is None:
            self._warn_throttled(
                "ground_plane", "No near-horizontal ground plane found; falling "
                f"back to a flat z >= {self.ground_min_z} cut")
            return points_base[:, 2] >= self.ground_min_z
        return result.non_ground_mask

    def _cluster(self, points_base: np.ndarray) -> List[np.ndarray]:
        """DBSCAN in the ground plane, then a cone-shaped-blob filter."""
        labels = self._dbscan_xy(points_base[:, :2])
        clusters = []
        for label in range(int(labels.max()) + 1 if labels.size else 0):
            indices = np.flatnonzero(labels == label)
            if indices.size < self.cluster_min_points:
                continue
            member = points_base[indices]
            height = float(member[:, 2].max() - member[:, 2].min())
            if not self.cluster_min_height <= height <= self.cluster_max_height:
                continue
            span = member[:, :2].max(axis=0) - member[:, :2].min(axis=0)
            if float(np.hypot(*span)) > self.cluster_max_width:
                continue
            clusters.extend(self._split_pair_cluster(points_base, indices))
        return clusters

    def _split_pair_cluster(
        self, points_base: np.ndarray, indices: np.ndarray
    ) -> List[np.ndarray]:
        """Split a cluster that is really TWO cones standing in a pair.

        The start/finish gate places big cones 0.4 m apart per side; with a
        ~0.28 m base that leaves ~0.1 m of clear air — under cluster_eps
        (0.35), so DBSCAN chains BOTH cones into one cluster that still passes
        cluster_max_width (0.90). Downstream that lopsided single observation
        collapsed a gate pair into one SLAM landmark and biased every
        gate-anchor correction (autocross_kase2026 ghost-map, 2026-07-19).

        Only clusters wider than one cone (pair_split_min_width) are touched:
        split at the midpoint of the principal axis, accept the split only if
        BOTH halves look like individual cones (enough points, each narrower
        than pair_split_min_width, centers at least pair_split_min_separation
        apart). Anything else returns the original cluster unchanged, so
        ordinary cones keep the exact partition the perception covariances
        were fitted on.
        """
        if not self.pair_split_enabled:
            return [indices]
        member_xy = points_base[indices, :2]
        span = member_xy.max(axis=0) - member_xy.min(axis=0)
        if float(np.hypot(*span)) <= self.pair_split_min_width:
            return [indices]
        centered = member_xy - member_xy.mean(axis=0)
        # Principal axis via 2x2 SVD: the pair's separation direction.
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        projection = centered @ vt[0]
        left = indices[projection < 0.0]
        right = indices[projection >= 0.0]
        if left.size < self.cluster_min_points or right.size < self.cluster_min_points:
            return [indices]
        halves = []
        centers = []
        for half in (left, right):
            half_xy = points_base[half, :2]
            half_span = half_xy.max(axis=0) - half_xy.min(axis=0)
            if float(np.hypot(*half_span)) > self.pair_split_min_width:
                return [indices]
            halves.append(half)
            centers.append(half_xy.mean(axis=0))
        if float(np.hypot(*(centers[0] - centers[1]))) < self.pair_split_min_separation:
            return [indices]
        return halves

    def _dbscan_xy(self, xy: np.ndarray) -> np.ndarray:
        """DBSCAN in the ground plane, in C rather than in Python.

        This replaces a hand-rolled grid DBSCAN whose docstring said "No sklearn
        dependency". The dependency it avoided cost 113-138 ms of every 220 ms
        cloud -- 55 % of the node -- because the inner test was a numpy call per
        NEIGHBOUR PAIR (`np.sum((xy[other] - xy[index]) ** 2)`), at ~2 us of
        interpreter overhead each, and the BFS re-derived a point's neighbours
        every time it reached it.

        MEASURED against the implementation it replaces, same eps and
        min_samples, on track-shaped scenes, checked for IDENTICAL partitions
        (not merely similar ones -- every covariance constant in perception.yaml
        was fitted against the old clusters, so a different partition would
        silently invalidate them):

            N      old        sklearn   same partition?
            400     13.3 ms    0.9 ms   identical
            960     37.0 ms    1.5 ms   identical
            1460    45.3 ms    2.8 ms   identical
            3800   125.9 ms    6.0 ms   identical      <- this is the real load
            8400   350.4 ms   17.1 ms   identical

        The 3800 row is the one that matters because it is where the bench meets
        the 113-138 ms the node actually measured -- so ~3800 points is INFERRED
        to be what survives ROI and ground removal, not counted. If you need the
        real number, count it; the swap does not rest on it either way.

        Not a GPU: at 3800 points the host-to-device copy alone is ~1 ms and the
        pairwise form is O(N^2). sklearn's kd-tree is 6 ms on one core and the
        machine has 15 idle ones. A GPU would win somewhere past ~100k points,
        which is 6x more than the whole cloud.
        """
        count = xy.shape[0]
        if count == 0:
            return np.full(0, -1, dtype=np.int64)
        # Imported here, not at module scope: sklearn pulls in scipy and takes
        # ~1 s to import, and every OTHER entry point in this package (the curve
        # fitter, the rate measurer, the evaluator) imports this module without
        # ever clustering.
        from sklearn.cluster import DBSCAN
        return DBSCAN(eps=self.cluster_eps,
                      min_samples=self.cluster_min_points).fit(xy).labels_

    def _scene_at(
        self,
        frame: Optional[ClusterFrame],
        image_stamp,
        camera_matrix: np.ndarray,
    ) -> Optional[Scene]:
        """Resolve the cached cloud for this LiDAR frame's output.

        Positions and association want different instants, and this is the one
        place that difference is expressible. The cones go out at the cloud's
        own stamp, so their positions are native and need no compensation at
        all. The pixels have to line up with an image taken up to one bbox
        period earlier, so the projection, and only the projection, time-travels
        back to the image stamp. At 6.5 m/s a 33 ms gap is 0.21 m, which is
        roughly 20 px at 15 m against a cone about 30 px wide, so skipping that
        compensation would mis-associate real cones at range.

        Both transforms start from the LiDAR frame, so each is a single
        transform of the raw points rather than a correction on a correction.
        """
        if frame is None or frame.points_lidar.shape[0] == 0:
            return None
        age = abs(self._stamp_sec(image_stamp) - self._stamp_sec(frame.stamp))
        if age > self.max_cluster_age_sec:
            self._warn_throttled(
                "cluster_age",
                f"Newest bbox frame is {age:.2f}s from the cloud stamp (limit "
                f"{self.max_cluster_age_sec}s); publishing uncoloured cones")
            return None
        to_base = self._lookup_transform(
            self.output_frame, frame.frame_id, frame.stamp)
        to_camera = self._lookup_transform_between(
            self.camera_frame, image_stamp, frame.frame_id, frame.stamp)
        if to_base is None or to_camera is None:
            leg = ("cloud->base" if to_base is None else "cloud->image-stamp camera")
            lag = self._newest_tf_lag(image_stamp)
            self._warn_throttled(
                "cluster_tf",
                f"No TF to project the LiDAR points into the image stamp "
                f"({leg} failed); image stamp is {lag} the newest "
                f"{self.output_frame} transform; last error: "
                f"{getattr(self, '_last_tf_error', 'n/a')}; "
                f"keeping the LiDAR backbone uncoloured")
            return None
        return Scene(
            points_base=self._transform_points(frame.points_lidar, to_base),
            pixels=self._project(
                self._transform_points(frame.points_lidar, to_camera),
                camera_matrix),
            cluster_indices=frame.cluster_indices,
        )

    # ----------------------------------------------------------------- cones

    def _build_cones(
        self,
        detections: List[Detection],
        scene: Optional[Scene],
        camera_info: CameraInfo,
        camera_matrix: np.ndarray,
        stamp,
        left: Optional[Image],
        right: Optional[Image],
    ) -> List[Cone]:
        """Backbone, then colour, then sparse, then vision. In that order.

        The order IS the design: a LiDAR position always beats a vision one, and
        a cluster publishes whether or not the camera confirmed it.
        """
        cones: List[Cone] = []
        claimed: set = set()

        matched = self._match(detections, scene)
        if scene is not None:
            for index, indices in enumerate(scene.cluster_indices):
                centroid = scene.points_base[indices].mean(axis=0)
                if not np.all(np.isfinite(centroid[:2])):
                    continue
                detection = matched.get(index)
                if detection is not None:
                    claimed.add(detection)
                cx, cy = float(centroid[0]), float(centroid[1])
                rng = float(np.hypot(cx, cy))
                # The LiDAR samples only the near face, so the centroid sits
                # short of the cone axis along the bearing (measured lon mean
                # -0.046 m, lat -0.002: pure range bias). Push it forward.
                if self.cluster_range_bias_m and rng > 1e-6:
                    scale = (rng + self.cluster_range_bias_m) / rng
                    cx, cy = cx * scale, cy * scale
                    rng += self.cluster_range_bias_m
                cones.append(Cone(
                    x=cx, y=cy,
                    color=detection.color if detection else "unknown",
                    provenance=PROV_CLUSTER if detection else PROV_CLUSTER_ONLY,
                    range_m=rng,
                ))

        if self.sparse_enabled and scene is not None:
            clustered = (np.concatenate(scene.cluster_indices)
                         if scene.cluster_indices
                         else np.empty((0,), dtype=np.int64))
            for detection in detections:
                if detection in claimed:
                    continue
                cone = self._sparse_cone(detection, scene, clustered)
                if cone is None:
                    continue
                claimed.add(detection)
                cones.append(cone)

        if self.monocular_enabled:
            for detection in detections:
                if detection in claimed:
                    continue
                # The curve's cone is a CANDIDATE: it sizes the disparity search
                # and is then thrown away. Only a stereo-measured cone is
                # published, so no cone's position rests on the bbox height.
                candidate = self._monocular_cone(detection, camera_info,
                                                 camera_matrix, stamp)
                if candidate is None:
                    continue
                cone = self._stereo_cone(candidate, detection, camera_matrix,
                                         left, right, stamp)
                if cone is None:
                    continue
                # A cluster we failed to match plus a monocular estimate of the
                # same cone is exactly the duplicate this guards.
                if any(math.hypot(cone.x - c.x, cone.y - c.y)
                       <= self.vision_dedup_radius_m for c in cones):
                    continue
                cones.append(cone)
        return cones

    def _match(
        self,
        detections: List[Detection],
        scene: Optional[Scene],
    ) -> Dict[int, Detection]:
        """Greedy one-to-one cluster<->detection matching by pixel support.

        Scored by how many of a cluster's points land inside the box, weighted
        by confidence: a cone filling the box beats one clipping its corner, and
        a confident box beats a marginal one.
        """
        if scene is None or not detections or not scene.cluster_indices:
            return {}
        scores = []
        for d_index, detection in enumerate(detections):
            box = self._expanded_box(detection)
            for c_index, indices in enumerate(scene.cluster_indices):
                inside = self._inside_count(scene.pixels[indices], box)
                if inside < self.min_cluster_support_px:
                    continue
                scores.append((inside * detection.probability, c_index, d_index))
        matched: Dict[int, Detection] = {}
        used = set()
        for _score, c_index, d_index in sorted(scores, reverse=True):
            if c_index in matched or d_index in used:
                continue
            matched[c_index] = detections[d_index]
            used.add(d_index)
        return matched

    @staticmethod
    def _inside_count(pixels: np.ndarray, box) -> int:
        if pixels.shape[0] == 0:
            return 0
        return int(np.count_nonzero(
            (pixels[:, 0] >= box[0]) & (pixels[:, 0] <= box[2])
            & (pixels[:, 1] >= box[1]) & (pixels[:, 1] <= box[3])))

    def _sparse_cone(
        self,
        detection: Detection,
        scene: Scene,
        clustered: np.ndarray,
    ) -> Optional[Cone]:
        """A cone with LiDAR returns too thin to have clustered.

        Real support, so it keeps a LiDAR position and a LiDAR covariance. The
        checks are what stop a box from fusing whatever happens to lie behind
        it: the points must be cone-sized, at one depth, and enough of them for
        the range they claim.
        """
        box = self._expanded_box(detection)
        pixels = scene.pixels
        inside = np.flatnonzero(
            (pixels[:, 0] >= box[0]) & (pixels[:, 0] <= box[2])
            & (pixels[:, 1] >= box[1]) & (pixels[:, 1] <= box[3]))
        if inside.size == 0:
            return None
        # Points already spoken for by a cluster cannot also make a sparse cone.
        support = np.setdiff1d(inside, clustered, assume_unique=False)
        if support.size == 0:
            return None
        points = scene.points_base[support]
        centroid = points.mean(axis=0)
        range_m = float(np.hypot(centroid[0], centroid[1]))
        if not math.isfinite(range_m) or range_m <= 0.0:
            return None
        # Robust-inside-10-m scope: past the cap, far-and-thin is where a
        # false positive is cheapest, and the map does not need the reach.
        # Clusters are NOT capped -- a far cluster is accurate when it exists.
        if 0.0 < self.sparse_max_range_m < range_m:
            return None
        if support.size < self._sparse_min_points(range_m):
            return None
        span = points[:, :2].max(axis=0) - points[:, :2].min(axis=0)
        if float(np.hypot(*span)) > self.sparse_max_width_m:
            return None
        # One cone is at one depth. A spread of ranges means the box caught a
        # wall, or a cone and the ground behind it.
        ranges = np.hypot(points[:, 0], points[:, 1])
        allowed = max(self.sparse_max_depth_span_m,
                      range_m * self.sparse_max_depth_span_ratio)
        if float(ranges.max() - ranges.min()) > allowed:
            return None
        if range_m >= self.sparse_far_range_m:
            # Far and thin is where a false positive is cheapest to make, so
            # ask the detector to be sure and the box to be a real box.
            if (detection.probability < self.sparse_far_min_probability
                    or detection.width_px < self.sparse_far_min_bbox_width_px
                    or detection.height_px < self.sparse_far_min_bbox_height_px):
                return None
        x, y = float(centroid[0]), float(centroid[1])
        # Same front-face centroid bias as a cluster: these are raw LiDAR
        # returns off the cone's near surface (measured lon mean -0.054).
        if self.cluster_range_bias_m and range_m > 1e-6:
            scale = (range_m + self.cluster_range_bias_m) / range_m
            x, y = x * scale, y * scale
            range_m += self.cluster_range_bias_m
        return Cone(x=x, y=y,
                    color=detection.color, provenance=PROV_SPARSE,
                    range_m=range_m)

    def _sparse_min_points(self, range_m: float) -> int:
        if range_m < self.sparse_near_range_m:
            return self.sparse_near_min_points
        if range_m < self.sparse_far_range_m:
            return self.sparse_mid_min_points
        return self.sparse_far_min_points

    def _monocular_cone(
        self,
        detection: Detection,
        camera_info: CameraInfo,
        camera_matrix: np.ndarray,
        stamp,
    ) -> Optional[Cone]:
        """A camera-only cone, only where the estimate can be trusted."""
        if detection.height_px < self.monocular_min_bbox_height_px:
            return None                       # past the detector's cliff
        cone_height = self._cone_height_m(detection.color)
        if cone_height is None:
            return None                       # unknown class: no scale to use
        depth = self._monocular_depth(detection, camera_info, cone_height)
        if depth is None:
            return None
        base = self._back_project(detection, depth, camera_matrix, stamp)
        if base is None:
            return None
        return Cone(
            x=float(base[0]), y=float(base[1]), color=detection.color,
            provenance=PROV_MONOCULAR,
            range_m=float(np.hypot(base[0], base[1])),
            relative_depth_sigma=monocular_relative_depth_sigma(
                detection.height_px, self.monocular_depth_exponent,
                self.sigma_h_px),
            sigma_u_px=self.monocular_sigma_u_px,
            fx_px=float(camera_matrix[0, 0]),
        )

    def _monocular_depth(
        self,
        detection: Detection,
        camera_info: CameraInfo,
        cone_height_m: float,
    ) -> Optional[float]:
        """``D = c*h_n^e``, with ``c`` rescaled for a non-standard cone.

        ``c`` is the only part of the curve carrying the cone's size, so a big
        orange cone rescales it and keeps the exponent, which is a property of
        the detector rather than of the cone.
        """
        coefficient = self.monocular_depth_coefficient * (
            cone_height_m / self.standard_cone_height_m)
        depth = monocular_depth_from_bbox(
            detection.height_px, float(camera_info.height),
            coefficient=coefficient, exponent=self.monocular_depth_exponent)
        if depth is None or not (
                self.monocular_min_depth_m <= depth <= self.monocular_max_depth_m):
            return None
        return depth

    def _back_project(
        self,
        detection: Detection,
        depth: float,
        camera_matrix: np.ndarray,
        stamp,
    ) -> Optional[np.ndarray]:
        center_u, center_v = detection.center
        point = camera_point_from_depth(center_u, center_v, depth, camera_matrix)
        if point is None:
            return None
        camera_to_base = self._lookup_transform(
            self.output_frame, self.camera_frame, stamp)
        if camera_to_base is None:
            return None
        base = self._transform_points(
            self._to_camera_axes(point).reshape(1, 3), camera_to_base)[0]
        return base if np.all(np.isfinite(base[:2])) else None

    def _cone_height_m(self, color: str) -> Optional[float]:
        if color == "big_orange":
            return self.big_cone_height_m
        if color in ("blue", "yellow", "orange"):
            return self.standard_cone_height_m
        return None

    # ------------------------------------------------------------ ZNCC check

    def _stereo_cone(
        self,
        candidate: Cone,
        detection: Detection,
        camera_matrix: np.ndarray,
        left: Optional[Image],
        right: Optional[Image],
        stamp,
    ) -> Optional[Cone]:
        """Measure a vision cone's depth by stereo disparity, or publish nothing.

        ``candidate`` is NOT a publishable cone. It is the monocular curve's
        estimate, and it exists for one reason: to centre the disparity search.
        Its depth never reaches /perception/cones. A box-derived prior is 8 % low at 6 m and
        46 % low at 15 m, so the search window has to come from the fitted curve
        or it misses the true peak outright at range -- but the MEASUREMENT is
        the correlation peak, which does not use the cone-size assumption at all.

        Fails CLOSED. Every path that cannot produce a disparity peak returns
        None and the cone is dropped. This is the whole point of removing the
        monocular tier: a published cone's position must never trace back to
        "the box was this many pixels tall, and we assumed a cone is 450 mm".
        Failing open is what let that estimate out under a different name.
        """
        if not self.zncc_enabled or left is None or right is None:
            self._warn_throttled(
                "zncc_unavailable", "No stereo pair for the ZNCC measurement; "
                "dropping the vision cone (monocular depth is not published)")
            return None
        for image in (left, right):
            if abs(self._stamp_sec(stamp)
                   - self._stamp_sec(image.header.stamp)) > self.max_image_age_sec:
                self._warn_throttled(
                    "image_age", "Stereo images are too far from the bbox stamp "
                    "for the ZNCC measurement; dropping the vision cone")
                return None
        # The pair must be the same instant as EACH OTHER, which is a stricter
        # thing than both being near the bbox. Disparity between images taken at
        # different times contains the car's motion, and a triangulation cannot
        # tell that apart from depth: 33 ms at 6.5 m/s is 0.21 m of pure error,
        # reported with a confident sigma. Checking only against the bbox stamp
        # let two images 2x max_image_age_sec apart both pass.
        pair_gap = abs(self._stamp_sec(left.header.stamp)
                       - self._stamp_sec(right.header.stamp))
        if pair_gap > self.max_pair_skew_sec:
            self._warn_throttled(
                "pair_skew",
                f"Stereo pair is {pair_gap * 1000:.0f} ms apart (limit "
                f"{self.max_pair_skew_sec * 1000:.0f} ms); the disparity would "
                f"be ego motion, not depth. Dropping the vision cone")
            return None
        left_gray = self._to_gray(left)
        right_gray = self._to_gray(right)
        if left_gray is None or right_gray is None:
            return None

        fx = float(camera_matrix[0, 0])
        # The candidate's depth chooses WHERE to look, and nothing else. A
        # box-derived prior (d = h*B/H) is 8 % low at 6 m and 46 % low at 15 m,
        # so a [0.70, 1.45] window around one misses the true peak outright at
        # range; the fitted curve's depth puts the window on the truth. What
        # comes out is the correlation peak, which never uses the cone-size
        # assumption -- which is why the curve may pick the window without its
        # depth reaching /perception/cones.
        prior = fx * self.stereo_baseline_m / max(candidate.range_m, 1.0e-3)
        match = zncc_disparity(
            left_gray, right_gray,
            (detection.xmin, detection.ymin, detection.xmax, detection.ymax),
            prior * self.zncc_search_low_ratio,
            prior * self.zncc_search_high_ratio,
            min_score=self.zncc_min_score,
        )
        self._zncc_stats[0] += 1
        if match is None:
            return None
        self._zncc_stats[1] += 1

        depth = fx * self.stereo_baseline_m / match.disparity_px
        if not (self.monocular_min_depth_m <= depth <= self.monocular_max_depth_m):
            return None
        base = self._back_project(detection, depth, camera_matrix, stamp)
        if base is None:
            return None
        return Cone(
            x=float(base[0]), y=float(base[1]), color=candidate.color,
            provenance=PROV_MONO_ZNCC,
            range_m=float(np.hypot(base[0], base[1])),
            relative_depth_sigma=stereo_relative_depth_sigma(
                depth, fx, self.stereo_baseline_m, self.sigma_d_px),
            # The bearing still comes from the same bbox centre, so it keeps the
            # curve's measured sigma_u. Stereo fixes depth, not bearing -- which
            # is why monocular_min_bbox_height_px still gates this path: past
            # ~9 m the box CENTRE degrades to 17-32 px and stereo cannot help.
            sigma_u_px=self.monocular_sigma_u_px,
            fx_px=fx,
        )

    def _to_gray(self, image: Image) -> Optional[np.ndarray]:
        """Grey the image once per cloud, not once per detection.

        This is called from _stereo_cone, which runs PER VISION CANDIDATE, and
        it converts the WHOLE image: 1280x720x3 uint8 -> float64 is a 22 MB
        allocation before the mean, twice (left and right) for every candidate.
        The images do not change within a cloud, so the second candidate onward
        was re-deriving an identical array.

        It also explains the tail rather than just the mean: the cost was
        proportional to the number of vision candidates in the frame, which is
        why build_cones measured 31 ms median against a 129 ms p90 -- a frame
        with six candidates paid six times.

        Keyed on id(), which is safe here and only here: the cache is cleared at
        the top of each cloud callback and the images are held alive by `left`
        and `right` for the whole of it, so an id cannot be recycled underneath.
        """
        cached = self._gray_cache.get(id(image))
        if cached is not None:
            return cached
        try:
            array = image_message_to_numpy(image)
        except Exception:
            return None
        if array is None or array.size == 0:
            return None
        gray = None
        if array.ndim == 2:
            gray = array.astype(np.float64)
        elif array.ndim == 3 and array.shape[2] >= 3:
            # Luma only; ZNCC is normalised so the exact weights do not matter.
            #
            # `@` rather than `.astype(np.float64).mean(axis=2)`, which is not a
            # style preference: MEASURED 11.04 ms per image, of which the cast is
            # 0.81 and the mean is 10.08. A reduction over a length-3 axis is
            # pathological in numpy -- it strides the wrong way and gets no
            # vectorisation -- while the same arithmetic as a matrix product goes
            # to BLAS. 2.69 ms per image, 5.2 vs 23.0 per stereo pair, and the
            # two agree to 2.8e-14, which is float64 rounding.
            #
            # float32 would be 1.11 ms but the results differ by 2e-5, and this
            # feeds ZNCC's correlation peak. Not worth 3 ms.
            gray = array[:, :, :3] @ np.array([1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0])
        if gray is not None:
            self._gray_cache[id(image)] = gray
        return gray

    # ------------------------------------------------------------ covariance

    def _covariance(self, cone: Cone) -> List[float]:
        """The trust handed to SLAM. Two models, split by what was measured.

        A vision cone's error is an ellipse along the line of sight: weak in
        depth, strong in bearing, its major axis on the ray rather than on x or
        y. A LiDAR cone's is not -- it ranges directly to ~1 %, so the
        near-circle a constant describes is roughly the truth.
        """
        ellipse = self._vision_covariance(cone)
        if ellipse is not None:
            return ellipse
        if cone.provenance == PROV_SPARSE:
            ellipse = self._sparse_covariance(cone)
            if ellipse is not None:
                return ellipse
        base_x, base_y = {
            PROV_CLUSTER: (self.lidar_variance_x, self.lidar_variance_y),
        }.get(cone.provenance,
              (self.lidar_only_variance_x, self.lidar_only_variance_y))
        # Speed-proportional timing variance (see the parameter comment):
        # the alignment residual scales with speed, so the fixed variances
        # above (fit at ~6 m/s) undercover at trackdrive speed exactly where
        # SLAM gates on this covariance.
        timing_var = 0.0
        twist = self._twist
        if twist is not None:
            speed = (twist[0] * twist[0] + twist[1] * twist[1]) ** 0.5
            timing_sigma = self.lidar_timing_sigma_per_mps * speed
            timing_var = timing_sigma * timing_sigma
        return [
            max(base_x + self.range_variance_scale * cone.range_m + timing_var,
                self.min_variance),
            0.0, 0.0,
            max(base_y + self.range_variance_scale * cone.range_m + timing_var,
                self.min_variance),
        ]

    def _sparse_covariance(self, cone: Cone) -> Optional[List[float]]:
        """A sparse cone's error is an ellipse too, and it points the other way.

        A CLUSTER has enough returns to find the cone's centre, and measures it
        near-isotropically: lateral/longitudinal rms 0.041 / 0.035 m (n=5203).
        One constant is the truth for it.

        A SPARSE cone was thought not to: 2-3 returns on the cone's FRONT FACE
        should fix the BEARING while leaving the RANGE noisy. THE CLEAN DATA
        REFUSES THAT. Measured on a verified single stack, three runs:

            lateral       0.056  0.058  0.078 m
            longitudinal  0.064  0.065  0.051 m

        Those are the same number. The 2.7x longitudinal spread that motivated
        this ellipse came from two perception nodes publishing at once, which
        this code did not notice until n doubled for an unchanged 60 s window.

        It survives only because it is measured SAFE (lat z^2 0.32/0.34/0.61,
        lon 0.07/0.07/0.04 -- never over-confident) and deleting it is a change
        nobody has run yet. See perception.yaml: the honest end state is one
        isotropic constant at ~0.017 and this method gone.
        """
        sigmas = bearing_aligned_covariance(
            cone.x, cone.y, self.sparse_sigma_lon_m, self.sparse_sigma_lat_m)
        if sigmas is None:
            return None
        xx, xy, yx, yy = sigmas
        return [max(xx, self.min_variance), xy, yx, max(yy, self.min_variance)]

    def _vision_covariance(self, cone: Cone) -> Optional[List[float]]:
        if (cone.relative_depth_sigma is None or cone.sigma_u_px is None
                or cone.fx_px is None or not math.isfinite(cone.range_m)
                or cone.range_m <= 0.0):
            return None
        # Both sigmas scale with range: the bearing one because a pixel subtends
        # a widening arc, the depth one because the measurement bounds the
        # FRACTION sigma_D/D -- which is why it survives the projection above.
        sigma_lon = cone.range_m * cone.relative_depth_sigma
        # The pixel term alone is a straight line through the origin, and the
        # error is not: MEASURED lateral rms by range band was 0.136 / 0.128 /
        # 0.258 / 0.281 / 0.139 / 0.229 / 0.197 / 0.368 m from 0 to 20 m (n=480)
        # -- no range trend at all, while the pixel term ran 0.052 -> 0.406 m.
        # So the model was over-confident by 2.6x at 1 m and over-cautious by
        # 0.9x at 18 m, and NO value of sigma_u_px fixes both: 3.0 implied 10,
        # 10 implied 18.6. A floor is the missing term, and adding it is what
        # makes the sigma honest near, which is the half that corrupts the map.
        #
        # The floor's CAUSE is not established. A pixel error cannot produce a
        # range-independent error in metres, so something else dominates --
        # _carry_vision_cones moves a vision cone by (stamp gap x speed), and the
        # bbox-to-cloud stamp gap was measured at 10 ms median but 173 ms p90 and
        # 407 ms max, which at 5 m/s is 0.05 to 2 m of pure carry. That jitter is
        # Gazebo's bursty rendering and does NOT exist on the car, so this floor
        # may be fitting the simulator. It is set to cover the observed error
        # rather than to model it: over-stating costs information, under-stating
        # corrupts the map. Re-measure on real sensor data before trusting it.
        sigma_lat = math.hypot(
            self.vision_lateral_floor_m,
            cone.range_m * cone.sigma_u_px / cone.fx_px)
        covariance = bearing_aligned_covariance(cone.x, cone.y, sigma_lon,
                                                sigma_lat)
        if covariance is None:
            return None
        xx, xy, yx, yy = covariance
        # Clamp the diagonal only. Scaling the off-diagonal to match would
        # rotate the ellipse off the bearing; letting the floor lift the axes
        # can only make it rounder, which is the safe direction for an optimizer
        # that believes what it is told.
        return [max(xx, self.min_variance), xy, yx, max(yy, self.min_variance)]

    # ---------------------------------------------------------------- output

    def _publish(self, cones: List[Cone], stamp) -> None:
        msg = ConeArrayWithCovariance()
        msg.header.stamp = stamp
        msg.header.frame_id = self.output_frame
        buckets: Dict[str, list] = {color: [] for color in CONE_COLORS}
        # Compute each covariance once and hand the same one to the message and
        # to RViz, so what is drawn is what SLAM was told rather than a
        # re-derivation that could drift from it.
        covariances = [self._covariance(cone) for cone in cones]
        for cone, covariance in zip(cones, covariances):
            out = ConeWithCovariance()
            out.point.x, out.point.y, out.point.z = cone.x, cone.y, 0.0
            out.covariance = covariance
            buckets.get(cone.color, buckets["unknown"]).append(out)
        msg.blue_cones = buckets["blue"]
        msg.yellow_cones = buckets["yellow"]
        msg.orange_cones = buckets["orange"]
        msg.big_orange_cones = buckets["big_orange"]
        msg.unknown_color_cones = buckets["unknown"]
        self._record_latency(stamp)
        self.cones_pub.publish(msg)
        self.cones_viz_pub.publish(
            self._cone_markers(cones, covariances, stamp))

    def _record_latency(self, stamp) -> None:
        """Age of the data in /perception/cones: now - header.stamp.

        The header carries the bbox capture time, so this is the whole
        pipeline's latency as the planner experiences it -- detector, transport
        and this node -- not this node's compute time.
        """
        if self.latency_log_period_sec <= 0.0:
            return
        now_ns = int(self.get_clock().now().nanoseconds)
        if now_ns <= 0:                        # before the first /clock
            return
        self._latencies.append((now_ns - self._stamp_ns(stamp)) / 1e9)
        if self._last_latency_log_ns is None:
            self._last_latency_log_ns = now_ns
            return
        elapsed = (now_ns - self._last_latency_log_ns) / 1e9
        # A clock jump backwards (bag loop, sim reset) would otherwise park this
        # until sim time caught back up.
        if elapsed < 0.0:
            self._last_latency_log_ns = now_ns
            self._latencies.clear()
            return
        if elapsed < self.latency_log_period_sec:
            return
        samples = sorted(self._latencies)
        self._latencies.clear()
        self._last_latency_log_ns = now_ns
        if not samples:
            return
        attempted, confirmed = self._zncc_stats
        zncc = (f"  zncc {confirmed}/{attempted} confirmed"
                if attempted else "")
        self._zncc_stats = [0, 0]
        self.get_logger().info(
            f"{self.output_cones_topic} latency (now - cloud stamp): "
            f"mean {1000 * sum(samples) / len(samples):.0f} ms  "
            f"p90 {1000 * samples[min(len(samples) - 1, int(0.9 * len(samples)))]:.0f} ms  "
            f"max {1000 * samples[-1]:.0f} ms  over {len(samples)} msgs / "
            f"{elapsed:.1f} s ({len(samples) / elapsed:.1f} Hz){zncc}")
        self._log_stages()

    def _cone_markers(self, cones: List[Cone], covariances: List[List[float]],
                      stamp) -> MarkerArray:
        rgb = {"blue": (0.0, 0.0, 1.0), "yellow": (1.0, 1.0, 0.0),
               "orange": (1.0, 0.5, 0.0), "big_orange": (1.0, 0.3, 0.0),
               "unknown": (0.6, 0.6, 0.6)}
        array = MarkerArray()
        array.markers.append(self._delete_all(stamp))
        for index, (cone, covariance) in enumerate(zip(cones, covariances)):
            marker = self._marker(stamp, cone.color, index, cone)
            marker.type = Marker.SPHERE
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            red, green, blue = rgb.get(cone.color, rgb["unknown"])
            marker.color.r, marker.color.g, marker.color.b = red, green, blue
            marker.color.a = 0.9
            array.markers.append(marker)
            ellipse = self._covariance_marker(cone, covariance, index, stamp)
            if ellipse is not None:
                array.markers.append(ellipse)
        return array

    def _covariance_marker(self, cone: Cone, covariance: List[float],
                           index: int, stamp) -> Optional[Marker]:
        """The 1-sigma error ellipse, drawn flat on the ground.

        Worth looking at rather than trusting: this is the number SLAM weights
        the cone by, and it is the whole point of the vision tiers reporting an
        ellipse instead of a circle. A vision cone should look like a sliver
        pointing back at the car -- precise across the track, vague along the
        ray. A LiDAR cone should look like a small disc. If a far vision cone
        renders as a tight disc, its covariance is lying and the map will be
        corrupted rather than merely noisy.
        """
        axes = self._covariance_axes(covariance)
        if axes is None:
            return None
        major, minor, yaw = axes
        marker = self._marker(stamp, f"{cone.provenance}_covariance", index, cone)
        marker.type = Marker.CYLINDER
        # Flat on the ground: this is a 2D covariance and drawing it as a
        # sphere would imply a vertical extent that was never measured.
        marker.scale.x = 2.0 * major        # full width = 2 sigma across
        marker.scale.y = 2.0 * minor
        marker.scale.z = 0.01
        marker.pose.orientation.z = math.sin(0.5 * yaw)
        marker.pose.orientation.w = math.cos(0.5 * yaw)
        red, green, blue = ((1.0, 0.2, 0.2) if cone.provenance
                            in (PROV_MONOCULAR, PROV_MONO_ZNCC)
                            else (0.2, 1.0, 0.4))
        marker.color.r, marker.color.g, marker.color.b = red, green, blue
        marker.color.a = 0.35
        return marker

    @staticmethod
    def _covariance_axes(
        covariance: List[float],
    ) -> Optional[Tuple[float, float, float]]:
        """(major sigma, minor sigma, yaw of the major axis) from [xx,xy,yx,yy].

        Eigen-decomposed rather than read off the diagonal, because the whole
        reason this covariance exists is that it is NOT axis-aligned.
        """
        xx, xy, _yx, yy = (float(v) for v in covariance)
        if not all(math.isfinite(v) for v in (xx, xy, yy)):
            return None
        trace, det = xx + yy, xx * yy - xy * xy
        spread = math.sqrt(max(0.0, 0.25 * trace * trace - det))
        major_var, minor_var = 0.5 * trace + spread, 0.5 * trace - spread
        if major_var <= 0.0 or minor_var <= 0.0:
            return None
        # Eigenvector of the LARGER eigenvalue. atan2(0, 0) when the matrix is
        # already isotropic, which is correct: any axis will do.
        yaw = 0.5 * math.atan2(2.0 * xy, xx - yy)
        return math.sqrt(major_var), math.sqrt(minor_var), yaw

    def _provenance_markers(self, cones: List[Cone], stamp) -> MarkerArray:
        """One label per published cone, at the position that was published.

        /perception/cones carries no provenance, so error measured against ground truth
        cannot otherwise be attributed to the thing that produced it.
        """
        array = MarkerArray()
        array.markers.append(self._delete_all(stamp))
        for index, cone in enumerate(cones):
            marker = self._marker(stamp, cone.provenance, index, cone)
            marker.type = Marker.TEXT_VIEW_FACING
            marker.pose.position.z = 0.4
            marker.scale.z = 0.2
            marker.color.r = marker.color.g = marker.color.b = 1.0
            marker.color.a = 0.9
            marker.text = cone.provenance
            array.markers.append(marker)
        return array

    def _marker(self, stamp, namespace: str, index: int, cone: Cone) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.output_frame
        marker.ns = namespace
        marker.id = index
        marker.action = Marker.ADD
        marker.pose.position.x = cone.x
        marker.pose.position.y = cone.y
        marker.pose.orientation.w = 1.0
        return marker

    def _delete_all(self, stamp) -> Marker:
        # Required: the marker count varies per frame, so stale ids would linger.
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.output_frame
        marker.action = Marker.DELETEALL
        return marker

    # --------------------------------------------------------------- helpers

    def _extract_detections(
        self,
        msg: BoundingBoxes,
        camera_info: CameraInfo,
    ) -> List[Detection]:
        detections = []
        for box in getattr(msg, "bounding_boxes", []):
            probability = float(getattr(box, "probability", 1.0))
            if not math.isfinite(probability) or not 0.0 <= probability <= 1.0:
                continue
            if probability < self.min_bbox_probability:
                continue
            corners = self._box_to_pixels(box, camera_info)
            if corners is None or corners[2] <= corners[0] or corners[3] <= corners[1]:
                continue
            detections.append(Detection(
                color=self._normalize_color(getattr(box, "color", "")),
                probability=probability,
                xmin=corners[0], ymin=corners[1],
                xmax=corners[2], ymax=corners[3]))
        return detections

    def _box_to_pixels(self, box, camera_info) -> Optional[Tuple[float, ...]]:
        values = [float(box.xmin), float(box.ymin), float(box.xmax),
                  float(box.ymax)]
        if not all(math.isfinite(v) for v in values):
            return None
        if self.bbox_coordinates == "PERCENTAGE":
            width, height = float(camera_info.width), float(camera_info.height)
            values = [values[0] * width, values[1] * height,
                      values[2] * width, values[3] * height]
        xmin, xmax = sorted((values[0], values[2]))
        ymin, ymax = sorted((values[1], values[3]))
        return (xmin, ymin, xmax, ymax)

    def _expanded_box(self, detection: Detection) -> Tuple[float, ...]:
        margin_x = max(self.bbox_match_margin_px,
                       detection.width_px * self.bbox_match_margin_ratio)
        margin_y = max(self.bbox_match_margin_px,
                       detection.height_px * self.bbox_match_margin_ratio)
        return (detection.xmin - margin_x, detection.ymin - margin_y,
                detection.xmax + margin_x, detection.ymax + margin_y)

    @staticmethod
    def _normalize_color(raw: str) -> str:
        text = str(raw).strip().lower()
        if "big" in text and "orange" in text:
            return "big_orange"
        for color in ("blue", "yellow", "orange"):
            if color in text:
                return color
        return "unknown"

    @staticmethod
    def _camera_matrix(camera_info: CameraInfo) -> np.ndarray:
        """Prefer P over K: P is the rectified projection the images match."""
        projection = np.asarray(camera_info.p, dtype=np.float64)
        if projection.size == 12:
            candidate = projection.reshape(3, 4)[:, :3]
            if (np.all(np.isfinite(candidate)) and candidate[0, 0] > 0.0
                    and candidate[1, 1] > 0.0
                    and abs(float(np.linalg.det(candidate))) > 1.0e-12):
                return candidate.copy()
        intrinsic = np.asarray(camera_info.k, dtype=np.float64)
        if intrinsic.size != 9:
            return np.full((3, 3), np.nan, dtype=np.float64)
        return intrinsic.reshape(3, 3).copy()

    def _to_camera_axes(self, projection_point: np.ndarray) -> np.ndarray:
        """Pinhole (x right, y down, z forward) -> the model's camera axes.

        The inverse of the permutation in _project. The two MUST stay inverses.
        """
        if self.projection_model == "eufs_bbox":
            return np.asarray([projection_point[2], -projection_point[0],
                               -projection_point[1]], dtype=np.float64)
        return np.asarray(projection_point, dtype=np.float64)

    def _project(self, points_camera: np.ndarray,
                 camera_matrix: np.ndarray) -> np.ndarray:
        """Camera-frame points -> pixels, NaN where behind the camera.

        NaN rather than a filtered array so the result stays index-aligned with
        the points: every consumer here addresses points by index.
        """
        count = points_camera.shape[0]
        pixels = np.full((count, 2), np.nan, dtype=np.float64)
        if count == 0:
            return pixels
        if self.projection_model == "eufs_bbox":
            projection = np.column_stack((-points_camera[:, 1],
                                          -points_camera[:, 2],
                                          points_camera[:, 0]))
        else:
            projection = points_camera
        valid = projection[:, 2] > self.min_project_depth
        if not np.any(valid):
            return pixels
        points = projection[valid]
        fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
        cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
        pixels[valid, 0] = fx * points[:, 0] / points[:, 2] + cx
        pixels[valid, 1] = fy * points[:, 1] / points[:, 2] + cy
        return pixels

    def _newest_tf_lag(self, stamp) -> str:
        """How far the requested stamp sits past the newest transform we hold.

        A positive number means the lookup asked TF to extrapolate into the
        future, which it refuses — that is a producer-latency fault, not a
        frame-wiring one, and it is the failure the load-dependent colour
        collapse looks like.
        """
        try:
            # Time() = "latest available". The dynamic link is the one SLAM
            # publishes (map->odom->base); the lidar link is static and always
            # current, so asking for it would say nothing.
            newest = self.tf_buffer.lookup_transform(
                self.motion_compensation_frame or "map", self.output_frame,
                rclpy.time.Time())
            gap = self._stamp_sec(stamp) - self._stamp_sec(newest.header.stamp)
            return f"{gap:+.3f}s from"
        except Exception:
            return "(newest unknown) vs"

    def _lookup_transform(self, target: str, source: str,
                          stamp) -> Optional[np.ndarray]:
        try:
            transform = self.tf_buffer.lookup_transform(
                target, source, stamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec))
        except tf2_ros.TransformException as exc:
            # Keep the reason. "No TF" is three different faults wearing one
            # message — extrapolation past the newest transform, a missing
            # static link, and a buffer that no longer reaches back — and they
            # have different fixes.
            self._last_tf_error = f"{target}<-{source}: {exc}"
            return None
        return self._transform_to_matrix(transform)

    def _lookup_transform_between(self, target: str, target_stamp, source: str,
                                  source_stamp) -> Optional[np.ndarray]:
        """Transform across BOTH frames and time, through a fixed frame.

        This is the motion compensation: the cloud was captured while the car
        was somewhere else, so its points must be carried to where the car is at
        the bbox stamp, or every LiDAR cone lags by the cloud's age.
        """
        if (self._stamp_ns(target_stamp) == self._stamp_ns(source_stamp)
                or not self.motion_compensation_frame):
            return self._lookup_transform(target, source, source_stamp)
        try:
            transform = self.tf_buffer.lookup_transform_full(
                target, target_stamp, source, source_stamp,
                self.motion_compensation_frame,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec))
        except tf2_ros.TransformException as exc:
            self._last_tf_error = (
                f"{target}@target_stamp<-{source}@source_stamp "
                f"via {self.motion_compensation_frame}: {exc}")
            # Measured: the request lands 4 ms past the newest transform and the
            # whole frame is published uncoloured. At 6.5 m/s that compensation
            # is worth 2.6 cm, against the 0.21 m of cone lag it exists to
            # remove — so dropping colour to avoid it inverts the trade. Fall
            # back to the uncompensated transform for gaps this small, and only
            # give up when the gap is big enough to actually mis-associate.
            gap = abs(self._stamp_sec(target_stamp) - self._stamp_sec(source_stamp))
            if gap <= self.max_uncompensated_gap_sec:
                fallback = self._lookup_transform(target, source, source_stamp)
                if fallback is not None:
                    self._warn_throttled(
                        "tf_uncompensated",
                        f"No cross-time TF ({gap*1000:.0f} ms); colouring with "
                        f"the uncompensated transform instead of dropping the "
                        f"frame")
                    return fallback
            return None
        return self._transform_to_matrix(transform)

    @staticmethod
    def _transform_to_matrix(transform) -> np.ndarray:
        t = transform.transform.translation
        q = transform.transform.rotation
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        matrix = np.eye(4, dtype=np.float64)
        if norm > 0.0:
            x, y, z, w = x / norm, y / norm, z / norm, w / norm
            matrix[:3, :3] = np.asarray([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ], dtype=np.float64)
        matrix[:3, 3] = [float(t.x), float(t.y), float(t.z)]
        return matrix

    @staticmethod
    def _transform_points(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        return points.dot(matrix[:3, :3].T) + matrix[:3, 3]

    @staticmethod
    def _bbox_stamp(msg: BoundingBoxes):
        """The image capture time, which every cone in the output belongs to."""
        image_header = getattr(msg, "image_header", None)
        if image_header is not None and (image_header.stamp.sec
                                         or image_header.stamp.nanosec):
            return image_header.stamp
        return msg.header.stamp

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @classmethod
    def _stamp_sec(cls, stamp) -> float:
        return cls._stamp_ns(stamp) / 1e9

    def _warn_throttled(self, key: str, message: str,
                        period_sec: float = 5.0) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._warned.get(key, -math.inf) < period_sec:
            return
        self._warned[key] = now
        self.get_logger().warning(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PerceptionNode()
        # Multi-threaded, so the TF listener's ReentrantCallbackGroup is served
        # while the cone callback is busy. The sensor callbacks stay in the
        # node's default (mutually exclusive) group and so still serialise with
        # each other -- only /tf and /tf_static gain a thread of their own.
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
