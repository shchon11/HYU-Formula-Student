import math
import struct
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

import numpy as np
import rclpy
from eufs_msgs.msg import (
    BoundingBox,
    BoundingBoxes,
    ConeArrayWithCovariance,
    ConeWithCovariance,
)
from geometry_msgs.msg import Point
from rclpy.clock import ClockChange, JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from eufs_perception_baseline.fusion_core import (
    baseline_from_projection,
    camera_point_from_depth,
    classify_cone_condition,
    estimate_stereo_depth,
    monocular_depth_from_bbox,
    remove_ground_ransac,
)
from eufs_perception_baseline.ros_image_utils import image_message_to_numpy
from eufs_perception_baseline.sync_buffer import TimestampBuffer

try:
    from sensor_msgs_py import point_cloud2
except ImportError:
    from eufs_perception_baseline import point_cloud2_compat as point_cloud2


StampKey = Tuple[int, int]
SyncKey = Tuple[StampKey, StampKey]
INT64_MAX = (1 << 63) - 1
INT64_MIN = -(1 << 63)


def _sift_available() -> bool:
    """Return whether the active OpenCV build provides the Tier-3 primitive."""
    try:
        import cv2
    except ImportError:
        return False
    return callable(getattr(cv2, "SIFT_create", None))


@dataclass
class Detection:
    color: str
    probability: float
    xmin: float
    ymin: float
    xmax: float
    ymax: float


@dataclass
class Cluster:
    points_base: np.ndarray
    points_camera: np.ndarray
    centroid_base: np.ndarray
    range_m: float
    indices: np.ndarray
    source: str = "cluster"
    support_count: int = 0
    consumed_indices: Optional[np.ndarray] = None


@dataclass
class Assignment:
    detection_index: int
    detection: Detection
    cluster: Cluster
    source: str
    support_count: int


@dataclass
class DetectionDebugReport:
    detection_index: int
    detection: Detection
    raw_projected_points: int
    roi_projected_points: int
    cluster_projected_points: int
    assigned_source: str
    reason: str
    support_centroid_base: Optional[np.ndarray]


class PerceptionBaselineNode(Node):
    """
    Baseline perception node for EUFS graph SLAM.

    Two input modes are intentionally kept in one node:
    - Oracle adapter mode republishes simulator cone arrays into the SLAM contract.
    - Sensor fusion mode follows the IIT Bombay tier priority: projected LiDAR
      support first, normalized bbox-height monocular depth for good cones, and
      central slender-region SIFT stereo for bad cones.
    """

    def __init__(self) -> None:
        super().__init__("perception_baseline_node")

        self._declare_parameters()
        self._load_parameters()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.sync_queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.image_sync_queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cones_pub = self.create_publisher(
            ConeArrayWithCovariance,
            self.output_cones_topic,
            10,
        )
        self.cones_viz_pub = self.create_publisher(
            MarkerArray,
            self._viz_topic_for(self.output_cones_topic),
            10,
        )
        self.debug_roi_points_pub = None
        self.debug_sparse_support_pub = None
        self.debug_cluster_candidates_pub = None
        self.debug_bbox_support_pub = None
        self.debug_rejections_pub = None
        if self.publish_fusion_debug:
            self.debug_roi_points_pub = self.create_publisher(
                PointCloud2,
                f"{self.fusion_debug_prefix}/roi_points",
                10,
            )
            self.debug_sparse_support_pub = self.create_publisher(
                PointCloud2,
                f"{self.fusion_debug_prefix}/sparse_support_points",
                10,
            )
            self.debug_cluster_candidates_pub = self.create_publisher(
                MarkerArray,
                f"{self.fusion_debug_prefix}/cluster_candidates",
                10,
            )
            self.debug_bbox_support_pub = self.create_publisher(
                MarkerArray,
                f"{self.fusion_debug_prefix}/bbox_support",
                10,
            )
            self.debug_rejections_pub = self.create_publisher(
                MarkerArray,
                f"{self.fusion_debug_prefix}/rejections",
                10,
            )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            image_qos,
        )
        self.right_image_sub = self.create_subscription(
            Image,
            self.right_image_topic,
            self._right_image_callback,
            image_qos,
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._pointcloud_callback,
            sensor_qos,
        )
        self.bbox_sub = self.create_subscription(
            BoundingBoxes,
            self.bbox_topic,
            self._bbox_callback,
            10,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            sensor_qos,
        )
        self.right_camera_info_sub = self.create_subscription(
            CameraInfo,
            self.right_camera_info_topic,
            self._right_camera_info_callback,
            sensor_qos,
        )

        self.oracle_sub = None
        if self.oracle_cones_topic:
            self.oracle_sub = self.create_subscription(
                ConeArrayWithCovariance,
                self.oracle_cones_topic,
                self._oracle_callback,
                10,
            )

        self.latest_image: Optional[Image] = None
        self.latest_right_image: Optional[Image] = None
        self.latest_pointcloud: Optional[PointCloud2] = None
        self.latest_bboxes: Optional[BoundingBoxes] = None
        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_right_camera_info: Optional[CameraInfo] = None
        self.image_buffer = TimestampBuffer(self.image_sync_queue_size)
        self.right_image_buffer = TimestampBuffer(self.image_sync_queue_size)
        self.pointcloud_buffer = TimestampBuffer(self.sync_queue_size)
        self.bbox_buffer = TimestampBuffer(self.sync_queue_size)
        self.latest_stream_stamp_ns = {}
        self.sensor_epoch = 0
        self.epoch_start_ros_time_ns: Optional[int] = None
        self.replay_guard_end_ros_time_ns: Optional[int] = None
        self.replay_guard_ends_ros_time_ns: List[int] = []
        self._pending_fallback_rollback: Optional[Tuple[int, int]] = None
        self.last_observed_ros_time_ns = self.get_clock().now().nanoseconds
        self.last_empty_key: Optional[SyncKey] = None
        self.last_fusion_key: Optional[SyncKey] = None
        self.last_warning_time = {}
        self.clock_jump_handler = self.get_clock().create_jump_callback(
            JumpThreshold(
                # Galactic converts None to zero, which schedules this callback
                # for every positive /clock tick.  Use the largest Duration to
                # disable practical forward-jump callbacks; the post-callback
                # still filters delta defensively for version differences.
                min_forward=Duration(nanoseconds=(1 << 63) - 1),
                min_backward=Duration(
                    nanoseconds=-self.timestamp_reset_threshold_ns
                ),
                on_clock_change=True,
            ),
            post_callback=self._on_clock_jump,
        )

        self.get_logger().info(
            "Publishing SLAM cone observations on "
            f"{self.output_cones_topic} as eufs_msgs/msg/ConeArrayWithCovariance"
        )
        self.get_logger().info(
            "Publishing RViz cone markers on "
            f"{self._viz_topic_for(self.output_cones_topic)} "
            "as visualization_msgs/msg/MarkerArray"
        )
        self.get_logger().info(
            f"Fusion inputs: left={self.image_topic}, right={self.right_image_topic}, "
            f"pointcloud={self.pointcloud_topic}, bboxes={self.bbox_topic}, "
            f"left_info={self.camera_info_topic}, right_info={self.right_camera_info_topic}"
        )
        if self.oracle_cones_topic:
            self.get_logger().info(
                f"Oracle adapter enabled: {self.oracle_cones_topic} -> {self.output_cones_topic}"
            )
        if self.fusion_enabled:
            self.get_logger().info(
                "LiDAR-camera fusion enabled. BBox class/color is fused with LiDAR clusters."
            )
        if self.publish_fusion_debug:
            self.get_logger().info(
                f"Fusion debug topics enabled under {self.fusion_debug_prefix}"
            )

    def _declare_parameters(self) -> None:
        # The simulator's /noisy_bounding_boxes and /custom_camera_info are
        # produced in the right optical-camera contract.  YOLO launch mode
        # explicitly overrides these defaults with the left image/frame.
        self.declare_parameter("image_topic", "/zed/right/image_rect_color")
        self.declare_parameter("right_image_topic", "/zed/right/image_rect_color")
        self.declare_parameter("pointcloud_topic", "/velodyne_points")
        self.declare_parameter("bbox_topic", "/noisy_bounding_boxes")
        self.declare_parameter("camera_info_topic", "/custom_camera_info")
        self.declare_parameter("right_camera_info_topic", "/zed/right/camera_info")
        self.declare_parameter("camera_frame", "zed_right_camera_optical_frame")
        self.declare_parameter("right_camera_frame", "zed_right_camera_optical_frame")
        self.declare_parameter("projection_model", "eufs_bbox")
        self.declare_parameter("clip_bboxes_to_image", False)
        self.declare_parameter("clip_projected_points_to_image", False)
        self.declare_parameter("output_cones_topic", "/cones")
        self.declare_parameter("output_frame", "base_footprint")
        self.declare_parameter("motion_compensation_frame", "map")
        self.declare_parameter("marker_scale", 0.35)
        self.declare_parameter("sync_tolerance_sec", 0.15)
        self.declare_parameter("image_sync_tolerance_sec", 0.05)
        self.declare_parameter("sync_queue_size", 12)
        self.declare_parameter("image_sync_queue_size", 64)
        self.declare_parameter("timestamp_reset_threshold_sec", 0.1)
        self.declare_parameter("max_future_stamp_lead_sec", 0.09)
        self.declare_parameter("publish_empty_on_sync", False)
        self.declare_parameter("fusion_enabled", True)
        self.declare_parameter("publish_fusion_debug", True)
        self.declare_parameter("fusion_debug_prefix", "/fusion/debug")
        self.declare_parameter("oracle_cones_topic", "")
        self.declare_parameter("oracle_rewrite_frame", True)

        self.declare_parameter("roi_min_x", 0.5)
        self.declare_parameter("roi_max_x", 30.0)
        self.declare_parameter("roi_abs_y", 15.0)
        self.declare_parameter("roi_min_z", -0.2)
        self.declare_parameter("roi_max_z", 1.5)
        self.declare_parameter("ground_min_z", 0.05)
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

        self.declare_parameter("min_bbox_probability", 0.0)
        self.declare_parameter("min_projected_points", 1)
        self.declare_parameter("min_project_depth", 0.2)

        self.declare_parameter("sparse_association_enabled", True)
        self.declare_parameter("sparse_bbox_margin_px", 4.0)
        self.declare_parameter("sparse_bbox_margin_ratio", 0.15)
        self.declare_parameter("sparse_near_range_m", 6.0)
        self.declare_parameter("sparse_far_range_m", 12.0)
        self.declare_parameter("sparse_near_min_points", 4)
        self.declare_parameter("sparse_mid_min_points", 2)
        self.declare_parameter("sparse_far_min_points", 2)
        self.declare_parameter("sparse_far_min_probability", 0.25)
        self.declare_parameter("sparse_far_min_bbox_width_px", 4.0)
        self.declare_parameter("sparse_far_min_bbox_height_px", 4.0)
        self.declare_parameter("sparse_max_depth_span_m", 1.0)
        self.declare_parameter("sparse_max_depth_span_ratio", 0.35)
        self.declare_parameter("sparse_max_width", 0.90)
        self.declare_parameter("sparse_variance_x", 0.12)
        self.declare_parameter("sparse_variance_y", 0.12)

        self.declare_parameter("monocular_fallback_enabled", True)
        self.declare_parameter("monocular_depth_coefficient", 0.498)
        self.declare_parameter("monocular_depth_exponent", -0.954)
        self.declare_parameter("monocular_min_depth_m", 0.5)
        self.declare_parameter("monocular_max_depth_m", 30.0)
        self.declare_parameter("good_cone_min_height_to_width", 1.2)
        self.declare_parameter("good_cone_border_margin_ratio", 0.01)
        self.declare_parameter("monocular_variance_x", 0.35)
        self.declare_parameter("monocular_variance_y", 0.35)

        # Direct execution defaults to the simulator's right-camera bbox
        # contract, which has no left-view bbox for stereo correspondence.
        # The YOLO launch path explicitly enables stereo with left-view boxes.
        self.declare_parameter("stereo_fallback_enabled", False)
        self.declare_parameter("stereo_baseline_m", 0.12)
        self.declare_parameter("stereo_min_depth_m", 0.5)
        self.declare_parameter("stereo_max_depth_m", 30.0)
        self.declare_parameter("stereo_epipolar_tolerance_px", 2.0)
        self.declare_parameter("stereo_slender_fraction", 0.5)
        self.declare_parameter("stereo_ratio_threshold", 0.75)
        self.declare_parameter("stereo_min_matches", 3)
        self.declare_parameter("stereo_variance_x", 0.50)
        self.declare_parameter("stereo_variance_y", 0.50)

        self.declare_parameter("default_variance_x", 0.04)
        self.declare_parameter("default_variance_y", 0.04)
        self.declare_parameter("fused_variance_x", 0.04)
        self.declare_parameter("fused_variance_y", 0.04)
        self.declare_parameter("range_variance_scale", 0.0005)
        self.declare_parameter("min_variance", 1.0e-4)

    def _load_parameters(self) -> None:
        self.image_topic = self.get_parameter("image_topic").value
        self.right_image_topic = self.get_parameter("right_image_topic").value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.right_camera_info_topic = self.get_parameter(
            "right_camera_info_topic"
        ).value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.right_camera_frame = self.get_parameter("right_camera_frame").value
        self.projection_model = self.get_parameter("projection_model").value
        self.clip_bboxes_to_image = self._as_bool(
            self.get_parameter("clip_bboxes_to_image").value
        )
        self.clip_projected_points_to_image = self._as_bool(
            self.get_parameter("clip_projected_points_to_image").value
        )
        self.output_cones_topic = self.get_parameter("output_cones_topic").value
        self.output_frame = self.get_parameter("output_frame").value
        self.motion_compensation_frame = str(
            self.get_parameter("motion_compensation_frame").value
        ).strip()
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.sync_tolerance_sec = float(self.get_parameter("sync_tolerance_sec").value)
        self.image_sync_tolerance_sec = float(
            self.get_parameter("image_sync_tolerance_sec").value
        )
        self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self.image_sync_queue_size = int(
            self.get_parameter("image_sync_queue_size").value
        )
        self.timestamp_reset_threshold_sec = float(
            self.get_parameter("timestamp_reset_threshold_sec").value
        )
        self.max_future_stamp_lead_sec = float(
            self.get_parameter("max_future_stamp_lead_sec").value
        )
        self.publish_empty_on_sync = self._as_bool(
            self.get_parameter("publish_empty_on_sync").value
        )
        self.fusion_enabled = self._as_bool(self.get_parameter("fusion_enabled").value)
        self.publish_fusion_debug = self._as_bool(
            self.get_parameter("publish_fusion_debug").value
        )
        self.fusion_debug_prefix = str(
            self.get_parameter("fusion_debug_prefix").value
        ).rstrip("/")
        self.oracle_cones_topic = self.get_parameter("oracle_cones_topic").value
        self.oracle_rewrite_frame = self._as_bool(
            self.get_parameter("oracle_rewrite_frame").value
        )

        self.roi_min_x = float(self.get_parameter("roi_min_x").value)
        self.roi_max_x = float(self.get_parameter("roi_max_x").value)
        self.roi_abs_y = float(self.get_parameter("roi_abs_y").value)
        self.roi_min_z = float(self.get_parameter("roi_min_z").value)
        self.roi_max_z = float(self.get_parameter("roi_max_z").value)
        self.ground_min_z = float(self.get_parameter("ground_min_z").value)
        self.ground_ransac_enabled = self._as_bool(
            self.get_parameter("ground_ransac_enabled").value
        )
        self.ground_ransac_distance_threshold = float(
            self.get_parameter("ground_ransac_distance_threshold").value
        )
        self.ground_ransac_max_iterations = int(
            self.get_parameter("ground_ransac_max_iterations").value
        )
        self.ground_ransac_max_tilt_degrees = float(
            self.get_parameter("ground_ransac_max_tilt_degrees").value
        )
        self.ground_ransac_min_inliers = int(
            self.get_parameter("ground_ransac_min_inliers").value
        )
        self.ground_ransac_seed = int(
            self.get_parameter("ground_ransac_seed").value
        )

        self.self_mask_enabled = self._as_bool(
            self.get_parameter("self_mask_enabled").value
        )
        self.self_mask_min_x = float(self.get_parameter("self_mask_min_x").value)
        self.self_mask_max_x = float(self.get_parameter("self_mask_max_x").value)
        self.self_mask_abs_y = float(self.get_parameter("self_mask_abs_y").value)
        self.self_mask_min_z = float(self.get_parameter("self_mask_min_z").value)
        self.self_mask_max_z = float(self.get_parameter("self_mask_max_z").value)

        self.cluster_eps = float(self.get_parameter("cluster_eps").value)
        self.cluster_min_points = int(self.get_parameter("cluster_min_points").value)
        self.cluster_min_height = float(self.get_parameter("cluster_min_height").value)
        self.cluster_max_height = float(self.get_parameter("cluster_max_height").value)
        self.cluster_max_width = float(self.get_parameter("cluster_max_width").value)

        self.min_bbox_probability = float(self.get_parameter("min_bbox_probability").value)
        self.min_projected_points = int(self.get_parameter("min_projected_points").value)
        self.min_project_depth = float(self.get_parameter("min_project_depth").value)

        self.sparse_association_enabled = self._as_bool(
            self.get_parameter("sparse_association_enabled").value
        )
        self.sparse_bbox_margin_px = float(
            self.get_parameter("sparse_bbox_margin_px").value
        )
        self.sparse_bbox_margin_ratio = float(
            self.get_parameter("sparse_bbox_margin_ratio").value
        )
        self.sparse_near_range_m = float(
            self.get_parameter("sparse_near_range_m").value
        )
        self.sparse_far_range_m = float(
            self.get_parameter("sparse_far_range_m").value
        )
        self.sparse_near_min_points = int(
            self.get_parameter("sparse_near_min_points").value
        )
        self.sparse_mid_min_points = int(
            self.get_parameter("sparse_mid_min_points").value
        )
        self.sparse_far_min_points = int(
            self.get_parameter("sparse_far_min_points").value
        )
        self.sparse_far_min_probability = float(
            self.get_parameter("sparse_far_min_probability").value
        )
        self.sparse_far_min_bbox_width_px = float(
            self.get_parameter("sparse_far_min_bbox_width_px").value
        )
        self.sparse_far_min_bbox_height_px = float(
            self.get_parameter("sparse_far_min_bbox_height_px").value
        )
        self.sparse_max_depth_span_m = float(
            self.get_parameter("sparse_max_depth_span_m").value
        )
        self.sparse_max_depth_span_ratio = float(
            self.get_parameter("sparse_max_depth_span_ratio").value
        )
        self.sparse_max_width = float(self.get_parameter("sparse_max_width").value)
        self.sparse_variance_x = float(self.get_parameter("sparse_variance_x").value)
        self.sparse_variance_y = float(self.get_parameter("sparse_variance_y").value)

        self.monocular_fallback_enabled = self._as_bool(
            self.get_parameter("monocular_fallback_enabled").value
        )
        self.monocular_depth_coefficient = float(
            self.get_parameter("monocular_depth_coefficient").value
        )
        self.monocular_depth_exponent = float(
            self.get_parameter("monocular_depth_exponent").value
        )
        self.monocular_min_depth_m = float(
            self.get_parameter("monocular_min_depth_m").value
        )
        self.monocular_max_depth_m = float(
            self.get_parameter("monocular_max_depth_m").value
        )
        self.good_cone_min_height_to_width = float(
            self.get_parameter("good_cone_min_height_to_width").value
        )
        self.good_cone_border_margin_ratio = float(
            self.get_parameter("good_cone_border_margin_ratio").value
        )
        self.monocular_variance_x = float(
            self.get_parameter("monocular_variance_x").value
        )
        self.monocular_variance_y = float(
            self.get_parameter("monocular_variance_y").value
        )

        self.stereo_fallback_enabled = self._as_bool(
            self.get_parameter("stereo_fallback_enabled").value
        )
        self.stereo_baseline_m = float(
            self.get_parameter("stereo_baseline_m").value
        )
        self.stereo_min_depth_m = float(
            self.get_parameter("stereo_min_depth_m").value
        )
        self.stereo_max_depth_m = float(
            self.get_parameter("stereo_max_depth_m").value
        )
        self.stereo_epipolar_tolerance_px = float(
            self.get_parameter("stereo_epipolar_tolerance_px").value
        )
        self.stereo_slender_fraction = float(
            self.get_parameter("stereo_slender_fraction").value
        )
        self.stereo_ratio_threshold = float(
            self.get_parameter("stereo_ratio_threshold").value
        )
        self.stereo_min_matches = int(
            self.get_parameter("stereo_min_matches").value
        )
        self.stereo_variance_x = float(
            self.get_parameter("stereo_variance_x").value
        )
        self.stereo_variance_y = float(
            self.get_parameter("stereo_variance_y").value
        )

        self.default_variance_x = float(self.get_parameter("default_variance_x").value)
        self.default_variance_y = float(self.get_parameter("default_variance_y").value)
        self.fused_variance_x = float(self.get_parameter("fused_variance_x").value)
        self.fused_variance_y = float(self.get_parameter("fused_variance_y").value)
        self.range_variance_scale = float(self.get_parameter("range_variance_scale").value)
        self.min_variance = float(self.get_parameter("min_variance").value)
        self._validate_parameters()

    def _validate_parameters(self) -> None:
        if self.projection_model not in ("pinhole", "eufs_bbox"):
            raise ValueError(
                "projection_model must be 'pinhole' or 'eufs_bbox', got "
                f"{self.projection_model!r}"
            )
        if self.sync_queue_size <= 0:
            raise ValueError("sync_queue_size must be greater than zero")
        if self.image_sync_queue_size <= 0:
            raise ValueError("image_sync_queue_size must be greater than zero")
        self._validate_time_parameters()
        if self.sync_tolerance_sec < 0.0 or self.image_sync_tolerance_sec < 0.0:
            raise ValueError("synchronization tolerances must not be negative")
        if not self.camera_frame:
            raise ValueError("camera_frame must not be empty")
        if not self.motion_compensation_frame:
            raise ValueError("motion_compensation_frame must not be empty")
        if self.stereo_fallback_enabled and not self.right_camera_frame:
            raise ValueError("right_camera_frame must not be empty when stereo is enabled")
        self._validate_stereo_capability()
        if not (self.roi_min_x < self.roi_max_x):
            raise ValueError("roi_min_x must be less than roi_max_x")
        if not (self.roi_min_z < self.roi_max_z):
            raise ValueError("roi_min_z must be less than roi_max_z")
        if self.roi_abs_y <= 0.0:
            raise ValueError("roi_abs_y must be greater than zero")
        if self.ground_ransac_distance_threshold <= 0.0:
            raise ValueError("ground_ransac_distance_threshold must be positive")
        if self.ground_ransac_max_iterations <= 0:
            raise ValueError("ground_ransac_max_iterations must be positive")
        if not 0.0 <= self.ground_ransac_max_tilt_degrees < 90.0:
            raise ValueError("ground_ransac_max_tilt_degrees must be in [0, 90)")
        if self.ground_ransac_min_inliers < 3:
            raise ValueError("ground_ransac_min_inliers must be at least 3")
        if self.cluster_eps <= 0.0 or self.cluster_min_points <= 0:
            raise ValueError("cluster_eps and cluster_min_points must be positive")
        if self.min_projected_points <= 0 or self.min_project_depth <= 0.0:
            raise ValueError("projection support thresholds must be positive")
        if self.monocular_depth_coefficient <= 0.0:
            raise ValueError("monocular_depth_coefficient must be positive")
        if not 0.0 < self.good_cone_min_height_to_width:
            raise ValueError("good_cone_min_height_to_width must be positive")
        if not 0.0 <= self.good_cone_border_margin_ratio < 0.5:
            raise ValueError("good_cone_border_margin_ratio must be in [0, 0.5)")
        if not 0.0 < self.stereo_slender_fraction <= 1.0:
            raise ValueError("stereo_slender_fraction must be in (0, 1]")
        if not 0.0 < self.stereo_ratio_threshold < 1.0:
            raise ValueError("stereo_ratio_threshold must be in (0, 1)")
        if self.stereo_min_matches <= 0:
            raise ValueError("stereo_min_matches must be positive")
        for name, minimum, maximum in (
            (
                "monocular depth",
                self.monocular_min_depth_m,
                self.monocular_max_depth_m,
            ),
            ("stereo depth", self.stereo_min_depth_m, self.stereo_max_depth_m),
        ):
            if minimum <= 0.0 or maximum <= minimum:
                raise ValueError(f"{name} range must satisfy 0 < min < max")

    def _validate_time_parameters(self) -> None:
        if (
            not math.isfinite(self.timestamp_reset_threshold_sec)
            or self.timestamp_reset_threshold_sec <= 0.0
        ):
            raise ValueError(
                "timestamp_reset_threshold_sec must be finite, positive, and "
                "representable as an int64 nanosecond duration"
            )
        if (
            not math.isfinite(self.max_future_stamp_lead_sec)
            or self.max_future_stamp_lead_sec <= 0.0
        ):
            raise ValueError(
                "max_future_stamp_lead_sec must be finite, positive, and "
                "representable as an int64 nanosecond duration"
            )
        try:
            reset_threshold_ns = int(
                self.timestamp_reset_threshold_sec * 1.0e9
            )
        except (OverflowError, ValueError):
            raise ValueError(
                "timestamp_reset_threshold_sec must convert to a positive "
                "int64 nanosecond duration"
            ) from None
        try:
            max_future_ns = int(self.max_future_stamp_lead_sec * 1.0e9)
        except (OverflowError, ValueError):
            raise ValueError(
                "max_future_stamp_lead_sec must convert to a positive int64 "
                "nanosecond duration"
            ) from None
        max_duration_ns = (1 << 63) - 1
        if reset_threshold_ns <= 0 or reset_threshold_ns > max_duration_ns:
            raise ValueError(
                "timestamp_reset_threshold_sec must convert to a positive "
                "int64 nanosecond duration"
            )
        if max_future_ns <= 0 or max_future_ns > max_duration_ns:
            raise ValueError(
                "max_future_stamp_lead_sec must convert to a positive int64 "
                "nanosecond duration"
            )
        if max_future_ns >= reset_threshold_ns:
            raise ValueError(
                "max_future_stamp_lead_sec must be smaller than "
                "timestamp_reset_threshold_sec to bound previous-epoch "
                "watermark contamination after a rollback"
            )
        self.timestamp_reset_threshold_ns = reset_threshold_ns
        self.max_future_stamp_lead_ns = max_future_ns

    def _validate_stereo_capability(self) -> None:
        if self.stereo_fallback_enabled and not _sift_available():
            raise RuntimeError(
                "stereo_fallback_enabled requires an OpenCV build that provides "
                "cv2.SIFT_create; use the project conda environment, install a "
                "SIFT-capable OpenCV build, or explicitly disable the Tier-3 fallback"
            )

    def _validate_camera_info(
        self,
        camera_info: CameraInfo,
        image: Optional[Image] = None,
        expected_frame: Optional[str] = None,
    ) -> None:
        if int(camera_info.width) <= 0 or int(camera_info.height) <= 0:
            raise RuntimeError("CameraInfo width and height must be positive")
        matrix = self._camera_matrix(camera_info)
        if (
            matrix.shape != (3, 3)
            or not np.all(np.isfinite(matrix))
            or matrix[0, 0] <= 0.0
            or matrix[1, 1] <= 0.0
            or abs(float(np.linalg.det(matrix))) <= 1.0e-12
        ):
            raise RuntimeError("CameraInfo does not contain a usable K/P matrix")
        if image is not None and (
            int(image.width) != int(camera_info.width)
            or int(image.height) != int(camera_info.height)
        ):
            raise RuntimeError(
                "Image and CameraInfo dimensions differ: "
                f"image={image.width}x{image.height}, "
                f"camera_info={camera_info.width}x{camera_info.height}"
            )
        if image is not None:
            image_frame = str(image.header.frame_id).strip()
            info_frame = str(camera_info.header.frame_id).strip()
            if image_frame and info_frame and image_frame != info_frame:
                raise RuntimeError(
                    "Image and CameraInfo frame_id differ: "
                    f"image={image_frame!r}, camera_info={info_frame!r}"
                )
            self._validate_camera_frame(
                image_frame,
                expected_frame,
                "Image",
            )
        info_frame = str(camera_info.header.frame_id).strip()
        expected_frame = str(expected_frame or "").strip()
        if info_frame and expected_frame and info_frame != expected_frame:
            raise RuntimeError(
                "CameraInfo frame_id differs from configured camera frame: "
                f"camera_info={info_frame!r}, configured={expected_frame!r}"
            )

    @staticmethod
    def _validate_camera_frame(
        frame_id: str,
        expected_frame: Optional[str],
        source_name: str,
    ) -> None:
        actual = str(frame_id or "").strip()
        expected = str(expected_frame or "").strip()
        if actual and expected and actual != expected:
            raise RuntimeError(
                f"{source_name} frame_id differs from configured camera frame: "
                f"{source_name.lower()}={actual!r}, configured={expected!r}"
            )

    @staticmethod
    def _camera_matrix(camera_info: CameraInfo) -> np.ndarray:
        projection = np.asarray(camera_info.p, dtype=np.float64)
        if projection.size == 12:
            projection = projection.reshape(3, 4)
            candidate = projection[:, :3]
            if (
                np.all(np.isfinite(candidate))
                and candidate[0, 0] > 0.0
                and candidate[1, 1] > 0.0
                and abs(float(np.linalg.det(candidate))) > 1.0e-12
            ):
                return candidate.copy()
        intrinsic = np.asarray(camera_info.k, dtype=np.float64)
        if intrinsic.size != 9:
            return np.full((3, 3), np.nan, dtype=np.float64)
        return intrinsic.reshape(3, 3).copy()

    def _image_callback(self, msg: Image) -> None:
        stamp_ns = self._prepare_sensor_message_stamp(
            "left_image", msg.header.stamp
        )
        if stamp_ns is None:
            return
        self.latest_image = msg
        self.image_buffer.add(stamp_ns, msg)
        self._try_publish_empty_from_sensor_pair()
        self._try_publish_fusion()

    def _right_image_callback(self, msg: Image) -> None:
        stamp_ns = self._prepare_sensor_message_stamp(
            "right_image", msg.header.stamp
        )
        if stamp_ns is None:
            return
        self.latest_right_image = msg
        self.right_image_buffer.add(stamp_ns, msg)
        self._try_publish_fusion()

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        stamp_ns = self._prepare_sensor_message_stamp(
            "pointcloud", msg.header.stamp
        )
        if stamp_ns is None:
            return
        self.latest_pointcloud = msg
        self.pointcloud_buffer.add(stamp_ns, msg)
        self._try_publish_empty_from_sensor_pair()
        self._try_publish_fusion()

    def _bbox_callback(self, msg: BoundingBoxes) -> None:
        stamp_ns = self._prepare_sensor_message_stamp(
            "bboxes", self._bounding_boxes_stamp(msg)
        )
        if stamp_ns is None:
            return
        self.latest_bboxes = msg
        self.bbox_buffer.add(stamp_ns, msg)
        self._try_publish_fusion()

    def _clear_fusion_epoch(
        self,
        replay_guard_end_ros_time_ns: Optional[int] = None,
        epoch_start_ros_time_ns: Optional[int] = None,
        fallback_rollback: bool = False,
        preserve_replay_guards: bool = False,
    ) -> None:
        """Discard all cross-sensor state at an authoritative clock epoch change."""
        self.image_buffer.clear()
        self.right_image_buffer.clear()
        self.pointcloud_buffer.clear()
        self.bbox_buffer.clear()
        self.latest_stream_stamp_ns.clear()
        self.latest_image = None
        self.latest_right_image = None
        self.latest_pointcloud = None
        self.latest_bboxes = None
        self.last_empty_key = None
        self.last_fusion_key = None
        if epoch_start_ros_time_ns is None:
            epoch_start_ns = self.get_clock().now().nanoseconds
        else:
            epoch_start_ns = int(epoch_start_ros_time_ns)
        replay_guard_end_ns = self._valid_replay_guard_end(
            epoch_start_ns,
            replay_guard_end_ros_time_ns,
        )
        replay_guard_ends = []
        if preserve_replay_guards:
            replay_guard_ends.extend(
                guard_end_ns
                for guard_end_ns in getattr(
                    self,
                    "replay_guard_ends_ros_time_ns",
                    [],
                )
                if guard_end_ns > epoch_start_ns
            )
            existing_guard_end_ns = getattr(
                self,
                "replay_guard_end_ros_time_ns",
                None,
            )
            if (
                existing_guard_end_ns is not None
                and existing_guard_end_ns > epoch_start_ns
            ):
                replay_guard_ends.append(existing_guard_end_ns)
        if replay_guard_end_ns is not None:
            replay_guard_ends.append(replay_guard_end_ns)
        replay_guard_ends = sorted(set(replay_guard_ends))
        self.epoch_start_ros_time_ns = epoch_start_ns
        self.replay_guard_ends_ros_time_ns = replay_guard_ends
        self.replay_guard_end_ros_time_ns = (
            replay_guard_ends[0] if replay_guard_ends else None
        )
        self._pending_fallback_rollback = (
            (epoch_start_ns, replay_guard_end_ns)
            if fallback_rollback and replay_guard_end_ns is not None
            else None
        )
        self.last_observed_ros_time_ns = epoch_start_ns
        self._reset_tf_epoch()
        self.sensor_epoch += 1

    def _reset_tf_epoch(self) -> None:
        """Recreate the TF subscription while retaining cached static transforms."""
        old_listener = self.tf_listener
        listener_unregistered = False
        try:
            old_listener.unregister()
            listener_unregistered = True
        except Exception as exc:  # pragma: no cover - rclpy destruction failure
            self._warn_throttled(
                "tf_listener_reset",
                f"Old TF listener could not be unregistered after epoch reset: {exc}",
                period_sec=1.0,
            )
        old_buffer = self.tf_buffer
        old_buffer.clear()
        # Buffer.clear() retains static TF in Galactic. Reusing that buffer
        # therefore supports rosbag/one-shot static publishers that have
        # already exited, while listener recreation flushes queued volatile
        # /tf. If unregister failed, isolate the still-active old subscription
        # with a fresh buffer instead of allowing it to repopulate live state.
        new_buffer = old_buffer if listener_unregistered else Buffer()
        new_listener = TransformListener(new_buffer, self)
        self.tf_buffer = new_buffer
        self.tf_listener = new_listener

    def _on_clock_jump(self, time_jump) -> None:
        """Start a new epoch only for a source transition or real rollback."""
        if not self._is_authoritative_clock_epoch_change(time_jump):
            return

        new_now_ns = int(self.get_clock().now().nanoseconds)
        clock_change = getattr(time_jump, "clock_change", None)
        clock_source_changed = clock_change in (
            ClockChange.ROS_TIME_ACTIVATED,
            ClockChange.ROS_TIME_DEACTIVATED,
        )
        delta_ns = self._clock_jump_delta_ns(time_jump)
        replay_guard_end_ns = None
        if not clock_source_changed and delta_ns is not None:
            replay_guard_end_ns = self._rollback_replay_end_ns(
                new_now_ns,
                delta_ns,
            )

        if self._matches_pending_fallback_rollback(
            new_now_ns,
            delta_ns,
            clock_source_changed,
        ):
            self._pending_fallback_rollback = None
            return

        self._clear_fusion_epoch(
            replay_guard_end_ros_time_ns=replay_guard_end_ns,
            epoch_start_ros_time_ns=new_now_ns,
            preserve_replay_guards=not clock_source_changed,
        )
        self._warn_throttled(
            "sensor_time_reset",
            "ROS clock changed or moved backward; cleared fusion buffers for "
            f"sensor epoch {self.sensor_epoch}",
            period_sec=1.0,
        )

    def _is_authoritative_clock_epoch_change(self, time_jump) -> bool:
        """Defensively filter Galactic callbacks that include forward ticks."""
        if time_jump is None:
            return False

        clock_change = getattr(time_jump, "clock_change", None)
        if clock_change in (
            ClockChange.ROS_TIME_ACTIVATED,
            ClockChange.ROS_TIME_DEACTIVATED,
        ):
            return True

        delta_ns = self._clock_jump_delta_ns(time_jump)
        if delta_ns is None:
            return False

        return delta_ns <= -self.timestamp_reset_threshold_ns

    @staticmethod
    def _clock_jump_delta_ns(time_jump) -> Optional[int]:
        delta = getattr(time_jump, "delta", None)
        delta_ns = getattr(delta, "nanoseconds", None)
        if delta_ns is None:
            return None
        try:
            delta_ns = int(delta_ns)
        except (TypeError, ValueError, OverflowError):
            return None
        if delta_ns < INT64_MIN or delta_ns > INT64_MAX:
            return None
        return delta_ns

    @staticmethod
    def _rollback_replay_end_ns(
        epoch_start_ns: int,
        clock_delta_ns: int,
    ) -> Optional[int]:
        """Recover the pre-rollback high watermark without int64 overflow."""
        try:
            epoch_start_ns = int(epoch_start_ns)
            clock_delta_ns = int(clock_delta_ns)
        except (TypeError, ValueError, OverflowError):
            return None
        if (
            epoch_start_ns < INT64_MIN
            or epoch_start_ns > INT64_MAX
            or clock_delta_ns < INT64_MIN
            or clock_delta_ns >= 0
        ):
            return None
        if epoch_start_ns > INT64_MAX + clock_delta_ns:
            return INT64_MAX
        return epoch_start_ns - clock_delta_ns

    @staticmethod
    def _valid_replay_guard_end(
        epoch_start_ns: int,
        replay_guard_end_ns: Optional[int],
    ) -> Optional[int]:
        if replay_guard_end_ns is None:
            return None
        try:
            replay_guard_end_ns = int(replay_guard_end_ns)
        except (TypeError, ValueError, OverflowError):
            return None
        if (
            replay_guard_end_ns < INT64_MIN
            or replay_guard_end_ns <= epoch_start_ns
            or replay_guard_end_ns > INT64_MAX
        ):
            return None
        return replay_guard_end_ns

    def _matches_pending_fallback_rollback(
        self,
        new_now_ns: int,
        delta_ns: Optional[int],
        clock_source_changed: bool,
    ) -> bool:
        """Suppress the jump callback when the input fallback handled it first."""
        pending = getattr(self, "_pending_fallback_rollback", None)
        if pending is None or clock_source_changed or delta_ns is None:
            return False
        epoch_start_ns, replay_guard_end_ns = pending
        expected_delta_ns = epoch_start_ns - replay_guard_end_ns
        return delta_ns == expected_delta_ns and new_now_ns == epoch_start_ns

    def _prepare_sensor_message_stamp(self, stream_name: str, stamp) -> Optional[int]:
        """Validate a raw ROS Time before converting it to integer nanoseconds."""
        if not self._is_canonical_nonzero_stamp(stamp):
            self._warn_throttled(
                f"invalid_stamp_{stream_name}",
                f"Dropping {stream_name} with a non-canonical or zero ROS timestamp",
                period_sec=1.0,
            )
            return None
        stamp_ns = self._stamp_to_ns(stamp)
        if not self._prepare_sensor_stamp(stream_name, stamp_ns):
            return None
        return stamp_ns

    def _prepare_sensor_stamp(self, stream_name: str, stamp_ns: int) -> bool:
        """Accept in-epoch stamps while dropping stale or implausibly future data."""
        clock = self.get_clock()
        now_ns = clock.now().nanoseconds
        last_now_ns = self.last_observed_ros_time_ns
        if (
            last_now_ns is not None
            and now_ns <= last_now_ns - self.timestamp_reset_threshold_ns
        ):
            self._clear_fusion_epoch(
                replay_guard_end_ros_time_ns=last_now_ns,
                epoch_start_ros_time_ns=now_ns,
                fallback_rollback=True,
                preserve_replay_guards=True,
            )
            now_ns = self.epoch_start_ros_time_ns
            self._warn_throttled(
                "observed_ros_time_reset",
                "Observed the node clock move backward while processing input; "
                f"started sensor epoch {self.sensor_epoch}",
                period_sec=1.0,
            )
        elif last_now_ns is None or now_ns > last_now_ns:
            self.last_observed_ros_time_ns = now_ns
        ros_time_is_active = bool(
            getattr(clock, "ros_time_is_active", False)
        )
        replay_guard_end_ns = getattr(
            self,
            "replay_guard_end_ros_time_ns",
            None,
        )
        replay_guard_ends = list(
            getattr(self, "replay_guard_ends_ros_time_ns", [])
        )
        if (
            replay_guard_end_ns is not None
            and replay_guard_end_ns not in replay_guard_ends
        ):
            replay_guard_ends.append(replay_guard_end_ns)
        replay_guard_ends = sorted(
            set(
                guard_end_ns
                for guard_end_ns in replay_guard_ends
                if guard_end_ns > now_ns
            )
        )
        self.replay_guard_ends_ros_time_ns = replay_guard_ends
        self.replay_guard_end_ros_time_ns = (
            replay_guard_ends[0] if replay_guard_ends else None
        )
        replay_guard_end_ns = self.replay_guard_end_ros_time_ns
        if replay_guard_end_ns is None:
            self._pending_fallback_rollback = None
        replaying_rollback_interval = replay_guard_end_ns is not None
        max_future_ns = self.max_future_stamp_lead_ns
        if stamp_ns <= 0:
            self._warn_throttled(
                f"invalid_stamp_{stream_name}",
                f"Dropping {stream_name} with non-positive timestamp {stamp_ns}",
                period_sec=1.0,
            )
            return False
        epoch_start_ns = self.epoch_start_ros_time_ns
        if epoch_start_ns is not None and stamp_ns < epoch_start_ns:
            self._warn_throttled(
                f"previous_epoch_{stream_name}",
                f"Dropping {stream_name} timestamp {stamp_ns} before sensor "
                f"epoch {self.sensor_epoch} start {epoch_start_ns}",
                period_sec=1.0,
            )
            return False
        if (
            replaying_rollback_interval
            and stamp_ns >= replay_guard_end_ns
        ):
            self._warn_throttled(
                f"replay_guard_{stream_name}",
                f"Dropping {stream_name} timestamp {stamp_ns} at or beyond "
                f"the pre-rollback clock watermark {replay_guard_end_ns}",
                period_sec=1.0,
            )
            return False
        if (
            (replaying_rollback_interval or ros_time_is_active or now_ns > 0)
            and stamp_ns > now_ns + max_future_ns
        ):
            self._warn_throttled(
                f"future_{stream_name}",
                f"Dropping {stream_name} timestamp {stamp_ns} more than "
                f"{max_future_ns * 1.0e-9:.3f} s ahead of ROS time"
                + (
                    " while replaying a rolled-back clock interval"
                    if replaying_rollback_interval
                    else ""
                ),
                period_sec=1.0,
            )
            return False

        previous = self.latest_stream_stamp_ns.get(stream_name)
        if previous is not None and stamp_ns <= previous:
            self._warn_throttled(
                f"stale_{stream_name}",
                f"Dropping duplicate or out-of-order {stream_name} message within sensor "
                f"epoch {self.sensor_epoch}",
                period_sec=1.0,
            )
            return False

        if previous is None or stamp_ns > previous:
            self.latest_stream_stamp_ns[stream_name] = stamp_ns
        return True

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg
        self._try_publish_fusion()

    def _right_camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_right_camera_info = msg
        self._try_publish_fusion()

    def _try_publish_empty_from_sensor_pair(self) -> None:
        if not self.publish_empty_on_sync or self.fusion_enabled:
            return
        if self.latest_image is None or self.latest_pointcloud is None:
            return

        image_time = self._stamp_to_sec(self.latest_image.header.stamp)
        cloud_time = self._stamp_to_sec(self.latest_pointcloud.header.stamp)
        if abs(image_time - cloud_time) > self.sync_tolerance_sec:
            return

        key = (
            self._stamp_key(self.latest_image.header.stamp),
            self._stamp_key(self.latest_pointcloud.header.stamp),
        )
        if key == self.last_empty_key:
            return
        self.last_empty_key = key

        msg = ConeArrayWithCovariance()
        msg.header.stamp = self.latest_pointcloud.header.stamp
        msg.header.frame_id = self.output_frame
        self._publish_cones(msg)

    def _try_publish_fusion(self) -> None:
        if not self.fusion_enabled:
            return
        missing_inputs = []
        if len(self.pointcloud_buffer) == 0:
            missing_inputs.append(f"pointcloud={self.pointcloud_topic}")
        if len(self.bbox_buffer) == 0:
            missing_inputs.append(f"bboxes={self.bbox_topic}")
        if self.latest_camera_info is None:
            missing_inputs.append(f"camera_info={self.camera_info_topic}")
        if missing_inputs:
            self._warn_throttled(
                "missing_fusion_inputs",
                "Fusion waiting for inputs: " + ", ".join(missing_inputs),
                period_sec=5.0,
            )
            return

        tolerance_ns = int(round(self.sync_tolerance_sec * 1.0e9))
        image_tolerance_ns = int(round(self.image_sync_tolerance_sec * 1.0e9))
        selected = self._select_ready_fusion_pair(tolerance_ns)

        if selected is None:
            self._warn_throttled(
                "fusion_sync_mismatch",
                "Fusion waiting for a one-to-one bbox/LiDAR pair within "
                f"{self.sync_tolerance_sec:.3f}s",
                period_sec=2.0,
            )
            return

        bbox_entry, cloud_entry = selected
        pointcloud = cloud_entry.value
        bboxes = bbox_entry.value
        cloud_stamp = pointcloud.header.stamp
        bbox_stamp = self._bounding_boxes_stamp(bboxes)
        key = (self._stamp_key(cloud_stamp), self._stamp_key(bbox_stamp))
        if key == self.last_fusion_key:
            self.bbox_buffer.remove(bbox_entry.stamp_ns)
            self.pointcloud_buffer.remove(cloud_entry.stamp_ns)
            return

        left_entry = self.image_buffer.nearest(
            bbox_entry.stamp_ns,
            image_tolerance_ns,
        )
        right_entry = self.right_image_buffer.nearest(
            bbox_entry.stamp_ns,
            image_tolerance_ns,
        )
        if (
            left_entry is not None
            and right_entry is not None
            and abs(left_entry.stamp_ns - right_entry.stamp_ns)
            > image_tolerance_ns
        ):
            # Each image may independently be close to the bbox while the
            # stereo pair itself is too far apart.  Disable only the stereo
            # tier for this bundle; LiDAR and monocular paths remain valid.
            right_entry = None

        try:
            cones = self._run_lidar_camera_fusion(
                pointcloud,
                bboxes,
                self.latest_camera_info,
                left_image=left_entry.value if left_entry is not None else None,
                right_image=right_entry.value if right_entry is not None else None,
                right_camera_info=self.latest_right_camera_info,
            )
        except TransformException as exc:
            self._warn_throttled("tf", f"Fusion skipped: missing TF ({exc})")
            if self._drop_failed_pair_if_superseded(
                bbox_entry,
                cloud_entry,
                tolerance_ns,
            ):
                self._try_publish_fusion()
            return
        except RuntimeError as exc:
            self._warn_throttled("fusion", f"Fusion skipped: {exc}")
            if self._drop_failed_pair_if_superseded(
                bbox_entry,
                cloud_entry,
                tolerance_ns,
            ):
                self._try_publish_fusion()
            return

        # Commit synchronization state only after successful fusion.  This
        # keeps a transient TF/calibration failure retryable and guarantees
        # that neither bbox nor cloud is reused in another output.
        self.bbox_buffer.remove(bbox_entry.stamp_ns)
        self.pointcloud_buffer.remove(cloud_entry.stamp_ns)
        self.last_fusion_key = key
        self._info_throttled(
            "fusion_success",
            "Fusion published cones: " + self._cone_count_summary(cones),
            period_sec=2.0,
        )
        self._publish_cones(cones)

    def _select_ready_fusion_pair(self, tolerance_ns: int):
        """
        Match the oldest later-front sensor frame to its nearest counterpart.

        The stream whose queue front is later becomes the anchor. The other
        stream must reach that timestamp before its predecessor/successor pair
        is complete. This avoids an old high-rate bbox consuming a scarce
        LiDAR frame, while remaining symmetric if detector output is slower
        than LiDAR. Skipped high-rate samples are discarded one-to-one.
        """
        while len(self.bbox_buffer) > 0 and len(self.pointcloud_buffer) > 0:
            bbox_entries = self.bbox_buffer.entries()
            cloud_entries = self.pointcloud_buffer.entries()
            bbox_entry = bbox_entries[0]
            cloud_entry = cloud_entries[0]
            if bbox_entry.stamp_ns == cloud_entry.stamp_ns:
                return bbox_entry, cloud_entry

            if bbox_entry.stamp_ns < cloud_entry.stamp_ns:
                if self._discard_entries_before(
                    self.bbox_buffer,
                    cloud_entry.stamp_ns - tolerance_ns,
                ):
                    continue
                bbox_entries = self.bbox_buffer.entries()
                if not bbox_entries or bbox_entries[-1].stamp_ns < cloud_entry.stamp_ns:
                    return None
                bbox_match = self.bbox_buffer.nearest(
                    cloud_entry.stamp_ns,
                    tolerance_ns,
                )
                if bbox_match is None:
                    self.pointcloud_buffer.remove(cloud_entry.stamp_ns)
                    continue
                self._discard_entries_before(
                    self.bbox_buffer,
                    bbox_match.stamp_ns,
                )
                return bbox_match, cloud_entry

            if self._discard_entries_before(
                self.pointcloud_buffer,
                bbox_entry.stamp_ns - tolerance_ns,
            ):
                continue
            cloud_entries = self.pointcloud_buffer.entries()
            if not cloud_entries or cloud_entries[-1].stamp_ns < bbox_entry.stamp_ns:
                return None
            cloud_match = self.pointcloud_buffer.nearest(
                bbox_entry.stamp_ns,
                tolerance_ns,
            )
            if cloud_match is None:
                self.bbox_buffer.remove(bbox_entry.stamp_ns)
                continue
            self._discard_entries_before(
                self.pointcloud_buffer,
                cloud_match.stamp_ns,
            )
            return bbox_entry, cloud_match
        return None

    def _drop_failed_pair_if_superseded(
        self,
        bbox_entry,
        cloud_entry,
        tolerance_ns: int,
    ) -> bool:
        """Drop a failed pair only when a newer synchronized pair is ready."""
        newer_bboxes = [
            entry
            for entry in self.bbox_buffer.entries()
            if entry.stamp_ns > bbox_entry.stamp_ns
        ]
        newer_clouds = [
            entry
            for entry in self.pointcloud_buffer.entries()
            if entry.stamp_ns > cloud_entry.stamp_ns
        ]
        newer_pair_ready = any(
            abs(new_bbox.stamp_ns - new_cloud.stamp_ns) <= tolerance_ns
            for new_bbox in newer_bboxes
            for new_cloud in newer_clouds
        )
        if not newer_pair_ready:
            return False

        # Keep a lone pair retryable for normal TF callback ordering, but never
        # let an irrecoverable old timestamp head-of-line block fresher data.
        self.bbox_buffer.remove(bbox_entry.stamp_ns)
        self.pointcloud_buffer.remove(cloud_entry.stamp_ns)
        return True

    @staticmethod
    def _discard_entries_before(buffer, cutoff_ns: int) -> bool:
        removed = False
        for entry in buffer.entries():
            if entry.stamp_ns >= cutoff_ns:
                break
            buffer.remove(entry.stamp_ns)
            removed = True
        return removed

    def _run_lidar_camera_fusion(
        self,
        pointcloud: PointCloud2,
        bboxes: BoundingBoxes,
        camera_info: CameraInfo,
        left_image: Optional[Image] = None,
        right_image: Optional[Image] = None,
        right_camera_info: Optional[CameraInfo] = None,
    ) -> ConeArrayWithCovariance:
        self._validate_camera_info(camera_info, left_image, self.camera_frame)
        self._validate_camera_frame(
            bboxes.image_header.frame_id,
            self.camera_frame,
            "BoundingBoxes.image_header",
        )
        detections = self._extract_detections(bboxes, camera_info)
        bbox_stamp = self._bounding_boxes_stamp(bboxes)
        msg = ConeArrayWithCovariance()
        # Every cone in one array uses the bbox acquisition time. LiDAR points
        # are ego-motion compensated from cloud time into this canonical base
        # frame before association, so downstream consumers can trust the
        # single header timestamp for every measurement in the array.
        msg.header.stamp = bbox_stamp
        msg.header.frame_id = self.output_frame

        if not detections:
            self._warn_throttled(
                "no_detections",
                "Fusion produced no cones: no bbox detections after filtering",
            )
            return msg

        if not self.camera_frame:
            raise RuntimeError("camera frame is empty")

        points_lidar = self._pointcloud_to_xyz(pointcloud)
        raw_point_count = int(points_lidar.shape[0])
        points_base_all = np.empty((0, 3), dtype=np.float64)
        points_camera_all = np.empty((0, 3), dtype=np.float64)
        points_base = np.empty((0, 3), dtype=np.float64)
        points_camera = np.empty((0, 3), dtype=np.float64)
        clusters = []
        cluster_assignments = []
        sparse_assignments = []

        if points_lidar.size > 0:
            lidar_frame = pointcloud.header.frame_id
            if not lidar_frame:
                raise RuntimeError("point cloud frame_id is empty")
            lidar_to_base = self._lookup_transform_matrix_between_times(
                self.output_frame,
                bbox_stamp,
                lidar_frame,
                pointcloud.header.stamp,
            )
            lidar_to_camera = self._lookup_transform_matrix_between_times(
                self.camera_frame,
                bbox_stamp,
                lidar_frame,
                pointcloud.header.stamp,
            )
            points_base_all = self._transform_points(points_lidar, lidar_to_base)
            points_camera_all = self._transform_points(points_lidar, lidar_to_camera)

            roi_mask = self._spatial_roi_mask(points_base_all)
            roi_base = points_base_all[roi_mask]
            roi_camera = points_camera_all[roi_mask]
            if roi_base.size > 0:
                ground_mask = self._non_ground_mask(roi_base)
                points_base = roi_base[ground_mask]
                points_camera = roi_camera[ground_mask]

        debug_header = self._debug_header(pointcloud, bbox_stamp)
        self._publish_debug_pointcloud(
            self.debug_roi_points_pub,
            debug_header,
            points_base,
        )
        if points_base.size == 0 and points_lidar.size > 0:
            self._warn_throttled(
                "empty_roi",
                "No LiDAR points survived spatial/ground filtering; "
                "routing detections to vision fallback",
            )

        lidar_supported_detection_indices = set()
        if points_base.size > 0:
            clusters = self._cluster_cone_candidates(points_base, points_camera)
            if not clusters:
                self._warn_throttled(
                    "no_clusters",
                    f"Fusion found no global cone clusters from {len(points_base)} "
                    "non-ground points; trying bbox-guided LiDAR support",
                )
            cluster_assignments = (
                self._associate_detections_to_clusters(
                    detections,
                    clusters,
                    camera_info,
                    supported_detection_indices=(
                        lidar_supported_detection_indices
                    ),
                )
                if clusters
                else []
            )

        used_detection_indices = {
            assignment.detection_index for assignment in cluster_assignments
        }
        used_roi_indices = set()
        for assignment in cluster_assignments:
            consumed_indices = assignment.cluster.consumed_indices
            if consumed_indices is None:
                consumed_indices = assignment.cluster.indices
            used_roi_indices.update(int(index) for index in consumed_indices)

        if points_base.size > 0:
            sparse_assignments = self._associate_sparse_detections(
                detections,
                points_base,
                points_camera,
                camera_info,
                used_detection_indices,
                used_roi_indices,
                supported_detection_indices=(
                    lidar_supported_detection_indices
                ),
            )

        lidar_assignments = cluster_assignments + sparse_assignments
        used_detection_indices.update(
            assignment.detection_index for assignment in sparse_assignments
        )
        # A detection with valid LiDAR support must not fall through to a
        # visual tier merely because another detection won the one-to-one
        # cluster/point competition.  Doing so would duplicate one physical
        # cone with unrelated depth estimates.
        used_detection_indices.update(lidar_supported_detection_indices)
        visual_assignments = self._associate_visual_detections(
            detections,
            used_detection_indices,
            camera_info,
            bbox_stamp,
            left_image,
            right_image,
            right_camera_info,
        )
        assignments = lidar_assignments + visual_assignments
        reports = self._build_detection_debug_reports(
            detections,
            points_base_all,
            points_camera_all,
            points_base,
            points_camera,
            clusters,
            camera_info,
            assignments,
        )
        self._publish_debug_pointcloud(
            self.debug_sparse_support_pub,
            debug_header,
            self._assignment_support_points(sparse_assignments),
        )
        self._publish_fusion_debug_markers(
            debug_header,
            clusters,
            assignments,
            reports,
        )
        self._log_fusion_debug_summary(
            raw_point_count,
            int(points_base.shape[0]),
            detections,
            clusters,
            cluster_assignments,
            sparse_assignments,
            visual_assignments,
            reports,
        )
        if visual_assignments:
            sources = ", ".join(
                f"#{assignment.detection_index}:{assignment.source}"
                for assignment in visual_assignments
            )
            self._info_throttled(
                "visual_fallback",
                f"Vision fallback assignments: {sources}",
                period_sec=2.0,
            )
        if not assignments:
            detection = detections[0]
            self._warn_throttled(
                "no_assignments",
                "Fusion produced no cones: "
                f"0 bbox/cluster assignments from {len(detections)} detections "
                f"and {len(clusters)} clusters; "
                f"first bbox {detection.color}=({detection.xmin:.1f}, "
                f"{detection.ymin:.1f})-({detection.xmax:.1f}, "
                f"{detection.ymax:.1f}); "
                f"cluster pixels {self._cluster_summaries(clusters, camera_info)}",
            )

        for assignment in assignments:
            cone = self._cluster_to_cone(assignment.cluster)
            self._append_cone_by_color(msg, assignment.detection.color, cone)

        return msg

    def _extract_detections(
        self,
        bboxes: BoundingBoxes,
        camera_info: CameraInfo,
    ) -> List[Detection]:
        detections = []
        for bbox in bboxes.bounding_boxes:
            probability = float(bbox.probability)
            if probability < self.min_bbox_probability:
                continue

            xmin, ymin, xmax, ymax = self._bbox_to_pixels(bbox, camera_info)
            if xmax <= xmin or ymax <= ymin:
                continue

            detections.append(
                Detection(
                    color=self._normalize_color(bbox.color),
                    probability=probability,
                    xmin=xmin,
                    ymin=ymin,
                    xmax=xmax,
                    ymax=ymax,
                )
            )
        return detections

    def _bbox_to_pixels(
        self,
        bbox: BoundingBox,
        camera_info: CameraInfo,
    ) -> Tuple[float, float, float, float]:
        xmin = float(bbox.xmin)
        ymin = float(bbox.ymin)
        xmax = float(bbox.xmax)
        ymax = float(bbox.ymax)

        if int(bbox.type) == int(BoundingBox.PERCENTAGE):
            xmin *= float(camera_info.width)
            xmax *= float(camera_info.width)
            ymin *= float(camera_info.height)
            ymax *= float(camera_info.height)

        xmin, xmax = sorted((xmin, xmax))
        ymin, ymax = sorted((ymin, ymax))

        if self.clip_bboxes_to_image:
            xmin = max(0.0, min(float(camera_info.width - 1), xmin))
            xmax = max(0.0, min(float(camera_info.width - 1), xmax))
            ymin = max(0.0, min(float(camera_info.height - 1), ymin))
            ymax = max(0.0, min(float(camera_info.height - 1), ymax))
        return xmin, ymin, xmax, ymax

    def _pointcloud_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        points = []
        for point in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            points.append((float(point[0]), float(point[1]), float(point[2])))

        if not points:
            return np.empty((0, 3), dtype=np.float64)
        return np.asarray(points, dtype=np.float64)

    def _roi_mask(self, points_base: np.ndarray) -> np.ndarray:
        """Legacy fixed-height ROI retained for compatibility tests/tools."""
        if points_base.size == 0:
            return np.zeros(points_base.shape[0], dtype=bool)
        return self._spatial_roi_mask(points_base) & (
            points_base[:, 2] >= self.ground_min_z
        )

    def _spatial_roi_mask(self, points_base: np.ndarray) -> np.ndarray:
        """Apply only spatial and vehicle-body masks before plane fitting."""
        if points_base.size == 0:
            return np.zeros(points_base.shape[0], dtype=bool)
        mask = (
            (points_base[:, 0] >= self.roi_min_x)
            & (points_base[:, 0] <= self.roi_max_x)
            & (np.abs(points_base[:, 1]) <= self.roi_abs_y)
            & (points_base[:, 2] >= self.roi_min_z)
            & (points_base[:, 2] <= self.roi_max_z)
            & np.all(np.isfinite(points_base[:, :3]), axis=1)
        )
        if self.self_mask_enabled:
            mask &= ~self._self_mask(points_base)
        return mask

    def _non_ground_mask(self, points_base: np.ndarray) -> np.ndarray:
        """Return points above the fitted ground, failing safely to fixed-z."""
        if points_base.size == 0:
            return np.zeros(points_base.shape[0], dtype=bool)
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
                "ground_ransac",
                "Ground RANSAC found no valid near-horizontal plane; "
                "using configured fixed-z fallback",
                period_sec=5.0,
            )
            return points_base[:, 2] >= self.ground_min_z
        return result.non_ground_mask

    def _self_mask(self, points_base: np.ndarray) -> np.ndarray:
        if points_base.size == 0:
            return np.zeros(points_base.shape[0], dtype=bool)
        return (
            (points_base[:, 0] >= self.self_mask_min_x)
            & (points_base[:, 0] <= self.self_mask_max_x)
            & (np.abs(points_base[:, 1]) <= self.self_mask_abs_y)
            & (points_base[:, 2] >= self.self_mask_min_z)
            & (points_base[:, 2] <= self.self_mask_max_z)
        )

    def _cluster_cone_candidates(
        self,
        points_base: np.ndarray,
        points_camera: np.ndarray,
    ) -> List[Cluster]:
        labels = self._dbscan_xy(points_base[:, :2])
        clusters = []

        for label in sorted(set(labels)):
            if label < 0:
                continue
            indices = np.flatnonzero(labels == label)
            if len(indices) < self.cluster_min_points:
                continue

            cluster_base = points_base[indices]
            cluster_camera = points_camera[indices]
            min_values = cluster_base.min(axis=0)
            max_values = cluster_base.max(axis=0)
            height = max_values[2] - min_values[2]
            width = float(np.linalg.norm(max_values[:2] - min_values[:2]))

            if height < self.cluster_min_height or height > self.cluster_max_height:
                continue
            if width > self.cluster_max_width:
                continue

            centroid_base = cluster_base.mean(axis=0)
            clusters.append(
                Cluster(
                    points_base=cluster_base,
                    points_camera=cluster_camera,
                    centroid_base=centroid_base,
                    range_m=float(np.hypot(centroid_base[0], centroid_base[1])),
                    indices=indices,
                    support_count=len(indices),
                )
            )

        return clusters

    def _dbscan_xy(self, points_xy: np.ndarray) -> np.ndarray:
        labels = np.full(points_xy.shape[0], -1, dtype=np.int32)
        if points_xy.shape[0] == 0:
            return labels

        cell_size = self.cluster_eps
        cells = {}
        for index, point in enumerate(points_xy):
            key = (int(math.floor(point[0] / cell_size)), int(math.floor(point[1] / cell_size)))
            cells.setdefault(key, []).append(index)

        visited = np.zeros(points_xy.shape[0], dtype=bool)
        cluster_id = 0
        eps_sq = self.cluster_eps * self.cluster_eps

        for start_index in range(points_xy.shape[0]):
            if visited[start_index]:
                continue
            visited[start_index] = True

            neighbors = self._region_query(points_xy, cells, start_index, cell_size, eps_sq)
            if len(neighbors) < self.cluster_min_points:
                continue

            labels[start_index] = cluster_id
            queue = list(neighbors)
            while queue:
                current = queue.pop()
                if not visited[current]:
                    visited[current] = True
                    current_neighbors = self._region_query(
                        points_xy,
                        cells,
                        current,
                        cell_size,
                        eps_sq,
                    )
                    if len(current_neighbors) >= self.cluster_min_points:
                        queue.extend(current_neighbors)

                if labels[current] < 0:
                    labels[current] = cluster_id

            cluster_id += 1

        return labels

    def _region_query(
        self,
        points_xy: np.ndarray,
        cells,
        index: int,
        cell_size: float,
        eps_sq: float,
    ) -> List[int]:
        point = points_xy[index]
        cell = (int(math.floor(point[0] / cell_size)), int(math.floor(point[1] / cell_size)))
        neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for candidate in cells.get((cell[0] + dx, cell[1] + dy), []):
                    diff = points_xy[candidate] - point
                    if float(diff.dot(diff)) <= eps_sq:
                        neighbors.append(candidate)
        return neighbors

    def _associate_detections_to_clusters(
        self,
        detections: List[Detection],
        clusters: List[Cluster],
        camera_info: CameraInfo,
        supported_detection_indices=None,
    ) -> List[Assignment]:
        candidates = []
        for detection_index, detection in enumerate(detections):
            for cluster_index, cluster in enumerate(clusters):
                pixels, projected_indices = self._project_points_with_indices(
                    cluster.points_camera,
                    camera_info,
                )
                if pixels.size == 0:
                    continue

                inside = (
                    (pixels[:, 0] >= detection.xmin)
                    & (pixels[:, 0] <= detection.xmax)
                    & (pixels[:, 1] >= detection.ymin)
                    & (pixels[:, 1] <= detection.ymax)
                )
                inside_count = int(np.count_nonzero(inside))
                if inside_count < self.min_projected_points:
                    continue

                if supported_detection_indices is not None:
                    supported_detection_indices.add(detection_index)

                local_support_indices = projected_indices[inside]
                support_base = cluster.points_base[local_support_indices]
                support_camera = cluster.points_camera[local_support_indices]
                centroid_base = support_base.mean(axis=0)
                support_cluster = Cluster(
                    points_base=support_base,
                    points_camera=support_camera,
                    centroid_base=centroid_base,
                    range_m=float(np.hypot(centroid_base[0], centroid_base[1])),
                    indices=cluster.indices[local_support_indices],
                    source="lidar",
                    support_count=inside_count,
                    # Publish/estimate from bbox-supported points only, while
                    # consuming the complete DBSCAN cluster so its leftover
                    # points cannot create a duplicate sparse cone.
                    consumed_indices=cluster.indices.copy(),
                )
                score = inside_count * max(float(detection.probability), 0.01)
                candidates.append(
                    (
                        score,
                        detection_index,
                        cluster_index,
                        inside_count,
                        support_cluster,
                    )
                )

        candidates.sort(key=lambda item: (-item[0], item[1], item[2]))
        used_detections = set()
        used_clusters = set()
        assignments = []
        for (
            _,
            detection_index,
            cluster_index,
            inside_count,
            support_cluster,
        ) in candidates:
            if detection_index in used_detections or cluster_index in used_clusters:
                continue
            used_detections.add(detection_index)
            used_clusters.add(cluster_index)
            assignments.append(
                Assignment(
                    detection_index=detection_index,
                    detection=detections[detection_index],
                    cluster=support_cluster,
                    source="lidar",
                    support_count=inside_count,
                )
            )

        return assignments

    def _associate_sparse_detections(
        self,
        detections: List[Detection],
        points_base: np.ndarray,
        points_camera: np.ndarray,
        camera_info: CameraInfo,
        used_detection_indices,
        used_roi_indices,
        supported_detection_indices=None,
    ) -> List[Assignment]:
        if not self.sparse_association_enabled or points_base.size == 0:
            return []

        pixels, projected_indices = self._project_points_with_indices(
            points_camera,
            camera_info,
        )
        if pixels.size == 0:
            return []

        candidates = []
        unavailable_indices = set(int(index) for index in used_roi_indices)
        for detection_index, detection in enumerate(detections):
            if detection_index in used_detection_indices:
                continue

            xmin, ymin, xmax, ymax = self._expanded_bbox(detection)
            inside = (
                (pixels[:, 0] >= xmin)
                & (pixels[:, 0] <= xmax)
                & (pixels[:, 1] >= ymin)
                & (pixels[:, 1] <= ymax)
            )
            support_indices = [
                int(index)
                for index in projected_indices[inside]
                if int(index) not in unavailable_indices
            ]
            if not support_indices:
                continue

            support_indices = np.asarray(sorted(set(support_indices)), dtype=np.int64)
            support_base = points_base[support_indices]
            candidate = self._sparse_candidate(
                detection,
                support_indices,
                support_base,
                points_camera[support_indices],
            )
            if candidate is None:
                continue
            if supported_detection_indices is not None:
                supported_detection_indices.add(detection_index)
            score = (
                float(candidate.support_count)
                * max(float(detection.probability), 0.01)
                / max(float(candidate.range_m), 0.1)
            )
            candidates.append((score, detection_index, candidate))

        candidates.sort(reverse=True, key=lambda item: item[0])
        assignments = []
        used_sparse_indices = set()
        for _, detection_index, candidate in candidates:
            candidate_indices = set(int(index) for index in candidate.indices)
            if candidate_indices & used_sparse_indices:
                continue
            used_sparse_indices.update(candidate_indices)
            assignments.append(
                Assignment(
                    detection_index=detection_index,
                    detection=detections[detection_index],
                    cluster=candidate,
                    source="sparse",
                    support_count=candidate.support_count,
                )
            )
        return assignments

    def _sparse_candidate(
        self,
        detection: Detection,
        support_indices: np.ndarray,
        support_base: np.ndarray,
        support_camera: np.ndarray,
    ) -> Optional[Cluster]:
        centroid_base = support_base.mean(axis=0)
        range_m = float(np.hypot(centroid_base[0], centroid_base[1]))
        min_points = self._sparse_min_points_for_range(range_m)
        if len(support_indices) < min_points:
            return None

        min_values = support_base.min(axis=0)
        max_values = support_base.max(axis=0)
        width = float(np.linalg.norm(max_values[:2] - min_values[:2]))
        if width > self.sparse_max_width:
            return None

        point_ranges = np.hypot(support_base[:, 0], support_base[:, 1])
        depth_span = float(point_ranges.max() - point_ranges.min())
        allowed_depth_span = max(
            self.sparse_max_depth_span_m,
            range_m * self.sparse_max_depth_span_ratio,
        )
        if depth_span > allowed_depth_span:
            return None

        if range_m >= self.sparse_far_range_m:
            bbox_width = detection.xmax - detection.xmin
            bbox_height = detection.ymax - detection.ymin
            if detection.probability < self.sparse_far_min_probability:
                return None
            if bbox_width < self.sparse_far_min_bbox_width_px:
                return None
            if bbox_height < self.sparse_far_min_bbox_height_px:
                return None

        return Cluster(
            points_base=support_base,
            points_camera=support_camera,
            centroid_base=centroid_base,
            range_m=range_m,
            indices=support_indices,
            source="sparse",
            support_count=len(support_indices),
        )

    def _sparse_min_points_for_range(self, range_m: float) -> int:
        if range_m < self.sparse_near_range_m:
            return self.sparse_near_min_points
        if range_m < self.sparse_far_range_m:
            return self.sparse_mid_min_points
        return self.sparse_far_min_points

    def _expanded_bbox(self, detection: Detection) -> Tuple[float, float, float, float]:
        width = detection.xmax - detection.xmin
        height = detection.ymax - detection.ymin
        margin_x = max(self.sparse_bbox_margin_px, width * self.sparse_bbox_margin_ratio)
        margin_y = max(self.sparse_bbox_margin_px, height * self.sparse_bbox_margin_ratio)
        return (
            detection.xmin - margin_x,
            detection.ymin - margin_y,
            detection.xmax + margin_x,
            detection.ymax + margin_y,
        )

    def _associate_visual_detections(
        self,
        detections: List[Detection],
        used_detection_indices,
        camera_info: CameraInfo,
        stamp,
        left_image: Optional[Image],
        right_image: Optional[Image],
        right_camera_info: Optional[CameraInfo],
    ) -> List[Assignment]:
        """Route LiDAR-unmatched detections to exactly one vision tier."""
        if not self.monocular_fallback_enabled and not self.stereo_fallback_enabled:
            return []
        remaining = [
            index
            for index in range(len(detections))
            if index not in used_detection_indices
        ]
        if not remaining:
            return []

        # Vision rays belong to the bbox/image timestamp, not the paired
        # LiDAR timestamp.  Use a separate exact-time TF even when LiDAR was
        # available for other detections in the same bundle.
        camera_to_base = self._lookup_transform_matrix(
            self.output_frame,
            self.camera_frame,
            stamp,
        )

        camera_matrix = self._camera_matrix(camera_info)
        image_size = (int(camera_info.width), int(camera_info.height))
        assignments = []
        stereo_context = None
        stereo_context_attempted = False

        for detection_index in remaining:
            detection = detections[detection_index]
            bbox = (
                detection.xmin,
                detection.ymin,
                detection.xmax,
                detection.ymax,
            )
            condition = classify_cone_condition(
                bbox,
                image_size,
                min_height_to_width=self.good_cone_min_height_to_width,
                border_margin_ratio=self.good_cone_border_margin_ratio,
            )

            # The paper routes upright, fully visible cones to its fitted
            # bbox-height curve.  Here h is explicitly image-height normalized.
            if condition == "good" and self.monocular_fallback_enabled:
                depth = monocular_depth_from_bbox(
                    detection.ymax - detection.ymin,
                    float(camera_info.height),
                    coefficient=self.monocular_depth_coefficient,
                    exponent=self.monocular_depth_exponent,
                )
                if (
                    depth is not None
                    and self.monocular_min_depth_m <= depth <= self.monocular_max_depth_m
                ):
                    cluster = self._visual_cluster(
                        detection,
                        depth,
                        camera_matrix,
                        camera_to_base,
                        source="monocular",
                        support_count=1,
                    )
                    if cluster is not None:
                        assignments.append(
                            Assignment(
                                detection_index=detection_index,
                                detection=detection,
                                cluster=cluster,
                                source="monocular",
                                support_count=1,
                            )
                        )
                # A good cone belongs only to the monocular tier.  Invalid or
                # out-of-calibration estimates fail closed instead of silently
                # changing the paper's routing contract.
                continue

            if condition != "bad" or not self.stereo_fallback_enabled:
                continue
            if not stereo_context_attempted:
                stereo_context_attempted = True
                stereo_context = self._prepare_stereo_context(
                    camera_info,
                    right_camera_info,
                    stamp,
                    left_image,
                    right_image,
                )
            if stereo_context is None:
                continue

            left_gray, right_gray, baseline_m, principal_offset_px = stereo_context
            estimate = estimate_stereo_depth(
                left_gray,
                right_gray,
                bbox,
                fx=float(camera_matrix[0, 0]),
                baseline_m=baseline_m,
                min_depth_m=self.stereo_min_depth_m,
                max_depth_m=self.stereo_max_depth_m,
                epipolar_tolerance_px=self.stereo_epipolar_tolerance_px,
                slender_fraction=self.stereo_slender_fraction,
                ratio_threshold=self.stereo_ratio_threshold,
                min_matches=self.stereo_min_matches,
                principal_point_offset_px=principal_offset_px,
            )
            if estimate is None:
                continue
            cluster = self._visual_cluster(
                detection,
                estimate.depth_m,
                camera_matrix,
                camera_to_base,
                source="stereo_sift",
                support_count=estimate.match_count,
            )
            if cluster is None:
                continue
            assignments.append(
                Assignment(
                    detection_index=detection_index,
                    detection=detection,
                    cluster=cluster,
                    source="stereo_sift",
                    support_count=estimate.match_count,
                )
            )
        return assignments

    def _prepare_stereo_context(
        self,
        left_camera_info: CameraInfo,
        right_camera_info: Optional[CameraInfo],
        stamp,
        left_image: Optional[Image],
        right_image: Optional[Image],
    ):
        if left_image is None or right_image is None or right_camera_info is None:
            self._warn_throttled(
                "stereo_inputs",
                "Stereo fallback waiting for synchronized left/right images and "
                "right CameraInfo",
                period_sec=5.0,
            )
            return None
        try:
            self._validate_camera_info(
                right_camera_info,
                right_image,
                self.right_camera_frame,
            )
            if (
                int(left_image.width) != int(right_image.width)
                or int(left_image.height) != int(right_image.height)
            ):
                raise ValueError("left and right rectified images differ in size")
            left_gray = image_message_to_numpy(left_image, desired_encoding="mono8")
            right_gray = image_message_to_numpy(right_image, desired_encoding="mono8")
        except (RuntimeError, TypeError, ValueError) as exc:
            self._warn_throttled(
                "stereo_image",
                f"Stereo fallback rejected invalid image/calibration input ({exc})",
                period_sec=5.0,
            )
            return None

        baseline_m = self._resolve_stereo_baseline(
            left_camera_info,
            right_camera_info,
            stamp,
        )
        if baseline_m is None:
            self._warn_throttled(
                "stereo_baseline",
                "Stereo fallback has no valid projection, TF, or configured baseline",
                period_sec=5.0,
            )
            return None
        left_matrix = self._camera_matrix(left_camera_info)
        right_matrix = self._camera_matrix(right_camera_info)
        principal_offset_px = float(left_matrix[0, 2] - right_matrix[0, 2])
        return left_gray, right_gray, baseline_m, principal_offset_px

    def _resolve_stereo_baseline(
        self,
        left_camera_info: CameraInfo,
        right_camera_info: CameraInfo,
        stamp,
    ) -> Optional[float]:
        baseline = baseline_from_projection(
            left_camera_info.p,
            right_camera_info.p,
        )
        if baseline is not None:
            return baseline

        left_frame = str(left_camera_info.header.frame_id).strip() or self.camera_frame
        right_frame = (
            str(right_camera_info.header.frame_id).strip()
            or self.right_camera_frame
        )
        if left_frame and right_frame and left_frame != right_frame:
            try:
                right_to_left = self._lookup_transform_matrix(
                    left_frame,
                    right_frame,
                    stamp,
                )
                baseline = float(np.linalg.norm(right_to_left[:3, 3]))
                if math.isfinite(baseline) and baseline > 0.0:
                    return baseline
            except TransformException:
                pass

        if math.isfinite(self.stereo_baseline_m) and self.stereo_baseline_m > 0.0:
            return self.stereo_baseline_m
        return None

    def _visual_cluster(
        self,
        detection: Detection,
        depth_m: float,
        camera_matrix: np.ndarray,
        camera_to_base: np.ndarray,
        source: str,
        support_count: int,
    ) -> Optional[Cluster]:
        center_u = 0.5 * (detection.xmin + detection.xmax)
        center_v = 0.5 * (detection.ymin + detection.ymax)
        projection_point = camera_point_from_depth(
            center_u,
            center_v,
            depth_m,
            camera_matrix,
        )
        if projection_point is None:
            return None
        if self.projection_model == "eufs_bbox":
            point_camera = np.asarray(
                [
                    projection_point[2],
                    -projection_point[0],
                    -projection_point[1],
                ],
                dtype=np.float64,
            )
        elif self.projection_model == "pinhole":
            point_camera = projection_point
        else:
            return None
        point_base = self._transform_points(
            point_camera.reshape(1, 3),
            camera_to_base,
        )[0]
        if not np.all(np.isfinite(point_base)):
            return None
        range_m = float(np.hypot(point_base[0], point_base[1]))
        return Cluster(
            points_base=point_base.reshape(1, 3),
            points_camera=point_camera.reshape(1, 3),
            centroid_base=point_base,
            range_m=range_m,
            indices=np.empty((0,), dtype=np.int64),
            source=source,
            support_count=int(support_count),
        )

    def _project_points(self, points_camera: np.ndarray, camera_info: CameraInfo) -> np.ndarray:
        pixels, _ = self._project_points_with_indices(points_camera, camera_info)
        return pixels

    def _project_points_with_indices(
        self,
        points_camera: np.ndarray,
        camera_info: CameraInfo,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if points_camera.size == 0:
            return (
                np.empty((0, 2), dtype=np.float64),
                np.empty((0,), dtype=np.int64),
            )

        if self.projection_model == "eufs_bbox":
            projection_points = np.column_stack(
                (-points_camera[:, 1], -points_camera[:, 2], points_camera[:, 0])
            )
        elif self.projection_model == "pinhole":
            projection_points = points_camera
        else:
            raise RuntimeError(
                f"Unsupported projection_model {self.projection_model!r}"
            )

        z = projection_points[:, 2]
        valid = z > self.min_project_depth
        if not np.any(valid):
            return (
                np.empty((0, 2), dtype=np.float64),
                np.empty((0,), dtype=np.int64),
            )

        points = projection_points[valid]
        indices = np.flatnonzero(valid)
        camera_matrix = self._camera_matrix(camera_info)
        fx = float(camera_matrix[0, 0])
        fy = float(camera_matrix[1, 1])
        cx = float(camera_matrix[0, 2])
        cy = float(camera_matrix[1, 2])

        u = fx * points[:, 0] / points[:, 2] + cx
        v = fy * points[:, 1] / points[:, 2] + cy
        if self.clip_projected_points_to_image:
            in_image = (
                (u >= 0.0)
                & (u < float(camera_info.width))
                & (v >= 0.0)
                & (v < float(camera_info.height))
            )
            if not np.any(in_image):
                return (
                    np.empty((0, 2), dtype=np.float64),
                    np.empty((0,), dtype=np.int64),
                )
            return np.column_stack((u[in_image], v[in_image])), indices[in_image]

        return np.column_stack((u, v)), indices

    def _build_detection_debug_reports(
        self,
        detections: List[Detection],
        points_base_all: np.ndarray,
        points_camera_all: np.ndarray,
        points_base_roi: np.ndarray,
        points_camera_roi: np.ndarray,
        clusters: List[Cluster],
        camera_info: CameraInfo,
        assignments: List[Assignment],
    ) -> List[DetectionDebugReport]:
        assignment_by_detection = {
            assignment.detection_index: assignment for assignment in assignments
        }
        reports = []
        for detection_index, detection in enumerate(detections):
            raw_count, _ = self._bbox_point_support(
                detection,
                points_base_all,
                points_camera_all,
                camera_info,
            )
            roi_count, roi_centroid = self._bbox_point_support(
                detection,
                points_base_roi,
                points_camera_roi,
                camera_info,
                expanded=True,
            )
            cluster_count = 0
            for cluster in clusters:
                cluster_count += self._bbox_point_support(
                    detection,
                    cluster.points_base,
                    cluster.points_camera,
                    camera_info,
                )[0]

            assignment = assignment_by_detection.get(detection_index)
            assigned_source = assignment.source if assignment else ""
            reason = "assigned_" + assigned_source if assignment else self._unmatched_reason(
                raw_count,
                roi_count,
                cluster_count,
            )
            reports.append(
                DetectionDebugReport(
                    detection_index=detection_index,
                    detection=detection,
                    raw_projected_points=raw_count,
                    roi_projected_points=roi_count,
                    cluster_projected_points=cluster_count,
                    assigned_source=assigned_source,
                    reason=reason,
                    support_centroid_base=roi_centroid,
                )
            )
        return reports

    def _bbox_point_support(
        self,
        detection: Detection,
        points_base: np.ndarray,
        points_camera: np.ndarray,
        camera_info: CameraInfo,
        expanded: bool = False,
    ) -> Tuple[int, Optional[np.ndarray]]:
        if points_base.size == 0 or points_camera.size == 0:
            return 0, None
        pixels, indices = self._project_points_with_indices(points_camera, camera_info)
        if pixels.size == 0:
            return 0, None
        if expanded:
            xmin, ymin, xmax, ymax = self._expanded_bbox(detection)
        else:
            xmin, ymin, xmax, ymax = (
                detection.xmin,
                detection.ymin,
                detection.xmax,
                detection.ymax,
            )
        inside = (
            (pixels[:, 0] >= xmin)
            & (pixels[:, 0] <= xmax)
            & (pixels[:, 1] >= ymin)
            & (pixels[:, 1] <= ymax)
        )
        count = int(np.count_nonzero(inside))
        if count == 0:
            return 0, None
        support = points_base[indices[inside]]
        return count, support.mean(axis=0)

    @staticmethod
    def _unmatched_reason(
        raw_count: int,
        roi_count: int,
        cluster_count: int,
    ) -> str:
        if raw_count <= 0:
            return "no_lidar_support"
        if roi_count <= 0:
            return "rejected_by_roi_or_self_ground"
        if cluster_count <= 0:
            return "insufficient_cluster_support"
        return "duplicate_or_low_score"

    def _assignment_support_points(self, assignments: List[Assignment]) -> np.ndarray:
        sparse_points = [
            assignment.cluster.points_base
            for assignment in assignments
            if assignment.source == "sparse" and assignment.cluster.points_base.size > 0
        ]
        if not sparse_points:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(sparse_points)

    def _debug_header(self, pointcloud: PointCloud2, stamp=None):
        header = pointcloud.header.__class__()
        header.stamp = pointcloud.header.stamp if stamp is None else stamp
        header.frame_id = self.output_frame
        return header

    def _publish_debug_pointcloud(
        self,
        publisher,
        header,
        points: np.ndarray,
    ) -> None:
        if publisher is None:
            return
        publisher.publish(self._xyz_to_pointcloud2(header, points))

    @staticmethod
    def _xyz_to_pointcloud2(header, points: np.ndarray) -> PointCloud2:
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(points.shape[0]) if points.size else 0
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        if points.size:
            data = bytearray()
            for point in points:
                data.extend(
                    struct.pack(
                        "<fff",
                        float(point[0]),
                        float(point[1]),
                        float(point[2]),
                    )
                )
            msg.data = bytes(data)
        else:
            msg.data = b""
        return msg

    def _publish_fusion_debug_markers(
        self,
        header,
        clusters: List[Cluster],
        assignments: List[Assignment],
        reports: List[DetectionDebugReport],
    ) -> None:
        if self.debug_cluster_candidates_pub is not None:
            self.debug_cluster_candidates_pub.publish(
                self._cluster_debug_markers(header, clusters, assignments)
            )
        if self.debug_bbox_support_pub is not None:
            self.debug_bbox_support_pub.publish(
                self._bbox_support_markers(header, reports, rejections_only=False)
            )
        if self.debug_rejections_pub is not None:
            self.debug_rejections_pub.publish(
                self._bbox_support_markers(header, reports, rejections_only=True)
            )

    def _cluster_debug_markers(
        self,
        header,
        clusters: List[Cluster],
        assignments: List[Assignment],
    ) -> MarkerArray:
        markers = MarkerArray()
        markers.markers.append(self._clear_marker(header))
        marker_id = 1
        for cluster in clusters:
            marker = self._sphere_marker(
                header,
                "accepted_cluster",
                marker_id,
                cluster.centroid_base,
                0.22,
                (0.0, 0.9, 1.0, 0.85),
            )
            markers.markers.append(marker)
            marker_id += 1
        for assignment in assignments:
            if assignment.source not in ("sparse", "monocular", "stereo_sift"):
                continue
            if assignment.source == "sparse":
                namespace = "sparse_candidate"
                scale = 0.28
                color = (1.0, 0.55, 0.0, 0.9)
            elif assignment.source == "monocular":
                namespace = "monocular_candidate"
                scale = 0.30
                color = (0.8, 0.2, 1.0, 0.9)
            else:
                namespace = "stereo_sift_candidate"
                scale = 0.30
                color = (0.2, 1.0, 0.4, 0.9)
            marker = self._sphere_marker(
                header,
                namespace,
                marker_id,
                assignment.cluster.centroid_base,
                scale,
                color,
            )
            markers.markers.append(marker)
            marker_id += 1
        return markers

    def _bbox_support_markers(
        self,
        header,
        reports: List[DetectionDebugReport],
        rejections_only: bool,
    ) -> MarkerArray:
        markers = MarkerArray()
        markers.markers.append(self._clear_marker(header))
        marker_id = 1
        for report in reports:
            is_assigned = bool(report.assigned_source)
            if rejections_only and is_assigned:
                continue
            if not rejections_only and not is_assigned:
                continue
            position = report.support_centroid_base
            if position is None:
                position = np.array(
                    [0.6, -2.5 + 0.25 * float(report.detection_index), 1.0],
                    dtype=np.float64,
                )
            text = (
                f"{report.detection_index}:{report.detection.color} "
                f"raw={report.raw_projected_points} roi={report.roi_projected_points} "
                f"cl={report.cluster_projected_points} {report.reason}"
            )
            color = (
                (0.2, 1.0, 0.2, 0.9)
                if is_assigned
                else (1.0, 0.2, 0.2, 0.9)
            )
            if is_assigned:
                namespace = f"bbox_{report.assigned_source}"
            else:
                namespace = report.reason
            markers.markers.append(
                self._text_marker(header, namespace, marker_id, position, text, color)
            )
            marker_id += 1
        return markers

    @staticmethod
    def _sphere_marker(header, namespace, marker_id, position, scale, color) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker

    @staticmethod
    def _text_marker(header, namespace, marker_id, position, text, color) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2]) + 0.4
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = text
        return marker

    def _log_fusion_debug_summary(
        self,
        raw_point_count: int,
        roi_point_count: int,
        detections: List[Detection],
        clusters: List[Cluster],
        cluster_assignments: List[Assignment],
        sparse_assignments: List[Assignment],
        visual_assignments: List[Assignment],
        reports: List[DetectionDebugReport],
    ) -> None:
        compact_reports = []
        for report in reports[:8]:
            compact_reports.append(
                f"#{report.detection_index}:{report.detection.color}"
                f"(raw={report.raw_projected_points},roi={report.roi_projected_points},"
                f"cl={report.cluster_projected_points},{report.reason})"
            )
        self._info_throttled(
            "fusion_debug_summary",
            "Fusion debug: "
            f"raw={raw_point_count}, roi={roi_point_count}, "
            f"detections={len(detections)}, clusters={len(clusters)}, "
            f"cluster_assignments={len(cluster_assignments)}, "
            f"sparse_assignments={len(sparse_assignments)}, "
            f"visual_assignments={len(visual_assignments)}; "
            + "; ".join(compact_reports),
            period_sec=2.0,
        )

    def _cluster_pixel_summary(self, cluster: Cluster, camera_info: CameraInfo) -> str:
        pixels = self._project_points(cluster.points_camera, camera_info)
        if pixels.size == 0:
            return "empty"
        min_uv = pixels.min(axis=0)
        max_uv = pixels.max(axis=0)
        return (
            f"({min_uv[0]:.1f}, {min_uv[1]:.1f})-"
            f"({max_uv[0]:.1f}, {max_uv[1]:.1f})"
        )

    def _cluster_summaries(
        self,
        clusters: List[Cluster],
        camera_info: CameraInfo,
    ) -> str:
        summaries = []
        for index, cluster in enumerate(clusters[:5]):
            centroid = cluster.centroid_base
            summaries.append(
                f"#{index}@base({centroid[0]:.2f}, {centroid[1]:.2f}, "
                f"{centroid[2]:.2f})="
                f"{self._cluster_pixel_summary(cluster, camera_info)}"
            )
        return "; ".join(summaries)

    def _cluster_to_cone(self, cluster: Cluster) -> ConeWithCovariance:
        cone = ConeWithCovariance()
        cone.point.x = float(cluster.centroid_base[0])
        cone.point.y = float(cluster.centroid_base[1])
        cone.point.z = 0.0

        source_variances = {
            "sparse": (self.sparse_variance_x, self.sparse_variance_y),
            "monocular": (
                self.monocular_variance_x,
                self.monocular_variance_y,
            ),
            "stereo_sift": (self.stereo_variance_x, self.stereo_variance_y),
        }
        base_var_x, base_var_y = source_variances.get(
            cluster.source,
            (self.fused_variance_x, self.fused_variance_y),
        )
        var_x = base_var_x + self.range_variance_scale * cluster.range_m
        var_y = base_var_y + self.range_variance_scale * cluster.range_m
        cone.covariance = [
            max(var_x, self.min_variance),
            0.0,
            0.0,
            max(var_y, self.min_variance),
        ]
        return cone

    def _append_cone_by_color(
        self,
        msg: ConeArrayWithCovariance,
        color: str,
        cone: ConeWithCovariance,
    ) -> None:
        if color == "blue":
            msg.blue_cones.append(cone)
        elif color == "yellow":
            msg.yellow_cones.append(cone)
        elif color == "orange":
            msg.orange_cones.append(cone)
        elif color == "big_orange":
            msg.big_orange_cones.append(cone)
        else:
            msg.unknown_color_cones.append(cone)

    def _lookup_transform_matrix(
        self,
        target_frame: str,
        source_frame: str,
        stamp=None,
    ) -> np.ndarray:
        if stamp is None:
            lookup_time = Time()
        elif isinstance(stamp, Time):
            lookup_time = stamp
        else:
            lookup_time = Time.from_msg(stamp)
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            lookup_time,
            # This node uses a single-threaded executor. Blocking here would
            # prevent the /tf callback that could satisfy the lookup from
            # running; the synchronized sensor pair is retained and retried.
            timeout=Duration(seconds=0.0),
        )
        return self._transform_to_matrix(transform.transform)

    def _lookup_transform_matrix_between_times(
        self,
        target_frame: str,
        target_stamp,
        source_frame: str,
        source_stamp,
    ) -> np.ndarray:
        """Transform a source sample into the target frame at another time."""
        if self._stamp_key(target_stamp) == self._stamp_key(source_stamp):
            return self._lookup_transform_matrix(
                target_frame,
                source_frame,
                target_stamp,
            )

        transform = self.tf_buffer.lookup_transform_full(
            target_frame,
            Time.from_msg(target_stamp),
            source_frame,
            Time.from_msg(source_stamp),
            self.motion_compensation_frame,
            timeout=Duration(seconds=0.0),
        )
        return self._transform_to_matrix(transform.transform)

    @staticmethod
    def _transform_to_matrix(transform) -> np.ndarray:
        translation = transform.translation
        rotation = transform.rotation
        x = float(rotation.x)
        y = float(rotation.y)
        z = float(rotation.z)
        w = float(rotation.w)

        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 0.0:
            x, y, z, w = 0.0, 0.0, 0.0, 1.0
        else:
            x, y, z, w = x / norm, y / norm, z / norm, w / norm

        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
        matrix[:3, 3] = np.array(
            [float(translation.x), float(translation.y), float(translation.z)],
            dtype=np.float64,
        )
        return matrix

    @staticmethod
    def _transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
        rotation = transform[:3, :3]
        translation = transform[:3, 3]
        return points.dot(rotation.T) + translation

    def _oracle_callback(self, msg: ConeArrayWithCovariance) -> None:
        stamp_ns = self._prepare_sensor_message_stamp(
            "oracle", msg.header.stamp
        )
        if stamp_ns is None:
            return

        normalized = ConeArrayWithCovariance()
        normalized.header.stamp = msg.header.stamp
        normalized.header.frame_id = msg.header.frame_id
        transform = None
        source_frame = str(msg.header.frame_id).strip()
        if self.oracle_rewrite_frame and source_frame != self.output_frame:
            if not source_frame:
                self._warn_throttled(
                    "oracle_frame",
                    "Dropping oracle cones without a source frame; coordinates "
                    "cannot be rewritten safely",
                )
                return
            try:
                transform = self._lookup_transform_matrix(
                    self.output_frame,
                    source_frame,
                    msg.header.stamp,
                )
            except TransformException as exc:
                self._warn_throttled(
                    "oracle_tf",
                    f"Dropping oracle cones because {source_frame} -> "
                    f"{self.output_frame} TF is unavailable: {exc}",
                )
                return
            normalized.header.frame_id = self.output_frame

        normalized.blue_cones = self._normalize_cones(msg.blue_cones, transform)
        normalized.yellow_cones = self._normalize_cones(msg.yellow_cones, transform)
        normalized.orange_cones = self._normalize_cones(msg.orange_cones, transform)
        normalized.big_orange_cones = self._normalize_cones(
            msg.big_orange_cones, transform
        )
        normalized.unknown_color_cones = self._normalize_cones(
            msg.unknown_color_cones, transform
        )
        self._publish_cones(normalized)

    def _publish_cones(self, msg: ConeArrayWithCovariance) -> None:
        self.cones_pub.publish(msg)
        self.cones_viz_pub.publish(self._cones_to_markers(msg, self.marker_scale))

    @staticmethod
    def _viz_topic_for(topic: str) -> str:
        return topic.rstrip("/") + "/viz"

    @classmethod
    def _cones_to_markers(
        cls,
        msg: ConeArrayWithCovariance,
        marker_scale: float = 0.25,
    ) -> MarkerArray:
        markers = MarkerArray()
        markers.markers.append(cls._clear_marker(msg.header))
        markers.markers.extend(
            [
                cls._cone_list_marker(
                    msg.header,
                    "blue",
                    0,
                    msg.blue_cones,
                    (0.0, 0.2, 1.0, 0.95),
                    marker_scale,
                ),
                cls._cone_list_marker(
                    msg.header,
                    "yellow",
                    1,
                    msg.yellow_cones,
                    (1.0, 0.9, 0.0, 0.95),
                    marker_scale,
                ),
                cls._cone_list_marker(
                    msg.header,
                    "orange",
                    2,
                    msg.orange_cones,
                    (1.0, 0.45, 0.0, 0.95),
                    marker_scale,
                ),
                cls._cone_list_marker(
                    msg.header,
                    "big_orange",
                    3,
                    msg.big_orange_cones,
                    (1.0, 0.2, 0.0, 0.95),
                    marker_scale,
                ),
                cls._cone_list_marker(
                    msg.header,
                    "unknown",
                    4,
                    msg.unknown_color_cones,
                    (0.7, 0.7, 0.7, 0.95),
                    marker_scale,
                ),
            ]
        )
        return markers

    @staticmethod
    def _clear_marker(header) -> Marker:
        marker = Marker()
        marker.header = header
        marker.action = Marker.DELETEALL
        return marker

    @staticmethod
    def _cone_list_marker(
        header,
        namespace,
        marker_id,
        cones,
        color,
        marker_scale: float,
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker_scale
        marker.scale.y = marker_scale
        marker.scale.z = marker_scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        for cone in cones:
            point = Point()
            point.x = float(cone.point.x)
            point.y = float(cone.point.y)
            point.z = float(cone.point.z) + 0.5 * marker_scale
            marker.points.append(point)
        return marker

    def _normalize_cones(
        self,
        cones: Iterable[ConeWithCovariance],
        transform: Optional[np.ndarray] = None,
    ):
        normalized = []
        for cone in cones:
            if not self._finite_xyz(cone.point.x, cone.point.y, cone.point.z):
                continue

            point = np.array(
                [float(cone.point.x), float(cone.point.y), float(cone.point.z), 1.0],
                dtype=np.float64,
            )
            covariance = np.asarray(
                self._normalize_covariance(cone.covariance), dtype=np.float64
            ).reshape(2, 2)
            if transform is not None:
                point = transform @ point
                planar_rotation = transform[:2, :2]
                covariance = planar_rotation @ covariance @ planar_rotation.T
            if not self._finite_xy(point[0], point[1]):
                continue

            clean = ConeWithCovariance()
            clean.point.x = float(point[0])
            clean.point.y = float(point[1])
            clean.point.z = 0.0
            clean.covariance = self._normalize_covariance(covariance.reshape(-1))
            normalized.append(clean)
        return normalized

    def _normalize_covariance(self, covariance) -> list:
        values = list(covariance)
        if len(values) != 4 or not all(math.isfinite(float(value)) for value in values):
            return [self.default_variance_x, 0.0, 0.0, self.default_variance_y]

        xx = max(float(values[0]), self.min_variance)
        xy = 0.5 * (float(values[1]) + float(values[2]))
        yy = max(float(values[3]), self.min_variance)

        if xx * yy - xy * xy <= self.min_variance * self.min_variance:
            return [self.default_variance_x, 0.0, 0.0, self.default_variance_y]

        return [xx, xy, xy, yy]

    def _bounding_boxes_stamp(self, msg: BoundingBoxes):
        if int(msg.image_header.stamp.sec) != 0 or int(msg.image_header.stamp.nanosec) != 0:
            return msg.image_header.stamp
        return msg.header.stamp

    def _warn_throttled(self, key: str, message: str, period_sec: float = 2.0) -> None:
        now = time.monotonic()
        last_time = self.last_warning_time.get(key, 0.0)
        if now - last_time >= period_sec:
            self.last_warning_time[key] = now
            self.get_logger().warn(message)

    def _info_throttled(self, key: str, message: str, period_sec: float = 2.0) -> None:
        now = time.monotonic()
        key = f"info:{key}"
        last_time = self.last_warning_time.get(key, 0.0)
        if now - last_time >= period_sec:
            self.last_warning_time[key] = now
            self.get_logger().info(message)

    @staticmethod
    def _cone_count_summary(msg: ConeArrayWithCovariance) -> str:
        return (
            f"blue={len(msg.blue_cones)}, yellow={len(msg.yellow_cones)}, "
            f"orange={len(msg.orange_cones)}, big_orange={len(msg.big_orange_cones)}, "
            f"unknown={len(msg.unknown_color_cones)}"
        )

    @staticmethod
    def _normalize_color(color: str) -> str:
        value = color.strip().lower().replace(" ", "_").replace("-", "_")
        if "blue" in value:
            return "blue"
        if "yellow" in value:
            return "yellow"
        if "big" in value and "orange" in value:
            return "big_orange"
        if "orange" in value:
            return "orange"
        return "unknown"

    @staticmethod
    def _finite_xy(x_value: float, y_value: float) -> bool:
        return math.isfinite(float(x_value)) and math.isfinite(float(y_value))

    @staticmethod
    def _finite_xyz(x_value: float, y_value: float, z_value: float) -> bool:
        return (
            math.isfinite(float(x_value))
            and math.isfinite(float(y_value))
            and math.isfinite(float(z_value))
        )

    @staticmethod
    def _is_canonical_nonzero_stamp(stamp) -> bool:
        try:
            seconds = int(stamp.sec)
            nanoseconds = int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False
        return (
            seconds >= 0
            and 0 <= nanoseconds < 1_000_000_000
            and (seconds != 0 or nanoseconds != 0)
        )

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _format_stamp(stamp) -> str:
        return f"{int(stamp.sec)}.{int(stamp.nanosec):09d}"

    @staticmethod
    def _stamp_key(stamp) -> StampKey:
        return int(stamp.sec), int(stamp.nanosec)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionBaselineNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
