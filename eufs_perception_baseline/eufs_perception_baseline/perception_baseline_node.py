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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

try:
    from sensor_msgs_py import point_cloud2
except ImportError:
    from eufs_perception_baseline import point_cloud2_compat as point_cloud2


StampKey = Tuple[int, int]
SyncKey = Tuple[StampKey, StampKey]


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
    """Baseline perception node for EUFS graph SLAM.

    Two modes are intentionally kept in one node:
    - Oracle adapter mode republishes simulator cone arrays into the SLAM contract.
    - LiDAR-camera fusion mode follows the IIT Bombay baseline idea: camera
      detections provide cone class and image boxes, while LiDAR clusters provide
      metric cone positions.
    """

    def __init__(self) -> None:
        super().__init__("perception_baseline_node")

        self._declare_parameters()
        self._load_parameters()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
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
            sensor_qos,
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

        self.oracle_sub = None
        if self.oracle_cones_topic:
            self.oracle_sub = self.create_subscription(
                ConeArrayWithCovariance,
                self.oracle_cones_topic,
                self._oracle_callback,
                10,
            )

        self.latest_image: Optional[Image] = None
        self.latest_pointcloud: Optional[PointCloud2] = None
        self.latest_bboxes: Optional[BoundingBoxes] = None
        self.latest_camera_info: Optional[CameraInfo] = None
        self.last_empty_key: Optional[SyncKey] = None
        self.last_fusion_key: Optional[SyncKey] = None
        self.last_warning_time = {}

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
            f"Fusion inputs: image={self.image_topic}, pointcloud={self.pointcloud_topic}, "
            f"bboxes={self.bbox_topic}, camera_info={self.camera_info_topic}"
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
        self.declare_parameter("image_topic", "/zed/left/image_rect_color")
        self.declare_parameter("pointcloud_topic", "/velodyne_points")
        self.declare_parameter("bbox_topic", "/noisy_bounding_boxes")
        self.declare_parameter("camera_info_topic", "/custom_camera_info")
        self.declare_parameter("camera_frame", "zed_right_camera_optical_frame")
        self.declare_parameter("projection_model", "eufs_bbox")
        self.declare_parameter("clip_bboxes_to_image", False)
        self.declare_parameter("clip_projected_points_to_image", False)
        self.declare_parameter("output_cones_topic", "/cones")
        self.declare_parameter("output_frame", "base_footprint")
        self.declare_parameter("marker_scale", 0.35)
        self.declare_parameter("sync_tolerance_sec", 0.15)
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
        self.declare_parameter("sparse_mid_min_points", 3)
        self.declare_parameter("sparse_far_min_points", 2)
        self.declare_parameter("sparse_far_min_probability", 0.25)
        self.declare_parameter("sparse_far_min_bbox_width_px", 4.0)
        self.declare_parameter("sparse_far_min_bbox_height_px", 4.0)
        self.declare_parameter("sparse_max_depth_span_m", 1.0)
        self.declare_parameter("sparse_max_depth_span_ratio", 0.35)
        self.declare_parameter("sparse_max_width", 0.90)
        self.declare_parameter("sparse_variance_x", 0.12)
        self.declare_parameter("sparse_variance_y", 0.12)

        self.declare_parameter("default_variance_x", 0.04)
        self.declare_parameter("default_variance_y", 0.04)
        self.declare_parameter("fused_variance_x", 0.04)
        self.declare_parameter("fused_variance_y", 0.04)
        self.declare_parameter("range_variance_scale", 0.0005)
        self.declare_parameter("min_variance", 1.0e-4)

    def _load_parameters(self) -> None:
        self.image_topic = self.get_parameter("image_topic").value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.projection_model = self.get_parameter("projection_model").value
        self.clip_bboxes_to_image = self._as_bool(
            self.get_parameter("clip_bboxes_to_image").value
        )
        self.clip_projected_points_to_image = self._as_bool(
            self.get_parameter("clip_projected_points_to_image").value
        )
        self.output_cones_topic = self.get_parameter("output_cones_topic").value
        self.output_frame = self.get_parameter("output_frame").value
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.sync_tolerance_sec = float(self.get_parameter("sync_tolerance_sec").value)
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

        self.default_variance_x = float(self.get_parameter("default_variance_x").value)
        self.default_variance_y = float(self.get_parameter("default_variance_y").value)
        self.fused_variance_x = float(self.get_parameter("fused_variance_x").value)
        self.fused_variance_y = float(self.get_parameter("fused_variance_y").value)
        self.range_variance_scale = float(self.get_parameter("range_variance_scale").value)
        self.min_variance = float(self.get_parameter("min_variance").value)

    def _image_callback(self, msg: Image) -> None:
        self.latest_image = msg
        self._try_publish_empty_from_sensor_pair()

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        self.latest_pointcloud = msg
        self._try_publish_empty_from_sensor_pair()
        self._try_publish_fusion()

    def _bbox_callback(self, msg: BoundingBoxes) -> None:
        self.latest_bboxes = msg
        self._try_publish_fusion()

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id
        self._try_publish_fusion()

    def _try_publish_empty_from_sensor_pair(self) -> None:
        if not self.publish_empty_on_sync:
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
        if self.latest_pointcloud is None:
            missing_inputs.append(f"pointcloud={self.pointcloud_topic}")
        if self.latest_bboxes is None:
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

        cloud_stamp = self.latest_pointcloud.header.stamp
        bbox_stamp = self._bounding_boxes_stamp(self.latest_bboxes)
        stamp_delta = abs(
            self._stamp_to_sec(cloud_stamp) - self._stamp_to_sec(bbox_stamp)
        )
        if stamp_delta > self.sync_tolerance_sec:
            self._warn_throttled(
                "fusion_sync_mismatch",
                "Fusion waiting for synchronized bbox/LiDAR: "
                f"delta={stamp_delta:.3f}s > tolerance={self.sync_tolerance_sec:.3f}s "
                f"(cloud={self._format_stamp(cloud_stamp)}, "
                f"bbox={self._format_stamp(bbox_stamp)})",
                period_sec=2.0,
            )
            return

        key = (self._stamp_key(cloud_stamp), self._stamp_key(bbox_stamp))
        if key == self.last_fusion_key:
            return
        self.last_fusion_key = key

        try:
            cones = self._run_lidar_camera_fusion(
                self.latest_pointcloud,
                self.latest_bboxes,
                self.latest_camera_info,
            )
        except TransformException as exc:
            self._warn_throttled("tf", f"Fusion skipped: missing TF ({exc})")
            return
        except RuntimeError as exc:
            self._warn_throttled("fusion", f"Fusion skipped: {exc}")
            return

        self._info_throttled(
            "fusion_success",
            "Fusion published cones: " + self._cone_count_summary(cones),
            period_sec=2.0,
        )
        self._publish_cones(cones)

    def _run_lidar_camera_fusion(
        self,
        pointcloud: PointCloud2,
        bboxes: BoundingBoxes,
        camera_info: CameraInfo,
    ) -> ConeArrayWithCovariance:
        detections = self._extract_detections(bboxes, camera_info)
        msg = ConeArrayWithCovariance()
        msg.header.stamp = pointcloud.header.stamp
        msg.header.frame_id = self.output_frame

        if not detections:
            self._warn_throttled(
                "no_detections",
                "Fusion produced no cones: no bbox detections after filtering",
            )
            return msg

        lidar_frame = pointcloud.header.frame_id
        if not lidar_frame:
            raise RuntimeError("point cloud frame_id is empty")
        if not self.camera_frame:
            raise RuntimeError("camera frame is empty")

        points_lidar = self._pointcloud_to_xyz(pointcloud)
        raw_point_count = int(points_lidar.shape[0])
        if points_lidar.size == 0:
            self._warn_throttled(
                "no_lidar_points",
                "Fusion produced no cones: point cloud has no finite xyz points",
            )
            return msg

        lidar_to_base = self._lookup_transform_matrix(self.output_frame, lidar_frame)
        lidar_to_camera = self._lookup_transform_matrix(self.camera_frame, lidar_frame)

        points_base_all = self._transform_points(points_lidar, lidar_to_base)
        points_camera_all = self._transform_points(points_lidar, lidar_to_camera)

        roi_mask = self._roi_mask(points_base_all)
        points_base = points_base_all[roi_mask]
        points_camera = points_camera_all[roi_mask]
        debug_header = self._debug_header(pointcloud)
        self._publish_debug_pointcloud(
            self.debug_roi_points_pub,
            debug_header,
            points_base,
        )
        if points_base.size == 0:
            self._warn_throttled(
                "empty_roi",
                "Fusion produced no cones: no LiDAR points survived ROI filtering",
            )
            reports = self._build_detection_debug_reports(
                detections,
                points_base_all,
                points_camera_all,
                points_base,
                points_camera,
                [],
                camera_info,
                [],
            )
            self._publish_fusion_debug_markers(debug_header, [], [], reports)
            return msg

        clusters = self._cluster_cone_candidates(points_base, points_camera)
        if not clusters:
            self._warn_throttled(
                "no_clusters",
                f"Fusion found no global cone clusters from {len(points_base)} ROI points; "
                "trying sparse bbox-guided association if enabled",
            )

        cluster_assignments = (
            self._associate_detections_to_clusters(detections, clusters, camera_info)
            if clusters
            else []
        )
        used_detection_indices = {
            assignment.detection_index for assignment in cluster_assignments
        }
        used_roi_indices = set()
        for assignment in cluster_assignments:
            used_roi_indices.update(int(index) for index in assignment.cluster.indices)

        sparse_assignments = self._associate_sparse_detections(
            detections,
            points_base,
            points_camera,
            camera_info,
            used_detection_indices,
            used_roi_indices,
        )
        assignments = cluster_assignments + sparse_assignments
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
            reports,
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
        mask = (
            (points_base[:, 0] >= self.roi_min_x)
            & (points_base[:, 0] <= self.roi_max_x)
            & (np.abs(points_base[:, 1]) <= self.roi_abs_y)
            & (points_base[:, 2] >= self.roi_min_z)
            & (points_base[:, 2] <= self.roi_max_z)
            & (points_base[:, 2] >= self.ground_min_z)
        )
        if self.self_mask_enabled:
            mask &= ~self._self_mask(points_base)
        return mask

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
    ) -> List[Assignment]:
        candidates = []
        for detection_index, detection in enumerate(detections):
            for cluster_index, cluster in enumerate(clusters):
                pixels = self._project_points(cluster.points_camera, camera_info)
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

                score = inside_count * max(detection.probability, 1.0)
                candidates.append(
                    (score, detection_index, cluster_index, inside_count)
                )

        candidates.sort(reverse=True, key=lambda item: item[0])
        used_detections = set()
        used_clusters = set()
        assignments = []
        for _, detection_index, cluster_index, inside_count in candidates:
            if detection_index in used_detections or cluster_index in used_clusters:
                continue
            used_detections.add(detection_index)
            used_clusters.add(cluster_index)
            assignments.append(
                Assignment(
                    detection_index=detection_index,
                    detection=detections[detection_index],
                    cluster=clusters[cluster_index],
                    source="cluster",
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
        else:
            projection_points = points_camera

        z = projection_points[:, 2]
        valid = z > self.min_project_depth
        if not np.any(valid):
            return (
                np.empty((0, 2), dtype=np.float64),
                np.empty((0,), dtype=np.int64),
            )

        points = projection_points[valid]
        indices = np.flatnonzero(valid)
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])

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

    def _debug_header(self, pointcloud: PointCloud2):
        header = pointcloud.header.__class__()
        header.stamp = pointcloud.header.stamp
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
            if assignment.source != "sparse":
                continue
            marker = self._sphere_marker(
                header,
                "sparse_candidate",
                marker_id,
                assignment.cluster.centroid_base,
                0.28,
                (1.0, 0.55, 0.0, 0.9),
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
                namespace = (
                    "bbox_sparse"
                    if report.assigned_source == "sparse"
                    else "bbox_supported"
                )
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
            f"sparse_assignments={len(sparse_assignments)}; "
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

        base_var_x = (
            self.sparse_variance_x if cluster.source == "sparse" else self.fused_variance_x
        )
        base_var_y = (
            self.sparse_variance_y if cluster.source == "sparse" else self.fused_variance_y
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

    def _lookup_transform_matrix(self, target_frame: str, source_frame: str) -> np.ndarray:
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.05),
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
        normalized = ConeArrayWithCovariance()
        normalized.header = msg.header
        if self.oracle_rewrite_frame:
            normalized.header.frame_id = self.output_frame

        normalized.blue_cones = self._normalize_cones(msg.blue_cones)
        normalized.yellow_cones = self._normalize_cones(msg.yellow_cones)
        normalized.orange_cones = self._normalize_cones(msg.orange_cones)
        normalized.big_orange_cones = self._normalize_cones(msg.big_orange_cones)
        normalized.unknown_color_cones = self._normalize_cones(msg.unknown_color_cones)
        self._publish_cones(normalized)

    def _publish_cones(self, msg: ConeArrayWithCovariance) -> None:
        self.cones_pub.publish(msg)
        self.cones_viz_pub.publish(self._cones_to_markers(msg, self.marker_scale))

    @staticmethod
    def _viz_topic_for(topic: str) -> str:
        return topic.rstrip("/") + "/viz"

    # Real 3D cone meshes — the same assets Gazebo places on the track, so the
    # detections in RViz look identical to the simulated cones. The `unknown`
    # class reuses the small-cone mesh with a grey tint (materials disabled).
    _CONE_MESHES = {
        "blue": ("package://eufs_tracks/meshes/cone_blue.dae", None),
        "yellow": ("package://eufs_tracks/meshes/cone_yellow.dae", None),
        "orange": ("package://eufs_tracks/meshes/cone.dae", None),
        "big_orange": ("package://eufs_tracks/meshes/cone_big.dae", None),
        "unknown": ("package://eufs_tracks/meshes/cone.dae", (0.6, 0.6, 0.6, 0.9)),
    }

    @classmethod
    def _cones_to_markers(
        cls,
        msg: ConeArrayWithCovariance,
        marker_scale: float = 0.25,  # noqa: ARG003 — kept for call compatibility
    ) -> MarkerArray:
        markers = MarkerArray()
        markers.markers.append(cls._clear_marker(msg.header))
        cone_sets = [
            ("blue", msg.blue_cones),
            ("yellow", msg.yellow_cones),
            ("orange", msg.orange_cones),
            ("big_orange", msg.big_orange_cones),
            ("unknown", msg.unknown_color_cones),
        ]
        for namespace, cones in cone_sets:
            mesh, tint = cls._CONE_MESHES[namespace]
            for i, cone in enumerate(cones):
                markers.markers.append(
                    cls._cone_mesh_marker(msg.header, namespace, i, cone, mesh, tint)
                )
        return markers

    @staticmethod
    def _cone_mesh_marker(header, namespace, marker_id, cone, mesh, tint) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = mesh
        marker.pose.position.x = float(cone.point.x)
        marker.pose.position.y = float(cone.point.y)
        marker.pose.position.z = float(cone.point.z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        if tint is None:
            marker.mesh_use_embedded_materials = True
        else:
            marker.mesh_use_embedded_materials = False
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = tint
        return marker

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

    def _normalize_cones(self, cones: Iterable[ConeWithCovariance]):
        normalized = []
        for cone in cones:
            if not self._finite_xy(cone.point.x, cone.point.y):
                continue

            clean = ConeWithCovariance()
            clean.point.x = float(cone.point.x)
            clean.point.y = float(cone.point.y)
            clean.point.z = 0.0
            clean.covariance = self._normalize_covariance(cone.covariance)
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
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

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
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
