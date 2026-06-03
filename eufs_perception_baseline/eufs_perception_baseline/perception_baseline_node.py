import math
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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


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
            10,
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
        self.declare_parameter("sync_tolerance_sec", 0.15)
        self.declare_parameter("publish_empty_on_sync", False)
        self.declare_parameter("fusion_enabled", True)
        self.declare_parameter("oracle_cones_topic", "")
        self.declare_parameter("oracle_rewrite_frame", True)

        self.declare_parameter("roi_min_x", 0.5)
        self.declare_parameter("roi_max_x", 30.0)
        self.declare_parameter("roi_abs_y", 15.0)
        self.declare_parameter("roi_min_z", -0.2)
        self.declare_parameter("roi_max_z", 1.5)
        self.declare_parameter("ground_min_z", 0.05)

        self.declare_parameter("cluster_eps", 0.35)
        self.declare_parameter("cluster_min_points", 3)
        self.declare_parameter("cluster_min_height", 0.02)
        self.declare_parameter("cluster_max_height", 0.80)
        self.declare_parameter("cluster_max_width", 0.90)

        self.declare_parameter("min_bbox_probability", 0.0)
        self.declare_parameter("min_projected_points", 1)
        self.declare_parameter("min_project_depth", 0.2)

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
        self.sync_tolerance_sec = float(self.get_parameter("sync_tolerance_sec").value)
        self.publish_empty_on_sync = self._as_bool(
            self.get_parameter("publish_empty_on_sync").value
        )
        self.fusion_enabled = self._as_bool(self.get_parameter("fusion_enabled").value)
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

        self.cluster_eps = float(self.get_parameter("cluster_eps").value)
        self.cluster_min_points = int(self.get_parameter("cluster_min_points").value)
        self.cluster_min_height = float(self.get_parameter("cluster_min_height").value)
        self.cluster_max_height = float(self.get_parameter("cluster_max_height").value)
        self.cluster_max_width = float(self.get_parameter("cluster_max_width").value)

        self.min_bbox_probability = float(self.get_parameter("min_bbox_probability").value)
        self.min_projected_points = int(self.get_parameter("min_projected_points").value)
        self.min_project_depth = float(self.get_parameter("min_project_depth").value)

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
        self.cones_pub.publish(msg)

    def _try_publish_fusion(self) -> None:
        if not self.fusion_enabled:
            return
        if (
            self.latest_pointcloud is None
            or self.latest_bboxes is None
            or self.latest_camera_info is None
        ):
            return

        cloud_stamp = self.latest_pointcloud.header.stamp
        bbox_stamp = self._bounding_boxes_stamp(self.latest_bboxes)
        if abs(self._stamp_to_sec(cloud_stamp) - self._stamp_to_sec(bbox_stamp)) > self.sync_tolerance_sec:
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

        self.cones_pub.publish(cones)

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
            return msg

        lidar_frame = pointcloud.header.frame_id
        if not lidar_frame:
            raise RuntimeError("point cloud frame_id is empty")
        if not self.camera_frame:
            raise RuntimeError("camera frame is empty")

        points_lidar = self._pointcloud_to_xyz(pointcloud)
        if points_lidar.size == 0:
            return msg

        lidar_to_base = self._lookup_transform_matrix(self.output_frame, lidar_frame)
        lidar_to_camera = self._lookup_transform_matrix(self.camera_frame, lidar_frame)

        points_base = self._transform_points(points_lidar, lidar_to_base)
        points_camera = self._transform_points(points_lidar, lidar_to_camera)

        roi_mask = self._roi_mask(points_base)
        points_base = points_base[roi_mask]
        points_camera = points_camera[roi_mask]
        if points_base.size == 0:
            return msg

        clusters = self._cluster_cone_candidates(points_base, points_camera)
        if not clusters:
            return msg

        assignments = self._associate_detections_to_clusters(
            detections,
            clusters,
            camera_info,
        )

        for detection, cluster in assignments:
            cone = self._cluster_to_cone(cluster)
            self._append_cone_by_color(msg, detection.color, cone)

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
        return (
            (points_base[:, 0] >= self.roi_min_x)
            & (points_base[:, 0] <= self.roi_max_x)
            & (np.abs(points_base[:, 1]) <= self.roi_abs_y)
            & (points_base[:, 2] >= self.roi_min_z)
            & (points_base[:, 2] <= self.roi_max_z)
            & (points_base[:, 2] >= self.ground_min_z)
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
    ) -> List[Tuple[Detection, Cluster]]:
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
                candidates.append((score, detection_index, cluster_index))

        candidates.sort(reverse=True, key=lambda item: item[0])
        used_detections = set()
        used_clusters = set()
        assignments = []
        for _, detection_index, cluster_index in candidates:
            if detection_index in used_detections or cluster_index in used_clusters:
                continue
            used_detections.add(detection_index)
            used_clusters.add(cluster_index)
            assignments.append((detections[detection_index], clusters[cluster_index]))

        return assignments

    def _project_points(self, points_camera: np.ndarray, camera_info: CameraInfo) -> np.ndarray:
        if points_camera.size == 0:
            return np.empty((0, 2), dtype=np.float64)

        if self.projection_model == "eufs_bbox":
            projection_points = np.column_stack(
                (-points_camera[:, 1], -points_camera[:, 2], points_camera[:, 0])
            )
        else:
            projection_points = points_camera

        z = projection_points[:, 2]
        valid = z > self.min_project_depth
        if not np.any(valid):
            return np.empty((0, 2), dtype=np.float64)

        points = projection_points[valid]
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
                return np.empty((0, 2), dtype=np.float64)
            return np.column_stack((u[in_image], v[in_image]))

        return np.column_stack((u, v))

    def _cluster_to_cone(self, cluster: Cluster) -> ConeWithCovariance:
        cone = ConeWithCovariance()
        cone.point.x = float(cluster.centroid_base[0])
        cone.point.y = float(cluster.centroid_base[1])
        cone.point.z = 0.0

        var_x = self.fused_variance_x + self.range_variance_scale * cluster.range_m
        var_y = self.fused_variance_y + self.range_variance_scale * cluster.range_m
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
        self.cones_pub.publish(normalized)

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
