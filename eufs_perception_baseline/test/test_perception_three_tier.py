import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from eufs_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo, PointCloud2

import eufs_perception_baseline.perception_baseline_node as node_module
from eufs_perception_baseline.fusion_core import StereoDepthEstimate
from eufs_perception_baseline.perception_baseline_node import (
    Cluster,
    Detection,
    PerceptionBaselineNode,
)
from eufs_perception_baseline.sync_buffer import TimestampBuffer


class ThreeTierRoutingTest(unittest.TestCase):
    def _node(self):
        node = object.__new__(PerceptionBaselineNode)
        node.monocular_fallback_enabled = True
        node.monocular_depth_coefficient = 0.498
        node.monocular_depth_exponent = -0.954
        node.monocular_min_depth_m = 0.5
        node.monocular_max_depth_m = 30.0
        node.good_cone_min_height_to_width = 1.2
        node.good_cone_border_margin_ratio = 0.01
        node.stereo_fallback_enabled = True
        node.stereo_min_depth_m = 0.5
        node.stereo_max_depth_m = 30.0
        node.stereo_epipolar_tolerance_px = 2.0
        node.stereo_slender_fraction = 0.5
        node.stereo_ratio_threshold = 0.75
        node.stereo_min_matches = 3
        node.projection_model = "pinhole"
        node.output_frame = "base_footprint"
        node.camera_frame = "left_optical"
        return node

    @staticmethod
    def _camera_info():
        info = CameraInfo()
        info.width = 100
        info.height = 100
        info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]
        return info

    @staticmethod
    def _camera_to_base():
        # Optical z becomes base x, optical x becomes -base y.
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = np.asarray(
            [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
        )
        return transform

    def test_lidar_assignment_prevents_all_lower_tiers(self):
        node = self._node()
        detection = Detection("blue", 0.9, 40.0, 20.0, 60.0, 80.0)

        with patch.object(
            node,
            "_lookup_transform_matrix",
            side_effect=AssertionError("lower tier must not be evaluated"),
        ):
            assignments = node._associate_visual_detections(
                [detection],
                {0},
                self._camera_info(),
                TimeMsg(sec=1),
                None,
                None,
                None,
            )

        self.assertEqual(assignments, [])

    def test_good_unmatched_cone_routes_only_to_monocular(self):
        node = self._node()
        detection = Detection("yellow", 0.8, 40.0, 20.0, 60.0, 80.0)

        with patch.object(
            node,
            "_prepare_stereo_context",
            side_effect=AssertionError("good cone must not enter stereo"),
        ), patch.object(
            node,
            "_lookup_transform_matrix",
            return_value=self._camera_to_base(),
        ) as lookup:
            stamp = TimeMsg(sec=1, nanosec=23)
            assignments = node._associate_visual_detections(
                [detection],
                set(),
                self._camera_info(),
                stamp,
                None,
                None,
                None,
            )

        self.assertEqual(len(assignments), 1)
        self.assertEqual(assignments[0].source, "monocular")
        expected_depth = 0.498 * (0.6 ** -0.954)
        self.assertAlmostEqual(assignments[0].cluster.centroid_base[0], expected_depth)
        lookup.assert_called_once_with(
            "base_footprint",
            "left_optical",
            stamp,
        )

    def test_bad_unmatched_cone_routes_only_to_stereo_sift(self):
        node = self._node()
        detection = Detection("orange", 0.8, 20.0, 40.0, 80.0, 60.0)
        stereo_context = (
            np.zeros((100, 100), dtype=np.uint8),
            np.zeros((100, 100), dtype=np.uint8),
            0.12,
            0.0,
        )

        stereo_patch = patch.object(
            node,
            "_prepare_stereo_context",
            return_value=stereo_context,
        )
        estimate_patch = patch.object(
            node_module,
            "estimate_stereo_depth",
            return_value=StereoDepthEstimate(6.0, 8.0, 4),
        )
        mono_patch = patch.object(
            node_module,
            "monocular_depth_from_bbox",
            side_effect=AssertionError("bad cone must not enter monocular"),
        )
        transform_patch = patch.object(
            node,
            "_lookup_transform_matrix",
            return_value=self._camera_to_base(),
        )
        with stereo_patch, estimate_patch, mono_patch, transform_patch:
            assignments = node._associate_visual_detections(
                [detection],
                set(),
                self._camera_info(),
                TimeMsg(sec=1),
                None,
                None,
                None,
            )

        self.assertEqual(len(assignments), 1)
        self.assertEqual(assignments[0].source, "stereo_sift")
        self.assertEqual(assignments[0].support_count, 4)
        self.assertAlmostEqual(assignments[0].cluster.centroid_base[0], 6.0)


class FusionPairSelectionTest(unittest.TestCase):
    def _node(self):
        node = object.__new__(PerceptionBaselineNode)
        node.image_buffer = TimestampBuffer(64)
        node.right_image_buffer = TimestampBuffer(64)
        node.bbox_buffer = TimestampBuffer(8)
        node.pointcloud_buffer = TimestampBuffer(8)
        node.sync_tolerance_sec = 0.2
        node.timestamp_reset_threshold_sec = 0.1
        node.latest_stream_stamp_ns = {}
        node.latest_image = None
        node.latest_right_image = None
        node.latest_pointcloud = None
        node.latest_bboxes = None
        node.last_empty_key = None
        node.last_fusion_key = None
        node._warn_throttled = lambda *args, **kwargs: None
        return node

    def test_waits_for_first_future_cloud_before_selecting_nearest(self):
        node = self._node()
        node.bbox_buffer.add(1_000, "bbox")
        node.pointcloud_buffer.add(850, "older-cloud")

        self.assertIsNone(node._select_ready_fusion_pair(200))

        node.pointcloud_buffer.add(1_050, "closer-future-cloud")
        bbox_entry, cloud_entry = node._select_ready_fusion_pair(200)
        self.assertEqual(bbox_entry.value, "bbox")
        self.assertEqual(cloud_entry.value, "closer-future-cloud")

    def test_exact_stamp_pair_is_ready_without_waiting(self):
        node = self._node()
        node.bbox_buffer.add(1_000, "bbox")
        node.pointcloud_buffer.add(1_000, "cloud")

        bbox_entry, cloud_entry = node._select_ready_fusion_pair(200)
        self.assertEqual(bbox_entry.stamp_ns, 1_000)
        self.assertEqual(cloud_entry.stamp_ns, 1_000)

    def test_mature_bbox_without_cloud_match_is_dropped(self):
        node = self._node()
        node.bbox_buffer.add(1_000, "expired")
        node.bbox_buffer.add(2_000, "pending")
        node.pointcloud_buffer.add(1_500, "watermark")

        self.assertIsNone(node._select_ready_fusion_pair(200))
        self.assertEqual(
            [entry.value for entry in node.bbox_buffer.entries()],
            ["pending"],
        )

    def test_high_rate_bboxes_do_not_steal_scarce_cloud(self):
        node = self._node()
        for stamp in (20, 40, 60, 80, 100):
            node.bbox_buffer.add(stamp, "bbox-{}".format(stamp))
        node.pointcloud_buffer.add(100, "cloud-100")

        bbox_entry, cloud_entry = node._select_ready_fusion_pair(50)

        self.assertEqual(bbox_entry.stamp_ns, 100)
        self.assertEqual(cloud_entry.stamp_ns, 100)
        self.assertEqual(
            [entry.stamp_ns for entry in node.bbox_buffer.entries()],
            [100],
        )

    def test_high_rate_clouds_do_not_steal_scarce_bbox(self):
        node = self._node()
        for stamp in (100, 120, 140, 160, 180, 200):
            node.pointcloud_buffer.add(stamp, "cloud-{}".format(stamp))
        node.bbox_buffer.add(200, "bbox-200")

        bbox_entry, cloud_entry = node._select_ready_fusion_pair(50)

        self.assertEqual(bbox_entry.stamp_ns, 200)
        self.assertEqual(cloud_entry.stamp_ns, 200)
        self.assertEqual(
            [entry.stamp_ns for entry in node.pointcloud_buffer.entries()],
            [200],
        )

    def test_image_history_retains_frame_across_detector_latency(self):
        node = self._node()
        frame_period_ns = 16_666_667
        for frame_index in range(40):
            stamp = frame_index * frame_period_ns
            node.image_buffer.add(stamp, frame_index)

        acquisition_stamp = 0
        delayed_match = node.image_buffer.nearest(acquisition_stamp, 0)
        self.assertIsNotNone(delayed_match)
        self.assertEqual(delayed_match.value, 0)

    def test_subsecond_timestamp_rollback_starts_new_buffer_epoch(self):
        node = self._node()
        old_stamp = 800_000_000
        node.image_buffer.add(old_stamp, "left")
        node.right_image_buffer.add(old_stamp, "right")
        node.pointcloud_buffer.add(old_stamp, "cloud")
        for stamp in range(660_000_000, old_stamp + 1, 20_000_000):
            node.bbox_buffer.add(stamp, "bbox")
        node.latest_stream_stamp_ns["left_image"] = old_stamp
        node.latest_image = object()
        node.latest_right_image = object()
        node.latest_pointcloud = object()
        node.latest_bboxes = object()
        node.last_empty_key = ((1, 0), (1, 0))
        node.last_fusion_key = ((1, 0), (1, 0))

        node._prepare_sensor_stamp("left_image", 0)

        self.assertEqual(len(node.image_buffer), 0)
        self.assertEqual(len(node.right_image_buffer), 0)
        self.assertEqual(len(node.pointcloud_buffer), 0)
        self.assertEqual(len(node.bbox_buffer), 0)
        self.assertEqual(node.latest_stream_stamp_ns, {"left_image": 0})
        self.assertIsNone(node.latest_image)
        self.assertIsNone(node.latest_right_image)
        self.assertIsNone(node.latest_pointcloud)
        self.assertIsNone(node.latest_bboxes)
        self.assertIsNone(node.last_empty_key)
        self.assertIsNone(node.last_fusion_key)


class CalibrationAndTfTest(unittest.TestCase):
    def test_fusion_array_uses_bbox_time_as_canonical_timestamp(self):
        node = object.__new__(PerceptionBaselineNode)
        node.output_frame = "base_footprint"
        node._warn_throttled = lambda *args, **kwargs: None
        cloud = PointCloud2()
        cloud.header.stamp = TimeMsg(sec=10)
        boxes = BoundingBoxes()
        boxes.image_header.stamp = TimeMsg(sec=11, nanosec=42)

        with patch.object(node, "_validate_camera_info"), patch.object(
            node,
            "_extract_detections",
            return_value=[],
        ):
            result = node._run_lidar_camera_fusion(
                cloud,
                boxes,
                CameraInfo(),
            )

        self.assertEqual(result.header.stamp.sec, 11)
        self.assertEqual(result.header.stamp.nanosec, 42)
        self.assertEqual(result.header.frame_id, "base_footprint")

    def test_rectified_projection_matrix_is_preferred_over_k(self):
        info = CameraInfo()
        info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]
        info.p = [200.0, 0.0, 60.0, 0.0, 0.0, 200.0, 40.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        np.testing.assert_allclose(
            PerceptionBaselineNode._camera_matrix(info),
            [[200.0, 0.0, 60.0], [0.0, 200.0, 40.0], [0.0, 0.0, 1.0]],
        )

    def test_tf_lookup_uses_message_timestamp(self):
        node = object.__new__(PerceptionBaselineNode)
        captured = {}

        class FakeBuffer:
            def lookup_transform(self, target, source, stamp, timeout):
                captured["target"] = target
                captured["source"] = source
                captured["nanoseconds"] = stamp.nanoseconds
                transform = SimpleNamespace(
                    translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
                return SimpleNamespace(transform=transform)

        node.tf_buffer = FakeBuffer()
        stamp = TimeMsg(sec=12, nanosec=345)
        matrix = node._lookup_transform_matrix("base", "lidar", stamp)

        self.assertEqual(captured["target"], "base")
        self.assertEqual(captured["source"], "lidar")
        self.assertEqual(captured["nanoseconds"], 12_000_000_345)
        np.testing.assert_allclose(matrix, np.eye(4))

    def test_cross_timestamp_tf_uses_fixed_frame_motion_compensation(self):
        node = object.__new__(PerceptionBaselineNode)
        node.motion_compensation_frame = "map"
        captured = {}

        class FakeBuffer:
            def lookup_transform_full(
                self,
                target,
                target_time,
                source,
                source_time,
                fixed,
                timeout,
            ):
                captured["target"] = target
                captured["target_ns"] = target_time.nanoseconds
                captured["source"] = source
                captured["source_ns"] = source_time.nanoseconds
                captured["fixed"] = fixed
                transform = SimpleNamespace(
                    translation=SimpleNamespace(x=1.0, y=0.0, z=0.0),
                    rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
                return SimpleNamespace(transform=transform)

        node.tf_buffer = FakeBuffer()
        target_stamp = TimeMsg(sec=12, nanosec=100)
        source_stamp = TimeMsg(sec=11, nanosec=900_000_000)
        matrix = node._lookup_transform_matrix_between_times(
            "base",
            target_stamp,
            "lidar",
            source_stamp,
        )

        self.assertEqual(captured["target"], "base")
        self.assertEqual(captured["target_ns"], 12_000_000_100)
        self.assertEqual(captured["source"], "lidar")
        self.assertEqual(captured["source_ns"], 11_900_000_000)
        self.assertEqual(captured["fixed"], "map")
        np.testing.assert_allclose(matrix[:3, 3], [1.0, 0.0, 0.0])

    def test_covariance_uses_visual_source_contract(self):
        node = object.__new__(PerceptionBaselineNode)
        node.sparse_variance_x = 0.1
        node.sparse_variance_y = 0.2
        node.monocular_variance_x = 0.3
        node.monocular_variance_y = 0.4
        node.stereo_variance_x = 0.5
        node.stereo_variance_y = 0.6
        node.fused_variance_x = 0.04
        node.fused_variance_y = 0.05
        node.range_variance_scale = 0.0
        node.min_variance = 1.0e-4
        cluster = Cluster(
            points_base=np.asarray([[5.0, 0.0, 0.0]]),
            points_camera=np.asarray([[0.0, 0.0, 5.0]]),
            centroid_base=np.asarray([5.0, 0.0, 0.0]),
            range_m=5.0,
            indices=np.empty((0,), dtype=np.int64),
            source="stereo_sift",
        )

        cone = node._cluster_to_cone(cluster)

        self.assertEqual(list(cone.covariance), [0.5, 0.0, 0.0, 0.6])


if __name__ == "__main__":
    unittest.main()
