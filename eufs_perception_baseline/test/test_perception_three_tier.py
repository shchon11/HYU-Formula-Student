import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from eufs_msgs.msg import (
    BoundingBox,
    BoundingBoxes,
    ConeArrayWithCovariance,
    ConeWithCovariance,
)
from geometry_msgs.msg import TransformStamped
from rclpy.clock import ClockChange, TimeJump
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformException

import eufs_perception_baseline.perception_baseline_node as node_module
from eufs_perception_baseline.fusion_core import StereoDepthEstimate
from eufs_perception_baseline.rektnet import cone_keypoint_template
from eufs_perception_baseline.perception_baseline_node import (
    Assignment,
    Cluster,
    Detection,
    FusionComputation,
    PerceptionBaselineNode,
    StereoContext,
)
from eufs_perception_baseline.latest_only_worker import WorkerCompletion
from eufs_perception_baseline.sync_buffer import TimestampBuffer


class ThreeTierRoutingTest(unittest.TestCase):
    def _node(self):
        node = object.__new__(PerceptionBaselineNode)
        node.monocular_fallback_enabled = True
        node.monocular_allowed_colors = {"blue", "yellow", "orange"}
        node.monocular_depth_coefficient = 0.498
        node.monocular_depth_exponent = -0.954
        node.monocular_min_depth_m = 0.5
        node.monocular_max_depth_m = 30.0
        node.good_cone_min_height_to_width = 1.2
        node.good_cone_border_margin_ratio = 0.01
        node.horizontal_clip_fallback_enabled = True
        node.horizontal_clip_min_probability = 0.7
        node.horizontal_clip_min_bbox_height_px = 20.0
        node.stereo_fallback_enabled = True
        node.stereo_min_depth_m = 0.5
        node.stereo_max_depth_m = 30.0
        node.stereo_epipolar_tolerance_px = 2.0
        node.stereo_slender_fraction = 0.5
        node.stereo_ratio_threshold = 0.75
        node.stereo_min_matches = 3
        node.rektnet_estimator = Mock()
        node.rektnet_min_heatmap_peak = 0.0
        node.rektnet_pnp_max_reprojection_error_px = 4.0
        node.rektnet_right_roi_padding_ratio = 0.08
        node.rektnet_standard_object_points = cone_keypoint_template(0.31, 0.11)
        node.rektnet_large_object_points = cone_keypoint_template(0.53, 0.13)
        node.projection_model = "pinhole"
        node.output_frame = "base_footprint"
        node.camera_frame = "left_optical"
        return node

    @staticmethod
    def _stereo_context():
        return StereoContext(
            left_image=np.zeros((100, 100, 3), dtype=np.uint8),
            right_image=np.zeros((100, 100, 3), dtype=np.uint8),
            left_camera_matrix=np.asarray(
                [[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]]
            ),
            right_camera_matrix=np.asarray(
                [[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]]
            ),
            left_distortion=np.zeros(5),
            right_distortion=np.zeros(5),
            right_from_left=np.asarray(
                [[1.0, 0.0, 0.0, -0.12], [0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
            ),
            baseline_m=0.12,
            principal_offset_px=0.0,
        )

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

    def test_rektnet_template_selection_preserves_metric_seven_points(self):
        node = self._node()

        np.testing.assert_array_equal(
            node._rektnet_cone_template("blue"),
            node.rektnet_standard_object_points,
        )
        np.testing.assert_array_equal(
            node._rektnet_cone_template("big_orange"),
            node.rektnet_large_object_points,
        )

    def test_rektnet_template_rejects_nonmetric_or_degenerate_input(self):
        with self.assertRaisesRegex(ValueError, "exactly 21"):
            PerceptionBaselineNode._validated_rektnet_template(
                [0.0] * 20,
                "template",
            )
        with self.assertRaisesRegex(ValueError, "non-collinear"):
            PerceptionBaselineNode._validated_rektnet_template(
                [0.0] * 21,
                "template",
            )

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

    def test_bad_unmatched_cone_routes_to_rektnet_pnp_sift(self):
        node = self._node()
        detection = Detection("orange", 0.8, 20.0, 40.0, 80.0, 60.0)
        stereo_context = self._stereo_context()

        stereo_patch = patch.object(
            node,
            "_prepare_stereo_context",
            return_value=stereo_context,
        )
        estimate_patch = patch.object(
            node_module,
            "estimate_rektnet_stereo_depth",
            return_value=SimpleNamespace(
                depth=StereoDepthEstimate(6.0, 8.0, 4)
            ),
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
        self.assertEqual(assignments[0].source, "stereo_rektnet_pnp_sift")
        self.assertEqual(assignments[0].support_count, 4)
        self.assertAlmostEqual(assignments[0].cluster.centroid_base[0], 6.0)

    def test_big_orange_good_bbox_avoids_standard_cone_mono_curve(self):
        node = self._node()
        detection = Detection("big_orange", 0.8, 40.0, 20.0, 60.0, 80.0)
        stereo_context = self._stereo_context()

        with patch.object(
            node,
            "_prepare_stereo_context",
            return_value=stereo_context,
        ) as prepare_stereo, patch.object(
            node_module,
            "estimate_rektnet_stereo_depth",
            return_value=SimpleNamespace(
                depth=StereoDepthEstimate(6.0, 8.0, 4)
            ),
        ), patch.object(
            node_module,
            "monocular_depth_from_bbox",
            side_effect=AssertionError(
                "big orange cone must not use standard-cone mono calibration"
            ),
        ), patch.object(
            node,
            "_lookup_transform_matrix",
            return_value=self._camera_to_base(),
        ):
            assignments = node._associate_visual_detections(
                [detection],
                set(),
                self._camera_info(),
                TimeMsg(sec=1),
                None,
                None,
                None,
            )

        prepare_stereo.assert_called_once()
        self.assertEqual(len(assignments), 1)
        self.assertEqual(assignments[0].source, "stereo_rektnet_pnp_sift")

    def test_horizontal_clip_uses_high_uncertainty_mono_after_stereo_failure(self):
        node = self._node()
        detection = Detection("blue", 0.9, 92.0, 20.0, 100.0, 80.0)

        with patch.object(
            node,
            "_prepare_stereo_context",
            return_value=None,
        ) as prepare_stereo, patch.object(
            node,
            "_lookup_transform_matrix",
            return_value=self._camera_to_base(),
        ):
            assignments = node._associate_visual_detections(
                [detection],
                set(),
                self._camera_info(),
                TimeMsg(sec=1),
                None,
                None,
                None,
            )

        prepare_stereo.assert_called_once()
        self.assertEqual(len(assignments), 1)
        self.assertEqual(assignments[0].source, "monocular_horizontal_clip")

    def test_vertical_clip_stays_fail_closed_when_stereo_fails(self):
        node = self._node()
        detection = Detection("blue", 0.9, 40.0, 0.0, 60.0, 60.0)

        with patch.object(
            node,
            "_prepare_stereo_context",
            return_value=None,
        ), patch.object(
            node,
            "_lookup_transform_matrix",
            return_value=self._camera_to_base(),
        ):
            assignments = node._associate_visual_detections(
                [detection],
                set(),
                self._camera_info(),
                TimeMsg(sec=1),
                None,
                None,
                None,
            )

        self.assertEqual(assignments, [])


class FusionPairSelectionTest(unittest.TestCase):
    class _InlineWorker:
        def __init__(self, node):
            self.node = node

        def submit(self, job):
            try:
                completion = WorkerCompletion(
                    job=job,
                    result=self.node._compute_fusion_job(job),
                )
            except BaseException as exc:  # noqa: BLE001 - test worker seam
                completion = WorkerCompletion(job=job, error=exc)
            self.node._commit_fusion_completion(completion)
            return True

        def clear_pending(self):
            pass

    class _DeferredWorker:
        def __init__(self):
            self.jobs = []
            self.clear_pending_calls = 0

        def submit(self, job):
            self.jobs.append(job)
            return True

        def clear_pending(self):
            self.clear_pending_calls += 1

    def _node(self):
        node = object.__new__(PerceptionBaselineNode)
        node.image_buffer = TimestampBuffer(64)
        node.right_image_buffer = TimestampBuffer(64)
        node.stereo_buffer = TimestampBuffer(64)
        node.bbox_buffer = TimestampBuffer(8)
        node.pointcloud_buffer = TimestampBuffer(8)
        node.sync_tolerance_sec = 0.2
        node.image_sync_tolerance_sec = 0.05
        node.pointcloud_topic = "/points"
        node.bbox_topic = "/boxes"
        node.camera_info_topic = "/camera_info"
        node.timestamp_reset_threshold_sec = 0.1
        node.max_future_stamp_lead_sec = 0.09
        node.max_deferred_future_stamp_lead_sec = 0.30
        node.stereo_pair_wait_sec = 0.80
        node.timestamp_reset_threshold_ns = 100_000_000
        node.max_future_stamp_lead_ns = 90_000_000
        node.max_deferred_future_stamp_lead_ns = 300_000_000
        node.stereo_pair_wait_ns = 800_000_000
        node.stereo_fallback_enabled = False
        node.sensor_epoch = 0
        node._clock_generation = 0
        node._shutting_down = False
        node._fusion_job_sequence = 0
        node._fusion_job_inflight = None
        node._deferred_fusion_completion = None
        node.epoch_start_ros_time_ns = None
        node.replay_guard_end_ros_time_ns = None
        node.replay_guard_ends_ros_time_ns = []
        node._pending_fallback_rollback = None
        node.last_observed_ros_time_ns = 10_000_000_000
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_000_000_000),
            ros_time_is_active=True,
        )
        node.latest_stream_stamp_ns = {}
        node.latest_image = None
        node.latest_right_image = None
        node.latest_pointcloud = None
        node.latest_bboxes = None
        node.latest_camera_info = None
        node.latest_right_camera_info = None
        node.last_empty_key = None
        node.last_fusion_key = None
        node.debug_roi_points_pub = None
        node.debug_sparse_support_pub = None
        node.debug_cluster_candidates_pub = None
        node.debug_bbox_support_pub = None
        node.debug_rejections_pub = None
        node.tf_buffer = Mock()
        node._reset_tf_epoch = Mock()
        node._warn_throttled = lambda *args, **kwargs: None
        node._fusion_worker = self._InlineWorker(node)
        return node

    @staticmethod
    def _fusion_messages(stamp_sec=1):
        cloud = PointCloud2()
        cloud.header.stamp = TimeMsg(sec=stamp_sec)
        boxes = BoundingBoxes()
        boxes.image_header.stamp = TimeMsg(sec=stamp_sec)
        return cloud, boxes

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

    def test_stereo_images_are_paired_jointly_by_acquisition_stamp(self):
        node = self._node()
        left_images = []
        right_images = []
        for stamp_ns in (1_000, 2_000):
            left = Image()
            right = Image()
            left_images.append(left)
            right_images.append(right)
            node.image_buffer.add(stamp_ns, left)
            node.right_image_buffer.add(stamp_ns, right)

        node._pair_ready_stereo_images()

        pairs = node.stereo_buffer.entries()
        self.assertEqual([entry.stamp_ns for entry in pairs], [1_000, 2_000])
        self.assertEqual(len(node.image_buffer), 0)
        self.assertEqual(len(node.right_image_buffer), 0)
        self.assertIs(pairs[0].value.left_image, left_images[0])
        self.assertIs(pairs[0].value.right_image, right_images[0])

    def test_fusion_waits_boundedly_for_delayed_right_stereo_frame(self):
        node = self._node()
        node.fusion_enabled = True
        node.stereo_fallback_enabled = True
        node.latest_camera_info = CameraInfo()
        node.latest_right_camera_info = CameraInfo()
        node._info_throttled = Mock()
        node._publish_cones = Mock()
        now_ns = [1_200_000_000]
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now_ns[0]),
            ros_time_is_active=True,
        )
        cloud, boxes = self._fusion_messages(stamp_sec=1)
        node.pointcloud_buffer.add(1_000_000_000, cloud)
        node.bbox_buffer.add(1_000_000_000, boxes)
        node.right_image_buffer.add(700_000_000, Image())
        recovered = ConeArrayWithCovariance()
        node._run_lidar_camera_fusion = Mock(return_value=recovered)

        node._try_publish_fusion()

        node._run_lidar_camera_fusion.assert_not_called()
        self.assertEqual(len(node.pointcloud_buffer), 1)
        self.assertEqual(len(node.bbox_buffer), 1)

        left = Image()
        right = Image()
        node.image_buffer.add(1_000_000_000, left)
        node.right_image_buffer.add(1_000_000_000, right)
        node._pair_ready_stereo_images()
        node._try_publish_fusion()

        node._run_lidar_camera_fusion.assert_called_once()
        self.assertIs(
            node._run_lidar_camera_fusion.call_args.kwargs["left_image"],
            left,
        )
        self.assertIs(
            node._run_lidar_camera_fusion.call_args.kwargs["right_image"],
            right,
        )
        self.assertEqual(len(node.pointcloud_buffer), 0)
        self.assertEqual(len(node.bbox_buffer), 0)

    def test_stereo_wait_expires_or_stops_after_right_stream_passes_target(self):
        node = self._node()
        target_ns = 1_000_000_000
        now_ns = [1_200_000_000]
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now_ns[0]),
            ros_time_is_active=True,
        )
        node.right_image_buffer.add(700_000_000, Image())

        self.assertTrue(node._stereo_inputs_may_still_arrive(target_ns, 50_000_000))

        node.right_image_buffer.add(1_060_000_000, Image())
        self.assertFalse(node._stereo_inputs_may_still_arrive(target_ns, 50_000_000))

        node.right_image_buffer.clear()
        now_ns[0] = 1_800_000_001
        self.assertFalse(node._stereo_inputs_may_still_arrive(target_ns, 50_000_000))

    def test_short_future_skew_is_buffered_until_clock_catches_up(self):
        node = self._node()
        now_ns = [10_000_000_000]
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now_ns[0]),
            ros_time_is_active=True,
        )
        stamp_ns = 10_100_000_000

        self.assertTrue(node._prepare_sensor_stamp("pointcloud", stamp_ns))
        self.assertFalse(node._stamp_ready_for_processing(stamp_ns))

        now_ns[0] = 10_020_000_000
        self.assertTrue(node._stamp_ready_for_processing(stamp_ns))

    def test_first_active_ros_clock_sample_is_awaited_without_dropping_input(self):
        node = self._node()
        node.last_observed_ros_time_ns = None
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=0),
            ros_time_is_active=True,
        )

        self.assertTrue(node._prepare_sensor_stamp("pointcloud", 5_000_000_000))
        self.assertFalse(node._stamp_ready_for_processing(5_000_000_000))

    def test_missing_tf_keeps_pair_retryable_until_dynamic_tf_returns(self):
        node = self._node()
        node.fusion_enabled = True
        node.image_sync_tolerance_sec = 0.05
        node.latest_camera_info = CameraInfo()
        node.latest_right_camera_info = None
        node._info_throttled = Mock()
        node._publish_cones = Mock()
        pointcloud = PointCloud2()
        pointcloud.header.stamp = TimeMsg(sec=1)
        boxes = BoundingBoxes()
        boxes.image_header.stamp = TimeMsg(sec=1)
        node.pointcloud_buffer.add(1_000_000_000, pointcloud)
        node.bbox_buffer.add(1_000_000_000, boxes)
        recovered = ConeArrayWithCovariance()
        node._run_lidar_camera_fusion = Mock(
            side_effect=[TransformException("dynamic TF unavailable"), recovered]
        )

        node._try_publish_fusion()

        self.assertEqual(len(node.pointcloud_buffer), 1)
        self.assertEqual(len(node.bbox_buffer), 1)
        node._publish_cones.assert_not_called()

        node._try_publish_fusion()

        self.assertEqual(len(node.pointcloud_buffer), 0)
        self.assertEqual(len(node.bbox_buffer), 0)
        node._publish_cones.assert_called_once_with(recovered)

    def test_missing_tf_pair_does_not_block_newer_ready_pair(self):
        node = self._node()
        node.fusion_enabled = True
        node.sync_tolerance_sec = 0.05
        node.image_sync_tolerance_sec = 0.05
        node.latest_camera_info = CameraInfo()
        node.latest_right_camera_info = None
        node._info_throttled = Mock()
        node._publish_cones = Mock()

        for stamp_sec in (1, 2):
            pointcloud = PointCloud2()
            pointcloud.header.stamp = TimeMsg(sec=stamp_sec)
            boxes = BoundingBoxes()
            boxes.image_header.stamp = TimeMsg(sec=stamp_sec)
            stamp_ns = stamp_sec * 1_000_000_000
            node.pointcloud_buffer.add(stamp_ns, pointcloud)
            node.bbox_buffer.add(stamp_ns, boxes)

        recovered = ConeArrayWithCovariance()
        node._run_lidar_camera_fusion = Mock(
            side_effect=[TransformException("old epoch TF unavailable"), recovered]
        )

        node._try_publish_fusion()

        self.assertEqual(len(node.pointcloud_buffer), 0)
        self.assertEqual(len(node.bbox_buffer), 0)
        self.assertEqual(node._run_lidar_camera_fusion.call_count, 2)
        node._publish_cones.assert_called_once_with(recovered)

    def test_previous_epoch_completion_cannot_publish_or_consume_reused_stamp(self):
        node = self._node()
        worker = self._DeferredWorker()
        node._fusion_worker = worker
        node.fusion_enabled = True
        node.image_sync_tolerance_sec = 0.05
        node.latest_camera_info = CameraInfo()
        node._publish_cones = Mock()
        node._info_throttled = Mock()

        old_cloud, old_boxes = self._fusion_messages()
        stamp_ns = 1_000_000_000
        node.pointcloud_buffer.add(stamp_ns, old_cloud)
        node.bbox_buffer.add(stamp_ns, old_boxes)
        node._try_publish_fusion()
        self.assertEqual(len(worker.jobs), 1)
        stale_job = worker.jobs[0]

        node._clear_fusion_epoch(epoch_start_ros_time_ns=500_000_000)
        node.latest_camera_info = CameraInfo()
        new_cloud, new_boxes = self._fusion_messages()
        node.pointcloud_buffer.add(stamp_ns, new_cloud)
        node.bbox_buffer.add(stamp_ns, new_boxes)
        new_cloud_entry = node.pointcloud_buffer.entries()[0]
        new_bbox_entry = node.bbox_buffer.entries()[0]

        node._commit_fusion_completion(
            WorkerCompletion(
                job=stale_job,
                result=FusionComputation(ConeArrayWithCovariance()),
            )
        )

        node._publish_cones.assert_not_called()
        self.assertTrue(
            any(
                entry is new_cloud_entry
                for entry in node.pointcloud_buffer.entries()
            )
        )
        self.assertTrue(
            any(
                entry is new_bbox_entry for entry in node.bbox_buffer.entries()
            )
        )
        self.assertEqual(worker.clear_pending_calls, 1)
        self.assertEqual(len(worker.jobs), 2)
        self.assertEqual(worker.jobs[1].generation, node._clock_generation)

    def test_fusion_completion_waits_for_clock_settle_before_publish(self):
        node = self._node()
        node.fusion_enabled = True
        node.image_sync_tolerance_sec = 0.05
        node.latest_camera_info = CameraInfo()
        node._publish_cones = Mock()
        node._info_throttled = Mock()
        node.output_commit_settle_ns = 100_000_000
        now_ns = [10_050_000_000]
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now_ns[0]),
            ros_time_is_active=True,
        )
        cloud, boxes = self._fusion_messages(stamp_sec=10)
        node.pointcloud_buffer.add(10_000_000_000, cloud)
        node.bbox_buffer.add(10_000_000_000, boxes)
        result = ConeArrayWithCovariance()
        node._run_lidar_camera_fusion = Mock(return_value=result)

        node._try_publish_fusion()

        node._publish_cones.assert_not_called()
        self.assertIsNotNone(node._deferred_fusion_completion)
        self.assertEqual(len(node.pointcloud_buffer), 1)
        self.assertEqual(len(node.bbox_buffer), 1)

        now_ns[0] = 10_100_000_000
        node._retry_deferred_fusion_completion()

        node._publish_cones.assert_called_once_with(result)
        self.assertIsNone(node._deferred_fusion_completion)
        self.assertEqual(len(node.pointcloud_buffer), 0)
        self.assertEqual(len(node.bbox_buffer), 0)

    def test_out_of_order_sensor_stamp_is_dropped_without_epoch_reset(self):
        node = self._node()
        old_stamp = 800_000_000
        node.image_buffer.add(old_stamp, "left")
        node.right_image_buffer.add(old_stamp, "right")
        node.stereo_buffer.add(old_stamp, "stereo")
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

        accepted = node._prepare_sensor_stamp("left_image", 0)

        self.assertFalse(accepted)
        self.assertEqual(len(node.image_buffer), 1)
        self.assertEqual(len(node.right_image_buffer), 1)
        self.assertEqual(len(node.stereo_buffer), 1)
        self.assertEqual(len(node.pointcloud_buffer), 1)
        self.assertGreater(len(node.bbox_buffer), 0)
        self.assertEqual(node.latest_stream_stamp_ns, {"left_image": old_stamp})
        self.assertEqual(node.sensor_epoch, 0)

    def test_positive_clock_tick_does_not_start_new_epoch(self):
        node = self._node()
        node.image_buffer.add(9_900_000_000, "left")

        node._on_clock_jump(
            TimeJump(
                ClockChange.ROS_TIME_NO_CHANGE,
                Duration(nanoseconds=20_000_000),
            )
        )

        self.assertEqual(node.sensor_epoch, 0)
        self.assertEqual(len(node.image_buffer), 1)
        node._reset_tf_epoch.assert_not_called()

    def test_subthreshold_backward_clock_tick_does_not_start_new_epoch(self):
        node = self._node()

        node._on_clock_jump(
            TimeJump(
                ClockChange.ROS_TIME_NO_CHANGE,
                Duration(nanoseconds=-99_999_999),
            )
        )

        self.assertEqual(node.sensor_epoch, 0)
        node._reset_tf_epoch.assert_not_called()

    def test_malformed_clock_jump_does_not_start_new_epoch(self):
        node = self._node()

        node._on_clock_jump(None)
        node._on_clock_jump(SimpleNamespace(delta=object()))

        self.assertEqual(node.sensor_epoch, 0)
        node._reset_tf_epoch.assert_not_called()

    def test_backward_clock_jump_starts_new_sensor_and_tf_epoch(self):
        node = self._node()
        old_stamp = 800_000_000
        node.image_buffer.add(old_stamp, "left")
        node.right_image_buffer.add(old_stamp, "right")
        node.stereo_buffer.add(old_stamp, "stereo")
        node.pointcloud_buffer.add(old_stamp, "cloud")
        node.bbox_buffer.add(old_stamp, "bbox")
        node.latest_stream_stamp_ns["left_image"] = old_stamp
        node.latest_image = object()
        node.latest_right_image = object()
        node.latest_pointcloud = object()
        node.latest_bboxes = object()
        node.latest_camera_info = CameraInfo()
        node.latest_right_camera_info = CameraInfo()
        node.last_empty_key = ((1, 0), (1, 0))
        node.last_fusion_key = ((1, 0), (1, 0))

        node._on_clock_jump(
            TimeJump(
                ClockChange.ROS_TIME_NO_CHANGE,
                Duration(nanoseconds=-100_000_000),
            )
        )

        self.assertEqual(len(node.image_buffer), 0)
        self.assertEqual(len(node.right_image_buffer), 0)
        self.assertEqual(len(node.stereo_buffer), 0)
        self.assertEqual(len(node.pointcloud_buffer), 0)
        self.assertEqual(len(node.bbox_buffer), 0)
        self.assertEqual(node.latest_stream_stamp_ns, {})
        self.assertIsNone(node.latest_image)
        self.assertIsNone(node.latest_right_image)
        self.assertIsNone(node.latest_pointcloud)
        self.assertIsNone(node.latest_bboxes)
        self.assertIsNone(node.latest_camera_info)
        self.assertIsNone(node.latest_right_camera_info)
        self.assertIsNone(node.last_empty_key)
        self.assertIsNone(node.last_fusion_key)
        self.assertEqual(node.sensor_epoch, 1)
        self.assertEqual(node.epoch_start_ros_time_ns, 10_000_000_000)
        self.assertEqual(node.replay_guard_end_ros_time_ns, 10_100_000_000)
        node._reset_tf_epoch.assert_called_once_with()

    def test_ros_time_source_transition_starts_new_epoch(self):
        node = self._node()
        node.replay_guard_end_ros_time_ns = 10_100_000_000

        node._on_clock_jump(
            TimeJump(
                ClockChange.ROS_TIME_ACTIVATED,
                Duration(nanoseconds=0),
            )
        )

        self.assertEqual(node.sensor_epoch, 1)
        self.assertIsNone(node.replay_guard_end_ros_time_ns)
        node._reset_tf_epoch.assert_called_once_with()

    def test_input_gate_detects_clock_rollback_without_jump_callback(self):
        node = self._node()
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=9_800_000_000),
            ros_time_is_active=False,
        )

        accepted = node._prepare_sensor_stamp("pointcloud", 9_800_000_000)

        self.assertTrue(accepted)
        self.assertEqual(node.sensor_epoch, 1)
        self.assertEqual(node.epoch_start_ros_time_ns, 9_800_000_000)
        self.assertEqual(node.replay_guard_end_ros_time_ns, 10_000_000_000)
        node._reset_tf_epoch.assert_called_once_with()

    def test_input_fallback_and_jump_callback_reset_same_rollback_once(self):
        node = self._node()
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=9_800_000_000),
            ros_time_is_active=True,
        )

        self.assertTrue(
            node._prepare_sensor_stamp("pointcloud", 9_800_000_000)
        )
        node._on_clock_jump(
            TimeJump(
                ClockChange.ROS_TIME_NO_CHANGE,
                Duration(nanoseconds=-200_000_000),
            )
        )

        self.assertEqual(node.sensor_epoch, 1)
        self.assertEqual(node.replay_guard_end_ros_time_ns, 10_000_000_000)
        node._reset_tf_epoch.assert_called_once_with()

    def test_pending_fallback_does_not_hide_later_distinct_rollback(self):
        node = self._node()
        now_ns = [9_800_000_000]
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now_ns[0]),
            ros_time_is_active=True,
        )
        self.assertTrue(
            node._prepare_sensor_stamp("pointcloud", 9_800_000_000)
        )

        # No sensor callback observed the catch-up to 10.1 s. A later,
        # distinct 10.1 -> 9.9 s rollback must not be mistaken for the delayed
        # callback of the already-handled 10.0 -> 9.8 s rollback.
        now_ns[0] = 9_900_000_000
        node._on_clock_jump(
            TimeJump(
                ClockChange.ROS_TIME_NO_CHANGE,
                Duration(nanoseconds=-200_000_000),
            )
        )

        self.assertEqual(node.sensor_epoch, 2)
        self.assertEqual(node.epoch_start_ros_time_ns, 9_900_000_000)
        self.assertEqual(node.replay_guard_end_ros_time_ns, 10_000_000_000)
        self.assertEqual(
            node.replay_guard_ends_ros_time_ns,
            [10_000_000_000, 10_100_000_000],
        )
        self.assertEqual(node._reset_tf_epoch.call_count, 2)

    def test_rollback_rejects_samples_outside_new_epoch_boundaries(self):
        node = self._node()
        now_ns = [9_800_000_000]
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now_ns[0]),
            ros_time_is_active=True,
        )

        node._on_clock_jump(
            TimeJump(
                ClockChange.ROS_TIME_NO_CHANGE,
                Duration(nanoseconds=-200_000_000),
            )
        )

        self.assertFalse(node._prepare_sensor_stamp("bboxes", 9_700_000_000))
        self.assertTrue(node._prepare_sensor_stamp("bboxes", 9_800_000_000))
        self.assertTrue(
            node._prepare_sensor_stamp("pointcloud", 9_890_000_000)
        )
        self.assertFalse(
            node._prepare_sensor_stamp("left_image", 9_890_000_001)
        )
        now_ns[0] = 9_950_000_000
        self.assertFalse(
            node._prepare_sensor_stamp("oracle", 10_000_000_000)
        )
        self.assertEqual(
            node.latest_stream_stamp_ns,
            {
                "bboxes": 9_800_000_000,
                "pointcloud": 9_890_000_000,
            },
        )

        now_ns[0] = 10_000_000_000
        self.assertTrue(node._prepare_sensor_stamp("bboxes", 10_040_000_000))
        self.assertIsNone(node.replay_guard_end_ros_time_ns)

    def test_replay_gate_accepts_later_cloud_within_bounded_clock_skew(self):
        node = self._node()
        now_ns = [290_000_000]
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now_ns[0]),
            ros_time_is_active=True,
        )
        node._on_clock_jump(
            TimeJump(
                ClockChange.ROS_TIME_NO_CHANGE,
                Duration(nanoseconds=-5_822_000_000),
            )
        )

        now_ns[0] = 822_000_000
        self.assertTrue(
            node._prepare_sensor_stamp("pointcloud", 902_000_000)
        )
        self.assertTrue(
            node._prepare_sensor_stamp("left_image", 912_000_000)
        )
        self.assertFalse(
            node._prepare_sensor_stamp("bboxes", 912_000_001)
        )
        now_ns[0] = 1_352_000_000
        self.assertTrue(
            node._prepare_sensor_stamp("pointcloud", 1_402_000_000)
        )
        self.assertEqual(
            node.replay_guard_end_ros_time_ns,
            6_112_000_000,
        )

    def test_nested_rollback_keeps_outer_fence_after_inner_catchup(self):
        node = self._node()
        now_ns = [0]
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now_ns[0]),
            ros_time_is_active=True,
        )

        node._on_clock_jump(
            TimeJump(
                ClockChange.ROS_TIME_NO_CHANGE,
                Duration(nanoseconds=-10_000_000_000),
            )
        )
        now_ns[0] = 1_000_000_000
        node._on_clock_jump(
            TimeJump(
                ClockChange.ROS_TIME_NO_CHANGE,
                Duration(nanoseconds=-1_000_000_000),
            )
        )

        self.assertEqual(
            node.replay_guard_ends_ros_time_ns,
            [2_000_000_000, 10_000_000_000],
        )
        self.assertFalse(
            node._prepare_sensor_stamp("pointcloud", 2_000_000_000)
        )

        now_ns[0] = 2_000_000_000
        self.assertTrue(
            node._prepare_sensor_stamp("pointcloud", 2_010_000_000)
        )
        self.assertEqual(node.replay_guard_end_ros_time_ns, 10_000_000_000)
        self.assertFalse(
            node._prepare_sensor_stamp("bboxes", 10_000_000_000)
        )

        now_ns[0] = 10_000_000_000
        self.assertTrue(
            node._prepare_sensor_stamp("bboxes", 10_010_000_000)
        )
        self.assertIsNone(node.replay_guard_end_ros_time_ns)

    def test_replay_future_bound_applies_at_zero_in_inactive_ros_time(self):
        node = self._node()
        node.epoch_start_ros_time_ns = 0
        node.replay_guard_end_ros_time_ns = 10_000_000_000
        node.last_observed_ros_time_ns = 0
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=0),
            ros_time_is_active=False,
        )

        self.assertTrue(node._prepare_sensor_stamp("pointcloud", 90_000_000))
        self.assertFalse(node._prepare_sensor_stamp("bboxes", 90_000_001))

    def test_rollback_replay_end_saturates_without_int64_overflow(self):
        node = self._node()
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=0),
            ros_time_is_active=True,
        )

        node._on_clock_jump(
            SimpleNamespace(
                clock_change=ClockChange.ROS_TIME_NO_CHANGE,
                delta=SimpleNamespace(nanoseconds=-(1 << 63)),
            )
        )

        self.assertEqual(node.sensor_epoch, 1)
        self.assertEqual(node.replay_guard_end_ros_time_ns, (1 << 63) - 1)
        self.assertIsNone(node._rollback_replay_end_ns("bad", -1))
        self.assertIsNone(node._rollback_replay_end_ns(0, "bad"))

    def test_non_positive_sensor_stamp_is_rejected(self):
        node = self._node()

        self.assertFalse(node._prepare_sensor_stamp("pointcloud", 0))
        self.assertFalse(node._prepare_sensor_stamp("pointcloud", -1))
        self.assertEqual(node.latest_stream_stamp_ns, {})

    def test_duplicate_sensor_stamp_is_rejected(self):
        node = self._node()

        self.assertTrue(node._prepare_sensor_stamp("pointcloud", 9_900_000_000))
        self.assertFalse(node._prepare_sensor_stamp("pointcloud", 9_900_000_000))
        self.assertEqual(
            node.latest_stream_stamp_ns,
            {"pointcloud": 9_900_000_000},
        )

    def test_time_parameters_reject_nonfinite_or_unsafe_contracts(self):
        node = self._node()
        for reset_threshold, future_lead in (
            (float("nan"), 0.05),
            (0.1, float("inf")),
            (0.1, 0.1),
            (1.0e308, 0.05),
            (0.1, 1.0e308),
            (float(1 << 63) / 1.0e9 + 1.0, 0.05),
        ):
            node.timestamp_reset_threshold_sec = reset_threshold
            node.max_future_stamp_lead_sec = future_lead
            with self.assertRaises(ValueError):
                node._validate_time_parameters()

    def test_raw_ros_stamp_must_be_canonical_and_nonzero(self):
        invalid_stamps = (
            TimeMsg(),
            TimeMsg(sec=-1),
            TimeMsg(sec=1, nanosec=1_000_000_000),
        )
        for stamp in invalid_stamps:
            with self.subTest(sec=stamp.sec, nanosec=stamp.nanosec):
                node = self._node()
                self.assertIsNone(
                    node._prepare_sensor_message_stamp("pointcloud", stamp)
                )
                self.assertEqual(node.latest_stream_stamp_ns, {})

        node = self._node()
        self.assertEqual(
            node._prepare_sensor_message_stamp(
                "pointcloud",
                TimeMsg(nanosec=1),
            ),
            1,
        )

    def test_all_acquisition_callbacks_use_raw_stamp_gate(self):
        cases = []

        left = Image()
        left.header.stamp = TimeMsg(sec=1)
        cases.append(("_image_callback", left, "left_image", left.header.stamp))

        right = Image()
        right.header.stamp = TimeMsg(sec=2)
        cases.append(
            ("_right_image_callback", right, "right_image", right.header.stamp)
        )

        cloud = PointCloud2()
        cloud.header.stamp = TimeMsg(sec=3)
        cases.append(
            ("_pointcloud_callback", cloud, "pointcloud", cloud.header.stamp)
        )

        boxes = BoundingBoxes()
        boxes.header.stamp = TimeMsg(sec=4)
        boxes.image_header.stamp = TimeMsg(sec=5)
        cases.append(
            ("_bbox_callback", boxes, "bboxes", boxes.image_header.stamp)
        )

        oracle = ConeArrayWithCovariance()
        oracle.header.stamp = TimeMsg(sec=6)
        cases.append(
            ("_oracle_callback", oracle, "oracle", oracle.header.stamp)
        )

        left_info = CameraInfo()
        left_info.header.stamp = TimeMsg(sec=7)
        cases.append(
            (
                "_camera_info_callback",
                left_info,
                "left_camera_info",
                left_info.header.stamp,
            )
        )

        right_info = CameraInfo()
        right_info.header.stamp = TimeMsg(sec=8)
        cases.append(
            (
                "_right_camera_info_callback",
                right_info,
                "right_camera_info",
                right_info.header.stamp,
            )
        )

        for callback_name, message, stream_name, stamp in cases:
            with self.subTest(callback=callback_name):
                node = object.__new__(PerceptionBaselineNode)
                node._prepare_sensor_message_stamp = Mock(return_value=None)

                getattr(node, callback_name)(message)

                node._prepare_sensor_message_stamp.assert_called_once_with(
                    stream_name,
                    stamp,
                )

    def test_bbox_does_not_fallback_from_malformed_image_acquisition_stamp(self):
        node = self._node()
        boxes = BoundingBoxes()
        boxes.header.stamp = TimeMsg(sec=2)
        boxes.image_header.stamp = TimeMsg(
            sec=1,
            nanosec=1_000_000_000,
        )

        node._bbox_callback(boxes)

        self.assertEqual(len(node.bbox_buffer), 0)
        self.assertIsNone(node.latest_bboxes)

    def test_previous_epoch_future_stamp_is_rejected(self):
        node = self._node()
        node.last_observed_ros_time_ns = 0
        node.epoch_start_ros_time_ns = 0
        node.replay_guard_end_ros_time_ns = 10_000_000_000
        node.replay_guard_ends_ros_time_ns = [10_000_000_000]
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=0),
            ros_time_is_active=True,
        )

        accepted = node._prepare_sensor_stamp("bboxes", 11_000_000_000)

        self.assertFalse(accepted)
        self.assertEqual(node.latest_stream_stamp_ns, {})
        self.assertEqual(len(node.bbox_buffer), 0)


class CalibrationAndTfTest(unittest.TestCase):
    def test_fusion_array_uses_bbox_time_as_canonical_timestamp(self):
        node = object.__new__(PerceptionBaselineNode)
        node.output_frame = "base_footprint"
        node.camera_frame = "zed_left_camera_optical_frame"
        node.publish_unmatched_lidar_clusters = False
        node._warn_throttled = lambda *args, **kwargs: None
        cloud = PointCloud2()
        cloud.header.stamp = TimeMsg(sec=10)
        boxes = BoundingBoxes()
        boxes.image_header.stamp = TimeMsg(sec=11, nanosec=42)
        boxes.image_header.frame_id = "zed_left_camera_optical_frame"

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

    def test_rectified_stereo_context_does_not_reapply_raw_distortion(self):
        node = object.__new__(PerceptionBaselineNode)
        node.right_camera_frame = "right_optical"
        node._warn_throttled = Mock()
        left_info = CameraInfo()
        right_info = CameraInfo()
        for info, translation in ((left_info, 0.0), (right_info, -24.0)):
            info.p = [
                200.0, 0.0, 60.0, translation,
                0.0, 200.0, 40.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]
            info.d = [0.1, -0.2, 0.01, 0.02, 0.03]
        left_image = Image(width=120, height=80)
        right_image = Image(width=120, height=80)
        right_from_left = np.eye(4, dtype=np.float64)
        right_from_left[0, 3] = -0.12

        with patch.object(node, "_validate_camera_info"), patch.object(
            node,
            "_resolve_stereo_transform",
            return_value=right_from_left,
        ), patch.object(
            node_module,
            "image_message_to_numpy",
            return_value=np.zeros((80, 120, 3), dtype=np.uint8),
        ):
            context = node._prepare_stereo_context(
                left_info,
                right_info,
                TimeMsg(sec=1),
                left_image,
                right_image,
            )

        self.assertIsNotNone(context)
        np.testing.assert_array_equal(context.left_distortion, np.zeros(5))
        np.testing.assert_array_equal(context.right_distortion, np.zeros(5))

    def test_camera_info_must_match_configured_projection_frame(self):
        node = object.__new__(PerceptionBaselineNode)
        info = CameraInfo()
        info.width = 100
        info.height = 100
        info.header.frame_id = "wrong_optical_frame"
        info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]

        with self.assertRaisesRegex(RuntimeError, "configured camera frame"):
            node._validate_camera_info(
                info,
                expected_frame="zed_left_camera_optical_frame",
            )

    def test_empty_camera_and_bbox_frames_are_rejected(self):
        node = object.__new__(PerceptionBaselineNode)
        info = CameraInfo()
        info.width = 100
        info.height = 100
        info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]

        with self.assertRaisesRegex(RuntimeError, "CameraInfo frame_id is empty"):
            node._validate_camera_info(
                info,
                expected_frame="zed_left_camera_optical_frame",
            )
        with self.assertRaisesRegex(
            RuntimeError,
            "BoundingBoxes.image_header frame_id is empty",
        ):
            node._validate_camera_frame(
                "",
                "zed_left_camera_optical_frame",
                "BoundingBoxes.image_header",
            )

    def test_empty_camera_info_does_not_hide_wrong_image_frame(self):
        node = object.__new__(PerceptionBaselineNode)
        info = CameraInfo()
        info.width = 100
        info.height = 100
        info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]
        image = Image()
        image.width = 100
        image.height = 100
        image.header.frame_id = "wrong_optical_frame"

        with self.assertRaisesRegex(RuntimeError, "Image frame_id"):
            node._validate_camera_info(
                info,
                image=image,
                expected_frame="zed_left_camera_optical_frame",
            )

    def test_camera_info_does_not_hide_wrong_bbox_image_frame(self):
        node = object.__new__(PerceptionBaselineNode)
        node.camera_frame = "zed_left_camera_optical_frame"
        info = CameraInfo()
        info.width = 100
        info.height = 100
        info.header.frame_id = "zed_left_camera_optical_frame"
        info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]
        boxes = BoundingBoxes()
        boxes.image_header.frame_id = "wrong_optical_frame"

        with self.assertRaisesRegex(RuntimeError, "BoundingBoxes.image_header"):
            node._run_lidar_camera_fusion(PointCloud2(), boxes, info)

    def test_nonfinite_or_out_of_range_bboxes_are_rejected(self):
        node = object.__new__(PerceptionBaselineNode)
        node.min_bbox_probability = 0.25
        node.clip_bboxes_to_image = False
        info = CameraInfo()
        info.width = 100
        info.height = 100
        boxes = BoundingBoxes()
        for probability, xmin, ymin, xmax, ymax in (
            (0.9, 10.0, 10.0, 20.0, 20.0),
            (float("nan"), 10.0, 10.0, 20.0, 20.0),
            (1.1, 10.0, 10.0, 20.0, 20.0),
            (0.9, float("-inf"), 10.0, 20.0, 20.0),
            (0.9, 10.0, 10.0, float("inf"), 20.0),
        ):
            bbox = BoundingBox()
            bbox.type = BoundingBox.PIXEL
            bbox.color = "blue"
            bbox.probability = probability
            bbox.xmin = xmin
            bbox.ymin = ymin
            bbox.xmax = xmax
            bbox.ymax = ymax
            boxes.bounding_boxes.append(bbox)

        detections = node._extract_detections(boxes, info)

        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].xmin, 10.0)

    def test_stereo_enabled_requires_sift_capable_opencv(self):
        node = object.__new__(PerceptionBaselineNode)
        node.stereo_fallback_enabled = True

        with patch.object(node_module, "_sift_available", return_value=False):
            with self.assertRaisesRegex(RuntimeError, "cv2.SIFT_create"):
                node._validate_stereo_capability()

    def test_oracle_frame_transform_moves_points_and_rotates_covariance(self):
        node = object.__new__(PerceptionBaselineNode)
        node.default_variance_x = 0.04
        node.default_variance_y = 0.04
        node.min_variance = 1.0e-4
        cone = ConeWithCovariance()
        cone.point.x = 1.0
        cone.point.y = 2.0
        cone.covariance = [4.0, 0.0, 0.0, 1.0]
        transform = np.array(
            [
                [0.0, -1.0, 0.0, 10.0],
                [1.0, 0.0, 0.0, 20.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        result = node._normalize_cones([cone], transform)

        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0].point.x, 8.0)
        self.assertAlmostEqual(result[0].point.y, 21.0)
        np.testing.assert_allclose(result[0].covariance, [1.0, 0.0, 0.0, 4.0])

    def test_oracle_normalization_rejects_nan_z_and_nonfinite_transformed_xy(self):
        node = object.__new__(PerceptionBaselineNode)
        node.default_variance_x = 0.04
        node.default_variance_y = 0.04
        node.min_variance = 1.0e-4
        nan_z = ConeWithCovariance()
        nan_z.point.x = 1.0
        nan_z.point.y = 2.0
        nan_z.point.z = float("nan")
        finite = ConeWithCovariance()
        finite.point.x = 1.0
        finite.point.y = 2.0
        transform = np.eye(4)
        transform[0, 3] = float("nan")

        self.assertEqual(node._normalize_cones([nan_z]), [])
        self.assertEqual(node._normalize_cones([finite], transform), [])

    def test_oracle_callback_uses_sensor_epoch_gate(self):
        node = object.__new__(PerceptionBaselineNode)
        node._prepare_sensor_stamp = Mock(return_value=False)
        node._publish_cones = Mock()
        msg = ConeArrayWithCovariance()
        msg.header.stamp = TimeMsg(sec=12, nanosec=34)

        node._oracle_callback(msg)

        node._prepare_sensor_stamp.assert_called_once_with(
            "oracle",
            12_000_000_034,
        )
        node._publish_cones.assert_not_called()

    def test_tf_buffer_clear_preserves_static_and_accepts_new_epoch_dynamic_tf(self):
        buffer = Buffer()
        static_tf = TransformStamped()
        static_tf.header.frame_id = "base"
        static_tf.child_frame_id = "sensor"
        static_tf.transform.rotation.w = 1.0
        buffer.set_transform_static(static_tf, "test")
        old_dynamic = TransformStamped()
        old_dynamic.header.frame_id = "map"
        old_dynamic.child_frame_id = "base"
        old_dynamic.header.stamp = TimeMsg(sec=100)
        old_dynamic.transform.rotation.w = 1.0
        buffer.set_transform(old_dynamic, "test")

        buffer.clear()

        self.assertTrue(buffer.can_transform("base", "sensor", Time()))
        self.assertFalse(buffer.can_transform("map", "base", Time()))
        new_dynamic = TransformStamped()
        new_dynamic.header.frame_id = "map"
        new_dynamic.child_frame_id = "base"
        new_dynamic.header.stamp = TimeMsg(sec=1)
        new_dynamic.transform.rotation.w = 1.0
        buffer.set_transform(new_dynamic, "test")
        self.assertTrue(buffer.can_transform("map", "base", Time()))

    def test_tf_epoch_reset_reuses_cleared_buffer_after_unregister(self):
        node = object.__new__(PerceptionBaselineNode)
        old_buffer = Mock()
        old_listener = Mock()
        new_listener = Mock()
        node.tf_buffer = old_buffer
        node.tf_listener = old_listener
        node._warn_throttled = Mock()

        with patch.object(node_module, "Buffer") as buffer_factory, patch.object(
            node_module,
            "TransformListener",
            return_value=new_listener,
        ) as listener_factory:
            node._reset_tf_epoch()

        old_listener.unregister.assert_called_once_with()
        old_buffer.clear.assert_called_once_with()
        buffer_factory.assert_not_called()
        listener_factory.assert_called_once_with(old_buffer, node)
        self.assertIs(node.tf_buffer, old_buffer)
        self.assertIs(node.tf_listener, new_listener)

    def test_tf_epoch_reset_retains_static_tf_after_writer_is_gone(self):
        node = object.__new__(PerceptionBaselineNode)
        buffer = Buffer()
        static_tf = TransformStamped()
        static_tf.header.frame_id = "base"
        static_tf.child_frame_id = "sensor"
        static_tf.transform.rotation.w = 1.0
        buffer.set_transform_static(static_tf, "one-shot-writer")
        node.tf_buffer = buffer
        node.tf_listener = Mock()
        node._warn_throttled = Mock()
        new_listener = Mock()

        with patch.object(
            node_module,
            "TransformListener",
            return_value=new_listener,
        ):
            node._reset_tf_epoch()

        self.assertIs(node.tf_buffer, buffer)
        self.assertTrue(buffer.can_transform("base", "sensor", Time()))
        self.assertIs(node.tf_listener, new_listener)

    def test_tf_epoch_reset_uses_new_buffer_if_unregister_fails(self):
        node = object.__new__(PerceptionBaselineNode)
        old_buffer = Mock()
        old_listener = Mock()
        old_listener.unregister.side_effect = RuntimeError("still active")
        new_buffer = Mock()
        new_listener = Mock()
        node.tf_buffer = old_buffer
        node.tf_listener = old_listener
        node._warn_throttled = Mock()

        with patch.object(
            node_module,
            "Buffer",
            return_value=new_buffer,
        ), patch.object(
            node_module,
            "TransformListener",
            return_value=new_listener,
        ):
            node._reset_tf_epoch()

        old_buffer.clear.assert_called_once_with()
        self.assertIs(node.tf_buffer, new_buffer)
        self.assertIs(node.tf_listener, new_listener)
        node._warn_throttled.assert_called_once()

    def test_tf_lookup_uses_message_timestamp(self):
        node = object.__new__(PerceptionBaselineNode)
        captured = {}

        class FakeBuffer:
            def lookup_transform(self, target, source, stamp, timeout):
                captured["target"] = target
                captured["source"] = source
                captured["nanoseconds"] = stamp.nanoseconds
                captured["timeout_ns"] = timeout.nanoseconds
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
        self.assertEqual(captured["timeout_ns"], 0)
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
                captured["timeout_ns"] = timeout.nanoseconds
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
        self.assertEqual(captured["timeout_ns"], 0)
        np.testing.assert_allclose(matrix[:3, 3], [1.0, 0.0, 0.0])

    def test_covariance_uses_visual_source_contract(self):
        node = object.__new__(PerceptionBaselineNode)
        node.sparse_variance_x = 0.1
        node.sparse_variance_y = 0.2
        node.monocular_variance_x = 0.3
        node.monocular_variance_y = 0.4
        node.horizontal_clip_variance_x = 0.7
        node.horizontal_clip_variance_y = 1.4
        node.stereo_variance_x = 0.5
        node.stereo_variance_y = 0.6
        node.lidar_only_variance_x = 0.7
        node.lidar_only_variance_y = 0.8
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
            source="stereo_rektnet_pnp_sift",
        )

        cone = node._cluster_to_cone(cluster)

        self.assertEqual(list(cone.covariance), [0.5, 0.0, 0.0, 0.6])

    def test_covariance_covers_all_sources_range_term_and_minimum(self):
        node = object.__new__(PerceptionBaselineNode)
        node.sparse_variance_x = 0.10
        node.sparse_variance_y = 0.20
        node.monocular_variance_x = 0.30
        node.monocular_variance_y = 0.40
        node.horizontal_clip_variance_x = 0.70
        node.horizontal_clip_variance_y = 1.40
        node.stereo_variance_x = 0.50
        node.stereo_variance_y = 0.60
        node.fused_variance_x = 0.04
        node.fused_variance_y = 0.05
        node.range_variance_scale = 0.02
        node.min_variance = 0.01

        expected = {
            "cluster": (0.14, 0.15),
            "lidar": (0.14, 0.15),
            "sparse": (0.20, 0.30),
            "monocular": (0.40, 0.50),
            "monocular_horizontal_clip": (0.80, 1.50),
            "stereo_rektnet_pnp_sift": (0.60, 0.70),
        }
        for source, (expected_x, expected_y) in expected.items():
            with self.subTest(source=source):
                cluster = Cluster(
                    points_base=np.asarray([[5.0, 0.0, 0.0]]),
                    points_camera=np.asarray([[0.0, 0.0, 5.0]]),
                    centroid_base=np.asarray([5.0, 0.0, 0.0]),
                    range_m=5.0,
                    indices=np.empty((0,), dtype=np.int64),
                    source=source,
                )
                cone = node._cluster_to_cone(cluster)
                np.testing.assert_allclose(
                    cone.covariance,
                    [expected_x, 0.0, 0.0, expected_y],
                )

        node.fused_variance_x = 0.001
        node.fused_variance_y = 0.002
        node.range_variance_scale = 0.0
        cluster.source = "lidar"
        cone = node._cluster_to_cone(cluster)
        self.assertEqual(list(cone.covariance), [0.01, 0.0, 0.0, 0.01])


class LidarOnlyClusterEmitTest(unittest.TestCase):
    def _node(self):
        node = object.__new__(PerceptionBaselineNode)
        node.lidar_only_variance_x = 0.2
        node.lidar_only_variance_y = 0.3
        node.fused_variance_x = 0.04
        node.fused_variance_y = 0.04
        node.sparse_variance_x = 0.1
        node.sparse_variance_y = 0.1
        node.monocular_variance_x = 0.3
        node.monocular_variance_y = 0.3
        node.stereo_variance_x = 0.5
        node.stereo_variance_y = 0.5
        node.range_variance_scale = 0.0
        node.min_variance = 1.0e-4
        node.lidar_only_dedup_radius_m = 0.5
        node._info_throttled = lambda *args, **kwargs: None
        return node

    def _cluster(self, x, y, indices):
        return Cluster(
            points_base=np.asarray([[x, y, 0.0]]),
            points_camera=np.asarray([[0.0, 0.0, x]]),
            centroid_base=np.asarray([x, y, 0.0]),
            range_m=float(np.hypot(x, y)),
            indices=np.asarray(indices, dtype=np.int64),
            source="cluster",
        )

    def test_unmatched_cluster_becomes_unknown_cone(self):
        node = self._node()
        msg = ConeArrayWithCovariance()

        node._append_unmatched_lidar_cluster_cones(
            msg, [self._cluster(4.0, 2.0, [0, 1, 2])], []
        )

        self.assertEqual(len(msg.unknown_color_cones), 1)
        self.assertEqual(list(msg.blue_cones), [])
        cone = msg.unknown_color_cones[0]
        self.assertAlmostEqual(cone.point.x, 4.0)
        self.assertAlmostEqual(cone.point.y, 2.0)
        # lidar_only source variance, with range scaling disabled.
        self.assertEqual(list(cone.covariance), [0.2, 0.0, 0.0, 0.3])

    def test_cluster_consumed_by_assignment_is_skipped(self):
        node = self._node()
        msg = ConeArrayWithCovariance()
        cluster = self._cluster(4.0, 2.0, [0, 1, 2])
        matched = Cluster(
            points_base=cluster.points_base,
            points_camera=cluster.points_camera,
            centroid_base=cluster.centroid_base,
            range_m=cluster.range_m,
            indices=cluster.indices,
            source="lidar",
            consumed_indices=np.asarray([0, 1, 2], dtype=np.int64),
        )
        assignment = Assignment(
            detection_index=0,
            detection=Detection("blue", 0.9, 0.0, 0.0, 10.0, 20.0),
            cluster=matched,
            source="lidar",
            support_count=3,
        )

        node._append_unmatched_lidar_cluster_cones(msg, [cluster], [assignment])

        self.assertEqual(len(msg.unknown_color_cones), 0)

    def test_cluster_near_existing_cone_is_deduped(self):
        node = self._node()
        msg = ConeArrayWithCovariance()
        near = ConeWithCovariance()
        near.point.x = 4.1
        near.point.y = 2.0
        msg.blue_cones.append(near)

        node._append_unmatched_lidar_cluster_cones(
            msg, [self._cluster(4.0, 2.0, [5, 6, 7])], []
        )

        self.assertEqual(len(msg.unknown_color_cones), 0)


if __name__ == "__main__":
    unittest.main()
