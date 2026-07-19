from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from sensor_msgs.msg import Image

from hyu_perception.latest_only_worker import WorkerCompletion
from hyu_perception.yolov8_bbox_node import (
    YoloComputation,
    YoloJob,
    YoloV8BBoxNode,
)


class _PublisherRecorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _LoggerRecorder:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


class _FailingModel:
    names = {0: "blue_cone"}

    def predict(self, **_kwargs):
        raise RuntimeError("synthetic inference failure")


class _SuccessfulModel:
    names = {0: "blue_cone"}

    def predict(self, **_kwargs):
        return [object()]


class YoloV8BBoxNodeTest(unittest.TestCase):
    class _InlineWorker:
        def __init__(self, node):
            self.node = node

        def submit(self, job):
            try:
                completion = WorkerCompletion(
                    job=job,
                    result=self.node._compute_yolo_job(job),
                )
            except BaseException as exc:  # noqa: BLE001 - test worker seam
                completion = WorkerCompletion(job=job, error=exc)
            self.node._commit_yolo_completion(completion)
            return True

        def clear_pending(self):
            pass

    def _callback_node(self):
        node = object.__new__(YoloV8BBoxNode)
        node.bbox_pub = _PublisherRecorder()
        node.keypoints_pub = None
        node.debug_image_pub = None
        node.model = _FailingModel()
        node.confidence_threshold = 0.25
        node.iou_threshold = 0.45
        node.imgsz = 640
        node.max_det = 100
        node.device = ""
        node.class_map = {"blue_cone": "blue"}
        node.unknown_color_policy = "unknown"
        node.output_commit_settle_sec = 0.1
        node.inference_mode = "all"
        node._clock_generation = 0
        node._shutting_down = False
        node._worker = self._InlineWorker(node)
        return node

    @staticmethod
    def _image_message():
        message = Image()
        message.header.stamp.sec = 123
        message.header.stamp.nanosec = 456
        message.header.frame_id = "zed_left_camera_optical_frame"
        message.height = 1
        message.width = 1
        message.encoding = "bgr8"
        message.step = 3
        message.data = bytes((1, 2, 3))
        return message

    def assert_empty_detection_for_same_frame(self, source, published):
        self.assertEqual(len(published.bounding_boxes), 0)
        self.assertEqual(published.header, source.header)
        self.assertEqual(published.image_header, source.header)

    def test_model_path_requires_existing_regular_pt_file(self):
        with TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            weight = root / "cone.pt"
            weight.write_bytes(b"local model")

            validated = YoloV8BBoxNode._validated_model_path(str(weight))

            self.assertEqual(validated, str(weight.resolve()))
            with self.assertRaisesRegex(RuntimeError, "existing local .pt"):
                YoloV8BBoxNode._validated_model_path(str(root / "missing.pt"))

            directory_named_like_weight = root / "directory.pt"
            directory_named_like_weight.mkdir()
            with self.assertRaisesRegex(RuntimeError, "regular file"):
                YoloV8BBoxNode._validated_model_path(
                    str(directory_named_like_weight)
                )

    def test_model_path_rejects_non_pt_artifact(self):
        with TemporaryDirectory() as temporary_directory:
            artifact = Path(temporary_directory) / "cone.onnx"
            artifact.write_bytes(b"local model")

            with self.assertRaisesRegex(RuntimeError, "local .pt"):
                YoloV8BBoxNode._validated_model_path(str(artifact))

    def test_model_path_rejects_unreadable_pt_artifact(self):
        with TemporaryDirectory() as temporary_directory:
            artifact = Path(temporary_directory) / "cone.pt"
            artifact.write_bytes(b"local model")

            with patch.object(Path, "open", side_effect=PermissionError):
                with self.assertRaisesRegex(RuntimeError, "not readable"):
                    YoloV8BBoxNode._validated_model_path(str(artifact))

    def test_model_must_contain_every_configured_class(self):
        node = object.__new__(YoloV8BBoxNode)
        node.model = _FailingModel()
        node.class_map = {"blue_cone": "blue"}
        node._validate_model_classes()

        node.class_map = {"yellow_cone": "yellow"}
        with self.assertRaisesRegex(RuntimeError, "missing configured cone classes"):
            node._validate_model_classes()

    def test_invalid_numeric_parameters_fail_closed(self):
        node = self._callback_node()
        node.timestamp_reset_threshold_sec = 0.1
        node.unknown_color_policy = "unknown"

        cases = (
            ("confidence_threshold", float("nan")),
            ("confidence_threshold", -0.1),
            ("iou_threshold", float("inf")),
            ("iou_threshold", 1.1),
            ("imgsz", 0),
            ("max_det", 0),
            ("timestamp_reset_threshold_sec", 0.0),
            ("timestamp_reset_threshold_sec", 1.0e20),
            ("timestamp_reset_threshold_sec", 1.0e308),
            ("output_commit_settle_sec", 1.0e308),
        )
        for attribute, invalid_value in cases:
            with self.subTest(attribute=attribute, value=invalid_value):
                original = getattr(node, attribute)
                setattr(node, attribute, invalid_value)
                with self.assertRaises(ValueError):
                    node._validate_parameters()
                setattr(node, attribute, original)

        node.class_map = {"blue_cone": "blue", "yellow_cone": "yellow"}
        with self.assertRaisesRegex(RuntimeError, "yellow_cone"):
            node._validate_model_classes()

    def test_image_conversion_failure_drops_frame(self):
        node = self._callback_node()
        logger = _LoggerRecorder()
        image = self._image_message()

        with patch.object(YoloV8BBoxNode, "get_logger", return_value=logger), patch(
            "hyu_perception.yolov8_bbox_node.image_message_to_numpy",
            side_effect=ValueError("bad encoding"),
        ):
            node._image_callback(image)

        self.assertEqual(node.bbox_pub.messages, [])
        self.assertIn("dropped invalid frame", logger.warnings[0])

    def test_inference_failure_drops_frame(self):
        node = self._callback_node()
        logger = _LoggerRecorder()
        image = self._image_message()

        with patch.object(YoloV8BBoxNode, "get_logger", return_value=logger), patch(
            "hyu_perception.yolov8_bbox_node.image_message_to_numpy",
            return_value=np.zeros((1, 1, 3), dtype=np.uint8),
        ):
            node._image_callback(image)

        self.assertEqual(node.bbox_pub.messages, [])
        self.assertIn("dropped frame", logger.warnings[0])

    def test_result_conversion_failure_drops_frame(self):
        node = self._callback_node()
        node.model = _SuccessfulModel()
        logger = _LoggerRecorder()
        image = self._image_message()

        with patch.object(YoloV8BBoxNode, "get_logger", return_value=logger), patch(
            "hyu_perception.yolov8_bbox_node.image_message_to_numpy",
            return_value=np.zeros((1, 1, 3), dtype=np.uint8),
        ), patch(
            "hyu_perception.yolov8_bbox_node."
            "detections_from_ultralytics_results",
            side_effect=ValueError("malformed result"),
        ):
            node._image_callback(image)

        self.assertEqual(node.bbox_pub.messages, [])
        self.assertIn("dropped frame", logger.warnings[0])

    def test_genuine_zero_detections_publishes_stamped_empty_boxes(self):
        node = self._callback_node()
        node.model = _SuccessfulModel()
        image = self._image_message()

        with patch(
            "hyu_perception.yolov8_bbox_node.image_message_to_numpy",
            return_value=np.zeros((1, 1, 3), dtype=np.uint8),
        ), patch(
            "hyu_perception.yolov8_bbox_node."
            "detections_from_ultralytics_results",
            return_value=[],
        ):
            node._image_callback(image)

        self.assertEqual(len(node.bbox_pub.messages), 1)
        self.assert_empty_detection_for_same_frame(
            image,
            node.bbox_pub.messages[0],
        )

    def test_completion_from_previous_clock_epoch_is_not_published(self):
        node = self._callback_node()
        image = self._image_message()
        stale_job = YoloJob(generation=0, image_msg=image)
        debug_publisher = _PublisherRecorder()
        node.debug_image_pub = debug_publisher

        node._clock_generation = 1
        node._commit_yolo_completion(
            WorkerCompletion(
                job=stale_job,
                result=YoloComputation([], debug_msg=Image()),
            )
        )

        self.assertEqual(node.bbox_pub.messages, [])
        self.assertEqual(debug_publisher.messages, [])

    def test_completion_from_current_clock_epoch_publishes_once(self):
        node = self._callback_node()
        image = self._image_message()
        current_job = YoloJob(generation=7, image_msg=image)
        node._clock_generation = 7

        node._commit_yolo_completion(
            WorkerCompletion(
                job=current_job,
                result=YoloComputation([]),
            )
        )

        self.assertEqual(len(node.bbox_pub.messages), 1)
        self.assert_empty_detection_for_same_frame(
            image,
            node.bbox_pub.messages[0],
        )

    def test_completion_waits_for_clock_settle_before_publish(self):
        node = self._callback_node()
        image = self._image_message()
        image.header.stamp.sec = 1
        image.header.stamp.nanosec = 0
        now_ns = [1_050_000_000]
        node.output_commit_settle_ns = 100_000_000
        node._deferred_completion = None
        node.get_clock = lambda: SimpleNamespace(
            ros_time_is_active=True,
            now=lambda: SimpleNamespace(nanoseconds=now_ns[0]),
        )
        completion = WorkerCompletion(
            job=YoloJob(generation=0, image_msg=image),
            result=YoloComputation([]),
        )

        node._commit_yolo_completion(completion)

        self.assertEqual(node.bbox_pub.messages, [])
        self.assertIs(node._deferred_completion, completion)

        now_ns[0] = 1_100_000_000
        node._retry_deferred_completion()

        self.assertEqual(len(node.bbox_pub.messages), 1)
        self.assertIsNone(node._deferred_completion)


class _SubmissionRecorder:
    def __init__(self):
        self.jobs = []

    def submit(self, job):
        self.jobs.append(job)


class LidarLockedElectionTest(unittest.TestCase):
    """The lock's invariant: one submission per LiDAR period whenever ANY
    frame exists in it -- under drops, boundary-straddling drops, and clock
    resets. The phase-wrap detector this replaced failed the straddle case:
    two dropped frames around the scan boundary produced no phase decrease,
    and the whole period silently went unsubmitted."""

    def _node(self):
        node = object.__new__(YoloV8BBoxNode)
        node.inference_mode = "lidar_locked"
        node.lidar_period_sec = 0.1
        node.lock_tolerance_sec = 0.0167
        node.timestamp_reset_threshold_sec = 0.1
        node._last_cloud_stamp = 0.0
        node._lidar_period_est = 0.1
        node._last_taken_stamp = None
        node._clock_generation = 0
        node._worker = _SubmissionRecorder()
        return node

    @staticmethod
    def _image(stamp_sec):
        message = Image()
        message.header.stamp.sec = int(stamp_sec)
        message.header.stamp.nanosec = int(round((stamp_sec % 1.0) * 1e9))
        return message

    def _feed(self, node, stamps):
        for stamp in stamps:
            node._image_callback(self._image(stamp))
        return [j.image_msg.header.stamp.sec
                + j.image_msg.header.stamp.nanosec * 1e-9
                for j in node._worker.jobs]

    def test_clean_30hz_stream_elects_one_frame_per_period(self):
        stamps = [round(n / 30.0, 3) for n in range(30)]   # 1 s of 30 Hz
        taken = self._feed(self._node(), stamps)
        self.assertEqual(len(taken), 10)
        deltas = [round(b - a, 3) for a, b in zip(taken, taken[1:])]
        self.assertTrue(all(0.084 <= d <= 0.134 for d in deltas), deltas)

    def test_boundary_straddling_double_drop_does_not_skip_the_period(self):
        # Frames at 0, .033, .067 then BOTH frames around the boundary
        # (.100, .133) drop; the period containing .167 must still submit.
        taken = self._feed(self._node(),
                           [0.0, 0.033, 0.067, 0.167, 0.200, 0.233])
        self.assertIn(0.167, [round(t, 3) for t in taken])

    def test_a_frameless_period_skips_and_the_next_recovers_immediately(self):
        # Nothing arrives in [0.084, 0.184]; the first frame after the outage
        # is submitted at once rather than waiting out another period.
        taken = self._feed(self._node(), [0.0, 0.033, 0.067, 0.300, 0.333])
        self.assertEqual([round(t, 3) for t in taken], [0.0, 0.3])

    def test_clock_jump_backwards_resets_the_lock(self):
        # A sim reset rewinds stamps; the lock must not wait for the old
        # timeline's next period.
        taken = self._feed(self._node(), [5.0, 0.5])
        self.assertEqual([round(t, 3) for t in taken], [5.0, 0.5])


if __name__ == "__main__":
    unittest.main()
