from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np
from sensor_msgs.msg import Image

from eufs_perception_baseline.yolov8_bbox_node import YoloV8BBoxNode


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
    def _callback_node(self):
        node = object.__new__(YoloV8BBoxNode)
        node.bbox_pub = _PublisherRecorder()
        node.debug_image_pub = None
        node.model = _FailingModel()
        node.confidence_threshold = 0.25
        node.iou_threshold = 0.45
        node.imgsz = 640
        node.max_det = 100
        node.device = ""
        node.class_map = {"blue_cone": "blue"}
        node.unknown_color_policy = "unknown"
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

    def test_model_class_map_must_overlap_loaded_model_names(self):
        node = object.__new__(YoloV8BBoxNode)
        node.model = _FailingModel()
        node.class_map = {"blue_cone": "blue"}
        node._validate_model_classes()

        node.class_map = {"yellow_cone": "yellow"}
        with self.assertRaisesRegex(RuntimeError, "does not match"):
            node._validate_model_classes()

    def test_image_conversion_failure_drops_frame(self):
        node = self._callback_node()
        logger = _LoggerRecorder()
        image = self._image_message()

        with patch.object(YoloV8BBoxNode, "get_logger", return_value=logger), patch(
            "eufs_perception_baseline.yolov8_bbox_node.image_message_to_numpy",
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
            "eufs_perception_baseline.yolov8_bbox_node.image_message_to_numpy",
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
            "eufs_perception_baseline.yolov8_bbox_node.image_message_to_numpy",
            return_value=np.zeros((1, 1, 3), dtype=np.uint8),
        ), patch(
            "eufs_perception_baseline.yolov8_bbox_node."
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
            "eufs_perception_baseline.yolov8_bbox_node.image_message_to_numpy",
            return_value=np.zeros((1, 1, 3), dtype=np.uint8),
        ), patch(
            "eufs_perception_baseline.yolov8_bbox_node."
            "detections_from_ultralytics_results",
            return_value=[],
        ):
            node._image_callback(image)

        self.assertEqual(len(node.bbox_pub.messages), 1)
        self.assert_empty_detection_for_same_frame(
            image,
            node.bbox_pub.messages[0],
        )


if __name__ == "__main__":
    unittest.main()
