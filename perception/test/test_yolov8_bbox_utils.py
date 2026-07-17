import unittest

import numpy as np

from hyu_perception.yolov8_bbox_utils import (
    detections_from_ultralytics_results,
    looks_like_coco_pretrained_yolov8_weight,
    normalize_color,
    parse_class_map,
)


class FakeBoxes:
    def __init__(self, xyxy, conf, cls):
        self.xyxy = np.asarray(xyxy, dtype=np.float64)
        self.conf = np.asarray(conf, dtype=np.float64)
        self.cls = np.asarray(cls, dtype=np.float64)


class FakeResult:
    def __init__(self, boxes, names):
        self.boxes = boxes
        self.names = names


class YoloV8BBoxUtilsTest(unittest.TestCase):
    def test_parse_class_map_accepts_aliases(self):
        parsed = parse_class_map(
            "blue_cone:blue,yellow cone=yellow,big-orange:big_orange"
        )

        self.assertEqual(
            parsed,
            {
                "blue_cone": "blue",
                "yellow_cone": "yellow",
                "big_orange": "big_orange",
            },
        )

    def test_normalize_color_infers_formula_student_colors(self):
        self.assertEqual(normalize_color("blue_cone"), "blue")
        self.assertEqual(normalize_color("yellow cone"), "yellow")
        self.assertEqual(normalize_color("big-orange-cone"), "big_orange")
        self.assertEqual(normalize_color("traffic light"), "unknown")
        self.assertIsNone(
            normalize_color("traffic light", unknown_color_policy="skip")
        )

    def test_detections_from_ultralytics_results_converts_pixel_boxes(self):
        result = FakeResult(
            boxes=FakeBoxes(
                xyxy=[[40, 20, 10, 60], [5, 5, 15, 15], [1, 1, 2, 2]],
                conf=[0.91, 0.20, 0.70],
                cls=[0, 1, 2],
            ),
            names={0: "blue_cone", 1: "yellow_cone", 2: "traffic light"},
        )

        detections = detections_from_ultralytics_results(
            [result],
            confidence_threshold=0.25,
            unknown_color_policy="skip",
        )

        self.assertEqual(len(detections), 1)
        detection = detections[0]
        self.assertEqual(detection.color, "blue")
        self.assertEqual(detection.probability, 0.91)
        self.assertEqual(
            (detection.xmin, detection.ymin, detection.xmax, detection.ymax),
            (
                10.0,
                20.0,
                40.0,
                60.0,
            ),
        )

    def test_detections_from_ultralytics_results_uses_class_map(self):
        result = FakeResult(
            boxes=FakeBoxes(xyxy=[[0, 1, 2, 3]], conf=[0.88], cls=[9]),
            names={9: "traffic light"},
        )

        detections = detections_from_ultralytics_results(
            [result],
            class_map={"traffic_light": "orange"},
            confidence_threshold=0.25,
        )

        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].color, "orange")

    def test_detections_from_ultralytics_results_accepts_empty_boxes(self):
        result = FakeResult(
            boxes=FakeBoxes(xyxy=[], conf=[], cls=[]),
            names={},
        )

        detections = detections_from_ultralytics_results(
            [result],
            confidence_threshold=0.25,
        )

        self.assertEqual(detections, [])

    def test_detections_from_ultralytics_results_skips_nonfinite_rows(self):
        result = FakeResult(
            boxes=FakeBoxes(
                xyxy=[
                    [0, 0, 10, 10],
                    [0, 0, float("inf"), 10],
                    [0, float("nan"), 10, 10],
                    [0, 0, 10, 10],
                    [0, 0, 10, 10],
                ],
                conf=[0.9, 0.9, 0.9, float("nan"), 0.9],
                cls=[0, 0, 0, 0, float("inf")],
            ),
            names={0: "blue_cone"},
        )

        detections = detections_from_ultralytics_results([result])

        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].color, "blue")

    def test_detections_from_ultralytics_results_skips_invalid_class_ids(self):
        result = FakeResult(
            boxes=FakeBoxes(
                xyxy=[[0, 0, 10, 10]] * 3,
                conf=[0.9, 0.9, 0.9],
                cls=[-1, 0.5, 0],
            ),
            names={0: "blue_cone"},
        )

        detections = detections_from_ultralytics_results([result])

        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].color, "blue")

    def test_looks_like_coco_pretrained_yolov8_weight(self):
        self.assertTrue(
            looks_like_coco_pretrained_yolov8_weight("/models/yolov8n.pt")
        )
        self.assertFalse(
            looks_like_coco_pretrained_yolov8_weight("/models/cone_yolov8n.pt")
        )
