import tempfile
import unittest
from pathlib import Path

from PIL import Image

from eufs_perception_baseline.fsoco_yolo_dataset import (
    ConversionError,
    FSOCO_CLASSES,
    convert_dataset,
    yolo_label_from_rectangle,
)


class FsocoYoloDatasetTest(unittest.TestCase):
    def test_yolo_label_from_rectangle_sorts_and_normalizes(self):
        label, clamped = yolo_label_from_rectangle(
            class_id=0,
            class_name="blue_cone",
            exterior=[[80, 60], [20, 10]],
            image_width=100,
            image_height=100,
        )

        self.assertFalse(clamped)
        self.assertIsNotNone(label)
        self.assertEqual(label.as_yolo_row(), "0 0.500000 0.350000 0.600000 0.500000")

    def test_yolo_label_from_rectangle_clamps_and_discards_zero_area(self):
        label, clamped = yolo_label_from_rectangle(
            class_id=0,
            class_name="blue_cone",
            exterior=[[-10, -10], [0, 10]],
            image_width=100,
            image_height=100,
        )

        self.assertTrue(clamped)
        self.assertIsNone(label)

    def test_convert_dataset_writes_yolo_layout(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir) / "fsoco"
            group = root / "team_a"
            (group / "ann").mkdir(parents=True)
            (group / "img").mkdir(parents=True)
            image_path = group / "img" / "team_a_0001.jpg"
            Image.new("RGB", (100, 80), "black").save(image_path)
            annotation_path = group / "ann" / "team_a_0001.jpg.json"
            annotation_path.write_text(
                """
                {
                  "size": {"width": 100, "height": 80},
                  "tags": [{"name": "train"}],
                  "objects": [
                    {
                      "classTitle": "blue_cone",
                      "geometryType": "rectangle",
                      "points": {"exterior": [[10, 20], [30, 60]]},
                      "tags": [{"name": "truncated"}]
                    }
                  ]
                }
                """,
                encoding="utf-8",
            )

            output = Path(tmpdir) / "out"
            result = convert_dataset(root, output, require_split_classes=False)

            self.assertEqual(result["records"], 1)
            self.assertTrue((output / "data.yaml").exists())
            self.assertTrue((output / "split_manifest.csv").exists())
            self.assertTrue((output / "reports" / "conversion_report.json").exists())
            self.assertTrue(any((output / "labels").glob("*/*.txt")))

    def test_convert_dataset_rejects_unknown_class(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir) / "fsoco"
            group = root / "team_a"
            (group / "ann").mkdir(parents=True)
            (group / "img").mkdir(parents=True)
            Image.new("RGB", (100, 80), "black").save(group / "img" / "bad.jpg")
            (group / "ann" / "bad.jpg.json").write_text(
                """
                {
                  "size": {"width": 100, "height": 80},
                  "objects": [
                    {
                      "classTitle": "not_a_cone",
                      "geometryType": "rectangle",
                      "points": {"exterior": [[10, 20], [30, 60]]}
                    }
                  ]
                }
                """,
                encoding="utf-8",
            )

            with self.assertRaises(ConversionError):
                convert_dataset(root, Path(tmpdir) / "out")

    def test_class_order_is_stable(self):
        self.assertEqual(
            FSOCO_CLASSES,
            (
                "blue_cone",
                "yellow_cone",
                "orange_cone",
                "large_orange_cone",
                "unknown_cone",
            ),
        )


if __name__ == "__main__":
    unittest.main()
