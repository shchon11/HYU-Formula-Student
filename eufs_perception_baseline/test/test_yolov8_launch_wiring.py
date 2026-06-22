from pathlib import Path
import unittest


class YoloV8LaunchWiringTest(unittest.TestCase):
    def setUp(self):
        package_root = Path(__file__).resolve().parents[1]
        self.launch_text = (
            package_root / "launch" / "perception_baseline.launch.py"
        ).read_text()
        self.config_text = (
            package_root / "config" / "perception_baseline.yaml"
        ).read_text()

    def test_launch_exposes_yolov8_bbox_source(self):
        self.assertIn("yaml.safe_load", self.launch_text)
        self.assertIn(
            '"simulated_bbox_topic"',
            self.launch_text,
        )
        self.assertIn('"bbox_topic"', self.launch_text)
        self.assertIn(
            "Deprecated alias for simulated_bbox_topic.",
            self.launch_text,
        )
        self.assertIn(
            'DeclareLaunchArgument("bbox_source", default_value="simulated")',
            self.launch_text,
        )
        self.assertIn('"yolo_bbox_topic"', self.launch_text)
        self.assertIn('"yolo_image_topic"', self.launch_text)
        self.assertIn('"yolo_camera_info_topic"', self.launch_text)
        self.assertIn('"yolo_camera_frame"', self.launch_text)
        self.assertIn('"yolo_projection_model"', self.launch_text)
        self.assertIn('"yolo_sync_tolerance_sec"', self.launch_text)
        self.assertIn('"marker_scale"', self.launch_text)
        self.assertIn('"publish_fusion_debug"', self.launch_text)
        self.assertIn('"fusion_debug_prefix"', self.launch_text)
        self.assertIn('"self_mask_enabled"', self.launch_text)
        self.assertIn('"sparse_association_enabled"', self.launch_text)
        self.assertIn('"python_executable"', self.launch_text)
        self.assertIn("CONDA_PREFIX", self.launch_text)
        self.assertIn('prefix=LaunchConfiguration("python_executable")', self.launch_text)
        self.assertIn('executable="yolov8_bbox_node"', self.launch_text)
        self.assertIn(
            'executable="perception_baseline_node"',
            self.launch_text,
        )
        self.assertIn('bbox_source == "yolov8"', self.launch_text)

    def test_launch_uses_fsoco_finetuned_yolov8_default(self):
        self.assertIn(
            '"yolo_model_path"',
            self.launch_text,
        )
        self.assertIn(
            '"model_path": LaunchConfiguration("yolo_model_path")',
            self.launch_text,
        )
        self.assertIn(
            (
                "model_path: "
                "/home/dohyun/FS/artifacts/yolov8/"
                "fsoco_yolov8n/weights/best.pt"
            ),
            self.config_text,
        )
        self.assertIn(
            (
                "class_map: "
                '"blue_cone:blue,yellow_cone:yellow,'
                "orange_cone:orange,large_orange_cone:big_orange,"
                'unknown_cone:unknown"'
            ),
            self.config_text,
        )

    def test_yolov8_source_uses_zed_left_camera_contract(self):
        self.assertIn(
            '"image_topic": yolo_image_topic',
            self.launch_text,
        )
        self.assertIn("default_value=\"/zed/left/camera_info\"", self.launch_text)
        self.assertIn(
            "default_value=\"zed_left_camera_optical_frame\"",
            self.launch_text,
        )
        self.assertIn("default_value=\"pinhole\"", self.launch_text)
        self.assertIn("fusion_image_topic", self.launch_text)
        self.assertIn("fusion_camera_info_topic", self.launch_text)
        self.assertIn("fusion_camera_frame", self.launch_text)
        self.assertIn("fusion_projection_model", self.launch_text)
        self.assertIn("fusion_sync_tolerance_sec", self.launch_text)
        self.assertIn(
            'default_value="2.0"',
            self.launch_text,
        )

    def test_fusion_debug_and_sparse_defaults_are_configured(self):
        self.assertIn("publish_fusion_debug: true", self.config_text)
        self.assertIn("fusion_debug_prefix: /fusion/debug", self.config_text)
        self.assertIn("self_mask_enabled: true", self.config_text)
        self.assertIn("sparse_association_enabled: true", self.config_text)
        self.assertIn("sparse_near_min_points: 4", self.config_text)
        self.assertIn("sparse_mid_min_points: 3", self.config_text)
        self.assertIn("sparse_far_min_points: 2", self.config_text)
