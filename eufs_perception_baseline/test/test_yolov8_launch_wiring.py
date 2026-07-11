import ast
from pathlib import Path
import unittest

import yaml


def _call_name(call):
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return None


def _declared_launch_arguments(tree):
    names = set()
    for node in ast.walk(tree):
        if (
            not isinstance(node, ast.Call)
            or _call_name(node) != "DeclareLaunchArgument"
        ):
            continue
        name_node = node.args[0] if node.args else None
        for keyword in node.keywords:
            if keyword.arg == "name":
                name_node = keyword.value
                break
        if (
            isinstance(name_node, ast.Constant)
            and isinstance(name_node.value, str)
        ):
            names.add(name_node.value)
    return names


def _node_parameter_contracts(tree):
    contracts = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or _call_name(node) != "Node":
            continue
        keywords = {
            keyword.arg: keyword.value
            for keyword in node.keywords
            if keyword.arg
        }
        executable = keywords.get("executable")
        parameters = keywords.get("parameters")
        if not (
            isinstance(executable, ast.Constant)
            and isinstance(executable.value, str)
            and isinstance(parameters, ast.List)
        ):
            continue
        override_keys = set()
        for item in parameters.elts:
            if not isinstance(item, ast.Dict):
                continue
            for key in item.keys:
                if (
                    isinstance(key, ast.Constant)
                    and isinstance(key.value, str)
                ):
                    override_keys.add(key.value)
        contracts[executable.value] = (parameters, override_keys, node)
    return contracts


def _explicit_python_default(tree):
    for node in tree.body:
        if (
            isinstance(node, ast.FunctionDef)
            and node.name == "_default_python_executable"
        ):
            returns = [
                item for item in ast.walk(node) if isinstance(item, ast.Return)
            ]
            if (
                len(returns) == 1
                and isinstance(returns[0].value, ast.Constant)
            ):
                return returns[0].value.value
    raise AssertionError(
        "_default_python_executable must have one constant return"
    )


def _included_launch_argument_names(tree):
    names = set()
    for node in ast.walk(tree):
        if (
            not isinstance(node, ast.Call)
            or _call_name(node) != "IncludeLaunchDescription"
        ):
            continue
        for keyword in node.keywords:
            if (
                keyword.arg != "launch_arguments"
                or not isinstance(keyword.value, ast.List)
            ):
                continue
            for item in keyword.value.elts:
                if not isinstance(item, ast.Tuple) or not item.elts:
                    continue
                key = item.elts[0]
                if (
                    isinstance(key, ast.Constant)
                    and isinstance(key.value, str)
                ):
                    names.add(key.value)
    return names


class YoloV8LaunchWiringTest(unittest.TestCase):
    def setUp(self):
        package_root = Path(__file__).resolve().parents[1]
        repository_root = package_root.parent
        self.launch_path = (
            package_root / "launch" / "perception_baseline.launch.py"
        )
        self.config_path = package_root / "config" / "perception_baseline.yaml"
        self.simulation_launch_path = (
            repository_root
            / "eufs_sim"
            / "eufs_launcher"
            / "launch"
            / "simulation.launch.py"
        )
        self.launch_tree = ast.parse(self.launch_path.read_text())
        self.simulation_tree = ast.parse(
            self.simulation_launch_path.read_text()
        )
        self.config = yaml.safe_load(self.config_path.read_text())
        self.perception_params = self.config["perception_baseline_node"][
            "ros__parameters"
        ]
        self.yolo_params = self.config["yolov8_bbox_node"]["ros__parameters"]

    def test_launch_exposes_three_tier_input_contract(self):
        arguments = _declared_launch_arguments(self.launch_tree)
        self.assertTrue(
            {
                "bbox_source",
                "simulated_bbox_topic",
                "yolo_bbox_topic",
                "yolo_image_topic",
                "yolo_camera_info_topic",
                "yolo_camera_frame",
                "yolo_projection_model",
                "right_image_topic",
                "right_camera_info_topic",
                "right_camera_frame",
                "sync_tolerance_sec",
                "yolo_sync_tolerance_sec",
                "image_sync_tolerance_sec",
                "sync_queue_size",
                "image_sync_queue_size",
                "timestamp_reset_threshold_sec",
                "motion_compensation_frame",
                "stereo_fallback_enabled",
                "python_executable",
            }.issubset(arguments)
        )

    def test_yaml_is_passed_to_both_nodes_before_runtime_overrides(self):
        contracts = _node_parameter_contracts(self.launch_tree)
        self.assertEqual(
            {"perception_baseline_node", "yolov8_bbox_node"},
            set(contracts),
        )
        for parameters, _, _ in contracts.values():
            self.assertGreaterEqual(len(parameters.elts), 2)
            config_source = parameters.elts[0]
            self.assertIsInstance(config_source, ast.Call)
            self.assertEqual("_default_config_file", _call_name(config_source))

        _, fusion_overrides, _ = contracts["perception_baseline_node"]
        self.assertTrue(
            {
                "use_sim_time",
                "image_topic",
                "right_image_topic",
                "camera_info_topic",
                "right_camera_info_topic",
                "camera_frame",
                "right_camera_frame",
                "sync_tolerance_sec",
                "image_sync_tolerance_sec",
                "sync_queue_size",
                "image_sync_queue_size",
                "timestamp_reset_threshold_sec",
                "motion_compensation_frame",
                "stereo_fallback_enabled",
            }.issubset(fusion_overrides)
        )

    def test_sync_and_stereo_defaults_match_runtime_contract(self):
        self.assertEqual(0.15, self.perception_params["sync_tolerance_sec"])
        self.assertEqual(
            0.05,
            self.perception_params["image_sync_tolerance_sec"],
        )
        self.assertEqual(12, self.perception_params["sync_queue_size"])
        self.assertEqual(64, self.perception_params["image_sync_queue_size"])
        self.assertEqual(
            0.1,
            self.perception_params["timestamp_reset_threshold_sec"],
        )
        self.assertEqual(
            "map",
            self.perception_params["motion_compensation_frame"],
        )
        self.assertEqual(
            "/zed/right/image_rect_color",
            self.perception_params["image_topic"],
        )
        self.assertEqual(
            "/zed/right/image_rect_color",
            self.perception_params["right_image_topic"],
        )
        self.assertEqual(
            "/zed/right/camera_info",
            self.perception_params["right_camera_info_topic"],
        )
        self.assertEqual(
            "zed_right_camera_optical_frame",
            self.perception_params["right_camera_frame"],
        )
        self.assertTrue(self.perception_params["ground_ransac_enabled"])
        self.assertTrue(self.perception_params["monocular_fallback_enabled"])
        self.assertTrue(self.perception_params["stereo_fallback_enabled"])
        self.assertEqual(2, self.perception_params["sparse_mid_min_points"])

    def test_launch_does_not_select_an_ambient_conda_interpreter(self):
        self.assertEqual("", _explicit_python_default(self.launch_tree))
        self.assertEqual("", _explicit_python_default(self.simulation_tree))
        contracts = _node_parameter_contracts(self.launch_tree)
        for _, _, node in contracts.values():
            self.assertTrue(
                any(keyword.arg is None for keyword in node.keywords)
            )

    def test_simulation_forwards_clock_stereo_and_explicit_python_inputs(self):
        declared = _declared_launch_arguments(self.simulation_tree)
        self.assertTrue(
            {
                "use_sim_time",
                "perception_right_image_topic",
                "perception_right_camera_info_topic",
                "perception_right_camera_frame",
                "perception_motion_compensation_frame",
                "perception_python_executable",
            }.issubset(declared)
        )
        forwarded = _included_launch_argument_names(self.simulation_tree)
        self.assertTrue(
            {
                "use_sim_time",
                "right_image_topic",
                "right_camera_info_topic",
                "right_camera_frame",
                "motion_compensation_frame",
                "python_executable",
            }.issubset(forwarded)
        )

    def test_fsoco_finetuned_yolov8_defaults_are_preserved(self):
        self.assertEqual(
            "/home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt",
            self.yolo_params["model_path"],
        )
        self.assertEqual(
            "blue_cone:blue,yellow_cone:yellow,orange_cone:orange,"
            "large_orange_cone:big_orange,unknown_cone:unknown",
            self.yolo_params["class_map"],
        )


if __name__ == "__main__":
    unittest.main()
