import os
import sys
from functools import lru_cache
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "eufs_perception_baseline"
CONFIG_FILE = "perception_baseline.yaml"


def _default_config_file() -> str:
    return str(
        Path(get_package_share_directory(PACKAGE_NAME))
        / "config"
        / CONFIG_FILE
    )


@lru_cache(maxsize=1)
def _load_defaults() -> dict:
    with open(_default_config_file(), "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    return {
        node_name: config.get(node_name, {}).get("ros__parameters", {})
        for node_name in ("perception_baseline_node", "yolov8_bbox_node")
    }


def _default(node_name: str, parameter_name: str) -> str:
    value = _load_defaults()[node_name][parameter_name]
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def _default_python_executable() -> str:
    conda_prefix = os.environ.get("CONDA_PREFIX", "").strip()
    if conda_prefix:
        conda_python = Path(conda_prefix) / "bin" / "python3"
        if conda_python.exists():
            return str(conda_python)
    return sys.executable


def _launch_nodes(context):
    bbox_source = (
        LaunchConfiguration("bbox_source")
        .perform(context)
        .strip()
        .lower()
    )
    legacy_bbox_topic = (
        LaunchConfiguration("bbox_topic")
        .perform(context)
        .strip()
    )
    simulated_bbox_topic = LaunchConfiguration(
        "simulated_bbox_topic"
    ).perform(context)
    yolo_bbox_topic = LaunchConfiguration("yolo_bbox_topic").perform(context)
    yolo_image_topic = LaunchConfiguration("yolo_image_topic").perform(context)
    yolo_camera_info_topic = LaunchConfiguration(
        "yolo_camera_info_topic"
    ).perform(context)
    yolo_camera_frame = LaunchConfiguration("yolo_camera_frame").perform(
        context
    )
    yolo_projection_model = LaunchConfiguration(
        "yolo_projection_model"
    ).perform(context)
    if legacy_bbox_topic:
        simulated_bbox_topic = legacy_bbox_topic
    fusion_bbox_topic = (
        yolo_bbox_topic
        if bbox_source == "yolov8"
        else simulated_bbox_topic
    )
    fusion_image_topic = (
        yolo_image_topic
        if bbox_source == "yolov8"
        else LaunchConfiguration("image_topic").perform(context)
    )
    fusion_camera_info_topic = (
        yolo_camera_info_topic
        if bbox_source == "yolov8"
        else LaunchConfiguration("camera_info_topic").perform(context)
    )
    fusion_camera_frame = (
        yolo_camera_frame
        if bbox_source == "yolov8"
        else LaunchConfiguration("camera_frame").perform(context)
    )
    fusion_projection_model = (
        yolo_projection_model
        if bbox_source == "yolov8"
        else LaunchConfiguration("projection_model").perform(context)
    )
    fusion_sync_tolerance_sec = float(
        LaunchConfiguration("yolo_sync_tolerance_sec").perform(context)
        if bbox_source == "yolov8"
        else LaunchConfiguration("sync_tolerance_sec").perform(context)
    )

    if bbox_source not in ("simulated", "yolov8"):
        raise RuntimeError(
            "bbox_source must be 'simulated' or 'yolov8', "
            f"got '{bbox_source}'"
        )

    nodes = []
    if bbox_source == "yolov8":
        nodes.append(
            Node(
                package="eufs_perception_baseline",
                executable="yolov8_bbox_node",
                name="yolov8_bbox_node",
                output="screen",
                prefix=LaunchConfiguration("python_executable"),
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "image_topic": yolo_image_topic,
                        "bbox_topic": yolo_bbox_topic,
                        "model_path": LaunchConfiguration("yolo_model_path"),
                        "confidence_threshold": LaunchConfiguration(
                            "yolo_confidence_threshold"
                        ),
                        "iou_threshold": LaunchConfiguration(
                            "yolo_iou_threshold"
                        ),
                        "imgsz": LaunchConfiguration("yolo_imgsz"),
                        "device": LaunchConfiguration("yolo_device"),
                        "max_det": LaunchConfiguration("yolo_max_det"),
                        "class_map": LaunchConfiguration("yolo_class_map"),
                        "unknown_color_policy": LaunchConfiguration(
                            "yolo_unknown_color_policy"
                        ),
                        "publish_debug_image": LaunchConfiguration(
                            "publish_yolo_debug_image"
                        ),
                        "debug_image_topic": LaunchConfiguration(
                            "yolo_debug_image_topic"
                        ),
                    }
                ],
            )
        )

    nodes.append(
        Node(
            package="eufs_perception_baseline",
            executable="perception_baseline_node",
            name="perception_baseline_node",
            output="screen",
            prefix=LaunchConfiguration("python_executable"),
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "image_topic": fusion_image_topic,
                    "pointcloud_topic": LaunchConfiguration(
                        "pointcloud_topic"
                    ),
                    "bbox_topic": fusion_bbox_topic,
                    "camera_info_topic": fusion_camera_info_topic,
                    "camera_frame": fusion_camera_frame,
                    "projection_model": fusion_projection_model,
                    "output_cones_topic": LaunchConfiguration(
                        "output_cones_topic"
                    ),
                    "output_frame": LaunchConfiguration("output_frame"),
                    "marker_scale": LaunchConfiguration("marker_scale"),
                    "oracle_cones_topic": LaunchConfiguration(
                        "oracle_cones_topic"
                    ),
                    "publish_empty_on_sync": LaunchConfiguration(
                        "publish_empty_on_sync"
                    ),
                    "sync_tolerance_sec": fusion_sync_tolerance_sec,
                    "fusion_enabled": LaunchConfiguration("fusion_enabled"),
                    "publish_fusion_debug": LaunchConfiguration(
                        "publish_fusion_debug"
                    ),
                    "fusion_debug_prefix": LaunchConfiguration(
                        "fusion_debug_prefix"
                    ),
                    "self_mask_enabled": LaunchConfiguration("self_mask_enabled"),
                    "self_mask_min_x": LaunchConfiguration("self_mask_min_x"),
                    "self_mask_max_x": LaunchConfiguration("self_mask_max_x"),
                    "self_mask_abs_y": LaunchConfiguration("self_mask_abs_y"),
                    "self_mask_min_z": LaunchConfiguration("self_mask_min_z"),
                    "self_mask_max_z": LaunchConfiguration("self_mask_max_z"),
                    "sparse_association_enabled": LaunchConfiguration(
                        "sparse_association_enabled"
                    ),
                    "sparse_bbox_margin_px": LaunchConfiguration(
                        "sparse_bbox_margin_px"
                    ),
                    "sparse_bbox_margin_ratio": LaunchConfiguration(
                        "sparse_bbox_margin_ratio"
                    ),
                    "sparse_near_range_m": LaunchConfiguration(
                        "sparse_near_range_m"
                    ),
                    "sparse_far_range_m": LaunchConfiguration(
                        "sparse_far_range_m"
                    ),
                    "sparse_near_min_points": LaunchConfiguration(
                        "sparse_near_min_points"
                    ),
                    "sparse_mid_min_points": LaunchConfiguration(
                        "sparse_mid_min_points"
                    ),
                    "sparse_far_min_points": LaunchConfiguration(
                        "sparse_far_min_points"
                    ),
                    "sparse_far_min_probability": LaunchConfiguration(
                        "sparse_far_min_probability"
                    ),
                    "sparse_far_min_bbox_width_px": LaunchConfiguration(
                        "sparse_far_min_bbox_width_px"
                    ),
                    "sparse_far_min_bbox_height_px": LaunchConfiguration(
                        "sparse_far_min_bbox_height_px"
                    ),
                    "sparse_max_depth_span_m": LaunchConfiguration(
                        "sparse_max_depth_span_m"
                    ),
                    "sparse_max_depth_span_ratio": LaunchConfiguration(
                        "sparse_max_depth_span_ratio"
                    ),
                    "sparse_max_width": LaunchConfiguration("sparse_max_width"),
                    "sparse_variance_x": LaunchConfiguration("sparse_variance_x"),
                    "sparse_variance_y": LaunchConfiguration("sparse_variance_y"),
                }
            ],
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulator clock for perception nodes.",
            ),
            DeclareLaunchArgument(
                "python_executable",
                default_value=_default_python_executable(),
                description=(
                    "Python interpreter used to run perception nodes. Defaults "
                    "to $CONDA_PREFIX/bin/python3 when a conda env is active."
                ),
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value=_default(
                    "perception_baseline_node",
                    "image_topic",
                ),
            ),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value=_default(
                    "perception_baseline_node",
                    "pointcloud_topic",
                ),
            ),
            DeclareLaunchArgument(
                "simulated_bbox_topic",
                default_value=_default(
                    "perception_baseline_node",
                    "bbox_topic",
                ),
                description="BBox topic used when bbox_source:=simulated.",
            ),
            DeclareLaunchArgument(
                "bbox_topic",
                default_value="",
                description="Deprecated alias for simulated_bbox_topic.",
            ),
            DeclareLaunchArgument("bbox_source", default_value="simulated"),
            DeclareLaunchArgument(
                "yolo_bbox_topic",
                default_value=_default("yolov8_bbox_node", "bbox_topic"),
                description="BBox topic used when bbox_source:=yolov8.",
            ),
            DeclareLaunchArgument(
                "yolo_image_topic",
                default_value=_default("yolov8_bbox_node", "image_topic"),
                description="ZED image topic consumed by yolov8_bbox_node.",
            ),
            DeclareLaunchArgument(
                "yolo_camera_info_topic",
                default_value="/zed/left/camera_info",
                description=(
                    "CameraInfo topic paired with the YOLO image topic."
                ),
            ),
            DeclareLaunchArgument(
                "yolo_camera_frame",
                default_value="zed_left_camera_optical_frame",
                description=(
                    "Camera optical frame paired with the YOLO image topic."
                ),
            ),
            DeclareLaunchArgument(
                "yolo_projection_model",
                default_value="pinhole",
                description=(
                    "Projection model used by fusion for real ZED images."
                ),
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value=_default(
                    "perception_baseline_node",
                    "camera_info_topic",
                ),
            ),
            DeclareLaunchArgument(
                "camera_frame",
                default_value=_default(
                    "perception_baseline_node",
                    "camera_frame",
                ),
            ),
            DeclareLaunchArgument(
                "projection_model",
                default_value=_default(
                    "perception_baseline_node",
                    "projection_model",
                ),
            ),
            DeclareLaunchArgument(
                "output_cones_topic",
                default_value=_default(
                    "perception_baseline_node",
                    "output_cones_topic",
                ),
            ),
            DeclareLaunchArgument(
                "output_frame",
                default_value=_default(
                    "perception_baseline_node",
                    "output_frame",
                ),
            ),
            DeclareLaunchArgument(
                "oracle_cones_topic",
                default_value=_default(
                    "perception_baseline_node",
                    "oracle_cones_topic",
                ),
            ),
            DeclareLaunchArgument(
                "publish_empty_on_sync",
                default_value=_default(
                    "perception_baseline_node",
                    "publish_empty_on_sync",
                ),
            ),
            DeclareLaunchArgument(
                "sync_tolerance_sec",
                default_value=_default(
                    "perception_baseline_node",
                    "sync_tolerance_sec",
                ),
                description=(
                    "Maximum timestamp difference between bbox and LiDAR "
                    "messages accepted by fusion."
                ),
            ),
            DeclareLaunchArgument(
                "yolo_sync_tolerance_sec",
                default_value="2.0",
                description=(
                    "YOLO mode timestamp tolerance between detector image "
                    "boxes and LiDAR points. Used only when bbox_source:=yolov8."
                ),
            ),
            DeclareLaunchArgument(
                "marker_scale",
                default_value=_default(
                    "perception_baseline_node",
                    "marker_scale",
                ),
                description="RViz sphere-list marker diameter for fused cones.",
            ),
            DeclareLaunchArgument(
                "fusion_enabled",
                default_value=_default(
                    "perception_baseline_node",
                    "fusion_enabled",
                ),
            ),
            DeclareLaunchArgument(
                "publish_fusion_debug",
                default_value=_default(
                    "perception_baseline_node",
                    "publish_fusion_debug",
                ),
                description="Publish /fusion/debug pointcloud and marker topics.",
            ),
            DeclareLaunchArgument(
                "fusion_debug_prefix",
                default_value=_default(
                    "perception_baseline_node",
                    "fusion_debug_prefix",
                ),
            ),
            DeclareLaunchArgument(
                "self_mask_enabled",
                default_value=_default(
                    "perception_baseline_node",
                    "self_mask_enabled",
                ),
            ),
            DeclareLaunchArgument(
                "self_mask_min_x",
                default_value=_default("perception_baseline_node", "self_mask_min_x"),
            ),
            DeclareLaunchArgument(
                "self_mask_max_x",
                default_value=_default("perception_baseline_node", "self_mask_max_x"),
            ),
            DeclareLaunchArgument(
                "self_mask_abs_y",
                default_value=_default("perception_baseline_node", "self_mask_abs_y"),
            ),
            DeclareLaunchArgument(
                "self_mask_min_z",
                default_value=_default("perception_baseline_node", "self_mask_min_z"),
            ),
            DeclareLaunchArgument(
                "self_mask_max_z",
                default_value=_default("perception_baseline_node", "self_mask_max_z"),
            ),
            DeclareLaunchArgument(
                "sparse_association_enabled",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_association_enabled",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_bbox_margin_px",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_bbox_margin_px",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_bbox_margin_ratio",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_bbox_margin_ratio",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_near_range_m",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_near_range_m",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_far_range_m",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_far_range_m",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_near_min_points",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_near_min_points",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_mid_min_points",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_mid_min_points",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_far_min_points",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_far_min_points",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_far_min_probability",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_far_min_probability",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_far_min_bbox_width_px",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_far_min_bbox_width_px",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_far_min_bbox_height_px",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_far_min_bbox_height_px",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_max_depth_span_m",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_max_depth_span_m",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_max_depth_span_ratio",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_max_depth_span_ratio",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_max_width",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_max_width",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_variance_x",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_variance_x",
                ),
            ),
            DeclareLaunchArgument(
                "sparse_variance_y",
                default_value=_default(
                    "perception_baseline_node",
                    "sparse_variance_y",
                ),
            ),
            DeclareLaunchArgument(
                "yolo_model_path",
                default_value=_default("yolov8_bbox_node", "model_path"),
            ),
            DeclareLaunchArgument(
                "yolo_confidence_threshold",
                default_value=_default(
                    "yolov8_bbox_node",
                    "confidence_threshold",
                ),
            ),
            DeclareLaunchArgument(
                "yolo_iou_threshold",
                default_value=_default("yolov8_bbox_node", "iou_threshold"),
            ),
            DeclareLaunchArgument(
                "yolo_imgsz",
                default_value=_default("yolov8_bbox_node", "imgsz"),
            ),
            DeclareLaunchArgument(
                "yolo_device",
                default_value=_default("yolov8_bbox_node", "device"),
            ),
            DeclareLaunchArgument(
                "yolo_max_det",
                default_value=_default("yolov8_bbox_node", "max_det"),
            ),
            DeclareLaunchArgument(
                "yolo_class_map",
                default_value=_default("yolov8_bbox_node", "class_map"),
            ),
            DeclareLaunchArgument(
                "yolo_unknown_color_policy",
                default_value=_default(
                    "yolov8_bbox_node",
                    "unknown_color_policy",
                ),
            ),
            DeclareLaunchArgument(
                "publish_yolo_debug_image",
                default_value=_default(
                    "yolov8_bbox_node",
                    "publish_debug_image",
                ),
            ),
            DeclareLaunchArgument(
                "yolo_debug_image_topic",
                default_value=_default(
                    "yolov8_bbox_node",
                    "debug_image_topic",
                ),
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
