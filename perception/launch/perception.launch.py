"""Launch cone perception: the YOLO detector and the fusion node.

Small on purpose. The old launch file carried ~890 lines of arguments because
the node behind it had a stereo tier, a keypoint stream, a right camera, a
RektNet model and a pairing gate to configure. None of that exists any more, so
neither do its arguments. What is left is the handful of things that genuinely
differ between the simulator and the car.

    ros2 launch hyu_perception perception.launch.py

Everything else lives in config/perception.yaml, next to the comment explaining
why it has the value it has.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE = "hyu_perception"


def _config() -> str:
    return os.path.join(get_package_share_directory(PACKAGE), "config",
                        "perception.yaml")


def _resolve_weights(context, *_args, **_kwargs):
    """Anchor a package-relative weight path to the installed share tree.

    The weights ship inside the package, so the config carries a relative path
    and there is no dependency on anything outside the workspace. That path only
    becomes loadable once anchored: launch does not run from the package root,
    so 'models/...' resolves against whatever directory the user happened to be
    in. An absolute override passes through untouched.
    """
    value = LaunchConfiguration("yolo_model_path").perform(context).strip()
    if not value or os.path.isabs(value):
        return []
    return [SetLaunchConfiguration(
        "yolo_model_path",
        os.path.join(get_package_share_directory(PACKAGE), value))]


def _config_value(key: str, fallback: str) -> str:
    """Read one perception_node parameter out of the shipped config.

    Used to seed a launch argument's DEFAULT from the config, so exposing a
    knob on the command line does not silently change its value when nobody
    passes it. Falls back if the config is unreadable — a launch file must not
    fail to generate because a comment got mangled.
    """
    try:
        import yaml
        with open(_config()) as handle:
            data = yaml.safe_load(handle) or {}
        value = data["perception_node"]["ros__parameters"][key]
        return str(value)
    except Exception:
        return fallback


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    left_image_topic = LaunchConfiguration("left_image_topic")
    bbox_topic = LaunchConfiguration("bbox_topic")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "left_image_topic", default_value="/sensors/zed/left/color/rect/image",
            description="The image YOLO detects on and perception projects "
                        "from. These must be the same camera."),
        DeclareLaunchArgument(
            "right_image_topic", default_value="/sensors/zed/right/color/rect/image",
            description="Used only by the ZNCC cross-check."),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/sensors/zed/left/color/rect/camera_info",
            description="Intrinsics for left_image_topic. fx and the baseline "
                        "come from here and the TF, never from a constant."),
        DeclareLaunchArgument(
            "camera_frame", default_value="zed_left_camera_optical_frame"),
        DeclareLaunchArgument("bbox_topic", default_value="/perception/bounding_boxes"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/sensors/lidar/points"),
        DeclareLaunchArgument("output_cones_topic", default_value="/perception/cones"),
        DeclareLaunchArgument("output_frame", default_value="base_footprint"),
        DeclareLaunchArgument(
            "motion_compensation_frame", default_value="map",
            description="Fixed frame the cloud-stamp -> bbox-stamp transform "
                        "goes through. Use odom when GraphSLAM owns map->base."),
        DeclareLaunchArgument(
            "projection_model", default_value="pinhole",
            description="pinhole for a real camera (and the ZED in sim). "
                        "eufs_bbox ONLY for the simulator's synthetic "
                        "/noisy_bounding_boxes contract. Getting this wrong "
                        "does not raise: it silently puts every vision cone in "
                        "the wrong place."),
        DeclareLaunchArgument("publish_debug", default_value="true"),
        # How stale a bbox may be relative to the cloud before its cones are
        # published uncoloured. Defaults to the config, so exposing it here
        # changes nothing unless you pass it.
        #
        # Measured end to end on the Jetson (bag replay, RViz up): median
        # 137 ms, p90 138 ms, max 237 ms against the 250 ms gate -- it fits,
        # but only just. Detection is not the cost: the FP16 engine is 3.1 ms
        # of inference (7.0 ms with pre/post). The budget is perception_node's
        # own per-cloud work, dominated by clustering (~43 ms) with roi+ground
        # (~16 ms) and publish (~10 ms) behind it.
        #
        # Raise this only with a reason. A stale bbox is a bbox from where the
        # car WAS, and only motion compensation makes that recoverable; the
        # honest signal when the budget is blown is uncoloured cones plus the
        # warning, not a gate quietly widened until they stop.
        DeclareLaunchArgument(
            "max_cluster_age_sec",
            default_value=_config_value("max_cluster_age_sec", "0.25")),
        # Keep in step with yolov8_bbox_node.model_path in config/perception.yaml:
        # this argument is passed as a parameter override, so it WINS over the
        # config and a stale default here silently loads the wrong weight.
        DeclareLaunchArgument("yolo_model_path",
                              default_value="models/cone_detect_yolo26n/weights/best.pt"),
        DeclareLaunchArgument("yolo_device", default_value=""),
        DeclareLaunchArgument("yolo_imgsz", default_value="640"),
        DeclareLaunchArgument("publish_yolo_debug_image", default_value="false"),
        DeclareLaunchArgument(
            "launch_detector", default_value="true",
            description="Set false to run perception against an existing bbox "
                        "stream (a bag, or a detector started elsewhere)."),

        OpaqueFunction(function=_resolve_weights),

        Node(
            package=PACKAGE,
            executable="yolov8_bbox_node",
            name="yolov8_bbox_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_detector")),
            parameters=[_config(), {
                "use_sim_time": use_sim_time,
                "image_topic": left_image_topic,
                # The detector phase-locks to the LiDAR (inference_mode:
                # lidar_locked), so it must follow pointcloud_topic. Without
                # this it stays pinned to the config value and never ticks
                # wherever the cloud is published under another name.
                "lidar_topic": LaunchConfiguration("pointcloud_topic"),
                "bbox_topic": bbox_topic,
                "model_path": LaunchConfiguration("yolo_model_path"),
                "device": LaunchConfiguration("yolo_device"),
                "imgsz": LaunchConfiguration("yolo_imgsz"),
                "publish_debug_image": LaunchConfiguration(
                    "publish_yolo_debug_image"),
            }],
        ),
        Node(
            package=PACKAGE,
            executable="perception_node",
            name="perception_node",
            output="screen",
            parameters=[_config(), {
                "use_sim_time": use_sim_time,
                "left_image_topic": left_image_topic,
                "right_image_topic": LaunchConfiguration("right_image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "bbox_topic": bbox_topic,
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "output_cones_topic": LaunchConfiguration("output_cones_topic"),
                "output_frame": LaunchConfiguration("output_frame"),
                "motion_compensation_frame": LaunchConfiguration(
                    "motion_compensation_frame"),
                "projection_model": LaunchConfiguration("projection_model"),
                "publish_debug": LaunchConfiguration("publish_debug"),
                "max_cluster_age_sec": ParameterValue(
                    LaunchConfiguration("max_cluster_age_sec"), value_type=float),
            }],
        ),
    ])
