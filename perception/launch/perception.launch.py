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


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    left_image_topic = LaunchConfiguration("left_image_topic")
    bbox_topic = LaunchConfiguration("bbox_topic")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "left_image_topic", default_value="/zed/left/image_rect_color",
            description="The image YOLO detects on and perception projects "
                        "from. These must be the same camera."),
        DeclareLaunchArgument(
            "right_image_topic", default_value="/zed/right/image_rect_color",
            description="Used only by the ZNCC cross-check."),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/zed/left/camera_info",
            description="Intrinsics for left_image_topic. fx and the baseline "
                        "come from here and the TF, never from a constant."),
        DeclareLaunchArgument(
            "camera_frame", default_value="zed_left_camera_optical_frame"),
        DeclareLaunchArgument("bbox_topic", default_value="/perception/bounding_boxes"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/velodyne_points"),
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
        DeclareLaunchArgument("yolo_model_path",
                              default_value="models/cone_pose_8kpt/weights/best.pt"),
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
            }],
        ),
    ])
