from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    bbox_topic = LaunchConfiguration("bbox_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    camera_frame = LaunchConfiguration("camera_frame")
    projection_model = LaunchConfiguration("projection_model")
    output_cones_topic = LaunchConfiguration("output_cones_topic")
    output_frame = LaunchConfiguration("output_frame")
    oracle_cones_topic = LaunchConfiguration("oracle_cones_topic")
    publish_empty_on_sync = LaunchConfiguration("publish_empty_on_sync")
    fusion_enabled = LaunchConfiguration("fusion_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/zed/left/image_rect_color"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/velodyne_points"),
            DeclareLaunchArgument("bbox_topic", default_value="/noisy_bounding_boxes"),
            DeclareLaunchArgument("camera_info_topic", default_value="/custom_camera_info"),
            DeclareLaunchArgument("camera_frame", default_value="zed_right_camera_optical_frame"),
            DeclareLaunchArgument("projection_model", default_value="eufs_bbox"),
            DeclareLaunchArgument("output_cones_topic", default_value="/cones"),
            DeclareLaunchArgument("output_frame", default_value="base_footprint"),
            DeclareLaunchArgument("oracle_cones_topic", default_value=""),
            DeclareLaunchArgument("publish_empty_on_sync", default_value="false"),
            DeclareLaunchArgument("fusion_enabled", default_value="true"),
            Node(
                package="eufs_perception_baseline",
                executable="perception_baseline_node",
                name="perception_baseline_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "pointcloud_topic": pointcloud_topic,
                        "bbox_topic": bbox_topic,
                        "camera_info_topic": camera_info_topic,
                        "camera_frame": camera_frame,
                        "projection_model": projection_model,
                        "output_cones_topic": output_cones_topic,
                        "output_frame": output_frame,
                        "oracle_cones_topic": oracle_cones_topic,
                        "publish_empty_on_sync": publish_empty_on_sync,
                        "fusion_enabled": fusion_enabled,
                    }
                ],
            ),
        ]
    )
