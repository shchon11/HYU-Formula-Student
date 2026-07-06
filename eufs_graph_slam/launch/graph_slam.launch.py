# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Saved maps live in the package's map/ directory (this launch file is at
# eufs_graph_slam/launch/, so ../map resolves to eufs_graph_slam/map even with
# a symlink install).
DEFAULT_MAP_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "map")
)


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    car_state_topic = LaunchConfiguration("car_state_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    slam_base_frame = LaunchConfiguration("slam_base_frame")
    publish_tf = LaunchConfiguration("publish_tf")
    localization_mode = LaunchConfiguration("localization_mode")
    load_map_path = LaunchConfiguration("load_map_path")
    gui = LaunchConfiguration("gui")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("eufs_graph_slam"),
                        "config",
                        "graph_slam.yaml",
                    ]
                ),
                description="Path to the graph SLAM parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the simulator clock.",
            ),
            DeclareLaunchArgument(
                "car_state_topic",
                default_value="/odometry_integration/car_state",
                description="CarState topic used as the graph SLAM motion input.",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Frame used for the optimized SLAM map.",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Continuous odometry frame corrected by graph SLAM.",
            ),
            DeclareLaunchArgument(
                "slam_base_frame",
                default_value="base_footprint",
                description="Child frame used by graph SLAM odometry and TF.",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description=(
                    "Publish map->slam_base_frame TF. Disable simulator publish_gt_tf "
                    "when this is true."
                ),
            ),
            DeclareLaunchArgument(
                "localization_mode",
                default_value="false",
                description="Localize against a saved map instead of mapping.",
            ),
            DeclareLaunchArgument(
                "load_map_path",
                default_value="",
                description="CSV map to load when localization_mode is true.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Launch the graph SLAM control GUI.",
            ),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Limit ROS discovery to localhost.",
            ),
            SetEnvironmentVariable(
                name="ROS_LOCALHOST_ONLY",
                value=LaunchConfiguration("ros_localhost_only"),
            ),
            Node(
                package="eufs_graph_slam",
                executable="graph_slam_node",
                name="graph_slam",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "car_state_topic": car_state_topic,
                        "map_frame": map_frame,
                        "odom_frame": odom_frame,
                        "slam_base_frame": slam_base_frame,
                        "publish_tf": publish_tf,
                        "map_save_dir": DEFAULT_MAP_DIR,
                        "localization_mode": localization_mode,
                        "load_map_path": load_map_path,
                    },
                ],
            ),
            Node(
                package="eufs_graph_slam",
                executable="slam_gui",
                name="slam_gui",
                output="screen",
                arguments=["--map-dir", DEFAULT_MAP_DIR],
                condition=IfCondition(gui),
            ),
        ]
    )
