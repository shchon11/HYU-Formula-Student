# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Saved maps live in the package's map/ directory (this launch file is at
# localization/launch/, so ../map resolves to localization/map even with
# a symlink install).
DEFAULT_MAP_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "map")
)


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    car_state_topic = LaunchConfiguration("car_state_topic")
    map_topic = LaunchConfiguration("map_topic")
    slam_odom_topic = LaunchConfiguration("slam_odom_topic")
    status_topic = LaunchConfiguration("status_topic")
    map_converged_topic = LaunchConfiguration("map_converged_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    slam_base_frame = LaunchConfiguration("slam_base_frame")
    publish_tf = LaunchConfiguration("publish_tf")
    localization_mode = LaunchConfiguration("localization_mode")
    auto_localization_after_lap = LaunchConfiguration("auto_localization_after_lap")
    load_map_path = LaunchConfiguration("load_map_path")
    gui = LaunchConfiguration("gui")
    ate_monitor = LaunchConfiguration("ate_monitor")
    ate_status_topic = LaunchConfiguration("ate_status_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("hyu_localization"),
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
                default_value="/localization/wheel_odom",
                description="CarState topic used as the graph SLAM motion input.",
            ),
            DeclareLaunchArgument(
                "map_topic",
                default_value="/localization/cone_map",
                description="Planner-facing cone map topic published by graph SLAM.",
            ),
            DeclareLaunchArgument(
                "slam_odom_topic",
                default_value="/localization/ego_odom",
                description="Planner-facing ego odometry topic published by graph SLAM.",
            ),
            DeclareLaunchArgument(
                "status_topic",
                default_value="~/status",
                description="Latched graph SLAM lifecycle String topic.",
            ),
            DeclareLaunchArgument(
                "map_converged_topic",
                default_value="~/map_converged",
                description="Latched graph SLAM map-ready/converged Bool topic.",
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
                "auto_localization_after_lap",
                default_value="true",
                description=(
                    "Freeze the map and switch to localization after the mapping "
                    "lap closes. Disable for missions that keep discovering new "
                    "track after re-passing the start (e.g. skidpad)."
                ),
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
                "ate_monitor",
                default_value="true",
                description=(
                    "Publish the path-tracking CTE HUD (Frenet d vs the "
                    "d=0 raceline) and the SLAM status overlay."
                ),
            ),
            DeclareLaunchArgument(
                "ate_status_topic",
                default_value="/localization/status",
                description="Graph SLAM lifecycle status topic consumed by ate_monitor.",
            ),
            DeclareLaunchArgument(
                "frenet_odom_topic",
                default_value="/planning/frenet_odom",
                description="Frenet ego odometry (s,d) consumed by the CTE monitor.",
            ),
            DeclareLaunchArgument(
                "global_path_valid_topic",
                default_value="/planning/global_path_valid",
                description="Planner validity heartbeat consumed by the CTE monitor.",
            ),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value=EnvironmentVariable(
                    'ROS_LOCALHOST_ONLY', default_value='1'),
                description="Limit ROS discovery to localhost.",
            ),
            DeclareLaunchArgument(
                "wheel_odometry",
                default_value="true",
                description=(
                    "Start the wheel+INS odometry node that publishes the "
                    "default SLAM motion input (/localization/wheel_odom)."
                ),
            ),
            SetEnvironmentVariable(
                name="ROS_LOCALHOST_ONLY",
                value=LaunchConfiguration("ros_localhost_only"),
            ),
            # GNSS-independent motion source: rear wheel speeds + the INS
            # body-rate/acceleration log, identical wiring in sim and on the
            # car. Publishes the graph_slam default car_state_topic.
            Node(
                package="hyu_localization",
                executable="wheel_odometry",
                name="wheel_odometry",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("wheel_odometry")),
            ),
            Node(
                package="hyu_localization",
                executable="graph_slam_node",
                name="graph_slam",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "car_state_topic": car_state_topic,
                        "map_topic": map_topic,
                        "slam_odom_topic": slam_odom_topic,
                        "status_topic": status_topic,
                        "map_converged_topic": map_converged_topic,
                        "map_frame": map_frame,
                        "odom_frame": odom_frame,
                        "slam_base_frame": slam_base_frame,
                        "publish_tf": publish_tf,
                        "map_save_dir": DEFAULT_MAP_DIR,
                        "localization_mode": localization_mode,
                        "auto_localization_after_lap": auto_localization_after_lap,
                        "load_map_path": load_map_path,
                    },
                ],
            ),
            Node(
                package="hyu_localization",
                executable="slam_gui",
                name="slam_gui",
                output="screen",
                arguments=["--map-dir", DEFAULT_MAP_DIR, "--map-topic", map_topic],
                condition=IfCondition(gui),
            ),
            Node(
                package="hyu_localization",
                executable="ate_monitor",
                name="ate_monitor",
                output="screen",
                arguments=[
                    "--frenet-odom",
                    LaunchConfiguration("frenet_odom_topic"),
                    "--path-valid-topic",
                    LaunchConfiguration("global_path_valid_topic"),
                    "--status-topic",
                    ate_status_topic,
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(ate_monitor),
            ),
        ]
    )
