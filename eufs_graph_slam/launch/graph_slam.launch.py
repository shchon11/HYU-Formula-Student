# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_frame = LaunchConfiguration("map_frame")
    slam_base_frame = LaunchConfiguration("slam_base_frame")
    publish_tf = LaunchConfiguration("publish_tf")

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
                "map_frame",
                default_value="map",
                description="Frame used for the optimized SLAM map.",
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
                        "map_frame": map_frame,
                        "slam_base_frame": slam_base_frame,
                        "publish_tf": publish_tf,
                    },
                ],
            ),
        ]
    )
