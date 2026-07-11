# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    car_state_topic = LaunchConfiguration("car_state_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    slam_base_frame = LaunchConfiguration("slam_base_frame")
    publish_tf = LaunchConfiguration("publish_tf")
    pose_history_duration = LaunchConfiguration("pose_history_duration")
    pose_history_max_samples = LaunchConfiguration("pose_history_max_samples")
    clock_rollback_threshold = LaunchConfiguration("clock_rollback_threshold")
    max_pending_cone_messages = LaunchConfiguration("max_pending_cone_messages")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

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
                "pose_history_duration",
                default_value="3.0",
                description="Seconds of CarState history retained for cone-time interpolation.",
            ),
            DeclareLaunchArgument(
                "pose_history_max_samples",
                default_value="1024",
                description="Hard sample bound for the CarState interpolation history.",
            ),
            DeclareLaunchArgument(
                "clock_rollback_threshold",
                default_value="0.1",
                description="Backward CarState jump in seconds that starts a new graph epoch.",
            ),
            DeclareLaunchArgument(
                "max_pending_cone_messages",
                default_value="32",
                description="Bound for cone frames waiting on a future CarState bracket.",
            ),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Limit ROS discovery to localhost.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz with the graph SLAM map frame config.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("eufs_launcher"),
                        "config",
                        "graph_slam.rviz",
                    ]
                ),
                description="RViz config used when graph SLAM owns the map frame.",
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
                        "pose_history_duration": pose_history_duration,
                        "pose_history_max_samples": pose_history_max_samples,
                        "clock_rollback_threshold": clock_rollback_threshold,
                        "max_pending_cone_messages": max_pending_cone_messages,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="graph_slam_rviz",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz),
            ),
        ]
    )
