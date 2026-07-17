"""Trajectory data publisher + frenet odom + wpnt_publisher + debug visualizer.

  ros2 launch hyu_global_planner hyu_global_planner_trajectory_publisher_debug.launch.py

Includes hyu_global_planner_trajectory_publisher.launch.py (which starts the data
publisher, frenet_odom_node, and wpnt_publisher) and adds the debug visualizer node. All nodes
share a single params file (config/hyu_global_planner.yaml); each reads only its own
node-name section, so one params_file:= override drives them all. For the data +
frenet + wpnt_publisher nodes without the visualizer use
hyu_global_planner_trajectory_publisher.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("hyu_global_planner")

    default_params = PathJoinSubstitution([pkg, "config", "hyu_global_planner.yaml"])
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Shared parameter YAML for hyu_global_planner and frenet nodes.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/ground_truth/odom",
        description="Vehicle odometry topic consumed by frenet_odom_node.",
    )

    trajectory_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg, "launch", "hyu_global_planner_trajectory_publisher.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "odom_topic": LaunchConfiguration("odom_topic"),
        }.items(),
    )

    debug_node = Node(
        package="hyu_global_planner",
        executable="hyu_global_planner_debug_visualizer_node",
        name="hyu_global_planner_debug_visualizer_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_arg,
        odom_topic_arg,
        trajectory_launch,
        debug_node,
    ])
