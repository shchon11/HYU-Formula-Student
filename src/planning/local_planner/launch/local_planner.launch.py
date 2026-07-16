from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    source_mode = LaunchConfiguration("source_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_status_topic = LaunchConfiguration("slam_status_topic")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("local_planner"),
                "config",
                "local_planner.yaml",
            ]),
        ),
        DeclareLaunchArgument("source_mode", default_value="slam_map"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam_status_topic", default_value="/graph_slam/status"),
        Node(
            package="local_planner",
            executable="local_planner_node",
            name="local_planner_node",
            output="screen",
            parameters=[
                params_file,
                {
                    "source_mode": source_mode,
                    "slam_status_topic": slam_status_topic,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
    ])
