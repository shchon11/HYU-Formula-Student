from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # This launch path owns the CSV global waypoint writer. Do not start
    # planner_node here unless /global_waypoints and /planning/global_path_valid
    # are remapped so the two producers are not competing writers.
    default_params = PathJoinSubstitution([
        FindPackageShare("global_planner"),
        "config",
        "global_planner.yaml",
    ])

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Shared parameter YAML for global_planner and frenet nodes.",
    )

    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/ground_truth/odom",
        description="Vehicle odometry topic consumed by frenet_odom_node.",
    )

    return LaunchDescription([
        params_arg,
        odom_topic_arg,
        Node(
            package="global_planner",
            executable="global_planner_trajectory_publisher_node",
            name="global_planner_trajectory_publisher_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
        Node(
            package="frenet_conversion",
            executable="frenet_odom_node",
            name="frenet_odom_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"odom_topic": LaunchConfiguration("odom_topic")},
            ],
        ),
        Node(
            package="global_planner",
            executable="wpnt_publisher_node",
            name="wpnt_publisher",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
