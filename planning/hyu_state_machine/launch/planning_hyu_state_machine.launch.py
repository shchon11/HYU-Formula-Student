from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("hyu_state_machine"),
        "config",
        "planning_hyu_state_machine.yaml",
    ])

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to the planning state machine parameter YAML file.",
    )

    return LaunchDescription([
        params_arg,
        Node(
            package="hyu_state_machine",
            executable="planning_hyu_state_machine_node",
            name="planning_hyu_state_machine_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
