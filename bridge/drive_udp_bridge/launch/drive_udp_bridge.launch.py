# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""Launch the standalone Speedgoat UDP command bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the bridge launch description."""
    default_params = PathJoinSubstitution(
        [FindPackageShare('drive_udp_bridge'), 'config', 'drive_udp_bridge.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='Absolute path to the drive UDP bridge YAML file.',
            ),
            Node(
                package='drive_udp_bridge',
                executable='drive_udp_bridge',
                name='drive_udp_bridge',
                output='screen',
                parameters=[LaunchConfiguration('params_file')],
            ),
        ]
    )
