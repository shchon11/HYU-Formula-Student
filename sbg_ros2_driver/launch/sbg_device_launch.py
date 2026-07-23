import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sbg_driver'),
        'config',
        'sbg_device_uart_default.yaml'
    )

    # NTRIP client is opt-in: pass ntrip:=true plus the caster credentials to
    # feed RTCM to the device for RTK. Off by default, so a plain launch is
    # unchanged. The driver forwards ntrip_client/rtcm when rtcm.subscribe:true
    # (set in the config).
    ntrip_args = [
        DeclareLaunchArgument('ntrip', default_value='false',
                              description='Start the NTRIP client for RTK'),
        DeclareLaunchArgument('host', default_value='',
                              description='NTRIP caster hostname'),
        DeclareLaunchArgument('port', default_value='2101'),
        DeclareLaunchArgument('mountpoint', default_value=''),
        DeclareLaunchArgument('username', default_value=''),
        DeclareLaunchArgument('password', default_value=''),
        DeclareLaunchArgument('send_gga', default_value='true',
                              description='VRS: uplink rover GGA (false for a single base)'),
        DeclareLaunchArgument('initial_latitude', default_value='nan'),
        DeclareLaunchArgument('initial_longitude', default_value='nan'),
    ]
    ntrip_types = {
        'host': str, 'port': int, 'mountpoint': str, 'username': str,
        'password': str, 'send_gga': bool,
        'initial_latitude': float, 'initial_longitude': float,
    }
    ntrip_params = {
        k: ParameterValue(LaunchConfiguration(k), value_type=t)
        for k, t in ntrip_types.items()
    }

    return LaunchDescription(ntrip_args + [
        Node(
            package='sbg_driver',
            # name='sbg_device_1',
            executable='sbg_device',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='sbg_driver',
            executable='ntrip_client',
            name='ntrip_client',
            output='screen',
            parameters=[ntrip_params],
            condition=IfCondition(LaunchConfiguration('ntrip')),
        ),
    ])
