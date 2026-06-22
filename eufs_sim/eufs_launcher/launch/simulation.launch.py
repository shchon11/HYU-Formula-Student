import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _default_python_executable():
    conda_prefix = os.environ.get('CONDA_PREFIX', '').strip()
    if conda_prefix:
        conda_python = Path(conda_prefix) / 'bin' / 'python3'
        if conda_python.exists():
            return str(conda_python)
    return sys.executable


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='track',
            default_value='small_track',
            description="Determines which track is launched"),

        DeclareLaunchArgument(
            name='vehicleModel',
            default_value='DynamicBicycle',
            description="Determines which vehicle model is used"),

        DeclareLaunchArgument(
            name='vehicleModelConfig',
            default_value='configDry.yaml',
            description="Determines the file from which the vehicle model parameters are read"),

        DeclareLaunchArgument(
            name='commandMode',
            default_value='acceleration',
            description="Determines the vehicle control mode (acceleration or velocity)"),

        DeclareLaunchArgument(
            name='robot_name',
            default_value='eufs',
            description="Determines which robot urdf is used in the sim"),

        DeclareLaunchArgument(
            name='gazebo_gui',
            default_value='false',
            description="Condition to launch the Gazebo GUI"),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description="Use the simulator clock"),

        DeclareLaunchArgument(
            name='ros_localhost_only',
            default_value='1',
            description="Limit ROS discovery to localhost"),

        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description="Condition to launch the Rviz GUI"),

        DeclareLaunchArgument(
            name='show_rqt_gui',
            default_value='true',
            description="Show the RQT GUI"),

        DeclareLaunchArgument(
            name='publish_gt_tf',
            default_value='true',
            description="Condition to use ground truth transform"),

        DeclareLaunchArgument(
            name='pub_ground_truth',
            default_value='true',
            description="Condition to publish ground truth"),

        DeclareLaunchArgument(
            name='perception',
            default_value='false',
            description="Launch eufs_perception_baseline with the simulator"),

        DeclareLaunchArgument(
            name='perception_bbox_source',
            default_value='yolov8',
            description="Perception bbox source: simulated or yolov8"),

        DeclareLaunchArgument(
            name='perception_output_cones_topic',
            default_value='/cones',
            description="Perception output cone topic"),

        DeclareLaunchArgument(
            name='perception_publish_fusion_debug',
            default_value='true',
            description="Publish fusion debug topics for RViz"),

        DeclareLaunchArgument(
            name='perception_publish_yolo_debug_image',
            default_value='false',
            description="Publish YOLO bbox debug image"),

        DeclareLaunchArgument(
            name='perception_python_executable',
            default_value=_default_python_executable(),
            description="Python interpreter used to run perception nodes"),

        # Set to 'no_perception' to turn off the perception code and use ground truth cones.
        DeclareLaunchArgument(
            name='launch_group',
            default_value='default',
            description="Determines which launch files are used in the state_machine node"),

        SetEnvironmentVariable(
            name='ROS_LOCALHOST_ONLY',
            value=LaunchConfiguration('ros_localhost_only')),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('eufs_tracks'),
                    'launch',
                    PythonExpression(["'", LaunchConfiguration('track'), "'", "+ '.launch'"])
                ]),
            ),
            launch_arguments=[
                ('vehicleModel', LaunchConfiguration('vehicleModel')),
                ('vehicleModelConfig', LaunchConfiguration('vehicleModelConfig')),
                ('commandMode', LaunchConfiguration('commandMode')),
                ('robot_name', LaunchConfiguration('robot_name')),
                ('gazebo_gui', LaunchConfiguration('gazebo_gui')),
                ('use_sim_time', LaunchConfiguration('use_sim_time')),
                ('ros_localhost_only', LaunchConfiguration('ros_localhost_only')),
                ('rviz', LaunchConfiguration('rviz')),
                ('show_rqt_gui', LaunchConfiguration('show_rqt_gui')),
                ('publish_gt_tf', LaunchConfiguration('publish_gt_tf')),
                ('pub_ground_truth', LaunchConfiguration('pub_ground_truth')),
                ('launch_group', LaunchConfiguration('launch_group')),
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('eufs_perception_baseline'),
                    'launch',
                    'perception_baseline.launch.py'
                ]),
            ),
            condition=IfCondition(LaunchConfiguration('perception')),
            launch_arguments=[
                ('bbox_source', LaunchConfiguration('perception_bbox_source')),
                ('output_cones_topic',
                 LaunchConfiguration('perception_output_cones_topic')),
                ('use_sim_time', LaunchConfiguration('use_sim_time')),
                ('publish_fusion_debug',
                 LaunchConfiguration('perception_publish_fusion_debug')),
                ('publish_yolo_debug_image',
                 LaunchConfiguration('perception_publish_yolo_debug_image')),
                ('python_executable',
                 LaunchConfiguration('perception_python_executable')),
            ],
        ),
    ])
