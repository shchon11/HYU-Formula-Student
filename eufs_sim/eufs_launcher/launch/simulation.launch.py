import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _default_python_executable():
    """Pick the interpreter that has ultralytics/torch for the YOLO node.

    Order: active conda env -> the workspace YOLO venv (~/fsk/.venv-yolo,
    a --system-site-packages venv with CUDA torch + ultralytics) -> the
    interpreter running this launch.
    """
    conda_prefix = os.environ.get('CONDA_PREFIX', '').strip()
    if conda_prefix:
        conda_python = Path(conda_prefix) / 'bin' / 'python3'
        if conda_python.exists():
            return str(conda_python)
    eufs_master = os.environ.get('EUFS_MASTER', str(Path.home() / 'fsk'))
    venv_python = Path(eufs_master) / '.venv-yolo' / 'bin' / 'python3'
    if venv_python.exists():
        return str(venv_python)
    return sys.executable


PERCEPTION_LAUNCH_ARGUMENTS = [
    (
        'perception_camera_view_distance',
        '8',
        'Camera range used by the /cones simulated perception plugin.',
    ),
    (
        'perception_lidar_view_distance',
        '100',
        'Radial lidar range used by the /cones simulated perception plugin.',
    ),
    (
        'perception_lidar_x_view_distance',
        '20',
        'Forward/back lidar clipping range used by simulated perception.',
    ),
    (
        'perception_lidar_y_view_distance',
        '20',
        'Lateral lidar clipping range used by simulated perception.',
    ),
    (
        'perception_lidar_min_view_distance',
        '1',
        'Minimum lidar range used by simulated perception.',
    ),
    (
        'perception_camera_min_view_distance',
        '0.5',
        'Minimum camera range used by simulated perception.',
    ),
    (
        'perception_camera_fov',
        '2.8',
        'Camera FOV in radians used by the /cones simulated perception plugin.',
    ),
    (
        'perception_lidar_fov',
        '3.141593',
        'Lidar FOV in radians used by the /cones simulated perception plugin.',
    ),
    (
        'perception_lidar_on',
        'false',
        'Whether lidar-only cones are included in simulated perception.',
    ),
    (
        'perception_detection_probability',
        '0.9',
        'Probability that an in-range /cones simulated perception cone is published.',
    ),
    (
        'camera_cones_view_distance',
        '13',
        'Camera range used by /camera_*/cones oracle topics.',
    ),
    (
        'camera_cones_min_view_distance',
        '0.5',
        'Minimum camera range used by /camera_*/cones oracle topics.',
    ),
    (
        'camera_cones_fov',
        '1.4',
        'Camera FOV in radians used by /camera_*/cones oracle topics.',
    ),
    (
        'camera_cones_detection_probability',
        '1.0',
        'Probability that an in-range /camera_*/cones oracle cone is published.',
    ),
]


def _validate_perception_wiring(context):
    """Fail fast on /cones wiring mistakes.

    launch_group semantics are inverted vs. intuition: 'no_perception' turns the
    raw camera/lidar sensors OFF and makes the sim publish simulated cones on
    /cones; 'default' turns the raw sensors ON and publishes no simulated cones.
    """
    perception = LaunchConfiguration('perception').perform(context).lower()
    launch_group = LaunchConfiguration('launch_group').perform(context)
    perception_on = perception in ('true', '1')

    if perception_on and launch_group == 'no_perception':
        raise RuntimeError(
            "perception:=true cannot be combined with launch_group:=no_perception: "
            "'no_perception' removes the ZED/velodyne sensors (fusion would be starved) "
            "AND publishes simulated cones on /cones (two publishers would collide). "
            "Use launch_group:=default with perception:=true for the real pipeline, "
            "or drop perception:=true to use simulated cones.")

    if not perception_on and launch_group != 'no_perception':
        return [LogInfo(msg=(
            "[simulation.launch.py] WARNING: perception:=false with "
            f"launch_group:={launch_group} means NOTHING publishes /cones — "
            "graph SLAM / planning will silently receive no cone observations. "
            "Pass perception:=true (real YOLO+fusion pipeline) or "
            "launch_group:=no_perception (simulated cones)."))]
    return []


def generate_launch_description():
    perception_argument_declarations = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in PERCEPTION_LAUNCH_ARGUMENTS
    ]
    perception_launch_arguments = [
        (name, LaunchConfiguration(name))
        for name, _, _ in PERCEPTION_LAUNCH_ARGUMENTS
    ]

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
            default_value='false',
            description="Condition to use ground truth transform"),

        DeclareLaunchArgument(
            name='pub_ground_truth',
            default_value='true',
            description="Condition to publish ground truth"),

        # Set to 'no_perception' to turn off the perception code and use ground truth cones.
        DeclareLaunchArgument(
            name='launch_group',
            default_value='default',
            description="Determines which launch files are used in the state_machine node"),
        *perception_argument_declarations,

        # --- eufs_perception_baseline node integration ---
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

        # Runs after all arguments above are declared; fails fast on /cones
        # wiring mistakes (see _validate_perception_wiring docstring).
        OpaqueFunction(function=_validate_perception_wiring),

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
                *perception_launch_arguments,
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
