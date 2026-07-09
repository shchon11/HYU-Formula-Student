from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.launch_description_sources import FrontendLaunchDescriptionSource


PERCEPTION_LAUNCH_ARGUMENTS = [
    (
        'perception_camera_view_distance',
        '15',
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
        '2.09',
        'Camera FOV in radians used by the /cones simulated perception plugin.',
    ),
    (
        'perception_lidar_fov',
        '3.141593',
        'Lidar FOV in radians used by the /cones simulated perception plugin.',
    ),
    (
        'perception_lidar_on',
        'true',
        'Whether lidar-only cones are included in simulated perception.',
    ),
    (
        'perception_detection_probability',
        '1.0',
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
        '2.09',
        'Camera FOV in radians used by /camera_*/cones oracle topics.',
    ),
    (
        'camera_cones_detection_probability',
        '1.0',
        'Probability that an in-range /camera_*/cones oracle cone is published.',
    ),
]


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
            default_value='true',
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
    ])
