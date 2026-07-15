from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource


PERCEPTION_LAUNCH_ARGUMENTS = [
    (
        'perception_camera_view_distance',
        '12',
        'Camera range used by the /cones simulated perception plugin.',
    ),
    (
        'perception_lidar_view_distance',
        '15',
        'Radial lidar range used by the /cones simulated perception plugin.',
    ),
    (
        'perception_lidar_x_view_distance',
        '15',
        'Forward/back lidar clipping range used by simulated perception.',
    ),
    (
        'perception_lidar_y_view_distance',
        '10',
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
        '1.91986',
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
        '0.9',
        'Probability that an in-range /cones simulated perception cone is published.',
    ),
    (
        'camera_cones_view_distance',
        '10',
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

# Procedural bump field. Off by default: it is real geometry in the world, so it
# changes what the LiDAR sees, and no run should get that without asking.
TERRAIN_LAUNCH_ARGUMENTS = [
    (
        'terrain',
        'false',
        'Insert a procedural bump field into the world: real geometry the LiDAR '
        'ray-traces and the car rides, instead of a perfectly flat floor.',
    ),
    (
        'terrain_seed',
        '7',
        'Seed for the bump field; the same seed is the same bumps every run.',
    ),
    (
        'terrain_density',
        '0.02',
        'Bumps per square metre.',
    ),
    (
        'terrain_height_mean',
        '0.020',
        'Mean bump peak height [m]. Raise to stress ground-plane removal.',
    ),
    (
        'road_noise',
        'true',
        'Speed-scaled roughness vibration on the body. It is louder than the '
        "car's own load transfer at speed, so turn it off to observe that.",
    ),
]


def _resolve_perception_mode(context):
    """Map the single `perception_mode` knob onto launch_group + perception.

    - real   : real YOLO+LiDAR fusion (launch_group=default, perception=true)
    - sim     : lightweight simulated perception — Gazebo publishes /cones from
                the ground-truth cone plugin (colored in camera FOV, unknown for
                LiDAR-only), no YOLO/fusion nodes (launch_group=no_perception,
                perception=false). Use this when the CPU/GPU can't feed YOLO.
    - manual : leave launch_group/perception exactly as passed (advanced).
    """
    mode = LaunchConfiguration('perception_mode').perform(context).lower()
    if mode == 'real':
        return [
            SetLaunchConfiguration('launch_group', 'default'),
            SetLaunchConfiguration('perception', 'true'),
        ]
    if mode == 'sim':
        return [
            SetLaunchConfiguration('launch_group', 'no_perception'),
            SetLaunchConfiguration('perception', 'false'),
        ]
    if mode == 'manual':
        return []
    raise RuntimeError(
        f"perception_mode must be real|sim|manual, got '{mode}'")


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
    forwarded_arguments = PERCEPTION_LAUNCH_ARGUMENTS + TERRAIN_LAUNCH_ARGUMENTS
    perception_argument_declarations = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in forwarded_arguments
    ]
    perception_launch_arguments = [
        (name, LaunchConfiguration(name))
        for name, _, _ in forwarded_arguments
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

        # High-level perception selector; overrides launch_group/perception
        # unless set to 'manual'. See _resolve_perception_mode.
        DeclareLaunchArgument(
            name='perception_mode',
            default_value='manual',
            description="real | sim | manual — sim uses lightweight Gazebo cones (no YOLO)."),

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

        # These wrap eufs_perception_baseline's launch interface. Anything the
        # baseline does not declare cannot be forwarded: an IncludeLaunchDescription
        # that passes an unknown argument fails the whole launch. The tier-era
        # knobs (bbox_source, monocular/stereo_fallback_enabled, python_executable)
        # went away with the tiers; the node now picks its provenance per cone
        # from what the sensors actually support, so there is nothing to switch.
        DeclareLaunchArgument(
            name='perception_output_cones_topic',
            default_value='/cones',
            description="Perception output cone topic"),

        DeclareLaunchArgument(
            name='perception_publish_debug',
            default_value='true',
            description="Publish cone provenance/covariance markers for RViz"),

        DeclareLaunchArgument(
            name='perception_publish_yolo_debug_image',
            default_value='false',
            description="Publish YOLO bbox debug image"),

        DeclareLaunchArgument(
            name='perception_right_image_topic',
            default_value='/zed/right/image_rect_color',
            description="Right rectified image consumed by perception"),

        DeclareLaunchArgument(
            name='perception_motion_compensation_frame',
            default_value='map',
            description=(
                "Fixed TF frame for cross-time perception transforms; map is "
                "the ground-truth default, use odom when GraphSLAM owns TF"
            )),

        # Resolve perception_mode -> launch_group/perception FIRST, then
        # validate the (possibly resolved) combination.
        OpaqueFunction(function=_resolve_perception_mode),
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
                    'perception.launch.py'
                ]),
            ),
            condition=IfCondition(LaunchConfiguration('perception')),
            launch_arguments=[
                ('output_cones_topic',
                 LaunchConfiguration('perception_output_cones_topic')),
                ('use_sim_time', LaunchConfiguration('use_sim_time')),
                ('publish_debug',
                 LaunchConfiguration('perception_publish_debug')),
                ('publish_yolo_debug_image',
                 LaunchConfiguration('perception_publish_yolo_debug_image')),
                ('right_image_topic',
                 LaunchConfiguration('perception_right_image_topic')),
                ('motion_compensation_frame',
                 LaunchConfiguration('perception_motion_compensation_frame')),
            ],
        ),
    ])
