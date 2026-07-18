import os
from os.path import join
from os.path import isfile

import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


PERCEPTION_XACRO_ARGS = [
    (
        'perception_camera_view_distance',
        '12',
        'Camera range used by the /perception/cones simulated perception plugin.',
    ),
    (
        'perception_lidar_view_distance',
        '15',
        'Radial lidar range used by the /perception/cones simulated perception plugin.',
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
        'Camera FOV in radians used by the /perception/cones simulated perception plugin.',
    ),
    (
        'perception_lidar_fov',
        '3.141593',
        'Lidar FOV in radians used by the /perception/cones simulated perception plugin.',
    ),
    (
        'perception_camera_noise_percentage',
        '0.0',
        'Blend of camera depth noise vs lidar noise on /perception/cones positions; '
        '0.0 = lidar-accurate positions like the real fusion pipeline.',
    ),
    (
        'perception_lidar_on',
        'true',
        'Whether lidar-only cones are included in simulated perception.',
    ),
    (
        'perception_detection_probability',
        '1.0',
        'Probability that an in-range /perception/cones simulated perception cone is published.',
    ),
    (
        'camera_cones_view_distance',
        '13',
        'Camera range used by /camera_*/perception/cones oracle topics.',
    ),
    (
        'camera_cones_min_view_distance',
        '0.5',
        'Minimum camera range used by /camera_*/perception/cones oracle topics.',
    ),
    (
        'camera_cones_fov',
        '2.09',
        'Camera FOV in radians used by /camera_*/perception/cones oracle topics.',
    ),
    (
        'camera_cones_detection_probability',
        '1.0',
        'Probability that an in-range /camera_*/perception/cones oracle cone is published.',
    ),
]

# Procedural bump field. Off by default: it is real geometry in the world, so
# turning it on changes what the LiDAR sees, and no run should get that without
# asking for it.
TERRAIN_XACRO_ARGS = [
    (
        'terrain',
        'false',
        'Insert a procedural bump field into the world. The LiDAR ray-traces it '
        'and the car rides it, so the floor stops being perfectly flat.',
    ),
    (
        'terrain_seed',
        '7',
        'Seed for the bump field. Same seed = same bumps in the same places, '
        'every run; change it to drive a different surface.',
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


def spawn_car(context, *args, **kwargs):
    # Get the values of the arguments
    launch_group = get_argument(context, 'launch_group')
    namespace = get_argument(context, 'namespace')
    robot_name = get_argument(context, 'robot_name')
    vehicle_model = get_argument(context, 'vehicleModel')
    command_mode = get_argument(context, 'commandMode')
    vehicle_model_config = get_argument(context, "vehicleModelConfig")
    publish_tf = get_argument(context, 'publish_gt_tf')
    pub_ground_truth = get_argument(context, 'pub_ground_truth')
    use_sim_time = get_argument(context, 'use_sim_time').lower() == 'true'
    x = get_argument(context, 'x')
    y = get_argument(context, 'y')
    z = get_argument(context, 'z')
    roll = get_argument(context, 'roll')
    pitch = get_argument(context, 'pitch')
    yaw = get_argument(context, 'yaw')
    perception_mappings = {
        name: get_argument(context, name)
        for name, _, _ in PERCEPTION_XACRO_ARGS
    }
    terrain_mappings = {
        name: get_argument(context, name)
        for name, _, _ in TERRAIN_XACRO_ARGS
    }

    simulate_perception = 'true' if launch_group == 'no_perception' else 'false'
    config_file = join(get_package_share_directory('eufs_racecar'), 'robots', robot_name,
                       vehicle_model_config)
    noise_file = join(get_package_share_directory('eufs_models'), 'config', 'noise.yaml')
    recolor_config = join(get_package_share_directory('eufs_plugins'), 'config',
                          'cone_recolor.yaml')
    bounding_boxes_file = os.path.join(get_package_share_directory('eufs_plugins'),
                                       'config', 'boundingBoxes.yaml')

    xacro_path = join(get_package_share_directory('eufs_racecar'),
                      'robots', robot_name, 'robot.urdf.xacro')
    urdf_path = join(get_package_share_directory('eufs_racecar'),
                     'robots', robot_name, 'robot.urdf')

    if not isfile(urdf_path):
        os.mknod(urdf_path)

    doc = xacro.process_file(xacro_path,
                             mappings={
                                 'robot_name': robot_name,
                                 'vehicle_model': vehicle_model,
                                 'command_mode': command_mode,
                                 'config_file': config_file,
                                 'noise_config': noise_file,
                                 'recolor_config': recolor_config,
                                 'publish_tf': publish_tf,
                                 'simulate_perception': simulate_perception,
                                 'pub_ground_truth': pub_ground_truth,
                                 'bounding_box_settings': bounding_boxes_file,
                                 **perception_mappings,
                                 **terrain_mappings,
                             })
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    spawn_robot = Node(
        name='spawn_robot',
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', namespace,
            '-file', urdf_path,
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '-timeout', '60.0',
            '--ros-args', '--log-level', 'warn'
        ]
    )

    rqt_perspective_file = join(get_package_share_directory('eufs_rqt'),
                                'config', 'eufs_sim.perspective')
    rqt_gui = Node(
        name='eufs_sim_rqt',
        package='rqt_gui',
        executable='rqt_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--force-discover', '--perspective-file',
                   str(rqt_perspective_file)],
        condition=IfCondition(LaunchConfiguration('show_rqt_gui'))
    )

    # Always the shipped config so every checkout sees the same displays.
    # A stale ~/.rviz2/default.rviz used to silently override this and hide
    # new HUD/preset displays; pass rviz_config:=<path> to use another file.
    rviz_config_file = LaunchConfiguration('rviz_config').perform(context)
    if not rviz_config_file:
        rviz_config_file = join(
            get_package_share_directory('eufs_launcher'), 'config', 'default.rviz')

    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    actions = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    TimerAction(
                        period=1.0,
                        actions=[rviz, rqt_gui]
                    )
                ]
            )
        ),

        TimerAction(
            period=5.0,
            actions=[spawn_robot]
        ),

        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 200.0,
                'use_sim_time': use_sim_time,
            }],
            remappings=[('/joint_states', '/eufs/joint_states')],
            arguments=['--ros-args', '--log-level', 'warn']
        ),
    ]

    # The gazebo ray sensor publishes the geometrically ideal scan on
    # /velodyne_points_ideal; this bridge owns /velodyne_points, adding motion
    # distortion, far-grazing-ground dropout and the real velodyne driver's
    # ring/time field layout (see eufs_sensors/scripts/lidar_realism.py).
    # Only spawned when the raw sensors exist at all; spin_hz must match the
    # VLP-16R macro's hz in robots/*/robot.urdf.xacro.
    if simulate_perception == 'false':
        actions.append(Node(
            name='lidar_realism',
            package='eufs_sensors',
            executable='lidar_realism',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'spin_hz': 10.0,
            }],
        ))

    return actions


def generate_launch_description():
    perception_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in PERCEPTION_XACRO_ARGS + TERRAIN_XACRO_ARGS
    ]

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('launch_group', default_value='default',
                              description='The launch group (default or '
                                          'no_perception)'),

        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz'),

        DeclareLaunchArgument('rviz_config', default_value='',
                              description='RViz config path; empty = the '
                                          'shipped eufs_launcher default.rviz'),

        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the simulator clock'),

        DeclareLaunchArgument('ros_localhost_only',
                              default_value=EnvironmentVariable(
                                  'ROS_LOCALHOST_ONLY', default_value='1'),
                              description='Limit ROS discovery to localhost'),

        DeclareLaunchArgument('show_rqt_gui', default_value='true',
                              description='Show the RQT GUI (with '
                                          'ros_can_sim GUI and EUFS Robot '
                                          'Steering GUI)'),

        DeclareLaunchArgument('namespace', default_value='eufs',
                              description='Namespace of the gazebo robot'),

        DeclareLaunchArgument('robot_name', default_value='eufs',
                              description='The name of the robot (must be '
                                          'directory in eufs_racecar/robots '
                                          'called '
                                          '{robot_name} with '
                                          'robot.urdf.xacro and {'
                                          'vehicle_model_config}'),

        DeclareLaunchArgument('vehicleModel', default_value='DynamicBicycle',
                              description='The vehicle model class to use in '
                                          'the gazebo_ros_race_car_model'),

        DeclareLaunchArgument('commandMode', default_value='acceleration',
                              description='Determines whether to use '
                                          'acceleration or velocity to '
                                          'control the vehicle'),

        DeclareLaunchArgument('vehicleModelConfig',
                              default_value='configDry.yaml',
                              description="Determines the file from which "
                                          "the vehicle model parameters are "
                                          "read"),

        DeclareLaunchArgument('publish_gt_tf', default_value='true',
                              description='If the gazebo_ros_race_car_model '
                                          'should publish the ground truth '
                                          'tf'),
        DeclareLaunchArgument('pub_ground_truth', default_value='true',
                              description='Publish ground truth topics'),
        *perception_arguments,

        DeclareLaunchArgument('x', default_value='0',
                              description='Vehicle initial x position'),
        DeclareLaunchArgument('y', default_value='0',
                              description='Vehicle initial y position'),
        DeclareLaunchArgument('z', default_value='0',
                              description='Vehicle initial z position'),
        DeclareLaunchArgument('roll', default_value='0',
                              description='Vehicle initial roll'),
        DeclareLaunchArgument('pitch', default_value='0',
                              description='Vehicle initial pitch'),
        DeclareLaunchArgument('yaw', default_value='0',
                              description='Vehicle initial yaw'),

        SetEnvironmentVariable(
            name='ROS_LOCALHOST_ONLY',
            value=LaunchConfiguration('ros_localhost_only')),

        # Spawn the car!!!
        OpaqueFunction(function=spawn_car)
    ])


def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)
