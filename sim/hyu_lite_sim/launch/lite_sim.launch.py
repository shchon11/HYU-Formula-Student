"""Step 1 on the simulator WITHOUT Gazebo: the lite sim + the car's own
vehicle-side nodes, so steps 2/3 ('stack', 'mission ...') run unchanged.

    ros2 launch hyu_lite_sim lite_sim.launch.py track:=small_track
    ros2 launch hyu_lite_sim lite_sim.launch.py track:=trackdrive_kase2026 clutter_count:=120 clutter_seed:=5
    ros2 launch hyu_lite_sim lite_sim.launch.py ecu:=ros                   # no bridge, /vehicle/cmd direct
    ros2 launch hyu_lite_sim lite_sim.launch.py ekf:=true rviz:=true       # standalone: + sbg_raw_ekf + RViz

What comes up:
    race_car (hyu_lite_sim)      car + ECU + SBG + perception emulation, ground truth
    vehicle_state.py             /vehicle/as_state, set_mission, reset, ebs (the car's own)
    drive_udp_bridge (ecu:=udp)  the car's own bridge on loopback ports (config/drive_udp_bridge_sim.yaml)
    sbg_raw_ekf (ekf:=true)      only for standalone use; the run scripts start it in step 2
                                 so that 'mission reset' can restart it after a teleport
    rviz2 (rviz:=true)           config/lite_sim.rviz (fixed frame odom = sim world)

auto_button:=true (default) latches the AS button ON in vehicle_state, so
arming a mission (step 3) drives straight away, like the Gazebo flow; with
false the car waits for 'mission go' exactly like the vehicle.
"""
import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node

PKG = 'hyu_lite_sim'


def _truthy(v: str) -> bool:
    return str(v).strip().lower() in ('true', '1', 'on', 'yes')


def _setup(context):
    lc = context.launch_configurations
    share = get_package_share_directory(PKG)
    ecu = lc['ecu'].strip().lower()
    actions = []

    params = {
        'track': lc['track'],
        'ecu_mode': ecu,
        'clutter_count': int(lc['clutter_count']),
        'clutter_seed': int(lc['clutter_seed']),
        'clutter_file': lc['clutter_file'],
        'clutter_save': lc['clutter_save'],
        'datum_latitude': float(lc['datum_latitude']),
        'datum_longitude': float(lc['datum_longitude']),
        'antenna_offset_x': float(lc['antenna_offset_x']),
        'antenna_offset_y': float(lc['antenna_offset_y']),
        'world_frame': lc['world_frame'],
        'sbg.fix_schedule': lc['fix_schedule'],
        'use_sim_time': False,
    }
    for key, name in (('start_x', 'start_x'), ('start_y', 'start_y'), ('start_yaw_deg', 'start_yaw_deg')):
        v = lc[key].strip()
        if v:
            params[name] = float(v)
    # The plant follows the car's geometry file when it is built here: rear
    # axle position in base_footprint and the steering lock (wheel +-90 deg =
    # bicycle +-0.335 rad) live in hyu_sensor_bringup/config/vehicle_mount.yaml
    # `vehicle:` -- the same numbers the planning launch hands Pure Pursuit.
    try:
        mount = os.path.join(get_package_share_directory('hyu_sensor_bringup'), 'config', 'vehicle_mount.yaml')
        with open(mount) as f:
            veh = (yaml.safe_load(f) or {}).get('vehicle') or {}
    except Exception:  # noqa: BLE001 - sensors package not built: lite_sim.yaml values stand
        mount, veh = '', {}
    geometry_note = []
    if 'rear_axle_from_base_m' in veh:
        params['vehicle.rear_axle_x_m'] = float(veh['rear_axle_from_base_m'])
        geometry_note.append(f"rear axle {params['vehicle.rear_axle_x_m']} m ahead of base")
    if 'max_steering_rad' in veh:
        params['vehicle.max_steer_rad'] = float(veh['max_steering_rad'])
        geometry_note.append(f"steering lock {params['vehicle.max_steer_rad']} rad")
    if 'wheelbase_m' in veh:
        params['vehicle.wheelbase_m'] = float(veh['wheelbase_m'])
    actions.append(LogInfo(msg=(
        f"[lite_sim] car geometry from {os.path.basename(mount)}: {', '.join(geometry_note)}" if geometry_note
        else '[lite_sim] car geometry: vehicle_mount.yaml not available, lite_sim.yaml values stand')))
    actions.append(Node(
        package=PKG, executable='lite_sim', name='race_car', output='screen',
        parameters=[lc['params_file'], params]))

    if _truthy(lc['vehicle_state']):
        actions.append(Node(
            package='hyu_planning_bringup', executable='vehicle_state.py', name='vehicle_state',
            output='screen',
            parameters=[{'as_button_initial': _truthy(lc['auto_button']),
                         # the simulator owns the teleport service
                         'serve_reset_vehicle_pos': False}]))
        actions.append(LogInfo(msg=(
            '[lite_sim] AS button ' + ('latched ON: arming a mission drives at once'
                                       if _truthy(lc['auto_button']) else
                                       "OFF: after arming, 'mission go' (or /vehicle/set_as_button) releases the car"))))

    bridge = lc['bridge'].strip().lower()
    if bridge == 'auto':
        bridge = 'true' if ecu == 'udp' else 'false'
    if _truthy(bridge):
        bridge_params = [lc['bridge_params_file']]
        rmr = lc['bridge_require_map_reset'].strip().lower()
        if rmr != 'auto':
            bridge_params.append({'require_map_reset': _truthy(rmr)})
        actions.append(Node(
            package='drive_udp_bridge', executable='drive_udp_bridge', name='drive_udp_bridge',
            output='screen', parameters=bridge_params))
        actions.append(LogInfo(msg=(
            f"[lite_sim] drive_udp_bridge on LOOPBACK ({os.path.basename(lc['bridge_params_file'])}): "
            "commands 127.0.0.1:15000 -> fake ECU, RPM feedback 127.0.0.1:15001 -> /vehicle/wheel_speeds. "
            "Nothing leaves this machine.")))
    elif ecu == 'udp':
        actions.append(LogInfo(msg='[lite_sim] ecu:=udp without the bridge: start drive_udp_bridge with '
                                   'config/drive_udp_bridge_sim.yaml yourself or the car will not move.'))

    if _truthy(lc['ekf']):
        ekf_params = [{'use_sim_time': False,
                       'datum_latitude': float(lc['datum_latitude']),
                       'datum_longitude': float(lc['datum_longitude']),
                       'antenna_offset_x': float(lc['antenna_offset_x']),
                       'antenna_offset_y': float(lc['antenna_offset_y']),
                       'allow_gnss_denied_init': True}]
        if lc['ekf_params_file'].strip():
            ekf_params.append(lc['ekf_params_file'].strip())   # overrides (evaluation A/B, tuning)
        actions.append(Node(
            package='hyu_localization', executable='sbg_raw_ekf', name='sbg_raw_ekf', output='screen',
            parameters=ekf_params))

    if _truthy(lc['rviz']):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(share, 'config', 'lite_sim.rviz')]))
    return actions


def generate_launch_description():
    share = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('track', default_value='small_track',
                              description='eufs_tracks csv name or a csv path'),
        DeclareLaunchArgument('ecu', default_value='udp', choices=['udp', 'ros'],
                              description='udp = through drive_udp_bridge on loopback (the car path); '
                                          'ros = /vehicle/cmd direct, no bridge'),
        DeclareLaunchArgument('clutter_count', default_value='60',
                              description='off-track objects that show up as unknown-colour cones'),
        DeclareLaunchArgument('clutter_seed', default_value='1'),
        DeclareLaunchArgument('clutter_file', default_value='',
                              description='load clutter from this yaml (clutter_tool) instead of generating'),
        DeclareLaunchArgument('clutter_save', default_value='',
                              description='write the generated clutter to this yaml'),
        DeclareLaunchArgument('fix_schedule', default_value='',
                              description='receiver fix schedule "t:preset,..." presets rtk_fixed rtk_float dgps single outage'),
        DeclareLaunchArgument('datum_latitude', default_value='37.5552263',
                              description='lat of the world origin (give sbg_raw_ekf the same datum)'),
        DeclareLaunchArgument('datum_longitude', default_value='127.0454965'),
        DeclareLaunchArgument('antenna_offset_x', default_value='1.25',
                              description='primary antenna in base_footprint; must equal sbg_raw_ekf antenna_offset_x'),
        DeclareLaunchArgument('antenna_offset_y', default_value='0.0'),
        DeclareLaunchArgument('start_x', default_value='', description='override the csv car_start'),
        DeclareLaunchArgument('start_y', default_value=''),
        DeclareLaunchArgument('start_yaw_deg', default_value=''),
        DeclareLaunchArgument('world_frame', default_value='odom',
                              description='frame the ground truth is published in (== EKF frame with the datum)'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'lite_sim.yaml')),
        DeclareLaunchArgument('vehicle_state', default_value='true'),
        DeclareLaunchArgument('auto_button', default_value='true',
                              description='latch the AS button ON so a mission arms straight into driving'),
        DeclareLaunchArgument('bridge', default_value='auto', choices=['auto', 'true', 'false'],
                              description='auto = with ecu:=udp'),
        DeclareLaunchArgument('bridge_params_file',
                              default_value=os.path.join(share, 'config', 'drive_udp_bridge_sim.yaml')),
        DeclareLaunchArgument('bridge_require_map_reset', default_value='auto',
                              choices=['auto', 'true', 'false'],
                              description='auto = yaml (true: the bridge resets /graph_slam/reset before '
                                          'enabling, as on the car); false for a step-1-only run without SLAM'),
        DeclareLaunchArgument('ekf_params_file', default_value='',
                              description='extra sbg_raw_ekf parameter yaml (evaluation A/B, tuning)'),
        DeclareLaunchArgument('ekf', default_value='false',
                              description='also start sbg_raw_ekf here (standalone use; the run scripts '
                                          'start it in step 2)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
