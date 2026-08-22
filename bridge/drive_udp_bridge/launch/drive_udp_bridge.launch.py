# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""
Launch the Speedgoat UDP bridge, alone (default) or as a bench ECU rig.

    ros2 launch drive_udp_bridge drive_udp_bridge.launch.py               # bridge only
    ros2 launch drive_udp_bridge drive_udp_bridge.launch.py bench:=true   # + AS button
                                                                          #   + vehicle state

Default = just the bridge node, which is what race.sh wants (the sensor
bringup already runs the AS button driver and race.sh runs vehicle_state in
its own pane). bench:=true is the ECU-comms rig without the rest of the
stack: the AS button driver (Jetson header pin 31, when Jetson.GPIO is
available) -> vehicle_state (a mission pre-selected, so the button alone
reaches AS_DRIVING) -> this bridge with require_map_reset off (no SLAM on the
bench). Pressing the button then flips the autonomous-enable byte 0 -> 1 on
the wire, and encoder feedback (once encoder_counts_per_revolution is set)
shows up on /vehicle/wheel_speeds.

Without a button: ros2 service call /vehicle/set_as_button std_srvs/srv/SetBool "{data: true}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _truthy(value: str) -> bool:
    return str(value).strip().lower() in ('true', '1', 'on', 'yes')


def _setup(context):
    lc = context.launch_configurations
    bench = _truthy(lc['bench'])
    as_button = lc['as_button'].strip().lower()
    vehicle_state = lc['vehicle_state'].strip().lower()
    require_map_reset = lc['require_map_reset'].strip().lower()
    actions = []

    # --- the bridge -------------------------------------------------------
    bridge_params = [lc['params_file']]
    if require_map_reset != 'auto':
        bridge_params.append({'require_map_reset': _truthy(require_map_reset)})
    elif bench:
        # No graph SLAM on the bench: raise the autonomous byte at once.
        bridge_params.append({'require_map_reset': False})
    actions.append(Node(
        package='drive_udp_bridge',
        executable='drive_udp_bridge',
        name='drive_udp_bridge',
        output='screen',
        parameters=bridge_params,
    ))

    # --- /vehicle/as_state provider ----------------------------------------
    start_vehicle_state = _truthy(vehicle_state) if vehicle_state != 'auto' else bench
    if start_vehicle_state:
        vs_params = {}
        if bench and lc['bench_mission'].strip():
            vs_params['initial_mission'] = lc['bench_mission'].strip()
        actions.append(Node(
            package='hyu_planning_bringup',
            executable='vehicle_state.py',
            name='vehicle_state',
            output='screen',
            parameters=[vs_params] if vs_params else [],
        ))

    # --- AS button driver (Jetson GPIO) ------------------------------------
    if as_button != 'off' and (as_button == 'on' or bench):
        try:
            import Jetson.GPIO  # noqa: F401
            have_gpio = True
        except Exception:  # noqa: BLE001 - any import failure means no GPIO here
            have_gpio = False
        if as_button == 'on' or have_gpio:
            actions.append(Node(
                package='hyu_sensor_bringup',
                executable='as_button.py',
                name='as_button',
                output='screen',
                parameters=[{
                    'pin': int(lc['as_button_pin']),
                    'active_low': _truthy(lc['as_button_active_low']),
                    'mode': lc['as_button_mode'],
                }],
            ))
        else:
            actions.append(LogInfo(msg=(
                '[bridge] no Jetson.GPIO here -- AS button driver not started; '
                'flip the switch with: ros2 service call /vehicle/set_as_button '
                'std_srvs/srv/SetBool "{data: true}"  (force the driver with as_button:=on)')))
    return actions


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
            DeclareLaunchArgument(
                'bench', default_value='false',
                description='ECU-comms rig: also start the AS button driver and '
                            'vehicle_state (mission pre-selected), and skip the '
                            'SLAM map reset gate. Leave false on the car (race.sh).',
            ),
            DeclareLaunchArgument(
                'as_button', default_value='auto', choices=['auto', 'on', 'off'],
                description='AS button GPIO driver: auto = with bench:=true when '
                            'Jetson.GPIO is available; on = always; off = never.',
            ),
            DeclareLaunchArgument(
                'vehicle_state', default_value='auto', choices=['auto', 'true', 'false'],
                description='/vehicle/as_state provider: auto = with bench:=true.',
            ),
            DeclareLaunchArgument(
                'bench_mission', default_value='TRACK_DRIVE',
                description='Mission pre-selected in vehicle_state under bench:=true '
                            '(CanState AMI name); "" = none (button alone stays AS_OFF).',
            ),
            DeclareLaunchArgument(
                'require_map_reset', default_value='auto', choices=['auto', 'true', 'false'],
                description='Bridge require_map_reset override: auto = yaml value, '
                            'or false under bench:=true.',
            ),
            DeclareLaunchArgument(
                'as_button_pin', default_value='31',
                description='Jetson 40-pin header pin (BOARD numbering) of the AS button.',
            ),
            DeclareLaunchArgument(
                'as_button_active_low', default_value='true',
                description='pressed reads 0 (button to GND with a pull-up)',
            ),
            DeclareLaunchArgument(
                'as_button_mode', default_value='toggle', choices=['toggle', 'level'],
                description='toggle = each press flips AS; level = switch position',
            ),
            OpaqueFunction(function=_setup),
        ]
    )
