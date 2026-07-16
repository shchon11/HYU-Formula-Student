from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = (
    ("use_sim_time", "true", "Use the simulator clock in planning and control nodes."),
    ("planner_source", "slam", "Global path source: slam or csv."),
    ("start_graph_slam", "true", "Start graph SLAM with planning."),
    ("graph_slam_ate_monitor", "true", "Start the graph-SLAM CTE monitor."),
    ("local_source_mode", "slam_map", "Local planner source: slam_map or live_cones."),
    ("local_max_stamp_skew_sec", "0.1", "Local planner cones/odom stamp-skew gate (sec, sim time)."),
    ("local_max_input_age_sec", "0.5", "Local planner input freshness gate (sec, sim time)."),
    ("local_max_start_distance_m", "4.0", "Max distance from ego to the local path start (m)."),
    ("enable_hud", "true", "Start the planning HUD."),
    ("planning_state_topic", "/planning/state", "LOCAL/GLOBAL/STOP state."),
    ("stop_request_topic", "/planning/stop_request", "Planning safety stop request."),
    ("pure_pursuit_cmd_topic", "/cmd/pure_pursuit", "Pure Pursuit candidate command."),
    ("tmpc_cmd_topic", "/cmd/tmpc", "TMPC candidate command."),
    ("final_cmd_topic", "/cmd", "Selector-owned final command."),
    ("selector_status_topic", "/tmpc/cmd_selector/status", "Selector state output."),
    ("tmpc_vehicle_state_topic", "/tmpc/vehicle_state", "Formula vehicle-state input."),
    (
        "tmpc_performance_trajectory_topic",
        "/tmpc/trajectory_performance",
        "Formula performance trajectory.",
    ),
    (
        "tmpc_emergency_trajectory_topic",
        "/tmpc/trajectory_emergency",
        "Formula emergency trajectory.",
    ),
    ("tmpc_output_topic", "/tmpc/output", "Raw Formula TMPC output."),
    ("tmpc_valid_topic", "/tmpc/cmd_valid", "Output-bridge validity heartbeat."),
    ("localization_topic", "/localization/ego_odom", "Vehicle-state localization input."),
    ("car_state_topic", "/wheel_odometry/car_state", "Vehicle-state dynamics input."),
    ("wheel_speeds_topic", "/ros_can/wheel_speeds", "Vehicle-state wheel-speed input."),
    ("publish_rate_hz", "100.0", "Bridge and selector command rate."),
    ("state_timeout_sec", "0.25", "Planning state freshness limit."),
    ("stop_timeout_sec", "0.25", "Stop-request freshness limit."),
    ("local_command_timeout_sec", "0.25", "Pure Pursuit command freshness limit."),
    ("tmpc_command_timeout_sec", "0.1", "TMPC command freshness limit."),
    ("tmpc_valid_timeout_sec", "0.1", "TMPC valid heartbeat freshness limit."),
    ("tmpc_ready_dwell_sec", "0.1", "Continuous readiness required for takeover."),
    ("tmpc_steering_min_rad", "-0.52", "TMPC minimum road-wheel steering angle."),
    ("tmpc_steering_max_rad", "0.52", "TMPC maximum road-wheel steering angle."),
    ("safe_brake_mps2", "-5.0", "Fail-closed braking acceleration."),
)


def generate_launch_description() -> LaunchDescription:
    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]
    values = {name: LaunchConfiguration(name) for name, _, _ in ARGUMENTS}
    use_sim_time = ParameterValue(values["use_sim_time"], value_type=bool)
    publish_rate = ParameterValue(values["publish_rate_hz"], value_type=float)
    tmpc_steering_min = ParameterValue(values["tmpc_steering_min_rad"], value_type=float)
    tmpc_steering_max = ParameterValue(values["tmpc_steering_max_rad"], value_type=float)
    safe_brake = ParameterValue(values["safe_brake_mps2"], value_type=float)

    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("planning_bringup"), "launch", "local_global_planning.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": values["use_sim_time"],
            "planner_source": values["planner_source"],
            "start_graph_slam": values["start_graph_slam"],
            "graph_slam_ate_monitor": values["graph_slam_ate_monitor"],
            "local_source_mode": values["local_source_mode"],
            "local_max_stamp_skew_sec": values["local_max_stamp_skew_sec"],
            "local_max_input_age_sec": values["local_max_input_age_sec"],
            "local_max_start_distance_m": values["local_max_start_distance_m"],
            "enable_hud": values["enable_hud"],
            "enable_controller": "true",
            "state_topic": values["planning_state_topic"],
            "stop_request_topic": values["stop_request_topic"],
            "controller_cmd_topic": values["pure_pursuit_cmd_topic"],
            "cmd_topic": values["final_cmd_topic"],
            "tmpc_performance_trajectory_topic": values[
                "tmpc_performance_trajectory_topic"
            ],
            "tmpc_emergency_trajectory_topic": values["tmpc_emergency_trajectory_topic"],
            "ego_odom_topic": values["localization_topic"],
            "car_state_topic": values["car_state_topic"],
        }.items(),
    )

    vehicle_state_bridge = Node(
        package="tum_vehicle_state_bridge",
        executable="tum_vehicle_state_bridge",
        name="tum_vehicle_state_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "tum_vehicle_state_bridge/localization_topic": values["localization_topic"],
                "tum_vehicle_state_bridge/car_state_topic": values["car_state_topic"],
                "tum_vehicle_state_bridge/wheel_speeds_topic": values["wheel_speeds_topic"],
                "tum_vehicle_state_bridge/output_topic": values["tmpc_vehicle_state_topic"],
                "tum_vehicle_state_bridge/publish_rate_hz": publish_rate,
            }
        ],
    )

    tmpc_wrapper = Node(
        package="hyu_formular_control",
        executable="hyu_formular_control",
        name="hyu_formular_control",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "hyu_formular_control/vehicle_state_topic": values["tmpc_vehicle_state_topic"],
                "hyu_formular_control/performance_trajectory_topic": values[
                    "tmpc_performance_trajectory_topic"
                ],
                "hyu_formular_control/emergency_trajectory_topic": values[
                    "tmpc_emergency_trajectory_topic"
                ],
                "hyu_formular_control/output_topic": values["tmpc_output_topic"],
                "hyu_formular_control/loop_rate_hz": publish_rate,
                "hyu_formular_control/steering_angle_min_rad": tmpc_steering_min,
                "hyu_formular_control/steering_angle_max_rad": tmpc_steering_max,
            }
        ],
    )

    output_bridge = Node(
        package="tum_mpc_output_bridge",
        executable="tum_mpc_output_bridge",
        name="tum_mpc_output_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "tum_mpc_output_bridge/input_topic": values["tmpc_output_topic"],
                "tum_mpc_output_bridge/output_topic": values["tmpc_cmd_topic"],
                "tum_mpc_output_bridge/valid_topic": values["tmpc_valid_topic"],
                "tum_mpc_output_bridge/output_timeout_sec": ParameterValue(
                    values["tmpc_command_timeout_sec"], value_type=float
                ),
                "tum_mpc_output_bridge/publish_rate_hz": publish_rate,
                "tum_mpc_output_bridge/steering_min_rad": tmpc_steering_min,
                "tum_mpc_output_bridge/steering_max_rad": tmpc_steering_max,
                "tum_mpc_output_bridge/safe_brake_mps2": safe_brake,
            }
        ],
    )

    selector = Node(
        package="tmpc_cmd_selector",
        executable="tmpc_cmd_selector",
        name="tmpc_cmd_selector",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "planning_state_topic": values["planning_state_topic"],
                "stop_request_topic": values["stop_request_topic"],
                "local_command_topic": values["pure_pursuit_cmd_topic"],
                "tmpc_command_topic": values["tmpc_cmd_topic"],
                "tmpc_valid_topic": values["tmpc_valid_topic"],
                "output_topic": values["final_cmd_topic"],
                "status_topic": values["selector_status_topic"],
                "publish_rate_hz": publish_rate,
                "state_timeout_sec": ParameterValue(
                    values["state_timeout_sec"], value_type=float
                ),
                "stop_timeout_sec": ParameterValue(values["stop_timeout_sec"], value_type=float),
                "local_command_timeout_sec": ParameterValue(
                    values["local_command_timeout_sec"], value_type=float
                ),
                "tmpc_command_timeout_sec": ParameterValue(
                    values["tmpc_command_timeout_sec"], value_type=float
                ),
                "tmpc_valid_timeout_sec": ParameterValue(
                    values["tmpc_valid_timeout_sec"], value_type=float
                ),
                "tmpc_ready_dwell_sec": ParameterValue(
                    values["tmpc_ready_dwell_sec"], value_type=float
                ),
                "safe_brake_mps2": safe_brake,
            }
        ],
    )

    return LaunchDescription(
        [
            *declarations,
            planning,
            vehicle_state_bridge,
            tmpc_wrapper,
            output_bridge,
            selector,
        ]
    )
