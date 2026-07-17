from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = (
    ("use_sim_time", "true", "Use the simulator clock for every planning node."),
    ("start_graph_slam", "true", "Start graph SLAM with the planning graph."),
    ("graph_slam_gui", "false", "Start the graph SLAM GUI."),
    ("graph_slam_ate_monitor", "true", "Start the path-tracking CTE monitor (HUD + /planning/cte)."),
    ("graph_slam_localization_mode", "false", "Localize against a saved graph SLAM map."),
    ("graph_slam_load_map_path", "", "Saved graph SLAM map for localization mode."),
    ("graph_slam_publish_tf", "true", "Allow graph SLAM to publish the map TF."),
    # Fused INS odometry (RTK-fused yaw, mode-managed DR fallback) — the
    # always-DR wheel odometry carried a ~1.1 deg constant heading error that
    # fought the GNSS anchors inside the graph (2026-07-18 budget analysis).
    ("car_state_topic", "/ins_odom/car_state", "Graph SLAM motion input."),
    ("cones_topic", "/perception/cones", "Live cone observations for local planning and state."),
    ("cone_map_topic", "/localization/cone_map", "Graph SLAM cone-map output."),
    ("ego_odom_topic", "/localization/ego_odom", "Planner-facing odometry."),
    ("graph_slam_status_topic", "/localization/status", "Graph SLAM lifecycle status."),
    ("graph_slam_map_converged_topic", "/localization/map_converged", "Graph SLAM map-ready status."),
    ("global_waypoints_topic", "/global_waypoints", "Latched full global waypoint path."),
    ("global_path_valid_topic", "/planning/global_path_valid", "Global-path validity heartbeat."),
    ("global_path_reason_topic", "/planning/global_path_reason", "Why the global path is invalid (latched)."),
    ("global_path_waypoints_topic", "/planning/global_path_waypoints", "Global rolling waypoint window."),
    ("global_path_topic", "/planning/global_path_waypoints/path", "Global rolling window visualization."),
    ("frenet_odom_topic", "/planning/frenet_odom", "Frenet odometry topic."),
    ("tmpc_performance_trajectory_topic", "/planning/trajectory_performance", "Formula TMPC performance trajectory."),
    ("tmpc_emergency_trajectory_topic", "/planning/trajectory_emergency", "Formula TMPC emergency trajectory."),
    ("local_waypoints_topic", "/planning/local_waypoints", "Local planner waypoint output."),
    ("local_path_topic", "/planning/local_waypoints/path", "Local planner path visualization."),
    ("local_path_valid_topic", "/planning/local_path_valid", "Local-path validity heartbeat."),
    ("local_path_reason_topic", "/planning/local_path_reason", "Why the local path is invalid (heartbeat)."),
    ("state_topic", "/planning/state", "Planning state output."),
    ("path_source_topic", "/planning/path_source", "State-selected path source."),
    ("lap_count_topic", "/planning/lap_count", "Planning lap-count output."),
    ("lap_time_last_topic", "/planning/lap_time_last", "Last completed lap time (orange-gate)."),
    ("lap_time_best_topic", "/planning/lap_time_best", "Best lap time (orange-gate)."),
    ("stop_request_topic", "/planning/stop_request", "State-machine stop request."),
    ("planning_debug_topic", "/planning/debug", "State-machine debug output."),
    ("selected_path_topic", "/planning/path", "Selector-owned controller path."),
    ("selected_path_viz_topic", "/planning/debug/path", "Selector path visualization."),
    ("selected_path_valid_topic", "/planning/selected_path_valid", "Selector validity heartbeat."),
    ("global_handoff_ready_topic", "/planning/global_handoff_ready", "Selector handoff readiness."),
    ("selector_debug_topic", "/planning/hyu_path_selector/debug", "Selector debug output."),
    ("stop_zone_s_start_topic", "/planning/stop_zone/s_start", "Final stop-zone start position."),
    ("stop_zone_s_end_topic", "/planning/stop_zone/s_end", "Final stop-zone end position."),
    ("stop_zone_valid_topic", "/planning/stop_zone/valid", "Final stop-zone validity."),
    ("local_max_stamp_skew_sec", "0.1", "Local planner cones/odom stamp-skew gate (sec, sim time)."),
    ("local_max_input_age_sec", "0.5", "Local planner input freshness gate (sec, sim time)."),
    ("local_max_start_distance_m", "4.0", "Max distance from ego to the local path start (m)."),
    ("enable_controller", "true", "Start the Pure Pursuit controller."),
    (
        "controller_max_speed_mps",
        "10.0",
        "Pure Pursuit speed cap override (trackdrive yaml default 10.0). The TMPC "
        "hybrid launch lowers it to the TMPC reference cap so takeover/fallback "
        "hand the tube MPC a state inside its feasible envelope.",
    ),
    ("cmd_topic", "/vehicle/cmd", "Command topic monitored by the HUD."),
    (
        "controller_cmd_topic",
        LaunchConfiguration("cmd_topic"),
        "Pure Pursuit command output; defaults to cmd_topic.",
    ),
    ("enable_hud", "true", "Start the RViz stack HUD overlay aggregator."),
    ("hud_topic", "/planning/stack_hud", "Stack HUD board overlay."),
    ("hud_banner_topic", "/planning/stack_hud_banner", "Stack HUD banner overlay."),
    # Skidpad mission: the director gates the SLAM cone map by mission phase
    # (entry -> right circle xN -> left circle xN -> exit -> stop) and the
    # unchanged local planner drives that feed. The global planner is not
    # started, and graph SLAM keeps mapping (no auto freeze) because the left
    # circle is still unseen when the car first re-passes the start.
    ("skidpad", "false", "Skidpad mission: phase-gated local-only planning."),
    ("skidpad_right_laps", "2", "Skidpad right-circle laps before switching left."),
    ("skidpad_left_laps", "2", "Skidpad left-circle laps before exiting."),
    ("skidpad_cone_map_topic", "/skidpad/cone_map", "Phase-gated cone map for the local planner."),
    ("skidpad_phase_topic", "/skidpad/phase", "Latched skidpad mission phase."),
    # Acceleration mission: local-only straight-line sprint. Like skidpad the
    # global planner stays down, but there is no phase director and no map
    # gating -- the local planner drives the full SLAM cone map straight down
    # the corridor and the controller brakes on its own when the cones run out.
    ("acceleration", "false", "Acceleration mission: local-only straight-line, no global planner."),
)

PARAMETER_FILES = (
    ("graph_slam_params_file", "hyu_localization", "graph_slam.yaml", "Graph SLAM parameter file."),
    ("global_params_file", "hyu_global_planner", "hyu_global_planner.yaml", "Global planner and Frenet parameter file."),
    ("local_params_file", "hyu_local_planner", "hyu_local_planner.yaml", "Local planner parameter file."),
    ("state_params_file", "hyu_state_machine", "planning_hyu_state_machine.yaml", "Planning state-machine parameter file."),
    ("selector_params_file", "hyu_path_selector", "hyu_path_selector.yaml", "Path selector parameter file."),
    ("controller_params_file", "hyu_pure_pursuit", "hyu_pure_pursuit.yaml", "Pure Pursuit controller parameter file."),
)


def _params_file(package: str, filename: str) -> PathJoinSubstitution:
    return PathJoinSubstitution([FindPackageShare(package), "config", filename])


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]
    arguments.extend(
        [
            DeclareLaunchArgument(
                "planner_source",
                default_value="slam",
                choices=["slam", "csv"],
                description="Exclusive global waypoint writer.",
            ),
            DeclareLaunchArgument(
                "local_source_mode",
                default_value="slam_map",
                choices=["live_cones", "slam_map"],
                description="Local planner input source; slam_map uses the latched map and ego pose.",
            ),
            DeclareLaunchArgument(
                "global_requires_graph_slam_localization",
                default_value=PythonExpression(
                    ["'false' if '", LaunchConfiguration("planner_source"), "' == 'csv' else 'true'"]
                ),
                choices=["true", "false"],
                description="Require Graph SLAM localization before GLOBAL; defaults false only for CSV.",
            ),
        ]
    )
    arguments.extend(
        DeclareLaunchArgument(
            name,
            default_value=_params_file(package, filename),
            description=description,
        )
        for name, package, filename, description in PARAMETER_FILES
    )
    configuration_names = [name for name, _, _ in ARGUMENTS]
    configuration_names.extend(
        ("planner_source", "local_source_mode", "global_requires_graph_slam_localization")
    )
    configuration_names.extend(name for name, _, _, _ in PARAMETER_FILES)
    values = {name: LaunchConfiguration(name) for name in configuration_names}

    # Local-only missions (skidpad, acceleration) keep the global planner and
    # its Frenet/handoff group down; the state machine stays in LOCAL for the
    # whole run. Resolves to the literal 'true'/'false' for launch conditions.
    local_only = PythonExpression(
        [
            "'true' if '",
            values["skidpad"],
            "' == 'true' or '",
            values["acceleration"],
            "' == 'true' else 'false'",
        ]
    )

    graph_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("hyu_localization"), "launch", "graph_slam.launch.py"]
            )
        ),
        condition=IfCondition(values["start_graph_slam"]),
        launch_arguments={
            "params_file": values["graph_slam_params_file"],
            "use_sim_time": values["use_sim_time"],
            "car_state_topic": values["car_state_topic"],
            "map_topic": values["cone_map_topic"],
            "slam_odom_topic": values["ego_odom_topic"],
            "status_topic": values["graph_slam_status_topic"],
            "map_converged_topic": values["graph_slam_map_converged_topic"],
            "publish_tf": values["graph_slam_publish_tf"],
            "localization_mode": values["graph_slam_localization_mode"],
            "auto_localization_after_lap": PythonExpression(
                ["'false' if '", local_only, "' == 'true' else 'true'"]
            ),
            "load_map_path": values["graph_slam_load_map_path"],
            "gui": values["graph_slam_gui"],
            "ate_monitor": values["graph_slam_ate_monitor"],
            "ate_status_topic": values["graph_slam_status_topic"],
            # CTE monitor inputs must follow this launch's topic overrides.
            "frenet_odom_topic": values["frenet_odom_topic"],
            "global_path_valid_topic": values["global_path_valid_topic"],
        }.items(),
    )
    hyu_global_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("hyu_global_planner"), "launch", "slam_hyu_global_planner.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": values["global_params_file"],
            "planner_source": values["planner_source"],
            "cone_map_topic": values["cone_map_topic"],
            "ego_odom_topic": values["ego_odom_topic"],
            "graph_slam_status_topic": values["graph_slam_status_topic"],
            "global_waypoints_topic": values["global_waypoints_topic"],
            "global_path_valid_topic": values["global_path_valid_topic"],
            "global_path_reason_topic": values["global_path_reason_topic"],
            "frenet_odom_topic": values["frenet_odom_topic"],
            "path_source_topic": values["path_source_topic"],
            "lap_count_topic": values["lap_count_topic"],
            "tmpc_performance_trajectory_topic": values[
                "tmpc_performance_trajectory_topic"
            ],
            "tmpc_emergency_trajectory_topic": values[
                "tmpc_emergency_trajectory_topic"
            ],
            "path_waypoints_topic": values["global_path_waypoints_topic"],
            "path_topic": values["global_path_topic"],
            "use_sim_time": values["use_sim_time"],
        }.items(),
    )
    hyu_global_planner = GroupAction(
        actions=[
            SetRemap(
                src="/localization/map_converged",
                dst=values["graph_slam_map_converged_topic"],
            ),
            hyu_global_planner_launch,
        ],
        # Local-only missions never hand off to a global path; the whole global
        # group (planner, frenet odometry, waypoint window) stays down.
        condition=UnlessCondition(local_only),
    )
    skidpad_director = Node(
        package="hyu_planning_bringup",
        executable="skidpad_director.py",
        name="skidpad_director",
        output="screen",
        condition=IfCondition(values["skidpad"]),
        parameters=[
            {
                "use_sim_time": values["use_sim_time"],
                "cone_map_topic": values["cone_map_topic"],
                "ego_odom_topic": values["ego_odom_topic"],
                "output_cone_map_topic": values["skidpad_cone_map_topic"],
                "phase_topic": values["skidpad_phase_topic"],
                "right_laps": values["skidpad_right_laps"],
                "left_laps": values["skidpad_left_laps"],
            },
        ],
    )
    # Each local-only mission swaps in its own tuned parameter files (skidpad:
    # slow, short lookahead; acceleration: full-speed straight) without touching
    # the trackdrive defaults.
    local_params_selected = PythonExpression(
        [
            "'",
            _params_file("hyu_local_planner", "hyu_local_planner_skidpad.yaml"),
            "' if '",
            values["skidpad"],
            "' == 'true' else ('",
            _params_file("hyu_local_planner", "hyu_local_planner_acceleration.yaml"),
            "' if '",
            values["acceleration"],
            "' == 'true' else '",
            values["local_params_file"],
            "')",
        ]
    )
    controller_params_selected = PythonExpression(
        [
            "'",
            _params_file("hyu_pure_pursuit", "hyu_pure_pursuit_skidpad.yaml"),
            "' if '",
            values["skidpad"],
            "' == 'true' else ('",
            _params_file("hyu_pure_pursuit", "hyu_pure_pursuit_acceleration.yaml"),
            "' if '",
            values["acceleration"],
            "' == 'true' else '",
            values["controller_params_file"],
            "')",
        ]
    )
    hyu_local_planner = Node(
        package="hyu_local_planner",
        executable="hyu_local_planner_node",
        name="hyu_local_planner_node",
        output="screen",
        parameters=[
            local_params_selected,
            {
                "source_mode": values["local_source_mode"],
                "max_stamp_skew_sec": values["local_max_stamp_skew_sec"],
                "max_input_age_sec": values["local_max_input_age_sec"],
                "max_start_distance_m": values["local_max_start_distance_m"],
                "cones_topic": values["cones_topic"],
                # Skidpad feeds the planner the director's phase-gated map.
                "slam_map_topic": PythonExpression(
                    [
                        "'",
                        values["skidpad_cone_map_topic"],
                        "' if '",
                        values["skidpad"],
                        "' == 'true' else '",
                        values["cone_map_topic"],
                        "'",
                    ]
                ),
                "odom_topic": values["ego_odom_topic"],
                "slam_status_topic": values["graph_slam_status_topic"],
                "waypoints_topic": values["local_waypoints_topic"],
                "path_topic": values["local_path_topic"],
                "validity_topic": values["local_path_valid_topic"],
                "reason_topic": values["local_path_reason_topic"],
                "use_sim_time": values["use_sim_time"],
            },
        ],
    )
    hyu_state_machine = Node(
        package="hyu_state_machine",
        executable="planning_hyu_state_machine_node",
        name="planning_hyu_state_machine_node",
        output="screen",
        parameters=[
            values["state_params_file"],
            {
                "frenet_odom_topic": values["frenet_odom_topic"],
                "global_waypoints_topic": values["global_waypoints_topic"],
                "graph_slam_status_topic": values["graph_slam_status_topic"],
                "global_path_valid_topic": values["global_path_valid_topic"],
                "local_path_valid_topic": values["local_path_valid_topic"],
                "global_handoff_ready_topic": values["global_handoff_ready_topic"],
                "cone_map_topic": values["cones_topic"],
                "slam_cone_map_topic": values["cone_map_topic"],
                "ego_odom_topic": values["ego_odom_topic"],
                "stop_zone_s_start_topic": values["stop_zone_s_start_topic"],
                "stop_zone_s_end_topic": values["stop_zone_s_end_topic"],
                "stop_zone_valid_topic": values["stop_zone_valid_topic"],
                "global_requires_graph_slam_localization": values[
                    "global_requires_graph_slam_localization"
                ],
                "use_sim_time": values["use_sim_time"],
            },
        ],
        remappings=[
            ("/planning/state", values["state_topic"]),
            ("/planning/path_source", values["path_source_topic"]),
            ("/planning/lap_count", values["lap_count_topic"]),
            ("/planning/lap_time_last", values["lap_time_last_topic"]),
            ("/planning/lap_time_best", values["lap_time_best_topic"]),
            ("/planning/stop_request", values["stop_request_topic"]),
            ("/planning/debug", values["planning_debug_topic"]),
        ],
    )
    selector = Node(
        package="hyu_path_selector",
        executable="hyu_path_selector_node",
        name="hyu_path_selector_node",
        output="screen",
        parameters=[
            values["selector_params_file"],
            {
                "path_source_topic": values["path_source_topic"],
                "local_path_topic": values["local_waypoints_topic"],
                "local_validity_topic": values["local_path_valid_topic"],
                "global_path_topic": values["global_path_waypoints_topic"],
                "global_validity_topic": values["global_path_valid_topic"],
                "odometry_topic": values["ego_odom_topic"],
                "selected_path_topic": values["selected_path_topic"],
                "selected_path_viz_topic": values["selected_path_viz_topic"],
                "selected_validity_topic": values["selected_path_valid_topic"],
                "handoff_ready_topic": values["global_handoff_ready_topic"],
                "debug_topic": values["selector_debug_topic"],
                "use_sim_time": values["use_sim_time"],
            },
        ],
    )
    controller_remappings = [
        ("/planning/path", values["selected_path_topic"]),
        ("/planning/selected_path_valid", values["selected_path_valid_topic"]),
        ("/planning/stop_request", values["stop_request_topic"]),
        ("/localization/ego_odom", values["ego_odom_topic"]),
        ("/vehicle/cmd", values["controller_cmd_topic"]),
    ]
    controller = Node(
        package="hyu_pure_pursuit",
        executable="hyu_pure_pursuit_node",
        name="hyu_pure_pursuit_node",
        output="screen",
        condition=IfCondition(values["enable_controller"]),
        parameters=[
            controller_params_selected,
            {
                "use_sim_time": values["use_sim_time"],
                "max_speed_mps": ParameterValue(
                    values["controller_max_speed_mps"], value_type=float),
            },
        ],
        remappings=controller_remappings,
    )
    stack_hud = Node(
        package="hyu_planning_bringup",
        executable="stack_hud.py",
        name="stack_hud",
        output="screen",
        condition=IfCondition(values["enable_hud"]),
        parameters=[
            {
                "hud_topic": values["hud_topic"],
                "banner_topic": values["hud_banner_topic"],
                "cones_topic": values["cones_topic"],
                "cone_map_topic": values["cone_map_topic"],
                "slam_status_topic": values["graph_slam_status_topic"],
                "ego_odom_topic": values["ego_odom_topic"],
                "planning_debug_topic": values["planning_debug_topic"],
                "selector_debug_topic": values["selector_debug_topic"],
                "local_path_valid_topic": values["local_path_valid_topic"],
                "local_path_reason_topic": values["local_path_reason_topic"],
                "global_path_valid_topic": values["global_path_valid_topic"],
                "global_path_reason_topic": values["global_path_reason_topic"],
                "selected_path_valid_topic": values["selected_path_valid_topic"],
                "cmd_topic": values["cmd_topic"],
                # Deliberately NOT use_sim_time: the HUD renders on wall time
                # (all freshness is monotonic wall-clock) so it keeps drawing
                # "SILENT" diagnoses even if /clock stalls.
            },
        ],
    )

    return LaunchDescription([
        *arguments,
        graph_slam,
        hyu_global_planner,
        skidpad_director,
        hyu_local_planner,
        hyu_state_machine,
        selector,
        controller,
        stack_hud,
    ])
