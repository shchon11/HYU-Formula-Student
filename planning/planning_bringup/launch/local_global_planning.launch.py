from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = (
    ("use_sim_time", "true", "Use the simulator clock for every planning node."),
    ("start_graph_slam", "true", "Start graph SLAM with the planning graph."),
    ("graph_slam_gui", "false", "Start the graph SLAM GUI."),
    ("graph_slam_ate_monitor", "true", "Start the path-tracking CTE monitor (HUD + /planning/cte)."),
    ("graph_slam_localization_mode", "false", "Localize against a saved graph SLAM map."),
    ("graph_slam_load_map_path", "", "Saved graph SLAM map for localization mode."),
    ("graph_slam_publish_tf", "true", "Allow graph SLAM to publish the map TF."),
    ("car_state_topic", "/odometry_integration/car_state", "Graph SLAM motion input."),
    ("cones_topic", "/cones", "Live cone observations for local planning and state."),
    ("cone_map_topic", "/localization/cone_map", "Graph SLAM cone-map output."),
    ("ego_odom_topic", "/localization/ego_odom", "Planner-facing odometry."),
    ("graph_slam_status_topic", "/graph_slam/status", "Graph SLAM lifecycle status."),
    ("graph_slam_map_converged_topic", "/graph_slam/map_converged", "Graph SLAM map-ready status."),
    ("global_waypoints_topic", "/global_waypoints", "Latched full global waypoint path."),
    ("global_path_valid_topic", "/planning/global_path_valid", "Global-path validity heartbeat."),
    ("global_path_waypoints_topic", "/planning/global_path_waypoints", "Global rolling waypoint window."),
    ("global_path_topic", "/planning/global_path_waypoints/path", "Global rolling window visualization."),
    ("frenet_odom_topic", "/car_state/frenet/odom", "Frenet odometry topic."),
    ("local_waypoints_topic", "/planning/local_waypoints", "Local planner waypoint output."),
    ("local_path_topic", "/planning/local_waypoints/path", "Local planner path visualization."),
    ("local_path_valid_topic", "/planning/local_path_valid", "Local-path validity heartbeat."),
    ("state_topic", "/planning/state", "Planning state output."),
    ("path_source_topic", "/planning/path_source", "State-selected path source."),
    ("lap_count_topic", "/planning/lap_count", "Planning lap-count output."),
    ("stop_request_topic", "/planning/stop_request", "State-machine stop request."),
    ("planning_debug_topic", "/planning/debug", "State-machine debug output."),
    ("selected_path_topic", "/path_waypoints", "Selector-owned controller path."),
    ("selected_path_viz_topic", "/path_waypoints/path", "Selector path visualization."),
    ("selected_path_valid_topic", "/planning/selected_path_valid", "Selector validity heartbeat."),
    ("global_handoff_ready_topic", "/planning/global_handoff_ready", "Selector handoff readiness."),
    ("selector_debug_topic", "/planning/path_selector/debug", "Selector debug output."),
    ("stop_zone_s_start_topic", "/stop_zone_s_start", "Final stop-zone start position."),
    ("stop_zone_s_end_topic", "/stop_zone_s_end", "Final stop-zone end position."),
    ("stop_zone_valid_topic", "/stop_zone_valid", "Final stop-zone validity."),
    ("enable_controller", "true", "Start the sole /cmd writer."),
    ("cmd_topic", "/cmd", "Controller command output."),
)

PARAMETER_FILES = (
    ("graph_slam_params_file", "eufs_graph_slam", "graph_slam.yaml", "Graph SLAM parameter file."),
    ("global_params_file", "global_planner", "global_planner.yaml", "Global planner and Frenet parameter file."),
    ("local_params_file", "local_planner", "local_planner.yaml", "Local planner parameter file."),
    ("state_params_file", "state_machine", "planning_state_machine.yaml", "Planning state-machine parameter file."),
    ("selector_params_file", "path_selector", "path_selector.yaml", "Path selector parameter file."),
    ("controller_params_file", "pure_pursuit_controller", "pure_pursuit_controller.yaml", "Pure Pursuit controller parameter file."),
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
                default_value="live_cones",
                choices=["live_cones", "slam_map"],
                description="Local planner input source.",
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
    configuration_names.extend(("planner_source", "local_source_mode"))
    configuration_names.extend(name for name, _, _, _ in PARAMETER_FILES)
    values = {name: LaunchConfiguration(name) for name in configuration_names}

    graph_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("eufs_graph_slam"), "launch", "graph_slam.launch.py"]
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
            "load_map_path": values["graph_slam_load_map_path"],
            "gui": values["graph_slam_gui"],
            "ate_monitor": values["graph_slam_ate_monitor"],
            "ate_status_topic": values["graph_slam_status_topic"],
        }.items(),
    )
    global_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("global_planner"), "launch", "slam_global_planner.launch.py"]
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
            "path_waypoints_topic": values["global_path_waypoints_topic"],
            "path_topic": values["global_path_topic"],
            "use_sim_time": values["use_sim_time"],
        }.items(),
    )
    global_planner = GroupAction(
        actions=[
            SetRemap(
                src="/graph_slam/map_converged",
                dst=values["graph_slam_map_converged_topic"],
            ),
            global_planner_launch,
        ]
    )
    local_planner = Node(
        package="local_planner",
        executable="local_planner_node",
        name="local_planner_node",
        output="screen",
        parameters=[
            values["local_params_file"],
            {
                "source_mode": values["local_source_mode"],
                "cones_topic": values["cones_topic"],
                "slam_map_topic": values["cone_map_topic"],
                "odom_topic": values["ego_odom_topic"],
                "waypoints_topic": values["local_waypoints_topic"],
                "path_topic": values["local_path_topic"],
                "validity_topic": values["local_path_valid_topic"],
                "use_sim_time": values["use_sim_time"],
            },
        ],
    )
    state_machine = Node(
        package="state_machine",
        executable="planning_state_machine_node",
        name="planning_state_machine_node",
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
                "stop_zone_s_start_topic": values["stop_zone_s_start_topic"],
                "stop_zone_s_end_topic": values["stop_zone_s_end_topic"],
                "stop_zone_valid_topic": values["stop_zone_valid_topic"],
                "use_sim_time": values["use_sim_time"],
            },
        ],
        remappings=[
            ("/planning/state", values["state_topic"]),
            ("/planning/path_source", values["path_source_topic"]),
            ("/planning/lap_count", values["lap_count_topic"]),
            ("/planning/stop_request", values["stop_request_topic"]),
            ("/planning/debug", values["planning_debug_topic"]),
        ],
    )
    selector = Node(
        package="path_selector",
        executable="path_selector_node",
        name="path_selector_node",
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
    controller = Node(
        package="pure_pursuit_controller",
        executable="pure_pursuit_controller_node",
        name="pure_pursuit_controller_node",
        output="screen",
        condition=IfCondition(values["enable_controller"]),
        parameters=[
            values["controller_params_file"],
            {"use_sim_time": values["use_sim_time"]},
        ],
        remappings=[
            ("/path_waypoints", values["selected_path_topic"]),
            ("/planning/selected_path_valid", values["selected_path_valid_topic"]),
            ("/planning/stop_request", values["stop_request_topic"]),
            ("/localization/ego_odom", values["ego_odom_topic"]),
            ("/cmd", values["cmd_topic"]),
        ],
    )

    return LaunchDescription([
        *arguments,
        graph_slam,
        global_planner,
        local_planner,
        state_machine,
        selector,
        controller,
    ])
