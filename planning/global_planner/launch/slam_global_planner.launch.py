from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    path_waypoints_topic = LaunchConfiguration(
        "path_waypoints_topic", default="/path_waypoints"
    )
    path_topic = LaunchConfiguration("path_topic", default="/path_waypoints/path")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    default_params = PathJoinSubstitution([
        FindPackageShare("global_planner"),
        "config",
        "global_planner.yaml",
    ])

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Shared parameter YAML for global_planner and frenet nodes.",
    )
    planner_source_arg = DeclareLaunchArgument(
        "planner_source",
        default_value="slam",
        choices=["slam", "csv"],
        description="Global waypoint writer to start: slam planner_node or csv publisher.",
    )
    cone_map_topic_arg = DeclareLaunchArgument(
        "cone_map_topic",
        default_value="/localization/cone_map",
        description="SLAM cone map consumed by planner_node.",
    )
    ego_odom_topic_arg = DeclareLaunchArgument(
        "ego_odom_topic",
        default_value="/localization/ego_odom",
        description="Localization ego odometry consumed by planner_node and frenet_odom_node.",
    )
    graph_slam_status_topic_arg = DeclareLaunchArgument(
        "graph_slam_status_topic",
        default_value="/graph_slam/status",
        description="Graph SLAM status topic consumed by planner_node.",
    )
    global_waypoints_topic_arg = DeclareLaunchArgument(
        "global_waypoints_topic",
        default_value="/global_waypoints",
        description="Global waypoint topic shared by the selected writer and consumers.",
    )
    global_path_valid_topic_arg = DeclareLaunchArgument(
        "global_path_valid_topic",
        default_value="/planning/global_path_valid",
        description="Reliable volatile global path validity heartbeat topic.",
    )
    path_waypoints_topic_arg = DeclareLaunchArgument(
        "path_waypoints_topic",
        default_value="/path_waypoints",
        description=(
            "Rolling global waypoint window published by wpnt_publisher_node; "
            "the integrated launch overrides it to /planning/global_path_waypoints."
        ),
    )
    path_topic_arg = DeclareLaunchArgument(
        "path_topic",
        default_value="/path_waypoints/path",
        description="RViz path matching the rolling global waypoint window.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use ROS simulated time for all nodes started by this launch.",
    )

    slam_writer = Node(
        package="global_planner",
        executable="planner_node",
        name="planner_node",
        output="screen",
        condition=LaunchConfigurationEquals("planner_source", "slam"),
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "cone_map_topic": LaunchConfiguration("cone_map_topic"),
                "ego_odom_topic": LaunchConfiguration("ego_odom_topic"),
                "graph_slam_status_topic": LaunchConfiguration("graph_slam_status_topic"),
                "global_waypoints_topic": LaunchConfiguration("global_waypoints_topic"),
                "global_path_valid_topic": LaunchConfiguration("global_path_valid_topic"),
                "use_sim_time": use_sim_time,
            },
        ],
    )

    csv_writer = Node(
        package="global_planner",
        executable="global_planner_trajectory_publisher_node",
        name="global_planner_trajectory_publisher_node",
        output="screen",
        condition=LaunchConfigurationEquals("planner_source", "csv"),
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "global_waypoints_topic": LaunchConfiguration("global_waypoints_topic"),
                "global_path_valid_topic": LaunchConfiguration("global_path_valid_topic"),
                "use_sim_time": use_sim_time,
            },
        ],
    )

    frenet_odom = Node(
        package="frenet_conversion",
        executable="frenet_odom_node",
        name="frenet_odom_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "odom_topic": LaunchConfiguration("ego_odom_topic"),
                "waypoint_topic": LaunchConfiguration("global_waypoints_topic"),
                "global_path_valid_topic": LaunchConfiguration("global_path_valid_topic"),
                "use_sim_time": use_sim_time,
            },
        ],
    )

    waypoint_publisher = Node(
        package="global_planner",
        executable="wpnt_publisher_node",
        name="wpnt_publisher",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "global_waypoints_topic": LaunchConfiguration("global_waypoints_topic"),
                "global_path_valid_topic": LaunchConfiguration("global_path_valid_topic"),
                "path_waypoints_topic": path_waypoints_topic,
                "path_topic": path_topic,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription([
        params_arg,
        planner_source_arg,
        cone_map_topic_arg,
        ego_odom_topic_arg,
        graph_slam_status_topic_arg,
        global_waypoints_topic_arg,
        global_path_valid_topic_arg,
        path_waypoints_topic_arg,
        path_topic_arg,
        use_sim_time_arg,
        slam_writer,
        csv_writer,
        frenet_odom,
        waypoint_publisher,
    ])
