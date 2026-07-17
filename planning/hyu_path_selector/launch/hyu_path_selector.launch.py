from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("hyu_path_selector"), "config", "hyu_path_selector.yaml"]
    )
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path selector parameter file.",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the ROS simulation clock.",
    )

    return LaunchDescription(
        [
            params_file,
            use_sim_time,
            Node(
                package="hyu_path_selector",
                executable="hyu_path_selector_node",
                name="hyu_path_selector_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
