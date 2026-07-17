from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    default_params = PathJoinSubstitution(
        [FindPackageShare("hyu_pure_pursuit"), "config", "hyu_pure_pursuit.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="hyu_pure_pursuit",
                executable="hyu_pure_pursuit_node",
                name="hyu_pure_pursuit_node",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
