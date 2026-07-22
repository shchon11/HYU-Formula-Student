"""KASE 제22조 inspection mission — the 'planning graph' of the inspection.

No planner, no SLAM: just the inspection command profile plus the DSSI state
mapper, which must keep indicating through the inspection (yellow flashing →
RTAD → blue → off) exactly as in a driving mission. mission.sh swaps the
planning pane to this launch for 'mission inspection'.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use the simulator clock."),
        Node(
            package="hyu_planning_bringup",
            executable="inspection_mission.py",
            name="inspection_mission",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="hyu_planning_bringup",
            executable="dssi_state.py",
            name="dssi_state",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
