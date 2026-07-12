# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Simulated Ellipse-D INS pipeline: sim INS -> SBG bridge -> graph SLAM.

One-command harness for running the real-hardware GNSS/INS pipeline against
the EUFS simulator. The bridge output is remapped to /ins_odom/car_state
because the simulator's race-car plugin already publishes drift odometry on
the default /odometry_integration/car_state — without the remap the two
sources collide and SLAM receives interleaved poses from different frames.

    ros2 launch eufs_graph_slam ins_pipeline.launch.py \
        mode_schedule:="30:3,45:4" correction_schedule:="60:single,70:rtk_fixed"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    car_state_topic = LaunchConfiguration("car_state_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the simulator clock.",
            ),
            DeclareLaunchArgument(
                "car_state_topic",
                default_value="/ins_odom/car_state",
                description=(
                    "INS odometry topic for bridge output and SLAM input. Kept "
                    "off /odometry_integration/car_state, which the simulator "
                    "race-car plugin already publishes."
                ),
            ),
            DeclareLaunchArgument(
                "mode_schedule",
                default_value="",
                description="solution_mode schedule, e.g. '30:3,45:4'.",
            ),
            DeclareLaunchArgument(
                "correction_schedule",
                default_value="",
                description="RTK correction schedule, e.g. '60:single,70:rtk_fixed'.",
            ),
            DeclareLaunchArgument(
                "odometer_aided",
                default_value="false",
                description="Simulate odometer aiding (0.5%/distance outage drift).",
            ),
            DeclareLaunchArgument(
                "ekf_rate",
                default_value="200.0",
                description=(
                    "Simulated Ellipse-D EKF output rate in Hz (sim time). The "
                    "real unit outputs up to 200 Hz; effective rate is capped "
                    "by the /ground_truth/state rate and, on the wall clock, "
                    "by Gazebo's real-time factor."
                ),
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="true",
                description="Also launch graph SLAM pointed at the INS odometry.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Forwarded to graph_slam.launch.py.",
            ),
            DeclareLaunchArgument(
                "ate_monitor",
                default_value="true",
                description="Forwarded to graph_slam.launch.py.",
            ),
            Node(
                package="eufs_sensors",
                executable="sim_ellipse_d",
                name="sim_ellipse_d",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "mode_schedule": LaunchConfiguration("mode_schedule"),
                        "correction_schedule": LaunchConfiguration(
                            "correction_schedule"
                        ),
                        "odometer_aided": LaunchConfiguration("odometer_aided"),
                        "ekf_rate": LaunchConfiguration("ekf_rate"),
                    }
                ],
            ),
            Node(
                package="eufs_graph_slam",
                executable="sbg_odometry_bridge",
                name="sbg_odometry_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "car_state_topic": car_state_topic,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("eufs_graph_slam"),
                            "launch",
                            "graph_slam.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "car_state_topic": car_state_topic,
                    "gui": LaunchConfiguration("gui"),
                    "ate_monitor": LaunchConfiguration("ate_monitor"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("slam")),
            ),
        ]
    )
