# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Ellipse-D INS pipeline: raw SBG topics -> sbg_raw_ekf -> graph SLAM.

One-command harness for the GNSS/INS pipeline. sbg_raw_ekf (C++) fuses the
receiver's raw outputs (/sbg/imu_data, gps_pos, gps_vel, gps_hdt) -- the
device EKF (ekf_nav / ekf_euler) is not used -- and publishes the INS
odometry on /localization/ins_odom and the raw fix on /localization/gnss_odom.
NOTE: the simulated Ellipse-D (sim_ellipse_d) still emits only ekf_nav /
ekf_euler, so with sim_ins:=true the odometry stays silent until it grows
raw gps_pos/gps_vel/gps_hdt/imu_data outputs. The simulator
race-car plugin's synthetic localisation car state is disabled in the robot
xacro (publishLocalisationCarState=false) — this pipeline plus wheel
odometry is the only state-estimation source, in sim and on the car alike.

    ros2 launch hyu_localization ins_pipeline.launch.py \
        mode_schedule:="30:3,45:4" correction_schedule:="60:single,70:rtk_fixed"
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _sbg_raw_ekf(context):
    """sbg_raw_ekf with only the overrides that were actually given.

    datum/antenna arguments default to "" so that the node keeps its own
    defaults (first-fix datum, 1.25/0.0 antenna) unless a caller states a
    value; a NaN written through a launch parameter would not round-trip.
    """
    lc = context.launch_configurations
    if lc["odometry"].strip().lower() in ("false", "0"):
        return []
    params = {
        "use_sim_time": lc["use_sim_time"].strip().lower() in ("true", "1"),
        "car_state_topic": lc["ins_odom_topic"],
    }
    for name in ("datum_latitude", "datum_longitude", "antenna_offset_x", "antenna_offset_y"):
        text = lc[name].strip()
        if not text:
            continue
        value = float(text)
        if math.isfinite(value):
            params[name] = value
    denied = lc["allow_gnss_denied_init"].strip().lower()
    if denied in ("true", "1"):
        params["allow_gnss_denied_init"] = True
    return [
        Node(
            package="hyu_localization",
            executable="sbg_raw_ekf",
            name="sbg_raw_ekf",
            output="screen",
            parameters=[params],
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    # Deliberately NOT named car_state_topic: launch configurations are global
    # across includes, so a same-named argument here would silently override
    # graph_slam.launch.py's car_state_topic (the SLAM motion input).
    ins_odom_topic = LaunchConfiguration("ins_odom_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the simulator clock.",
            ),
            DeclareLaunchArgument(
                "slam_motion_topic",
                default_value="/localization/ins_odom",
                description="SLAM motion input (sbg_raw_ekf's fused "
                "GNSS/IMU odometry).",
            ),
            DeclareLaunchArgument(
                "ins_odom_topic",
                default_value="/localization/ins_odom",
                description="INS odometry topic published by sbg_raw_ekf.",
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
                "odometry",
                default_value="true",
                description="Start the odometry conditioning node "
                            "(sbg_raw_ekf). Set false when step 1's sensor "
                            "bringup already runs it, which is the vehicle "
                            "and bag-replay case.",
            ),
            # Optional sbg_raw_ekf overrides (the simulator flow passes these so
            # the EKF frame coincides with the simulator's world frame): a
            # fixed datum instead of the first fix, and the primary antenna
            # position in base_footprint. Empty = the node's own defaults.
            DeclareLaunchArgument(
                "datum_latitude",
                default_value="",
                description="sbg_raw_ekf datum_latitude (deg); empty = first valid fix.",
            ),
            DeclareLaunchArgument(
                "allow_gnss_denied_init",
                default_value="false",
                description="Start the EKF at the datum origin (IMU+wheel dead-reckon) "
                            "if no GNSS fix arrives; GNSS corrects on return. Needs a datum.",
            ),
            DeclareLaunchArgument(
                "datum_longitude",
                default_value="",
                description="sbg_raw_ekf datum_longitude (deg); empty = first valid fix.",
            ),
            DeclareLaunchArgument(
                "antenna_offset_x",
                default_value="",
                description="sbg_raw_ekf antenna_offset_x (m, base_footprint); empty = node default.",
            ),
            DeclareLaunchArgument(
                "antenna_offset_y",
                default_value="",
                description="sbg_raw_ekf antenna_offset_y (m, base_footprint); empty = node default.",
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
            # The SIMULATED Ellipse-D. It lives in eufs_sensors, which is part
            # of the Gazebo simulator and is not built on the vehicle compute
            # (no arm64 gazebo exists at all). Unconditional, it took the whole
            # INS pipeline down on the car with "package 'eufs_sensors' not
            # found" -- and with it the INS odometry, so /localization/
            # ins_odom stayed silent and SLAM never got a motion input while
            # the real SBG topics were arriving at 25 Hz. Gate it on the clock:
            # a simulated INS is only ever meaningful against a simulated one.
            DeclareLaunchArgument(
                "sim_ins",
                default_value="false",
                description="Run the simulated Ellipse-D instead of a real "
                            "INS. Off by default: eufs_sensors is part of the "
                            "simulator and is not built on the car, so a wrong "
                            "value here takes the whole INS pipeline down. "
                            "fsk-session.sh sets it from FSK_ENV, not from the "
                            "clock -- bag replay uses sim time with a REAL "
                            "INS chain.",
            ),
            Node(
                package="eufs_sensors",
                executable="sim_ellipse_d",
                name="sim_ellipse_d",
                output="screen",
                condition=IfCondition(LaunchConfiguration("sim_ins")),
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
            # Sensor conditioning, not SLAM: every input is a step-1 driver
            # topic (/sbg/imu_data, gps_pos, gps_vel, gps_hdt). On the car the
            # sensor bringup owns it so it is up in step 1, where perception's
            # deskew can already see /localization/ins_odom; the simulator
            # has no sensor bringup, so there it stays here with sim_ellipse_d.
            OpaqueFunction(function=_sbg_raw_ekf),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("hyu_localization"),
                            "launch",
                            "graph_slam.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    # SLAM motion input: sbg_raw_ekf's raw-GNSS/IMU odometry
                    # (RTK position + dual-antenna yaw, IMU between epochs,
                    # honest per-mode sigmas). The always-DR wheel odometry carried a
                    # constant ~1.1 deg initial-heading error that fought the
                    # mm-grade GNSS anchors inside the graph (2026-07-18
                    # error-budget decomposition) and was retired 2026-08-22;
                    # wheel speeds are fused inside the EKF instead.
                    "car_state_topic": LaunchConfiguration("slam_motion_topic"),
                    "gui": LaunchConfiguration("gui"),
                    "ate_monitor": LaunchConfiguration("ate_monitor"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("slam")),
            ),
        ]
    )
