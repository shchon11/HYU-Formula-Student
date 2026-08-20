#!/usr/bin/env python3
"""Publish a stopped car's INS solution, for bench and bag-replay testing.

Companion to stationary_wheels.py for when no SBG Ellipse-D is on the desk --
or, just as often, when a bag HAS SBG data that is unusable. The 0726 cone
bag is exactly that case: 3643 ekf_nav messages, every one of them
solution_mode 1 (VERTICAL_GYRO), i.e. the unit never got an absolute fix. The
odometry bridge is right to refuse those ("waiting for absolute fix"), so it
never publishes /localization/ins_odom, GraphSLAM never gets a motion input,
and no map is ever built. Replaying that data cannot be made to work; the
data simply is not there.

So synthesise the solution instead: a stationary NAV_POSITION fix at a fixed
anchor, zero velocity, zero rates, constant heading. That is a truthful model
of a car standing still with a good fix, which is what these bags are.

    ros2 run hyu_sensor_bringup stationary_ins.py
    ros2 run hyu_sensor_bringup stationary_ins.py --ros-args \
        -p latitude:=37.5563 -p longitude:=127.0448 -p heading_deg:=0.0

Publishes the RAW receiver/IMU topics sbg_raw_ekf fuses (/sbg/imu_data at
rate_hz, /sbg/gps_pos, /sbg/gps_vel, /sbg/gps_hdt at rate_hz/5) plus
/sbg/ekf_nav, /sbg/ekf_euler and /sbg/ekf_rot_accel_body for wheel_odometry
and the wheel synthesiser. Never run this alongside a real SBG: two publishers
on those topics interleave two different truths.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sbg_driver.msg import (SbgEkfEuler, SbgEkfNav, SbgEkfRotAccel, SbgEkfStatus,
                            SbgGpsHdt, SbgGpsPos, SbgGpsVel, SbgImuData)

NAV_POSITION = 4  # absolute, GNSS-anchored
GPS_SOL_COMPUTED = 0
GPS_TYPE_RTK_INT = 7
HDT_BASELINE_VALID = 0x40


class StationaryIns(Node):
    def __init__(self):
        super().__init__("stationary_ins")
        self.declare_parameter("latitude", 37.5563)     # HYU Seoul, arbitrary
        self.declare_parameter("longitude", 127.0448)   # but self-consistent
        self.declare_parameter("altitude", 50.0)
        self.declare_parameter("heading_deg", 0.0)
        self.declare_parameter("rate_hz", 25.0)
        self.declare_parameter("frame_id", "base_footprint")
        # Dual-antenna heading -> vehicle heading offset the consumer applies
        # (sbg_raw_ekf hdt_offset_deg, 180 with the rear-secondary mount); the
        # synthetic gps_hdt is emitted so that offset recovers heading_deg.
        self.declare_parameter("hdt_offset_deg", 180.0)

        self.lat = float(self.get_parameter("latitude").value)
        self.lon = float(self.get_parameter("longitude").value)
        self.alt = float(self.get_parameter("altitude").value)
        self.yaw = math.radians(float(self.get_parameter("heading_deg").value))
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.hdt_offset_deg = float(self.get_parameter("hdt_offset_deg").value)
        rate = float(self.get_parameter("rate_hz").value)
        self._tick = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.nav_pub = self.create_publisher(SbgEkfNav, "/sbg/ekf_nav", qos)
        self.euler_pub = self.create_publisher(SbgEkfEuler, "/sbg/ekf_euler", qos)
        self.rot_pub = self.create_publisher(
            SbgEkfRotAccel, "/sbg/ekf_rot_accel_body", qos)
        self.imu_pub = self.create_publisher(SbgImuData, "/sbg/imu_data", qos)
        self.gps_pos_pub = self.create_publisher(SbgGpsPos, "/sbg/gps_pos", qos)
        self.gps_vel_pub = self.create_publisher(SbgGpsVel, "/sbg/gps_vel", qos)
        self.gps_hdt_pub = self.create_publisher(SbgGpsHdt, "/sbg/gps_hdt", qos)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().warn(
            f"publishing a SYNTHETIC stationary NAV_POSITION fix at "
            f"({self.lat:.6f}, {self.lon:.6f}) heading "
            f"{math.degrees(self.yaw):.1f} deg -- bench/replay only, never "
            f"alongside a real SBG")

    def _status(self):
        s = SbgEkfStatus()
        s.solution_mode = NAV_POSITION
        # wheel_odometry / sbg_wheels read the mode and the valid flags. A
        # stopped car with a fix has all of these.
        s.attitude_valid = True
        s.heading_valid = True
        s.velocity_valid = True
        s.position_valid = True
        s.gps1_pos_used = True
        s.gps1_vel_used = True
        s.gps1_hdt_used = True
        # Standing still IS zero-velocity-update territory; say so rather than
        # pretending the solution is being carried by wheel or air data.
        s.zupt_used = True
        s.align_valid = True
        return s

    def tick(self):
        now = self.get_clock().now().to_msg()
        status = self._status()

        nav = SbgEkfNav()
        nav.header.stamp = now
        nav.header.frame_id = self.frame_id
        nav.latitude, nav.longitude, nav.altitude = self.lat, self.lon, self.alt
        nav.velocity.x = nav.velocity.y = nav.velocity.z = 0.0
        # Accuracies are what downstream turns into covariances. Quote a good
        # RTK-grade fix rather than zeros: a zero sigma is not "perfect", it is
        # a degenerate covariance that can make an estimator blow up.
        nav.velocity_accuracy.x = nav.velocity_accuracy.y = 0.02
        nav.velocity_accuracy.z = 0.03
        nav.position_accuracy.x = nav.position_accuracy.y = 0.02
        nav.position_accuracy.z = 0.03
        nav.status = status
        self.nav_pub.publish(nav)

        euler = SbgEkfEuler()
        euler.header.stamp = now
        euler.header.frame_id = self.frame_id
        euler.angle.x = euler.angle.y = 0.0
        euler.angle.z = self.yaw
        euler.accuracy.x = euler.accuracy.y = math.radians(0.1)
        euler.accuracy.z = math.radians(0.2)
        euler.status = status
        self.euler_pub.publish(euler)

        rot = SbgEkfRotAccel()
        rot.header.stamp = now
        rot.header.frame_id = self.frame_id
        rot.rate.x = rot.rate.y = rot.rate.z = 0.0
        # Body-frame specific force of a level, stationary vehicle is gravity
        # on +Z; zeros would read as freefall.
        rot.acceleration.x = rot.acceleration.y = 0.0
        rot.acceleration.z = 9.80665
        self.rot_pub.publish(rot)

        # --- raw receiver/IMU topics (what sbg_raw_ekf fuses) ---------------
        # Device clock: uint32 microseconds, shared by all four messages.
        ts = (self.get_clock().now().nanoseconds // 1000) & 0xFFFFFFFF
        imu = SbgImuData()
        imu.header.stamp = now
        imu.header.frame_id = "imu_link_ned"
        imu.time_stamp = ts
        # NED body (x fwd, y right, z down): the specific force of a level,
        # stationary unit is -g on z (the real unit reads ~-9.74 with its tilt).
        imu.accel.x = imu.accel.y = 0.0
        imu.accel.z = -9.80665
        imu.gyro.x = imu.gyro.y = imu.gyro.z = 0.0
        imu.temp = 30.0
        self.imu_pub.publish(imu)

        self._tick += 1
        if self._tick % 5 != 0:
            return  # receiver epochs at rate_hz / 5 (5 Hz for the default)
        pos = SbgGpsPos()
        pos.header.stamp = now
        pos.header.frame_id = self.frame_id
        pos.time_stamp = ts
        pos.status.status = GPS_SOL_COMPUTED
        pos.status.type = GPS_TYPE_RTK_INT
        pos.status.ifm = 2       # CLEAN
        pos.status.spoofing = 2  # CLEAN
        pos.latitude, pos.longitude, pos.altitude = self.lat, self.lon, self.alt
        pos.position_accuracy.x = pos.position_accuracy.y = 0.02
        pos.position_accuracy.z = 0.03
        pos.num_sv_used, pos.num_sv_tracked = 20, 24
        pos.base_station_id = 0xFFFF
        pos.diff_age = 100
        self.gps_pos_pub.publish(pos)

        vel = SbgGpsVel()
        vel.header.stamp = now
        vel.header.frame_id = self.frame_id
        vel.time_stamp = ts
        vel.status.vel_status = GPS_SOL_COMPUTED
        vel.status.vel_type = 2  # DOPPLER
        vel.velocity.x = vel.velocity.y = vel.velocity.z = 0.0
        vel.velocity_accuracy.x = vel.velocity_accuracy.y = 0.02
        vel.velocity_accuracy.z = 0.03
        vel.course = math.degrees(self.yaw) % 360.0
        vel.course_acc = 10.0  # undefined at rest; only used above 1 m/s
        self.gps_vel_pub.publish(vel)

        hdt = SbgGpsHdt()
        hdt.header.stamp = now
        hdt.header.frame_id = self.frame_id
        hdt.time_stamp = ts
        hdt.status = HDT_BASELINE_VALID  # low 6 bits SOL_COMPUTED
        hdt.true_heading = (math.degrees(self.yaw) - self.hdt_offset_deg) % 360.0
        hdt.true_heading_acc = 0.2
        hdt.pitch = 0.0
        hdt.pitch_acc = 0.5
        hdt.baseline = 1.2567
        hdt.num_sv_tracked, hdt.num_sv_used = 24, 20
        self.gps_hdt_pub.publish(hdt)


def main():
    rclpy.init()
    node = StationaryIns()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
