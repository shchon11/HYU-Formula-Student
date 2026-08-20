#!/usr/bin/env python3
"""Synthesise wheel encoders from the INS, for bag replay of a MOVING car.

The car has no wheel-speed sensor fitted, so on a bag replay the choice is
between stationary_wheels.py (zeros) and this. Zeros are the truthful reading
for a parked car and a lie for a driven one: perception's LiDAR deskew reads
/localization/wheel_odom, and telling it the car stood still through a corner
smears the cloud exactly where the cones are.

This reads the replayed INS instead and back-computes what the encoders would
have read:

    v_fwd   = v_ned . heading            signed, so reversing stays signed
    v_left  = v_fwd - track/2 * yaw_rate
    v_right = v_fwd + track/2 * yaw_rate
    rpm     = v / (wheel_radius * 2*pi/60)

    ros2 run hyu_sensor_bringup sbg_wheels.py --ros-args -p use_sim_time:=true

CAVEAT, and it is not a small one: wheel_odometry fuses these speeds with the
same INS they came from, so /localization/wheel_odom stops being an independent
sensor and becomes a restatement of the INS. That is fine for deskew, which
only needs to know how far the car moved during a sweep. It is NOT fine for
judging the dead-reckoning fallback tier -- with this running, a replay cannot
show what happens when the INS drops out, because the fallback drops out with
it. Use stationary_wheels.py on a parked bag, and never run either on a car.

Below min_solution_mode the INS velocity is not trustworthy, so zeros go out
instead: that is the start-of-bag case where the car sits waiting for a fix,
and a silent topic would stall wheel_odometry's dt integration entirely.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from hyu_msgs.msg import WheelSpeedsStamped
from sbg_driver.msg import SbgEkfEuler, SbgEkfNav, SbgEkfRotAccel

RPM_TO_RAD_S = 2.0 * math.pi / 60.0


class SbgWheels(Node):
    def __init__(self):
        super().__init__("sbg_wheels")
        self.declare_parameter("topic", "/vehicle/wheel_speeds")
        self.declare_parameter("ekf_nav_topic", "/sbg/ekf_nav")
        self.declare_parameter("ekf_euler_topic", "/sbg/ekf_euler")
        self.declare_parameter("rot_accel_topic", "/sbg/ekf_rot_accel_body")
        self.declare_parameter("frame_id", "base_footprint")
        # Defaults match hyu_localization's wheel_odometry, so a round trip
        # through both returns the speed this node started from.
        self.declare_parameter("wheel_radius", 0.2525)   # m
        self.declare_parameter("track_width", 1.4)       # m
        self.declare_parameter("wheelbase", 1.58)        # m
        self.declare_parameter("min_solution_mode", 3)   # NAV_VELOCITY and up
        self.declare_parameter("steering_min_speed", 0.5)  # m/s

        self.radius = float(self.get_parameter("wheel_radius").value)
        self.track = float(self.get_parameter("track_width").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.min_mode = int(self.get_parameter("min_solution_mode").value)
        self.steer_min_v = float(self.get_parameter("steering_min_speed").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.yaw = None
        self.yaw_rate = 0.0
        self.published = 0
        self.degraded = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            WheelSpeedsStamped, str(self.get_parameter("topic").value), qos)
        self.create_subscription(
            SbgEkfEuler, str(self.get_parameter("ekf_euler_topic").value),
            self.on_euler, qos)
        self.create_subscription(
            SbgEkfRotAccel, str(self.get_parameter("rot_accel_topic").value),
            self.on_rot_accel, qos)
        self.create_subscription(
            SbgEkfNav, str(self.get_parameter("ekf_nav_topic").value),
            self.on_nav, qos)
        self.create_timer(5.0, self.report)
        self.get_logger().warn(
            "synthesising wheel speeds from the INS -- bag replay only. "
            "/localization/wheel_odom is NOT independent of the INS while this runs")

    def on_euler(self, msg):
        self.yaw = msg.angle.z

    def on_rot_accel(self, msg):
        self.yaw_rate = msg.rate.z

    def on_nav(self, msg):
        if msg.status.solution_mode >= self.min_mode:
            vn, ve = msg.velocity.x, msg.velocity.y
            if self.yaw is None:
                # No heading yet: unsigned ground speed is the best available,
                # and a car waiting for its first fix is not reversing.
                v_fwd = math.hypot(vn, ve)
            else:
                v_fwd = vn * math.cos(self.yaw) + ve * math.sin(self.yaw)
            w = self.yaw_rate
        else:
            self.degraded += 1
            v_fwd = w = 0.0

        v_left = v_fwd - 0.5 * self.track * w
        v_right = v_fwd + 0.5 * self.track * w
        to_rpm = 1.0 / (self.radius * RPM_TO_RAD_S)

        out = WheelSpeedsStamped()
        # The INS stamp, not the wall clock: wheel_odometry integrates dt from
        # consecutive stamps, and on a replay these are the only stamps that
        # line up with the LiDAR sweeps the deskew is correcting.
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        # Bicycle model, w = v*tan(d)/L -> d = atan(L*w/v). atan of the ratio,
        # NOT atan2 of the pair: these bags contain real reversing (17_31_57 is
        # about half of it), and atan2 answers ~+-pi for a car backing up
        # straight, which is not a steering angle any node can use.
        out.speeds.steering = (
            math.atan(self.wheelbase * w / v_fwd)
            if abs(v_fwd) > self.steer_min_v else 0.0)
        out.speeds.lf_speed = out.speeds.lb_speed = v_left * to_rpm
        out.speeds.rf_speed = out.speeds.rb_speed = v_right * to_rpm
        self.pub.publish(out)
        self.published += 1

    def report(self):
        if self.degraded:
            self.get_logger().warn(
                f"{self.published} wheel speed msgs, {self.degraded} of them zeroed "
                f"(INS below solution_mode {self.min_mode})")
            self.degraded = 0


def main():
    rclpy.init()
    node = SbgWheels()
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
