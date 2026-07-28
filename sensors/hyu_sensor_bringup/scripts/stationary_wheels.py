#!/usr/bin/env python3
"""Publish a stopped car's wheel encoders, for bench and bag-replay testing.

The car has no wheel-speed sensor fitted yet, and hyu_localization's
wheel_odometry subscribes /vehicle/wheel_speeds unconditionally. With nothing
on that topic it never publishes /localization/wheel_odom, so the INS bridge
loses its dead-reckoning fallback tier and anything reading wheel odometry
(perception deskew, the TMPC state bridge) waits forever on a topic that is
advertised and silent -- the hardest failure to spot, because every node looks
healthy.

This publishes zeros with a live timestamp, which is the truthful reading for
a car that is standing still. That matters: wheel_odometry integrates dt from
consecutive header stamps, so a constant stamp (what `ros2 topic pub` gives
you without a stamp field) yields dt = 0 forever, and it is NOT the same thing
as a stopped car.

    ros2 run hyu_sensor_bringup stationary_wheels.py
    ros2 run hyu_sensor_bringup stationary_wheels.py --ros-args -p rate_hz:=100.0

Do not leave this running on a moving car: it would assert zero speed against
real motion and quietly poison dead reckoning.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from hyu_msgs.msg import WheelSpeedsStamped


class StationaryWheels(Node):
    def __init__(self):
        super().__init__("stationary_wheels")
        self.declare_parameter("topic", "/vehicle/wheel_speeds")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("frame_id", "base_footprint")

        topic = str(self.get_parameter("topic").value)
        rate = float(self.get_parameter("rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.pub = self.create_publisher(
            WheelSpeedsStamped, topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().warn(
            f"publishing STOPPED wheel speeds on {topic} at {rate:g} Hz -- "
            f"bench/replay only, never on a moving car")

    def tick(self):
        msg = WheelSpeedsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # Steering is radians, wheel speeds are RPM (see WheelSpeeds.msg).
        msg.speeds.steering = 0.0
        msg.speeds.lf_speed = 0.0
        msg.speeds.rf_speed = 0.0
        msg.speeds.lb_speed = 0.0
        msg.speeds.rb_speed = 0.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = StationaryWheels()
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
