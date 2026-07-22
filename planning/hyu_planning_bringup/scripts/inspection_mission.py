#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
inspection_mission — KASE 제22조 검차 미션 (autonomous inspection).

The vehicle sits on quick jacks with the wheels off the ground. Once the
vehicle state machine reports AS_DRIVING (mission armed and started), this
node — nothing else, no planner, no SLAM — commands:

  - the drive axle at a slow constant speed, and
  - a steering sine wave, amplitude >= 15 deg at the tire and 0.5-1 Hz,
  - for 25-30 s, then terminates and reports mission_completed
    (-> AS_FINISHED, DSSI off).

DSB check (제22조 ③): during the run an official asks for a brake command.
Trigger it with

    ros2 service call /inspection/brake_test std_srvs/srv/Trigger

The node commands full braking, watches the measured axle/vehicle speed and
logs PASS if it reaches standstill within 3 s (the rule's limit), then
resumes the sine profile for the remainder of the duration.

In the simulator the car is NOT on jacks, so it slowly weaves forward —
that is expected and harmless; the point in sim is validating the command
profile and the state chain end-to-end.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from hyu_msgs.msg import CanState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class InspectionMission(Node):

    def __init__(self):
        super().__init__("inspection_mission")
        p = self.declare_parameter
        cmd_topic = p("cmd_topic", "/vehicle/cmd").value
        as_state_topic = p("as_state_topic", "/vehicle/as_state").value
        completed_topic = p(
            "mission_completed_topic", "/vehicle/mission_completed").value
        # 제22조 ②3: 25~30 s. ②2: amplitude >= ±15° at the tire, 0.5~1 Hz.
        self.duration_sec = float(p("duration_sec", 27.0).value)
        self.steer_amplitude_rad = math.radians(
            float(p("steer_amplitude_deg", 16.0).value))
        self.steer_freq_hz = float(p("steer_freq_hz", 0.7).value)
        self.drive_speed_mps = float(p("drive_speed_mps", 1.0).value)
        # ③2: the axle must stop within 3 s of the brake command.
        self.brake_stop_limit_sec = float(p("brake_stop_limit_sec", 3.0).value)
        self.brake_hold_sec = float(p("brake_hold_sec", 5.0).value)
        # Measured axle/vehicle speed. Sim: ground truth. Real car: the wheel
        # odometry topic (the wheels ARE the axle under test).
        speed_topic = p("measured_speed_topic", "/ground_truth/odom").value
        self.rest_speed_mps = float(p("rest_speed_mps", 0.05).value)

        if not 25.0 <= self.duration_sec <= 30.0:
            self.get_logger().warn(
                f"duration_sec={self.duration_sec} is outside 제22조's 25-30 s")

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, cmd_topic, 10)
        self.completed_pub = self.create_publisher(Bool, completed_topic, 10)

        be = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CanState, as_state_topic, self._on_can, be)
        self.create_subscription(Odometry, speed_topic, self._on_speed, be)
        self.create_service(Trigger, "/inspection/brake_test", self._on_brake_test)

        self.start_time = None      # set at first AS_DRIVING sighting
        self.brake_start = None     # set while a brake test is running
        self.brake_stop_time = None
        self.finished = False
        self.measured_speed = None

        self.create_timer(0.02, self._tick)  # 50 Hz command stream
        self.get_logger().info(
            "Inspection mission ready — waiting for AS_DRIVING. "
            f"(sine ±{math.degrees(self.steer_amplitude_rad):.0f}° @ "
            f"{self.steer_freq_hz} Hz, axle {self.drive_speed_mps} m/s, "
            f"{self.duration_sec:.0f} s; brake test: "
            "'ros2 service call /inspection/brake_test std_srvs/srv/Trigger')")

    def _on_can(self, msg):
        if self.start_time is None and msg.as_state == CanState.AS_DRIVING:
            self.start_time = self.get_clock().now()
            self.get_logger().info("AS_DRIVING — inspection profile started.")

    def _on_speed(self, msg):
        self.measured_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if self.brake_start is not None and self.brake_stop_time is None \
                and self.measured_speed <= self.rest_speed_mps:
            self.brake_stop_time = self.get_clock().now()
            took = (self.brake_stop_time - self.brake_start).nanoseconds * 1e-9
            verdict = "PASS" if took <= self.brake_stop_limit_sec else "FAIL"
            self.get_logger().info(
                f"DSB check: axle stopped {took:.2f} s after the brake command "
                f"(limit {self.brake_stop_limit_sec:.0f} s) — {verdict}")

    def _on_brake_test(self, _request, response):
        if self.start_time is None or self.finished:
            response.success = False
            response.message = "inspection profile is not running"
            return response
        # The rule tests a SPINNING axle stopping within 3 s — a not-yet-
        # spun-up axle would trivially "PASS" in 0.01 s and prove nothing.
        if self.measured_speed is None or self.measured_speed < 0.2:
            response.success = False
            response.message = "axle not spinning yet — retry in a few seconds"
            return response
        self.brake_start = self.get_clock().now()
        self.brake_stop_time = None
        self.get_logger().info("DSB check: braking NOW.")
        response.success = True
        response.message = "braking; watch the log for the stop verdict"
        return response

    def _elapsed(self, since):
        return (self.get_clock().now() - since).nanoseconds * 1e-9

    def _tick(self):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        if self.finished:
            # Keep reporting: the vehicle state machine's subscription is
            # plain volatile QoS, and the stream doubles as liveness.
            self.completed_pub.publish(Bool(data=True))
            self.cmd_pub.publish(cmd)  # zeros: hold still
            return

        if self.start_time is None:
            self.cmd_pub.publish(cmd)  # zeros until the mission starts
            return

        if self._elapsed(self.start_time) >= self.duration_sec:
            self.finished = True
            self.get_logger().info(
                "Inspection duration elapsed — stopping and reporting "
                "mission_completed (-> AS_FINISHED).")
            return

        if self.brake_start is not None:
            if self._elapsed(self.brake_start) < self.brake_hold_sec:
                cmd.drive.speed = 0.0
                cmd.drive.acceleration = -10.0
                self.cmd_pub.publish(cmd)
                return
            if self.brake_stop_time is None:
                self.get_logger().warn(
                    f"DSB check: axle still turning after {self.brake_hold_sec:.0f} s "
                    "of braking — FAIL")
            self.brake_start = None  # resume the profile

        t = self._elapsed(self.start_time)
        cmd.drive.steering_angle = self.steer_amplitude_rad * math.sin(
            2.0 * math.pi * self.steer_freq_hz * t)
        cmd.drive.speed = self.drive_speed_mps
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = InspectionMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
