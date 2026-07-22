#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
dssi_state — KASE 제20조 DSSI indication state from the vehicle AS/AMI state.

Maps /vehicle/as_state (hyu_msgs/CanState) onto the four DSSI indications the
rulebook allows (제20조), published as a semantic command string so the future
LED driver node can subscribe it directly and own the physical blink duty:

    /vehicle/dssi (std_msgs/String):
        "off"               everything not listed below (제20조 1항 — includes
                            AMI not selected / manual, AS_EMERGENCY_BRAKE and
                            AS_FINISHED / 자율주행 종료 상태)
        "yellow_flashing"   시스템점검 (System Check, 제6조 ④): a mission is
                            selected and the DS is activating / self-checking
        "yellow_continuous" RTAD (제6조 ⑤): AS_READY, entered only after the
                            system-check state has lasted >= rtad_dwell_sec
        "blue_continuous"   자율주행 상태 (Driverless): AS_DRIVING

The 5 s dwell of 제6조 ⑤ is enforced HERE, not assumed of the vehicle state
machine: the sim's AS_OFF -> AS_READY hop is instant, so without the dwell the
yellow-flashing system-check phase would never be visible (and a real RTAD
indication before 5 s would violate the rule). Dwell time runs on the wall
clock (time.monotonic) because the rule is about real seconds, not sim time.

The same mapping is rendered in RViz by stack_hud.py (DSSI board line).
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from hyu_msgs.msg import CanState
from std_msgs.msg import String

OFF = "off"
YELLOW_FLASHING = "yellow_flashing"
YELLOW_CONTINUOUS = "yellow_continuous"
BLUE_CONTINUOUS = "blue_continuous"


class DssiState(Node):

    def __init__(self):
        super().__init__("dssi_state")
        as_state_topic = self.declare_parameter(
            "as_state_topic", "/vehicle/as_state").value
        dssi_topic = self.declare_parameter("dssi_topic", "/vehicle/dssi").value
        self.rtad_dwell_sec = float(
            self.declare_parameter("rtad_dwell_sec", 5.0).value)

        self.can = None
        self.check_started = None  # monotonic start of the system-check phase

        # Best-effort: the sim's as_state publisher is best-effort, and this
        # subscription existing is what makes the sim publish it at all.
        self.create_subscription(
            CanState, as_state_topic, self._on_can,
            QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.pub = self.create_publisher(String, dssi_topic, 10)
        # 10 Hz republish: the state string is tiny, and a periodic stream
        # doubles as a liveness signal for the LED driver (silent = fault =
        # driver falls back to off, the rule-safe default).
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"DSSI state on '{dssi_topic}'")

    def _on_can(self, msg):
        self.can = msg

    def _state(self):
        if self.can is None:
            return OFF
        ami = self.can.ami_state
        as_state = self.can.as_state
        mission_selected = ami not in (
            CanState.AMI_NOT_SELECTED, CanState.AMI_MANUAL)
        if not mission_selected:
            self.check_started = None
            return OFF
        if as_state in (CanState.AS_EMERGENCY_BRAKE, CanState.AS_FINISHED):
            # 제20조 1항: not one of the three lit states -> off. Finished ends
            # the run, so the next mission gets a fresh system-check dwell.
            self.check_started = None
            return OFF
        if as_state == CanState.AS_DRIVING:
            return BLUE_CONTINUOUS
        # AS_OFF-with-mission / AS_READY: system check, then RTAD after dwell.
        if self.check_started is None:
            self.check_started = time.monotonic()
        in_check = time.monotonic() - self.check_started < self.rtad_dwell_sec
        if as_state == CanState.AS_READY and not in_check:
            return YELLOW_CONTINUOUS
        return YELLOW_FLASHING

    def _tick(self):
        msg = String()
        msg.data = self._state()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DssiState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
