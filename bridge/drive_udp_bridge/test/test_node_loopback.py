# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

import socket
import struct
import time

from ackermann_msgs.msg import AckermannDriveStamped
from drive_udp_bridge.drive_udp_bridge_node import DriveUdpBridge
import pytest
import rclpy
from rclpy.parameter import Parameter


def _collect_packets(node, receiver, duration_sec):
    packets = []
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        while True:
            try:
                packet, _ = receiver.recvfrom(64)
                packets.append(packet)
            except BlockingIOError:
                break
    return packets


def test_timer_repeats_latest_command_then_sends_disabled_zeros_after_timeout():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    receiver.setblocking(False)
    receiver_port = receiver.getsockname()[1]

    rclpy.init()
    node = DriveUdpBridge(
        parameter_overrides=[
            Parameter('ecu_ip', value='127.0.0.1'),
            Parameter('ecu_port', value=receiver_port),
            Parameter('local_bind_ip', value='127.0.0.1'),
            Parameter('local_bind_port', value=0),
            Parameter('send_rate_hz', value=100.0),
            Parameter('command_timeout_sec', value=0.15),
        ]
    )

    try:
        initial_packets = _collect_packets(node, receiver, 0.04)

        command = AckermannDriveStamped()
        command.drive.speed = 6.5
        command.drive.steering_angle = -0.3
        node._on_command(command)
        fresh_packets = _collect_packets(node, receiver, 0.08)
        stale_packets = _collect_packets(node, receiver, 0.12)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        receiver.close()

    assert len(initial_packets) >= 2
    assert all(struct.unpack('<ffB', packet) == (0.0, 0.0, 0)
               for packet in initial_packets)

    assert len(fresh_packets) >= 4
    assert all(struct.unpack('<ffB', packet)[2] == 1 for packet in fresh_packets)
    speed, steering, enable = struct.unpack('<ffB', fresh_packets[-1])
    assert speed == 6.5
    assert steering == pytest.approx(-0.3)
    assert enable == 1

    assert len(stale_packets) >= 5
    assert struct.unpack('<ffB', stale_packets[-1]) == (0.0, 0.0, 0)
