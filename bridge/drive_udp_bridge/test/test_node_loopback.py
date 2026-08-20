# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

import socket
import struct
import time

from ackermann_msgs.msg import AckermannDriveStamped
from drive_udp_bridge.drive_udp_bridge_node import DriveUdpBridge
from hyu_msgs.msg import CanState
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
            Parameter('auto_state_timeout_sec', value=1.0),
        ]
    )

    try:
        initial_packets = _collect_packets(node, receiver, 0.04)

        command = AckermannDriveStamped()
        command.drive.speed = 6.5
        command.drive.steering_angle = -0.3
        node._on_command(command)
        auto_state = CanState()
        auto_state.as_state = CanState.AS_DRIVING
        node._on_auto_state(auto_state)
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


def test_non_driving_state_gates_fresh_command():
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
            Parameter('command_timeout_sec', value=1.0),
            Parameter('auto_state_timeout_sec', value=1.0),
        ]
    )

    try:
        command = AckermannDriveStamped()
        command.drive.speed = 4.0
        command.drive.steering_angle = 0.2
        node._on_command(command)

        off_packets = _collect_packets(node, receiver, 0.04)

        state = CanState()
        state.as_state = CanState.AS_DRIVING
        node._on_auto_state(state)
        on_packets = _collect_packets(node, receiver, 0.04)

        state.as_state = CanState.AS_READY
        node._on_auto_state(state)
        disabled_packets = _collect_packets(node, receiver, 0.04)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        receiver.close()

    assert off_packets
    assert all(struct.unpack('<ffB', packet) == (0.0, 0.0, 0)
               for packet in off_packets)
    assert on_packets
    assert all(struct.unpack('<ffB', packet)[2] == 1 for packet in on_packets)
    assert disabled_packets
    assert all(struct.unpack('<ffB', packet) == (0.0, 0.0, 0)
               for packet in disabled_packets)
