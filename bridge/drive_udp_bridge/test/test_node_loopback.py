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


def test_command_timeout_zeros_motion_but_keeps_autonomous_switch_on():
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
            Parameter('require_map_reset', value=False),
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
    assert all(struct.unpack('<ffBB', packet) == (0.0, 0.0, 1, 0)
               for packet in initial_packets)

    assert len(fresh_packets) >= 4
    assert all(struct.unpack('<ffBB', packet)[2:] == (1, 1)
               for packet in fresh_packets)
    speed, steering, enable, autonomous_enable = struct.unpack(
        '<ffBB', fresh_packets[-1]
    )
    assert speed == 6.5
    assert steering == pytest.approx(-0.3)
    assert enable == 1
    assert autonomous_enable == 1

    assert len(stale_packets) >= 5
    assert struct.unpack('<ffBB', stale_packets[-1]) == (0.0, 0.0, 1, 1)


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
            Parameter('require_map_reset', value=False),
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
    assert all(struct.unpack('<ffBB', packet) == (0.0, 0.0, 1, 0)
               for packet in off_packets)
    assert on_packets
    assert all(struct.unpack('<ffBB', packet)[2:] == (1, 1)
               for packet in on_packets)
    assert disabled_packets
    assert all(struct.unpack('<ffBB', packet) == (0.0, 0.0, 1, 0)
               for packet in disabled_packets)


def test_driving_state_sets_switch_byte_without_command():
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
            Parameter('require_map_reset', value=False),
        ]
    )

    try:
        state = CanState()
        state.as_state = CanState.AS_DRIVING
        node._on_auto_state(state)
        packets = _collect_packets(node, receiver, 0.04)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        receiver.close()

    assert packets
    assert all(struct.unpack('<ffBB', packet) == (0.0, 0.0, 1, 1)
               for packet in packets)


class _FakeSlam:
    """A /graph_slam/reset stand-in that records calls and answers on request."""

    def __init__(self, success=True):
        from rclpy.node import Node
        from std_srvs.srv import Trigger
        self.node = Node('fake_graph_slam')
        self.calls = 0
        self.success = success
        self.node.create_service(Trigger, '/graph_slam/reset', self._handle)

    def _handle(self, request, response):
        self.calls += 1
        response.success = self.success
        response.message = 'fake reset'
        return response


def _collect_with(nodes, receiver, duration_sec):
    packets = []
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.005)
        while True:
            try:
                packet, _ = receiver.recvfrom(64)
                packets.append(packet)
            except BlockingIOError:
                break
    return packets


def _gated_bridge(receiver_port, timeout=0.3):
    return DriveUdpBridge(
        parameter_overrides=[
            Parameter('ecu_ip', value='127.0.0.1'),
            Parameter('ecu_port', value=receiver_port),
            Parameter('local_bind_ip', value='127.0.0.1'),
            Parameter('local_bind_port', value=0),
            Parameter('send_rate_hz', value=100.0),
            Parameter('command_timeout_sec', value=1.0),
            Parameter('auto_state_timeout_sec', value=1.0),
            Parameter('require_map_reset', value=True),
            Parameter('map_reset_timeout_sec', value=timeout),
        ]
    )


def test_switch_on_raises_autonomous_only_after_map_reset_and_off_drops_at_once():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    receiver.setblocking(False)
    receiver_port = receiver.getsockname()[1]

    rclpy.init()
    slam = _FakeSlam(success=True)
    node = _gated_bridge(receiver_port)
    try:
        # let the client discover the service
        _collect_with([node, slam.node], receiver, 0.3)
        state = CanState()
        state.as_state = CanState.AS_DRIVING
        node._on_auto_state(state)
        packets = _collect_with([node, slam.node], receiver, 0.3)
        # ON -> OFF: immediate
        state.as_state = CanState.AS_READY
        node._on_auto_state(state)
        off_packets = _collect_with([node, slam.node], receiver, 0.05)
    finally:
        node.destroy_node()
        slam.node.destroy_node()
        rclpy.shutdown()
        receiver.close()

    assert slam.calls == 1, 'one reset per OFF->ON edge'
    flags = [struct.unpack('<ffBB', p)[3] for p in packets]
    assert flags[0] == 0, 'no enable before the reset answered'
    assert flags[-1] == 1, 'enable once the reset succeeded'
    # monotone: 0...0 then 1...1
    first_on = flags.index(1)
    assert all(f == 0 for f in flags[:first_on]) and all(f == 1 for f in flags[first_on:])
    assert off_packets and all(struct.unpack('<ffBB', p)[3] == 0 for p in off_packets)


def test_switch_on_stays_disabled_while_reset_is_refused_or_absent():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    receiver.setblocking(False)
    receiver_port = receiver.getsockname()[1]

    rclpy.init()
    slam = _FakeSlam(success=False)
    node = _gated_bridge(receiver_port, timeout=0.1)
    try:
        _collect_with([node, slam.node], receiver, 0.2)
        state = CanState()
        state.as_state = CanState.AS_DRIVING
        node._on_auto_state(state)
        packets = _collect_with([node, slam.node], receiver, 0.8)
    finally:
        node.destroy_node()
        slam.node.destroy_node()
        rclpy.shutdown()
        receiver.close()

    assert slam.calls >= 2, 'refused resets are retried'
    assert packets and all(struct.unpack('<ffBB', p)[3] == 0 for p in packets)

    # No service at all: still 0, no exception.
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    receiver.setblocking(False)
    rclpy.init()
    node = _gated_bridge(receiver.getsockname()[1], timeout=0.1)
    try:
        state = CanState()
        state.as_state = CanState.AS_DRIVING
        node._on_auto_state(state)
        packets = _collect_with([node], receiver, 0.3)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        receiver.close()
    assert packets and all(struct.unpack('<ffBB', p)[3] == 0 for p in packets)
