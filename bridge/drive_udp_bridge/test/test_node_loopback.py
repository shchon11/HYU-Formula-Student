# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

import math
import socket
import struct
import time

from ackermann_msgs.msg import AckermannDriveStamped
from drive_udp_bridge.drive_udp_bridge_node import DriveUdpBridge
from hyu_msgs.msg import CanState, WheelSpeedsStamped
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


def _unused_udp_port():
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]
    finally:
        probe.close()


def _spin_nodes(nodes, duration_sec):
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.005)


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
        command.drive.steering_angle = math.radians(-12.63242)
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
    assert steering == pytest.approx(-math.pi / 3.0)
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
        command.drive.steering_angle = 0.5
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
    assert struct.unpack('<ffBB', on_packets[-1])[1] == pytest.approx(
        math.pi / 2.0
    )
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


def test_rpm_feedback_publishes_four_wheel_speeds_in_mps():
    command_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command_receiver.bind(('127.0.0.1', 0))
    feedback_port = _unused_udp_port()
    feedback_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    rclpy.init()
    node = DriveUdpBridge(
        parameter_overrides=[
            Parameter('ecu_ip', value='127.0.0.1'),
            Parameter(
                'ecu_port', value=command_receiver.getsockname()[1]),
            Parameter('local_bind_ip', value='127.0.0.1'),
            Parameter('feedback_bind_ip', value='127.0.0.1'),
            Parameter('feedback_port', value=feedback_port),
            Parameter('feedback_poll_rate_hz', value=500.0),
            Parameter('feedback_timeout_sec', value=0.5),
            # pi * D = 1 m per revolution: 60 RPM -> 1 m/s.
            Parameter('tire_diameter_m', value=1.0 / 3.141592653589793),
            Parameter('max_wheel_speed_mps', value=0.0),
            Parameter('wheel_speeds_topic', value='/test/wheel_speeds'),
            Parameter('wheel_speeds_frame_id', value='base_footprint'),
            Parameter('require_map_reset', value=False),
        ]
    )
    observer = rclpy.create_node('drive_udp_bridge_wheel_speed_observer')
    wheel_speeds = []
    observer.create_subscription(
        WheelSpeedsStamped,
        '/test/wheel_speeds',
        wheel_speeds.append,
        10,
    )

    try:
        # RPM is instantaneous: a single datagram is enough to publish.
        feedback_sender.sendto(
            struct.pack('<ffff', 60.0, 120.0, -180.0, 240.0),
            ('127.0.0.1', feedback_port),
        )
        _spin_nodes((node, observer), 0.08)
    finally:
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        feedback_sender.close()
        command_receiver.close()

    assert wheel_speeds
    message = wheel_speeds[-1]
    assert message.header.frame_id == 'base_footprint'
    assert message.speeds.lf_speed == pytest.approx(1.0, rel=1.0e-5)
    assert message.speeds.rf_speed == pytest.approx(2.0, rel=1.0e-5)
    assert message.speeds.lb_speed == pytest.approx(-3.0, rel=1.0e-5)
    assert message.speeds.rb_speed == pytest.approx(4.0, rel=1.0e-5)


def test_feedback_value_type_float64_and_wrong_size_is_dropped():
    command_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command_receiver.bind(('127.0.0.1', 0))
    feedback_port = _unused_udp_port()
    feedback_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    rclpy.init()
    node = _feedback_bridge(
        command_receiver.getsockname()[1], feedback_port,
        feedback_value_type='float64')
    observer = rclpy.create_node('drive_udp_bridge_wheel_speed_observer')
    wheel_speeds = []
    observer.create_subscription(
        WheelSpeedsStamped, '/test/wheel_speeds', wheel_speeds.append, 10)
    try:
        # 16-byte float32 datagram under a float64 contract: rejected, no output.
        feedback_sender.sendto(
            struct.pack('<ffff', 60.0, 60.0, 60.0, 60.0), ('127.0.0.1', feedback_port))
        _spin_nodes((node, observer), 0.06)
        assert wheel_speeds == []
        feedback_sender.sendto(
            struct.pack('<dddd', 60.0, 120.0, 180.0, 240.0), ('127.0.0.1', feedback_port))
        _spin_nodes((node, observer), 0.08)
    finally:
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        feedback_sender.close()
        command_receiver.close()

    assert wheel_speeds
    assert wheel_speeds[-1].speeds.lf_speed == pytest.approx(1.0, rel=1.0e-5)
    assert wheel_speeds[-1].speeds.rb_speed == pytest.approx(4.0, rel=1.0e-5)


def _feedback_bridge(command_port, feedback_port, **extra):
    overrides = [
        Parameter('ecu_ip', value='127.0.0.1'),
        Parameter('ecu_port', value=command_port),
        Parameter('local_bind_ip', value='127.0.0.1'),
        Parameter('feedback_bind_ip', value='127.0.0.1'),
        Parameter('feedback_port', value=feedback_port),
        Parameter('feedback_poll_rate_hz', value=500.0),
        Parameter('feedback_timeout_sec', value=0.5),
        Parameter('tire_diameter_m', value=1.0 / 3.141592653589793),
        Parameter('max_wheel_speed_mps', value=0.0),
        Parameter('wheel_speeds_topic', value='/test/wheel_speeds'),
        Parameter('require_map_reset', value=False),
    ]
    overrides.extend(Parameter(name, value=value) for name, value in extra.items())
    return DriveUdpBridge(parameter_overrides=overrides)


def _run_feedback_pair(feedback_source_ip):
    """Feed one RPM packet from 127.0.0.1; return the published messages."""
    command_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command_receiver.bind(('127.0.0.1', 0))
    feedback_port = _unused_udp_port()
    feedback_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    rclpy.init()
    node = _feedback_bridge(
        command_receiver.getsockname()[1], feedback_port,
        feedback_source_ip=feedback_source_ip)
    observer = rclpy.create_node('drive_udp_bridge_wheel_speed_observer')
    wheel_speeds = []
    observer.create_subscription(
        WheelSpeedsStamped, '/test/wheel_speeds', wheel_speeds.append, 10)
    try:
        feedback_sender.sendto(
            struct.pack('<ffff', 60.0, 120.0, 180.0, 240.0), ('127.0.0.1', feedback_port))
        _spin_nodes((node, observer), 0.08)
        now = node.get_clock().now().nanoseconds * 1.0e-9
    finally:
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        feedback_sender.close()
        command_receiver.close()
    return wheel_speeds, now


def test_feedback_source_filter_drops_other_senders_and_any_accepts_all():
    # Pinned to an address the loopback sender is not: nothing may come out.
    filtered, _ = _run_feedback_pair('10.255.255.1')
    assert filtered == []

    # 0.0.0.0 = accept any source: the same packets publish, stamped close to
    # the moment they were received (not some old poll tick).
    accepted, now = _run_feedback_pair('0.0.0.0')
    assert accepted
    stamp = accepted[-1].header.stamp
    stamp_sec = stamp.sec + stamp.nanosec * 1.0e-9
    assert 0.0 <= now - stamp_sec < 0.2
    assert accepted[-1].speeds.rb_speed == pytest.approx(
        4.0 * accepted[-1].speeds.lf_speed, rel=1.0e-5)
