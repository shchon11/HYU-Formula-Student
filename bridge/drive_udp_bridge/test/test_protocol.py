# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

import math
import struct

from drive_udp_bridge.protocol import (
    AutonomousStateWatchdog,
    CommandSnapshot,
    CommandWatchdog,
    feedback_format,
    feedback_size,
    pack_command,
    PACKET_FORMAT,
    PACKET_SIZE,
    unpack_encoder_feedback,
)
import pytest


def test_packet_layout_is_exactly_little_endian_float_float_two_bytes():
    packet = pack_command(1.0, 2.0, 1, 1)

    assert PACKET_FORMAT == '<ffBB'
    assert PACKET_SIZE == 10
    assert len(packet) == 10
    assert packet == b'\x00\x00\x80?\x00\x00\x00@\x01\x01'
    assert struct.unpack('<ffBB', packet) == (1.0, 2.0, 1, 1)


@pytest.mark.parametrize('enable', [-1, 2, 255])
def test_packet_rejects_unknown_enable_value(enable):
    with pytest.raises(ValueError, match='enable'):
        pack_command(0.0, 0.0, enable, 0)


@pytest.mark.parametrize('autonomous_enable', [-1, 2, 255])
def test_packet_rejects_unknown_autonomous_enable_value(autonomous_enable):
    with pytest.raises(ValueError, match='autonomous_enable'):
        pack_command(0.0, 0.0, 1, autonomous_enable)


@pytest.mark.parametrize('bad_value', [math.nan, math.inf, -math.inf, 1.0e39])
def test_packet_rejects_values_that_are_not_finite_float32(bad_value):
    with pytest.raises(ValueError, match='float32'):
        pack_command(bad_value, 0.0, 1, 1)
    with pytest.raises(ValueError, match='float32'):
        pack_command(0.0, bad_value, 1, 1)


def test_encoder_feedback_default_is_four_little_endian_float32_rpm():
    packet = struct.pack('<ffff', 10.5, -20.0, 30.0, 40.0)

    feedback = unpack_encoder_feedback(packet)

    assert feedback_format() == '<ffff'
    assert feedback_size() == 16
    assert feedback.front_left_rpm == 10.5
    assert feedback.front_right_rpm == -20.0
    assert feedback.rear_left_rpm == 30.0
    assert feedback.rear_right_rpm == 40.0


@pytest.mark.parametrize(
    'value_type,fmt,size',
    [
        ('float32', '<ffff', 16),
        ('float64', '<dddd', 32),
        ('int16', '<hhhh', 8),
        ('uint16', '<HHHH', 8),
        ('int32', '<iiii', 16),
        ('uint32', '<IIII', 16),
    ],
)
def test_encoder_feedback_value_type_selects_element_format(value_type, fmt, size):
    assert feedback_format(value_type) == fmt
    assert feedback_size(value_type) == size

    feedback = unpack_encoder_feedback(struct.pack(fmt, 1, 2, 3, 4), value_type)

    assert (
        feedback.front_left_rpm,
        feedback.front_right_rpm,
        feedback.rear_left_rpm,
        feedback.rear_right_rpm,
    ) == (1.0, 2.0, 3.0, 4.0)


def test_encoder_feedback_rejects_wrong_size_naming_the_expected_one():
    with pytest.raises(ValueError, match='exactly 16 bytes.*float32'):
        unpack_encoder_feedback(b'bad')
    with pytest.raises(ValueError, match='exactly 32 bytes.*float64'):
        unpack_encoder_feedback(struct.pack('<ffff', 1, 2, 3, 4), 'float64')


def test_encoder_feedback_rejects_unknown_value_type_and_non_finite_rpm():
    with pytest.raises(ValueError, match='feedback_value_type'):
        unpack_encoder_feedback(b'\0' * 16, 'double')
    with pytest.raises(ValueError, match='finite'):
        unpack_encoder_feedback(struct.pack('<ffff', math.nan, 0, 0, 0))


def test_watchdog_starts_disabled_then_enables_only_while_fresh():
    now = [10.0]
    watchdog = CommandWatchdog(0.2, clock=lambda: now[0])

    assert watchdog.snapshot() == CommandSnapshot(0.0, 0.0, 0)
    assert watchdog.update(4.5, -0.25)
    assert watchdog.snapshot() == CommandSnapshot(4.5, -0.25, 1)

    now[0] = 10.2
    assert watchdog.snapshot() == CommandSnapshot(4.5, -0.25, 1)

    now[0] = 10.200001
    assert watchdog.snapshot() == CommandSnapshot(0.0, 0.0, 0)


@pytest.mark.parametrize('bad_value', [math.nan, math.inf, -math.inf, 1.0e39])
def test_bad_command_immediately_invalidates_previous_command(bad_value):
    watchdog = CommandWatchdog(0.2, clock=lambda: 10.0)
    assert watchdog.update(3.0, 0.1)

    assert not watchdog.update(bad_value, 0.2)
    assert watchdog.snapshot() == CommandSnapshot(0.0, 0.0, 0)


def test_negative_clock_age_fails_closed():
    watchdog = CommandWatchdog(0.2, clock=lambda: 9.0)
    assert watchdog.update(3.0, 0.1, received_at=10.0)

    assert watchdog.snapshot() == CommandSnapshot(0.0, 0.0, 0)


def test_autonomous_state_requires_recent_enabled_state():
    now = [10.0]
    watchdog = AutonomousStateWatchdog(0.5, clock=lambda: now[0])

    assert not watchdog.enabled()
    assert watchdog.update(True)
    assert watchdog.enabled()

    now[0] = 10.5
    assert watchdog.enabled()

    now[0] = 10.500001
    assert not watchdog.enabled()


def test_autonomous_off_state_disables_immediately():
    watchdog = AutonomousStateWatchdog(0.5, clock=lambda: 10.0)

    assert watchdog.update(True)
    assert watchdog.enabled()
    assert watchdog.update(False)
    assert not watchdog.enabled()
