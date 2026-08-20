# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

import math
import struct

from drive_udp_bridge.protocol import (
    AutonomousStateWatchdog,
    CommandSnapshot,
    CommandWatchdog,
    pack_command,
    PACKET_FORMAT,
    PACKET_SIZE,
)
import pytest


def test_packet_layout_is_exactly_little_endian_float_float_byte():
    packet = pack_command(1.0, 2.0, 1)

    assert PACKET_FORMAT == '<ffB'
    assert PACKET_SIZE == 9
    assert len(packet) == 9
    assert packet == b'\x00\x00\x80?\x00\x00\x00@\x01'
    assert struct.unpack('<ffB', packet) == (1.0, 2.0, 1)


@pytest.mark.parametrize('enable', [-1, 2, 255])
def test_packet_rejects_unknown_enable_value(enable):
    with pytest.raises(ValueError, match='enable'):
        pack_command(0.0, 0.0, enable)


@pytest.mark.parametrize('bad_value', [math.nan, math.inf, -math.inf, 1.0e39])
def test_packet_rejects_values_that_are_not_finite_float32(bad_value):
    with pytest.raises(ValueError, match='float32'):
        pack_command(bad_value, 0.0, 1)
    with pytest.raises(ValueError, match='float32'):
        pack_command(0.0, bad_value, 1)


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
