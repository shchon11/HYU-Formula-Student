# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""Binary packet and command-watchdog helpers for the UDP bridge."""

from dataclasses import dataclass
import math
import struct
import time
from typing import Callable, Optional, Tuple


PACKET_FORMAT = '<ffBB'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
# ECU -> AGX feedback: four per-wheel RPM values (FL FR RL RR), little endian.
# The element type is whatever the Speedgoat UDP Send block emits; pick it with
# feedback_value_type. float32 mirrors the single-precision command packet.
FEEDBACK_VALUE_TYPES = {
    'float32': 'f',
    'float64': 'd',
    'int16': 'h',
    'uint16': 'H',
    'int32': 'i',
    'uint32': 'I',
}
DEFAULT_FEEDBACK_VALUE_TYPE = 'float32'
FEEDBACK_WHEEL_COUNT = 4
FLOAT32_MAX = 3.4028234663852886e38
SAFE_COMMAND = (0.0, 0.0, 0)


def is_valid_float32(value: float) -> bool:
    """Return whether value is finite and representable as IEEE-754 float32."""
    return math.isfinite(value) and abs(value) <= FLOAT32_MAX


def pack_command(
    speed: float,
    steering_angle: float,
    enable: int,
    autonomous_enable: int,
) -> bytes:
    """Pack speed and steering-wheel radians into one Speedgoat datagram."""
    if not is_valid_float32(speed) or not is_valid_float32(steering_angle):
        raise ValueError(
            'speed and steering-wheel angle must be finite float32 values'
        )
    if enable not in (0, 1):
        raise ValueError('enable must be 0 or 1')
    if autonomous_enable not in (0, 1):
        raise ValueError('autonomous_enable must be 0 or 1')
    return struct.pack(
        PACKET_FORMAT,
        speed,
        steering_angle,
        enable,
        autonomous_enable,
    )


def feedback_format(value_type: str = DEFAULT_FEEDBACK_VALUE_TYPE) -> str:
    """Return the struct format of one four-wheel RPM datagram."""
    try:
        code = FEEDBACK_VALUE_TYPES[value_type]
    except KeyError:
        raise ValueError(
            'feedback_value_type must be one of '
            + ', '.join(sorted(FEEDBACK_VALUE_TYPES))
            + f', got {value_type!r}'
        ) from None
    return '<' + code * FEEDBACK_WHEEL_COUNT


def feedback_size(value_type: str = DEFAULT_FEEDBACK_VALUE_TYPE) -> int:
    """Return the byte length of one four-wheel RPM datagram."""
    return struct.calcsize(feedback_format(value_type))


@dataclass(frozen=True)
class EncoderFeedback:
    """One ECU wheel-encoder datagram: four wheel RPM readings."""

    front_left_rpm: float
    front_right_rpm: float
    rear_left_rpm: float
    rear_right_rpm: float


def unpack_encoder_feedback(
    packet: bytes,
    value_type: str = DEFAULT_FEEDBACK_VALUE_TYPE,
) -> EncoderFeedback:
    """Decode one little-endian FL FR RL RR RPM datagram of the given element type."""
    fmt = feedback_format(value_type)
    size = struct.calcsize(fmt)
    if len(packet) != size:
        raise ValueError(
            f'encoder feedback packet must be exactly {size} bytes '
            f'for feedback_value_type {value_type} (4 x {value_type}), '
            f'got {len(packet)}'
        )

    values = tuple(float(value) for value in struct.unpack(fmt, packet))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('encoder feedback RPM values must be finite')
    return EncoderFeedback(
        front_left_rpm=values[0],
        front_right_rpm=values[1],
        rear_left_rpm=values[2],
        rear_right_rpm=values[3],
    )


@dataclass(frozen=True)
class CommandSnapshot:
    """Values selected by the watchdog for one outgoing packet."""

    speed: float
    steering_angle: float
    enable: int


class CommandWatchdog:
    """Store the newest valid command and fail closed when it becomes stale."""

    def __init__(
        self,
        timeout_sec: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError('command_timeout_sec must be finite and greater than zero')
        self._timeout_sec = timeout_sec
        self._clock = clock
        self._latest: Optional[Tuple[float, float, float]] = None

    def update(
        self,
        speed: float,
        steering_angle: float,
        received_at: Optional[float] = None,
    ) -> bool:
        """Store a command, or immediately invalidate state for bad input."""
        if not is_valid_float32(speed) or not is_valid_float32(steering_angle):
            self.invalidate()
            return False

        timestamp = self._clock() if received_at is None else received_at
        if not math.isfinite(timestamp):
            self.invalidate()
            return False

        self._latest = (float(speed), float(steering_angle), float(timestamp))
        return True

    def invalidate(self) -> None:
        """Clear the stored command so the next packet is disabled and zeroed."""
        self._latest = None

    def snapshot(self, now: Optional[float] = None) -> CommandSnapshot:
        """Return a fresh command or the fail-closed command."""
        if self._latest is None:
            return CommandSnapshot(*SAFE_COMMAND)

        current_time = self._clock() if now is None else now
        speed, steering_angle, received_at = self._latest
        age_sec = current_time - received_at
        if not math.isfinite(age_sec) or age_sec < 0.0 or age_sec > self._timeout_sec:
            return CommandSnapshot(*SAFE_COMMAND)

        return CommandSnapshot(speed, steering_angle, 1)


class AutonomousStateWatchdog:
    """Fail closed unless a recent vehicle state says autonomous driving."""

    def __init__(
        self,
        timeout_sec: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError(
                'auto_state_timeout_sec must be finite and greater than zero'
            )
        self._timeout_sec = timeout_sec
        self._clock = clock
        self._latest: Optional[Tuple[bool, float]] = None

    def update(
        self,
        enabled: bool,
        received_at: Optional[float] = None,
    ) -> bool:
        """Store the latest autonomous switch state and receive time."""
        timestamp = self._clock() if received_at is None else received_at
        if not math.isfinite(timestamp):
            self.invalidate()
            return False

        self._latest = (bool(enabled), float(timestamp))
        return True

    def invalidate(self) -> None:
        """Clear the state so autonomous mode is disabled."""
        self._latest = None

    def enabled(self, now: Optional[float] = None) -> bool:
        """Return true only for a fresh autonomous-on state."""
        if self._latest is None:
            return False

        current_time = self._clock() if now is None else now
        enabled, received_at = self._latest
        age_sec = current_time - received_at
        if not math.isfinite(age_sec) or age_sec < 0.0 or age_sec > self._timeout_sec:
            return False

        return enabled
