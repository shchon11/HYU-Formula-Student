# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""Binary packet and command-watchdog helpers for the UDP bridge."""

from dataclasses import dataclass
import math
import struct
import time
from typing import Callable, Optional, Tuple


PACKET_FORMAT = '<ffB'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
FLOAT32_MAX = 3.4028234663852886e38
SAFE_COMMAND = (0.0, 0.0, 0)


def is_valid_float32(value: float) -> bool:
    """Return whether value is finite and representable as IEEE-754 float32."""
    return math.isfinite(value) and abs(value) <= FLOAT32_MAX


def pack_command(speed: float, steering_angle: float, enable: int) -> bytes:
    """Pack one 9-byte little-endian Speedgoat command datagram."""
    if not is_valid_float32(speed) or not is_valid_float32(steering_angle):
        raise ValueError('speed and steering_angle must be finite float32 values')
    if enable not in (0, 1):
        raise ValueError('enable must be 0 or 1')
    return struct.pack(PACKET_FORMAT, speed, steering_angle, enable)


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
