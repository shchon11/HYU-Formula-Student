# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""Convert cumulative wheel-encoder counts into per-wheel linear speeds."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from drive_udp_bridge.protocol import EncoderFeedback


UINT32_MODULUS = 1 << 32
UINT32_HALF_RANGE = 1 << 31


def encoder_count_delta(current: int, previous: int) -> int:
    """Return the shortest signed delta between wrapping uint32 counters."""
    return (
        (int(current) - int(previous) + UINT32_HALF_RANGE)
        % UINT32_MODULUS
        - UINT32_HALF_RANGE
    )


@dataclass(frozen=True)
class WheelSpeedConfig:
    """Encoder scale, timing, and validation limits."""

    encoder_counts_per_revolution: int
    tire_diameter_m: float
    feedback_timeout_sec: float
    max_wheel_speed_mps: float


@dataclass(frozen=True)
class WheelSpeedEstimate:
    """One four-wheel linear-speed sample in metres per second."""

    front_left_mps: float
    front_right_mps: float
    rear_left_mps: float
    rear_right_mps: float


class WheelSpeedConverter:
    """Calculate four wheel speeds from successive cumulative counters."""

    def __init__(self, config: WheelSpeedConfig) -> None:
        if config.encoder_counts_per_revolution <= 0:
            raise ValueError('encoder_counts_per_revolution must be positive')
        if not math.isfinite(config.tire_diameter_m) or config.tire_diameter_m <= 0.0:
            raise ValueError('tire_diameter_m must be finite and greater than zero')
        if (
            not math.isfinite(config.feedback_timeout_sec)
            or config.feedback_timeout_sec <= 0.0
        ):
            raise ValueError(
                'feedback_timeout_sec must be finite and greater than zero'
            )
        if (
            not math.isfinite(config.max_wheel_speed_mps)
            or config.max_wheel_speed_mps < 0.0
        ):
            raise ValueError('max_wheel_speed_mps must be finite and non-negative')

        self._config = config
        self._meters_per_count = (
            math.pi * config.tire_diameter_m
            / float(config.encoder_counts_per_revolution)
        )
        self._previous: Optional[Tuple[EncoderFeedback, float]] = None

    @property
    def meters_per_count(self) -> float:
        """Return the wheel travel represented by one encoder count, in metres."""
        return self._meters_per_count

    def reset_timing(self) -> None:
        """Require a new encoder baseline before calculating another speed."""
        self._previous = None

    def update(
        self,
        feedback: EncoderFeedback,
        received_at: float,
    ) -> Optional[WheelSpeedEstimate]:
        """Consume a cumulative-count sample and return four speeds when possible."""
        if not math.isfinite(received_at):
            raise ValueError('received_at must be finite')

        if self._previous is None:
            self._previous = (feedback, received_at)
            return None

        previous, previous_time = self._previous
        dt = received_at - previous_time
        self._previous = (feedback, received_at)
        if dt <= 0.0 or dt > self._config.feedback_timeout_sec:
            return None

        count_deltas = (
            encoder_count_delta(
                feedback.front_left_count, previous.front_left_count),
            encoder_count_delta(
                feedback.front_right_count, previous.front_right_count),
            encoder_count_delta(
                feedback.rear_left_count, previous.rear_left_count),
            encoder_count_delta(
                feedback.rear_right_count, previous.rear_right_count),
        )
        speeds = tuple(
            delta * self._meters_per_count / dt
            for delta in count_deltas
        )
        if (
            self._config.max_wheel_speed_mps > 0.0
            and any(
                abs(speed) > self._config.max_wheel_speed_mps
                for speed in speeds
            )
        ):
            raise ValueError('encoder delta exceeds max_wheel_speed_mps')

        return WheelSpeedEstimate(
            front_left_mps=speeds[0],
            front_right_mps=speeds[1],
            rear_left_mps=speeds[2],
            rear_right_mps=speeds[3],
        )
