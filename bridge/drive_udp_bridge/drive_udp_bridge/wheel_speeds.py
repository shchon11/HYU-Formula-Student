# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""Convert ECU per-wheel RPM feedback into per-wheel linear speeds."""

from dataclasses import dataclass
import math

from drive_udp_bridge.protocol import EncoderFeedback


@dataclass(frozen=True)
class WheelSpeedConfig:
    """Tire size, RPM scale, and validation limits."""

    tire_diameter_m: float
    # Revolutions reported by the ECU per one tire revolution: 1.0 = the ECU
    # already sends wheel-side RPM; >1.0 = motor-side RPM divided down.
    rpm_gear_ratio: float
    max_wheel_speed_mps: float


@dataclass(frozen=True)
class WheelSpeedEstimate:
    """One four-wheel linear-speed sample in metres per second."""

    front_left_mps: float
    front_right_mps: float
    rear_left_mps: float
    rear_right_mps: float


class WheelSpeedConverter:
    """Scale four RPM readings to m/s and reject impossible values."""

    def __init__(self, config: WheelSpeedConfig) -> None:
        if not math.isfinite(config.tire_diameter_m) or config.tire_diameter_m <= 0.0:
            raise ValueError('tire_diameter_m must be finite and greater than zero')
        if not math.isfinite(config.rpm_gear_ratio) or config.rpm_gear_ratio <= 0.0:
            raise ValueError('rpm_gear_ratio must be finite and greater than zero')
        if (
            not math.isfinite(config.max_wheel_speed_mps)
            or config.max_wheel_speed_mps < 0.0
        ):
            raise ValueError('max_wheel_speed_mps must be finite and non-negative')

        self._config = config
        # One wheel revolution per minute covers pi * D metres in 60 s.
        self._mps_per_rpm = (
            math.pi * config.tire_diameter_m / 60.0 / config.rpm_gear_ratio
        )

    @property
    def mps_per_rpm(self) -> float:
        """Return the linear speed represented by 1 RPM, in m/s."""
        return self._mps_per_rpm

    def convert(self, feedback: EncoderFeedback) -> WheelSpeedEstimate:
        """Scale one RPM sample to four wheel speeds; ValueError if implausible."""
        speeds = tuple(
            rpm * self._mps_per_rpm
            for rpm in (
                feedback.front_left_rpm,
                feedback.front_right_rpm,
                feedback.rear_left_rpm,
                feedback.rear_right_rpm,
            )
        )
        if (
            self._config.max_wheel_speed_mps > 0.0
            and any(
                abs(speed) > self._config.max_wheel_speed_mps
                for speed in speeds
            )
        ):
            raise ValueError('wheel RPM exceeds max_wheel_speed_mps')

        return WheelSpeedEstimate(
            front_left_mps=speeds[0],
            front_right_mps=speeds[1],
            rear_left_mps=speeds[2],
            rear_right_mps=speeds[3],
        )
