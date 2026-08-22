# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""Configuration model and startup validation for the UDP bridge."""

from dataclasses import dataclass
import ipaddress
import math


@dataclass(frozen=True)
class BridgeConfig:
    """Fully resolved ROS parameters used by the bridge."""

    command_topic: str
    auto_state_topic: str
    ecu_ip: str
    ecu_port: int
    local_bind_ip: str
    local_bind_port: int
    send_rate_hz: float
    command_timeout_sec: float
    auto_state_timeout_sec: float
    steering_calibration_csv: str
    max_steering_wheel_angle_deg: float
    feedback_bind_ip: str
    feedback_port: int
    feedback_poll_rate_hz: float
    feedback_timeout_sec: float
    encoder_counts_per_revolution: int
    tire_diameter_m: float
    max_wheel_speed_mps: float
    wheel_speeds_topic: str
    wheel_speeds_frame_id: str
    # Source address feedback datagrams must come from. '' = the ECU command
    # address (ecu_ip); '0.0.0.0' = accept any source.
    feedback_source_ip: str = ''

    @property
    def feedback_enabled(self) -> bool:
        """Return whether the ECU feedback endpoint was configured."""
        return bool(self.feedback_bind_ip.strip()) and self.feedback_port != 0

    @property
    def feedback_ready(self) -> bool:
        """
        Return whether feedback can actually be decoded (scale known).

        The endpoint is ours to pick and can be configured ahead of time; the
        counts-per-revolution comes from the ECU team. Until it is filled the
        node runs with feedback off (and says so) rather than refusing to
        start -- the command path must never depend on it.
        """
        return self.feedback_enabled and self.encoder_counts_per_revolution > 0

    @property
    def feedback_source_filter(self):
        """Return the source IPv4 feedback must come from, or None for any."""
        source = self.feedback_source_ip.strip() or self.ecu_ip.strip()
        return None if source == '0.0.0.0' else source


def _validate_ipv4(value: str, parameter_name: str) -> None:
    if not value:
        raise ValueError(f'{parameter_name} must not be empty')
    try:
        ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as error:
        raise ValueError(f'{parameter_name} must be a valid IPv4 address') from error


def validate_config(config: BridgeConfig) -> None:
    """Raise ValueError when a bridge parameter is unsafe or malformed."""
    if not config.command_topic.strip():
        raise ValueError('command_topic must not be empty')
    if not config.auto_state_topic.strip():
        raise ValueError('auto_state_topic must not be empty')

    _validate_ipv4(config.ecu_ip.strip(), 'ecu_ip')
    _validate_ipv4(config.local_bind_ip.strip(), 'local_bind_ip')

    if not 1 <= config.ecu_port <= 65535:
        raise ValueError('ecu_port must be in the range 1..65535')
    if not 0 <= config.local_bind_port <= 65535:
        raise ValueError('local_bind_port must be in the range 0..65535')
    if not math.isfinite(config.send_rate_hz) or config.send_rate_hz <= 0.0:
        raise ValueError('send_rate_hz must be finite and greater than zero')
    if (
        not math.isfinite(config.command_timeout_sec)
        or config.command_timeout_sec <= 0.0
    ):
        raise ValueError(
            'command_timeout_sec must be finite and greater than zero'
        )
    if (
        not math.isfinite(config.auto_state_timeout_sec)
        or config.auto_state_timeout_sec <= 0.0
    ):
        raise ValueError(
            'auto_state_timeout_sec must be finite and greater than zero'
        )
    if not config.steering_calibration_csv.strip():
        raise ValueError('steering_calibration_csv must not be empty')
    if (
        not math.isfinite(config.max_steering_wheel_angle_deg)
        or config.max_steering_wheel_angle_deg <= 0.0
    ):
        raise ValueError(
            'max_steering_wheel_angle_deg must be finite and greater than zero'
        )

    feedback_ip_set = bool(config.feedback_bind_ip.strip())
    feedback_port_set = config.feedback_port != 0
    if feedback_ip_set != feedback_port_set:
        raise ValueError(
            'feedback_bind_ip and feedback_port must both be set or both be unset'
        )
    if not feedback_ip_set:
        return

    _validate_ipv4(config.feedback_bind_ip.strip(), 'feedback_bind_ip')
    if config.feedback_source_ip.strip():
        _validate_ipv4(config.feedback_source_ip.strip(), 'feedback_source_ip')
    if not 1 <= config.feedback_port <= 65535:
        raise ValueError('feedback_port must be in the range 1..65535')
    if (
        not math.isfinite(config.feedback_poll_rate_hz)
        or config.feedback_poll_rate_hz <= 0.0
    ):
        raise ValueError(
            'feedback_poll_rate_hz must be finite and greater than zero'
        )
    if (
        not math.isfinite(config.feedback_timeout_sec)
        or config.feedback_timeout_sec <= 0.0
    ):
        raise ValueError(
            'feedback_timeout_sec must be finite and greater than zero'
        )
    # 0 = not known yet (feedback stays off, see feedback_ready); negative is a typo.
    if config.encoder_counts_per_revolution < 0:
        raise ValueError('encoder_counts_per_revolution must not be negative')
    if not math.isfinite(config.tire_diameter_m) or config.tire_diameter_m <= 0.0:
        raise ValueError('tire_diameter_m must be finite and greater than zero')
    if (
        not math.isfinite(config.max_wheel_speed_mps)
        or config.max_wheel_speed_mps < 0.0
    ):
        raise ValueError('max_wheel_speed_mps must be finite and non-negative')
    for name, value in (
        ('wheel_speeds_topic', config.wheel_speeds_topic),
        ('wheel_speeds_frame_id', config.wheel_speeds_frame_id),
    ):
        if not value.strip():
            raise ValueError(f'{name} must not be empty')
