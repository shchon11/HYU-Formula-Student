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
