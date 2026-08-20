# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

from dataclasses import replace

from drive_udp_bridge.config import BridgeConfig, validate_config
import pytest


@pytest.fixture
def valid_config():
    return BridgeConfig(
        command_topic='/vehicle/cmd',
        auto_state_topic='/vehicle/as_state',
        ecu_ip='127.0.0.1',
        ecu_port=5005,
        local_bind_ip='0.0.0.0',
        local_bind_port=0,
        send_rate_hz=100.0,
        command_timeout_sec=0.2,
        auto_state_timeout_sec=0.5,
    )


def test_valid_config_is_accepted(valid_config):
    validate_config(valid_config)


@pytest.mark.parametrize(
    'field,value,error_text',
    [
        ('command_topic', '', 'command_topic'),
        ('auto_state_topic', '', 'auto_state_topic'),
        ('ecu_ip', '', 'ecu_ip'),
        ('ecu_ip', 'not-an-ip', 'ecu_ip'),
        ('ecu_port', 0, 'ecu_port'),
        ('ecu_port', 65536, 'ecu_port'),
        ('local_bind_ip', '', 'local_bind_ip'),
        ('local_bind_ip', '192.168.1.999', 'local_bind_ip'),
        ('local_bind_port', -1, 'local_bind_port'),
        ('local_bind_port', 65536, 'local_bind_port'),
        ('send_rate_hz', 0.0, 'send_rate_hz'),
        ('send_rate_hz', float('nan'), 'send_rate_hz'),
        ('command_timeout_sec', 0.0, 'command_timeout_sec'),
        ('command_timeout_sec', float('inf'), 'command_timeout_sec'),
        ('auto_state_timeout_sec', 0.0, 'auto_state_timeout_sec'),
        ('auto_state_timeout_sec', float('inf'), 'auto_state_timeout_sec'),
    ],
)
def test_invalid_config_is_rejected(valid_config, field, value, error_text):
    with pytest.raises(ValueError, match=error_text):
        validate_config(replace(valid_config, **{field: value}))
