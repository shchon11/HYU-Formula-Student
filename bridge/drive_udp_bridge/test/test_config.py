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
        steering_calibration_csv='steering_kinematics.csv',
        max_steering_wheel_angle_deg=90.0,
        feedback_bind_ip='',
        feedback_port=0,
        feedback_poll_rate_hz=200.0,
        feedback_timeout_sec=0.2,
        feedback_value_type='float32',
        rpm_gear_ratio=1.0,
        tire_diameter_m=0.0,
        max_wheel_speed_mps=50.0,
        wheel_speeds_topic='/vehicle/wheel_speeds',
        wheel_speeds_frame_id='base_footprint',
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
        ('steering_calibration_csv', '', 'steering_calibration_csv'),
        (
            'max_steering_wheel_angle_deg',
            0.0,
            'max_steering_wheel_angle_deg',
        ),
        (
            'max_steering_wheel_angle_deg',
            float('nan'),
            'max_steering_wheel_angle_deg',
        ),
    ],
)
def test_invalid_config_is_rejected(valid_config, field, value, error_text):
    with pytest.raises(ValueError, match=error_text):
        validate_config(replace(valid_config, **{field: value}))


def test_feedback_parameters_must_be_set_together(valid_config):
    with pytest.raises(ValueError, match='both be set or both be unset'):
        validate_config(replace(valid_config, feedback_bind_ip='127.0.0.1'))
    with pytest.raises(ValueError, match='both be set or both be unset'):
        validate_config(replace(valid_config, feedback_port=5006))


def test_valid_feedback_config_is_accepted(valid_config):
    config = replace(
        valid_config,
        feedback_bind_ip='127.0.0.1',
        feedback_port=5006,
        tire_diameter_m=0.4572,
    )

    validate_config(config)


@pytest.mark.parametrize(
    'field,value,error_text',
    [
        ('feedback_bind_ip', 'not-an-ip', 'feedback_bind_ip'),
        ('feedback_port', 65536, 'feedback_port'),
        ('feedback_poll_rate_hz', 0.0, 'feedback_poll_rate_hz'),
        ('feedback_timeout_sec', 0.0, 'feedback_timeout_sec'),
        ('feedback_value_type', 'double', 'feedback_value_type'),
        ('feedback_value_type', '', 'feedback_value_type'),
        ('rpm_gear_ratio', 0.0, 'rpm_gear_ratio'),
        ('rpm_gear_ratio', float('nan'), 'rpm_gear_ratio'),
        ('tire_diameter_m', 0.0, 'tire_diameter_m'),
        ('max_wheel_speed_mps', -1.0, 'max_wheel_speed_mps'),
        ('wheel_speeds_topic', '', 'wheel_speeds_topic'),
        ('wheel_speeds_frame_id', '', 'wheel_speeds_frame_id'),
    ],
)
def test_invalid_feedback_config_is_rejected(
    valid_config, field, value, error_text
):
    config = replace(
        valid_config,
        feedback_bind_ip='127.0.0.1',
        feedback_port=5006,
        tire_diameter_m=0.4572,
    )

    with pytest.raises(ValueError, match=error_text):
        validate_config(replace(config, **{field: value}))


def test_feedback_source_defaults_to_ecu_ip_and_can_be_overridden(valid_config):
    enabled = replace(
        valid_config,
        feedback_bind_ip='127.0.0.1',
        feedback_port=6000,
        tire_diameter_m=0.4572,
    )
    validate_config(enabled)
    assert enabled.feedback_source_filter == '127.0.0.1'

    pinned = replace(enabled, feedback_source_ip='192.168.9.1')
    validate_config(pinned)
    assert pinned.feedback_source_filter == '192.168.9.1'

    any_source = replace(enabled, feedback_source_ip='0.0.0.0')
    validate_config(any_source)
    assert any_source.feedback_source_filter is None

    with pytest.raises(ValueError, match='feedback_source_ip'):
        validate_config(replace(enabled, feedback_source_ip='ecu'))


def test_every_feedback_value_type_validates(valid_config):
    # RPM needs no ECU-side scale, so the endpoint alone enables feedback; the
    # element type is the only per-ECU pick and every listed one must pass.
    enabled = replace(
        valid_config,
        feedback_bind_ip='0.0.0.0',
        feedback_port=5001,
        tire_diameter_m=0.4572,
    )
    for value_type in ('float32', 'float64', 'int16', 'uint16', 'int32', 'uint32'):
        validate_config(replace(enabled, feedback_value_type=value_type))
    assert enabled.feedback_enabled
