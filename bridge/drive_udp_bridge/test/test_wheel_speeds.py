# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

import math

from drive_udp_bridge.protocol import EncoderFeedback
from drive_udp_bridge.wheel_speeds import WheelSpeedConfig, WheelSpeedConverter
import pytest


def _config(**overrides):
    values = {
        # pi * D = 1 m per revolution -> 60 RPM = 1 m/s at gear ratio 1.
        'tire_diameter_m': 1.0 / math.pi,
        'rpm_gear_ratio': 1.0,
        'max_wheel_speed_mps': 100.0,
    }
    values.update(overrides)
    return WheelSpeedConfig(**values)


def _feedback(fl, fr, rl, rr):
    return EncoderFeedback(fl, fr, rl, rr)


def test_four_rpm_values_become_four_independent_mps_values():
    converter = WheelSpeedConverter(_config())

    estimate = converter.convert(_feedback(60.0, 120.0, 180.0, 240.0))

    assert converter.mps_per_rpm == pytest.approx(1.0 / 60.0)
    assert estimate.front_left_mps == pytest.approx(1.0)
    assert estimate.front_right_mps == pytest.approx(2.0)
    assert estimate.rear_left_mps == pytest.approx(3.0)
    assert estimate.rear_right_mps == pytest.approx(4.0)


def test_negative_rpm_produces_negative_speeds():
    converter = WheelSpeedConverter(_config())

    estimate = converter.convert(_feedback(-60.0, -60.0, -60.0, -60.0))

    assert estimate.front_left_mps == pytest.approx(-1.0)
    assert estimate.rear_right_mps == pytest.approx(-1.0)


def test_gear_ratio_divides_motor_rpm_down_to_wheel_speed():
    converter = WheelSpeedConverter(_config(rpm_gear_ratio=4.0))

    estimate = converter.convert(_feedback(240.0, 240.0, 240.0, 240.0))

    assert estimate.front_left_mps == pytest.approx(1.0)


def test_real_tire_scale_matches_pi_d_over_60():
    converter = WheelSpeedConverter(_config(tire_diameter_m=0.4572))

    assert converter.mps_per_rpm == pytest.approx(math.pi * 0.4572 / 60.0)


def test_impossible_rpm_is_rejected_without_state():
    converter = WheelSpeedConverter(_config(max_wheel_speed_mps=2.0))

    with pytest.raises(ValueError, match='max_wheel_speed_mps'):
        converter.convert(_feedback(600.0, 0.0, 0.0, 0.0))

    # Stateless: the next plausible sample converts normally.
    estimate = converter.convert(_feedback(60.0, 60.0, 60.0, 60.0))
    assert estimate.front_left_mps == pytest.approx(1.0)


@pytest.mark.parametrize(
    'field,value,error_text',
    [
        ('tire_diameter_m', 0.0, 'tire_diameter_m'),
        ('tire_diameter_m', float('nan'), 'tire_diameter_m'),
        ('rpm_gear_ratio', 0.0, 'rpm_gear_ratio'),
        ('rpm_gear_ratio', float('inf'), 'rpm_gear_ratio'),
        ('max_wheel_speed_mps', -1.0, 'max_wheel_speed_mps'),
    ],
)
def test_invalid_scale_is_rejected(field, value, error_text):
    with pytest.raises(ValueError, match=error_text):
        WheelSpeedConverter(_config(**{field: value}))
