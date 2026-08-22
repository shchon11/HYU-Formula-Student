# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

import math

from drive_udp_bridge.protocol import EncoderFeedback
from drive_udp_bridge.wheel_speeds import (
    encoder_count_delta,
    WheelSpeedConfig,
    WheelSpeedConverter,
)
import pytest


def _config(**overrides):
    values = {
        'encoder_counts_per_revolution': 1000,
        'tire_diameter_m': 1.0 / math.pi,
        'feedback_timeout_sec': 1.0,
        'max_wheel_speed_mps': 100.0,
    }
    values.update(overrides)
    return WheelSpeedConfig(**values)


def _feedback(fl, fr, rl, rr):
    return EncoderFeedback(fl, fr, rl, rr)


def test_four_encoder_deltas_become_four_independent_mps_values():
    converter = WheelSpeedConverter(_config())

    assert converter.update(_feedback(0, 0, 0, 0), 10.0) is None
    estimate = converter.update(_feedback(1000, 2000, 3000, 4000), 11.0)

    assert estimate.front_left_mps == pytest.approx(1.0)
    assert estimate.front_right_mps == pytest.approx(2.0)
    assert estimate.rear_left_mps == pytest.approx(3.0)
    assert estimate.rear_right_mps == pytest.approx(4.0)


def test_reverse_counts_produce_negative_speeds():
    converter = WheelSpeedConverter(_config())
    converter.update(_feedback(5000, 5000, 5000, 5000), 20.0)

    estimate = converter.update(_feedback(4000, 4000, 4000, 4000), 21.0)

    assert estimate.front_left_mps == pytest.approx(-1.0)
    assert estimate.front_right_mps == pytest.approx(-1.0)
    assert estimate.rear_left_mps == pytest.approx(-1.0)
    assert estimate.rear_right_mps == pytest.approx(-1.0)


def test_uint32_counter_rollover_and_reverse_are_unwrapped():
    assert encoder_count_delta(0, 0xFFFFFFFF) == 1
    assert encoder_count_delta(0xFFFFFFFF, 0) == -1


def test_timeout_rebaselines_without_reporting_gap_speed():
    converter = WheelSpeedConverter(_config(feedback_timeout_sec=0.2))
    converter.update(_feedback(0, 0, 0, 0), 30.0)

    assert converter.update(_feedback(1000, 1000, 1000, 1000), 31.0) is None
    estimate = converter.update(_feedback(1100, 1100, 1100, 1100), 31.1)

    assert estimate.front_left_mps == pytest.approx(1.0)


def test_impossible_encoder_jump_is_rejected_then_uses_new_baseline():
    converter = WheelSpeedConverter(_config(max_wheel_speed_mps=2.0))
    converter.update(_feedback(0, 0, 0, 0), 40.0)

    with pytest.raises(ValueError, match='max_wheel_speed_mps'):
        converter.update(_feedback(1000, 1000, 1000, 1000), 40.1)

    estimate = converter.update(_feedback(1100, 1100, 1100, 1100), 40.2)
    assert estimate.front_left_mps == pytest.approx(1.0)
