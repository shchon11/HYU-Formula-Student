# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

import math
from pathlib import Path

from drive_udp_bridge.steering_conversion import REQUIRED_COLUMNS
from drive_udp_bridge.steering_conversion import SteeringWheelConverter
import pytest


CALIBRATION_PATH = (
    Path(__file__).resolve().parents[1] / 'config' / 'steering_kinematics.csv'
)
HEADER = '\t'.join(REQUIRED_COLUMNS)


@pytest.fixture
def converter():
    return SteeringWheelConverter.from_csv(CALIBRATION_PATH, 90.0)


@pytest.mark.parametrize(
    'equivalent_angle_deg,expected_wheel_angle_deg',
    [
        (0.0, 0.0),
        (5.0093585, 24.0),
        (-5.0093585, -24.0),
        (12.63242, 60.0),
        (-12.63242, -60.0),
        (19.21249, 90.0),
        (-19.21249, -90.0),
    ],
)
def test_real_calibration_converts_known_points(
    converter,
    equivalent_angle_deg,
    expected_wheel_angle_deg,
):
    conversion = converter.convert(math.radians(equivalent_angle_deg))

    assert conversion.steering_wheel_angle_rad == pytest.approx(
        math.radians(expected_wheel_angle_deg),
        abs=1.0e-10,
    )
    assert not conversion.clipped


def test_real_calibration_interpolates_between_rows(converter):
    lower_input_deg = 5.0093585
    upper_input_deg = 5.5121585
    midpoint_input_rad = math.radians((lower_input_deg + upper_input_deg) / 2.0)

    conversion = converter.convert(midpoint_input_rad)

    assert conversion.steering_wheel_angle_rad == pytest.approx(
        math.radians(25.2)
    )
    assert not conversion.clipped


@pytest.mark.parametrize(
    'input_angle_rad,expected_output_rad',
    [
        (0.5, math.pi / 2.0),
        (-0.5, -math.pi / 2.0),
    ],
)
def test_real_calibration_clips_to_symmetric_limit(
    converter,
    input_angle_rad,
    expected_output_rad,
):
    conversion = converter.convert(input_angle_rad)

    assert conversion.steering_wheel_angle_rad == pytest.approx(
        expected_output_rad
    )
    assert conversion.clipped


def test_configured_limit_changes_input_and_output_limits():
    converter = SteeringWheelConverter.from_csv(CALIBRATION_PATH, 60.0)

    assert converter.negative_input_limit_rad == pytest.approx(
        math.radians(-12.63242)
    )
    assert converter.positive_input_limit_rad == pytest.approx(
        math.radians(12.63242)
    )
    assert converter.convert(0.5).steering_wheel_angle_rad == pytest.approx(
        math.radians(60.0)
    )


@pytest.mark.parametrize('limit', [0.0, -1.0, math.nan, math.inf, 120.1])
def test_invalid_or_unsupported_limit_is_rejected(limit):
    with pytest.raises(ValueError, match='max_steering_wheel_angle_deg'):
        SteeringWheelConverter.from_csv(CALIBRATION_PATH, limit)


@pytest.mark.parametrize('bad_value', [math.nan, math.inf, -math.inf])
def test_non_finite_input_is_rejected(converter, bad_value):
    with pytest.raises(ValueError, match='must be finite'):
        converter.convert(bad_value)


@pytest.mark.parametrize(
    'contents,error_text',
    [
        ('', 'empty'),
        ('wrong\theader\tnames\n0\t0\t0\n', 'exactly these columns'),
        (HEADER + '\n0\tbad\t0\n1\t-1\t1\n', 'non-numeric'),
        (HEADER + '\n0\tnan\t0\n1\t-1\t1\n', 'non-finite'),
        (HEADER + '\n0\t0\t0\n', 'at least two'),
        (
            HEADER + '\n0\t0\t0\n-1\t1\t-1\n',
            'steering-wheel angles',
        ),
        (
            HEADER + '\n0\t0\t0\n1\t1\t-1\n',
            'equivalent bicycle steering angles',
        ),
    ],
)
def test_malformed_calibration_is_rejected(tmp_path, contents, error_text):
    path = tmp_path / 'bad.csv'
    path.write_text(contents, encoding='utf-8')

    with pytest.raises(ValueError, match=error_text):
        SteeringWheelConverter.from_csv(path, 0.5)


def test_missing_calibration_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='cannot open'):
        SteeringWheelConverter.from_csv(tmp_path / 'missing.csv', 90.0)
