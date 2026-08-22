# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""Convert bicycle-model steering radians to steering-wheel radians."""

from bisect import bisect_right
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence


STEERING_WHEEL_COLUMN = 'steering_displacements.angle_front'
LEFT_TOE_COLUMN = 'suspension_kinematics.toe_left_front'
RIGHT_TOE_COLUMN = 'suspension_kinematics.toe_right_front'
REQUIRED_COLUMNS = (
    STEERING_WHEEL_COLUMN,
    LEFT_TOE_COLUMN,
    RIGHT_TOE_COLUMN,
)


@dataclass(frozen=True)
class SteeringConversion:
    """One converted steering command and whether it hit the configured limit."""

    steering_wheel_angle_rad: float
    clipped: bool


class SteeringWheelConverter:
    """Invert a monotonic suspension-kinematics table by linear interpolation."""

    def __init__(
        self,
        calibration_path: Path,
        equivalent_angles_rad: Sequence[float],
        steering_wheel_angles_deg: Sequence[float],
        max_steering_wheel_angle_deg: float,
    ) -> None:
        self.calibration_path = Path(calibration_path)
        self._equivalent_angles_rad = tuple(equivalent_angles_rad)
        self._steering_wheel_angles_deg = tuple(steering_wheel_angles_deg)
        self.max_steering_wheel_angle_deg = float(
            max_steering_wheel_angle_deg
        )

        if (
            not math.isfinite(self.max_steering_wheel_angle_deg)
            or self.max_steering_wheel_angle_deg <= 0.0
        ):
            raise ValueError(
                'max_steering_wheel_angle_deg must be finite and greater than zero'
            )

        minimum_wheel_angle = self._steering_wheel_angles_deg[0]
        maximum_wheel_angle = self._steering_wheel_angles_deg[-1]
        if (
            -self.max_steering_wheel_angle_deg < minimum_wheel_angle
            or self.max_steering_wheel_angle_deg > maximum_wheel_angle
        ):
            raise ValueError(
                'max_steering_wheel_angle_deg exceeds the calibration range '
                f'[{minimum_wheel_angle:.6f}, {maximum_wheel_angle:.6f}] deg'
            )

        self.negative_input_limit_rad = self._interpolate(
            -self.max_steering_wheel_angle_deg,
            self._steering_wheel_angles_deg,
            self._equivalent_angles_rad,
        )
        self.positive_input_limit_rad = self._interpolate(
            self.max_steering_wheel_angle_deg,
            self._steering_wheel_angles_deg,
            self._equivalent_angles_rad,
        )

    @classmethod
    def from_csv(
        cls,
        calibration_path: Path,
        max_steering_wheel_angle_deg: float,
    ) -> 'SteeringWheelConverter':
        """Load and validate a tab-separated steering calibration file."""
        path = Path(calibration_path)
        steering_wheel_angles_deg = []
        equivalent_angles_rad = []

        try:
            calibration_file = path.open(
                mode='r',
                encoding='utf-8-sig',
                newline='',
            )
        except OSError as error:
            raise ValueError(
                f'cannot open steering calibration CSV {path}: {error}'
            ) from error

        with calibration_file:
            reader = csv.reader(calibration_file, delimiter='\t')
            try:
                header = next(reader)
            except StopIteration as error:
                raise ValueError(
                    f'steering calibration CSV {path} is empty'
                ) from error

            if tuple(header) != REQUIRED_COLUMNS:
                raise ValueError(
                    'steering calibration CSV must contain exactly these columns: '
                    + ', '.join(REQUIRED_COLUMNS)
                )

            for line_number, row in enumerate(reader, start=2):
                if not row:
                    continue
                if len(row) != len(REQUIRED_COLUMNS):
                    raise ValueError(
                        f'steering calibration CSV line {line_number} must have '
                        f'{len(REQUIRED_COLUMNS)} columns'
                    )
                try:
                    steering_wheel_deg, left_toe_deg, right_toe_deg = (
                        float(value) for value in row
                    )
                except ValueError as error:
                    raise ValueError(
                        f'steering calibration CSV line {line_number} contains '
                        'a non-numeric value'
                    ) from error

                values = (steering_wheel_deg, left_toe_deg, right_toe_deg)
                if not all(math.isfinite(value) for value in values):
                    raise ValueError(
                        f'steering calibration CSV line {line_number} contains '
                        'a non-finite value'
                    )

                equivalent_angle_deg = (right_toe_deg - left_toe_deg) / 2.0
                steering_wheel_angles_deg.append(steering_wheel_deg)
                equivalent_angles_rad.append(math.radians(equivalent_angle_deg))

        if len(steering_wheel_angles_deg) < 2:
            raise ValueError(
                'steering calibration CSV must contain at least two data rows'
            )

        cls._validate_strictly_increasing(
            steering_wheel_angles_deg,
            'steering-wheel angles',
        )
        cls._validate_strictly_increasing(
            equivalent_angles_rad,
            'equivalent bicycle steering angles',
        )

        return cls(
            calibration_path=path,
            equivalent_angles_rad=equivalent_angles_rad,
            steering_wheel_angles_deg=steering_wheel_angles_deg,
            max_steering_wheel_angle_deg=max_steering_wheel_angle_deg,
        )

    @staticmethod
    def _validate_strictly_increasing(
        values: Sequence[float],
        description: str,
    ) -> None:
        for previous, current in zip(values, values[1:]):
            if current <= previous:
                raise ValueError(
                    f'steering calibration {description} must be strictly increasing'
                )

    @staticmethod
    def _interpolate(
        value: float,
        independent_values: Sequence[float],
        dependent_values: Sequence[float],
    ) -> float:
        if value <= independent_values[0]:
            return dependent_values[0]
        if value >= independent_values[-1]:
            return dependent_values[-1]

        upper_index = bisect_right(independent_values, value)
        lower_index = upper_index - 1
        lower_x = independent_values[lower_index]
        upper_x = independent_values[upper_index]
        fraction = (value - lower_x) / (upper_x - lower_x)
        return dependent_values[lower_index] + fraction * (
            dependent_values[upper_index] - dependent_values[lower_index]
        )

    @property
    def calibration_minimum_input_rad(self) -> float:
        """Return the lowest equivalent bicycle angle represented by the CSV."""
        return self._equivalent_angles_rad[0]

    @property
    def calibration_maximum_input_rad(self) -> float:
        """Return the highest equivalent bicycle angle represented by the CSV."""
        return self._equivalent_angles_rad[-1]

    @property
    def calibration_minimum_wheel_angle_deg(self) -> float:
        """Return the lowest steering-wheel angle represented by the CSV."""
        return self._steering_wheel_angles_deg[0]

    @property
    def calibration_maximum_wheel_angle_deg(self) -> float:
        """Return the highest steering-wheel angle represented by the CSV."""
        return self._steering_wheel_angles_deg[-1]

    @property
    def maximum_output_rad(self) -> float:
        """Return the configured symmetric UDP steering-wheel limit in radians."""
        return math.radians(self.max_steering_wheel_angle_deg)

    def convert(self, equivalent_steering_angle_rad: float) -> SteeringConversion:
        """Convert one bicycle-model angle in radians to a limited wheel angle."""
        if not math.isfinite(equivalent_steering_angle_rad):
            raise ValueError('equivalent steering angle must be finite')

        limited_input_rad = min(
            self.positive_input_limit_rad,
            max(self.negative_input_limit_rad, equivalent_steering_angle_rad),
        )
        clipped = limited_input_rad != equivalent_steering_angle_rad
        steering_wheel_angle_deg = self._interpolate(
            limited_input_rad,
            self._equivalent_angles_rad,
            self._steering_wheel_angles_deg,
        )
        steering_wheel_angle_deg = min(
            self.max_steering_wheel_angle_deg,
            max(-self.max_steering_wheel_angle_deg, steering_wheel_angle_deg),
        )
        return SteeringConversion(
            steering_wheel_angle_rad=math.radians(steering_wheel_angle_deg),
            clipped=clipped,
        )
