#!/usr/bin/env python3
"""Generate left/right boundary CSVs from a centerline map CSV.

Ported from the HYU-Formula-Student global planner concept (centerline +
track widths -> normal vectors -> boundaries) as a standalone script:
no ROS 2, no trajectory_planning_helpers required (scipy is used when
available for smooth periodic-spline normals, with a linear fallback).

Input:  a closed-track map CSV in the TUM global race trajectory optimizer
        format (the same format main_globaltraj.py consumes):
          # x_m,y_m,w_tr_right_m,w_tr_left_m
          0.0,0.0,5.0,5.0
          ...
        Accepted layouts:
          - 4 columns x_m,y_m,w_tr_right_m,w_tr_left_m (header optional,
            '#'-comment header lines are ignored)
          - 3 columns x_m,y_m,w_tr_m (width split half/half)
        Row order must follow the track; the loop is closed automatically.

Output: <output-dir>/left_boundary.csv and <output-dir>/right_boundary.csv
        (header "x,y"), by default under
        trajectory_generator/outputs/<map_name>/.

Convention: the normal vector points to the RIGHT of the driving direction
(TUM convention), so
  right_boundary = centerline + n_right * w_tr_right
  left_boundary  = centerline - n_right * w_tr_left

Example (from the f1tenth-racing-stack-ICRA22 directory):

  python3 trajectory_generator/map_csv_to_boundaries.py \
    --input-csv /path/to/map.csv \
    --map-name my_track

  # next step:
  python3 trajectory_generator/csv_to_track_mask.py \
    --left  trajectory_generator/outputs/my_track/left_boundary.csv \
    --right trajectory_generator/outputs/my_track/right_boundary.csv \
    --map-name my_track --resolution 0.03
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np


def _warn(message: str) -> None:
    print(f"WARNING: {message}", file=sys.stderr)


def _info(message: str) -> None:
    print(f"INFO: {message}")


def load_map_csv(csv_path: Path) -> np.ndarray:
    """Load a map CSV as an Nx4 array [x, y, w_tr_right, w_tr_left]."""

    if not csv_path.exists():
        raise FileNotFoundError(f"map CSV does not exist: {csv_path}")

    rows: List[List[str]] = []
    with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        for row in csv.reader(csv_file):
            if not row or not row[0].strip() or row[0].lstrip().startswith("#"):
                continue
            rows.append(row)
    if not rows:
        raise ValueError(f"map CSV has no data rows: {csv_path}")

    def _to_floats(row: List[str]) -> Optional[List[float]]:
        try:
            return [float(value) for value in row if value.strip() != ""]
        except ValueError:
            return None

    # a non-numeric first row is treated as a header line
    if _to_floats(rows[0]) is None:
        rows = rows[1:]

    records: List[Tuple[float, float, float, float]] = []
    for line_number, row in enumerate(rows, start=1):
        values = _to_floats(row)
        if values is None or len(values) < 3:
            raise ValueError(
                f"map CSV needs numeric rows with 3 or 4 columns "
                f"(x, y, w_tr_right[, w_tr_left]); line {line_number}: {row}"
            )
        if len(values) >= 4:
            x, y, w_right, w_left = values[0], values[1], values[2], values[3]
        else:
            x, y = values[0], values[1]
            w_right = w_left = values[2] / 2.0
        if not all(math.isfinite(v) for v in (x, y, w_right, w_left)):
            raise ValueError(f"non-finite value at map CSV line {line_number}: {row}")
        if w_right <= 0.0 or w_left <= 0.0:
            raise ValueError(f"track widths must be > 0 at map CSV line {line_number}: {row}")
        records.append((x, y, w_right, w_left))

    track = np.asarray(records, dtype=float)

    # drop consecutive duplicate points and a repeated closing point
    keep = np.ones(track.shape[0], dtype=bool)
    keep[1:] = np.linalg.norm(np.diff(track[:, :2], axis=0), axis=1) > 1e-9
    dropped = int(np.count_nonzero(~keep))
    if dropped:
        _warn(f"dropped {dropped} consecutive duplicate centerline point(s)")
        track = track[keep]
    if track.shape[0] >= 2 and np.linalg.norm(track[0, :2] - track[-1, :2]) < 1e-9:
        track = track[:-1]

    if track.shape[0] < 4:
        raise ValueError(f"map CSV needs at least 4 unique centerline points; got {track.shape[0]}")
    return track


def resample_centerline_with_normals(
    track: np.ndarray,
    spacing: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Closed-loop resampling of centerline + widths, with right-pointing normals.

    Returns (points, n_right, w_right, w_left) at the sampled stations. Uses a
    periodic cubic spline when scipy is available (positions and tangents),
    otherwise linear interpolation with central-difference tangents.
    """

    closed = np.vstack((track, track[0]))
    seg_lengths = np.linalg.norm(np.diff(closed[:, :2], axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg_lengths)))
    total_length = float(s[-1])
    if spacing <= 0.0:
        raise ValueError("spacing must be > 0")
    if total_length <= spacing:
        raise ValueError(f"centerline loop length {total_length:.3f} m is too short for spacing {spacing:.3f} m")

    sample_s = np.arange(0.0, total_length, spacing, dtype=float)
    try:
        from scipy.interpolate import CubicSpline

        position_spline = CubicSpline(s, closed[:, :2], axis=0, bc_type="periodic")
        points = position_spline(sample_s)
        tangents = position_spline(sample_s, 1)
    except ImportError:
        _warn("scipy not available: using linear resampling with central-difference normals")
        points = np.column_stack(
            (np.interp(sample_s, s, closed[:, 0]), np.interp(sample_s, s, closed[:, 1]))
        )
        tangents = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)

    # widths are interpolated linearly along arc length (periodic via the closed row)
    w_right = np.interp(sample_s, s, closed[:, 2])
    w_left = np.interp(sample_s, s, closed[:, 3])

    norms = np.linalg.norm(tangents, axis=1)
    if np.any(norms <= 1e-12):
        raise ValueError("degenerate tangent on the centerline; check the input point order")
    tangents = tangents / norms[:, None]
    n_right = np.column_stack((tangents[:, 1], -tangents[:, 0]))
    return points, n_right, w_right, w_left


def save_boundary_csv(points: np.ndarray, output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["x", "y"])
        for x, y in points:
            writer.writerow([f"{x:.9f}", f"{y:.9f}"])
    return output_path


def generate_boundaries(
    input_csv: Path,
    output_dir: Path,
    spacing: float = 0.5,
) -> Tuple[Path, Path]:
    track = load_map_csv(input_csv)
    _info(f"map CSV: {input_csv} ({track.shape[0]} centerline points, "
          f"width right {track[:, 2].min():.2f}-{track[:, 2].max():.2f} m, "
          f"left {track[:, 3].min():.2f}-{track[:, 3].max():.2f} m)")

    points, n_right, w_right, w_left = resample_centerline_with_normals(track, spacing)
    right_boundary = points + n_right * w_right[:, None]
    left_boundary = points - n_right * w_left[:, None]

    # sanity check: boundaries must stay on opposite sides (no self-crossing ring)
    min_track_width = float(np.min(w_right + w_left))
    min_gap = float(np.min(np.linalg.norm(right_boundary - left_boundary, axis=1)))
    if min_gap < 0.25 * min_track_width:
        _warn(
            f"minimum left/right boundary distance is {min_gap:.3f} m "
            f"(narrowest input width {min_track_width:.3f} m); "
            "sharp corners may pinch or fold the boundary"
        )

    left_path = save_boundary_csv(left_boundary, output_dir / "left_boundary.csv")
    right_path = save_boundary_csv(right_boundary, output_dir / "right_boundary.csv")
    _info(f"saved left boundary CSV:  {left_path} ({left_boundary.shape[0]} points)")
    _info(f"saved right boundary CSV: {right_path} ({right_boundary.shape[0]} points)")
    return left_path, right_path


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate left/right boundary CSVs from a centerline map CSV "
                    "(x_m,y_m,w_tr_right_m,w_tr_left_m)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--input-csv", required=True, type=Path,
                        help="map CSV with centerline and track widths")
    parser.add_argument("--map-name", required=True,
                        help="map name; used for the default output directory")
    parser.add_argument("--spacing", type=float, default=0.5,
                        help="arc-length spacing of the boundary points in m")
    parser.add_argument("--output-dir", type=Path, default=None,
                        help="output directory (default: trajectory_generator/outputs/<map_name>)")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = Path(__file__).resolve().parent / "outputs" / args.map_name
    try:
        generate_boundaries(input_csv=args.input_csv, output_dir=output_dir, spacing=args.spacing)
        return 0
    except Exception as exc:
        print(f"ERROR: boundary generation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
