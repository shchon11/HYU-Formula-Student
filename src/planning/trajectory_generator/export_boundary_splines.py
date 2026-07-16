#!/usr/bin/env python3
"""Export dense periodic-cubic-spline boundary CSVs for the RViz debug visualizer.

Reuses csv_to_track_mask.resample_closed_loop (the SAME arc-length periodic
CubicSpline used to build the track map PNG), so the boundaries drawn in RViz
match the map exactly instead of being polygonal cone-to-cone segments.

Formula Student convention: blue cones = LEFT boundary, yellow cones = RIGHT.
Outputs (header "x_m,y_m") under outputs/<map_name>/:
  left_boundary_spline.csv   (blue  cones, dense spline)  -> RViz blue
  right_boundary_spline.csv  (yellow cones, dense spline) -> RViz yellow

Example (from the trajectory_generator directory):
  python3 export_boundary_splines.py --cone-csv ../../eufs_sim/eufs_tracks/csv/peanut.csv --map-name peanut
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, List

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from csv_to_track_mask import (  # noqa: E402
    load_cone_map_csv,
    load_boundary_points_csv,
    validate_boundary_points,
    resample_closed_loop,
)

REPO_ROOT = HERE.parent.parent


def write_xy(path: Path, points: np.ndarray, label: str) -> None:
    with path.open("w", encoding="utf-8") as csv_file:
        csv_file.write("x_m,y_m\n")
        for x, y in points:
            csv_file.write(f"{x:.6f},{y:.6f}\n")
    print(f"INFO: wrote {path}  ({points.shape[0]} pts, {label})")


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export dense periodic-spline left/right boundary CSVs for RViz.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    src = parser.add_mutually_exclusive_group()
    src.add_argument("--cone-csv", type=Path, default=None,
                     help="eufs_sim cone map CSV (blue=left, yellow=right)")
    parser.add_argument("--left-csv", type=Path, default=None,
                        help="explicit left (blue) boundary CSV; overrides --cone-csv left")
    parser.add_argument("--right-csv", type=Path, default=None,
                        help="explicit right (yellow) boundary CSV; overrides --cone-csv right")
    parser.add_argument("--map-name", default="peanut", help="output map name (outputs/<map_name>/)")
    parser.add_argument("--spacing", type=float, default=0.1,
                        help="arc-length spacing of the spline samples in m (match the map: 0.1)")
    parser.add_argument("--output-root", type=Path, default=HERE / "outputs",
                        help="root of the per-map output directory")
    args = parser.parse_args(argv)
    if args.cone_csv is None and (args.left_csv is None or args.right_csv is None):
        args.cone_csv = REPO_ROOT / "eufs_sim" / "eufs_tracks" / "csv" / f"{args.map_name}.csv"
    return args


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    if args.left_csv is not None and args.right_csv is not None:
        left = validate_boundary_points(load_boundary_points_csv(args.left_csv), "left")
        right = validate_boundary_points(load_boundary_points_csv(args.right_csv), "right")
    else:
        if not args.cone_csv.exists():
            print(f"ERROR: cone map CSV not found: {args.cone_csv}", file=sys.stderr)
            return 1
        left_pts, right_pts = load_cone_map_csv(args.cone_csv)  # blue=left, yellow=right
        left = validate_boundary_points(left_pts, "blue/left")
        right = validate_boundary_points(right_pts, "yellow/right")

    left_samples = resample_closed_loop(left, args.spacing, "blue/left")
    right_samples = resample_closed_loop(right, args.spacing, "yellow/right")

    out_dir = args.output_root / args.map_name
    out_dir.mkdir(parents=True, exist_ok=True)
    write_xy(out_dir / "left_boundary_spline.csv", left_samples, "blue/left")
    write_xy(out_dir / "right_boundary_spline.csv", right_samples, "yellow/right")
    return 0


if __name__ == "__main__":
    sys.exit(main())
