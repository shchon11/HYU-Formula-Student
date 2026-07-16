#!/usr/bin/env python3
"""One-shot global planning pipeline: map CSV -> centerline -> min-curvature raceline.

Chains the individual steps of the planning/trajectory_generator pipeline
(each of which can also be run by hand from this directory):

  1. map_csv_to_boundaries.py   map CSV (x_m,y_m,w_tr_right_m,w_tr_left_m)
                                -> outputs/<map_name>/left_boundary.csv
                                -> outputs/<map_name>/right_boundary.csv
  2. csv_to_track_mask.py       boundary CSVs
                                -> maps/<map_name>.png + .yaml
  3. config/params.yaml         map_name / map_img_ext updated in-place
  4. lane_generator.py          -> outputs/<map_name>/centerline.csv, lane_*.csv
  5. main_globaltraj.py         (opt_type mincurv_iqp)
                                -> outputs/<map_name>/traj_race_cl.csv

Runs headless by default (no GUI windows, no key presses); pass --show-plots
to get the interactive cv2/matplotlib previews of the original scripts.

Runs on the stock system python3: trajectory_planning_helpers is vendored in
this directory and quadprog comes from pip (see requirements.txt). Override
the interpreter for main_globaltraj.py with --globaltraj-python if needed.

Example (from the planning/trajectory_generator directory):

  python3 map_csv_to_icra_global_plan.py \
    --input-csv inputs/tracks/rounded_rectangle.csv \
    --map-name rounded_rectangle \
    --resolution 0.1
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import List, Optional

from csv_to_track_mask import generate_track_mask
from map_csv_to_boundaries import generate_boundaries

SCRIPT_DIR = Path(__file__).resolve().parent      # .../planning/trajectory_generator
STEPS = ("boundaries", "mask", "config", "centerline", "raceline")


def _info(message: str) -> None:
    print(f"INFO: [pipeline] {message}")


def update_config(map_name: str, map_img_ext: str = ".png") -> None:
    """Set map_name/map_img_ext in config/params.yaml, keeping comments."""

    config_path = SCRIPT_DIR / "config" / "params.yaml"
    text = config_path.read_text(encoding="utf-8")
    new_text, n_name = re.subn(r"(?m)^map_name:.*$", f"map_name: '{map_name}'", text)
    new_text, n_ext = re.subn(r"(?m)^map_img_ext:.*$", f"map_img_ext: '{map_img_ext}'", new_text)
    if n_name != 1 or n_ext != 1:
        raise ValueError(f"could not update map_name/map_img_ext in {config_path}")
    config_path.write_text(new_text, encoding="utf-8")
    _info(f"config updated: map_name='{map_name}' map_img_ext='{map_img_ext}' ({config_path})")


def run_step(command: List[str], step_name: str, headless: bool) -> None:
    env = os.environ.copy()
    if headless:
        env["MPLBACKEND"] = "Agg"
    _info(f"running step '{step_name}': {' '.join(command)}")
    result = subprocess.run(command, cwd=SCRIPT_DIR, env=env)
    if result.returncode != 0:
        raise RuntimeError(f"step '{step_name}' failed with exit code {result.returncode}")


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="map CSV -> boundaries -> track map -> centerline -> min-curvature raceline",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--input-csv", required=True, type=Path,
                        help="map CSV with centerline and track widths (x_m,y_m,w_tr_right_m,w_tr_left_m)")
    parser.add_argument("--map-name", required=True, help="name for the generated map and output folder")
    parser.add_argument("--resolution", type=float, default=0.03, help="map resolution in m/px")
    parser.add_argument("--margin", type=float, default=1.0, help="free margin around the track in m")
    parser.add_argument("--boundary-spacing", type=float, default=0.5,
                        help="spacing of the generated boundary points in m")
    parser.add_argument("--sample-spacing", type=float, default=0.1,
                        help="boundary resampling spacing used for rasterization in m")
    parser.add_argument("--wall-thickness-px", type=int, default=3, help="boundary wall thickness in px")
    parser.add_argument("--until", choices=STEPS, default="raceline",
                        help="stop after this step (raceline = full pipeline)")
    parser.add_argument("--globaltraj-python", type=Path, default=Path(sys.executable),
                        help="python interpreter for main_globaltraj.py (needs quadprog; "
                             "trajectory_planning_helpers is vendored here)")
    parser.add_argument("--show-plots", action="store_true",
                        help="debug mode: show the interactive cv2/matplotlib windows "
                             "of lane_generator.py and main_globaltraj.py")
    parser.add_argument("--no-config-update", action="store_true",
                        help="do not rewrite map_name/map_img_ext in config/params.yaml")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    headless = not args.show_plots
    until_index = STEPS.index(args.until)
    outputs_dir = SCRIPT_DIR / "outputs" / args.map_name

    try:
        # 1. map CSV -> left/right boundary CSVs
        left_csv, right_csv = generate_boundaries(
            input_csv=args.input_csv,
            output_dir=outputs_dir,
            spacing=args.boundary_spacing,
        )
        if until_index < STEPS.index("mask"):
            return 0

        # 2. boundary CSVs -> maps/<map_name>.png + .yaml
        png_path, yaml_path = generate_track_mask(
            left_csv=left_csv,
            right_csv=right_csv,
            map_name=args.map_name,
            output_dir=SCRIPT_DIR / "maps",
            resolution=args.resolution,
            margin=args.margin,
            sample_spacing=args.sample_spacing,
            wall_thickness_px=args.wall_thickness_px,
            style="walls",
        )
        if until_index < STEPS.index("config"):
            return 0

        # 3. point config/params.yaml at the new map
        if not args.no_config_update:
            update_config(args.map_name, png_path.suffix)
        if until_index < STEPS.index("centerline"):
            return 0

        # 4. map -> centerline.csv (+ lanes)
        lane_generator_cmd = [sys.executable, str(SCRIPT_DIR / "lane_generator.py")]
        if headless:
            lane_generator_cmd.append("--headless")
        run_step(lane_generator_cmd, "centerline (lane_generator.py)", headless)
        centerline_csv = outputs_dir / "centerline.csv"
        if not centerline_csv.exists():
            raise RuntimeError(f"lane_generator.py finished but {centerline_csv} is missing")
        _info(f"centerline: {centerline_csv}")
        if until_index < STEPS.index("raceline"):
            return 0

        # 5. centerline.csv -> traj_race_cl.csv (mincurv_iqp)
        globaltraj_cmd = [str(args.globaltraj_python), str(SCRIPT_DIR / "main_globaltraj.py")]
        if headless:
            globaltraj_cmd.append("--headless")
        run_step(globaltraj_cmd, "raceline (main_globaltraj.py)", headless)
        output_path = outputs_dir / "traj_race_cl.csv"
        if not output_path.exists():
            raise RuntimeError(f"main_globaltraj.py finished but {output_path} is missing")
        _info(f"raceline output: {output_path}")

        _info("pipeline finished")
        return 0

    except Exception as exc:
        print(f"ERROR: pipeline failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
