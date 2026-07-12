#!/usr/bin/env python3
"""Generate a lane_generator-compatible track map PNG/YAML from boundary CSVs.

Ported from HYU-Formula-Student planning/global_planner
(boundary_track_mask_generator_node.py / boundary_centerline_utils.py) as a
standalone script: no ROS 2, no trajectory_planning_helpers required.

Input:  one of two modes.
        boundary mode (--left/--right): two CSV files with the ordered points
        of the left and right track boundaries (closed loop). Accepted layouts:
          - header with columns  x,y  (extra columns ignored)
          - header with columns  x_m,y_m
          - headerless numeric rows: first two columns are taken as x,y
        cone-map mode (--cone-csv): a single eufs_sim cone-map CSV
        (eufs_sim/eufs_tracks/csv/*.csv) with header
        `tag,x,y,direction,x_variance,y_variance,xy_covariance`. blue cones are
        taken as the left boundary and yellow cones as the right boundary, in
        file order; every other tag (orange, big_orange, car_start, midpoint)
        and the direction/variance columns are ignored.
Output: maps/<map_name>.png and maps/<map_name>.yaml (map_server-style
        metadata) that trajectory_generator/lane_generator.py can consume
        directly to produce outputs/<map_name>/centerline.csv.

Image styles (--style):
  walls (default): free-space ring = 255, outside/infield = 205 (unknown),
        boundary walls drawn as thin black (0) closed curves. This is the
        same convention as the shipped maps (levine_2nd.pgm, Spielberg_map.png)
        and is what lane_generator.py expects: it inverts the image and
        skeletonizes the walls to recover the outer/inner bounds.
  ring: pure free-space mask as produced by the original HYU generator
        (free ring = 255, everything else = 0). NOT usable by
        lane_generator.py; provided for occupancy-grid style consumers.

Example (from the planning/trajectory_generator directory):

  # cone-map mode: map-name defaults to the cone-CSV file stem (small_track)
  python3 csv_to_track_mask.py --cone-csv ../../eufs_sim/eufs_tracks/csv/small_track.csv

  # boundary mode
  python3 csv_to_track_mask.py \
    --left /path/to/left_boundary.csv \
    --right /path/to/right_boundary.csv \
    --map-name peanut \
    --resolution 0.03

  # then set map_name: 'peanut' / map_img_ext: '.png' in config/params.yaml
  python3 lane_generator.py --headless     # -> outputs/peanut/centerline.csv
  python3 main_globaltraj.py --headless    # -> outputs/peanut/traj_race_cl.csv (min-curvature raceline)
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

FREE_VALUE = 255
OCCUPIED_VALUE = 0
UNKNOWN_VALUE = 205


def _warn(message: str) -> None:
    print(f"WARNING: {message}", file=sys.stderr)


def _info(message: str) -> None:
    print(f"INFO: {message}")


# ---------------------------------------------------------------------------
# CSV loading (adapted from boundary_centerline_utils.load_boundary_points_csv)
# ---------------------------------------------------------------------------


def load_boundary_points_csv(csv_path: Path) -> np.ndarray:
    """Load one ordered closed-boundary loop from a CSV file as an Nx2 array."""

    if not csv_path.exists():
        raise FileNotFoundError(f"boundary CSV does not exist: {csv_path}")

    with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        rows = [row for row in csv.reader(csv_file) if row and row[0].strip() and not row[0].lstrip().startswith("#")]
    if not rows:
        raise ValueError(f"boundary CSV is empty: {csv_path}")

    def _is_numeric_row(row: List[str]) -> bool:
        try:
            float(row[0])
            float(row[1])
            return True
        except (IndexError, ValueError):
            return False

    if _is_numeric_row(rows[0]):
        x_idx, y_idx = 0, 1
        data_rows = rows
    else:
        header = [name.strip().lower() for name in rows[0]]
        for x_name, y_name in (("x", "y"), ("x_m", "y_m")):
            if x_name in header and y_name in header:
                x_idx, y_idx = header.index(x_name), header.index(y_name)
                break
        else:
            raise ValueError(
                f"CSV needs x/y or x_m/y_m columns (or headerless numeric rows); got {rows[0]}: {csv_path}"
            )
        data_rows = rows[1:]

    points: List[Tuple[float, float]] = []
    for line_number, row in enumerate(data_rows, start=2 if data_rows is not rows else 1):
        try:
            x = float(row[x_idx])
            y = float(row[y_idx])
        except (IndexError, ValueError) as exc:
            raise ValueError(f"invalid x/y at CSV line {line_number} of {csv_path}: {row}") from exc
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"non-finite x/y at CSV line {line_number} of {csv_path}: {row}")
        points.append((x, y))

    return np.asarray(points, dtype=float)


def load_cone_map_csv(csv_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Load an eufs_sim cone-map CSV, returning (blue=left, yellow=right) Nx2 arrays.

    The cone map has a header `tag,x,y,direction,...`. blue cones define the
    left boundary and yellow cones the right boundary, taken in file order (no
    re-ordering). Every other tag (orange, big_orange, car_start, midpoint) and
    the direction/variance columns are ignored.
    """

    if not csv_path.exists():
        raise FileNotFoundError(f"cone map CSV does not exist: {csv_path}")

    left: List[Tuple[float, float]] = []
    right: List[Tuple[float, float]] = []
    ignored: dict = {}

    with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise ValueError(f"cone map CSV has no header row: {csv_path}")
        lower_to_key = {name.strip().lower(): name for name in reader.fieldnames}
        for required in ("tag", "x", "y"):
            if required not in lower_to_key:
                raise ValueError(
                    f"cone map CSV needs tag,x,y columns; got {reader.fieldnames}: {csv_path}"
                )
        tag_key, x_key, y_key = lower_to_key["tag"], lower_to_key["x"], lower_to_key["y"]

        for line_number, row in enumerate(reader, start=2):
            tag = (row.get(tag_key) or "").strip().lower()
            if tag == "blue":
                target = left
            elif tag == "yellow":
                target = right
            else:
                ignored[tag or "<empty>"] = ignored.get(tag or "<empty>", 0) + 1
                continue
            try:
                x = float(row[x_key])
                y = float(row[y_key])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"invalid x/y at CSV line {line_number} of {csv_path}: {row}"
                ) from exc
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError(f"non-finite x/y at CSV line {line_number} of {csv_path}: {row}")
            target.append((x, y))

    if ignored:
        ignored_summary = ", ".join(f"{name}={count}" for name, count in sorted(ignored.items()))
        _info(f"cone map: ignored non-boundary cones ({ignored_summary})")

    if len(left) < 4:
        raise ValueError(f"cone map has {len(left)} blue (left) cone(s); need at least 4: {csv_path}")
    if len(right) < 4:
        raise ValueError(f"cone map has {len(right)} yellow (right) cone(s); need at least 4: {csv_path}")

    _info(f"cone map: {len(left)} blue (left) cone(s), {len(right)} yellow (right) cone(s)")
    return np.asarray(left, dtype=float), np.asarray(right, dtype=float)


def validate_boundary_points(points: np.ndarray, name: str, min_points: int = 4) -> np.ndarray:
    """Drop exact duplicates and the repeated closing point; sanity-check the loop."""

    seen = set()
    kept: List[np.ndarray] = []
    duplicates = 0
    for point in points:
        key = (float(point[0]), float(point[1]))
        if key in seen:
            duplicates += 1
            continue
        seen.add(key)
        kept.append(point)
    if duplicates > 0:
        _warn(f"{name} boundary: removed {duplicates} exact duplicate point(s)")

    cleaned = np.asarray(kept, dtype=float)
    if cleaned.shape[0] < min_points:
        raise ValueError(f"{name} boundary needs at least {min_points} unique points; got {cleaned.shape[0]}")
    return cleaned


# ---------------------------------------------------------------------------
# Closed-loop resampling (spline via scipy when available, else linear)
# ---------------------------------------------------------------------------


def resample_closed_loop(points: np.ndarray, spacing: float, name: str) -> np.ndarray:
    """Resample a closed loop at a fixed arc-length spacing.

    Uses a periodic cubic spline (scipy) so sparse cone-style boundaries are
    smoothed like the original TPH-based HYU pipeline; falls back to linear
    interpolation along the polygon when scipy is unavailable.
    """

    if spacing <= 0.0:
        raise ValueError("sample spacing must be > 0")

    closed = np.vstack((points, points[0]))
    seg_lengths = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    if np.any(seg_lengths <= 0.0):
        raise ValueError(f"{name} boundary has zero-length segments after duplicate removal")
    s = np.concatenate(([0.0], np.cumsum(seg_lengths)))
    total_length = float(s[-1])
    if total_length <= spacing:
        raise ValueError(f"{name} boundary loop length {total_length:.3f} m is too short for spacing {spacing:.3f} m")

    sample_s = np.arange(0.0, total_length, spacing, dtype=float)
    try:
        from scipy.interpolate import CubicSpline

        spline = CubicSpline(s, closed, axis=0, bc_type="periodic")
        samples = spline(sample_s)
    except ImportError:
        _warn(f"{name} boundary: scipy not available, using linear resampling (corners stay polygonal)")
        samples = np.column_stack(
            (np.interp(sample_s, s, closed[:, 0]), np.interp(sample_s, s, closed[:, 1]))
        )

    _info(f"{name} boundary: {points.shape[0]} input points, loop length {total_length:.3f} m, "
          f"{samples.shape[0]} samples at {spacing:.3f} m spacing")
    return np.asarray(samples, dtype=float)


# ---------------------------------------------------------------------------
# Outer/inner classification and geometry
# (copied/adapted from boundary_centerline_utils)
# ---------------------------------------------------------------------------


def polygon_abs_area(points: np.ndarray) -> float:
    """Absolute polygon area (shoelace formula); the loop is implicitly closed."""

    x = points[:, 0]
    y = points[:, 1]
    return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) * 0.5)


def classify_outer_inner(
    left: np.ndarray,
    right: np.ndarray,
    ambiguous_area_ratio: float = 0.9,
) -> Tuple[np.ndarray, np.ndarray, str, str]:
    """Pick the outer/inner loop purely by absolute polygon area, never by name."""

    left_area = polygon_abs_area(left)
    right_area = polygon_abs_area(right)
    if left_area >= right_area:
        outer, inner, outer_source, inner_source = left, right, "left", "right"
        outer_area, inner_area = left_area, right_area
    else:
        outer, inner, outer_source, inner_source = right, left, "right", "left"
        outer_area, inner_area = right_area, left_area

    if inner_area < 1.0e-6:
        raise ValueError(f"inner loop area {inner_area:.6f} m^2 is near zero; the boundary does not enclose a region")
    if inner_area / outer_area > ambiguous_area_ratio:
        raise ValueError(
            "cannot decide which loop is outer: "
            f"left area {left_area:.3f} m^2 vs right area {right_area:.3f} m^2"
        )

    _info(f"outer loop = {outer_source} ({outer_area:.3f} m^2), inner loop = {inner_source} ({inner_area:.3f} m^2)")
    return outer, inner, outer_source, inner_source


def fraction_of_points_inside_polygon(points: np.ndarray, polygon: np.ndarray, chunk_size: int = 1024) -> float:
    """Fraction of points strictly inside a polygon (ray casting, implicit closure)."""

    x1 = polygon[:, 0]
    y1 = polygon[:, 1]
    x2 = np.roll(x1, -1)
    y2 = np.roll(y1, -1)

    inside_count = 0
    for start in range(0, points.shape[0], chunk_size):
        px = points[start:start + chunk_size, 0][:, None]
        py = points[start:start + chunk_size, 1][:, None]
        straddles = (y1[None, :] <= py) != (y2[None, :] <= py)
        with np.errstate(divide="ignore", invalid="ignore"):
            x_at_ray = x1[None, :] + (py - y1[None, :]) * (x2 - x1)[None, :] / (y2 - y1)[None, :]
        crossings = straddles & (px < x_at_ray)
        inside_count += int(np.count_nonzero(np.count_nonzero(crossings, axis=1) % 2 == 1))
    return inside_count / points.shape[0]


def min_distance_between_loops(left: np.ndarray, right: np.ndarray, chunk_size: int = 512) -> float:
    min_distance = math.inf
    for start in range(0, left.shape[0], chunk_size):
        chunk = left[start:start + chunk_size]
        squared = ((chunk[:, None, :] - right[None, :, :]) ** 2).sum(axis=2)
        min_distance = min(min_distance, float(np.sqrt(squared.min())))
    return min_distance


def compute_mask_geometry(
    points: np.ndarray,
    resolution: float,
    margin: float,
    max_total_pixels: int = 50_000_000,
    max_edge_px: int = 20_000,
) -> Tuple[float, float, int, int]:
    """Image geometry from the bounding box of all boundary samples plus a margin."""

    if resolution <= 0.0:
        raise ValueError(f"resolution must be > 0; got {resolution}")
    if margin < 0.0:
        raise ValueError(f"margin must be >= 0; got {margin}")

    min_x, min_y = points.min(axis=0)
    max_x, max_y = points.max(axis=0)
    origin_x = float(min_x - margin)
    origin_y = float(min_y - margin)
    width_px = int(math.ceil((max_x - min_x + 2.0 * margin) / resolution))
    height_px = int(math.ceil((max_y - min_y + 2.0 * margin) / resolution))

    if width_px <= 0 or height_px <= 0:
        raise ValueError(f"mask image size is degenerate: {width_px}x{height_px} px")
    if width_px > max_edge_px or height_px > max_edge_px or width_px * height_px > max_total_pixels:
        raise ValueError(
            f"mask image {width_px}x{height_px} px is too large; "
            "increase --resolution or check the input coordinates"
        )
    return origin_x, origin_y, width_px, height_px


def world_to_pixel(
    points: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width_px: int,
    height_px: int,
) -> np.ndarray:
    """Map-frame (x, y) points to clamped integer (col, row) pixels, y flipped for image."""

    cols = np.rint((points[:, 0] - origin_x) / resolution).astype(np.int64)
    rows = np.rint((points[:, 1] - origin_y) / resolution).astype(np.int64)
    rows = height_px - 1 - rows
    cols = np.clip(cols, 0, width_px - 1)
    rows = np.clip(rows, 0, height_px - 1)
    return np.column_stack((cols, rows)).astype(np.int32)


# ---------------------------------------------------------------------------
# Rasterization
# ---------------------------------------------------------------------------


def _import_cv2():
    try:
        import cv2  # type: ignore

        return cv2
    except ImportError as exc:
        raise ImportError(
            "csv_to_track_mask.py needs OpenCV (the lane generator needs it anyway): "
            "pip install opencv-contrib-python"
        ) from exc


def rasterize_track_image(
    outer_px: np.ndarray,
    inner_px: np.ndarray,
    width_px: int,
    height_px: int,
    style: str,
    wall_thickness_px: int,
) -> np.ndarray:
    """Render the track image.

    walls: unknown background, free ring fill, thin occupied walls on both
           boundary loops (the format lane_generator.py parses).
    ring:  occupied background, free ring fill only (original HYU mask).
    """

    cv2 = _import_cv2()
    outer_poly = outer_px.reshape((-1, 1, 2))
    inner_poly = inner_px.reshape((-1, 1, 2))

    if style == "ring":
        image = np.full((height_px, width_px), OCCUPIED_VALUE, dtype=np.uint8)
        cv2.fillPoly(image, [outer_poly], FREE_VALUE)
        cv2.fillPoly(image, [inner_poly], OCCUPIED_VALUE)
        return image

    image = np.full((height_px, width_px), UNKNOWN_VALUE, dtype=np.uint8)
    cv2.fillPoly(image, [outer_poly], FREE_VALUE)
    cv2.fillPoly(image, [inner_poly], UNKNOWN_VALUE)
    cv2.polylines(image, [outer_poly], isClosed=True, color=OCCUPIED_VALUE, thickness=wall_thickness_px)
    cv2.polylines(image, [inner_poly], isClosed=True, color=OCCUPIED_VALUE, thickness=wall_thickness_px)
    return image


def save_map_yaml(
    yaml_path: Path,
    image_name: str,
    resolution: float,
    origin_x: float,
    origin_y: float,
    width_px: int,
    height_px: int,
    style: str,
) -> None:
    import yaml

    metadata = {
        "image": image_name,
        "resolution": float(resolution),
        "origin": [float(origin_x), float(origin_y), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        # extra metadata (ignored by lane_generator.py and map_server)
        "width_px": int(width_px),
        "height_px": int(height_px),
        "free_space_value": FREE_VALUE,
        "occupied_value": OCCUPIED_VALUE,
        "style": style,
    }
    with yaml_path.open("w", encoding="utf-8") as yaml_file:
        yaml.safe_dump(metadata, yaml_file, default_flow_style=None, sort_keys=False)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def build_track_mask_from_boundaries(
    left: np.ndarray,
    right: np.ndarray,
    map_name: str,
    output_dir: Path,
    resolution: float,
    margin: float,
    sample_spacing: float,
    wall_thickness_px: int,
    style: str,
) -> Tuple[Path, Path]:
    """Core mask pipeline from already-loaded/validated left & right point loops.

    Shared by generate_track_mask (boundary CSV mode) and the cone-map mode.
    """

    left_samples = resample_closed_loop(left, sample_spacing, "left")
    right_samples = resample_closed_loop(right, sample_spacing, "right")

    outer, inner, _, _ = classify_outer_inner(left_samples, right_samples)

    containment = fraction_of_points_inside_polygon(inner, outer)
    if containment < 0.95:
        raise ValueError(
            f"only {containment * 100.0:.1f}% of inner-loop points lie inside the outer loop; "
            "the left/right boundary inputs look wrong"
        )

    min_gap = min_distance_between_loops(left_samples, right_samples)
    wall_eats = wall_thickness_px * resolution
    if min_gap < 3.0 * resolution + wall_eats:
        _warn(
            f"minimum left/right boundary distance is {min_gap:.3f} m; with resolution "
            f"{resolution:.3f} m/px and {wall_thickness_px} px walls the ring may pinch shut locally"
        )

    all_samples = np.vstack((left_samples, right_samples))
    origin_x, origin_y, width_px, height_px = compute_mask_geometry(all_samples, resolution, margin)

    outer_px = world_to_pixel(outer, origin_x, origin_y, resolution, width_px, height_px)
    inner_px = world_to_pixel(inner, origin_x, origin_y, resolution, width_px, height_px)

    image = rasterize_track_image(outer_px, inner_px, width_px, height_px, style, wall_thickness_px)

    free_fraction = float(np.count_nonzero(image == FREE_VALUE)) / image.size
    if free_fraction <= 0.0:
        raise ValueError("mask has zero free-space pixels; check the inputs and --resolution")

    output_dir.mkdir(parents=True, exist_ok=True)
    png_path = output_dir / f"{map_name}.png"
    yaml_path = output_dir / f"{map_name}.yaml"

    cv2 = _import_cv2()
    if not cv2.imwrite(str(png_path), image):
        raise IOError(f"failed to write track map PNG: {png_path}")
    save_map_yaml(yaml_path, png_path.name, resolution, origin_x, origin_y, width_px, height_px, style)

    _info(f"map image: {width_px}x{height_px} px, resolution {resolution:.4f} m/px, "
          f"origin ({origin_x:.3f}, {origin_y:.3f}), free space {free_fraction * 100.0:.1f}% of image")
    _info(f"saved track map PNG:  {png_path}")
    _info(f"saved track map YAML: {yaml_path}")
    return png_path, yaml_path


def generate_track_mask(
    left_csv: Path,
    right_csv: Path,
    map_name: str,
    output_dir: Path,
    resolution: float,
    margin: float,
    sample_spacing: float,
    wall_thickness_px: int,
    style: str,
) -> Tuple[Path, Path]:
    if left_csv.resolve() == right_csv.resolve():
        raise ValueError(f"--left and --right point to the same file: {left_csv}")

    left = validate_boundary_points(load_boundary_points_csv(left_csv), "left")
    right = validate_boundary_points(load_boundary_points_csv(right_csv), "right")

    return build_track_mask_from_boundaries(
        left, right, map_name, output_dir, resolution, margin, sample_spacing, wall_thickness_px, style
    )


def generate_track_mask_from_cone_map(
    cone_csv: Path,
    map_name: str,
    output_dir: Path,
    resolution: float,
    margin: float,
    sample_spacing: float,
    wall_thickness_px: int,
    style: str,
) -> Tuple[Path, Path]:
    left_pts, right_pts = load_cone_map_csv(cone_csv)
    left = validate_boundary_points(left_pts, "blue/left")
    right = validate_boundary_points(right_pts, "yellow/right")

    return build_track_mask_from_boundaries(
        left, right, map_name, output_dir, resolution, margin, sample_spacing, wall_thickness_px, style
    )


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    default_output_dir = Path(__file__).resolve().parent / "maps"
    parser = argparse.ArgumentParser(
        description="Generate maps/<map_name>.png + .yaml from left/right boundary CSVs "
                    "for trajectory_generator/lane_generator.py",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--cone-csv", type=Path, default=None,
                        help="eufs_sim cone-map CSV (blue cones=left, yellow cones=right boundary); "
                             "mutually exclusive with --left/--right")
    parser.add_argument("--left", type=Path, default=None, help="left boundary CSV (ordered closed loop)")
    parser.add_argument("--right", type=Path, default=None, help="right boundary CSV (ordered closed loop)")
    parser.add_argument("--map-name", default=None,
                        help="output map name (config/params.yaml map_name); "
                             "defaults to the cone-CSV file stem in --cone-csv mode")
    parser.add_argument("--resolution", type=float, default=0.03, help="map resolution in m/px")
    parser.add_argument("--margin", type=float, default=1.0, help="free margin around the track bounding box in m")
    parser.add_argument("--output-dir", type=Path, default=default_output_dir, help="output directory")
    parser.add_argument("--sample-spacing", type=float, default=0.1,
                        help="arc-length spacing of the resampled boundary points in m")
    parser.add_argument("--wall-thickness-px", type=int, default=3,
                        help="boundary wall thickness in pixels (walls style)")
    parser.add_argument("--style", choices=("walls", "ring"), default="walls",
                        help="walls: lane_generator-compatible map; ring: original HYU free-space mask")
    args = parser.parse_args(argv)

    if args.cone_csv is not None:
        if args.left is not None or args.right is not None:
            parser.error("--cone-csv is mutually exclusive with --left/--right")
    else:
        if args.left is None or args.right is None:
            parser.error("provide either --cone-csv or both --left and --right")
        if args.map_name is None:
            parser.error("--map-name is required in --left/--right mode")

    return args


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    try:
        if args.cone_csv is not None:
            map_name = args.map_name or args.cone_csv.stem
            generate_track_mask_from_cone_map(
                cone_csv=args.cone_csv,
                map_name=map_name,
                output_dir=args.output_dir,
                resolution=args.resolution,
                margin=args.margin,
                sample_spacing=args.sample_spacing,
                wall_thickness_px=args.wall_thickness_px,
                style=args.style,
            )
        else:
            generate_track_mask(
                left_csv=args.left,
                right_csv=args.right,
                map_name=args.map_name,
                output_dir=args.output_dir,
                resolution=args.resolution,
                margin=args.margin,
                sample_spacing=args.sample_spacing,
                wall_thickness_px=args.wall_thickness_px,
                style=args.style,
            )
        return 0
    except Exception as exc:
        print(f"ERROR: track mask generation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
