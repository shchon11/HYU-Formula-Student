#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
# --- How to run ---
# python3 src/planning/global_planner/scripts/map_geometry_diagnostics.py \
#   --min-track-width 2.0 --max-track-width 6.0 \
#   --glob 'src/eufs_graph_slam/map/map_20260713_*.csv' \
#   --evidence src/.omo/evidence/slam-planner-pipeline-recovery/task-2/maps.json

from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Final


SIDE_TAGS: Final[frozenset[str]] = frozenset({"blue", "yellow"})
GEOMETRY_EPSILON: Final[float] = 1.0e-9
DEFAULT_MIN_TRACK_WIDTH_M: Final[float] = 2.0
DEFAULT_MAX_TRACK_WIDTH_M: Final[float] = 6.0
DEFAULT_DUPLICATE_DISTANCE_M: Final[float] = 0.5


@dataclass(frozen=True, slots=True)
class Point:
    x: float
    y: float


@dataclass(frozen=True, slots=True)
class MapDiagnostic:
    filename: str
    path: str
    sha256: str
    row_count: int
    blue_count: int
    yellow_count: int
    min_same_color_distance: float | None
    min_same_color_distance_m: float | None
    same_color_duplicate_count: int
    duplicate_same_color_pairs: int
    min_width: float | None
    min_width_m: float | None
    min_track_width_m: float | None
    max_width: float | None
    max_width_m: float | None
    invalid_width_count: int
    branch_jump_count: int
    branch_jumps: int
    self_intersection_count: int
    self_intersections: int
    blue_loop_gap_m: float | None
    yellow_loop_gap_m: float | None
    centerline_loop_gap_m: float | None
    closed_loop_ok: bool
    verdict: str
    reasons: list[str]


@dataclass(frozen=True, slots=True)
class Thresholds:
    min_track_width_m: float
    max_track_width_m: float
    duplicate_distance_m: float
    max_boundary_gap_m: float
    close_loop_distance_m: float


class DiagnosticError(Exception): pass


def distance(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def subtract(a: Point, b: Point) -> Point:
    return Point(a.x - b.x, a.y - b.y)


def dot(a: Point, b: Point) -> float:
    return a.x * b.x + a.y * b.y


def csv_points(path: Path) -> tuple[list[Point], list[Point], int]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            msg = f"{path}: missing CSV header"
            raise DiagnosticError(msg)
        required = {"tag", "x", "y"}
        missing = required.difference(reader.fieldnames)
        if missing:
            msg = f"{path}: missing CSV columns {sorted(missing)}"
            raise DiagnosticError(msg)
        blue: list[Point] = []
        yellow: list[Point] = []
        rows = 0
        for line_number, row in enumerate(reader, start=2):
            rows += 1
            tag = (row.get("tag") or "").strip()
            if tag not in SIDE_TAGS:
                continue
            try:
                point = Point(float(row["x"]), float(row["y"]))
            except ValueError as exc:
                msg = f"{path}:{line_number}: bad numeric value"
                raise DiagnosticError(msg) from exc
            if not math.isfinite(point.x) or not math.isfinite(point.y):
                msg = f"{path}:{line_number}: non-finite point"
                raise DiagnosticError(msg)
            if tag == "blue":
                blue.append(point)
            else:
                yellow.append(point)
    return blue, yellow, rows


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def min_same_color_distance(blue: list[Point], yellow: list[Point]) -> float | None:
    best: float | None = None
    for points in (blue, yellow):
        for i, first in enumerate(points):
            for second in points[i + 1 :]:
                current = distance(first, second)
                if best is None or current < best:
                    best = current
    return best


def nearest_widths(blue: list[Point], yellow: list[Point], thresholds: Thresholds) -> tuple[float | None, float | None, int]:
    widths: list[float] = []
    for points, opposite in ((blue, yellow), (yellow, blue)):
        for point in points:
            if opposite:
                widths.append(min(distance(point, candidate) for candidate in opposite))
    invalid = sum(
        1
        for width in widths
        if width < thresholds.min_track_width_m or width > thresholds.max_track_width_m
    )
    return min(widths) if widths else None, max(widths) if widths else None, invalid


def nearest_neighbor_order(points: list[Point]) -> list[Point]:
    if not points:
        return []
    unused = list(points)
    current = min(unused, key=lambda point: distance(point, Point(0.0, 0.0)))
    ordered = [current]
    unused.remove(current)
    while unused:
        current = min(unused, key=lambda point: distance(ordered[-1], point))
        ordered.append(current)
        unused.remove(current)
    return ordered


def turn_angle(previous: Point, current: Point, candidate: Point) -> float:
    incoming = subtract(current, previous)
    outgoing = subtract(candidate, current)
    incoming_length = math.hypot(incoming.x, incoming.y)
    outgoing_length = math.hypot(outgoing.x, outgoing.y)
    if incoming_length <= GEOMETRY_EPSILON or outgoing_length <= GEOMETRY_EPSILON:
        return 0.0
    cosine = max(-1.0, min(1.0, dot(incoming, outgoing) / (incoming_length * outgoing_length)))
    return math.acos(cosine)


def heading_aware_order(points: list[Point], thresholds: Thresholds) -> list[Point] | None:
    if not points:
        return []
    used = [False] * len(points)
    current = min(range(len(points)), key=lambda index: distance(points[index], Point(0.0, 0.0)))
    used[current] = True
    ordered = [points[current]]
    while len(ordered) < len(points):
        best_index: int | None = None
        best_score = math.inf
        for index, point in enumerate(points):
            candidate_distance = distance(points[current], point)
            if used[index] or candidate_distance > thresholds.max_boundary_gap_m:
                continue
            score = candidate_distance
            if len(ordered) >= 2:
                score += thresholds.max_boundary_gap_m * turn_angle(ordered[-2], ordered[-1], point)
            if score < best_score:
                best_index = index
                best_score = score
        if best_index is None:
            return None
        used[best_index] = True
        ordered.append(points[best_index])
        current = best_index
    return ordered


def branch_jumps(ordered: list[Point], thresholds: Thresholds) -> int:
    segments = [distance(ordered[i - 1], ordered[i]) for i in range(1, len(ordered))]
    if not segments:
        return 0
    sorted_segments = sorted(segments)
    median = sorted_segments[len(sorted_segments) // 2]
    jump_limit = min(thresholds.max_boundary_gap_m, max(median * 2.5, thresholds.min_track_width_m))
    return sum(1 for segment in segments if segment > jump_limit)


def max_segment_gap(ordered: list[Point]) -> float:
    if len(ordered) < 2:
        return 0.0
    return max(distance(ordered[i - 1], ordered[i]) for i in range(1, len(ordered)))


def topology_order(points: list[Point], thresholds: Thresholds) -> tuple[list[Point], list[str]]:
    ordered = heading_aware_order(points, thresholds)
    if ordered is None:
        return nearest_neighbor_order(points), ["branch_jump"]
    reasons: list[str] = []
    if self_intersections(ordered) > 0:
        reasons.append("self_intersection")
    if (
        max_segment_gap(ordered) > thresholds.max_boundary_gap_m
        or (len(ordered) >= 2 and distance(ordered[0], ordered[-1]) > thresholds.max_boundary_gap_m)
    ):
        reasons.append("branch_jump")
    return ordered, reasons


def orientation(a: Point, b: Point, c: Point) -> float:
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)


def segments_cross(a: Point, b: Point, c: Point, d: Point) -> bool:
    ab_c = orientation(a, b, c)
    ab_d = orientation(a, b, d)
    cd_a = orientation(c, d, a)
    cd_b = orientation(c, d, b)
    return ab_c * ab_d < -GEOMETRY_EPSILON and cd_a * cd_b < -GEOMETRY_EPSILON


def self_intersections(ordered: list[Point]) -> int:
    count = 0
    for i in range(len(ordered) - 1):
        for j in range(i + 2, len(ordered) - 1):
            if segments_cross(ordered[i], ordered[i + 1], ordered[j], ordered[j + 1]):
                count += 1
    return count


def continuity(blue: list[Point], yellow: list[Point], thresholds: Thresholds) -> tuple[float | None, float | None, float | None, bool]:
    blue_gap = distance(blue[0], blue[-1]) if len(blue) >= 2 else None
    yellow_gap = distance(yellow[0], yellow[-1]) if len(yellow) >= 2 else None
    center_gap = None
    if len(blue) >= 2 and len(yellow) >= 2:
        center_gap = distance(
            Point((blue[0].x + yellow[0].x) * 0.5, (blue[0].y + yellow[0].y) * 0.5),
            Point((blue[-1].x + yellow[-1].x) * 0.5, (blue[-1].y + yellow[-1].y) * 0.5),
        )
    gaps = [gap for gap in (blue_gap, yellow_gap, center_gap) if gap is not None]
    return blue_gap, yellow_gap, center_gap, bool(gaps) and max(gaps) <= thresholds.close_loop_distance_m


def diagnose_map(path: Path, thresholds: Thresholds) -> MapDiagnostic:
    blue, yellow, rows = csv_points(path)
    ordered_blue, blue_topology_reasons = topology_order(blue, thresholds)
    ordered_yellow, yellow_topology_reasons = topology_order(yellow, thresholds)
    same_color_min = min_same_color_distance(blue, yellow)
    min_width, max_width, invalid_width_count = nearest_widths(blue, yellow, thresholds)
    intersections = self_intersections(ordered_blue) + self_intersections(ordered_yellow)
    jumps = branch_jumps(ordered_blue, thresholds) + branch_jumps(ordered_yellow, thresholds)
    loop = continuity(ordered_blue, ordered_yellow, thresholds)
    reasons: list[str] = []
    if len(blue) < 3 or len(yellow) < 3:
        reasons.append("insufficient_blue_or_yellow_cones")
    if same_color_min is not None and same_color_min < thresholds.duplicate_distance_m:
        reasons.append("duplicate_ghost")
    if invalid_width_count > 0:
        reasons.append("invalid_width")
    if "branch_jump" in blue_topology_reasons or "branch_jump" in yellow_topology_reasons or jumps > 0:
        reasons.append("branch_jump")
    if "self_intersection" in blue_topology_reasons or "self_intersection" in yellow_topology_reasons or intersections > 0:
        reasons.append("self_intersection")
    return MapDiagnostic(
        filename=path.name, path=str(path), sha256=sha256_file(path), row_count=rows,
        blue_count=len(blue), yellow_count=len(yellow),
        min_same_color_distance=same_color_min, min_same_color_distance_m=same_color_min,
        same_color_duplicate_count=1 if (
            same_color_min is not None and same_color_min < thresholds.duplicate_distance_m
        ) else 0,
        duplicate_same_color_pairs=1 if (
            same_color_min is not None and same_color_min < thresholds.duplicate_distance_m
        ) else 0,
        min_width=min_width, min_width_m=min_width, min_track_width_m=min_width,
        max_width=max_width, max_width_m=max_width,
        invalid_width_count=invalid_width_count,
        branch_jump_count=jumps, branch_jumps=jumps,
        self_intersection_count=intersections, self_intersections=intersections,
        blue_loop_gap_m=loop[0], yellow_loop_gap_m=loop[1], centerline_loop_gap_m=loop[2],
        closed_loop_ok=loop[3], verdict="invalid" if reasons else "valid", reasons=reasons,
    )


def resolve_inputs(paths: list[str], patterns: list[str]) -> list[Path]:
    resolved = [Path(path) for path in paths]
    for pattern in patterns:
        matches = sorted(Path(match) for match in glob.glob(pattern))
        if not matches:
            msg = f"glob matched no files: {pattern}"
            raise DiagnosticError(msg)
        resolved.extend(matches)
    if not resolved:
        msg = "no input map files provided"
        raise DiagnosticError(msg)
    unique = sorted({path for path in resolved}, key=lambda path: str(path))
    missing = [str(path) for path in unique if not path.is_file()]
    if missing:
        msg = f"missing input map files: {missing}"
        raise DiagnosticError(msg)
    return unique


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Diagnose saved SLAM map CSV geometry.")
    parser.add_argument("maps", nargs="*", help="Saved map CSV files.")
    parser.add_argument("--glob", action="append", default=[], help="Glob for saved map CSV files.")
    parser.add_argument("--evidence", help="Write JSON report to this path.")
    parser.add_argument("--min-track-width", type=float, default=DEFAULT_MIN_TRACK_WIDTH_M)
    parser.add_argument("--max-track-width", type=float, default=DEFAULT_MAX_TRACK_WIDTH_M)
    parser.add_argument("--same-color-duplicate-distance", type=float, default=DEFAULT_DUPLICATE_DISTANCE_M)
    parser.add_argument("--max-boundary-gap", type=float, default=12.0)
    parser.add_argument("--close-loop-distance", type=float, default=5.0)
    parser.add_argument("--expect-valid", action="store_true")
    parser.add_argument("--expect-valid-postfix", action="store_true")
    parser.add_argument("--expect-invalid-width", action="store_true")
    return parser


def run(argv: list[str]) -> int:
    args = build_parser().parse_args(argv)
    thresholds = Thresholds(
        args.min_track_width, args.max_track_width, args.same_color_duplicate_distance,
        args.max_boundary_gap, args.close_loop_distance,
    )
    paths = resolve_inputs(args.maps, args.glob)
    maps = [diagnose_map(path, thresholds) for path in paths]
    payload = json.dumps({"schema_version": 1, "thresholds": asdict(thresholds), "maps": [asdict(item) for item in maps]}, indent=2, sort_keys=True) + "\n"
    if args.evidence:
        evidence_path = Path(args.evidence)
        evidence_path.parent.mkdir(parents=True, exist_ok=True)
        evidence_path.write_text(payload, encoding="utf-8")
    else:
        sys.stdout.write(payload)
    if args.expect_invalid_width:
        has_invalid_width = any(
            "invalid_width" in item.reasons
            for item in maps
        )
        if not has_invalid_width:
            sys.stderr.write("expected invalid width below minimum, but none was found\n")
            return 1
    if args.expect_valid:
        invalid_maps = [item.filename for item in maps if item.verdict != "valid"]
        if invalid_maps:
            sys.stderr.write(f"expected valid map geometry, but invalid maps were found: {invalid_maps}\n")
            return 1
    if args.expect_valid_postfix:
        invalid_maps = [
            item.filename
            for item in maps
            if (
                item.verdict != "valid"
                or item.min_track_width_m is None
                or item.min_track_width_m < thresholds.min_track_width_m
                or item.self_intersections != 0
                or item.duplicate_same_color_pairs != 0
                or item.branch_jumps != 0
            )
        ]
        if invalid_maps:
            sys.stderr.write(
                f"expected post-fix valid map geometry, but invalid maps were found: {invalid_maps}\n"
            )
            return 1
    return 0


def main() -> int:
    try:
        return run(sys.argv[1:])
    except DiagnosticError as exc:
        sys.stderr.write(f"map_geometry_diagnostics: {exc}\n")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
