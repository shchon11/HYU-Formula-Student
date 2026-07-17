from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path


SRC_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = SRC_ROOT / "planning/hyu_global_planner/scripts/map_geometry_diagnostics.py"
BAD_WIDTH_MAP = SRC_ROOT / "localization/map/map_20260713_002645.csv"


def run_diagnostic(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["python3", str(SCRIPT), "--min-track-width", "2.0", "--max-track-width", "6.0", *args],
        cwd=SRC_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


def write_map(path: Path, blue_x: float) -> None:
    path.write_text(
        "\n".join(
            [
                "tag,x,y,direction,x_variance,y_variance,xy_covariance",
                "car_start,0,0,0,0,0,0",
                f"blue,{blue_x},0,0,0,0,0",
                f"blue,{blue_x + 4},0,0,0,0,0",
                f"blue,{blue_x + 8},0,0,0,0,0",
                "yellow,0,3,0,0,0,0",
                "yellow,4,3,0,0,0,0",
                "yellow,8,3,0,0,0,0",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def test_reports_real_saved_map_invalid_width_without_waypoints() -> None:
    result = run_diagnostic([str(BAD_WIDTH_MAP), "--expect-invalid-width"])

    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    item = payload["maps"][0]
    assert item["filename"] == BAD_WIDTH_MAP.name
    assert item["sha256"] == hashlib.sha256(BAD_WIDTH_MAP.read_bytes()).hexdigest()
    assert item["min_same_color_distance"] == item["min_same_color_distance_m"]
    assert item["min_width"] == item["min_width_m"]
    assert item["max_width"] == item["max_width_m"]
    assert item["min_width_m"] < 2.0
    assert item["verdict"] == "invalid"
    assert "invalid_width" in item["reasons"]
    assert "waypoints" not in item


def test_expect_valid_postfix_accepts_strict_valid_map_with_plan_aliases(tmp_path: Path) -> None:
    map_csv = tmp_path / "valid.csv"
    write_map(map_csv, 0.0)

    result = run_diagnostic([str(map_csv), "--close-loop-distance", "20.0", "--expect-valid-postfix"])

    assert result.returncode == 0, result.stderr
    item = json.loads(result.stdout)["maps"][0]
    assert item["verdict"] == "valid"
    assert item["min_track_width_m"] >= 2.0
    assert item["self_intersections"] == item["self_intersection_count"] == 0
    assert item["duplicate_same_color_pairs"] == item["same_color_duplicate_count"] == 0
    assert item["branch_jumps"] == item["branch_jump_count"] == 0


def test_expect_valid_postfix_rejects_invalid_width_map() -> None:
    result = run_diagnostic([str(BAD_WIDTH_MAP), "--expect-valid-postfix"])

    assert result.returncode == 1
    assert "expected post-fix valid map geometry" in result.stderr


def test_rejects_missing_glob_when_no_inputs() -> None:
    result = run_diagnostic(["--glob", "localization/map/no-such-map-*.csv"])

    assert result.returncode == 2
    assert "glob matched no files" in result.stderr


def test_rejects_bad_numeric_csv_row(tmp_path: Path) -> None:
    bad_csv = tmp_path / "bad.csv"
    bad_csv.write_text(
        "tag,x,y,direction,x_variance,y_variance,xy_covariance\nblue,nope,0,0,0,0,0\n",
        encoding="utf-8",
    )

    result = run_diagnostic([str(bad_csv)])

    assert result.returncode == 2
    assert "bad numeric value" in result.stderr


def test_evidence_rewrites_with_current_input_hash(tmp_path: Path) -> None:
    map_csv = tmp_path / "map.csv"
    evidence = tmp_path / "maps.json"
    write_map(map_csv, 0.0)
    first = run_diagnostic([str(map_csv), "--evidence", str(evidence)])
    first_payload = json.loads(evidence.read_text(encoding="utf-8"))

    write_map(map_csv, 1.0)
    second = run_diagnostic([str(map_csv), "--evidence", str(evidence)])
    second_payload = json.loads(evidence.read_text(encoding="utf-8"))

    assert first.returncode == 0
    assert second.returncode == 0
    assert first_payload["maps"][0]["sha256"] != second_payload["maps"][0]["sha256"]
    assert second_payload["maps"][0]["sha256"] == hashlib.sha256(map_csv.read_bytes()).hexdigest()
