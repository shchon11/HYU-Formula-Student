#!/usr/bin/env python3
"""Grid-sweep runner for the headless map_harness binary.

Runs every combination of the --grid axes (cartesian product) through
map_harness in parallel, collects the JSON metrics, writes a CSV, and prints a
ranking: finishers first, then most laps, then best mean lap time, ties broken
by CTE RMSE.

Example:
  sweep_map.py \
    --bin install/control_harness/lib/control_harness/map_harness \
    --track src/sim/eufs_sim/eufs_tracks/csv/trackdrive_kase2026.csv \
    --plant-yaml src/sim/eufs_sim/eufs_racecar/robots/eufs/configDry.yaml \
    --grid map_lookahead_max_m=3.5,4.0,4.5,5.0 \
    --grid map_lookahead_slope_s=0.45,0.55,0.65 \
    --fixed two_sided_speed_mps=4.5 \
    --out sweep.csv
"""

import argparse
import csv
import itertools
import json
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor

METRIC_FIELDS = [
    "dnf", "dnf_reason", "laps", "best_lap_s", "mean_lap_s", "cte_rmse_m",
    "cte_max_m", "violation_frac", "boundary_min_m", "steer_rate_mean_radps",
    "steer_sat_frac", "speed_rmse_mps", "planner_invalid_frac", "avg_speed_mps",
    "sim_time_s",
]


def parse_grid(entries):
    axes = {}
    for entry in entries:
        key, _, values = entry.partition("=")
        if not values:
            sys.exit(f"malformed --grid entry: {entry}")
        axes[key] = values.split(",")
    return axes


def run_case(args):
    binary, base_args, params = args
    cmd = [binary] + base_args + [f"{k}={v}" for k, v in params.items()]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    row = dict(params)
    if proc.stdout.strip():
        try:
            row.update(json.loads(proc.stdout.strip().splitlines()[-1]))
        except json.JSONDecodeError:
            row["dnf"] = True
            row["dnf_reason"] = f"unparseable output: {proc.stdout[:200]}"
    else:
        row["dnf"] = True
        row["dnf_reason"] = f"no output (rc={proc.returncode}): {proc.stderr[:200]}"
    return row


def sort_key(row):
    dnf = bool(row.get("dnf", True))
    laps = int(row.get("laps", 0) or 0)
    mean_lap = row.get("mean_lap_s") or float("inf")
    cte = row.get("cte_rmse_m") or float("inf")
    return (dnf, -laps, mean_lap, cte)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bin", required=True, help="path to the map_harness binary")
    parser.add_argument("--track", required=True)
    parser.add_argument("--plant-yaml", required=True)
    parser.add_argument("--grid", action="append", default=[],
                        help="key=v1,v2,... sweep axis (repeatable)")
    parser.add_argument("--fixed", action="append", default=[],
                        help="key=value passed to every run (repeatable)")
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--out", default="sweep_results.csv")
    parser.add_argument("--top", type=int, default=10)
    args = parser.parse_args()

    axes = parse_grid(args.grid)
    if not axes:
        sys.exit("at least one --grid axis is required")
    fixed = dict(entry.partition("=")[::2] for entry in args.fixed)

    base_args = [f"track={args.track}", f"plant_yaml={args.plant_yaml}"]
    base_args += [f"{k}={v}" for k, v in fixed.items()]

    cases = [dict(zip(axes.keys(), combo)) for combo in itertools.product(*axes.values())]
    print(f"{len(cases)} cases x ({' * '.join(f'{k}[{len(v)}]' for k, v in axes.items())})")

    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        rows = list(pool.map(run_case, [(args.bin, base_args, c) for c in cases]))

    param_fields = list(axes.keys())
    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=param_fields + METRIC_FIELDS,
                                extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {args.out}")

    rows.sort(key=sort_key)
    header = param_fields + ["laps", "mean_lap_s", "cte_rmse_m", "violation_frac", "dnf_reason"]
    print("  ".join(f"{h:>18}" for h in header))
    for row in rows[:args.top]:
        cells = [str(row.get(k, "")) for k in param_fields]
        cells.append(str(row.get("laps", "")))
        cells.append(f"{row['mean_lap_s']:.2f}" if row.get("mean_lap_s") else "-")
        cells.append(f"{row['cte_rmse_m']:.3f}" if row.get("cte_rmse_m") is not None else "-")
        cells.append(f"{row['violation_frac']:.3f}" if row.get("violation_frac") is not None else "-")
        cells.append(str(row.get("dnf_reason", "")) if row.get("dnf") else "")
        print("  ".join(f"{c:>18}" for c in cells))


if __name__ == "__main__":
    main()
