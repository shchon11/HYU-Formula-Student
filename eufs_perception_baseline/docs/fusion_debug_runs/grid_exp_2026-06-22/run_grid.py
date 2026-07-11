#!/usr/bin/env python3
import os
import re
import signal
import subprocess
import time
from pathlib import Path


ROOT = Path("/home/dohyun/FS/HYU-Formula-Student")
OUT = ROOT / "eufs_perception_baseline/docs/fusion_debug_runs/grid_exp_2026-06-22"
EXE = (
    ROOT
    / "install/eufs_perception_baseline/lib/eufs_perception_baseline/perception_baseline_node"
)
PYTHON = Path("/home/dohyun/anaconda3/envs/eufs/bin/python3")

COMBOS = [
    ("c01_baseline_params", 4, 3, 2, 4.0, 0.15),
    ("c02_first_candidate", 3, 2, 2, 8.0, 0.25),
    ("c03_points_only", 3, 2, 2, 4.0, 0.15),
    ("c04_margin_only", 4, 3, 2, 8.0, 0.25),
    ("c05_mid_relaxed", 4, 2, 2, 6.0, 0.20),
    ("c06_near_relaxed", 3, 3, 2, 6.0, 0.20),
    ("c07_aggressive_points", 2, 2, 1, 6.0, 0.20),
    ("c08_aggressive_margin", 3, 2, 2, 12.0, 0.30),
    ("c09_balanced", 3, 2, 2, 6.0, 0.20),
    ("c10_far_conservative", 3, 2, 2, 8.0, 0.20),
    ("c11_margin_low", 3, 2, 2, 5.0, 0.18),
    ("c12_near2_mid2", 2, 2, 2, 8.0, 0.25),
]


def run_shell(command: str, output: Path, timeout: int) -> int:
    wrapped = (
        "source /opt/ros/galactic/setup.bash; "
        f"source {ROOT}/install/setup.bash; "
        f"cd {ROOT}; "
        f"{command}"
    )
    with output.open("w", encoding="utf-8") as handle:
        return subprocess.run(
            ["bash", "-lc", wrapped],
            stdout=handle,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        ).returncode


def launch_combo(
    index: int,
    name: str,
    near: int,
    mid: int,
    far: int,
    margin_px: float,
    margin_ratio: float,
):
    node = f"fusion_grid_{index:02d}"
    cones_topic = f"/cones_grid_{index:02d}"
    debug_prefix = f"/fusion/grid_{index:02d}"
    log_path = OUT / f"{index:02d}_{name}_node.log"
    args = [
        str(PYTHON),
        str(EXE),
        "--ros-args",
        "-r",
        f"__node:={node}",
        "-p",
        "use_sim_time:=true",
        "-p",
        "image_topic:=/zed/left/image_rect_color",
        "-p",
        "pointcloud_topic:=/velodyne_points",
        "-p",
        "bbox_topic:=/yolo_bounding_boxes",
        "-p",
        "camera_info_topic:=/zed/left/camera_info",
        "-p",
        "camera_frame:=zed_left_camera_optical_frame",
        "-p",
        "projection_model:=pinhole",
        "-p",
        f"output_cones_topic:={cones_topic}",
        "-p",
        "output_frame:=base_footprint",
        "-p",
        "sync_tolerance_sec:=2.0",
        "-p",
        "fusion_enabled:=true",
        "-p",
        "publish_fusion_debug:=true",
        "-p",
        f"fusion_debug_prefix:={debug_prefix}",
        "-p",
        f"sparse_near_min_points:={near}",
        "-p",
        f"sparse_mid_min_points:={mid}",
        "-p",
        f"sparse_far_min_points:={far}",
        "-p",
        f"sparse_bbox_margin_px:={margin_px}",
        "-p",
        f"sparse_bbox_margin_ratio:={margin_ratio}",
    ]
    quoted_args = " ".join(f"'{arg}'" for arg in args)
    cmd = (
        "source /opt/ros/galactic/setup.bash; "
        f"source {ROOT}/install/setup.bash; "
        f"cd {ROOT}; "
        "export LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7; "
        f"exec {quoted_args}"
    )
    log_handle = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
        cwd=str(ROOT),
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )
    return proc, log_handle, node, cones_topic, debug_prefix, log_path


def stop_proc(proc: subprocess.Popen, log_handle) -> None:
    if proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=5)
    log_handle.close()


def parse_counts(text: str):
    published = re.findall(
        r"Fusion published cones: blue=(\d+), yellow=(\d+), "
        r"orange=(\d+), big_orange=(\d+), unknown=(\d+)",
        text,
    )
    debug = re.findall(
        r"cluster_assignments=(\d+), sparse_assignments=(\d+); (.*)",
        text,
    )
    assigned_sparse = len(re.findall(r"assigned_sparse", text))
    assigned_cluster = len(re.findall(r"assigned_cluster", text))
    insufficient = len(re.findall(r"insufficient_cluster_support", text))
    no_lidar = len(re.findall(r"no_lidar_support", text))
    roi_reject = len(re.findall(r"rejected_by_roi_or_self_ground", text))
    last_counts = published[-1] if published else ("0", "0", "0", "0", "0")
    last_debug = debug[-1] if debug else ("0", "0", "")
    return {
        "blue": int(last_counts[0]),
        "yellow": int(last_counts[1]),
        "orange": int(last_counts[2]),
        "big_orange": int(last_counts[3]),
        "unknown": int(last_counts[4]),
        "cluster_assignments": int(last_debug[0]),
        "sparse_assignments": int(last_debug[1]),
        "assigned_sparse_mentions": assigned_sparse,
        "assigned_cluster_mentions": assigned_cluster,
        "insufficient_mentions": insufficient,
        "no_lidar_mentions": no_lidar,
        "roi_reject_mentions": roi_reject,
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, combo in enumerate(COMBOS, start=1):
        name, near, mid, far, margin_px, margin_ratio = combo
        proc, handle, node, cones_topic, debug_prefix, log_path = launch_combo(
            index, name, near, mid, far, margin_px, margin_ratio
        )
        time.sleep(8)
        run_shell(
            f"timeout 4s ros2 topic echo {cones_topic}",
            OUT / f"{index:02d}_{name}_cones.txt",
            timeout=8,
        )
        run_shell(
            f"timeout 4s ros2 topic echo {debug_prefix}/bbox_support",
            OUT / f"{index:02d}_{name}_bbox_support.txt",
            timeout=8,
        )
        stop_proc(proc, handle)
        text = log_path.read_text(encoding="utf-8", errors="replace")
        metrics = parse_counts(text)
        rows.append(
            {
                "index": index,
                "name": name,
                "near": near,
                "mid": mid,
                "far": far,
                "margin_px": margin_px,
                "margin_ratio": margin_ratio,
                "node": node,
                "cones_topic": cones_topic,
                "debug_prefix": debug_prefix,
                **metrics,
            }
        )
        time.sleep(1)

    summary = OUT / "summary.csv"
    headers = [
        "index",
        "name",
        "near",
        "mid",
        "far",
        "margin_px",
        "margin_ratio",
        "blue",
        "yellow",
        "orange",
        "big_orange",
        "unknown",
        "cluster_assignments",
        "sparse_assignments",
        "assigned_sparse_mentions",
        "assigned_cluster_mentions",
        "insufficient_mentions",
        "no_lidar_mentions",
        "roi_reject_mentions",
    ]
    with summary.open("w", encoding="utf-8") as handle:
        handle.write(",".join(headers) + "\n")
        for row in rows:
            handle.write(",".join(str(row[h]) for h in headers) + "\n")

    cleanup = OUT / "cleanup_check.txt"
    run_shell(
        "ros2 node list | grep fusion_grid || true; "
        "echo ---; ros2 topic list | grep -E 'cones_grid|/fusion/grid_' || true",
        cleanup,
        timeout=10,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
