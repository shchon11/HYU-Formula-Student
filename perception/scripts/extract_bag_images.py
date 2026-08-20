#!/usr/bin/env python3
"""Extract camera frames from rosbag2 (sqlite3) bags into a YOLO dataset tree.

Reads the raw sensor_msgs/Image (or CompressedImage) topic straight out of the
.db3 files -- no rosbag2_py, no playback -- and writes

    <out>/images/<bag_tag>_<seq>.jpg
    <out>/labels/                      (empty, for the labeler to fill)
    <out>/classes.txt                  (class contract of the active weight)
    <out>/manifest.csv                 (image -> bag / topic / stamp_ns)

A sourced ROS 2 environment is required for message deserialization:

    source /opt/ros/humble/setup.bash
    python3 src/perception/scripts/extract_bag_images.py \
        --bags bag/0801_outdoor --out datasets/0801_cones
"""

from __future__ import annotations

import argparse
import csv
import re
import sqlite3
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np

# The class contract of models/cone_detect_yolo26n/weights/best.pt, in the
# checkpoint's own index order. Fine-tuning that weight requires this order.
DEFAULT_CLASSES = ["BLUE", "ORANGE_BIG", "ORANGE", "UNDEFINED", "YELLOW"]

DEFAULT_TOPIC = "/sensors/zed/left/color/rect/image"

IMAGE_TYPE = "sensor_msgs/msg/Image"
COMPRESSED_TYPE = "sensor_msgs/msg/CompressedImage"

_BAG_STAMP = re.compile(r"(\d{4})_(\d{2})_(\d{2})-(\d{2})_(\d{2})_(\d{2})")


def import_ros():
    try:
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import CompressedImage, Image
    except ImportError as exc:  # pragma: no cover - environment guard
        sys.exit(
            f"ROS 2 message types are not importable ({exc}).\n"
            "Source the ROS environment first:  source /opt/ros/humble/setup.bash"
        )
    return deserialize_message, Image, CompressedImage


def bag_tag(bag_dir: Path) -> str:
    """rosbag2_2026_08_01-17_22_34 -> 0801_172234 (short, sortable, unique)."""
    m = _BAG_STAMP.search(bag_dir.name)
    if not m:
        return re.sub(r"[^A-Za-z0-9]+", "_", bag_dir.name).strip("_")
    _, mm, dd, hh, mi, ss = m.groups()
    return f"{mm}{dd}_{hh}{mi}{ss}"


def find_bags(roots: list[Path]) -> list[Path]:
    """Accept bag directories, or parents holding several of them."""
    bags: list[Path] = []
    for root in roots:
        if not root.exists():
            sys.exit(f"no such path: {root}")
        if list(root.glob("*.db3")):
            bags.append(root)
            continue
        found = sorted(p.parent for p in root.glob("*/*.db3"))
        if not found:
            sys.exit(f"no rosbag2 .db3 files under {root}")
        seen: set[Path] = set()
        for p in found:
            if p not in seen:
                seen.add(p)
                bags.append(p)
    return bags


def resolve_topic(con: sqlite3.Connection, wanted: str) -> tuple[int, str, str]:
    """Exact topic if present, else the closest left-camera image topic."""
    rows = con.execute("SELECT id, name, type FROM topics").fetchall()
    by_name = {name: (tid, name, typ) for tid, name, typ in rows}
    if wanted in by_name:
        return by_name[wanted]

    stem = wanted.rstrip("/")
    for tid, name, typ in rows:
        if name == stem + "/compressed" and typ == COMPRESSED_TYPE:
            return tid, name, typ

    key = "left"
    cands = [r for r in rows if key in r[1] and r[2] == IMAGE_TYPE]
    if not cands:
        cands = [r for r in rows if key in r[1] and r[2] == COMPRESSED_TYPE]
    if not cands:
        raise KeyError(wanted)
    tid, name, typ = sorted(cands, key=lambda r: len(r[1]))[0]
    return tid, name, typ


def raw_to_bgr(msg) -> np.ndarray:
    enc = msg.encoding.lower()
    channels = {"bgra8": 4, "rgba8": 4, "bgr8": 3, "rgb8": 3, "mono8": 1}.get(enc)
    if channels is None:
        raise ValueError(f"unsupported encoding: {msg.encoding}")
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    conv = {
        "bgra8": cv2.COLOR_BGRA2BGR,
        "rgba8": cv2.COLOR_RGBA2BGR,
        "rgb8": cv2.COLOR_RGB2BGR,
        "mono8": cv2.COLOR_GRAY2BGR,
    }.get(enc)
    return cv2.cvtColor(img, conv) if conv is not None else img.copy()


def encode_params(ext: str, quality: int) -> list[int]:
    if ext == ".jpg":
        return [cv2.IMWRITE_JPEG_QUALITY, quality]
    if ext == ".png":
        return [cv2.IMWRITE_PNG_COMPRESSION, 3]
    return []


def extract_bag(
    bag_dir: Path,
    out_images: Path,
    topic: str,
    stride: int,
    ext: str,
    quality: int,
    overwrite: bool,
    pool: ThreadPoolExecutor,
    writer: csv.writer,
) -> tuple[int, int]:
    deserialize_message, Image, CompressedImage = import_ros()
    db = sorted(bag_dir.glob("*.db3"))
    tag = bag_tag(bag_dir)
    written = skipped = 0
    seq = 0
    params = encode_params(ext, quality)

    for db_path in db:
        con = sqlite3.connect(f"file:{db_path}?immutable=1", uri=True)
        try:
            try:
                tid, tname, ttype = resolve_topic(con, topic)
            except KeyError:
                print(f"  {bag_dir.name}: no image topic matching {topic}, skipped")
                return 0, 0
            if len(db) == 1:
                print(f"  topic {tname} ({ttype.split('/')[-1]})")

            cur = con.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (tid,),
            )
            pending = []
            for stamp_ns, blob in cur:
                idx = seq
                seq += 1
                if idx % stride:
                    continue
                name = f"{tag}_{idx:05d}{ext}"
                path = out_images / name
                if path.exists() and not overwrite:
                    skipped += 1
                    writer.writerow([name, bag_dir.name, tname, stamp_ns, idx])
                    continue

                msg = deserialize_message(bytes(blob), Image if ttype == IMAGE_TYPE else CompressedImage)
                if ttype == IMAGE_TYPE:
                    bgr = raw_to_bgr(msg)
                else:
                    bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if bgr is None or bgr.size == 0:
                    continue

                pending.append(pool.submit(cv2.imwrite, str(path), bgr, params))
                writer.writerow([name, bag_dir.name, tname, stamp_ns, idx])
                written += 1

                if len(pending) >= 16:
                    for fut in pending:
                        fut.result()
                    pending.clear()
                    print(f"\r  {tag}: {written} written", end="", flush=True)
            for fut in pending:
                fut.result()
        finally:
            con.close()

    print(f"\r  {tag}: {written} written, {skipped} already present" + " " * 20)
    return written, skipped


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags", nargs="+", type=Path, required=True,
                    help="bag directories, or a parent directory holding several")
    ap.add_argument("--out", type=Path, required=True, help="dataset root to create")
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument("--stride", type=int, default=1, help="keep every Nth frame (default: all)")
    ap.add_argument("--format", choices=("jpg", "png"), default="jpg")
    ap.add_argument("--quality", type=int, default=95, help="JPEG quality (default 95)")
    ap.add_argument("--overwrite", action="store_true", help="re-extract frames already on disk")
    ap.add_argument("--workers", type=int, default=4, help="image encode/write threads")
    args = ap.parse_args()

    import_ros()  # fail fast before touching the bags

    bags = find_bags(args.bags)
    out = args.out
    images = out / "images"
    labels = out / "labels"
    images.mkdir(parents=True, exist_ok=True)
    labels.mkdir(parents=True, exist_ok=True)
    (out / "classes.txt").write_text("\n".join(DEFAULT_CLASSES) + "\n")

    ext = "." + args.format
    manifest = out / "manifest.csv"
    print(f"{len(bags)} bag(s) -> {images}")

    total_written = total_skipped = 0
    with manifest.open("w", newline="") as fh, ThreadPoolExecutor(args.workers) as pool:
        writer = csv.writer(fh)
        writer.writerow(["image", "bag", "topic", "stamp_ns", "seq"])
        for bag in bags:
            print(f"[{bag.name}]")
            w, s = extract_bag(bag, images, args.topic, args.stride, ext,
                               args.quality, args.overwrite, pool, writer)
            total_written += w
            total_skipped += s

    print(f"\ndone: {total_written} new, {total_skipped} existing, "
          f"{len(list(images.glob('*' + ext)))} images total in {images}")
    print(f"manifest: {manifest}")
    print(f"next: python3 src/perception/scripts/cone_labeler.py {out}")


if __name__ == "__main__":
    main()
