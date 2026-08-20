#!/usr/bin/env python3
"""Convert the public FSOCO bounding-box release into the datasets/ layout.

FSOCO ships as a Supervisely export -- <team>/img/<name> and <team>/ann/<name>.json
next to a top-level meta.json -- and every image carries the mandatory 140 px FSOCO
watermark border that will never show up on the ZED. The border is cropped and the
boxes travel with it, so what lands in datasets/ looks exactly like the 0801 set:

    python3 src/perception/scripts/import_fsoco.py \
        --zip datasets/.downloads/fsoco_bounding_boxes_train.zip \
        --out datasets/fsoco_cones

Class indices follow datasets/0801_cones/classes.txt rather than FSOCO's own order,
so the result merges with the hand-labelled set without a remap pass. Images are read
straight out of the zip -- no 24 GB intermediate extraction -- and re-encoded to jpg
q95 with the long side capped at the camera's 1280, since nothing downstream ever
sees more detail than the ZED delivers.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import zipfile
from collections import Counter
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

DEFAULT_CLASSES = ["BLUE", "ORANGE_BIG", "ORANGE", "UNDEFINED", "YELLOW"]

# FSOCO class titles -> our index. The seg_ variants belong to the segmentation
# release but cost nothing to accept here.
FSOCO_TO_NAME = {
    "blue_cone": "BLUE",
    "large_orange_cone": "ORANGE_BIG",
    "orange_cone": "ORANGE",
    "unknown_cone": "UNDEFINED",
    "yellow_cone": "YELLOW",
}
FSOCO_TO_NAME.update({f"seg_{k}": v for k, v in list(FSOCO_TO_NAME.items())})

# FSOCO_IMPORT_BORDER_THICKNESS in fsoco-devkit; the border is uniform on all sides.
BORDER = 140

_zip: zipfile.ZipFile | None = None
_cfg: dict = {}


def worker_init(zip_path: str, cfg: dict) -> None:
    global _zip, _cfg
    _zip = zipfile.ZipFile(zip_path)
    _cfg = cfg


def is_watermarked(img: np.ndarray) -> bool:
    """The FSOCO importer pads with a black frame; only the bottom band holds the logo.

    Sampled across teams these three bands come out under 0.05 mean, so anything that
    is not a watermark fails this by a wide margin and keeps its full frame rather
    than quietly losing 140 px of real image. The count is reported at the end.
    """
    return (img[:BORDER].mean() < 5.0
            and img[:, :BORDER].mean() < 5.0
            and img[:, -BORDER:].mean() < 5.0)


def convert(ann_member: str) -> dict:
    """Turn one Supervisely annotation into an image/label pair under out/."""
    img_member = ann_member.replace("/ann/", "/img/")[: -len(".json")]
    stem = Path(img_member).stem
    team = ann_member.split("/", 1)[0]
    result = {"stem": stem, "team": team, "counts": Counter(), "note": None}

    data = json.loads(_zip.read(ann_member))
    width, height = int(data["size"]["width"]), int(data["size"]["height"])
    tags = sorted({t["name"] for t in data.get("tags", [])})

    objects = data.get("objects", [])
    if not objects and not _cfg["keep_empty"]:
        result["note"] = "no boxes"
        return result

    buf = np.frombuffer(_zip.read(img_member), np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        result["note"] = "undecodable image"
        return result
    if img.shape[:2] != (height, width):
        # Boxes are stated in annotation coordinates; a mismatch makes them unplaceable.
        result["note"] = f"size mismatch ann {width}x{height} vs img {img.shape[1]}x{img.shape[0]}"
        return result

    crop = (_cfg["crop"] and width > 2 * BORDER and height > 2 * BORDER
            and is_watermarked(img))
    if _cfg["crop"] and not crop:
        result["counts"]["kept_unwatermarked_frame"] += 1
    off = BORDER if crop else 0
    cw, ch = (width - 2 * off), (height - 2 * off)

    lines = []
    for obj in objects:
        if obj.get("geometryType") != "rectangle":
            result["counts"]["skipped_not_rectangle"] += 1
            continue
        obj_tags = {t["name"] for t in obj.get("tags", [])}
        if obj_tags & _cfg["exclude_tags"]:
            result["counts"]["skipped_by_tag"] += 1
            continue
        name = FSOCO_TO_NAME.get(obj.get("classTitle"))
        if name is None:
            result["counts"]["skipped_unknown_class"] += 1
            continue

        (x1, y1), (x2, y2) = obj["points"]["exterior"][:2]
        left, right = sorted((float(x1) - off, float(x2) - off))
        top, bottom = sorted((float(y1) - off, float(y2) - off))
        # A handful of boxes run into the watermark; clamp instead of dropping them.
        left, right = max(left, 0.0), min(right, float(cw))
        top, bottom = max(top, 0.0), min(bottom, float(ch))
        if right - left < 1.0 or bottom - top < 1.0:
            result["counts"]["skipped_degenerate"] += 1
            continue

        idx = _cfg["classes"].index(name)
        lines.append("%d %.6f %.6f %.6f %.6f" % (
            idx, (left + right) / 2 / cw, (top + bottom) / 2 / ch,
            (right - left) / cw, (bottom - top) / ch))
        result["counts"][name] += 1

    if not lines and not _cfg["keep_empty"]:
        result["note"] = "no boxes"
        return result

    img = img[off:off + ch, off:off + cw]
    cap = _cfg["max_side"]
    if cap and max(ch, cw) > cap:
        scale = cap / max(ch, cw)
        img = cv2.resize(img, (round(cw * scale), round(ch * scale)), interpolation=cv2.INTER_AREA)

    if not _cfg["dry_run"]:
        ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, _cfg["quality"]])
        if not ok:
            result["note"] = "jpeg encode failed"
            return result
        Path(_cfg["out"], "images", f"{stem}.jpg").write_bytes(enc.tobytes())
        Path(_cfg["out"], "labels", f"{stem}.txt").write_text("\n".join(lines) + "\n" if lines else "")

    result["row"] = {
        "image": f"{stem}.jpg",
        "team": team,
        "source": img_member,
        "width": img.shape[1],
        "height": img.shape[0],
        "cones": len(lines),
        "tags": " ".join(tags),
    }
    return result


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zip", type=Path, default=Path("datasets/.downloads/fsoco_bounding_boxes_train.zip"),
                    help="the fsoco_bounding_boxes_train.zip as downloaded from fsoco-dataset.com")
    ap.add_argument("--out", type=Path, default=Path("datasets/fsoco_cones"),
                    help="dataset root to create, holding images/ labels/ classes.txt manifest.csv")
    ap.add_argument("--classes", type=Path, default=Path("datasets/0801_cones/classes.txt"),
                    help="class order to follow; must contain the five cone names")
    ap.add_argument("--max-side", type=int, default=1280,
                    help="cap the long side after cropping, 0 to keep native size (default 1280)")
    ap.add_argument("--quality", type=int, default=95, help="jpeg quality (default 95)")
    ap.add_argument("--keep-watermark", action="store_true",
                    help="leave the 140 px FSOCO border in place instead of cropping it away")
    ap.add_argument("--keep-empty", action="store_true",
                    help="also import frames whose annotation holds no boxes")
    ap.add_argument("--exclude-tags", nargs="*", default=[],
                    help="drop boxes carrying any of these FSOCO object tags, e.g. knocked_over truncated")
    ap.add_argument("--workers", type=int, default=len(os.sched_getaffinity(0)))
    ap.add_argument("--limit", type=int, default=0, help="only convert the first N frames, for a smoke test")
    ap.add_argument("--dry-run", action="store_true", help="parse and count, write no images or labels")
    args = ap.parse_args()

    if not args.zip.is_file():
        sys.exit(f"no zip at {args.zip}")
    classes = ([ln.strip() for ln in args.classes.read_text().splitlines() if ln.strip()]
               if args.classes.exists() else list(DEFAULT_CLASSES))
    missing = sorted(set(FSOCO_TO_NAME.values()) - set(classes))
    if missing:
        sys.exit(f"{args.classes} is missing {', '.join(missing)}")

    with zipfile.ZipFile(args.zip) as z:
        anns = sorted(n for n in z.namelist() if "/ann/" in n and n.endswith(".json"))
    if args.limit:
        anns = anns[:args.limit]
    print(f"{len(anns)} annotations in {args.zip}")

    if not args.dry_run:
        (args.out / "images").mkdir(parents=True, exist_ok=True)
        (args.out / "labels").mkdir(parents=True, exist_ok=True)

    cfg = {
        "out": str(args.out), "classes": classes, "crop": not args.keep_watermark,
        "max_side": args.max_side, "quality": args.quality, "keep_empty": args.keep_empty,
        "exclude_tags": set(args.exclude_tags), "dry_run": args.dry_run,
    }

    rows, counts, notes, teams = [], Counter(), Counter(), Counter()
    with Pool(args.workers, initializer=worker_init, initargs=(str(args.zip), cfg)) as pool:
        for done, res in enumerate(pool.imap_unordered(convert, anns, chunksize=8), 1):
            counts.update(res["counts"])
            if res["note"]:
                notes[res["note"].split(" ann ")[0]] += 1
            if "row" in res:
                rows.append(res["row"])
                teams[res["team"]] += 1
            if done % 500 == 0 or done == len(anns):
                print(f"  {done}/{len(anns)} frames, {len(rows)} written", flush=True)

    rows.sort(key=lambda r: r["image"])
    if not args.dry_run:
        (args.out / "classes.txt").write_text("\n".join(classes) + "\n")
        with (args.out / "manifest.csv").open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=["image", "team", "source", "width", "height", "cones", "tags"])
            writer.writeheader()
            writer.writerows(rows)

    print(f"\n{len(rows)} frames imported from {len(teams)} teams into {args.out}")
    width = max(len(c) for c in classes)
    total = 0
    for name in classes:
        total += counts.get(name, 0)
        print(f"  {name:<{width}}  {counts.get(name, 0):>7}")
    print(f"  {'total':<{width}}  {total:>7}")
    for key in sorted(k for k in counts if k not in classes):
        print(f"  {key}: {counts[key]}")
    for note, n in notes.most_common():
        print(f"  frames dropped ({note}): {n}")
    if args.dry_run:
        print("\ndry run: nothing written")


if __name__ == "__main__":
    main()
