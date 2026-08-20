#!/usr/bin/env python3
"""Add the boxes a teacher model finds and the existing labels are missing.

    python3 src/perception/scripts/expand_labels.py datasets/0801_rest \
        --model runs/teacher/stageB/weights/best.pt --imgsz 1280 --dry-run

Unlike prelabel_images.py, which writes a whole label file from scratch, this keeps
every existing box exactly as it is and only appends detections that overlap nothing.
That is the difference between seeding a dataset and repairing one: the 0801 labels
are right about the cones they contain, they are just missing the distant ones, so
rewriting them would throw away correct human work to fix an omission.

A teacher trained partly on FSOCO calls things cones that this course does not have
(--classes limits what may be added) and fires on yellow clothing at close range
(--max-height drops those). Both defaults are for the 0801 set; check them before
pointing this at anything else.

Nothing is committed without --write, and --write tars the labels directory into
datasets/.backups/ first.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")


def to_xyxy(rows: np.ndarray) -> np.ndarray:
    if not len(rows):
        return rows.reshape(-1, 4)
    x, y, w, h = rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3]
    return np.stack([x - w / 2, y - h / 2, x + w / 2, y + h / 2], 1)


def iou_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    if not len(a) or not len(b):
        return np.zeros((len(a), len(b)))
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    return inter / np.clip(area_a[:, None] + area_b[None, :] - inter, 1e-9, None)


def read_label(path: Path) -> tuple[list[int], np.ndarray]:
    cls, box = [], []
    if path.exists():
        for line in path.read_text().split("\n"):
            parts = line.split()
            if len(parts) >= 5:
                cls.append(int(float(parts[0])))
                box.append([float(v) for v in parts[1:5]])
    return cls, np.array(box, dtype=float).reshape(-1, 4)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", type=Path, help="dataset root holding images/ and labels/")
    ap.add_argument("--model", type=Path, required=True, help="teacher checkpoint")
    ap.add_argument("--imgsz", type=int, default=1280, help="teacher inference size (default 1280)")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--device", default="0")
    ap.add_argument("--iou-existing", type=float, default=0.3,
                    help="a detection overlapping an existing box by more than this is a duplicate (default 0.3)")
    ap.add_argument("--iou-added", type=float, default=0.6,
                    help="drop an addition overlapping a higher-confidence addition by more than this. "
                         "yolo26 is end-to-end, so its one-to-one head replaces NMS -- and that head leaks "
                         "duplicates on the tiny distant cones this script exists to add (default 0.6)")
    ap.add_argument("--classes", default="BLUE,YELLOW",
                    help="comma-separated class names allowed to be added, or ALL (default BLUE,YELLOW)")
    ap.add_argument("--max-height", type=float, default=0.06,
                    help="drop additions taller than this share of the image; the missing cones are the "
                         "distant ones, so a large new box is almost always a false positive (default 0.06)")
    ap.add_argument("--min-height", type=float, default=0.0, help="drop additions shorter than this share")
    ap.add_argument("--skip-reviewed", action="store_true",
                    help="leave human-reviewed frames untouched (default: expand them too, since review "
                         "corrected classes rather than adding distant cones)")
    ap.add_argument("--limit", type=int, default=0, help="stop after N images (0 = all)")
    ap.add_argument("--preview", type=Path, help="write overlay images here: existing green, added red")
    ap.add_argument("--preview-count", type=int, default=12)
    ap.add_argument("--write", action="store_true", help="actually modify labels (default is a dry run)")
    args = ap.parse_args()

    images, labels = args.root / "images", args.root / "labels"
    if not images.is_dir():
        sys.exit(f"no images directory at {images}")

    classes = [ln.strip() for ln in (args.root / "classes.txt").read_text().splitlines() if ln.strip()]
    upper = [c.upper() for c in classes]
    if args.classes.strip().upper() == "ALL":
        allowed = set(range(len(classes)))
    else:
        wanted = [c.strip().upper() for c in args.classes.split(",") if c.strip()]
        missing = [c for c in wanted if c not in upper]
        if missing:
            sys.exit(f"--classes names {missing} which are not in {args.root}/classes.txt")
        allowed = {upper.index(c) for c in wanted}

    reviewed: set[str] = set()
    state = args.root / ".labeler_state.json"
    if state.exists() and args.skip_reviewed:
        try:
            reviewed = {Path(n).stem for n in json.loads(state.read_text()).get("reviewed", [])}
        except (json.JSONDecodeError, OSError):
            pass

    files = sorted(p for p in images.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    todo = [p for p in files if p.stem not in reviewed]
    if reviewed:
        print(f"leaving {len(files) - len(todo)} human-reviewed frames untouched")
    if args.limit:
        todo = todo[: args.limit]

    from ultralytics import YOLO

    model = YOLO(str(args.model))
    names = {i: str(n).upper() for i, n in model.names.items()}
    mapping = {i: upper.index(n) for i, n in names.items() if n in upper}
    unmapped = sorted(n for n in names.values() if n not in upper)
    print(f"teacher {args.model} at imgsz {args.imgsz}, conf {args.conf}")
    print(f"may add: {', '.join(classes[i] for i in sorted(allowed))}"
          f"   height {args.min_height:.3f}-{args.max_height:.3f}")
    if unmapped:
        print(f"teacher classes not in classes.txt, ignored: {unmapped}")
    if not args.write:
        print("DRY RUN -- nothing will be written")

    if args.write:
        args.root.parent.mkdir(parents=True, exist_ok=True)
        backup_dir = args.root.parent / ".backups"
        backup_dir.mkdir(exist_ok=True)
        tag = time.strftime("%Y%m%d_%H%M%S")
        archive = backup_dir / f"{args.root.name}_labels_{tag}.tar.gz"
        subprocess.run(["tar", "czf", str(archive), "-C", str(args.root), "labels"], check=True)
        print(f"backed up labels to {archive}")

    previews: list[tuple[Path, np.ndarray, np.ndarray]] = []
    added_per_class = {i: 0 for i in range(len(classes))}
    dropped = {"duplicate": 0, "class": 0, "too_tall": 0, "too_short": 0, "self_duplicate": 0}
    frames_touched = existing_total = 0

    for start in range(0, len(todo), args.batch):
        chunk = todo[start : start + args.batch]
        results = model.predict(source=[str(p) for p in chunk], imgsz=args.imgsz,
                                conf=args.conf, device=args.device, verbose=False)
        for path, res in zip(chunk, results):
            label_path = labels / f"{path.stem}.txt"
            cls_old, box_old = read_label(label_path)
            existing_total += len(cls_old)

            H, W = res.orig_shape
            pred = res.boxes.xyxy.cpu().numpy()
            pred = pred / np.array([W, H, W, H]) if len(pred) else pred.reshape(-1, 4)
            pred_cls = [mapping.get(int(c)) for c in res.boxes.cls.cpu().tolist()]
            pred_conf = res.boxes.conf.cpu().numpy()

            overlaps = iou_matrix(pred, to_xyxy(box_old))
            best = overlaps.max(1) if overlaps.size else np.zeros(len(pred))

            keep_boxes, keep_cls = [], []
            for k in np.argsort(-pred_conf) if len(pred) else []:
                height = pred[k, 3] - pred[k, 1]
                if best[k] > args.iou_existing:
                    dropped["duplicate"] += 1
                elif pred_cls[k] is None or pred_cls[k] not in allowed:
                    dropped["class"] += 1
                elif height > args.max_height:
                    dropped["too_tall"] += 1
                elif height < args.min_height:
                    dropped["too_short"] += 1
                elif keep_boxes and iou_matrix(pred[k : k + 1], np.array(keep_boxes)).max() > args.iou_added:
                    dropped["self_duplicate"] += 1
                else:
                    keep_boxes.append(pred[k])
                    keep_cls.append(pred_cls[k])

            if not keep_boxes:
                continue
            frames_touched += 1
            for c in keep_cls:
                added_per_class[c] += 1

            if args.preview and len(previews) < args.preview_count:
                previews.append((path, to_xyxy(box_old), np.array(keep_boxes)))

            if args.write:
                lines = [f"{c} {b[0]:.6f} {b[1]:.6f} {b[2]:.6f} {b[3]:.6f}" for c, b in zip(cls_old, box_old)]
                for c, b in zip(keep_cls, keep_boxes):
                    cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
                    lines.append(f"{c} {cx:.6f} {cy:.6f} {b[2] - b[0]:.6f} {b[3] - b[1]:.6f}")
                label_path.write_text("\n".join(lines) + "\n")

        done = min(start + args.batch, len(todo))
        print(f"\r  {done}/{len(todo)} images, {sum(added_per_class.values())} additions", end="", flush=True)

    added = sum(added_per_class.values())
    print(f"\n\n{len(todo)} frames, {existing_total} existing boxes")
    print(f"added {added} boxes across {frames_touched} frames "
          f"({100 * added / max(existing_total, 1):.1f}% growth)")
    for i, name in enumerate(classes):
        if added_per_class[i]:
            print(f"  {i}  {name:<12} +{added_per_class[i]}")
    print("dropped: " + ", ".join(f"{k} {v}" for k, v in dropped.items()))

    if args.preview and previews:
        from PIL import Image, ImageDraw

        args.preview.mkdir(parents=True, exist_ok=True)
        for path, old, new in previews:
            im = Image.open(path).convert("RGB")
            draw = ImageDraw.Draw(im)
            W, H = im.size
            for b in old:
                draw.rectangle([b[0] * W, b[1] * H, b[2] * W, b[3] * H], outline=(0, 255, 0), width=2)
            for b in new:
                draw.rectangle([b[0] * W, b[1] * H, b[2] * W, b[3] * H], outline=(255, 0, 0), width=3)
            im.save(args.preview / f"{path.stem}.png")
        print(f"\nwrote {len(previews)} overlays to {args.preview} (green existing, red added)")

    if not args.write:
        print("\nre-run with --write to apply")


if __name__ == "__main__":
    main()
