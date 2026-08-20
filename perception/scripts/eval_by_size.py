#!/usr/bin/env python3
"""Score a detector by box size, because the average hides the only bucket that matters.

    python3 src/perception/scripts/eval_by_size.py datasets/0801_anchor \
        --model runs/teacher/stageA/weights/best.pt --imgsz 1280

Overall mAP on a mixed training set is dominated by whichever source contributes the
most boxes, and by the near cones that every checkpoint already gets right. The
distant cones this teacher exists to recover are ~5% of the labels, so a model can
lose all of them and barely move mAP. This reports recall and false positives per
size bucket instead, with one-to-one greedy matching so duplicate detections cannot
inflate the numbers.

Boxes are matched within a class by default; --class-agnostic scores localisation
alone, which is the right question for a teacher whose classes will be overridden.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")
BUCKETS = ((0.0, 0.022, "small   <16px"), (0.022, 0.033, "mid   16-24px"),
           (0.033, 0.067, "large 24-48px"), (0.067, 9.9, "huge    >48px"))


def bucket_of(height: float) -> str:
    for lo, hi, name in BUCKETS:
        if lo <= height < hi:
            return name
    return BUCKETS[-1][2]


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


def match(pred: np.ndarray, pred_cls: list, gt: np.ndarray, gt_cls: list,
          thr: float, agnostic: bool) -> tuple[list[int], list[int]]:
    """Greedy one-to-one by descending IoU; returns (gt index matched or -1 per pred, ...)."""
    m = iou_matrix(pred, gt)
    if m.size and not agnostic:
        same = np.array(pred_cls)[:, None] == np.array(gt_cls)[None, :]
        m = np.where(same, m, 0.0)
    pred_to_gt = [-1] * len(pred)
    gt_to_pred = [-1] * len(gt)
    if not m.size:
        return pred_to_gt, gt_to_pred
    order = np.dstack(np.unravel_index(np.argsort(-m, axis=None), m.shape))[0]
    for pi, gi in order:
        if m[pi, gi] < thr:
            break
        if pred_to_gt[pi] == -1 and gt_to_pred[gi] == -1:
            pred_to_gt[pi] = int(gi)
            gt_to_pred[gi] = int(pi)
    return pred_to_gt, gt_to_pred


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", type=Path, help="dataset root with images/ labels/ classes.txt")
    ap.add_argument("--model", type=Path, required=True)
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.4, help="IoU for a match (default 0.4)")
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--device", default="0")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--class-agnostic", action="store_true", help="score localisation only")
    args = ap.parse_args()

    images, labels = args.root / "images", args.root / "labels"
    if not images.is_dir():
        sys.exit(f"no images directory at {images}")
    classes = [ln.strip() for ln in (args.root / "classes.txt").read_text().splitlines() if ln.strip()]
    upper = [c.upper() for c in classes]

    files = sorted(p for p in images.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    if args.limit:
        files = files[: args.limit]

    from ultralytics import YOLO

    model = YOLO(str(args.model))
    mapping = {i: upper.index(str(n).upper()) for i, n in model.names.items() if str(n).upper() in upper}

    gt_n = {name: 0 for _, _, name in BUCKETS}
    hit_n = {name: 0 for _, _, name in BUCKETS}
    fp_n = {name: 0 for _, _, name in BUCKETS}
    wrong_class = 0

    for start in range(0, len(files), args.batch):
        chunk = files[start : start + args.batch]
        results = model.predict(source=[str(p) for p in chunk], imgsz=args.imgsz,
                                conf=args.conf, device=args.device, verbose=False)
        for path, res in zip(chunk, results):
            gt_cls, gt_box = [], []
            lp = labels / f"{path.stem}.txt"
            if lp.exists():
                for line in lp.read_text().split("\n"):
                    parts = line.split()
                    if len(parts) >= 5:
                        gt_cls.append(int(float(parts[0])))
                        gt_box.append([float(v) for v in parts[1:5]])
            gt = to_xyxy(np.array(gt_box, dtype=float).reshape(-1, 4))

            H, W = res.orig_shape
            pred = res.boxes.xyxy.cpu().numpy()
            pred = pred / np.array([W, H, W, H]) if len(pred) else pred.reshape(-1, 4)
            pred_cls = [mapping.get(int(c), -1) for c in res.boxes.cls.cpu().tolist()]

            p2g, g2p = match(pred, pred_cls, gt, gt_cls, args.iou, args.class_agnostic)
            for gi in range(len(gt)):
                b = bucket_of(gt[gi, 3] - gt[gi, 1])
                gt_n[b] += 1
                if g2p[gi] != -1:
                    hit_n[b] += 1
                    if pred_cls[g2p[gi]] != gt_cls[gi]:
                        wrong_class += 1
            for pi in range(len(pred)):
                if p2g[pi] == -1:
                    fp_n[bucket_of(pred[pi, 3] - pred[pi, 1])] += 1
        print(f"\r  {min(start + args.batch, len(files))}/{len(files)}", end="", flush=True)

    print(f"\n\n{args.model}  imgsz {args.imgsz}  conf {args.conf}"
          f"{'  (class-agnostic)' if args.class_agnostic else ''}")
    print(f"{len(files)} frames from {args.root}\n")
    print(f"  {'bucket':<14} {'labels':>7} {'recall':>8} {'missed':>8} {'FP':>7}")
    for _, _, name in BUCKETS:
        n, hit = gt_n[name], hit_n[name]
        rec = f"{100 * hit / n:.1f}%" if n else "-"
        print(f"  {name:<14} {n:>7} {rec:>8} {n - hit:>8} {fp_n[name]:>7}")
    total, hits = sum(gt_n.values()), sum(hit_n.values())
    print(f"  {'total':<14} {total:>7} {100 * hits / max(total, 1):>7.1f}% "
          f"{total - hits:>8} {sum(fp_n.values()):>7}")
    if not args.class_agnostic:
        print(f"\n  matched but wrong class: {wrong_class}")


if __name__ == "__main__":
    main()
