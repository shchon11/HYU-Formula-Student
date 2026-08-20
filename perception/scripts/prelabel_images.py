#!/usr/bin/env python3
"""Seed YOLO label files with the current cone checkpoint, for hand correction.

The 0801 run mislabels yellow cones as ORANGE, so these seeds are wrong on
exactly the axis being fixed -- which is the point: correcting a class with one
keystroke in cone_labeler.py is far faster than drawing every box from scratch.
Nothing here is training data until a human has reviewed it.

    python3 src/perception/scripts/prelabel_images.py datasets/0801_cones

Writes <root>/labels/<stem>.txt for every image that has no label file yet
(--overwrite re-seeds them all). Class indices are mapped by NAME from the
checkpoint to <root>/classes.txt, so a differently ordered weight is still safe.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")
DEFAULT_MODEL = "src/perception/models/cone_detect_yolo26n/weights/best.pt"
DEFAULT_CLASSES = ["BLUE", "ORANGE_BIG", "ORANGE", "UNDEFINED", "YELLOW"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", type=Path, help="dataset root holding images/ and labels/")
    ap.add_argument("--model", type=Path, default=Path(DEFAULT_MODEL))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default="0", help="'0' for GPU, 'cpu' otherwise")
    ap.add_argument("--overwrite", action="store_true", help="re-seed images that already have labels")
    ap.add_argument("--include-reviewed", action="store_true",
                    help="also re-seed frames a human already reviewed, discarding that work")
    ap.add_argument("--limit", type=int, default=0, help="stop after N images (0 = all)")
    args = ap.parse_args()

    images = args.root / "images"
    labels = args.root / "labels"
    if not images.is_dir():
        sys.exit(f"no images directory at {images}")
    labels.mkdir(parents=True, exist_ok=True)

    classes_file = args.root / "classes.txt"
    classes = ([ln.strip() for ln in classes_file.read_text().splitlines() if ln.strip()]
               if classes_file.exists() else list(DEFAULT_CLASSES))
    upper = [c.upper() for c in classes]

    # Re-seeding is how a better checkpoint gets applied, but it must never walk over
    # frames a human already corrected -- same rule remap_labels.py follows.
    reviewed: set[str] = set()
    state = args.root / ".labeler_state.json"
    if state.exists() and not args.include_reviewed:
        try:
            reviewed = {Path(n).stem for n in json.loads(state.read_text()).get("reviewed", [])}
        except (json.JSONDecodeError, OSError):
            pass

    files = sorted(p for p in images.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    todo = [p for p in files if args.overwrite or not (labels / (p.stem + ".txt")).exists()]
    protected = sum(1 for p in todo if p.stem in reviewed)
    todo = [p for p in todo if p.stem not in reviewed]
    if protected:
        print(f"keeping {protected} human-reviewed frames as they are")
    if args.limit:
        todo = todo[: args.limit]
    if not todo:
        print(f"nothing to do: all {len(files)} images already have label files")
        return

    from ultralytics import YOLO

    model = YOLO(str(args.model))
    names = {i: str(n).upper() for i, n in model.names.items()}
    mapping = {i: upper.index(n) for i, n in names.items() if n in upper}
    dropped = sorted(n for n in names.values() if n not in upper)
    print(f"checkpoint {args.model}")
    print(f"class map  " + ", ".join(f"{names[i]}->{classes[j]}({j})" for i, j in sorted(mapping.items())))
    if dropped:
        print(f"NOT in classes.txt, detections dropped: {dropped}")
    print(f"{len(todo)} of {len(files)} images to seed at conf {args.conf}")

    written = boxes_total = 0
    per_class = {i: 0 for i in range(len(classes))}
    for start in range(0, len(todo), args.batch):
        chunk = todo[start : start + args.batch]
        results = model.predict(source=[str(p) for p in chunk], imgsz=args.imgsz,
                                conf=args.conf, device=args.device, verbose=False)
        for path, res in zip(chunk, results):
            h, w = res.orig_shape
            lines = []
            for xyxy, cls in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist()):
                mapped = mapping.get(int(cls))
                if mapped is None:
                    continue
                x1, y1, x2, y2 = xyxy
                bw, bh = (x2 - x1) / w, (y2 - y1) / h
                if bw <= 0 or bh <= 0:
                    continue
                lines.append(f"{mapped} {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} {bw:.6f} {bh:.6f}")
                per_class[mapped] += 1
            (labels / (path.stem + ".txt")).write_text("\n".join(lines) + ("\n" if lines else ""))
            written += 1
            boxes_total += len(lines)
        print(f"\r  {written}/{len(todo)} images, {boxes_total} boxes", end="", flush=True)

    print(f"\ndone: {written} label files, {boxes_total} boxes")
    for i, name in enumerate(classes):
        print(f"  {i}  {name:<12} {per_class[i]}")
    print("\nThese are UNREVIEWED model guesses. Open the labeler and correct them:")
    print(f"  python3 src/perception/scripts/cone_labeler.py {args.root}")


if __name__ == "__main__":
    main()
