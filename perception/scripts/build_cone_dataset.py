#!/usr/bin/env python3
"""Merge cone datasets into one train/val split that ultralytics can train on.

    python3 src/perception/scripts/build_cone_dataset.py \
        datasets/0801_cones datasets/fsoco_cones --out datasets/cones_yolo

Every source keeps its own images/ and labels/ where it already lives; the split is
built out of symlinks, so merging costs a few MB of inodes rather than a second copy
of the images. The plain train/images + train/labels layout is deliberate: ultralytics
derives its label cache from the first label file's directory, so list-file splits
that mix sources make train and val fight over one cache and rescan on every run.

Re-running is safe -- the split directories only ever hold symlinks this script made,
and they are rebuilt from scratch each time.

The split is taken in contiguous blocks rather than per frame. The 0801 set is video,
so neighbouring frames are near-duplicates and a per-frame split would leak the
training set into val and flatter the numbers. Blocks are drawn per source, so the
validation set keeps the same source mix -- and therefore the same share of real
0801 track frames -- as the training set.
"""

from __future__ import annotations

import argparse
import csv
import os
import random
import sys
from collections import Counter, defaultdict
from pathlib import Path

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}
GROUP_COLUMNS = ("bag", "team", "group")


def read_groups(root: Path) -> dict[str, str]:
    """Map image stem -> recording it came from, so blocks never straddle two."""
    manifest = root / "manifest.csv"
    if not manifest.is_file():
        return {}
    with manifest.open(newline="") as fh:
        reader = csv.DictReader(fh)
        column = next((c for c in GROUP_COLUMNS if c in (reader.fieldnames or [])), None)
        if column is None:
            return {}
        return {Path(row["image"]).stem: row[column] for row in reader if row.get("image")}


def clear(link_dir: Path) -> None:
    """Empty a split directory, refusing anything that is not a symlink we made."""
    if link_dir.exists():
        stray = [p.name for p in link_dir.iterdir() if not p.is_symlink()]
        if stray:
            sys.exit(f"{link_dir} holds real files ({', '.join(stray[:3])}); refusing to clear it")
        for link in link_dir.iterdir():
            link.unlink()
    link_dir.mkdir(parents=True, exist_ok=True)
    # A stale ultralytics cache would describe the previous split.
    cache = link_dir.with_suffix(".cache")
    if cache.exists():
        cache.unlink()


def collect(root: Path, classes: list[str]) -> list[tuple[str, Path, Path]]:
    """Return (group, image, label) for every frame in a source that has both."""
    own = [ln.strip() for ln in (root / "classes.txt").read_text().splitlines() if ln.strip()]
    if own != classes:
        sys.exit(f"{root}/classes.txt is {own}, expected {classes}")

    groups = read_groups(root)
    found, orphans = [], 0
    for image in sorted((root / "images").iterdir()):
        if image.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        label = root / "labels" / f"{image.stem}.txt"
        if not label.is_file():
            orphans += 1
            continue
        found.append((groups.get(image.stem, "-"), image, label))
    if orphans:
        print(f"  {root.name}: {orphans} images without a label file, skipped")
    return found


def split(items: list, block: int, val_frac: float, rng: random.Random) -> tuple[list, list]:
    by_group = defaultdict(list)
    for item in items:
        by_group[item[0]].append(item)

    blocks = []
    for group in sorted(by_group):
        frames = sorted(by_group[group], key=lambda it: it[1].name)
        blocks += [frames[i:i + block] for i in range(0, len(frames), block)]

    rng.shuffle(blocks)
    target = round(len(items) * val_frac)
    val, train, taken = [], [], 0
    for chunk in blocks:
        if taken < target:
            val += chunk
            taken += len(chunk)
        else:
            train += chunk
    return train, val


def count_boxes(items: list, classes: list[str]) -> Counter:
    counts = Counter()
    for _, _, label in items:
        for line in label.read_text().split("\n"):
            parts = line.split()
            if len(parts) >= 5:
                counts[classes[int(float(parts[0]))]] += 1
    return counts


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sources", nargs="+", type=Path, help="dataset roots holding images/ labels/ classes.txt")
    ap.add_argument("--out", type=Path, default=Path("datasets/cones_yolo"), help="where to write data.yaml")
    ap.add_argument("--val-frac", type=float, default=0.1, help="validation share per source (default 0.1)")
    ap.add_argument("--block", type=int, default=50,
                    help="frames per indivisible block, keeps near-duplicate frames on one side (default 50)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    roots = [p.resolve() for p in args.sources]
    for root in roots:
        for needed in ("images", "labels", "classes.txt"):
            if not (root / needed).exists():
                sys.exit(f"{root} has no {needed}")
    classes = [ln.strip() for ln in (roots[0] / "classes.txt").read_text().splitlines() if ln.strip()]

    rng = random.Random(args.seed)
    train, val, per_source = [], [], []
    for root in roots:
        items = collect(root, classes)
        if not items:
            sys.exit(f"{root} has no usable frames")
        tr, va = split(items, args.block, args.val_frac, rng)
        train += tr
        val += va
        per_source.append((root.name, len(tr), len(va)))

    args.out.mkdir(parents=True, exist_ok=True)
    for name, items in (("train", train), ("val", val)):
        for kind in ("images", "labels"):
            link_dir = args.out / name / kind
            clear(link_dir)
        for _, image, label in items:
            for src, link_dir in ((image, args.out / name / "images"), (label, args.out / name / "labels")):
                link = link_dir / src.name
                if link.exists():
                    sys.exit(f"{src.name} appears in more than one source; stems must be unique")
                link.symlink_to(os.path.relpath(src, link_dir))

    names = "\n".join(f"  {i}: {c}" for i, c in enumerate(classes))
    (args.out / "data.yaml").write_text(
        "# generated by src/perception/scripts/build_cone_dataset.py -- regenerate, do not hand-edit\n"
        f"# sources: {', '.join(r.name for r in roots)}\n"
        f"path: {args.out.resolve()}\n"
        "train: train/images\n"
        "val: val/images\n"
        f"names:\n{names}\n"
    )

    print(f"\n{len(train) + len(val)} frames -> {len(train)} train / {len(val)} val")
    width = max(len(n) for n, _, _ in per_source)
    for name, tr, va in per_source:
        print(f"  {name:<{width}}  {tr:>6} train  {va:>6} val")
    train_counts, val_counts = count_boxes(train, classes), count_boxes(val, classes)
    width = max(len(c) for c in classes)
    print(f"\n  {'class':<{width}}   train     val")
    for name in classes:
        print(f"  {name:<{width}}  {train_counts[name]:>6}  {val_counts[name]:>6}")
    print(f"  {'total':<{width}}  {sum(train_counts.values()):>6}  {sum(val_counts.values()):>6}")
    print(f"\nwrote {args.out}/data.yaml\n"
          f"  yolo detect train data={args.out}/data.yaml model=yolo26n.pt imgsz=960 epochs=100")


if __name__ == "__main__":
    main()
