#!/usr/bin/env python3
"""Carve a contiguous stem range out of a cone dataset into its own source root.

    python3 src/perception/scripts/subset_dataset.py datasets/0801_cones \
        --to 0801_173157_00313 --out datasets/0801_anchor

The frames that were labelled with far cones drawn in are a different dataset from
the ones that were not, even though they live in the same directory -- fine-tuning a
teacher on frames whose distant cones are missing teaches it that distant cones are
background, which is the exact failure the teacher exists to fix. So the well-labelled
range gets to be its own source, and build_cone_dataset.py can weight it separately.

Images and labels are symlinked, not copied; classes.txt is copied verbatim so the
subset still satisfies build_cone_dataset.py's class-order check, and manifest.csv is
filtered so the block split still knows which recording each frame came from.
"""

from __future__ import annotations

import argparse
import csv
import shutil
import sys
from pathlib import Path

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", type=Path, help="dataset root holding images/ labels/ classes.txt")
    ap.add_argument("--out", type=Path, required=True, help="where to write the subset root")
    ap.add_argument("--from", dest="lo", default="", help="first stem to keep, inclusive (default: the first)")
    ap.add_argument("--to", dest="hi", default="￿", help="last stem to keep, inclusive (default: the last)")
    ap.add_argument("--list", type=Path, help="file of stems or filenames to keep, one per line; overrides --from/--to")
    ap.add_argument("--invert", action="store_true", help="keep everything the selection does NOT cover")
    args = ap.parse_args()

    images, labels = args.source / "images", args.source / "labels"
    classes = args.source / "classes.txt"
    for needed in (images, labels, classes):
        if not needed.exists():
            sys.exit(f"{args.source} has no {needed.name}")

    if args.list:
        wanted = {Path(ln.strip()).stem for ln in args.list.read_text().splitlines() if ln.strip()}
        keep = lambda stem: stem in wanted  # noqa: E731
    else:
        keep = lambda stem: args.lo <= stem <= args.hi  # noqa: E731

    frames = []
    orphans = 0
    for image in sorted(p for p in images.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES):
        if keep(image.stem) == args.invert:
            continue
        label = labels / f"{image.stem}.txt"
        if not label.is_file():
            orphans += 1
            continue
        frames.append((image, label))
    if not frames:
        sys.exit("selection matched no frames that have a label file")
    if orphans:
        print(f"{orphans} selected images have no label file, skipped")

    # Rebuilt from scratch each run; refuse to touch anything that is not our own symlink.
    for kind in ("images", "labels"):
        out_dir = args.out / kind
        if out_dir.exists():
            stray = [p.name for p in out_dir.iterdir() if not p.is_symlink()]
            if stray:
                sys.exit(f"{out_dir} holds real files ({', '.join(stray[:3])}); refusing to clear it")
            for link in out_dir.iterdir():
                link.unlink()
        out_dir.mkdir(parents=True, exist_ok=True)

    for image, label in frames:
        for src, out_dir in ((image, args.out / "images"), (label, args.out / "labels")):
            (out_dir / src.name).symlink_to(src.resolve())

    shutil.copyfile(classes, args.out / "classes.txt")

    manifest = args.source / "manifest.csv"
    if manifest.is_file():
        stems = {image.stem for image, _ in frames}
        with manifest.open(newline="") as fh:
            reader = csv.DictReader(fh)
            rows = [r for r in reader if Path(r.get("image", "")).stem in stems]
            fields = reader.fieldnames or []
        with (args.out / "manifest.csv").open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        print(f"manifest: {len(rows)} of {len(stems)} frames carried over")

    boxes = empty = 0
    per_class: dict[int, int] = {}
    for _, label in frames:
        lines = [ln.split() for ln in label.read_text().split("\n")]
        rows = [p for p in lines if len(p) >= 5]
        if not rows:
            empty += 1
        for parts in rows:
            boxes += 1
            per_class[int(float(parts[0]))] = per_class.get(int(float(parts[0])), 0) + 1

    names = [ln.strip() for ln in classes.read_text().splitlines() if ln.strip()]
    print(f"\n{args.out}: {len(frames)} frames, {boxes} boxes, {empty} with no cone")
    for i, name in enumerate(names):
        if per_class.get(i):
            print(f"  {i}  {name:<12} {per_class[i]}")


if __name__ == "__main__":
    main()
