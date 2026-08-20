#!/usr/bin/env python3
"""Rewrite class indices across a YOLO label directory.

For the 0801 set the physical right-hand cones are orange traffic cones, so the
checkpoint's ORANGE is what the track contract calls YELLOW. One pass fixes the
whole seed set:

    python3 src/perception/scripts/remap_labels.py datasets/0801_cones ORANGE:YELLOW

Both sides accept a class name from classes.txt or a bare index. The mapping is
applied simultaneously, so ORANGE:YELLOW YELLOW:ORANGE swaps rather than
collapses. Frames already marked reviewed in .labeler_state.json are left alone
unless --include-reviewed is given -- a remap must not silently overwrite work
that was done by hand.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

DEFAULT_CLASSES = ["BLUE", "ORANGE_BIG", "ORANGE", "UNDEFINED", "YELLOW"]


def resolve(token: str, classes: list[str]) -> int:
    upper = [c.upper() for c in classes]
    if token.upper() in upper:
        return upper.index(token.upper())
    try:
        idx = int(token)
    except ValueError:
        sys.exit(f"unknown class '{token}'; classes are {', '.join(classes)}")
    if not 0 <= idx < len(classes):
        sys.exit(f"class index {idx} is outside 0..{len(classes) - 1}")
    return idx


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", type=Path, help="dataset root holding labels/ and classes.txt")
    ap.add_argument("pairs", nargs="+", metavar="FROM:TO", help="e.g. ORANGE:YELLOW or 2:4")
    ap.add_argument("--labels", type=Path, default=None, help="override the labels directory")
    ap.add_argument("--include-reviewed", action="store_true",
                    help="also rewrite frames a human already reviewed")
    ap.add_argument("--dry-run", action="store_true", help="report what would change, write nothing")
    args = ap.parse_args()

    labels = args.labels or (args.root / "labels")
    if not labels.is_dir():
        sys.exit(f"no labels directory at {labels}")

    classes_file = args.root / "classes.txt"
    classes = ([ln.strip() for ln in classes_file.read_text().splitlines() if ln.strip()]
               if classes_file.exists() else list(DEFAULT_CLASSES))

    mapping: dict[int, int] = {}
    for pair in args.pairs:
        if ":" not in pair:
            sys.exit(f"expected FROM:TO, got '{pair}'")
        src, dst = pair.split(":", 1)
        mapping[resolve(src, classes)] = resolve(dst, classes)
    print("remap " + ", ".join(f"{classes[s]}({s}) -> {classes[d]}({d})"
                               for s, d in sorted(mapping.items())))

    reviewed: set[str] = set()
    state = args.root / ".labeler_state.json"
    if state.exists() and not args.include_reviewed:
        try:
            reviewed = {Path(n).stem for n in json.loads(state.read_text()).get("reviewed", [])}
        except (json.JSONDecodeError, OSError):
            pass

    files = sorted(labels.glob("*.txt"))
    before = {i: 0 for i in range(len(classes))}
    after = dict(before)
    changed_files = changed_boxes = skipped = 0

    for path in files:
        if path.stem in reviewed:
            skipped += 1
            continue
        out, hit = [], 0
        for line in path.read_text().splitlines():
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                cls = int(float(parts[0]))
            except ValueError:
                continue
            before[cls] = before.get(cls, 0) + 1
            new = mapping.get(cls, cls)
            after[new] = after.get(new, 0) + 1
            if new != cls:
                hit += 1
            out.append(" ".join([str(new)] + parts[1:]))
        if hit:
            changed_files += 1
            changed_boxes += hit
            if not args.dry_run:
                path.write_text("\n".join(out) + ("\n" if out else ""))

    verb = "would change" if args.dry_run else "changed"
    print(f"{len(files)} label files, {skipped} skipped as reviewed")
    print(f"{verb} {changed_boxes} boxes in {changed_files} files\n")
    width = max(len(c) for c in classes)
    print(f"  {'class':<{width}}  before   after")
    for i, name in enumerate(classes):
        print(f"  {name:<{width}}  {before.get(i, 0):>6}  {after.get(i, 0):>6}")
    if args.dry_run:
        print("\ndry run: nothing written")


if __name__ == "__main__":
    main()
