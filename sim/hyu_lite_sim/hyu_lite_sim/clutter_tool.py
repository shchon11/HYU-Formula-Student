#!/usr/bin/env python3
"""Generate / inspect a clutter file for hyu_lite_sim.

    ros2 run hyu_lite_sim clutter_tool small_track --count 80 --seed 3 --out ~/fsk/data/clutter_small_3.yaml
    ros2 run hyu_lite_sim clutter_tool --show ~/fsk/data/clutter_small_3.yaml

A saved file makes a clutter population repeatable across runs (pass it to the
launch as clutter_file:=...). Without --out the tool just prints the summary.
"""
import argparse
import collections

from .track import generate_clutter, load_clutter, load_track_csv, resolve_track_path, save_clutter


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('track', nargs='?', help='eufs track name or csv path')
    ap.add_argument('--count', type=int, default=60)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--min-dist', type=float, default=2.5, help='min distance to any cone [m]')
    ap.add_argument('--margin', type=float, default=25.0, help='bounding-box growth [m]')
    ap.add_argument('--max-dist', type=float, default=30.0, help='max distance to the nearest cone [m], 0 = none')
    ap.add_argument('--out', help='write the clutter yaml here')
    ap.add_argument('--show', help='print a saved clutter yaml and exit')
    a = ap.parse_args()
    if a.show:
        cl = load_clutter(a.show)
    else:
        if not a.track:
            ap.error('track is required unless --show is given')
        path = resolve_track_path(a.track)
        cones, _ = load_track_csv(path)
        cl = generate_clutter(cones, a.count, a.seed, a.min_dist, a.margin, a.max_dist)
        print(f'{path}: {len(cones)} cones -> {len(cl)} clutter objects (seed {a.seed})')
        if a.out:
            save_clutter(a.out, cl)
            print(f'written: {a.out}')
    kinds = collections.Counter(c.kind for c in cl)
    returns = sum(1 + len(c.offsets) for c in cl)
    print(f'{len(cl)} objects / {returns} LiDAR returns: ' + ', '.join(f'{k} {v}' for k, v in sorted(kinds.items())))
    for c in cl[:15]:
        print(f'  {c.kind:7s} ({c.x:7.1f}, {c.y:7.1f}) p={c.detect_prob:.2f} jitter={c.jitter_m:.2f} +{len(c.offsets)}')
    if len(cl) > 15:
        print(f'  ... {len(cl) - 15} more')


if __name__ == '__main__':
    main()
