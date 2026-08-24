"""Track loading (eufs_tracks csv) and off-track clutter.

Clutter = the fixed things around a real course -- poles, bushes, fences,
parked cars, people -- that LiDAR clustering turns into cones perception
cannot colour. On the 2026-08-01 runs a good part of the SLAM map was such
unknown-colour landmarks, so the simulator places a configurable population of
them outside the cone corridor, each with its own detection probability and
position jitter, and publishes them as unknown_color_cones exactly like the
real pipeline would.
"""
import csv
import math
import os
import random
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Sequence, Tuple

import yaml

COLOURS = ('blue', 'yellow', 'orange', 'big_orange', 'unknown')


@dataclass
class Cone:
    x: float
    y: float
    tag: str            # blue | yellow | orange | big_orange | unknown


@dataclass
class Clutter:
    x: float
    y: float
    kind: str           # pole | bush | fence | person | car | cone
    detect_prob: float  # per-frame detection probability when in range/FOV
    jitter_m: float     # per-frame position jitter (1 sigma) -- big for bushes/people
    size_m: float       # footprint, for the RViz marker only
    # A 'fence' or 'car' yields several clusters along its length: extra
    # returns relative to (x, y) in the world frame.
    offsets: List[Tuple[float, float]] = field(default_factory=list)


# Kind -> (weight, detect_prob, jitter_m, size_m, n_extra_returns range)
CLUTTER_KINDS = {
    'pole':   (0.30, 0.92, 0.03, 0.25, (0, 0)),
    'bush':   (0.30, 0.55, 0.25, 0.80, (0, 2)),
    'fence':  (0.15, 0.70, 0.10, 0.30, (2, 5)),
    'person': (0.10, 0.35, 0.45, 0.40, (0, 0)),
    'car':    (0.10, 0.80, 0.08, 1.50, (2, 4)),
    'cone':   (0.05, 0.85, 0.03, 0.25, (0, 0)),   # a stray cone lying around
}


def _share_csv_dir() -> Optional[str]:
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('eufs_tracks'), 'csv')
        return d if os.path.isdir(d) else None
    except Exception:  # noqa: BLE001 - not built / no ament
        return None


def resolve_track_path(name_or_path: str) -> str:
    """'small_track' | 'small_track.csv' | '/abs/path.csv' -> existing csv path."""
    p = os.path.expanduser(name_or_path)
    if os.path.isfile(p):
        return p
    base = p if p.endswith('.csv') else p + '.csv'
    candidates = []
    share = _share_csv_dir()
    if share:
        candidates.append(os.path.join(share, base))
    root = os.environ.get('EUFS_MASTER', os.path.expanduser('~/fsk'))
    candidates.append(os.path.join(root, 'src', 'sim', 'eufs_sim', 'eufs_tracks', 'csv', base))
    for c in candidates:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(f'track {name_or_path!r} not found (tried {candidates})')


def load_track_csv(path: str):
    """-> (cones, start) with start = (x, y, yaw) from the car_start row or None.

    eufs csv: tag,x,y,direction,x_variance,y_variance,xy_covariance. Tags used:
    blue, yellow, orange, big_orange, car_start; skidpad files also carry
    'midpoint' rows (planner aids, not cones) which are skipped.
    """
    cones: List[Cone] = []
    start = None
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            tag = (row.get('tag') or '').strip()
            try:
                x, y = float(row['x']), float(row['y'])
            except (KeyError, TypeError, ValueError):
                continue
            if tag == 'car_start':
                start = (x, y, float(row.get('direction') or 0.0))
            elif tag in ('blue', 'yellow', 'orange', 'big_orange'):
                cones.append(Cone(x, y, tag))
            elif tag in ('unknown', 'unknown_color'):
                cones.append(Cone(x, y, 'unknown'))
            # midpoint / anything else: ignored
    return cones, start


def _nearest_cone_dist(x: float, y: float, cones: Sequence[Cone]) -> float:
    best = float('inf')
    for c in cones:
        d = (c.x - x) ** 2 + (c.y - y) ** 2
        if d < best:
            best = d
    return math.sqrt(best)


def generate_clutter(cones: Sequence[Cone], count: int, seed: int,
                     min_dist_m: float = 2.5, margin_m: float = 25.0,
                     max_dist_m: float = 0.0,
                     kinds: Optional[dict] = None) -> List[Clutter]:
    """Scatter `count` clutter objects around the track.

    Candidates are drawn uniformly in the cone bounding box grown by
    `margin_m`; kept when at least `min_dist_m` from every cone (so never in
    the corridor itself) and, when max_dist_m > 0, at most that far from the
    nearest cone (keeps them where the sensors can actually see them).
    Deterministic for a given seed.
    """
    if count <= 0 or not cones:
        return []
    rng = random.Random(seed)
    kinds = kinds or CLUTTER_KINDS
    names = list(kinds.keys())
    weights = [kinds[k][0] for k in names]
    xs = [c.x for c in cones]
    ys = [c.y for c in cones]
    x0, x1 = min(xs) - margin_m, max(xs) + margin_m
    y0, y1 = min(ys) - margin_m, max(ys) + margin_m
    out: List[Clutter] = []
    tries = 0
    while len(out) < count and tries < count * 200:
        tries += 1
        x = rng.uniform(x0, x1)
        y = rng.uniform(y0, y1)
        d = _nearest_cone_dist(x, y, cones)
        if d < min_dist_m:
            continue
        if max_dist_m > 0.0 and d > max_dist_m:
            continue
        kind = rng.choices(names, weights=weights, k=1)[0]
        _, p, jitter, size, (n_lo, n_hi) = kinds[kind]
        n_extra = rng.randint(n_lo, n_hi)
        offsets = []
        if n_extra:
            heading = rng.uniform(0.0, math.pi)
            step = rng.uniform(0.8, 1.6)
            for i in range(1, n_extra + 1):
                offsets.append((i * step * math.cos(heading) + rng.gauss(0, 0.1),
                                i * step * math.sin(heading) + rng.gauss(0, 0.1)))
        out.append(Clutter(x, y, kind, p * rng.uniform(0.85, 1.1), jitter, size, offsets))
    return out


def save_clutter(path: str, clutter: Sequence[Clutter]) -> None:
    data = {'clutter': [asdict(c) for c in clutter]}
    with open(os.path.expanduser(path), 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)


def load_clutter(path: str) -> List[Clutter]:
    with open(os.path.expanduser(path)) as f:
        data = yaml.safe_load(f) or {}
    out = []
    for c in data.get('clutter', []):
        out.append(Clutter(float(c['x']), float(c['y']), str(c.get('kind', 'pole')),
                           float(c.get('detect_prob', 0.8)), float(c.get('jitter_m', 0.05)),
                           float(c.get('size_m', 0.3)),
                           [tuple(map(float, o)) for o in c.get('offsets', [])]))
    return out


def clutter_returns(clutter: Sequence[Clutter]):
    """Flatten clutter into individual returns: (x, y, detect_prob, jitter, owner_index)."""
    out = []
    for i, c in enumerate(clutter):
        out.append((c.x, c.y, c.detect_prob, c.jitter_m, i))
        for ox, oy in c.offsets:
            out.append((c.x + ox, c.y + oy, c.detect_prob, c.jitter_m, i))
    return out
