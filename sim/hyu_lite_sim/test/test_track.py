import math
import os

import pytest

from hyu_lite_sim.track import (generate_clutter, load_clutter, load_track_csv, resolve_track_path,
                                save_clutter, clutter_returns, Cone)


def _cones():
    return [Cone(math.cos(a) * 20, math.sin(a) * 20, 'blue') for a in [i * 0.2 for i in range(32)]] + \
           [Cone(math.cos(a) * 16, math.sin(a) * 16, 'yellow') for a in [i * 0.2 for i in range(32)]]


def test_clutter_keeps_clear_of_cones_and_is_deterministic():
    cones = _cones()
    a = generate_clutter(cones, 50, seed=4, min_dist_m=3.0, margin_m=20.0, max_dist_m=25.0)
    b = generate_clutter(cones, 50, seed=4, min_dist_m=3.0, margin_m=20.0, max_dist_m=25.0)
    assert len(a) == 50 and [(c.x, c.y, c.kind) for c in a] == [(c.x, c.y, c.kind) for c in b]
    for c in a:
        d = min(math.hypot(c.x - k.x, c.y - k.y) for k in cones)
        assert 3.0 <= d <= 25.0
    assert generate_clutter(cones, 50, seed=5)[0].x != a[0].x
    assert len(clutter_returns(a)) >= 50


def test_clutter_roundtrip(tmp_path):
    cl = generate_clutter(_cones(), 10, seed=1)
    p = tmp_path / 'c.yaml'
    save_clutter(str(p), cl)
    back = load_clutter(str(p))
    assert [(c.x, c.y, c.kind, c.detect_prob, c.offsets) for c in back] == \
           [(c.x, c.y, c.kind, c.detect_prob, [tuple(o) for o in c.offsets]) for c in cl]


def test_load_eufs_csv_if_available():
    try:
        path = resolve_track_path('small_track')
    except FileNotFoundError:
        pytest.skip('eufs_tracks csv not available')
    cones, start = load_track_csv(path)
    assert len(cones) == 71 and start is not None
    tags = {c.tag for c in cones}
    assert tags == {'blue', 'yellow', 'big_orange'}
