import math
import random

from hyu_lite_sim.perception_emu import PerceptionEmulator, PerceptionParams
from hyu_lite_sim.track import Clutter, Cone


def _emu(cones, clutter, **kw):
    p = PerceptionParams(false_positives_per_frame=0.0, outlier_prob=0.0, colour_confusion=0.0,
                         p_colour_near=1.0, p_colour_far=1.0, **kw)
    return PerceptionEmulator(p, cones, clutter, random.Random(3))


def test_near_cones_detected_in_base_frame_with_colour():
    cones = [Cone(5.0, 1.0, 'blue'), Cone(5.0, -1.0, 'yellow'), Cone(-5.0, 0.0, 'blue')]
    emu = _emu(cones, [])
    obs = emu.observe((0.0, 0.0, 0.0), 0.0)
    assert len(obs['blue']) == 1 and len(obs['yellow']) == 1      # the one behind is outside the wedge
    x, y, cov = obs['blue'][0]
    assert abs(x - 5.0) < 0.3 and abs(y - 1.0) < 0.3
    assert cov[0] == cov[3] == 0.07 and cov[1] == 0.0


def test_rotation_into_base_frame():
    cones = [Cone(0.0, 6.0, 'yellow')]
    emu = _emu(cones, [])
    obs = emu.observe((0.0, 0.0, math.pi / 2), 0.0)     # car facing +y: cone straight ahead
    x, y, _ = obs['yellow'][0]
    assert abs(x - 6.0) < 0.3 and abs(y) < 0.3


def test_clutter_is_unknown_and_far_cones_lose_colour():
    cones = [Cone(15.0, 0.0, 'blue')]
    clutter = [Clutter(6.0, 3.0, 'pole', 1.0, 0.0, 0.2)]
    emu = _emu(cones, clutter, p_detect_far=1.0)
    obs = emu.observe((0.0, 0.0, 0.0), 0.0)
    assert len(obs['blue']) == 0 and len(obs['unknown']) == 2     # far cone (>12 m) + the pole
    covs = sorted(c[2][0] for c in obs['unknown'])
    assert covs[0] == 0.23                                         # LiDAR-only tier


def test_truth_visible_respects_range_and_fov():
    cones = [Cone(2.0, 0.0, 'blue'), Cone(25.0, 0.0, 'blue'), Cone(0.0, -5.0, 'yellow'), Cone(-1.0, 3.0, 'blue')]
    emu = _emu(cones, [])
    vis = emu.truth_visible((0.0, 0.0, 0.0))
    assert len(vis['blue']) == 1 and len(vis['yellow']) == 1     # 25 m too far, -1 m behind the wedge
