#!/usr/bin/env python3
"""Measure what the perception pipeline puts on the map, against the full track.

This answers the question the stack actually cares about:

    Get every cone the camera can see into the map early, with no false
    positives, at a position good enough to plan on.

which is a recall/FP/covariance question, not a range-RMSE one.  Range error is
still reported (the paper's Table 1 metric), but it is no longer the headline:
a cone's error splits into LATERAL (across the corridor) and LONGITUDINAL
(along the line of sight), and only the lateral part moves the track boundary.
Those are reported separately.

Why not /ground_truth/cones
---------------------------
`/ground_truth/cones` is itself FOV/range-filtered by the simulated-perception
plugin (`cameraFOV`, `lidar*ViewDistance`).  Recall measured against it plots
THE INSTRUMENT'S LIMIT, NOT THE PIPELINE'S, and a real detection outside that
filter counts as a false positive.

So this uses `/ground_truth/track`: the unfiltered full track, every cone in the
world.  The catch is that it publishes in `map` (the car's SPAWN pose, not the
Gazebo origin), so it is transformed into `base_footprint` here using
`/ground_truth/odom` -- matched by stamp, because `/cones` is stamped at bbox
capture and trails sim time by the detector's latency.

Visibility gate
---------------
The full track includes cones behind the car, so a gate is needed or every cone
on the lap counts as a miss.  Unlike the plugin's, this gate is OURS: explicit,
logged, and tunable, so the recall curve is a statement about the pipeline
inside a stated envelope.  Defaults are the sim camera's 110 deg HFOV and its
**20 m far clip**, which is a hard ceiling on any recall curve measured in sim.

An estimate that matches a real cone OUTSIDE the gate is neither a hit nor a
false positive -- it is a real cone we simply did not require.  It is excluded,
so widening the gate cannot manufacture false positives.

Covariance consistency
----------------------
The headline check for the honest-covariance work.  For each matched cone the
error is rotated into the bearing frame and normalized by the reported sigma:

    (lat_err / sigma_lat)^2   should average 1.0
    (lon_err / sigma_lon)^2   should average 1.0
    NEES = e^T Sigma^-1 e     should average 2.0 (2 DOF)

Above 1.0 the tier is over-confident and will corrupt the map, because the
optimizer believes what it is told.  Below 1.0 it is over-cautious and is
throwing information away.  These numbers are what `sigma_u_px`, `sigma_h_px`
and `sigma_d_px` are tuned against.

Usage
-----
Run the sim with perception and debug enabled, drive a lap, then:

    ros2 run hyu_perception evaluate_perception_tiers.py --duration 60

or against a bag:

    ros2 bag play <bag> &
    ./evaluate_perception_tiers.py --duration 0     # until Ctrl-C
"""
import argparse
import math
import sys
import time
from collections import defaultdict, deque

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from eufs_msgs.msg import ConeArrayWithCovariance
    from nav_msgs.msg import Odometry
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:
    sys.exit(
        "This script needs a sourced ROS 2 workspace with eufs_msgs on the "
        "PYTHONPATH:\n"
        "    source /opt/ros/<distro>/setup.bash && source install/setup.bash"
    )


CONE_FIELDS = (
    ("blue_cones", "blue"),
    ("yellow_cones", "yellow"),
    ("orange_cones", "orange"),
    ("big_orange_cones", "big_orange"),
    ("unknown_color_cones", "unknown"),
)


def cones_of(msg):
    """Flatten a ConeArrayWithCovariance into [(x, y, colour, covariance), ...].

    ``covariance`` is the full [xx, xy, yx, yy], because a vision cone's error
    is an ellipse and its off-diagonal is the whole point.
    """
    out = []
    for field, colour in CONE_FIELDS:
        for cone in getattr(msg, field, []):
            x, y = float(cone.point.x), float(cone.point.y)
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            # cone.covariance is a numpy array; `array or [...]` raises
            # "truth value is ambiguous", so test it by length, not truthiness.
            raw_cov = getattr(cone, "covariance", None)
            covariance = (
                [float(v) for v in raw_cov]
                if raw_cov is not None and len(raw_cov) == 4
                else None
            )
            out.append((x, y, colour, covariance))
    return out


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def nearest_by_stamp(buffer, target_ns):
    """Entry in ``buffer`` whose stamp is closest to ``target_ns``."""
    best, best_d = None, None
    for stamp, value in buffer:
        delta = abs(stamp - target_ns)
        if best_d is None or delta < best_d:
            best, best_d = value, delta
    return best, best_d


def yaw_of(quaternion):
    """Yaw from a geometry_msgs quaternion; the track is planar."""
    x, y, z, w = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def map_to_base(cones, pose):
    """Express map-frame cones in base_footprint, given the car's map pose.

    Returns ``(truths, keys)``.  ``keys`` identifies each cone across frames,
    which is what makes "when was this cone first seen vs first mapped"
    answerable.

    The key is the cone's INDEX, not its position. Cones are physics objects --
    the car knocks them and they move, and the plugin reports their live
    ``WorldPose()`` -- so a position hash would mint a new identity every time a
    cone was nudged, and a rolling cone would become a new "cone" every frame.
    The plugin walks the track model's links in a fixed order and packs them
    into per-colour arrays, so the flattened index is stable and survives the
    cone moving.
    """
    car_x, car_y, car_yaw = pose
    cos_y, sin_y = math.cos(-car_yaw), math.sin(-car_yaw)
    truths = []
    for x, y, colour, _cov in cones:
        dx, dy = x - car_x, y - car_y
        truths.append((dx * cos_y - dy * sin_y, dx * sin_y + dy * cos_y, colour))
    return truths, list(range(len(truths)))


def tier_at(tiers, ex, ey, eps=0.05):
    """Tier whose cone_tiers marker sits on this estimate.

    The marker is emitted at the cone position the node published, so an exact
    hit is expected; eps only absorbs message round-tripping.
    """
    best, best_d2 = "unattributed", eps * eps
    for tx, ty, tier in tiers:
        d2 = (ex - tx) ** 2 + (ey - ty) ** 2
        if d2 <= best_d2:
            best, best_d2 = tier, d2
    return best


def error_split(ex, ey, tx, ty):
    """Split an estimate's error into (longitudinal, lateral) about the truth.

    Longitudinal runs along the line of sight to the true cone; lateral runs
    across it.  Signed, so a bias shows up as a non-zero mean rather than
    averaging into the noise.
    """
    theta = math.atan2(ty, tx)
    dx, dy = ex - tx, ey - ty
    lon = dx * math.cos(theta) + dy * math.sin(theta)
    lat = -dx * math.sin(theta) + dy * math.cos(theta)
    return lon, lat


def covariance_along_bearing(covariance, x, y):
    """Reported (sigma_lon, sigma_lat) for a cone at (x, y).

    Rotates the reported 2x2 into the bearing frame, which is where the tier's
    error model actually lives.
    """
    if covariance is None:
        return None
    xx, xy, _yx, yy = covariance
    if not all(math.isfinite(v) for v in (xx, xy, yy)):
        return None
    theta = math.atan2(y, x)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    # u = (cos, sin) is the line of sight; v = (-sin, cos) is across it.
    var_lon = xx * cos_t * cos_t + 2.0 * xy * cos_t * sin_t + yy * sin_t * sin_t
    var_lat = xx * sin_t * sin_t - 2.0 * xy * sin_t * cos_t + yy * cos_t * cos_t
    if var_lon <= 0.0 or var_lat <= 0.0:
        return None
    return math.sqrt(var_lon), math.sqrt(var_lat)


def nees(covariance, dx, dy):
    """Normalized estimation error squared: e^T Sigma^-1 e. Should average 2."""
    if covariance is None:
        return None
    xx, xy, _yx, yy = covariance
    det = xx * yy - xy * xy
    if not math.isfinite(det) or det <= 0.0:
        return None
    # Inverse of a 2x2, written out; e^T Sigma^-1 e.
    value = (yy * dx * dx - 2.0 * xy * dx * dy + xx * dy * dy) / det
    return value if math.isfinite(value) and value >= 0.0 else None


class Visibility:
    """The envelope this run holds the pipeline to."""

    def __init__(self, args):
        self.half_fov = math.radians(args.fov_deg) / 2.0
        self.min_range = args.min_range
        self.max_range = args.max_range

    def sees(self, x, y):
        rng = math.hypot(x, y)
        if not (self.min_range <= rng <= self.max_range):
            return False
        return abs(math.atan2(y, x)) <= self.half_fov


def greedy_match(estimates, truths, radius, visibility, tiers=()):
    """Pair each estimate with its nearest unused truth inside ``radius``.

    ``truths`` is the FULL track in base_footprint, so an estimate can pair
    with a cone outside the visibility gate.  That is deliberate: such a pair
    is excluded rather than counted as a false positive.
    """
    pairs, false_positives, excluded = [], 0, 0
    taken = set()
    for ex, ey, ecolour, ecov in estimates:
        best, best_d2 = None, radius * radius
        for index, (tx, ty, _tcolour) in enumerate(truths):
            if index in taken:
                continue
            d2 = (ex - tx) ** 2 + (ey - ty) ** 2
            if d2 <= best_d2:
                best, best_d2 = index, d2
        if best is None:
            # Matched no real cone at all: a genuine false positive.
            false_positives += 1
            continue
        taken.add(best)
        tx, ty, tcolour = truths[best]
        if not visibility.sees(tx, ty):
            excluded += 1
            continue
        lon_err, lat_err = error_split(ex, ey, tx, ty)
        sigmas = covariance_along_bearing(ecov, ex, ey)
        pairs.append({
            "truth_index": best,
            "est_range": math.hypot(ex, ey),
            "true_range": math.hypot(tx, ty),
            "lon_err": lon_err,
            "lat_err": lat_err,
            "sigma_lon": sigmas[0] if sigmas else None,
            "sigma_lat": sigmas[1] if sigmas else None,
            "nees": nees(ecov, ex - tx, ey - ty),
            "colour_ok": ecolour == tcolour,
            "tier": tier_at(tiers, ex, ey),
        })
    return pairs, false_positives, excluded


class TierEvaluator(Node):
    def __init__(self, args):
        # --duration is SIM seconds, not wall seconds. The sim runs at RTF 0.21
        # to 0.37 and it DRIFTS with load, so two runs at the same wall duration
        # collect different amounts of sim time -- measured 112 vs 64 cone frames
        # for the same `--duration 30`. That is not one experiment with a variable
        # changed; it is two different experiments, and every A/B done that way
        # silently compares different stretches of track.
        super().__init__(
            "evaluate_perception_tiers",
            parameter_overrides=(
                [] if args.wall_clock
                else [Parameter("use_sim_time", value=True)]
            ),
        )
        self.args = args
        self.visibility = Visibility(args)
        self.samples = []
        self.false_positives = 0
        self.excluded_out_of_gate = 0
        self.estimates_seen = 0
        self.frames = 0
        # /cones is stamped at BBOX CAPTURE time, which trails the simulator's
        # ground truth by the detector's latency (~0.5 s). Comparing it against
        # the NEWEST truth measures how far the car drove in between, not
        # perception error -- so buffer truth and odom by stamp and match on it.
        self.truth_buffer = deque(maxlen=600)
        self.odom_buffer = deque(maxlen=4000)   # 200 Hz needs a deeper buffer
        self.tier_buffer = {}
        self.tier_order = deque(maxlen=600)
        self.skews = []
        self.dropped_no_truth = 0
        self.dropped_no_odom = 0
        # "unattributed" has two causes that look identical in the table: the
        # provenance for that frame never arrived (THE APPARATUS FAILED), or a
        # marker did arrive but sat too far from the estimate (a real result).
        # Conflating them is how a dead tiers topic read as a measurement for a
        # whole session. Count the first cause so it can never be silent again.
        self.frames_without_provenance = 0
        self.cones_without_provenance = 0
        # Recall per range band, and time-to-confirm. Truth cones are static in
        # map, so their map coordinates are a stable identity across frames --
        # which is what makes "when was this cone first seen vs first mapped"
        # answerable at all.
        self.visible_by_band = defaultdict(int)
        self.hit_by_band = defaultdict(int)
        self.first_visible_ns = {}
        self.first_matched_ns = {}
        # Index-as-identity holds only while the track's cone count is fixed.
        # If it ever changes, every index past the change refers to a different
        # cone and time-to-confirm is silently wrong -- so watch for it.
        self.truth_counts = set()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            ConeArrayWithCovariance, args.truth_topic,
            self._on_truth, sensor_qos)
        self.create_subscription(
            Odometry, args.odom_topic, self._on_odom, sensor_qos)
        self.create_subscription(
            ConeArrayWithCovariance, args.cones_topic,
            self._on_cones, sensor_qos)
        self.create_subscription(
            MarkerArray, args.tiers_topic, self._on_tiers, sensor_qos)
        self.get_logger().info(
            f"comparing {args.cones_topic} against {args.truth_topic}; "
            f"match radius {args.match_radius} m; visibility gate "
            f"{args.fov_deg:.0f} deg, {args.min_range}-{args.max_range} m"
        )

    def _on_tiers(self, msg):
        # Emitted from the same fusion result as /cones, so it carries the same
        # bbox stamp -- an exact key, no nearest-match needed.
        # MarkerArray has no header of its own; each Marker carries one, and
        # they all share the fusion result's bbox stamp. markers[0] is the
        # DELETEALL, which carries that header too.
        if not msg.markers:
            return
        key = stamp_ns(msg.markers[0])
        self.tier_buffer[key] = [
            (m.pose.position.x, m.pose.position.y, m.ns)
            for m in msg.markers
            if m.action != Marker.DELETEALL
        ]
        self.tier_order.append(key)
        while len(self.tier_buffer) > self.tier_order.maxlen:
            self.tier_buffer.pop(self.tier_order.popleft(), None)

    def _on_odom(self, msg):
        pose = msg.pose.pose
        self.odom_buffer.append((
            stamp_ns(msg),
            (float(pose.position.x), float(pose.position.y),
             yaw_of(pose.orientation)),
        ))

    def _on_truth(self, msg):
        # Trust the message's own frame rather than a flag: /ground_truth/track
        # is map, /ground_truth/cones is already base_footprint, and this script
        # should stay usable against either.
        self.truth_buffer.append(
            (stamp_ns(msg), msg.header.frame_id, cones_of(msg))
        )

    def _truth_in_base(self, target_ns):
        """Full track in base_footprint at the /cones stamp.

        Returns ``(truths, keys, skew)``; ``keys`` is None when the truth topic
        is already base_footprint, because a cone's position in a moving frame
        is not an identity and time-to-confirm cannot be computed from it.
        """
        entry, skew = nearest_by_stamp(
            [(s, (f, c)) for s, f, c in self.truth_buffer], target_ns)
        if entry is None or skew > self.args.max_stamp_skew_sec * 1e9:
            self.dropped_no_truth += 1
            return None, None, None
        frame, cones = entry
        if frame != "map":
            return [(x, y, c) for x, y, c, _ in cones], None, skew
        pose, odom_skew = nearest_by_stamp(self.odom_buffer, target_ns)
        if pose is None or odom_skew > self.args.max_stamp_skew_sec * 1e9:
            self.dropped_no_odom += 1
            return None, None, None
        truths, keys = map_to_base(cones, pose)
        self.truth_counts.add(len(truths))
        return truths, keys, skew

    def _on_cones(self, msg):
        estimates = cones_of(msg)
        if not estimates:
            return
        target = stamp_ns(msg)
        truth, keys, skew = self._truth_in_base(target)
        if truth is None:
            return
        self.skews.append(skew / 1e9)

        tiers = self.tier_buffer.get(target)
        if tiers is None:
            self.frames_without_provenance += 1
            self.cones_without_provenance += len(estimates)
            tiers = []
        pairs, false_positives, excluded = greedy_match(
            estimates, truth, self.args.match_radius, self.visibility, tiers)
        self.samples.extend(pairs)
        self.false_positives += false_positives
        self.excluded_out_of_gate += excluded
        self.estimates_seen += len(estimates)
        self.frames += 1
        self._account_recall(truth, keys, pairs, target)

    def _account_recall(self, truth, keys, pairs, target_ns):
        """Recall and time-to-confirm, per truth cone this frame."""
        hit = {p["truth_index"] for p in pairs}
        for index, (tx, ty, _colour) in enumerate(truth):
            if not self.visibility.sees(tx, ty):
                continue
            band = self._band(math.hypot(tx, ty))
            self.visible_by_band[band] += 1
            if index in hit:
                self.hit_by_band[band] += 1
            if keys is None:
                continue
            key = keys[index]
            self.first_visible_ns.setdefault(key, target_ns)
            if index in hit:
                self.first_matched_ns.setdefault(key, target_ns)

    def _band(self, range_m):
        width = self.args.band_m
        return int(range_m // width) * width

    def report(self):
        if not self.samples:
            print("\nNo matched cones. Check that perception is publishing, "
                  f"that {self.args.truth_topic} and {self.args.odom_topic} are "
                  "live, and that the car has driven far enough for cones to "
                  "enter the visibility gate.")
            return

        print(f"\n{'=' * 66}")
        print(f"cone frames        {self.frames}")
        print(f"matched cones      {len(self.samples)}")
        print(f"false positives    {self.false_positives} "
              f"({100.0 * self.false_positives / max(1, self.estimates_seen):.1f} % "
              f"of {self.estimates_seen} published cones)")
        print(f"excluded           {self.excluded_out_of_gate} "
              f"(real cones outside the gate -- not counted either way)")
        if self.skews:
            print(f"stamp skew         mean "
                  f"{1000 * sum(self.skews) / len(self.skews):.1f} ms"
                  f"   max {1000 * max(self.skews):.1f} ms")
        if self.dropped_no_truth:
            print(f"frames dropped     {self.dropped_no_truth} "
                  f"(no truth within {self.args.max_stamp_skew_sec}s)")
        if self.dropped_no_odom:
            print(f"frames dropped     {self.dropped_no_odom} "
                  f"(no odom within {self.args.max_stamp_skew_sec}s)")
        if self.frames_without_provenance:
            share = 100.0 * self.frames_without_provenance / max(1, self.frames)
            print(f"NO PROVENANCE      {self.frames_without_provenance} frames "
                  f"({share:.1f} %) -> {self.cones_without_provenance} cones "
                  f"forced to\n                   'unattributed'. That row is "
                  f"THIS TOOL FAILING, not a result.")
            if share > 50.0:
                print(f"                   Over half the run. Check that "
                      f"{self.args.tiers_topic}\n                   is the "
                      f"topic the node actually publishes.")

        self._report_recall()
        self._report_error_split()
        self._report_covariance()
        self._report_range_error()
        self._report_time_to_confirm()

        colour_ok = sum(1 for s in self.samples if s["colour_ok"])
        print(f"\ncolour correct     "
              f"{100.0 * colour_ok / len(self.samples):5.1f} %")
        print(f"{'=' * 66}")
        print(f"\nVisibility gate: {self.args.fov_deg:.0f} deg HFOV, "
              f"{self.args.min_range}-{self.args.max_range} m. Recall is a "
              f"statement about\nthe pipeline INSIDE this envelope. The sim "
              f"camera's far clip is 20 m, which is a\nhard ceiling on any "
              f"recall curve measured in sim.")
        print("\nProvenance -- what produced the cone, in the order the node "
              "tries them:\n"
              "  cluster_camera   LiDAR cluster a detection coloured\n"
              "  cluster_only     LiDAR cluster the camera never confirmed "
              "(colour unknown)\n"
              "  sparse           raw LiDAR inside a box, too thin to cluster\n"
              "  monocular        no LiDAR: bbox height through the fitted "
              "depth curve\n"
              "  monocular_zncc   same bearing, depth replaced by the stereo "
              "ZNCC peak\n"
              "  unattributed     NOT a result: the provenance for that frame "
              "never arrived.")

    def _report_recall(self):
        print(f"\nrecall by range band (matched / visible truths):")
        for band in sorted(self.visible_by_band):
            visible = self.visible_by_band[band]
            hits = self.hit_by_band.get(band, 0)
            print(f"  {band:5.1f}-{band + self.args.band_m:5.1f} m   "
                  f"n={visible:5d}   recall {100.0 * hits / visible:5.1f} %")

    def _report_error_split(self):
        # The reframing that matters: cones lie ALONG the boundary, so a
        # longitudinal error slides a cone on the boundary it already defines
        # and barely moves its shape. Lateral error is what bends the track.
        print(f"\nposition error split (signed, m) -- lateral is what moves "
              f"the boundary:")
        print(f"  {'tier':26s} {'n':>5s} {'lat mean':>9s} {'lat rms':>8s} "
              f"{'lon mean':>9s} {'lon rms':>8s}")
        by_tier = defaultdict(list)
        for s in self.samples:
            by_tier[s["tier"]].append(s)
        for tier in sorted(by_tier):
            rows = by_tier[tier]
            print(f"  {tier:26s} {len(rows):5d} "
                  f"{_mean(r['lat_err'] for r in rows):9.3f} "
                  f"{_rms(r['lat_err'] for r in rows):8.3f} "
                  f"{_mean(r['lon_err'] for r in rows):9.3f} "
                  f"{_rms(r['lon_err'] for r in rows):8.3f}")

    def _report_covariance(self):
        rows = [s for s in self.samples if s["nees"] is not None]
        if not rows:
            print("\ncovariance consistency: no cone carried a usable 2x2.")
            return
        print(f"\ncovariance consistency -- 1.0 is honest, >1 over-confident, "
              f"<1 wasteful:")
        print(f"  {'tier':26s} {'n':>5s} {'lat z^2':>8s} {'lon z^2':>8s} "
              f"{'NEES/2':>8s}")
        by_tier = defaultdict(list)
        for s in rows:
            by_tier[s["tier"]].append(s)
        for tier in sorted(by_tier):
            group = by_tier[tier]
            lat_z = [
                (r["lat_err"] / r["sigma_lat"]) ** 2
                for r in group if r["sigma_lat"]
            ]
            lon_z = [
                (r["lon_err"] / r["sigma_lon"]) ** 2
                for r in group if r["sigma_lon"]
            ]
            print(f"  {tier:26s} {len(group):5d} "
                  f"{_mean(lat_z):8.2f} {_mean(lon_z):8.2f} "
                  f"{_mean(r['nees'] / 2.0 for r in group):8.2f}")
        print("  (lat z^2 tunes sigma_u_px; lon z^2 tunes sigma_h_px for "
              "monocular\n   and sigma_d_px for monocular_zncc -- scale each "
              "sigma by sqrt of its column.)")
        self._report_covariance_shape(rows)

    def _report_covariance_shape(self, rows):
        """z^2 per range band: does the model have the right SHAPE, not just size?

        The aggregate z^2 above says how big the reported sigma is on average.
        It cannot say whether the sigma is wrong by a CONSTANT (scale it and be
        done) or wrong by a FUNCTION OF RANGE (scaling just moves the error from
        one end to the other, and the fixed point never converges -- which is
        exactly what sigma_u_px did: 3.0 implied 10, 10 implied 18.6).

        A flat row means one constant can be right. A row that climbs means the
        model is missing a range term and no constant will do.
        """
        bands = sorted({self._band(s["true_range"]) for s in rows})
        if len(bands) < 2:
            return
        by_tier = defaultdict(lambda: defaultdict(list))
        for s in rows:
            if s["sigma_lat"]:
                by_tier[s["tier"]][self._band(s["true_range"])].append(
                    (s["lat_err"] / s["sigma_lat"]) ** 2)
        print(f"\ncovariance SHAPE -- lat z^2 by range band. Flat means one "
              f"constant can be\nright; a sloped row means the model needs a "
              f"range term and no constant will do:")
        print(f"  {'tier':22s}" + "".join(f"{b:>7.1f}m" for b in bands))
        for tier in sorted(by_tier):
            cells = []
            for b in bands:
                vals = by_tier[tier].get(b, [])
                cells.append(f"{_mean(vals):>8.2f}" if len(vals) >= 3
                             else f"{'-':>8s}")
            print(f"  {tier:22s}" + "".join(cells))
        print("  (n<3 in a band prints '-'. Read ACROSS a row, not down a "
              "column.)")
        self._report_lateral_fit(rows, bands)

    def _report_lateral_fit(self, rows, bands):
        """What the model SAYS vs what the error IS, per band, in metres.

        z^2 alone cannot tell you which side is wrong. This prints both, so the
        model's functional form can be read off directly: if the reported sigma
        is a straight line through the origin and the real error is not, the
        missing term is a FLOOR, and no value of sigma_u_px will fix it.
        """
        per = defaultdict(lambda: defaultdict(lambda: ([], [])))
        for s in rows:
            if s["sigma_lat"]:
                errs, sigmas = per[s["tier"]][self._band(s["true_range"])]
                errs.append(s["lat_err"])
                sigmas.append(s["sigma_lat"])
        for tier in sorted(per):
            usable = [b for b in bands if len(per[tier][b][0]) >= 3]
            if len(usable) < 3:
                continue
            print(f"\n  {tier}: lateral model vs reality (m)")
            print(f"    {'band':>8s} {'n':>5s} {'real rms':>9s} "
                  f"{'model says':>11s} {'ratio':>7s}")
            for b in usable:
                errs, sigmas = per[tier][b]
                real, model = _rms(errs), _mean(sigmas)
                print(f"    {b:7.1f}m {len(errs):5d} {real:9.3f} "
                      f"{model:11.3f} {real / model if model else float('nan'):7.2f}")
            print(f"    ratio > 1 = the model is too tight in that band. A ratio "
                  f"that FALLS with\n    range means the model's slope is too "
                  f"steep and it is missing a floor.")

    def _report_range_error(self):
        # The paper's Table 1 metric, kept for comparability. It mixes the two
        # error axes above into one number, which is why it is reported last.
        errors = sorted(
            abs(s["est_range"] - s["true_range"]) / s["true_range"] * 100.0
            for s in self.samples if s["true_range"] > 0.1
        )
        if not errors:
            return

        def pct(p):
            return errors[min(len(errors) - 1, int(p / 100.0 * len(errors)))]

        print(f"\nrange error vs ground truth (the paper's metric):")
        print(f"  mean {sum(errors) / len(errors):6.2f} %   "
              f"median {pct(50):6.2f} %   p90 {pct(90):6.2f} %   "
              f"max {errors[-1]:6.2f} %")
        print(f"\nrange error by tier:")
        by_tier = defaultdict(list)
        for s in self.samples:
            if s["true_range"] <= 0.1:
                continue
            err = abs(s["est_range"] - s["true_range"]) / s["true_range"] * 100.0
            by_tier[s["tier"]].append(err)
        if set(by_tier) == {"unattributed"}:
            print(f"  NO PROVENANCE RECEIVED on {self.args.tiers_topic} -- every "
                  f"number above is\n  a blend of all provenances and means "
                  f"nothing. Check that topic exists\n  (`ros2 topic list | grep "
                  f"cone_provenance`) before reading anything else.\n"
                  f"  The publisher flag is perception_publish_debug:=true "
                  f"(default true).")
        for tier in sorted(by_tier):
            values = sorted(by_tier[tier])
            print(f"  {tier:26s} n={len(values):4d}  "
                  f"mean {sum(values) / len(values):6.2f} %  "
                  f"median {values[len(values) // 2]:6.2f} %  "
                  f"p90 {values[min(len(values) - 1, int(0.9 * len(values)))]:6.2f} %")
        print(f"\n  paper Table 1: LiDAR-camera 0.85 %, monocular 4.49 %, "
              f"stereo 6.39 %, routed 3.38 %")

    def _report_time_to_confirm(self):
        deltas = [
            (self.first_matched_ns[k] - self.first_visible_ns[k]) / 1e9
            for k in self.first_matched_ns
            if k in self.first_visible_ns
        ]
        confirmed, visible = len(self.first_matched_ns), len(self.first_visible_ns)
        print(f"\ntime to confirm (first visible -> first mapped):")
        if not visible:
            print(f"  needs a map-frame truth topic for a stable cone "
                  f"identity; {self.args.truth_topic} publishes in "
                  f"base_footprint.")
            return
        if not deltas:
            print("  no cone was both seen and mapped.")
            return
        deltas.sort()
        print(f"  cones ever mapped  {confirmed}/{visible} "
              f"({100.0 * confirmed / visible:.1f} %)")
        print(f"  median {deltas[len(deltas) // 2]:5.2f} s   "
              f"p90 {deltas[min(len(deltas) - 1, int(0.9 * len(deltas)))]:5.2f} s   "
              f"max {deltas[-1]:5.2f} s")
        if len(self.truth_counts) > 1:
            print(f"  WARNING: the track's cone count changed "
                  f"({sorted(self.truth_counts)}), so the index identity these "
                  f"numbers rest on\n  is broken. Treat time-to-confirm as "
                  f"invalid for this run.")


def _mean(values):
    values = list(values)
    return sum(values) / len(values) if values else float("nan")


def _rms(values):
    values = list(values)
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cones-topic", default="/cones")
    parser.add_argument("--truth-topic", default="/ground_truth/track",
                        help="unfiltered full track, map frame; pass "
                             "/ground_truth/cones for the plugin's own "
                             "FOV-filtered view (already base_footprint)")
    parser.add_argument("--odom-topic", default="/ground_truth/odom",
                        help="map -> base_footprint pose, used only when the "
                             "truth topic publishes in map")
    parser.add_argument("--tiers-topic", default="/fusion/debug/cone_provenance",
                        help="Per-cone provenance markers. This was /fusion/debug/cone_tiers,\n"
                             "which the rebuilt node stopped publishing when the tiers\n"
                             "went away -- leaving every cone silently 'unattributed'.")
    parser.add_argument("--max-stamp-skew-sec", type=float, default=0.05,
                        help="max |cones - truth| stamp gap to accept (s); "
                             "truth publishes at 20 Hz, odom at 200 Hz")
    parser.add_argument("--match-radius", type=float, default=2.0,
                        help="max distance to call an estimate the same cone (m)")
    parser.add_argument("--fov-deg", type=float, default=110.0,
                        help="visibility gate: camera HFOV (sim ZED is 110)")
    parser.add_argument("--min-range", type=float, default=0.5,
                        help="visibility gate: near limit (m)")
    parser.add_argument("--max-range", type=float, default=20.0,
                        help="visibility gate: far limit (m); the sim camera's "
                             "far clip is 20 m")
    parser.add_argument("--band-m", type=float, default=2.5,
                        help="range band width for the recall curve (m)")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="SIM seconds to collect; 0 runs until Ctrl-C. Sim "
                             "seconds, not wall seconds, so the same --duration "
                             "always covers the same stretch of track no matter "
                             "what the machine is doing")
    parser.add_argument("--wall-clock", action="store_true",
                        help="measure --duration in wall seconds. Only for a "
                             "source with no /clock: RTF drifts (0.21-0.37 "
                             "measured here), so two wall-timed runs collect "
                             "different amounts of sim time and are not "
                             "comparable to each other")
    args = parser.parse_args()

    rclpy.init()
    node = TierEvaluator(args)
    try:
        if not _wait_for_clock(node):
            return
        if args.duration > 0:
            end = node.get_clock().now().nanoseconds + args.duration * 1e9
            while rclpy.ok() and node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()


def _wait_for_clock(node, timeout_sec=30.0):
    """Block until sim time starts, so the collection window is real.

    Under use_sim_time the clock reads 0 until the first /clock arrives. Taking
    the window's start from that zero would measure from the epoch and the loop
    would never exit -- a hang that looks exactly like "the sim is just slow".
    """
    if node.args.wall_clock:
        return True
    deadline = time.time() + timeout_sec
    while rclpy.ok() and node.get_clock().now().nanoseconds <= 0:
        if time.time() > deadline:
            node.get_logger().error(
                f"No /clock after {timeout_sec:.0f}s, so sim time never "
                f"started. Run the simulator, play the bag with --clock, or "
                f"pass --wall-clock if this source genuinely has no sim time."
            )
            return False
        rclpy.spin_once(node, timeout_sec=0.1)
    return rclpy.ok()


if __name__ == "__main__":
    main()
