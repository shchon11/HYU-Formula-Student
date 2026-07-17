# Perception Roadmap — robustness (latency + accuracy)

What this session measured, what those measurements mean for the architecture,
and the plan that follows from them. Written after a redesign discussion that
proposed replacing the tier pipeline with a world-frame cone tracker; the
central finding is that **the tracker already exists in GraphSLAM**, so the
plan below is much smaller than that proposal.

Every number here is measured on this branch unless marked otherwise. Where a
number is inherited from the paper or from a proposal, it is labelled as such —
several of those turned out to be wrong for this car.

---

## 0. What the pipeline is actually for

Stated by the team, and worth writing down because this document twice argued
against it without noticing:

> LiDAR clusters are the **backbone**. Fused with a camera detection they carry a
> colour; unfused they still publish as `unknown_color`. A cone the **camera sees
> but LiDAR does not cluster** gets a position estimate **only at a range where
> that estimate is trustworthy**. The trade being bought is **latency and
> robustness**.

So the vision tiers are a *bounded extension* of a LiDAR backbone, not a second
opinion that should reach as far as the camera can see. Two consequences that
§4 got wrong until it was measured:

- **`monocular_min_bbox_height_px` IS the "trustworthy range" boundary.** It is
  the design, not a heuristic to be optimised away. §4 Step 3 proposed deleting
  it; measurement put it back (see there).
- **`/cones` is paced by the LiDAR, by construction.** Its information rate
  cannot exceed the LiDAR's 10 Hz, because the backbone updates at 10 Hz.
  Publishing faster would republish the same clusters with only the vision cones
  moving. See §6b.

## 1. The problem, restated

Not *"estimate far cones precisely"* but:

> **Get every cone the camera can see into the map early, with no false
> positives, at a position good enough to plan on.**

This reframing is the discussion's best contribution and it holds up. The
supporting argument:

Split a cone's position error into **lateral** (across the corridor) and
**longitudinal** (along the line of sight).

- Lateral is a bearing measurement: `σ_lat = Z·σ_u/fx`. At 15 m with 1.5 px of
  detector noise that is **4.7 cm**.
- Longitudinal comes from the depth estimate and is **~1.6 m** at the same range.

Cones lie *along* the track boundary, so a longitudinal error slides a cone
back and forth **on the boundary it already defines** — the boundary's shape
barely moves. "The track turns left up ahead" is therefore already accurate at
15–20 m, with the depth we have today. There is much less to fix than the raw
range RMSE suggests.

This is why the `monocular_min_bbox_height_px: 16.0` hard cut was the wrong
shape of answer: it threw away the good lateral information to avoid the bad
longitudinal information. The right answer is to **report both honestly and let
SLAM weight them** — see §3. Both landed; see §4 Steps 2–3.

---

## 2. What was measured

### 2.1 The monocular curve was wrong; it is now fitted

`D = c·h_n^e` reads depth off the bbox height. The config carried the
*analytic* curve `c=0.2801, e=-1.0`, defended on the grounds that an
undistorted sim lens implies exactly `e=-1`.

That argument is sound about the *lens* and wrong about the *pipeline*: it
assumes the detector's box is exact. It is not. Projecting ground-truth cones
into the image and pairing them with the detector's own boxes (n=2745,
small_track):

| range | true projected h | detector box h | ratio | resulting depth error |
|---|---|---|---|---|
| 4–6 m | 47.4 px | 47.8 px | 1.008 | −0.8 % |
| 6–8 m | 25.7 px | 21.3 px | 0.832 | **+20.2 %** |
| 10–12 m | 17.5 px | 14.1 px | 0.802 | **+24.7 %** |

The box tracks the cone up close and **falls ~17–20 % short past 6 m**. A short
box reads as a distant cone, so depth is over-estimated.

| curve | mean | median | p90 |
|---|---|---|---|
| analytic `0.2801 / −1.0` | 17.60 % | 11.44 % | 50.06 % |
| **fitted `0.5575 / −0.7555`** | **7.00 %** | 4.42 % | 19.71 % |
| paper (ZED 2i) `0.498 / −0.954` | — | — | — |

It is the **detector's range-dependent box bias**, not the lens, that bends the
exponent away from −1 — which is the same reason the paper carries −0.954.

> **These two constants are specific to this camera AND this detector weight.**
> Re-fit on either change. This is the single most fragile thing in the
> pipeline: it is one fit, from one run, on one track.

`fit_mono_depth_curve.py --bag`, which would automate this, is **an
unimplemented stub** (`--analytic` works). The fit above was done by hand from a
recorded bag: pair `/yolo_bounding_boxes` with `/ground_truth/cones` by stamp,
project each truth cone with the camera's own `K` and the URDF extrinsics, match
to the nearest box, then least-squares `ln D = ln c + e·ln h_n`.

### 2.2 Tier 3 cannot replace Tier 2 — because of cost, not accuracy

Stereo's accuracy is fine (**1.39 % mean**, better than the paper's 6.39 %). Its
price is not. With the monocular tier disabled so every cone fell through to
stereo:

- `perception_baseline_node` → **287 % CPU**
- `/cones` → **~1 Hz** (from ~7 Hz)
- meanwhile YOLO idled at 9 % CPU / 12–18 % GPU

Per-crop `cv2.SIFT.detectAndCompute` on ~40 crops/frame is 150–300 ms of pyramid
construction alone. This is why the paper routes only clipped/fallen/big cones
to Tier 3, and why a short box must be **dropped, not demoted to stereo**.

The cost is currently masked: in normal routing only n≈24 cones/run reach
Tier 3. It is a latency bomb waiting for conditions that send more cones there.

### 2.3 Two starvation bugs, both fixed

- **`output_commit_settle_sec: 0.1`** + a single latest-only deferred slot +
  7.5 Hz sim `/clock` meant the newest frame always failed the age gate and was
  overwritten by the next one. Set to `0.0`: boxes **1.35 → 7.13 Hz**,
  keypoints **1.47 → 7.91 Hz**. (The protection was redundant — the
  `_clock_generation` fence and `max_future_stamp_lead_sec` already cover it.)
- **ZED `update_rate: 60`** was unachievable, so left/right free-ran at
  15.2/10.6 Hz, drifting past `image_sync_tolerance_sec: 0.05` and starving
  Tier 3. Both set to **15 Hz**: stamp gap **0.00 ms**, 100 % within tolerance.

### 2.4 The evaluator was measuring the wrong thing (fixed), and now measures against the wrong truth (open)

`/cones` is stamped at bbox capture but was compared against the *latest*
ground truth. Adding stamp matching moved every tier:

| tier | before | after | paper |
|---|---|---|---|
| lidar | 8.74 % | **0.97 %** | 0.85 % |
| sparse | 4.92 % | **0.59 %** | — |
| stereo | 10.12 % | **1.71 %** | 6.39 % |
| monocular | 10.04 % | 9.38 % | 4.49 % |

Monocular barely moved, which is what identified it as a real error rather than
a timing artefact — and led to §2.1.

**Fixed (§4 Step 1), unmeasured:** `/ground_truth/cones` is itself
FOV/range-filtered by the simulated-perception plugin (`cameraFOV`,
`lidar*ViewDistance`), which this session narrowed to 12 m camera / 15 m lidar.
So false positives and misses against it are not trustworthy, and **a recall
curve measured against it would plot the instrument's limit, not the
pipeline's**. The evaluator now measures against `/ground_truth/track` under a
gate we control.

The tier numbers in the table above therefore predate the new instrument and
are **not** directly comparable to what it will print.

---

## 3. The architecture finding: the tracker already exists

The proposal was to replace tiers with a world-frame tracker doing position
EKF, M-of-N existence, colour voting, and TENTATIVE→CONFIRMED→LOCKED states.
All four are already in `hyu_localization`:

```cpp
struct LandmarkRecord {
  Eigen::Matrix2d covariance;            // full 2x2 — can hold the ellipse
  std::size_t observations;              // M-of-N
  std::array<std::uint16_t, 5> color_votes;
  int consecutive_misses;
};
void voteLandmarkColor(LandmarkRecord &, ConeColor);
```

```yaml
landmark_confirm_observations: 3          # the proposal's "3-of-5"
landmark_min_observations_to_publish: 2   # TENTATIVE -> CONFIRMED
landmark_missed_observations_to_delete: 6
landmark_merge_distance: 0.85
update_existing_landmarks: true           # position EKF
use_cone_covariance: true                 # consumes what perception reports
```

Building a second tracker in perception would duplicate this and, worse,
**double-filter**: per-cone EKF output fed into SLAM's landmark filter presents
correlated measurements as independent.

### What is actually broken: perception lies to it

SLAM is ready for a full 2×2 matrix. Perception hands it a circle:

```python
var_x = base_var_x + range_variance_scale * cluster.range_m
var_y = base_var_y + range_variance_scale * cluster.range_m   # == var_x, xy = 0
```

With `range_variance_scale: 0.0005`, a 15 m stereo cone gets
`0.5 + 0.0075 = 0.5075` — the range term contributes **0.15 %**, so the model is
**effectively constant and isotropic**. For a 14 m monocular cone:

| | true σ | reported | error |
|---|---|---|---|
| lateral | 4.7 cm (var 0.002) | 0.35 | **160× over-pessimistic** |
| longitudinal | 1.6 m (var 2.56) | 0.35 | **7× over-optimistic** |

The truth is a **~1160:1 ellipse**; we report **1:1**. And the ellipse's major
axis lies along the **bearing**, not along x or y — so for a cone at 45° even an
anisotropic axis-aligned covariance would be wrong.

**Therefore:** the proposal's "lateral strong, longitudinal weak" EKF needs *no
new code*. It needs perception to stop lying. SLAM already does the rest.

> Implemented in §4 Step 2. One caveat on the numbers above: **1160:1 is a
> variance ratio**; the σ ratio is √1280 ≈ 36:1, which is what the shipped
> defaults reproduce (σ_lat 5.0 cm, σ_lon 1.84 m at 15 m).

---

## 4. Plan

Ordered by (robustness gained) / (cost), and by dependency.

### Step 1 — fix the instrument — **IMPLEMENTED, NOT YET RUN**

Nothing below can be judged without this.

- **Truth source**: `/ground_truth/cones` → `/ground_truth/track`, the
  unfiltered full track (`getConeArraysMessage()`, no `processCones()`).
  Transformed in the evaluator rather than by flipping `<trackFrame>`, so no
  other consumer's semantics change.
- **Metrics**: range-binned recall, FP rate, lateral/longitudinal split,
  time-to-confirm — plus **covariance consistency**, which was not in the
  original plan and turned out to be the acceptance test Steps 2–3 needed.
- **Latency probe**: `_record_output_latency` in the node logs
  `now − header.stamp` at `/cones` publish every `latency_log_period_sec`.

What the implementation had to add beyond the plan:

- **A visibility gate is mandatory, not optional.** The full track includes
  every cone on the lap, so without one, recall is ~5 %. The gate
  (`--fov-deg 110 --max-range 20`) is now *ours*: explicit, printed with the
  results, and tunable. That is the actual fix — §2.4's complaint was never
  "cones is filtered", it was "filtered by something we don't control".
- **Truth outside the gate is excluded, not counted as an FP.** An estimate
  that lands on a real cone at 21 m is not a false positive; it is a real cone
  we did not require. Without this, widening the gate manufactures FPs.
- **Time-to-confirm needs a stable cone identity, and it cannot be a position.**
  Cones are physics objects: the car knocks them and they move, and the plugin
  reports each cone's live `link->WorldPose()`, so the truth correctly follows a
  displaced cone. That means a position hash mints a new identity every time a
  cone is nudged, and a rolling cone becomes a new "cone" every frame. The key
  is the cone's **index** in the plugin's fixed link ordering instead, which
  survives the motion. The evaluator warns if the track's cone count ever
  changes, since that is the one thing that would break the index.
  Identity is unavailable against a base_footprint truth topic either way, and
  the evaluator says so rather than reporting nonsense.
- The truth frame is read from `header.frame_id`, so `--truth-topic
  /ground_truth/cones` still works for comparison against the old numbers.

`/ground_truth/odom` (200 Hz, `map`→`base_footprint`) supplies the transform.
Not `/odometry_integration/car_state`, which is deliberately drifted
(`driftOdometry: true`). Matched by stamp, since `/cones` is stamped at bbox
capture and trails sim time.

Note the sim camera's far clip is **20 m** — that is a hard ceiling on any
recall curve measured in sim, and it is the evaluator's default `--max-range`.

### Step 2 — honest covariance — **IMPLEMENTED, NOT YET RUN**

Geometry in `fusion_core` (`monocular_relative_depth_sigma`,
`stereo_relative_depth_sigma`, `bearing_aligned_covariance` — pure, no ROS);
`_cluster_to_cone` in the node stays the single covariance site.

Both tiers work in the **fraction** `σ_D/D`, not in absolute σ_D. That is what
makes the model survive the optical-depth → ground-range projection the tiers
have already applied by the time `_cluster_to_cone` sees the cluster: a
fraction is dimensionless, so `σ_lon = range_m · (σ_D/D)` is valid in
`base_footprint` without re-deriving anything.

Deviations from the plan as written, and why:

- **Vision tiers only.** The plan said three constants replace *six* per-tier
  pairs. They replace three (mono, horizontal-clip, stereo). LiDAR keeps its
  constants: it ranges directly to ~1 % (§2.4), so its error genuinely is a
  near-circle, and forcing it into a bearing model would need a fourth constant
  to say the same thing.
- **`horizontal_clip` widens `sigma_u_px` only** (30 px), not the whole
  ellipse. A side-clipped cone keeps its full pixel height, so its *depth* is a
  normal monocular estimate; only the bbox centre is pulled inward. That is a
  biased bearing and nothing else. 30 px ≈ 0.7 m at 10 m, which is what the
  retired `horizontal_clip_variance_x: 0.70` encoded.
- **`min_variance` clamps the diagonal only.** Scaling the off-diagonal to
  match would rotate the ellipse off the bearing; letting the floor lift the
  axes can only make it rounder, which is the safe direction.
- **Fails closed to the legacy constant, never drops the cone** — an all-NaN
  camera matrix (what `_camera_matrix` returns for malformed `CameraInfo`)
  costs the ellipse, not the detection.
- **`honest_vision_covariance: false`** restores the old behaviour, because
  without it there is no way to A/B this.

`sigma_d_px: 0.25` is a **placeholder** — the one number here with no
measurement behind it. It predicts 7.0 % depth error at 15 m and 2.8 % at 6 m,
against a measured stereo mean of 1.39 %, so it is likely over-pessimistic; but
the measured figure is aggregated over n≈24 cones at unknown ranges, which is
not enough to fit against. Step 1's `lon z^2` column for
`stereo_rektnet_pnp_sift` is exactly the missing measurement.

`min_observation_variance: 0.01` in SLAM is a sanity floor and will clamp the
lateral term at short range. That is fine and intended.

**Acceptance:** `lat z^2` and `lon z^2` ≈ 1.0 per tier, then `evaluate_slam.py`
map RMSE.

### Step 3 — remove the 16 px cut — **ATTEMPTED, THEN REVERTED BY MEASUREMENT**

The cut stays at 16 px. Step 1 falsified the premise this step rests on.

§1 argues a far cone's bearing is good (`σ_lat = Z·σ_u/fx` = 4.7 cm at 15 m) and
only its depth is bad, so the cut throws away good information. That is true of
an *ideal detector*. Measured on small_track, it is false of **this** detector —
monocular lateral error, binned by range, implied `σ_u`:

| band | implied σ_u |
|---|---|
| ≤ 8.8 m | **1.0 – 2.6 px** ← the model holds exactly |
| 8.8 – 10 m | 13.6 px |
| 10 – 20 m | **17 – 32 px** ← 15–30× worse |

The bearing does not stay good. It **collapses at ~9 m**, inside one 1.25 m
band, and the cliff survives both a 1.0 m and a 2.0 m match radius, so it is not
an association artefact. 9 m is where the cone is ~18 px tall at 720p — about
**9 px at the detector's `imgsz: 640` input, roughly one stride-8 cell**.

So the 16 px cut was never the arbitrary heuristic this document called it: it
sits on a real cliff in the detector. Removing it publishes cones whose true
lateral error is 0.3–1.0 m carrying a covariance claiming ~5 cm — 60×+
over-confident, which is precisely the "corrupt the map rather than just add
noise" failure §2.4's config comment warns about.

Direct evidence that the cut is doing this job: restoring it dropped the
monocular tier's `lat z^2` from **260 → 10.7** with no other change. It removes
exactly the population whose bearing has collapsed.

**Step 4 is the prerequisite for Step 3, not the other way round.** Raise the
detector's input resolution, re-measure the cliff, then re-derive the cut.

### Step 2/3 — what the numbers came out at

`evaluate_perception_tiers.py --duration 60`, small_track, driving ~6 m/s.
Covariance consistency (1.0 = honest, >1 over-confident, <1 wasteful):

| tier | n | lat z² | lon z² | NEES/2 |
|---|---|---|---|---|
| monocular *(before: σ_u 1.5, σ_h 1.5, no cut)* | 159 | 260.23 | 4.47 | **138.58** |
| **monocular** *(after)* | 178 | 2.00 | 0.84 | **1.43** |
| **stereo_rektnet_pnp_sift** | 130 | 1.83 | 0.53 | **1.11** |
| lidar | 227 | 0.07 | 0.09 | 0.08 |
| sparse | 66 | 0.08 | 0.10 | 0.09 |

Both vision tiers now sit within ~2× of honest, and conservative in depth. What
the constants cost to get there:

- **`sigma_d_px: 0.25`** — shipped as an admitted guess, measured `lon z² 0.96`
  first time out. It was right. Untouched.
- **`sigma_u_px: 1.5 → 3.0`** — from the stereo tier, whose errors sit far
  inside the match radius and so are not truncated by it.
- **`monocular_sigma_u_px: 10.0`** — a *fourth* constant, against this plan's
  "three constants replace six". The monocular tier works the smallest boxes
  the detector emits, right against the cliff, and measures ~10 px where stereo
  measures ~3. One constant cannot span that; pretending otherwise is the same
  mistake as the isotropic circle, one level up.
- **`sigma_h_px: 1.5 → 4.0`** — **this one is a patch, not a fix.** The tier has
  a measured **−0.88 m range bias** (`lon mean −0.880`, rms 1.216): cones read
  systematically too close. 4.0 inflates noise to cover a bias. The real fix is
  re-fitting `c`/`e` (§7 already flags the fit as one run, one track). Until
  then, covering the true error is the safe direction.

Recall inside the gate (110°, 0.5–20 m), FP 6.3 % of published cones:

| band | 2.5–5 | 5–7.5 | 7.5–10 | 10–12.5 | 12.5–15 | 15–17.5 |
|---|---|---|---|---|---|---|
| recall | 95.4 % | 87.3 % | 85.3 % | 62.1 % | 14.4 % | 2.4 % |

Recall falls off a cliff at the same ~10 m as the bearing does. **The detector,
not depth, is the far-cone ceiling** — Step 4's hypothesis, now measured.

**Still not run:** `evaluate_slam.py` map RMSE.

### Step 4 — detection ceiling (1 day) — **NOW THE TOP PRIORITY, AND CONFIRMED**

This was ranked fourth on a guess. Step 1 measured it and it is the binding
constraint: it gates Step 3, and it caps recall past 10 m at ≤14 %.

`imgsz: 640` on 720p halves the image: a 14 m cone is **~6 px** at the network's
input. The ceiling on far cones is the **detector's input resolution, not
depth** — confirmed by two independent signatures landing at the same ~9–10 m:
the bearing cliff (§Step 3) and the recall cliff (§Step 2/3), both where the
cone crosses ~8–9 px at the network input, i.e. one stride-8 cell.

Note the cliff hits **bearing**, not just depth — which is what breaks §1's
argument. This was not anticipated anywhere in this document.

Compare 640 / 960 / 1280 (and a horizon-band 2-pass) on Step 1's recall curve
**and** on the `implied σ_u` by-band table. Sim has GPU headroom (12–18 %).
Then re-derive `monocular_min_bbox_height_px` and `monocular_sigma_u_px`, both
of which are detector properties and both of which will move.

### Step 5 — ZNCC cross-check replaces SIFT (2–3 days) — **NOW A THROUGHPUT FIX**

Filed here as a robustness/cross-check improvement. Measured (§6b), it is what
stands between `/cones` and the sensor rate: the fusion node burns **200–235 %
CPU** and publishes at **4.0 Hz against an 8.8 Hz input, with ~0.8 s of
latency**, because n=130 cones/run now reach Tier 3's per-crop SIFT. §2.2 called
this a latency bomb; it has gone off under normal driving.


Swap the SIFT block inside `estimate_rektnet_stereo_depth`. On a rectified pair
SIFT's invariances solve a problem that does not exist: scale differs ~1 %,
rotation is 0, and correspondence is on the same row — a **1D search with a
known prior**.

- Anchor on the **pose model's existing keypoints** — no gradient top-K needed.
  They already sit on the stripe corners, and they make correspondence identity
  (`apex↔apex`), so no descriptor is required.
- `d_prior = (B/H)·h = 0.267·h` — **not the 0.369 in the proposal**, which
  assumes a 325 mm cone (see §5).
- Widen the search window **asymmetrically** (`[0.75, 1.45]·d_prior`): the box
  bias of §2.1 makes true disparity ≈ 1.2× the prior past 6 m.
- Far cones (h < ~20 px) have no room for interior points — match the whole box
  as a single template. Depth precision is not the goal there; **agreement is**.

The robustness payoff is the cross-check itself: monocular depends on the cone
size assumption, stereo depends on matching, and **they fail differently**. A
ZNCC peak near the prior is physical evidence that something geometrically
consistent exists at the distance the box implies — which YOLO false positives
(background texture, sign fragments) cannot produce. That guards the fragile
fitted curve of §2.1.

### Explicitly not doing

- **A new cone tracker** — §3. SLAM has it.
- **Removing the tiers** — tiers are the *measurement* layer; a tracker sits
  above them. They are orthogonal, currently tuned, and covered by 149 tests.
  Revisit only after the above lands.

---

## 5. Constants — corrected for this car

The redesign discussion repeatedly used FS-AI spec numbers. **Our cone is
450 mm** (confirmed by the user for the real car; confirmed by mesh measurement
in sim: `0.3034 × 1.483153 = 0.4500 m`, matching config exactly). Cone-derived
constants are therefore **identical between sim and the real car** — a rare
piece of luck.

| constant | value | note |
|---|---|---|
| cone height | **0.450 m** | sim **and** real. Not 0.325. |
| `d_prior` coefficient `B/H` | **0.267** | 0.12/0.45. Proposal's 0.369 is 40 % wrong for us — enough to miss the search window entirely. |
| baseline `B` | 0.12 m | URDF joint, left y=+0.06 / right y=−0.06 |
| `fx` @ 720p | **448.13** sim / ~527 real | read from `camera_info`, never hardcode |
| camera HFOV | 1.91986 rad (110°) | matches ZED 2i spec |
| velodyne mount height | **0.54 m** | TF: 0.2525+0.25+0.0377. Proposal assumed 0.79 m. |
| `cluster_min_points` | **3** | already at the value the proposal suggested lowering to |

Because cone height matches, `INTEGRATION.md`'s warning to *"replace with the
0.325 competition cone before running on the real car"* **does not apply to this
team** and will break the pipeline if followed. **Corrected** — that section now
says the opposite, and says why.

LiDAR's geometric limit, recomputed with our 0.54 m mount and 0.45 m cone (VLP-16,
2° vertical spacing): 2.6° elevation span at 10 m (1–2 channels), 1.7° at 15 m
(0–1 channels). So **~10–12 m** is the honest limit — consistent with the
measured tier distribution, where the 10–15 m band is monocular-dominated.

---

## 6. Sim vs real

| | sim | ZED 2i (real) |
|---|---|---|
| `fx` @ 720p | 448.13 | ~527 |
| stereo sync | fixed by matching rates | hardware-synced — problem does not exist |
| rectification | ideal (zero distortion) | factory calibration |
| cone height | 0.450 m | **0.450 m — same** |
| detector box bias | ~17–20 % short past 6 m | **unknown, must be re-measured** |
| far clip | 20 m | n/a |

Only two things must be redone on the car: **re-fit the mono curve** and
**re-derive the px gate** (Step 3 set it to 0 but kept the parameter for
exactly this). The §2.1 procedure transfers, except there is no ground truth on
the car — use **Tier-1 LiDAR depth as the reference** instead (0.85–1 % is a
good enough baseline).

The covariance model transfers better than the curve does: only the **exponent**
enters it, so re-fitting `c` alone leaves `sigma_h_px` valid. `sigma_u_px` and
`sigma_d_px` are detector/matcher properties, not camera ones, but `fx` and `B`
are read from `camera_info` and the TF at runtime, so the ellipse rescales to
the real camera on its own.

On Orin: the proposal's resource split (GPU for YOLO, depth on CPU) is right,
with one caveat — perception here is **Python/rclpy**, so intra-process
zero-copy composition presupposes a C++ port. Vectorised ZNCC is 1–3 ms/frame
and fits in the Python node today, but 720p image transport between nodes is the
real cost, so a C++ fusion node is likely needed eventually. `ZED SDK NEURAL
depth + bbox median` is a genuine option on a 2i and could replace Tier 3
outright — but it competes with YOLO for the GPU, so measure its occupancy
before committing. ZNCC is the safe default until then.

---

## 6b. Rate and latency — measured, and it is not where §2.2/§2.3 looked

Goal: **`/cones` should publish at the sensor input rate.** Per §0 that means
the **LiDAR's 10 Hz**, not the camera's: LiDAR clusters are the backbone, so the
backbone's rate is the pipeline's rate. Fusion requires a one-to-one bbox/LiDAR
pair, so `/cones` cannot structurally exceed 10 Hz today. A 30 Hz `/cones` would
need the vision tiers decoupled from the LiDAR frame — which is a different
pipeline from the one §0 describes, and would republish identical clusters
between LiDAR frames.

Measured by tagging
each hop with the image-capture stamp it carries, so `now − stamp` at each hop
is that hop's cumulative age (`--duration 30`, driving):

| hop | rate | age | stamps passed on |
|---|---|---|---|
| `/zed/left/image_rect_color` | 8.8 Hz | −30 ms | — |
| `/yolo_bounding_boxes` | **8.8 Hz** | **−28 ms** | 84 % of images |
| `/velodyne_points` | 10.0 Hz | −94 ms | — |
| **`/cones`** | **4.0 Hz** | **785 ms** | **44 % of boxes** |

(Ages are negative because sim `/clock` ticks at 7.9 Hz, so a subscriber's clock
lags the stamps by up to one 126 ms tick. It biases every row equally.)

**YOLO is not the bottleneck** — it tracks the camera at ~0 added latency. The
fusion node is: it halves the rate and adds ~0.8 s, at **200–235 % CPU**. That is
§2.2's bomb going off. That section said Tier 3's SIFT cost was "masked: in
normal routing only n≈24 cones/run reach Tier 3… a latency bomb waiting for
conditions that send more cones there." A 60 s driving run puts **n=130** through
the stereo tier. The conditions arrived.

### The Tier 3 A/B, which splits the problem in two

`perception_stereo_fallback_enabled:=false`, same track, same driving. Rates are
normalised by the **LiDAR's achieved rate**, because the host drifts between runs
and the LiDAR is the one thing none of this touches:

| | perception CPU | `/cones` ÷ LiDAR |
|---|---|---|
| Tier 3 **on** | **200–235 %** | 0.40 |
| Tier 3 **off** | **28 %** | 0.59 |

Two separate conclusions, and conflating them is what sent this document to
Step 5:

1. **SIFT is ~85–90 % of the node's CPU.** The bomb is real and it is Tier 3.
2. **SIFT is NOT the rate bottleneck.** With it gone the node idles at 28 % CPU
   and still publishes at only ~0.6× the LiDAR. The remaining loss is not
   compute — it is the **one-to-one bbox/LiDAR pairing**, failing because the
   camera free-runs at ~8.8 Hz against a LiDAR that hits 10.0 Hz exactly. That is
   §2.3's pathology, unfixed at this level.

So **ZNCC (Step 5) would not get `/cones` to the backbone's rate.** It would buy
back the CPU and most of the 0.8 s. The rate needs the sync fixed — or Tier 3
deleted rather than optimised, which is the question §0 raises: the design as
stated has a LiDAR backbone plus camera-only cones at trustworthy range, and
**no stereo tier at all**. Before spending 2–3 days making SIFT fast, measure
what Tier 3 is worth: it produced n=130 of ~600 cones at 1.34 % range error, but
those are the big-orange and fallen cones, and it is unmeasured whether the
LiDAR backbone already covers them.

Two things measured along the way, both worth knowing:

- **Everything this section used to say about camera rate was measured wrong,
  and the cause was me.** Over the session 17 processes leaked — old gzservers,
  a stale `spawn_entity`, orphaned `ros2 topic pub` — and drove the host to load
  7.36. Every rate number was taken on top of that, and it got worse as the
  session went on, so **later experiments measured the leak, not the change**.
  The velodyne was used as a control and did not catch it: it sat at 7.7–7.9 Hz
  while the camera collapsed from 8.8 to 3.

  Six conclusions were drawn from single runs on that host and **all six are
  void**: the arc cut's size, `gpu_ray`'s speed-up, the readback/resolution
  test, the cone-cylinder speed-up, "removing the chassis collision is 3× worse",
  and "`max_range: 35` is 3× worse". The last two were filed here as
  *unexplained anomalies*. They are explained: they were measured last, when the
  leak was worst.

  What survives is the **recall** evidence — `gpu_ray` and the cone cylinder
  each degraded returns, measured twice each, and host load does not move recall
  that way. Those rejections stand. The rate claims do not.

### What the rates actually are

Re-measured on an idle host (load 0.03 before launch), three runs per config,
40 s settle, `perception:=false`:

| | left camera | velodyne | **host load** |
|---|---|---|---|
| full arc (350 samples, ±114.6°) | 8.2 / 11.8 / **3.4** | 8.3 / 8.5 / 8.5 | 2.6 – 3.9 |
| **±60° arc (183 samples)** | **12.0 / 13.7 / 13.9** | 9.1 / 9.1 / 9.2 | **1.0 – 1.4** |

**The run-to-run noise floor is ±8 %, not 3×** — that was the leak. And three
independent signals agree that the arc cut is real: it cuts the host load
**2.6×**, lifts the velodyne's own achieved rate, and lifts the camera.

The unexpected part is the **stability**. The full arc does not merely run
slower, it runs *erratically*: 3.4 to 11.8 Hz across three identical runs
minutes apart. It pushes the machine to a load where scheduling jitter decides
the answer. The arc cut holds ±8 %. So the shipped config is not only faster on
average, it is the reason a rate measurement means anything at all — **the
original config was itself a second source of the variance I spent the session
blaming on my changes.**

### What is still open

- **30 Hz is still unproven either way.** The 23/17 ceiling that "closed" it was
  measured on the polluted host. It has not been re-run.
- **The mechanism is still unprofiled.** `ptrace_scope=1` and
  `perf_event_paranoid=4` blocked both gdb and perf, so every mechanism in this
  section — render thread, physics mutex, readback — was inference. The
  measurements that killed `gpu_ray` and the cone cylinder were recall, which
  needs no mechanism; the rate story needs one and does not have it.
- **Method, for whoever measures next.** Three runs minimum, 40 s settle, print
  the host load with every number, and verify the process table is empty first —
  `pgrep -f <pattern>` matches your own shell and will tell you it is clean when
  it is not.

## 7. Open

- **`evaluate_slam.py` map RMSE has not been run.** The covariance is honest per
  §4; whether the map got better is still unverified.
- **`/cones` is at 4 Hz against a 10 Hz backbone, with ~0.8 s of latency**, and
  the cause is measured: Tier 3 SIFT at 200–235 % CPU (§6b). This is the largest
  open item and it is Step 5.
- **The monocular tier's −0.5 m bias is NOT the curve, and the sign proves it.**
  `fit_mono_depth_curve.py --live` is now implemented (it was a stub). Two
  independent runs agree — **c=0.2913, e=-0.9814** (n=1571) and **c=0.3053,
  e=-0.9670** (n=1272) — and both land near the ANALYTIC pinhole curve
  (0.2801, −1.0), not near the shipped hand fit (0.5575, −0.7555). That refutes
  §2.1's premise: this detector's box does not run short enough to bend the
  exponent off −1. Re-fitting cut the tier's `lon z²` from 3.09 to 0.45 and its
  NEES/2 from 1.78 to 0.38 — and left `lon mean` at −0.516 m.

  The by-range residual says why that matters. Where the tier actually runs, the
  **curve reads cones too FAR**:

  | band | 6–8 m | 8–10 m | 10–12 m | 16–18 m | 18–20 m |
  |---|---|---|---|---|---|
  | curve residual | +0.28 m | **+0.79 m** | +0.46 m | −0.56 m | −1.37 m |

  So the curve over-predicts by +0.5 to +0.8 m at 8–12 m, and the published cone
  lands 0.5 m **short**. **Opposite signs**: there is an unexplained ~1.0–1.3 m
  term between the curve's output and the cone's position, pulling cones toward
  the car. The curve is exonerated; the search moves to `_back_project` and the
  camera→base transform. `sigma_h_px` is down 8.0 → 5.5 and still covers the
  remainder with noise.
- **The LiDAR's `max_range: 100` cannot be trimmed**, which makes no sense and
  is reproducible. The ROI needs only 33.5 m, so 2/3 of every ray is
  distance nothing consumes, and "a ray that stops early is a cheap ray" says
  cutting it should be free. Measured at 35 m, twice: left **11.5 → 3.3 → 2.3
  Hz**, with the velodyne control healthy at 7.8. `left/velodyne` is 0.42 and
  0.30 for this config against ≥0.88 for every other, so it is not host drift.
  **Mechanism unknown.** Not shipped, because shipping something that measures
  worse and cannot be explained is worse than leaving the rays long.
- **`monocular_sigma_u_px: 10.0` is a symptom, not a property.** It is large
  because the 16 px gate (10.2 m) reaches into the detector's cliff, which
  starts at ~8.8 m. Step 4 moves the cliff; then both numbers move.
- Everything measured here is **small_track, one host, driving ~6 m/s**. The
  §2.1 warning applies to the whole §4 table, not just the curve.
- `/cones` under-curves the skidpad left circle (`log_planner_diagnostics: true`
  is temporarily on in `local_planner_skidpad.yaml` for this; remove when done).
- Colour-correct varies 54–95 % run to run. SLAM's `voteLandmarkColor` should
  absorb this — verify rather than fix in perception.
- The fitted curve is one track, one run. Validate on another track before
  trusting it.
- `fit_mono_depth_curve.py --bag` is still a stub.
- Evaluator tier attribution is documented but was never implemented in the PR;
  the `/fusion/debug/cone_tiers` marker stream added this session fills that gap.
