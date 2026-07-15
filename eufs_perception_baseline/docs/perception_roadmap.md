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

This is why the current `monocular_min_bbox_height_px: 16.0` hard cut is the
wrong shape of answer: it throws away the good lateral information to avoid the
bad longitudinal information. The right answer is to **report both honestly and
let SLAM weight them** — see §3.

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

**Still open:** `/ground_truth/cones` is itself FOV/range-filtered by the
simulated-perception plugin (`cameraFOV`, `lidar*ViewDistance`), which this
session narrowed to 12 m camera / 15 m lidar. So false positives and misses
against it are not trustworthy, and **a recall curve measured against it would
plot the instrument's limit, not the pipeline's**. See §4 Step 1.

---

## 3. The architecture finding: the tracker already exists

The proposal was to replace tiers with a world-frame tracker doing position
EKF, M-of-N existence, colour voting, and TENTATIVE→CONFIRMED→LOCKED states.
All four are already in `eufs_graph_slam`:

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

---

## 4. Plan

Ordered by (robustness gained) / (cost), and by dependency.

### Step 1 — fix the instrument (½ day)

Nothing below can be judged without this.

- **Truth source**: `/ground_truth/cones` → `/ground_truth/track`. The latter is
  the unfiltered full track (`getConeArraysMessage()`, no `processCones()`).
  It publishes in `map` (`<trackFrame>map</trackFrame>`); the plugin also
  supports `base_footprint`, but **nothing else in the workspace subscribes to
  `/ground_truth/track`**, so either changing the xacro or transforming in the
  evaluator via ego odom is safe. Transforming in the evaluator is preferred —
  no shared-config side effects.
- **Metrics**: range RMSE → **range-binned recall**, **FP rate**,
  **lateral/longitudinal split**, **time-to-confirm**.
- **Latency probe**: log `now − header.stamp` at `/cones` publish. There is
  **no end-to-end latency instrumentation at all today**, which is untenable
  when latency is a stated priority.

Note the sim camera's far clip is **20 m** — that is a hard ceiling on any
recall curve measured in sim.

### Step 2 — honest covariance (~40 lines, ½ day) — best value

```python
sigma_lat = D * sigma_u_px / fx                  # bearing, linear in D
sigma_lon = abs(e) * sigma_h_px * D / h_px       # mono: h is already known
sigma_lon = D**2 * sigma_d_px / (fx * B)         # stereo
theta = atan2(y, x)
Sigma = R(theta) @ diag(sigma_lon**2, sigma_lat**2) @ R(theta).T
```

Note the monocular form needs **no distance model** — `h_px` is the measurement
itself, so `σ_D/D = |e|·σ_h/h` falls straight out of the curve. Three
constants (`sigma_u_px`, `sigma_h_px`, `sigma_d_px`) replace six hand-tuned
per-tier variance pairs, which is more robust, not less.

`min_observation_variance: 0.01` in SLAM is a sanity floor and will clamp the
lateral term at short range. That is fine and intended.

Verify with `evaluate_slam.py` map RMSE.

### Step 3 — remove the 16 px cut

Once Step 2 lands, the hard cut is redundant: far cones can be published with
an honest ellipse and SLAM will use the lateral component and ignore the
longitudinal one.

**Acceptance:** 15 m recall goes up **and** SLAM map RMSE does not get worse. If
it gets worse, the covariance is still lying.

### Step 4 — detection ceiling (1 day)

`imgsz: 640` on 720p halves the image: a 14 m cone is **~6 px** at the network's
input. The ceiling on far cones may be the **detector's input resolution, not
depth**. Compare 640 / 960 / 1280 (and a horizon-band 2-pass) on Step 1's recall
curve. Sim has GPU headroom (12–18 %).

### Step 5 — ZNCC cross-check replaces SIFT (2–3 days)

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
team** and will break the pipeline if followed. It should be corrected.

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
**re-derive the px gate** (if it still exists after Step 3). The §2.1 procedure
transfers, except there is no ground truth on the car — use **Tier-1 LiDAR
depth as the reference** instead (0.85–1 % is a good enough baseline).

On Orin: the proposal's resource split (GPU for YOLO, depth on CPU) is right,
with one caveat — perception here is **Python/rclpy**, so intra-process
zero-copy composition presupposes a C++ port. Vectorised ZNCC is 1–3 ms/frame
and fits in the Python node today, but 720p image transport between nodes is the
real cost, so a C++ fusion node is likely needed eventually. `ZED SDK NEURAL
depth + bbox median` is a genuine option on a 2i and could replace Tier 3
outright — but it competes with YOLO for the GPU, so measure its occupancy
before committing. ZNCC is the safe default until then.

---

## 7. Open

- `/cones` under-curves the skidpad left circle (`log_planner_diagnostics: true`
  is temporarily on in `local_planner_skidpad.yaml` for this; remove when done).
- Colour-correct varies 54–95 % run to run. SLAM's `voteLandmarkColor` should
  absorb this — verify rather than fix in perception.
- The fitted curve is one track, one run. Validate on another track before
  trusting it.
- `fit_mono_depth_curve.py --bag` is still a stub.
- Evaluator tier attribution is documented but was never implemented in the PR;
  the `/fusion/debug/cone_tiers` marker stream added this session fills that gap.
