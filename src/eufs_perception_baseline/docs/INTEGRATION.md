# Perception Integration Notes

Status of the perception stack on this branch, what changed, and what still has
to be verified before it is trusted. Read the "Not yet verified" section first
if you are the one integrating this.

**Author's honest summary:** the code is complete and its geometry is unit-tested,
but **it has never been run in the simulator**. It has not been built against ROS
either — the machine it was written on has only ROS foxy/noetic while this
workspace targets Galactic/Humble. Treat every runtime claim below as a design
intent to be checked, not a measured result.

---

## What this is

`eufs_perception_baseline` implements the three-tier perception pipeline from
the IIT Bombay Racing Driverless paper (arXiv 2408.06113), adapted to this
project's own cone-pose detector.

```
/zed/left/image_rect_color
    -> yolov8_bbox_node        (YOLO26n-pose: box + keypoints, one forward pass)
    -> /yolo_bounding_boxes     eufs_msgs/BoundingBoxes
       /yolo_cone_keypoints     eufs_msgs/ConeKeypointsArray   (same header stamp)
    -> perception_baseline_node
       + /velodyne_points
       + /zed/{left,right}/image_rect_color
       + /zed/{left,right}/camera_info
       + timestamped /tf
    -> /cones                   eufs_msgs/ConeArrayWithCovariance, base_footprint
    -> eufs_graph_slam -> /graph_slam/map -> global_planner -> pure_pursuit -> /cmd
```

### Tier routing

A detection is claimed by the highest tier that can serve it, and is excluded
from the lower ones, so one physical cone never becomes two.

| Tier | Condition | Method | Paper's error |
|---|---|---|---|
| 1 | LiDAR points land in the bbox | LiDAR-camera association | 0.85 % |
| 2 | No LiDAR, cone upright and fully visible | `D = c · h_n^e` on bbox height | 4.49 % |
| 3 | No LiDAR, cone clipped/fallen/big/unknown | keypoints → PnP → right ROI → SIFT disparity | 6.39 % |

**Tier 3 detail that is easy to get wrong:** PnP does **not** produce the depth.
The paper tried that and rejected it (9.14 % error, its worst result). PnP exists
only to project the cone into the right image so SIFT searches a narrow, correct
ROI. The depth comes from **SIFT disparity**. Do not "simplify" this by reading
the PnP translation.

---

## What changed on this branch

### The detector replaces two models with one

The paper runs YOLOv5 for boxes, then RektNet separately on each cone crop for
keypoints — one inference plus N crop inferences. This project's YOLO26n-pose
weight emits the box, the class, and the cone keypoints in a **single forward
pass**, so the RektNet stage is gone.

`DetectorKeypointSource` presents those keypoints under RektNet's predictor
contract, which keeps the downstream PnP → right-ROI → SIFT chain byte-for-byte
the paper's. `rektnet_model_path` is now unused whenever `cone_keypoints_topic`
is set; the RektNet code stays only for a checkpoint-based fallback.

Weight: `models/cone_pose_8kpt/weights/best.pt` (6.2 MB, in-tree).
box mAP50 0.9597, pose mAP50-95 0.8772.

### Keypoint count is class-dependent

Verified across 35,783 dataset instances:

- **Small cones** (blue / yellow / orange): **6 keypoints** — three left/right stripe pairs
- **Big orange**: **8 keypoints** — four pairs

`rektnet.py` was generalized from a hardcoded 7 keypoints to any N ≥ 4. The
template is built to match whatever the detector emitted for that cone, because
feeding PnP a mismatched template count silently corrupts the pose.

### Cone dimensions are measured, not the FS-AI spec

Taken from the simulator's own meshes (`eufs_tracks/meshes/*.dae` POSITION
arrays — note the *normals* array is unit vectors and will mislead you — with
`model.sdf <scale>` and the DAE node `<matrix>` applied):

| | this simulator | FS-AI spec |
|---|---|---|
| small cone | 0.270 × **0.450** m | 0.228 × 0.325 m |
| big orange | 0.261 × **0.5255** m | 0.285 × 0.505 m |

The simulator's small cone is **38 % taller** than the competition spec. PnP
depth scales linearly with the template, so using spec values here would
under-read small-cone depth by ~28 % — and small cones are 94 % of instances.
**If you port this to the real car, you must swap these** for the competition
cone (`standard_cone_*`, `big_cone_*` in the config).

### The cone is a square pyramid

Its silhouette half-width depends on the yaw you view it from:

```
w(yaw) = half · (|cos yaw| + |sin yaw|)      # half face-on … √2·half corner-on
```

Modelling it as circular (fixed `half`) makes every rotated cone look too narrow,
and PnP can only explain that by pulling it toward the camera: measured **−31.7 %
depth error at 45°**, −24 % at 20°, correct only when perfectly face-on.

The yaw is not recoverable in practice — a cone is ~13 px tall at 15 m, where 1 px
of keypoint error is already ~1.1 m of depth. (An iterated solve-then-re-estimate-
yaw scheme was tried and measured no better than the fix below.) So the template
commits to the **expectation over yaw**, `mean(|cos|+|sin|) = 4/π ≈ 1.273`, which
centres the error rather than always under-reading: worst case **−31.7 % → −11.3 %**,
exact at 0–10°.

Because PnP only places the SIFT window, `rektnet_right_roi_padding_ratio` was
raised 0.08 → 0.15 so the residual error cannot walk that window off the cone.
A missed ROI loses the cone entirely; an oversized one costs a few extra features.

### Tier 2 was recalibrated for this camera

The paper's `D = 0.498 · h_n^-0.954` is an empirical fit to **their** ZED 2i and
**their** cone. The paper explicitly says the fit must be redone when the camera
changes. It never was.

On this simulator's ZED it **overestimates depth by ~50 %** (a 10 m cone reads
15.1 m), and the 30 m gate then silently truncates the map at ~20 m true range
instead of raising an error.

For a pinhole camera and a rigid cone the law is exact:

```
h_px = fy · H_cone / D    ⇒    D = (fy · H_cone / H_img) · h_n^-1
```

With `fy = 448.13 px` (1280×720, 110° HFOV) and the measured 0.450 m cone:
**c = 0.2801, e = −1.0**. The exponent is −1 exactly because the simulated camera
is undistorted; the paper's −0.954 absorbs their real lens and mounting.

Reproduce with `scripts/fit_mono_depth_curve.py --analytic --cone-height 0.450`.
On real hardware, re-fit empirically against `/ground_truth/cones` (the `--bag`
path in that script is a **stub** — it is not implemented).

### Bugs fixed along the way

- **`model_path` was `/home/dohyun/FS/artifacts/...`** — an absolute path that
  exists on nobody else's machine. The node raised at startup, so
  `perception_baseline_node` waited forever for `/yolo_bounding_boxes` and
  `/cones` published **nothing**. Weights are now in-tree and resolved against
  the package share.
- **`ultralytics` was pinned to `==8.4.60`**, which predates YOLO26 and cannot
  load the checkpoint. Now `>=8.4.90,<9`.
- **`_canonical_color` demoted `large_orange_cone` to a small orange cone**
  (it tested `"big" in name`), which would have selected the wrong PnP template.
- **Sensor-timing tolerances**: porting main's config also pulled in its tighter
  `timestamp_reset_threshold_sec: 0.1` / `max_future_stamp_lead_sec: 0.09`.
  fullstack runs 0.5 / 0.4 and its LiDAR works with them; a stamp further ahead
  of `/clock` than `max_future_stamp_lead_sec` is **dropped**, so tightening it
  silently discards early-delivered frames. Restored to 0.5 / 0.4, and
  `max_deferred_future_stamp_lead_sec` raised to 0.6 (it is the *outer* bound and
  must be ≥ `max_future_stamp_lead_sec`, or the node refuses to start).

---

## Tuning

Everything physically meaningful is in one block in
`config/perception_baseline.yaml`, marked `TUNING STARTS HERE`. Anything above it
is plumbing (topics, frames, buffer sizes, clock fences).

The four you are most likely to touch:

1. **Cone geometry** — `standard_cone_*`, `big_cone_*`. Stated **once**, shared by
   every tier. Depth scales linearly with these.
2. **Mono depth curve** — `monocular_depth_coefficient` / `_exponent`.
   Camera-specific; re-fit when optics, mounting, or cone size change.
3. **Detection gates** — `confidence_threshold`, `min_bbox_probability`,
   `good_cone_*`, `cluster_*`, `sparse_*`.
4. **Covariances** — `*_variance_x/_y` per depth source. These are the trust
   levels handed to GraphSLAM. **A tier whose covariance understates its real
   error corrupts the map rather than merely adding noise**, because the optimizer
   believes it. Roughly: LiDAR ≪ stereo < monocular.

---

## Interface to GraphSLAM

Checked against `eufs_graph_slam/src/graph_slam_node.cpp` on this branch:

| SLAM requires | perception emits | |
|---|---|---|
| `/cones`, `ConeArrayWithCovariance` | same | ok |
| all five colour lists | `_append_cone_by_color` fills all five | ok |
| finite `point.x/y` | non-finite rejected | ok |
| range within `[0.2, 30.0]` m | depth bounds `[0.5, 30.0]` | ok |
| covariance `[xx, xy, yx, yy]` | same order | ok |
| frame `base_footprint` | `output_frame: base_footprint` | ok |

fullstack's SLAM is the permissive one: it uses `stampOrNow()` and does not check
`frame_id`, so a stamp or frame mistake will **not** be caught for you.

One thing to watch: `/cones` is published **RELIABLE** while the SLAM subscribes
**BEST_EFFORT** (`rclcpp::SensorDataQoS()`). DDS allows this (a reliable writer
can serve a best-effort reader), but it is inconsistent with the other sensor
topics and worth confirming on the wire.

---

## NOT YET VERIFIED — do this first

Nothing here has run in the simulator. In priority order:

1. **Build it.** The two new messages (`ConeKeypoints`, `ConeKeypointsArray`) have
   never been compiled.
   ```bash
   colcon build --packages-up-to eufs_msgs eufs_perception_baseline eufs_graph_slam
   ```

2. **Run the ROS tests.** Five test files (`test_perception_three_tier.py`,
   `test_yolov8_bbox_node.py`, `test_perception_fusion_sparse.py`,
   `test_perception_marker_viz.py`, `test_ros_image_utils.py`) were **never
   executed** — they need `std_msgs`/`sensor_msgs`. Only the 37 pure-geometry
   tests ran, and they pass.
   ```bash
   colcon test --packages-select eufs_perception_baseline
   ```

3. **Confirm the pipeline actually produces cones.**
   ```bash
   ros2 launch eufs_launcher simulation.launch.py \
     track:=small_track rviz:=true launch_group:=default \
     perception:=true perception_bbox_source:=yolov8 \
     perception_publish_fusion_debug:=true

   ros2 topic hz /yolo_bounding_boxes
   ros2 topic hz /yolo_cone_keypoints     # new
   ros2 topic hz /cones
   ros2 topic hz /graph_slam/map
   ```
   If `/cones` is empty, check `/fusion/debug/rejections` first.

4. **Measure the tiers against ground truth.** Tier-3's real accuracy is
   **unknown**. The end-to-end test run during development compared SIFT depth
   against PnP depth on a *synthetic* right image built from that same PnP depth —
   it proves the plumbing, not the accuracy.
   ```bash
   ros2 run eufs_perception_baseline evaluate_perception_tiers.py --duration 60
   ```
   This reproduces the paper's Table 1 metric (range error vs `/ground_truth/cones`)
   and breaks it down by distance band, so you can see which tier is failing.
   Compare against: LiDAR 0.85 %, mono 4.49 %, stereo 6.39 %.

5. **Sanity-check the recalibrated mono curve on real data.** The `c = 0.2801,
   e = −1.0` values are derived analytically, not fitted. If the >10 m band in
   step 4 shows a systematic bias, re-fit.

6. **Verify Tier-1 did not regress.** It was deliberately left untouched — it
   works because you launch with `bbox_source:=yolov8`, where the launch file
   overrides `projection_model` to `pinhole`. (On the `simulated` bbox path,
   `projection_model: eufs_bbox` double-applies the body→optical rotation and
   Tier-1 silently produces nothing. That path is unfixed; if you ever use it,
   fix that first.)

---

## Known limitations

- **PnP depth still carries up to −11.3 % error** on a corner-on cone. It is used
  only to place the SIFT ROI, which the 0.15 padding should absorb — but this is
  an argument, not a measurement.
- **Keypoint pixel noise dominates PnP at range.** At 15 m a cone is ~13 px tall
  and 1 px of error is ~1.1 m of depth. This is inherent, and is precisely why the
  paper uses SIFT disparity rather than PnP for depth.
- **`scripts/fit_mono_depth_curve.py --bag` is a stub.** Only `--analytic` works.
- **The `simulated` bbox path is broken** (see step 6 above) and was left as-is
  because this team does not use it.
