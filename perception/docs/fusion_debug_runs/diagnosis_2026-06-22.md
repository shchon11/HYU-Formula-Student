# Fusion Debug Diagnosis - 2026-06-22

## Scope

Executed the scenarios from `eufs_perception_baseline/docs/fusion_debug_scenarios.md`
against the currently running host ROS2 simulation/perception graph. No runtime code was changed.

Evidence files:

- `scenario_a_graph.txt`
- `scenario_b_yolo_echo.txt`
- `fusion_outputs_echo.txt`
- `scenario_cde_debug_echo.txt`
- `current_params.txt`
- `parsed_summary.txt`

## Scenario A: ROS graph

PASS.

Observed nodes:

- `/perception_baseline_node`
- `/yolov8_bbox_node`
- `/rviz`
- `/gazebo`

Observed required topics:

- `/yolo_bounding_boxes`
- `/yolo_bounding_boxes/debug_image`
- `/velodyne_points`
- `/zed/left/camera_info`
- `/zed/left/image_rect_color`
- `/cones`
- `/cones/viz`
- `/fusion/debug/bbox_support`
- `/fusion/debug/cluster_candidates`
- `/fusion/debug/rejections`
- `/fusion/debug/roi_points`
- `/fusion/debug/sparse_support_points`

## Scenario B: YOLO bbox

PASS.

`/yolo_bounding_boxes` has one publisher and one subscriber. The captured message
contained 17 bbox detections in one frame. Example probabilities were high:

- yellow: `0.894`
- big_orange: `0.887`
- blue: `0.882`
- blue: `0.876`
- yellow: `0.868`

This means the detector is active and fusion is receiving bbox input.

## Scenario C: LiDAR raw support

PARTIAL PASS.

`/fusion/debug/rejections` showed both supported and unsupported bboxes:

- Some bboxes had strong support, for example `raw=65 roi=44 cl=34 assigned_cluster`.
- Several bboxes had `raw=0 roi=0 cl=0 no_lidar_support`.
- Several bboxes had low raw support, for example `raw=2 roi=1 cl=0 insufficient_cluster_support`.

Interpretation:

- The YOLO-LiDAR projection contract is not globally broken, because many bboxes have high `raw`.
- The missed cones include a mix of true no-LiDAR-support cases and sparse-support cases.
- This is not primarily a missing-topic or wrong-camera-frame failure.

## Scenario D: ROI/self-mask/ground filtering

PARTIAL FAIL.

One repeated rejection pattern was:

```text
raw=3 roi=0 cl=0 rejected_by_roi_or_self_ground
```

Interpretation:

- At least one bbox has LiDAR points projected into the bbox before ROI filtering.
- Those points are then removed by ROI, self-mask, or ground filtering.
- This path can explain some missed near/low cones, but it is not the only failure mode.

Current runtime parameters:

```text
self_mask_enabled: True
sparse_near_min_points: 4
sparse_mid_min_points: 3
sparse_far_min_points: 2
sparse_bbox_margin_px: 4.0
sparse_bbox_margin_ratio: 0.15
```

## Scenario E: cluster/sparse support

FAIL for several detections.

Repeated patterns:

```text
raw=4 roi=2 cl=0 insufficient_cluster_support
raw=3 roi=1 cl=0 insufficient_cluster_support
raw=2 roi=1 cl=0 insufficient_cluster_support
```

Published `/cones` was stable but sparse:

```text
blue_cones: 1
yellow_cones: 0
orange_cones: 0
big_orange_cones: 2
```

Interpretation:

- Fusion is working, but only three cones are consistently published.
- Multiple YOLO detections have LiDAR support after ROI, but not enough clustered support.
- Because `sparse_near_min_points=4` and `sparse_mid_min_points=3`, detections with only 1-2 ROI points are rejected.
- This confirms the earlier hypothesis: the next tuning target should be sparse association thresholds, not broad TF/projection rewiring.

## Scenario F: cluster observation

PARTIAL PASS.

Cluster candidates exist for strong near cones, and those become assigned clusters.
However, lower-support cones remain at `cl=0`.

Interpretation:

- Cluster logic is not dead.
- DBSCAN-style cluster matching works for cones with enough points.
- The misses are mostly low-support detections where sparse association should be allowed to recover more cones.

## Diagnosis

Current dominant causes:

1. Low LiDAR support in YOLO bbox: `raw=0` for several far/small detections.
2. ROI/self-mask/ground rejection for at least one detection: `raw>0, roi=0`.
3. Sparse/cluster threshold too strict for low-support detections: repeated `raw>0, roi>0, cl=0`.

Best next experiment:

Run perception with sparse thresholds relaxed:

```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7 \
  ros2 launch eufs_perception_baseline perception_baseline.launch.py \
  bbox_source:=yolov8 \
  use_sim_time:=true \
  python_executable:=/home/dohyun/anaconda3/envs/eufs/bin/python3 \
  publish_fusion_debug:=true \
  publish_yolo_debug_image:=true \
  sparse_near_min_points:=3 \
  sparse_mid_min_points:=2 \
  sparse_far_min_points:=2 \
  sparse_bbox_margin_px:=8.0 \
  sparse_bbox_margin_ratio:=0.25
```

Expected improvement:

- Detections currently rejected as `raw=4 roi=2 cl=0`, `raw=3 roi=1 cl=0`,
  and `raw=2 roi=1 cl=0` should either become `assigned_sparse` or show increased
  support in `/fusion/debug/sparse_support_points`.

Risk:

- If false positives increase in RViz, reduce bbox margin first before raising cluster looseness.
- Do not tune `cluster_min_points` yet; the launch file does not currently expose it,
  and lowering it is more likely to create false positives than sparse threshold tuning.

## Cleanup

All diagnostic commands were bounded by `timeout`; no extra launch process was started,
and no ROS node was killed or modified.

