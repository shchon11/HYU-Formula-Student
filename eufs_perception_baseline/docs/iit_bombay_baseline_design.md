# IIT Bombay Inspired Perception Baseline Design

Reference paper:

- IIT Bombay Racing Driverless: Autonomous Driving Stack for Formula Student AI
- arXiv: https://arxiv.org/abs/2408.06113

## SLAM Output Contract

HYU graph SLAM subscribes to:

- Topic: `/cones`
- Type: `eufs_msgs/msg/ConeArrayWithCovariance`
- Frame: `base_footprint`

The message contains grouped cone arrays:

- `blue_cones`
- `yellow_cones`
- `orange_cones`
- `big_orange_cones`
- `unknown_color_cones`

Each cone is:

- `point.x`: forward position in the car frame
- `point.y`: left position in the car frame
- `point.z`: set to `0.0`
- `covariance`: `[xx, xy, yx, yy]`

## Recommended Baseline Architecture

The paper uses a three-tier perception strategy:

1. LiDAR-camera fusion for cones with LiDAR returns.
2. Monocular depth for upright and fully visible cones without LiDAR support.
3. Stereo depth for partially visible or fallen cones.

For this workspace, start smaller:

1. Publish the exact SLAM contract from the beginning.
2. Use an oracle/simulator cone adapter for integration testing.
3. Implement LiDAR-camera fusion as the first real detector.
4. Add mono/stereo fallback only after the fused baseline is stable.

## First Real Detector

Inputs:

- Left camera image
- PointCloud2
- Camera intrinsics
- LiDAR to camera extrinsics
- Camera or detector output with cone class and bounding boxes

Pipeline:

1. Detect cone bounding boxes and class on the image.
2. Remove the ground plane from the point cloud.
3. Cluster non-ground points using cone-size constraints.
4. Transform LiDAR cluster points into the camera frame.
5. Project points into image pixels.
6. Associate clusters to bounding boxes by overlap.
7. Use the associated 3D cluster centroid as cone position.
8. Transform the centroid to `base_footprint`.
9. Publish grouped cones on `/cones`.

## Covariance Policy

Use small covariance for LiDAR-camera fused cones and larger covariance for
vision-only cones.

Suggested initial values:

- LiDAR-camera fused cone: `[0.04, 0.0, 0.0, 0.04]`
- Mono/stereo cone: `[0.16, 0.0, 0.0, 0.25]`
- Unknown fallback: `[0.25, 0.0, 0.0, 0.25]`

These are variances in square meters. They should be tuned after SLAM behavior
is observed.

## Current Package State

`perception_baseline_node` currently:

- Publishes `/cones` with the correct SLAM message type.
- Supports LiDAR-camera fusion baseline mode.
- Supports simulator/oracle adapter mode for SLAM wiring checks.
- Uses `/noisy_bounding_boxes` as the temporary detector output.
- Uses `/velodyne_points`, `/custom_camera_info`, and `/tf` for metric fusion.
- Publishes fused cone positions in `base_footprint`.

This is intentional. It fixes the SLAM-facing interface first and gives the
perception team a clean path to replace the simulator bbox source with a real
YOLO detector later.
