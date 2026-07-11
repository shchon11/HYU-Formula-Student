# IIT Bombay-Inspired Perception Baseline Design

Reference paper:

- IIT Bombay Racing Driverless: Autonomous Driving Stack for Formula Student AI
- arXiv: https://arxiv.org/abs/2408.06113

## Scope and Fidelity

This package implements a prioritized three-tier pipeline inspired by the paper.
It does **not** claim bit-for-bit or model-for-model reproduction of the paper.

The detector is deliberately replaced with the team's FSOCO-fine-tuned YOLOv8n
model. This substitution changes the 2D detector, but preserves the downstream
contract: each detection supplies a pixel bounding box, confidence, and cone
class/color.

The paper's stereo tier depends on RekTNet, seven cone keypoints, a cone 3D
template, and the corresponding trained assets. Those assets are not present in
this workspace. Consequently, this package uses a central-slender-crop SIFT
stereo estimator and labels that path `stereo_sift`. It is a paper-inspired
fallback, not an exact reproduction of the RekTNet/keypoint/template pipeline.

## SLAM Output Contract

HYU graph SLAM subscribes to:

- Topic: `/cones`
- Type: `eufs_msgs/msg/ConeArrayWithCovariance`
- Frame: `base_footprint`

The message groups cones into `blue_cones`, `yellow_cones`, `orange_cones`,
`big_orange_cones`, and `unknown_color_cones`. Each
`ConeWithCovariance` contains:

- `point.x`: forward position in `base_footprint`
- `point.y`: left position in `base_footprint`
- `point.z`: `0.0`
- `covariance`: `[xx, xy, yx, yy]`

## Prioritized Three-Tier Routing

Detections are processed in strict priority order. Once a detection is assigned
by a tier, it is excluded from all lower tiers. A detection that has valid
LiDAR support but loses a one-to-one competition is also excluded from visual
fallback, so one physical cone cannot be duplicated with an unrelated depth.

1. **LiDAR-camera association**
   - Remove the ground with a tilt-constrained RANSAC plane.
   - Form cone-sized LiDAR clusters and project them into the bbox camera.
   - Associate a detection using only the LiDAR points supported by that bbox.
   - If no global cluster matches, try the bbox-guided sparse-LiDAR path.
2. **Normalized-bbox monocular depth**
   - Only unmatched detections classified as upright and fully visible enter
     this tier.
   - Estimate optical depth from the normalized bbox-height curve below.
3. **Paper-inspired SIFT stereo depth**
   - Only unmatched detections classified as clipped, wide/fallen, or otherwise
     unsuitable for the monocular curve enter this tier.
   - Match SIFT features from a slender central crop against a physically valid
     rectified-right-image search region, then convert robust disparity to
     depth.

If a tier lacks valid calibration, synchronized input, geometrically valid
depth, or sufficient feature support, it emits no cone for that detection. It
does not fabricate a distance or silently switch to a different stereo
descriptor.

## Tier 1: LiDAR-Camera Association

Inputs:

- YOLOv8 or simulator bounding boxes
- `PointCloud2`
- primary bbox-camera `CameraInfo` (simulator right or YOLO left)
- timestamped LiDAR-to-camera and LiDAR-to-`base_footprint` TF

Processing:

1. Restrict points to the configured spatial ROI and remove the vehicle body.
2. Fit a near-horizontal ground plane with deterministic RANSAC and remove its
   inliers.
3. Cluster the retained points with cone-size constraints.
4. Transform and project cluster points into the bbox camera image.
5. Associate bbox-supported points to detections, using detection confidence in
   competing-assignment scoring.
6. Use the supported 3D points, rather than the entire original cluster, to
   compute the cone centroid.
7. Try range-dependent bbox-guided sparse association for still-unmatched
   detections.

The RANSAC normal tilt constraint prevents a large vertical wall from being
selected as ground. If plane estimation is underconstrained, the pure geometry
helper reports no plane without inventing one. The ROS node then applies the
configured legacy `ground_min_z` filter; downstream cone geometry and bbox
association still apply. This fixed-height contingency is a project fallback,
not a paper claim.

## Tier 2: Normalized-Bbox Monocular Depth

The implemented curve is:

```text
h_n = bbox_height_px / image_height_px
D   = 0.498 * h_n^(-0.954)
```

`D` is treated as optical-axis depth and the bbox center is back-projected with
the primary-camera matrix before transforming it to `base_footprint`.

The paper reports the fitted power law but does not make the bbox-height unit
unambiguous enough to reconstruct the original calibration from this workspace.
This implementation therefore makes an explicit assumption: `h` is normalized
by image height. The coefficients are only a baseline under that assumption.
Changing image preprocessing, resolution/cropping, camera intrinsics, camera
height, camera pitch, or cone geometry requires collecting new calibration data
and refitting the curve.

The upright/full-visibility classifier is a deterministic heuristic based on
bbox height-to-width ratio and border margin. It is not a learned fallen-cone
classifier.

## Tier 3: Paper-Inspired SIFT Stereo

The local fallback expects a left-image detection, rectified left/right images,
and valid stereo calibration. Simulator bboxes are right-image detections, so
launch disables this tier in simulated mode. In YOLO-left mode it:

1. Crops the horizontal center of the left bbox to reduce background matches.
2. Derives a right-image search interval from focal length, baseline, and the
   configured depth bounds.
3. Computes SIFT features and applies ratio, reciprocal-best, epipolar,
   uniqueness, and depth-range checks.
4. Rejects disparity outliers with a median/MAD filter.
5. Converts the median positive disparity using `Z = f_x * B / d`.

The stereo baseline is resolved from rectified projection matrices when
available, then from timestamped TF, with the configured baseline as the final
explicit fallback. An OpenCV build without SIFT, too few robust matches,
non-positive disparity, an out-of-range depth, image-shape mismatch, or invalid
calibration disables this estimate for the affected detection.

## Time, TF, and Calibration Contract

Sensor data is matched through bounded integer-nanosecond timestamp buffers.
The node compares the oldest bbox and point-cloud queue fronts and uses the
later front as the anchor. It waits until the opposite stream reaches that
timestamp, then consumes the nearer predecessor/successor sample one-to-one.
Exact timestamp matches are safe immediately, and skipped samples from the
faster stream are discarded. This symmetric rule prevents a stale 50 Hz bbox
from consuming a scarce 10 Hz cloud and also handles a detector that runs more
slowly than LiDAR. Left and right images are independently matched to the bbox
timestamp with the tighter image tolerance.

Image history is sized separately from the cloud/bbox queues. The default 64
frames retain about 1.07 seconds at 60 Hz, covering the measured roughly
0.53-second YOLO delay with margin. Sensor message ordering never defines a
clock epoch: out-of-order samples are dropped without clearing other streams.
An authoritative backward ROS clock jump clears every synchronization buffer
and recreates the tf2 listener, flushing the volatile `/tf` subscription queue.
On the normal unregister path it clears and reuses the existing Galactic tf2
buffer, which retains cached static transforms even if a one-shot `/tf_static`
writer has already exited. If unregister fails, a fresh buffer isolates the live
old subscription. The callback defensively ignores positive clock ticks.
Acquisition stamps must be canonical, nonzero ROS `Time` values before integer
nanosecond conversion. An epoch-start lower fence rejects pre-rewind data. For a
rollback, the node reconstructs the pre-rollback clock high watermark. During
replay it permits only the configured bounded future lead (90 ms by default) to
absorb DDS clock-versus-sensor callback skew, while an exclusive upper fence
rejects stamps at or beyond the old watermark. The fence is removed when ROS
time catches up. Nested rollbacks retain each active watermark and release the
fences in time order. Timestamp-only inputs still cannot distinguish old payloads
whose overlapping timestamp falls within the accepted replay window.

The bbox acquisition time is the canonical timestamp of each output array.
LiDAR points are transformed from cloud time into the bbox-time camera and
output frames through the configured fixed TF frame (`map` by default), while
visual camera-to-output TF uses the bbox timestamp directly. Consequently all
measurements in one `ConeArrayWithCovariance` share the timestamp carried by
its header instead of mixing cloud-time and image-time base coordinates.
When GraphSLAM owns `map -> odom`, deployment overrides the fixed frame to
`odom` so an optimization correction jump is not mistaken for ego motion.
A pair is removed from the buffers only after fusion succeeds, so a top-level
TF or required primary-camera calibration failure remains retryable. A rejected
per-detection mono/stereo estimate fails closed for that detection; it does not
hold the whole sensor pair indefinitely. Camera dimensions, matrices, image
dimensions, projection model, and configured numeric ranges are validated at
the node boundary.

Calibration remains a deployment responsibility. In particular, rectification,
left/right intrinsics, stereo baseline, LiDAR-camera extrinsics, camera pose,
the dynamic global-to-`base_footprint` TF history, and the mono power-law calibration
must describe the same physical setup.

## Covariance Policy

Covariance is selected by the source that produced the 3D estimate:

- dense LiDAR-camera cluster: `fused_variance_x/y`
- bbox-guided sparse LiDAR: `sparse_variance_x/y`
- normalized-bbox mono: `monocular_variance_x/y`
- SIFT stereo: `stereo_variance_x/y`

The configured range-dependent term is then added and the result is clamped by
`min_variance`. Defaults intentionally assign more uncertainty to vision-only
estimates than to LiDAR-supported estimates. They are initial variances in
square metres, not measured sensor-noise guarantees, and must be tuned from
recorded data before relying on them for competition SLAM.

## Failure and Evidence Boundary

The package fails closed at the estimate boundary: an invalid mono/stereo result
does not become a cone, and missing SIFT support does not fall back to ORB.
Ground-plane estimation has the explicit fixed-height contingency described
above, so RANSAC failure does not fabricate a plane but can still use the legacy
`ground_min_z` policy.

Unit tests cover the pure RANSAC, normalized mono geometry, stereo geometry and
feature matching, timestamp buffer, ROS image conversion, and LiDAR association
contracts. Those tests do not replace a recorded-bag or live-vehicle validation
of detector recall, calibration, depth error, latency, or SLAM consistency.

## Oracle Adapter

The oracle adapter converts a simulator cone topic directly to the `/cones`
contract. It is useful for SLAM wiring checks, but bypasses detection and depth
estimation and therefore provides no evidence about perception performance.
