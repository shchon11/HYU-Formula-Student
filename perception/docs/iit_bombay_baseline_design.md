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

The stereo tier now restores the paper's complete method ordering and labels its
output `stereo_rektnet_pnp_sift`: public ReKTNet 7-keypoint inference, robust
PnP, left-to-right keypoint/ROI projection, and slender-ROI SIFT disparity. The
public model topology is checkpoint-compatible with MIT/Delft's Apache-2.0
implementation. IIT's extra 1,000-image weight and exact metric template were
not published, so deployment still requires a compatible locally trained/public
checkpoint and a vehicle-specific measured template/calibration.

## SLAM Output Contract

HYU graph SLAM subscribes to:

- Topic: `/perception/cones`
- Type: `hyu_msgs/msg/ConeArrayWithCovariance`
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
   - The single fitted curve is restricted to calibrated standard cone classes
     (`blue`, `yellow`, and standard `orange`). Large-orange and unknown classes
     are routed to stereo because their physical height is different.
   - Estimate optical depth from the normalized bbox-height curve below.
3. **ReKTNet/PnP-guided SIFT stereo depth**
   - Only unmatched detections classified as clipped, wide/fallen, or otherwise
     unsuitable for the monocular curve enter this tier.
   - Predict the seven semantic left-image keypoints with ReKTNet.
   - Solve 7-point PnP; if its reprojection error fails, evaluate all seven
     leave-one-out 6-point candidates and keep the valid minimum-error pose.
   - Compose the known left-to-right calibration, project the 3D cone template,
     and form the right ROI before running SIFT in both slender crops.
   - Convert robust feature disparity to depth. The optional side-clipped mono
     recovery remains available but is disabled in the paper-faithful default.

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

Applying this standard-cone curve to a 0.53 m large-orange cone as if it were a
roughly 0.31 m standard cone would create a large systematic depth bias. The
implementation therefore fails closed or uses stereo for that class instead of
claiming a calibrated monocular estimate.

## Tier 3: ReKTNet/PnP-Guided SIFT Stereo

The local fallback expects a left-image detection, rectified left/right images,
and valid stereo calibration. Simulator bboxes are right-image detections, so
launch disables this tier in simulated mode. In YOLO-left mode it:

1. Resizes each clipped YOLO bbox directly to 80x80 BGR, scales it by 1/255,
   and runs the public seven-heatmap ReKTNet topology plus spatial soft-argmax.
2. Uses the semantic order `top`, upper L/R, lower L/R, bottom L/R with a metric
   7x3 cone template and the rectified left projection in OpenCV PnP.
3. Evaluates both planar IPPE pose branches, keeps the positive-depth minimum
   reprojection solution, applies the upstream 7-to-6 keypoint fallback, and
   rejects poses that still exceed `rektnet_pnp_max_reprojection_error_px`.
4. Composes object-to-left with left-to-right extrinsics, projects all seven
   points using the rectified right projection, and creates a clipped right
   bbox. Raw-image distortion is not reapplied to rectified pixels.
5. Computes SIFT only in the slender left bbox and projected right bbox, then
   applies ratio, reciprocal-best, epipolar, uniqueness, and depth-range gates.
6. Selects the single best remaining SIFT descriptor pair, as reported most
   accurate in the IIT evaluation, and converts its positive disparity using
   `Z = f_x * B / d`.

The stereo baseline is resolved from rectified projection matrices when
available, then from timestamped TF, with the configured baseline as the final
explicit fallback. An OpenCV build without SIFT, no valid best match,
non-positive disparity, an out-of-range depth, image-shape mismatch, or invalid
calibration disables this estimate for the affected detection.

ReKTNet and SIFT are consecutive stages, not alternatives: PnP propagates a
right ROI and SIFT supplies the final stereo disparity. The official upstream
checkpoint URL currently returns HTTP 403, while IIT's fine-tuned weight is not
public. Consequently the node requires an explicit compatible checkpoint and
fails startup rather than silently returning to bbox-only SIFT. The configured
EUFS template places silhouette rows at one-third/two-thirds height; replace it
with measurements of the real cone/stripe geometry before vehicle deployment.

## Time, TF, and Calibration Contract

Sensor data is matched through bounded integer-nanosecond timestamp buffers.
The node compares the oldest bbox and point-cloud queue fronts and uses the
later front as the anchor. It waits until the opposite stream reaches that
timestamp, then consumes the nearer predecessor/successor sample one-to-one.
Exact timestamp matches are safe immediately, and skipped samples from the
faster stream are discarded. This symmetric rule prevents a stale 50 Hz bbox
from consuming a scarce 10 Hz cloud and also handles a detector that runs more
slowly than LiDAR. Left and right images are first paired one-to-one, preferring
equal acquisition stamps and otherwise accepting a mature pair within the image
tolerance. Fusion selects this joint stereo frame by bbox timestamp, so it cannot
combine independently nearest images from different acquisition cycles.
When the bbox/cloud pair arrives first, fusion waits for the delayed right stream
only until it passes the target stamp or the bounded `stereo_pair_wait_sec`
deadline. A missing camera cannot block the pipeline indefinitely.

Image history is sized separately from the cloud/bbox queues. The default 64
frames retain about 1.07 seconds at 60 Hz, covering the measured roughly
0.53-second YOLO delay with margin. Sensor message ordering never defines a
clock epoch: out-of-order samples are dropped without clearing other streams.
An authoritative backward ROS clock jump clears every synchronization buffer
and both cached `CameraInfo` messages, then recreates the tf2 listener, flushing
the volatile `/tf` subscription queue.
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
During a normal clock epoch, callback skew above 90 ms and up to
`max_deferred_future_stamp_lead_sec` (300 ms by default) is stored in the bounded
buffers and processed only after ROS time catches up. With simulated time, data
received before the first nonzero `/clock` sample is also retained; the strict
epoch gate is applied once the clock becomes usable.

YOLO inference and fusion compute run in separate single-slot latest-only
workers so slow CPU/GPU work cannot block the ROS executor from observing a
clock reset. All ROS topic commits remain serialized on the
`SingleThreadedExecutor` through a `GuardCondition`. Every job captures a clock
generation before compute; the pre-jump callback increments that generation and
purges pending work. A completion from an older generation is discarded before
any bbox, cone, or debug publication. Fusion additionally checks the identity of
the original queue entries, preventing a stale completion from consuming a new
rosbag loop message that reuses the same numeric timestamp.
Successful completions are also held until their acquisition timestamp trails
the active ROS clock by `output_commit_settle_sec`. This short commit-settle
fence lets an already queued clock rollback callback win before output is
published even when the executor reports the worker guard condition first.

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

Two models, split by what the tier actually measured.

**LiDAR tiers** report a near-isotropic constant, selected by source:

- dense LiDAR-camera cluster: `fused_variance_x/y`
- bbox-guided sparse LiDAR: `sparse_variance_x/y`
- LiDAR cluster the camera never confirmed: `lidar_only_variance_x/y`

The configured range-dependent term (`range_variance_scale`) is then added and
the result is clamped by `min_variance`. LiDAR ranges directly to ~1 %, so the
circle these describe is roughly the truth.

**Vision tiers** (mono, horizontal-clip mono, SIFT stereo) derive a
bearing-aligned ellipse from the measurement instead, because their error is
not isotropic and not axis-aligned:

    sigma_lat = D * sigma_u_px / fx                  bearing; ~5 cm at 15 m
    sigma_lon = |e| * sigma_h_px / h_px * D          mono; ~1.8 m at 15 m
    sigma_lon = D^2 * sigma_d_px / (fx * B)          stereo
    Sigma     = R(atan2(y, x)) @ diag(lon^2, lat^2) @ R(...).T

The two axes differ by ~35x in sigma at 15 m, so a single number was
simultaneously ~160x too pessimistic across the corridor and ~7x too optimistic
along it. Cones lie *along* the track boundary, so the longitudinal error
slides a cone on the boundary it already defines while the lateral error is
what bends it -- reporting them separately is what lets SLAM
(`use_cone_covariance`, full 2x2 landmark covariance) weight them apart, and is
why the monocular tier no longer needs a bbox-height cut to hide its far cones.

Three pixel sigmas (`sigma_u_px`, `sigma_h_px`, `sigma_d_px`) generate all of
it, and are tuned against the per-tier `lat z^2` / `lon z^2` columns that
`evaluate_perception_tiers.py` reports -- each should average 1.0. Set
`honest_vision_covariance: false` to fall back to the retired per-tier
constants for an A/B.

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

The oracle adapter converts a simulator cone topic directly to the `/perception/cones`
contract. It is useful for SLAM wiring checks, but bypasses detection and depth
estimation and therefore provides no evidence about perception performance.
