"""ReKTNet keypoints and PnP-guided stereo matching.

The network topology and preprocessing in this module are adapted from the
Apache-2.0 licensed MIT/Delft ReKTNet reference implementation:
https://github.com/cv-core/MIT-Driverless-CV-TrainingInfra/tree/master/RektNet

IIT Bombay uses the predicted seven left-image keypoints to estimate a cone
pose, project the cone into the right image, and only then run SIFT inside the
two cone ROIs.  PnP is therefore an ROI-propagation stage here; stereo
disparity, not the PnP translation, remains the published Tier-3 depth source.
"""

from dataclasses import dataclass
import math
import os
import threading
from typing import Optional, Sequence, Tuple

import numpy as np

from eufs_perception_baseline.fusion_core import (
    StereoDepthEstimate,
    estimate_stereo_depth,
)

try:
    import torch
    import torch.nn as nn
except ImportError:  # Keep pure geometry/test imports usable without PyTorch.
    torch = None
    nn = None


REKTNET_INPUT_SIZE = 80
REKTNET_KEYPOINT_NAMES = (
    "top",
    "mid_L_top",
    "mid_R_top",
    "mid_L_bot",
    "mid_R_bot",
    "bot_L",
    "bot_R",
)

# PnP needs at least four correspondences; leave-one-out must not go below it.
_MIN_PNP_POINTS = 4

# Cone dimensions measured from this simulator's meshes, NOT the FS-AI spec.
# eufs_tracks/meshes/cone_blue.dae POSITION span x model.sdf <scale>, and
# cone_big.dae POSITION span x its DAE node <matrix> scale (0.00655834).
# The sim's small cone is 0.450 m tall -- 38% taller than the 0.325 m FS spec.
# PnP depth scales linearly with the template, so using spec values here would
# under-estimate small-cone depth by ~28%.
SIM_SMALL_CONE_HEIGHT_M = 0.450
SIM_SMALL_CONE_RADIUS_M = 0.135
SIM_BIG_CONE_HEIGHT_M = 0.5255
SIM_BIG_CONE_RADIUS_M = 0.1307

_ModuleBase = nn.Module if nn is not None else object


class RektNetResidualBlock(_ModuleBase):
    """Spatially preserving residual block from the public ReKTNet model."""

    def __init__(self, in_channels: int, out_channels: int) -> None:
        if nn is None:
            raise RuntimeError("ReKTNet requires PyTorch")
        super().__init__()
        self.conv1 = nn.Conv2d(
            in_channels,
            out_channels,
            kernel_size=3,
            stride=1,
            padding=2,
            dilation=2,
        )
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.relu1 = nn.ReLU()
        self.conv2 = nn.Conv2d(
            out_channels,
            out_channels,
            kernel_size=3,
            stride=1,
            padding=1,
        )
        self.bn2 = nn.BatchNorm2d(out_channels)
        self.shortcut_conv = nn.Conv2d(
            in_channels,
            out_channels,
            kernel_size=1,
            stride=1,
        )
        self.shortcut_bn = nn.BatchNorm2d(out_channels)
        self.relu2 = nn.ReLU()

    def forward(self, tensor):
        residual = self.relu1(self.bn1(self.conv1(tensor)))
        residual = self.bn2(self.conv2(residual))
        shortcut = self.shortcut_bn(self.shortcut_conv(tensor))
        return self.relu2(shortcut + residual)


class RektNet(_ModuleBase):
    """ReKTNet architecture compatible with ``pretrained_kpt.pt``."""

    def __init__(self, input_size: int = REKTNET_INPUT_SIZE) -> None:
        if nn is None:
            raise RuntimeError("ReKTNet requires PyTorch")
        super().__init__()
        if int(input_size) <= 0:
            raise ValueError("input_size must be positive")
        self.input_size = int(input_size)
        self.num_keypoints = len(REKTNET_KEYPOINT_NAMES)
        self.conv = nn.Conv2d(3, 16, kernel_size=7, stride=1, padding=3)
        self.bn = nn.BatchNorm2d(16)
        self.relu = nn.ReLU()
        self.res1 = RektNetResidualBlock(16, 16)
        self.res2 = RektNetResidualBlock(16, 32)
        self.res3 = RektNetResidualBlock(32, 64)
        self.res4 = RektNetResidualBlock(64, 128)
        self.out = nn.Conv2d(128, self.num_keypoints, kernel_size=1)

    def forward(self, tensor):
        activation = self.relu(self.bn(self.conv(tensor)))
        activation = self.res1(activation)
        activation = self.res2(activation)
        activation = self.res3(activation)
        activation = self.res4(activation)
        logits = self.out(activation)
        heatmaps = torch.softmax(
            logits.reshape(-1, self.num_keypoints, self.input_size**2),
            dim=2,
        ).reshape(-1, self.num_keypoints, self.input_size, self.input_size)

        # Match the public implementation exactly: coordinates are normalized
        # to [0, (size - 1) / size], not [0, 1].
        coordinates = torch.linspace(
            0.0,
            (self.input_size - 1.0) / self.input_size,
            self.input_size,
            dtype=heatmaps.dtype,
            device=heatmaps.device,
        )
        expected_y = (heatmaps.sum(3) * coordinates).sum(-1)
        expected_x = (heatmaps.sum(2) * coordinates).sum(-1)
        points = torch.stack((expected_x, expected_y), dim=-1)
        return heatmaps, points


@dataclass(frozen=True)
class RektNetPrediction:
    """Seven semantic keypoints in full-image pixel coordinates."""

    points: np.ndarray
    heatmap_peaks: np.ndarray


class DetectorKeypointSource:
    """Serve keypoints a pose detector already produced, as a RektNet predictor.

    IIT Bombay runs a separate RektNet pass over each cone crop to obtain the
    keypoints that PnP propagates into the right image.  This project's detector
    is a pose model: it emits the cone's keypoints in the same forward pass as
    the box, so the crop-and-infer stage is redundant work.  Presenting those
    keypoints through the same ``predict`` contract keeps the downstream
    PnP -> right-ROI -> SIFT chain byte-for-byte identical to the paper's.

    ``points`` are already full-image pixels, so no crop offset is reapplied.
    """

    def __init__(self, points, confidence=None) -> None:
        self._points = np.asarray(points, dtype=np.float64)
        if confidence is None:
            peaks = np.ones(self._points.shape[0], dtype=np.float64)
        else:
            peaks = np.asarray(confidence, dtype=np.float64).reshape(-1)
        self._peaks = peaks

    def predict(self, image, bbox) -> Optional["RektNetPrediction"]:
        # The detector localized these keypoints on the full frame it was given,
        # so image and bbox are accepted only to satisfy the predictor contract.
        del image, bbox
        points = self._points
        if (
            points.ndim != 2
            or points.shape[1] != 2
            or points.shape[0] < _MIN_PNP_POINTS
            or not np.all(np.isfinite(points))
        ):
            return None
        peaks = self._peaks
        if peaks.shape[0] != points.shape[0] or not np.all(np.isfinite(peaks)):
            return None
        return RektNetPrediction(points=points, heatmap_peaks=peaks)


@dataclass(frozen=True)
class PnPPose:
    """Object-to-left-camera pose selected by reprojection error."""

    rotation_vector: np.ndarray
    translation_vector: np.ndarray
    reprojection_error_px: float
    used_keypoint_indices: Tuple[int, ...]


@dataclass(frozen=True)
class RektNetStereoEstimate:
    """Paper-faithful Tier-3 result and its geometric audit evidence."""

    depth: StereoDepthEstimate
    left_keypoints: np.ndarray
    right_keypoints: np.ndarray
    right_bbox: Tuple[int, int, int, int]
    pnp_reprojection_error_px: float
    pnp_keypoint_count: int


class RektNetInference:
    """Load one checkpoint once and provide serialized crop inference."""

    def __init__(self, model_path: str, device: str = "") -> None:
        if torch is None:
            raise RuntimeError(
                "ReKTNet stereo is enabled but PyTorch is unavailable; "
                "run the perception node in the project conda environment"
            )
        path = os.path.abspath(os.path.expanduser(str(model_path)))
        if not os.path.isfile(path):
            raise RuntimeError(f"ReKTNet checkpoint does not exist: {path}")

        requested_device = str(device).strip()
        if requested_device:
            self.device = torch.device(requested_device)
        else:
            self.device = torch.device(
                "cuda:0" if torch.cuda.is_available() else "cpu"
            )

        checkpoint = _load_torch_checkpoint(path, self.device)
        state_dict = (
            checkpoint.get("model")
            if isinstance(checkpoint, dict)
            else None
        )
        if state_dict is None and _looks_like_state_dict(checkpoint):
            state_dict = checkpoint
        if not _looks_like_state_dict(state_dict):
            raise RuntimeError(
                "ReKTNet checkpoint must contain a 'model' state_dict or be a "
                "raw state_dict"
            )

        model = RektNet()
        try:
            model.load_state_dict(state_dict, strict=True)
        except (RuntimeError, TypeError, ValueError) as exc:
            raise RuntimeError(
                "ReKTNet checkpoint is incompatible with the public "
                "7-keypoint "
                f"architecture ({exc})"
            ) from exc
        self.model = model.to(self.device).eval()
        self.model_path = path
        self._lock = threading.Lock()

    def predict(
        self,
        image,
        bbox: Sequence[float],
    ) -> Optional[RektNetPrediction]:
        prepared = prepare_rektnet_crop(image, bbox)
        if prepared is None:
            return None
        crop, clipped_bbox = prepared
        tensor = torch.from_numpy(
            np.ascontiguousarray(crop.transpose(2, 0, 1)[np.newaxis])
        ).to(device=self.device, dtype=torch.float32)
        tensor = tensor / 255.0
        with self._lock, torch.inference_mode():
            heatmaps, normalized_points = self.model(tensor)

        points = normalized_points[0].detach().cpu().numpy().astype(np.float64)
        peaks = (
            heatmaps[0]
            .reshape(len(REKTNET_KEYPOINT_NAMES), -1)
            .max(dim=1)
            .values.detach()
            .cpu()
            .numpy()
            .astype(np.float64)
        )
        x1, y1, x2, y2 = clipped_bbox
        points[:, 0] = x1 + points[:, 0] * (x2 - x1)
        points[:, 1] = y1 + points[:, 1] * (y2 - y1)
        if not np.all(np.isfinite(points)) or not np.all(np.isfinite(peaks)):
            return None
        return RektNetPrediction(points=points, heatmap_peaks=peaks)


def prepare_rektnet_crop(image, bbox: Sequence[float]):
    """Clip a YOLO bbox and resize it to the reference 80x80 BGR input."""
    array = np.asarray(image)
    if array.ndim != 3 or array.shape[2] != 3 or array.size == 0:
        return None
    parsed = _finite_bbox(bbox)
    if parsed is None:
        return None
    x1, y1, x2, y2 = parsed
    x1 = max(0, int(math.floor(x1)))
    y1 = max(0, int(math.floor(y1)))
    x2 = min(array.shape[1], int(math.ceil(x2)))
    y2 = min(array.shape[0], int(math.ceil(y2)))
    if x2 <= x1 or y2 <= y1:
        return None
    crop = array[y1:y2, x1:x2]
    try:
        import cv2
    except ImportError:
        return None
    resized = cv2.resize(
        crop,
        (REKTNET_INPUT_SIZE, REKTNET_INPUT_SIZE),
        interpolation=cv2.INTER_LINEAR,
    )
    return resized, (x1, y1, x2, y2)


def cone_keypoint_template(
    height_m: float,
    radius_m: float,
) -> Optional[np.ndarray]:
    """Build the seven coplanar cone silhouette points in semantic order.

    The public model fixes the topology but neither cited paper publishes its
    metric template.  This parameterized straight-sided cone model places the
    two intermediate rows at one-third and two-thirds of the known height.
    """
    dimensions = (height_m, radius_m)
    if not all(
        math.isfinite(float(value)) and float(value) > 0.0
        for value in dimensions
    ):
        return None
    height = float(height_m)
    radius = float(radius_m)
    return np.asarray(
        [
            [0.0, 0.0, 0.0],
            [-radius / 3.0, height / 3.0, 0.0],
            [radius / 3.0, height / 3.0, 0.0],
            [-2.0 * radius / 3.0, 2.0 * height / 3.0, 0.0],
            [2.0 * radius / 3.0, 2.0 * height / 3.0, 0.0],
            [-radius, height, 0.0],
            [radius, height, 0.0],
        ],
        dtype=np.float64,
    )


def paired_cone_keypoint_template(
    height_m: float,
    radius_m: float,
    pair_count: int,
) -> Optional[np.ndarray]:
    """Build the L/R-paired cone silhouette template used by the pose detector.

    The detector emits ``2 * pair_count`` keypoints ordered top-down, left then
    right within each row -- matching the dataset's
    ``flip_idx = [1, 0, 3, 2, 5, 4, 7, 6]``.  Small cones use three stripe rows
    (6 points); big orange cones use four (8 points).

    Rows are placed on a straight-sided cone silhouette: a row at normalized
    height ``t`` above the apex has half-width ``t * radius``.  Rows are spaced
    evenly over the cone body, and the object frame has its origin at the apex
    with +y pointing down toward the base, which is the convention the image
    keypoints are ordered in.
    """
    if not all(
        math.isfinite(float(value)) and float(value) > 0.0
        for value in (height_m, radius_m)
    ):
        return None
    try:
        pairs = int(pair_count)
    except (TypeError, ValueError):
        return None
    if pairs < 2:
        return None

    height = float(height_m)
    radius = float(radius_m)
    points = []
    for row in range(pairs):
        # Rows sample the visible cone body; the apex itself is not a labeled
        # keypoint in this dataset, so start below it and end at the base.
        fraction = float(row + 1) / float(pairs)
        y = fraction * height
        half_width = fraction * radius
        points.append([-half_width, y, 0.0])
        points.append([half_width, y, 0.0])
    return np.asarray(points, dtype=np.float64)


def silhouette_half_width(base_half_width_m: float, yaw_rad: float) -> float:
    """Apparent half-width of a square-based pyramid seen at ``yaw_rad``.

    A Formula Student cone is a square pyramid, not a circular one.  Its
    silhouette is widest across the diagonal and narrowest across a face:

        w(yaw) = half * (|cos yaw| + |sin yaw|)

    so it sweeps from ``half`` (face-on) to ``sqrt(2) * half`` (corner-on).
    Treating the cone as circular pins ``w`` at ``half`` and therefore makes a
    rotated cone look narrower than it is, which PnP can only explain by pushing
    it closer to the camera.
    """
    return float(base_half_width_m) * (
        abs(math.cos(float(yaw_rad))) + abs(math.sin(float(yaw_rad)))
    )


def mean_silhouette_half_width(base_half_width_m: float) -> float:
    """Yaw-averaged silhouette half-width of a square-based cone.

    The yaw is unobservable in practice -- keypoint noise swamps it at the pixel
    sizes a cone actually occupies -- so the template commits to the expectation
    over yaw instead of guessing:

        mean over yaw of (|cos y| + |sin y|)  =  4 / pi

    Using the face-on half-width instead makes a rotated cone look narrower than
    it is, and PnP can only explain that by pulling it toward the camera: depth
    reads up to 32% short on a corner-on cone.  Committing to the mean bounds
    that to about 13% and centres the error near zero.
    """
    base = float(base_half_width_m)
    if not math.isfinite(base) or base <= 0.0:
        return 0.0
    return base * 4.0 / math.pi


def solve_rektnet_pnp(
    object_points,
    image_points,
    camera_matrix,
    distortion_coefficients=None,
    max_reprojection_error_px: float = 4.0,
    leave_one_out: bool = True,
) -> Optional[PnPPose]:
    """Solve N-point PnP, then retry every (N-1)-point subset if needed.

    N is whatever the keypoint model emits: 6 for standard cones, 8 for big
    orange.  Leave-one-out is only attempted while at least four correspondences
    remain, which is the PnP minimum.
    """
    object_array = np.asarray(object_points, dtype=np.float64)
    image_array = np.asarray(image_points, dtype=np.float64)
    matrix = np.asarray(camera_matrix, dtype=np.float64)
    if (
        object_array.ndim != 2
        or object_array.shape[1] != 3
        or image_array.ndim != 2
        or image_array.shape[1] != 2
        or object_array.shape[0] != image_array.shape[0]
        or object_array.shape[0] < _MIN_PNP_POINTS
        or matrix.size != 9
        or not np.all(np.isfinite(object_array))
        or not np.all(np.isfinite(image_array))
        or not np.all(np.isfinite(matrix))
        or not math.isfinite(float(max_reprojection_error_px))
        or float(max_reprojection_error_px) <= 0.0
    ):
        return None
    matrix = matrix.reshape(3, 3)
    distortion = _distortion_array(distortion_coefficients)
    if distortion is None:
        return None

    keypoint_count = int(object_array.shape[0])
    all_indices = tuple(range(keypoint_count))
    full_pose = _solve_pnp_candidate(
        object_array,
        image_array,
        matrix,
        distortion,
        all_indices,
    )
    if (
        full_pose is not None
        and full_pose.reprojection_error_px <= float(max_reprojection_error_px)
    ):
        return full_pose
    if not leave_one_out or keypoint_count - 1 < _MIN_PNP_POINTS:
        return None

    candidates = []
    for omitted in range(keypoint_count):
        indices = tuple(index for index in all_indices if index != omitted)
        pose = _solve_pnp_candidate(
            object_array,
            image_array,
            matrix,
            distortion,
            indices,
        )
        if pose is not None:
            candidates.append(pose)
    if not candidates:
        return None
    best = min(candidates, key=lambda pose: pose.reprojection_error_px)
    if best.reprojection_error_px > float(max_reprojection_error_px):
        return None
    return best


def project_keypoints_to_right(
    object_points,
    pose_left: PnPPose,
    right_from_left,
    right_camera_matrix,
    right_distortion_coefficients=None,
) -> Optional[np.ndarray]:
    """Compose object-to-left and left-to-right, then project all points."""
    try:
        import cv2
    except ImportError:
        return None
    points = np.asarray(object_points, dtype=np.float64)
    transform = np.asarray(right_from_left, dtype=np.float64)
    matrix = np.asarray(right_camera_matrix, dtype=np.float64)
    distortion = _distortion_array(right_distortion_coefficients)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or points.shape[0] < _MIN_PNP_POINTS
        or transform.shape != (4, 4)
        or matrix.size != 9
        or distortion is None
        or not np.all(np.isfinite(points))
        or not np.all(np.isfinite(transform))
        or not np.all(np.isfinite(matrix))
    ):
        return None
    left_rotation, _ = cv2.Rodrigues(pose_left.rotation_vector)
    right_rotation = transform[:3, :3] @ left_rotation
    right_translation = (
        transform[:3, :3] @ pose_left.translation_vector.reshape(3, 1)
        + transform[:3, 3:4]
    )
    points_right = (right_rotation @ points.T + right_translation).T
    if np.any(points_right[:, 2] <= 1.0e-6):
        return None
    right_rotation_vector, _ = cv2.Rodrigues(right_rotation)
    projected, _ = cv2.projectPoints(
        points,
        right_rotation_vector,
        right_translation,
        matrix.reshape(3, 3),
        distortion,
    )
    projected = projected.reshape(-1, 2)
    return projected if np.all(np.isfinite(projected)) else None


def bbox_from_keypoints(
    keypoints,
    image_size: Sequence[int],
    padding_ratio: float = 0.08,
    minimum_padding_px: float = 2.0,
) -> Optional[Tuple[int, int, int, int]]:
    """Create the clipped right-image bbox used by the paper's SIFT stage."""
    points = np.asarray(keypoints, dtype=np.float64)
    if (
        points.ndim != 2
        or points.shape[1] != 2
        or points.shape[0] < _MIN_PNP_POINTS
        or not np.all(np.isfinite(points))
        or len(image_size) < 2
    ):
        return None
    width = int(image_size[0])
    height = int(image_size[1])
    if width <= 0 or height <= 0:
        return None
    if (
        not math.isfinite(float(padding_ratio))
        or float(padding_ratio) < 0.0
        or not math.isfinite(float(minimum_padding_px))
        or float(minimum_padding_px) < 0.0
    ):
        return None
    span_x = float(np.ptp(points[:, 0]))
    span_y = float(np.ptp(points[:, 1]))
    pad_x = max(float(minimum_padding_px), float(padding_ratio) * span_x)
    pad_y = max(float(minimum_padding_px), float(padding_ratio) * span_y)
    x1 = max(0, int(math.floor(float(np.min(points[:, 0])) - pad_x)))
    y1 = max(0, int(math.floor(float(np.min(points[:, 1])) - pad_y)))
    x2 = min(width, int(math.ceil(float(np.max(points[:, 0])) + pad_x)))
    y2 = min(height, int(math.ceil(float(np.max(points[:, 1])) + pad_y)))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def rectified_right_from_left(
    left_projection,
    right_projection,
) -> Optional[np.ndarray]:
    """Recover the rectified left-camera to right-camera transform from P."""
    left = _projection_matrix(left_projection)
    right = _projection_matrix(right_projection)
    if left is None or right is None:
        return None
    translations = []
    for row, focal_column in ((0, 0), (1, 1), (2, 2)):
        left_scale = float(left[row, focal_column])
        right_scale = float(right[row, focal_column])
        if abs(left_scale) <= 1.0e-12 or abs(right_scale) <= 1.0e-12:
            translations.append(0.0)
        else:
            translations.append(
                float(right[row, 3]) / right_scale
                - float(left[row, 3]) / left_scale
            )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = translations
    baseline = float(np.linalg.norm(transform[:3, 3]))
    return transform if math.isfinite(baseline) and baseline > 0.0 else None


def estimate_rektnet_stereo_depth(
    estimator,
    left_image,
    right_image,
    left_bbox: Sequence[float],
    object_points,
    left_camera_matrix,
    left_distortion_coefficients,
    right_camera_matrix,
    right_distortion_coefficients,
    right_from_left,
    baseline_m: float,
    min_depth_m: float,
    max_depth_m: float,
    max_reprojection_error_px: float,
    right_roi_padding_ratio: float,
    min_heatmap_peak: float = 0.0,
    epipolar_tolerance_px: float = 2.0,
    slender_fraction: float = 0.5,
    ratio_threshold: float = 0.75,
    min_matches: int = 1,
    principal_point_offset_px: float = 0.0,
) -> Optional[RektNetStereoEstimate]:
    """Run the paper-faithful ReKTNet->PnP->right ROI->SIFT ordering."""
    if estimator is None or not hasattr(estimator, "predict"):
        return None
    prediction = estimator.predict(left_image, left_bbox)
    if prediction is None:
        return None
    if (
        not math.isfinite(float(min_heatmap_peak))
        or float(min_heatmap_peak) < 0.0
        or np.any(prediction.heatmap_peaks < float(min_heatmap_peak))
    ):
        return None
    pose = solve_rektnet_pnp(
        object_points,
        prediction.points,
        left_camera_matrix,
        left_distortion_coefficients,
        max_reprojection_error_px=max_reprojection_error_px,
        leave_one_out=True,
    )
    if pose is None:
        return None
    right_points = project_keypoints_to_right(
        object_points,
        pose,
        right_from_left,
        right_camera_matrix,
        right_distortion_coefficients,
    )
    if right_points is None:
        return None
    right_array = np.asarray(right_image)
    if right_array.ndim < 2:
        return None
    right_bbox = bbox_from_keypoints(
        right_points,
        (right_array.shape[1], right_array.shape[0]),
        padding_ratio=right_roi_padding_ratio,
    )
    if right_bbox is None:
        return None
    depth = estimate_stereo_depth(
        left_image,
        right_image,
        left_bbox,
        right_bbox,
        fx=float(np.asarray(left_camera_matrix).reshape(3, 3)[0, 0]),
        baseline_m=baseline_m,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
        epipolar_tolerance_px=epipolar_tolerance_px,
        slender_fraction=slender_fraction,
        ratio_threshold=ratio_threshold,
        min_matches=min_matches,
        principal_point_offset_px=principal_point_offset_px,
    )
    if depth is None:
        return None
    return RektNetStereoEstimate(
        depth=depth,
        left_keypoints=prediction.points,
        right_keypoints=right_points,
        right_bbox=right_bbox,
        pnp_reprojection_error_px=pose.reprojection_error_px,
        pnp_keypoint_count=len(pose.used_keypoint_indices),
    )


def _solve_pnp_candidate(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    indices: Tuple[int, ...],
) -> Optional[PnPPose]:
    try:
        import cv2
    except ImportError:
        return None
    selected = np.asarray(indices, dtype=np.int64)
    selected_object = object_points[selected]
    selected_image = image_points[selected]
    solutions = []

    # All published ReKTNet silhouette templates are planar.  IPPE exposes
    # both planar pose solutions instead of silently returning one ambiguous
    # branch; the physically valid minimum-reprojection solution is selected
    # below.  Fall back to ITERATIVE for non-planar custom templates or OpenCV
    # builds without solvePnPGeneric/IPPE.
    centered = selected_object - np.mean(selected_object, axis=0)
    is_planar = np.linalg.matrix_rank(centered) <= 2
    if (
        is_planar
        and hasattr(cv2, "solvePnPGeneric")
        and hasattr(cv2, "SOLVEPNP_IPPE")
    ):
        try:
            result = cv2.solvePnPGeneric(
                selected_object,
                selected_image,
                camera_matrix,
                distortion,
                flags=cv2.SOLVEPNP_IPPE,
            )
            if result and bool(result[0]):
                solutions.extend(zip(result[1], result[2]))
        except cv2.error:
            pass

    if not solutions:
        try:
            success, rotation_vector, translation_vector = cv2.solvePnP(
                selected_object,
                selected_image,
                camera_matrix,
                distortion,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        except cv2.error:
            return None
        if not success:
            return None
        solutions.append((rotation_vector, translation_vector))

    candidates = []
    for rotation_vector, translation_vector in solutions:
        projected, _ = cv2.projectPoints(
            selected_object,
            rotation_vector,
            translation_vector,
            camera_matrix,
            distortion,
        )
        residuals = np.linalg.norm(
            projected.reshape(-1, 2) - selected_image,
            axis=1,
        )
        error = float(np.mean(residuals))
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        camera_points = (
            rotation_matrix @ selected_object.T
            + np.asarray(translation_vector).reshape(3, 1)
        ).T
        if (
            not math.isfinite(error)
            or not np.all(np.isfinite(rotation_vector))
            or not np.all(np.isfinite(translation_vector))
            or np.any(camera_points[:, 2] <= 1.0e-6)
        ):
            continue
        candidates.append(
            PnPPose(
                rotation_vector=np.asarray(
                    rotation_vector,
                    dtype=np.float64,
                ).reshape(3, 1),
                translation_vector=np.asarray(
                    translation_vector,
                    dtype=np.float64,
                ).reshape(3, 1),
                reprojection_error_px=error,
                used_keypoint_indices=indices,
            )
        )
    return (
        min(candidates, key=lambda pose: pose.reprojection_error_px)
        if candidates
        else None
    )


def _load_torch_checkpoint(path: str, device):
    try:
        return torch.load(path, map_location=device, weights_only=True)
    except TypeError:  # PyTorch < 2.0 has no weights_only argument.
        return torch.load(path, map_location=device)
    except Exception as exc:
        raise RuntimeError(
            f"Failed to load ReKTNet checkpoint {path}: {exc}"
        ) from exc


def _looks_like_state_dict(value) -> bool:
    return (
        isinstance(value, dict)
        and bool(value)
        and all(isinstance(key, str) for key in value)
        and all(hasattr(tensor, "shape") for tensor in value.values())
    )


def _distortion_array(coefficients) -> Optional[np.ndarray]:
    if coefficients is None:
        return np.zeros((5, 1), dtype=np.float64)
    array = np.asarray(coefficients, dtype=np.float64).reshape(-1, 1)
    if array.size == 0:
        return np.zeros((5, 1), dtype=np.float64)
    return array if np.all(np.isfinite(array)) else None


def _projection_matrix(values) -> Optional[np.ndarray]:
    array = np.asarray(values, dtype=np.float64)
    if array.size != 12:
        return None
    matrix = array.reshape(3, 4)
    return matrix if np.all(np.isfinite(matrix)) else None


def _finite_bbox(values: Sequence[float]):
    if values is None or len(values) != 4:
        return None
    try:
        x1, y1, x2, y2 = (float(value) for value in values)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
        return None
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2
