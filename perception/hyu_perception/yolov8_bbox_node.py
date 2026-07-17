import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import rclpy
from hyu_msgs.msg import (
    BoundingBox,
    BoundingBoxes,
    ConeKeypoints,
    ConeKeypointsArray,
)
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from hyu_perception.ros_image_utils import (
    image_message_to_numpy,
    numpy_to_image_message,
)
from hyu_perception.latest_only_worker import (
    LatestOnlyWorker,
    WorkerCompletion,
)
from hyu_perception.yolov8_bbox_utils import (
    detections_from_ultralytics_results,
    looks_like_coco_pretrained_yolov8_weight,
    parse_class_map,
)


@dataclass(frozen=True)
class YoloJob:
    generation: int
    image_msg: Image


@dataclass(frozen=True)
class YoloComputation:
    detections: object
    debug_msg: object = None
    debug_error: str = ""


class YoloFrameError(RuntimeError):
    """A categorized per-frame failure returned by the inference worker."""

    def __init__(self, category: str, message: str) -> None:
        super().__init__(message)
        self.category = category


class YoloV8BBoxNode(Node):
    """Publish YOLOv8 detections using the EUFS BoundingBoxes contract."""

    def __init__(self) -> None:
        super().__init__("yolov8_bbox_node")

        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        self.model_path = self._validated_model_path(self.model_path)
        self._warn_if_coco_smoke_test_model()
        self.model = self._load_model()
        self._validate_model_classes()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            # Keep only the freshest frame while inference is busy.  The
            # published bbox retains the acquisition timestamp, so fusion can
            # synchronize it with buffered LiDAR/stereo data without building
            # a stale camera backlog.
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            sensor_qos,
        )
        self._last_cloud_stamp = None
        self._lidar_period_est = self.lidar_period_sec
        self._prev_image_phase = None
        self._picked_this_period = False
        if self.inference_mode == "lidar_locked":
            from sensor_msgs.msg import PointCloud2
            self.create_subscription(
                PointCloud2,
                self.lidar_topic,
                self._cloud_stamp_callback,
                sensor_qos,
            )
        self.bbox_pub = self.create_publisher(
            BoundingBoxes,
            self.bbox_topic,
            10,
        )
        # A pose weight also carries the cone silhouette keypoints.  They ride a
        # separate topic with the same header stamp, so fusion pairs them with
        # the bounding boxes through its existing timestamp buffers, and a
        # detect-only weight simply never publishes here.
        self.keypoints_pub = None
        if self.publish_keypoints:
            self.keypoints_pub = self.create_publisher(
                ConeKeypointsArray,
                self.keypoints_topic,
                10,
            )
        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image,
                self.debug_image_topic,
                10,
            )

        self._clock_generation = 0
        self._shutting_down = False
        self._deferred_completion = None
        self._worker = LatestOnlyWorker(
            self,
            self._compute_yolo_job,
            self._commit_yolo_completion,
            "yolov8-inference",
        )
        self._completion_timer = self.create_timer(
            0.02,
            self._retry_deferred_completion,
        )
        self.clock_jump_handler = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=Duration(nanoseconds=(1 << 63) - 1),
                min_backward=Duration(
                    nanoseconds=-self.timestamp_reset_threshold_ns
                ),
                on_clock_change=True,
            ),
            pre_callback=self._before_clock_jump,
        )

        self.get_logger().info(
            "YOLOv8 bbox detector ready: "
            f"image={self.image_topic}, bboxes={self.bbox_topic}, "
            f"model={self.model_path}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/zed/left/image_rect_color")
        self.declare_parameter("bbox_topic", "/perception/bounding_boxes")
        # 'all' infers every frame the worker can take (30 Hz). 'lidar_locked'
        # infers ONE frame per LiDAR period — the frame landing just before
        # the NEXT predicted scan end, so inference (~30 ms) completes right
        # as the cloud arrives and the bbox<->cloud pairing gap drops to the
        # 30 Hz grid minimum (<=16 ms). Anchoring to the PREVIOUS cloud
        # instead would hand every cloud a 100+ ms-old bbox: the fusion node
        # grabs the newest COMPLETED bbox at cloud arrival.
        self.declare_parameter("inference_mode", "all")
        self.declare_parameter("lidar_topic", "/velodyne_points")
        self.declare_parameter("lidar_period_sec", 0.1)
        # Half the camera period: exactly one 30 Hz frame per LiDAR tick.
        self.declare_parameter("lock_tolerance_sec", 0.0167)
        self.declare_parameter(
            "model_path",
            "/home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt",
        )
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("max_det", 100)
        self.declare_parameter("timestamp_reset_threshold_sec", 0.1)
        self.declare_parameter("output_commit_settle_sec", 0.1)
        self.declare_parameter(
            "class_map",
            "blue_cone:blue,yellow_cone:yellow,orange_cone:orange,"
            "large_orange_cone:big_orange,unknown_cone:unknown",
        )
        self.declare_parameter("unknown_color_policy", "unknown")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter(
            "debug_image_topic",
            "/yolo_bounding_boxes/debug_image",
        )
        # Pose-weight keypoint output. Enabled by the pose launch path; a
        # detect-only weight leaves this off and the stereo tier stays disabled.
        self.declare_parameter("publish_keypoints", False)
        self.declare_parameter("keypoints_topic", "/yolo_cone_keypoints")

    def _load_parameters(self) -> None:
        self.image_topic = self.get_parameter("image_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.inference_mode = str(self.get_parameter("inference_mode").value)
        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.lidar_period_sec = float(
            self.get_parameter("lidar_period_sec").value)
        self.lock_tolerance_sec = float(
            self.get_parameter("lock_tolerance_sec").value)
        self.model_path = self.get_parameter("model_path").value
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value).strip()
        self.max_det = int(self.get_parameter("max_det").value)
        self.timestamp_reset_threshold_sec = float(
            self.get_parameter("timestamp_reset_threshold_sec").value
        )
        self.output_commit_settle_sec = float(
            self.get_parameter("output_commit_settle_sec").value
        )
        self.class_map: Dict[str, str] = parse_class_map(
            self.get_parameter("class_map").value
        )
        self.unknown_color_policy = str(
            self.get_parameter("unknown_color_policy").value
        ).strip()
        self.publish_debug_image = self._as_bool(
            self.get_parameter("publish_debug_image").value
        )
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.publish_keypoints = self._as_bool(
            self.get_parameter("publish_keypoints").value
        )
        self.keypoints_topic = self.get_parameter("keypoints_topic").value

    def _validate_parameters(self) -> None:
        for name, value in (
            ("confidence_threshold", self.confidence_threshold),
            ("iou_threshold", self.iou_threshold),
        ):
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be finite and in [0, 1]")
        if self.imgsz <= 0:
            raise ValueError("imgsz must be greater than zero")
        if self.max_det <= 0:
            raise ValueError("max_det must be greater than zero")
        if (
            not math.isfinite(self.timestamp_reset_threshold_sec)
            or self.timestamp_reset_threshold_sec <= 0.0
        ):
            raise ValueError(
                "timestamp_reset_threshold_sec must be finite and positive"
            )
        try:
            self.timestamp_reset_threshold_ns = int(
                round(self.timestamp_reset_threshold_sec * 1.0e9)
            )
        except (OverflowError, ValueError) as exc:
            raise ValueError(
                "timestamp_reset_threshold_sec must fit in an int64 "
                "nanosecond duration"
            ) from exc
        if not 0 < self.timestamp_reset_threshold_ns <= (1 << 63) - 1:
            raise ValueError(
                "timestamp_reset_threshold_sec must fit in an int64 "
                "nanosecond duration"
            )
        if (
            not math.isfinite(self.output_commit_settle_sec)
            or self.output_commit_settle_sec < 0.0
        ):
            raise ValueError(
                "output_commit_settle_sec must be finite and nonnegative"
            )
        try:
            self.output_commit_settle_ns = int(
                round(self.output_commit_settle_sec * 1.0e9)
            )
        except (OverflowError, ValueError) as exc:
            raise ValueError(
                "output_commit_settle_sec must fit in an int64 nanosecond "
                "duration"
            ) from exc
        if not 0 <= self.output_commit_settle_ns <= (1 << 63) - 1:
            raise ValueError(
                "output_commit_settle_sec must fit in an int64 nanosecond "
                "duration"
            )
        if self.unknown_color_policy.lower() not in ("skip", "unknown"):
            raise ValueError("unknown_color_policy must be 'skip' or 'unknown'")
        allowed_colors = {
            "blue",
            "yellow",
            "orange",
            "big_orange",
            "unknown",
        }
        invalid_colors = set(self.class_map.values()) - allowed_colors
        if invalid_colors:
            raise ValueError(
                "class_map contains unsupported output colors: "
                f"{sorted(invalid_colors)}"
            )

    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is required for yolov8_bbox_node. Install the "
                "package dependency, for example "
                "`pip install 'ultralytics>=8,<9'`, "
                "or run inside the project Docker image that provides it."
            ) from exc
        return YOLO(self.model_path)

    @staticmethod
    def _validated_model_path(model_path: str) -> str:
        """
        Return a canonical, readable local ``.pt`` model path.

        Ultralytics interprets well-known missing weight names as download
        requests.  Resolving and opening the file before constructing
        ``YOLO`` keeps this runtime strictly offline and makes a bad model
        configuration fail at node startup instead of silently fetching a
        different artifact.
        """
        configured_path = str(model_path).strip()
        if not configured_path:
            raise RuntimeError("YOLO model_path must not be empty")

        candidate = Path(configured_path).expanduser()
        try:
            resolved = candidate.resolve(strict=True)
        except (OSError, RuntimeError) as exc:
            raise RuntimeError(
                "YOLO model_path must reference an existing local .pt file: "
                f"{configured_path}"
            ) from exc

        if resolved.suffix.lower() != ".pt":
            raise RuntimeError(
                "YOLO model_path must reference a local .pt weight file: "
                f"{resolved}"
            )
        if not resolved.is_file():
            raise RuntimeError(
                "YOLO model_path is not a regular file: "
                f"{resolved}"
            )

        try:
            with resolved.open("rb") as model_file:
                model_file.read(1)
        except OSError as exc:
            raise RuntimeError(
                f"YOLO model_path is not readable: {resolved}"
            ) from exc
        return str(resolved)

    def _validate_model_classes(self) -> None:
        names = getattr(self.model, "names", None)
        if isinstance(names, dict):
            model_names = {str(value).strip().lower() for value in names.values()}
        elif isinstance(names, (list, tuple)):
            model_names = {str(value).strip().lower() for value in names}
        else:
            raise RuntimeError("YOLO model does not expose a class-name table")

        configured_names = {str(name).strip().lower() for name in self.class_map}
        missing_names = configured_names - model_names
        if missing_names:
            raise RuntimeError(
                "YOLO model is missing configured cone classes; "
                f"missing={sorted(missing_names)}, model classes={sorted(model_names)}"
            )

    def _warn_if_coco_smoke_test_model(self) -> None:
        if not looks_like_coco_pretrained_yolov8_weight(self.model_path):
            return
        if self.class_map:
            return
        self.get_logger().warn(
            "Using COCO-pretrained YOLOv8 weights without class_map. "
            "This is useful for ROS wiring smoke tests, but it does not "
            "provide "
            "Formula Student cone color classes. Use cone-trained weights and "
            "class_map for perception accuracy."
        )

    def _cloud_stamp_callback(self, msg) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # The real spin period wobbles and the config value is nominal:
        # estimate the actual period from consecutive stamps (EMA, clamped
        # to +/-10% of nominal so one bad stamp cannot derail the lock).
        if self._last_cloud_stamp is not None:
            delta = stamp - self._last_cloud_stamp
            nominal = self.lidar_period_sec
            if 0.5 * nominal < delta < 1.5 * nominal:
                blended = 0.9 * self._lidar_period_est + 0.1 * delta
                lo, hi = 0.9 * nominal, 1.1 * nominal
                self._lidar_period_est = min(max(blended, lo), hi)
        self._last_cloud_stamp = stamp

    def _image_callback(self, msg: Image) -> None:
        if self.inference_mode == "lidar_locked" and \
                self._last_cloud_stamp is not None:
            # One inference per LiDAR period, jitter-proof:
            # - preferred: the frame landing within lock_tolerance BEFORE the
            #   next predicted scan end (inference completes as it arrives);
            # - fallback: if jitter made every frame miss that window, the
            #   phase WRAP frame is submitted instead, so no period is ever
            #   skipped (gap <= one camera period).
            # The anchor refreshes every scan, so spin-period drift does not
            # accumulate; the period itself is estimated from the stream.
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            period = self._lidar_period_est
            phase = (stamp - self._last_cloud_stamp) % period
            wrapped = (self._prev_image_phase is not None
                       and phase < self._prev_image_phase)
            in_window = (period - phase) <= self.lock_tolerance_sec
            take = in_window or (wrapped and not self._picked_this_period)
            if wrapped:
                self._picked_this_period = False
            if not take:
                self._prev_image_phase = phase
                return
            self._picked_this_period = True
            self._prev_image_phase = phase
        self._worker.submit(YoloJob(self._clock_generation, msg))

    def _compute_yolo_job(self, job: YoloJob) -> YoloComputation:
        msg = job.image_msg
        try:
            image = image_message_to_numpy(msg, desired_encoding="bgr8")
        except (TypeError, ValueError) as exc:
            raise YoloFrameError(
                "image_conversion",
                "YOLO image conversion failed; dropped invalid frame "
                f"without publishing detections ({exc})",
            )

        predict_kwargs = {
            "source": image,
            "conf": self.confidence_threshold,
            "iou": self.iou_threshold,
            "imgsz": self.imgsz,
            "max_det": self.max_det,
            "verbose": False,
        }
        if self.device:
            predict_kwargs["device"] = self.device

        try:
            results = self.model.predict(**predict_kwargs)
        except Exception as exc:  # noqa: BLE001
            raise YoloFrameError(
                "inference",
                "YOLO inference failed; dropped frame without publishing "
                f"detections ({exc})",
            )

        try:
            detections = detections_from_ultralytics_results(
                results,
                names=getattr(self.model, "names", None),
                class_map=self.class_map,
                confidence_threshold=self.confidence_threshold,
                unknown_color_policy=self.unknown_color_policy,
            )
        except Exception as exc:  # noqa: BLE001
            raise YoloFrameError(
                "result_conversion",
                "YOLO result conversion failed; dropped frame without "
                f"publishing detections ({exc})",
            )
        debug_msg = None
        debug_error = ""
        if self.debug_image_pub is not None and results:
            try:
                annotated = results[0].plot()
                debug_msg = numpy_to_image_message(annotated, msg.header)
            except Exception as exc:  # noqa: BLE001
                debug_error = f"YOLO debug image skipped: render failed ({exc})"
        return YoloComputation(detections, debug_msg, debug_error)

    def _commit_yolo_completion(self, completion: WorkerCompletion) -> None:
        job = completion.job
        if self._shutting_down or job.generation != self._clock_generation:
            return
        if completion.error is not None:
            self.get_logger().warn(str(completion.error))
            return
        if self._completion_needs_settle(job.image_msg.header.stamp):
            self._deferred_completion = completion
            return

        computation = completion.result
        # Publish both views of the same detection list back to back so a
        # consumer can never see boxes without their matching keypoints.
        self.bbox_pub.publish(
            self._to_bounding_boxes_msg(job.image_msg, computation.detections)
        )
        if self.keypoints_pub is not None:
            self.keypoints_pub.publish(
                self._to_cone_keypoints_msg(
                    job.image_msg,
                    computation.detections,
                )
            )
        if computation.debug_error:
            self.get_logger().warn(computation.debug_error)
        if self.debug_image_pub is not None and computation.debug_msg is not None:
            self.debug_image_pub.publish(computation.debug_msg)

    def _retry_deferred_completion(self) -> None:
        completion = self._deferred_completion
        if completion is None:
            return
        job = completion.job
        if self._shutting_down or job.generation != self._clock_generation:
            self._deferred_completion = None
            return
        if self._completion_needs_settle(job.image_msg.header.stamp):
            return
        self._deferred_completion = None
        self._commit_yolo_completion(completion)

    def _completion_needs_settle(self, stamp) -> bool:
        settle_ns = int(getattr(self, "output_commit_settle_ns", 0))
        if settle_ns <= 0:
            return False
        clock = self.get_clock()
        if not bool(getattr(clock, "ros_time_is_active", False)):
            return False
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return stamp_ns > int(clock.now().nanoseconds) - settle_ns

    def _before_clock_jump(self) -> None:
        self._clock_generation += 1
        self._deferred_completion = None
        self._worker.clear_pending()

    def _to_bounding_boxes_msg(
        self,
        image_msg: Image,
        detections,
    ) -> BoundingBoxes:
        msg = BoundingBoxes()
        msg.header = image_msg.header
        msg.image_header = image_msg.header

        for detection in detections:
            bbox = BoundingBox()
            bbox.color = detection.color
            bbox.probability = detection.probability
            bbox.type = BoundingBox.PIXEL
            bbox.xmin = detection.xmin
            bbox.ymin = detection.ymin
            bbox.xmax = detection.xmax
            bbox.ymax = detection.ymax
            msg.bounding_boxes.append(bbox)
        return msg

    def _to_cone_keypoints_msg(
        self,
        image_msg: Image,
        detections,
    ) -> ConeKeypointsArray:
        msg = ConeKeypointsArray()
        msg.header = image_msg.header
        msg.image_header = image_msg.header

        for detection in detections:
            entry = ConeKeypoints()
            points = getattr(detection, "keypoints", None)
            if points is not None:
                entry.u = [float(value) for value in points[:, 0]]
                entry.v = [float(value) for value in points[:, 1]]
                scores = getattr(detection, "keypoint_confidence", None)
                if scores is not None:
                    entry.confidence = [float(value) for value in scores]
                else:
                    entry.confidence = [1.0] * len(entry.u)
            # An entry is appended for every detection, keypoints or not, so
            # index i always refers to bounding box i.
            msg.cone_keypoints.append(entry)
        return msg

    def destroy_node(self):
        self._shutting_down = True
        self._clock_generation += 1
        self._deferred_completion = None
        self._worker.clear_pending()
        self._worker.shutdown()
        return super().destroy_node()

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloV8BBoxNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
