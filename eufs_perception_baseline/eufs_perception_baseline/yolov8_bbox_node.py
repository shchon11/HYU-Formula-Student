from pathlib import Path
from typing import Dict

import rclpy
from eufs_msgs.msg import BoundingBox, BoundingBoxes
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from eufs_perception_baseline.ros_image_utils import (
    image_message_to_numpy,
    numpy_to_image_message,
)
from eufs_perception_baseline.yolov8_bbox_utils import (
    detections_from_ultralytics_results,
    looks_like_coco_pretrained_yolov8_weight,
    parse_class_map,
)


class YoloV8BBoxNode(Node):
    """Publish YOLOv8 detections using the EUFS BoundingBoxes contract."""

    def __init__(self) -> None:
        super().__init__("yolov8_bbox_node")

        self._declare_parameters()
        self._load_parameters()

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
        self.bbox_pub = self.create_publisher(
            BoundingBoxes,
            self.bbox_topic,
            10,
        )
        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image,
                self.debug_image_topic,
                10,
            )

        self.get_logger().info(
            "YOLOv8 bbox detector ready: "
            f"image={self.image_topic}, bboxes={self.bbox_topic}, "
            f"model={self.model_path}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/zed/left/image_rect_color")
        self.declare_parameter("bbox_topic", "/yolo_bounding_boxes")
        self.declare_parameter(
            "model_path",
            "/home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt",
        )
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("max_det", 100)
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

    def _load_parameters(self) -> None:
        self.image_topic = self.get_parameter("image_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value).strip()
        self.max_det = int(self.get_parameter("max_det").value)
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
        if configured_names and not (configured_names & model_names):
            raise RuntimeError(
                "YOLO class_map does not match any model class; "
                f"model classes={sorted(model_names)}"
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

    def _image_callback(self, msg: Image) -> None:
        try:
            image = image_message_to_numpy(msg, desired_encoding="bgr8")
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                "YOLO image conversion failed; dropped invalid frame "
                f"without publishing detections ({exc})"
            )
            return

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
            self.get_logger().warn(
                "YOLO inference failed; dropped frame without publishing "
                f"detections ({exc})"
            )
            return

        try:
            detections = detections_from_ultralytics_results(
                results,
                names=getattr(self.model, "names", None),
                class_map=self.class_map,
                confidence_threshold=self.confidence_threshold,
                unknown_color_policy=self.unknown_color_policy,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                "YOLO result conversion failed; dropped frame without "
                f"publishing detections ({exc})"
            )
            return
        self.bbox_pub.publish(self._to_bounding_boxes_msg(msg, detections))

        if self.debug_image_pub is not None and results:
            self._publish_debug_image(msg, results[0])

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

    def _publish_debug_image(self, image_msg: Image, result) -> None:
        try:
            annotated = result.plot()
            debug_msg = numpy_to_image_message(annotated, image_msg.header)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"YOLO debug image skipped: render failed ({exc})"
            )
            return
        self.debug_image_pub.publish(debug_msg)

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
