"""ROS Image conversion helpers that do not require ``cv_bridge``."""

from operator import index

import numpy as np


_CHANNELS = {
    "bgr8": 3,
    "rgb8": 3,
    "mono8": 1,
}


def image_message_to_numpy(message, desired_encoding: str = "bgr8") -> np.ndarray:
    """
    Convert a uint8 ROS Image-like message to an independent NumPy array.

    ``bgr8``, ``rgb8`` and ``mono8`` are supported.  The message may contain
    per-row padding; padding is removed from the returned C-contiguous copy.
    """
    source_encoding = _encoding(getattr(message, "encoding", None), "encoding")
    target_encoding = _encoding(desired_encoding, "desired_encoding")
    height = _positive_integer(getattr(message, "height", None), "height")
    width = _positive_integer(getattr(message, "width", None), "width")
    step = _positive_integer(getattr(message, "step", None), "step")

    row_bytes = width * _CHANNELS[source_encoding]
    if step < row_bytes:
        raise ValueError(
            "step {} is smaller than the {} bytes required by a row".format(
                step,
                row_bytes,
            )
        )

    data = getattr(message, "data", None)
    try:
        byte_view = memoryview(data).cast("B")
    except (TypeError, ValueError) as exc:
        raise ValueError("data must expose a contiguous byte buffer") from exc

    expected_size = height * step
    if byte_view.nbytes != expected_size:
        raise ValueError(
            "data has {} bytes, expected height * step = {}".format(
                byte_view.nbytes,
                expected_size,
            )
        )

    rows = np.frombuffer(byte_view, dtype=np.uint8).reshape(height, step)
    pixels = rows[:, :row_bytes]
    if _CHANNELS[source_encoding] == 1:
        source = pixels.reshape(height, width)
    else:
        source = pixels.reshape(height, width, 3)

    converted = _convert_encoding(source, source_encoding, target_encoding)
    # Always detach from the ROS message buffer and remove non-unit/negative
    # strides produced by padding removal or channel reversal.
    return np.array(converted, dtype=np.uint8, order="C", copy=True)


def numpy_to_image_message(array: np.ndarray, header=None):
    """Create a ``bgr8`` ROS Image from a contiguous uint8 HxWx3 array."""
    from sensor_msgs.msg import Image

    image = np.asarray(array)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("array must be a uint8 HxWx3 BGR image")
    image = np.ascontiguousarray(image)

    message = Image()
    if header is not None:
        message.header = header
    message.height = int(image.shape[0])
    message.width = int(image.shape[1])
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = int(image.shape[1] * 3)
    message.data = image.tobytes()
    return message


def _convert_encoding(
    source: np.ndarray,
    source_encoding: str,
    target_encoding: str,
) -> np.ndarray:
    if source_encoding == target_encoding:
        return source

    if source_encoding in ("bgr8", "rgb8") and target_encoding in ("bgr8", "rgb8"):
        return source[..., ::-1]

    if source_encoding == "mono8":
        return np.repeat(source[..., np.newaxis], 3, axis=2)

    if target_encoding == "mono8":
        if source_encoding == "rgb8":
            red = source[..., 0].astype(np.uint16)
            green = source[..., 1].astype(np.uint16)
            blue = source[..., 2].astype(np.uint16)
        else:
            blue = source[..., 0].astype(np.uint16)
            green = source[..., 1].astype(np.uint16)
            red = source[..., 2].astype(np.uint16)
        # Integer BT.601 approximation with rounding, matching the usual
        # luminance conversion without depending on OpenCV.
        return ((77 * red + 150 * green + 29 * blue + 128) >> 8).astype(np.uint8)

    # Encoding validation above means all supported combinations are handled.
    raise ValueError(
        "cannot convert {} to {}".format(source_encoding, target_encoding)
    )


def _encoding(value, name: str) -> str:
    if not isinstance(value, str):
        raise ValueError("{} must be a string".format(name))
    normalized = value.strip().lower()
    if normalized not in _CHANNELS:
        raise ValueError(
            "unsupported {} {!r}; expected bgr8, rgb8, or mono8".format(
                name,
                value,
            )
        )
    return normalized


def _positive_integer(value, name: str) -> int:
    if isinstance(value, bool):
        raise ValueError("{} must be a positive integer".format(name))
    try:
        result = index(value)
    except TypeError as exc:
        raise ValueError("{} must be a positive integer".format(name)) from exc
    if result <= 0:
        raise ValueError("{} must be greater than zero".format(name))
    return result
