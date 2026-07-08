import math
import struct
from typing import Iterable, Optional


_DATATYPES = {
    1: ("b", 1),
    2: ("B", 1),
    3: ("h", 2),
    4: ("H", 2),
    5: ("i", 4),
    6: ("I", 4),
    7: ("f", 4),
    8: ("d", 8),
}


def read_points(
    cloud,
    field_names: Optional[Iterable[str]] = None,
    skip_nans: bool = False,
    uvs: Optional[Iterable] = None,
):
    """Small PointCloud2 reader fallback for missing sensor_msgs_py."""
    requested = set(field_names) if field_names is not None else None
    fields = [
        field
        for field in sorted(cloud.fields, key=lambda item: item.offset)
        if requested is None or field.name in requested
    ]
    if requested is not None and len(fields) != len(requested):
        present = {field.name for field in fields}
        missing = ", ".join(sorted(requested - present))
        raise ValueError(f"PointCloud2 is missing requested fields: {missing}")

    endian = ">" if cloud.is_bigendian else "<"
    data = memoryview(cloud.data)
    point_step = int(cloud.point_step)
    width = int(cloud.width)
    height = int(cloud.height)

    if uvs is None:
        indices = range(width * height)
    else:
        indices = (int(v) * width + int(u) for u, v in uvs)

    for index in indices:
        base = index * point_step
        point = []
        has_nan = False
        for field in fields:
            fmt, size = _field_format(field)
            count = max(1, int(field.count))
            for offset_index in range(count):
                value = struct.unpack_from(
                    endian + fmt,
                    data,
                    base + int(field.offset) + offset_index * size,
                )[0]
                point.append(value)
                if isinstance(value, float) and math.isnan(value):
                    has_nan = True
        if skip_nans and has_nan:
            continue
        yield tuple(point)


def _field_format(field):
    try:
        return _DATATYPES[int(field.datatype)]
    except KeyError as exc:
        message = f"Unsupported PointField datatype: {field.datatype}"
        raise ValueError(message) from exc
