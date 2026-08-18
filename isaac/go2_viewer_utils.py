#!/usr/bin/env python3
"""Pure NumPy utilities used by the Isaac Sim live navigation viewer."""

import math
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

# sensor_msgs/msg/PointField numeric constants. Keeping these local makes this
# module importable for tests without a ROS installation.
POINTFIELD_DTYPES = {
    1: np.dtype("i1"),
    2: np.dtype("u1"),
    3: np.dtype("i2"),
    4: np.dtype("u2"),
    5: np.dtype("i4"),
    6: np.dtype("u4"),
    7: np.dtype("f4"),
    8: np.dtype("f8"),
}


def pointcloud2_xyz(msg, max_points: int) -> np.ndarray:
    """Extract finite XYZ samples, downsampling before materializing columns."""

    xyz_fields = {field.name: field for field in msg.fields if field.name in {"x", "y", "z"}}
    if set(xyz_fields) != {"x", "y", "z"}:
        return np.empty((0, 3), dtype=np.float32)
    if any(field.count != 1 for field in xyz_fields.values()):
        return np.empty((0, 3), dtype=np.float32)

    width = int(msg.width)
    height = int(msg.height)
    point_step = int(msg.point_step)
    row_step = int(msg.row_step)
    if width <= 0 or height <= 0 or point_step <= 0 or row_step < width * point_step:
        return np.empty((0, 3), dtype=np.float32)
    if len(msg.data) < row_step * height:
        return np.empty((0, 3), dtype=np.float32)

    names: List[str] = []
    formats: List[object] = []
    offsets: List[int] = []
    byte_order = ">" if msg.is_bigendian else "<"
    for field in msg.fields:
        base_dtype = POINTFIELD_DTYPES.get(field.datatype)
        if base_dtype is None or field.count <= 0:
            continue
        if base_dtype.itemsize > 1:
            base_dtype = base_dtype.newbyteorder(byte_order)
        field_size = base_dtype.itemsize * field.count
        if field.offset < 0 or field.offset + field_size > point_step:
            return np.empty((0, 3), dtype=np.float32)
        names.append(field.name)
        formats.append((base_dtype, field.count) if field.count > 1 else base_dtype)
        offsets.append(field.offset)

    if not {"x", "y", "z"}.issubset(names):
        return np.empty((0, 3), dtype=np.float32)

    try:
        dtype = np.dtype(
            {
                "names": names,
                "formats": formats,
                "offsets": offsets,
                "itemsize": point_step,
            }
        )
    except (TypeError, ValueError):
        return np.empty((0, 3), dtype=np.float32)

    total_points = width * height
    stride = 1
    if max_points > 0 and total_points > max_points:
        stride = int(math.ceil(total_points / max_points))
    sample_indices = np.arange(0, total_points, stride, dtype=np.int64)

    if row_step == width * point_step:
        cloud = np.frombuffer(msg.data, dtype=dtype, count=total_points)
        samples = cloud[sample_indices]
    else:
        cloud = np.ndarray(
            shape=(height, width),
            dtype=dtype,
            buffer=msg.data,
            strides=(row_step, point_step),
        )
        rows = sample_indices // width
        columns = sample_indices % width
        samples = cloud[rows, columns]

    xyz = np.empty((sample_indices.size, 3), dtype=np.float32)
    xyz[:, 0] = samples["x"]
    xyz[:, 1] = samples["y"]
    xyz[:, 2] = samples["z"]
    return xyz[np.isfinite(xyz).all(axis=1)]


class TransformGraph:
    """Bidirectional transform graph with cached topology and inverse matrices."""

    def __init__(self):
        self._adjacency: Dict[str, Dict[str, np.ndarray]] = {}
        self._path_cache: Dict[Tuple[str, str], Optional[Tuple[str, ...]]] = {}

    def update(self, parent: str, child: str, parent_from_child: np.ndarray) -> None:
        parent_name = parent.strip("/")
        child_name = child.strip("/")
        if not parent_name or not child_name or parent_name == child_name:
            return

        matrix = np.asarray(parent_from_child, dtype=np.float64)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            return
        try:
            child_from_parent = np.linalg.inv(matrix)
        except np.linalg.LinAlgError:
            return

        topology_changed = (
            child_name not in self._adjacency.get(parent_name, {})
            or parent_name not in self._adjacency.get(child_name, {})
        )
        self._adjacency.setdefault(child_name, {})[parent_name] = matrix
        self._adjacency.setdefault(parent_name, {})[child_name] = child_from_parent
        if topology_changed:
            self._path_cache.clear()

    def _find_path(self, source: str, target: str) -> Optional[Tuple[str, ...]]:
        key = (source, target)
        if key in self._path_cache:
            return self._path_cache[key]

        queue: Deque[Tuple[str, Tuple[str, ...]]] = deque([(source, (source,))])
        visited = {source}
        while queue:
            frame, path = queue.popleft()
            if frame == target:
                self._path_cache[key] = path
                return path
            for neighbor in self._adjacency.get(frame, {}):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + (neighbor,)))

        self._path_cache[key] = None
        return None

    def lookup(self, source_frame: str, target_frame: str) -> Optional[np.ndarray]:
        source = source_frame.strip("/")
        target = target_frame.strip("/")
        if not source or not target:
            return None
        if source == target:
            return np.eye(4, dtype=np.float64)

        path = self._find_path(source, target)
        if path is None:
            return None

        target_from_source = np.eye(4, dtype=np.float64)
        for current, neighbor in zip(path, path[1:]):
            target_from_source = self._adjacency[current][neighbor] @ target_from_source
        return target_from_source
