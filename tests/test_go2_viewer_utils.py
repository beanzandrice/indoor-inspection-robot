import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

MODULE_DIR = Path(__file__).resolve().parents[1] / "isaac"
sys.path.insert(0, str(MODULE_DIR))

from go2_viewer_utils import TransformGraph, pointcloud2_xyz  # noqa: E402


def field(name, offset, datatype=7, count=1):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


def cloud_message(points, big_endian=False, row_padding=0, width=None):
    width = len(points) if width is None else width
    height = len(points) // width
    point_step = 16
    row_step = width * point_step + row_padding
    data = bytearray(row_step * height)
    byte_order = ">" if big_endian else "<"
    for index, (x, y, z) in enumerate(points):
        row = index // width
        column = index % width
        struct.pack_into(
            byte_order + "fff",
            data,
            row * row_step + column * point_step,
            x,
            y,
            z,
        )
    return SimpleNamespace(
        fields=[field("x", 0), field("y", 4), field("z", 8)],
        is_bigendian=big_endian,
        width=width,
        height=height,
        point_step=point_step,
        row_step=row_step,
        data=bytes(data),
    )


def translation(x, y, z=0.0):
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, 3] = [x, y, z]
    return matrix


def test_pointcloud_downsamples_before_xyz_materialization_and_handles_padding():
    points = [(float(index), float(index + 10), 1.0) for index in range(8)]
    msg = cloud_message(points, row_padding=8, width=4)

    xyz = pointcloud2_xyz(msg, max_points=3)

    np.testing.assert_allclose(
        xyz,
        np.asarray([points[0], points[3], points[6]], dtype=np.float32),
    )


def test_pointcloud_handles_big_endian_and_removes_nonfinite_samples():
    msg = cloud_message([(1.0, 2.0, 3.0), (float("nan"), 5.0, 6.0)], big_endian=True)

    xyz = pointcloud2_xyz(msg, max_points=0)

    np.testing.assert_allclose(xyz, np.asarray([[1.0, 2.0, 3.0]], dtype=np.float32))


def test_pointcloud_rejects_truncated_or_invalid_layout():
    msg = cloud_message([(1.0, 2.0, 3.0)])
    msg.data = msg.data[:-1]
    assert pointcloud2_xyz(msg, max_points=10).shape == (0, 3)

    msg = cloud_message([(1.0, 2.0, 3.0)])
    msg.fields[0].offset = msg.point_step
    assert pointcloud2_xyz(msg, max_points=10).shape == (0, 3)


def test_transform_graph_composes_and_inverts_paths():
    graph = TransformGraph()
    graph.update("map", "odom", translation(1.0, 0.0))
    graph.update("odom", "base_link", translation(0.0, 2.0))

    np.testing.assert_allclose(graph.lookup("base_link", "map"), translation(1.0, 2.0))
    np.testing.assert_allclose(graph.lookup("map", "base_link"), translation(-1.0, -2.0))


def test_transform_graph_cached_path_uses_updated_dynamic_matrix():
    graph = TransformGraph()
    graph.update("map", "odom", translation(1.0, 0.0))
    graph.update("odom", "base_link", translation(0.0, 2.0))
    graph.lookup("base_link", "map")

    graph.update("map", "odom", translation(5.0, 0.0))

    np.testing.assert_allclose(graph.lookup("base_link", "map"), translation(5.0, 2.0))


def test_transform_graph_invalidates_missing_path_when_topology_changes():
    graph = TransformGraph()
    graph.update("map", "odom", translation(1.0, 0.0))
    assert graph.lookup("base_link", "map") is None

    graph.update("odom", "base_link", translation(0.0, 2.0))

    np.testing.assert_allclose(graph.lookup("base_link", "map"), translation(1.0, 2.0))
