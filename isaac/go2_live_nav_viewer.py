#!/usr/bin/env python3
"""Isaac Sim live viewer for the GO2 navigation bridge.

This is a visualization client for the laptop-side ROS graph created by
scripts/laptop_rviz_receiver.py. It does not run navigation or simulation
control; Nav2/SLAM still run on the GO2.
"""

import argparse
import math
import os
import time
from typing import Optional, Tuple

import numpy as np
from go2_viewer_utils import TransformGraph, pointcloud2_xyz

os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without opening a GUI.")
    parser.add_argument("--fixed-frame", default="map", help="Frame used as the Isaac world frame.")
    parser.add_argument("--robot-frame", default="base_link", help="Robot body frame to track.")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--tf-static-topic", default="/tf_static")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--pointcloud-topic", default="/trans_cloud")
    parser.add_argument("--initialpose-topic", default="/initialpose")
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--max-cloud-points", type=int, default=12000)
    parser.add_argument("--max-free-map-points", type=int, default=8000)
    parser.add_argument("--occupied-threshold", type=int, default=50)
    parser.add_argument("--map-point-size", type=float, default=0.035)
    parser.add_argument("--scan-point-size", type=float, default=0.045)
    parser.add_argument("--cloud-point-size", type=float, default=0.025)
    parser.add_argument("--status-period", type=float, default=5.0)
    parser.add_argument("--renderer", default="RaytracedLighting")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": ARGS.headless, "renderer": ARGS.renderer})

import omni  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, PointCloud2  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def matrix_to_rpy(matrix: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])
    if sy > 1e-6:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def transform_to_matrix(transform) -> np.ndarray:
    translation = transform.translation
    rotation = transform.rotation
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def pose_to_matrix(pose) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_to_matrix(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def transform_points(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points
    homogeneous = np.ones((points.shape[0], 4), dtype=np.float64)
    homogeneous[:, :3] = points[:, :3]
    return (matrix @ homogeneous.T).T[:, :3]


def laserscan_xyz(msg: LaserScan) -> np.ndarray:
    ranges = np.asarray(msg.ranges, dtype=np.float32)
    angles = msg.angle_min + np.arange(ranges.shape[0], dtype=np.float32) * msg.angle_increment
    valid = np.isfinite(ranges)
    if msg.range_min > 0.0:
        valid &= ranges >= msg.range_min
    if msg.range_max > 0.0:
        valid &= ranges <= msg.range_max
    ranges = ranges[valid]
    angles = angles[valid]
    return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles), np.zeros_like(ranges)))


def map_points(
    msg: OccupancyGrid,
    occupied_threshold: int,
    max_free_points: int,
) -> Tuple[np.ndarray, np.ndarray]:
    if not msg.data or msg.info.width == 0 or msg.info.height == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.float32)

    data = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
    occupied_rc = np.argwhere(data >= occupied_threshold)
    free_rc = np.argwhere(data == 0)

    if max_free_points > 0 and free_rc.shape[0] > max_free_points:
        stride = int(math.ceil(free_rc.shape[0] / max_free_points))
        free_rc = free_rc[::stride]

    def rc_to_points(rc: np.ndarray, z: float) -> np.ndarray:
        if rc.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        rows = rc[:, 0].astype(np.float64)
        cols = rc[:, 1].astype(np.float64)
        local_x = (cols + 0.5) * msg.info.resolution
        local_y = (rows + 0.5) * msg.info.resolution
        origin = msg.info.origin
        _, _, yaw = quaternion_to_rpy(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        points = np.column_stack(
            (
                origin.position.x + cos_yaw * local_x - sin_yaw * local_y,
                origin.position.y + sin_yaw * local_x + cos_yaw * local_y,
                np.full_like(local_x, z),
            )
        )
        return points.astype(np.float32)

    return rc_to_points(occupied_rc, 0.015), rc_to_points(free_rc, 0.0)


class IsaacGo2NavViewer(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("isaac_go2_nav_viewer")
        self.args = args
        self.transform_graph = TransformGraph()
        self.latest_odom = None  # type: Optional[Odometry]
        self.pending_map = None  # type: Optional[OccupancyGrid]
        self.pending_scan = None  # type: Optional[LaserScan]
        self.pending_cloud = None  # type: Optional[PointCloud2]
        self.latest_goal = None  # type: Optional[PoseStamped]
        self.latest_initialpose = None  # type: Optional[PoseWithCovarianceStamped]
        self.robot_dirty = True
        self.pose_markers_dirty = True
        self.last_status = time.monotonic()
        self.counts = {
            "map": 0,
            "tf": 0,
            "odom": 0,
            "scan": 0,
            "cloud": 0,
            "goal": 0,
            "initialpose": 0,
        }

        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        self.stage = omni.usd.get_context().get_stage()
        self.root_path = Sdf.Path("/World/Go2Live")
        self.stage.DefinePrim(self.root_path, "Xform")

        self.map_occ = self.make_points("/World/Go2Live/MapOccupied", (0.02, 0.02, 0.02), args.map_point_size)
        self.map_free = self.make_points("/World/Go2Live/MapFree", (0.72, 0.72, 0.72), args.map_point_size * 0.55)
        self.scan_points = self.make_points("/World/Go2Live/LaserScan", (0.0, 0.95, 0.15), args.scan_point_size)
        self.cloud_points = self.make_points("/World/Go2Live/PointCloud", (0.1, 0.55, 1.0), args.cloud_point_size)
        self.goal_points = self.make_points("/World/Go2Live/GoalMarkers", (1.0, 0.2, 0.05), 0.16)
        self.initial_points = self.make_points("/World/Go2Live/InitialPoseMarkers", (0.55, 0.2, 1.0), 0.14)
        self.robot_api = self.create_robot_marker()
        self.add_light()

        set_camera_view(eye=[2.8, -5.0, 4.0], target=[0.0, 0.0, 0.0])

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        tf_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.RELIABLE)
        default_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(OccupancyGrid, args.map_topic, self.map_cb, map_qos)
        self.create_subscription(TFMessage, args.tf_topic, self.tf_cb, tf_qos)
        self.create_subscription(TFMessage, args.tf_static_topic, self.tf_cb, map_qos)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, default_qos)
        self.create_subscription(LaserScan, args.scan_topic, self.scan_cb, default_qos)
        self.create_subscription(PointCloud2, args.pointcloud_topic, self.cloud_cb, default_qos)
        self.create_subscription(PoseStamped, args.goal_topic, self.goal_cb, default_qos)
        self.create_subscription(PoseWithCovarianceStamped, args.initialpose_topic, self.initialpose_cb, default_qos)

        self.get_logger().info(
            "Isaac Sim GO2 viewer ready: "
            f"fixed_frame={args.fixed_frame}, robot_frame={args.robot_frame}, "
            f"map={args.map_topic}, scan={args.scan_topic}, cloud={args.pointcloud_topic}"
        )

    def make_points(self, path: str, color: Tuple[float, float, float], width: float) -> UsdGeom.Points:
        points = UsdGeom.Points.Define(self.stage, Sdf.Path(path))
        points.CreatePointsAttr([])
        points.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        widths_attr = points.CreateWidthsAttr([width])
        try:
            UsdGeom.Primvar(widths_attr).SetInterpolation(UsdGeom.Tokens.constant)
        except Exception:
            pass
        return points

    def set_points(self, points_prim: UsdGeom.Points, points: np.ndarray) -> None:
        if points.size == 0:
            points_prim.GetPointsAttr().Set([])
            return
        gf_points = [Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in points[:, :3]]
        points_prim.GetPointsAttr().Set(gf_points)

    def add_light(self) -> None:
        light = UsdLux.DistantLight.Define(self.stage, Sdf.Path("/World/Go2Live/DistantLight"))
        light.CreateIntensityAttr(500)
        UsdGeom.XformCommonAPI(light.GetPrim()).SetRotate(
            (-45.0, 0.0, 35.0),
            UsdGeom.XformCommonAPI.RotationOrderXYZ,
        )

    def create_colored_cube(
        self,
        path: str,
        translate: Tuple[float, float, float],
        scale: Tuple[float, float, float],
        color: Tuple[float, float, float],
    ) -> None:
        cube = UsdGeom.Cube.Define(self.stage, Sdf.Path(path))
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        api = UsdGeom.XformCommonAPI(cube.GetPrim())
        api.SetTranslate(Gf.Vec3d(*translate))
        api.SetScale(Gf.Vec3f(*scale))

    def create_robot_marker(self) -> UsdGeom.XformCommonAPI:
        robot_prim = self.stage.DefinePrim(Sdf.Path("/World/Go2Live/Robot"), "Xform")
        self.create_colored_cube("/World/Go2Live/Robot/Body", (0.0, 0.0, 0.28), (0.50, 0.18, 0.10), (0.9, 0.9, 0.9))
        self.create_colored_cube("/World/Go2Live/Robot/Heading", (0.35, 0.0, 0.31), (0.28, 0.035, 0.035), (1.0, 0.05, 0.02))
        for name, x, y in (
            ("FL", 0.19, 0.13),
            ("FR", 0.19, -0.13),
            ("RL", -0.19, 0.13),
            ("RR", -0.19, -0.13),
        ):
            self.create_colored_cube(f"/World/Go2Live/Robot/{name}_Leg", (x, y, 0.13), (0.045, 0.035, 0.18), (0.05, 0.05, 0.08))
        return UsdGeom.XformCommonAPI(robot_prim)

    def map_cb(self, msg: OccupancyGrid) -> None:
        self.pending_map = msg
        self.counts["map"] += 1

    def tf_cb(self, msg: TFMessage) -> None:
        for stamped in msg.transforms:
            parent = stamped.header.frame_id.strip("/")
            child = stamped.child_frame_id.strip("/")
            if parent and child:
                self.transform_graph.update(parent, child, transform_to_matrix(stamped.transform))
                self.robot_dirty = True
                self.counts["tf"] += 1

    def odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if msg.header.frame_id and msg.child_frame_id:
            self.transform_graph.update(
                msg.header.frame_id,
                msg.child_frame_id,
                pose_to_matrix(msg.pose.pose),
            )
        self.robot_dirty = True
        self.counts["odom"] += 1

    def scan_cb(self, msg: LaserScan) -> None:
        self.pending_scan = msg
        self.counts["scan"] += 1

    def cloud_cb(self, msg: PointCloud2) -> None:
        self.pending_cloud = msg
        self.counts["cloud"] += 1

    def goal_cb(self, msg: PoseStamped) -> None:
        self.latest_goal = msg
        self.pose_markers_dirty = True
        self.counts["goal"] += 1

    def initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_initialpose = msg
        self.pose_markers_dirty = True
        self.counts["initialpose"] += 1

    def lookup_transform(self, source_frame: str, target_frame: str) -> Optional[np.ndarray]:
        return self.transform_graph.lookup(source_frame, target_frame)

    def update_scene(self) -> None:
        self.update_map()
        self.update_robot()
        self.update_scan()
        self.update_cloud()
        self.update_pose_markers()
        self.log_status()

    def update_map(self) -> None:
        if self.pending_map is None:
            return
        occupied, free = map_points(
            self.pending_map,
            occupied_threshold=self.args.occupied_threshold,
            max_free_points=self.args.max_free_map_points,
        )
        self.set_points(self.map_occ, occupied)
        self.set_points(self.map_free, free)
        self.pending_map = None

    def update_robot(self) -> None:
        if not self.robot_dirty:
            return
        fixed_from_robot = self.lookup_transform(self.args.robot_frame, self.args.fixed_frame)
        if fixed_from_robot is None and self.latest_odom is not None:
            odom_parent = self.latest_odom.header.frame_id.strip("/")
            odom_child = self.latest_odom.child_frame_id.strip("/")
            if (
                odom_parent == self.args.fixed_frame.strip("/")
                and odom_child == self.args.robot_frame.strip("/")
            ):
                fixed_from_robot = pose_to_matrix(self.latest_odom.pose.pose)
        if fixed_from_robot is None:
            self.robot_dirty = False
            return

        translation = fixed_from_robot[:3, 3]
        roll, pitch, yaw = matrix_to_rpy(fixed_from_robot[:3, :3])
        self.robot_api.SetTranslate(Gf.Vec3d(float(translation[0]), float(translation[1]), float(translation[2])))
        self.robot_api.SetRotate(
            (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)),
            UsdGeom.XformCommonAPI.RotationOrderXYZ,
        )
        self.robot_dirty = False

    def update_scan(self) -> None:
        if self.pending_scan is None:
            return
        msg = self.pending_scan
        points = laserscan_xyz(msg)
        transform = self.lookup_transform(msg.header.frame_id, self.args.fixed_frame)
        if transform is not None:
            points = transform_points(points, transform)
            self.set_points(self.scan_points, points)
        self.pending_scan = None

    def update_cloud(self) -> None:
        if self.pending_cloud is None:
            return
        msg = self.pending_cloud
        points = pointcloud2_xyz(msg, self.args.max_cloud_points)
        transform = self.lookup_transform(msg.header.frame_id, self.args.fixed_frame)
        if transform is not None:
            points = transform_points(points, transform)
            self.set_points(self.cloud_points, points)
        self.pending_cloud = None

    def update_pose_markers(self) -> None:
        if not self.pose_markers_dirty:
            return
        goal_points = []
        if self.latest_goal is not None:
            goal_points.append(
                [
                    self.latest_goal.pose.position.x,
                    self.latest_goal.pose.position.y,
                    max(self.latest_goal.pose.position.z, 0.2),
                ]
            )
        initial_points = []
        if self.latest_initialpose is not None:
            pose = self.latest_initialpose.pose.pose
            initial_points.append([pose.position.x, pose.position.y, max(pose.position.z, 0.18)])
        self.set_points(self.goal_points, np.asarray(goal_points, dtype=np.float32))
        self.set_points(self.initial_points, np.asarray(initial_points, dtype=np.float32))
        self.pose_markers_dirty = False

    def log_status(self) -> None:
        now = time.monotonic()
        if now - self.last_status < self.args.status_period:
            return
        elapsed = max(now - self.last_status, 0.001)
        active = [f"{name}={count / elapsed:.1f}Hz" for name, count in self.counts.items() if count]
        self.counts = {name: 0 for name in self.counts}
        self.last_status = now
        if active:
            self.get_logger().info("viewer inputs: " + ", ".join(active))
        else:
            self.get_logger().info("viewer waiting for relayed ROS data")


def main() -> None:
    rclpy.init()
    viewer = IsaacGo2NavViewer(ARGS)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    try:
        while simulation_app.is_running():
            viewer.world.step(render=True)
            rclpy.spin_once(viewer, timeout_sec=0.0)
            viewer.update_scene()
    except KeyboardInterrupt:
        pass
    finally:
        timeline.stop()
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
