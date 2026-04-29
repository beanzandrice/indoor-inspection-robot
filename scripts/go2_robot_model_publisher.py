#!/usr/bin/env python3
"""Publish a Go2 URDF and fixed TF tree for RViz RobotModel."""

import argparse
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def parse_xyz(value: str | None) -> tuple[float, float, float]:
    if not value:
        return 0.0, 0.0, 0.0
    parts = [float(part) for part in value.split()]
    return tuple((parts + [0.0, 0.0, 0.0])[:3])


def rewrite_mesh_paths(urdf_path: Path) -> str:
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        if filename.startswith(("package://", "file://", "http://", "https://")):
            continue
        path = Path(filename)
        if not path.is_absolute():
            path = (urdf_path.parent / path).resolve()
        mesh.set("filename", path.as_uri())

    return ET.tostring(root, encoding="unicode")


def root_link_name(robot: ET.Element) -> str | None:
    links = {link.get("name") for link in robot.findall("link") if link.get("name")}
    child_links = {
        child.get("link")
        for child in robot.findall("./joint/child")
        if child.get("link")
    }
    roots = sorted(links - child_links)
    return roots[0] if roots else None


class Go2RobotModelPublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("go2_robot_model_publisher")
        self.args = args
        self.urdf_path = Path(args.urdf).expanduser().resolve()
        self.robot = ET.parse(self.urdf_path).getroot()
        self.robot_description = rewrite_mesh_paths(self.urdf_path)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.description_pub = self.create_publisher(String, args.description_topic, qos)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.transforms = self.build_transforms()

        self.create_timer(args.republish_period, self.publish_all)
        self.publish_all()

        self.get_logger().info(
            f"Publishing Go2 robot model from {self.urdf_path} on {args.description_topic}; "
            f"{len(self.transforms)} static transforms rooted at {args.tf_root_frame}"
        )

    def build_transforms(self) -> list[TransformStamped]:
        stamp = self.get_clock().now().to_msg()
        transforms = []
        root = root_link_name(self.robot)

        if root and root != self.args.tf_root_frame:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.args.tf_root_frame
            transform.child_frame_id = root
            transform.transform.rotation.w = 1.0
            transforms.append(transform)

        skipped_moving_joints = 0
        for joint in self.robot.findall("joint"):
            if joint.get("type") != "fixed" and not self.args.include_moving_joints:
                skipped_moving_joints += 1
                continue

            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            parent_link = parent.get("link")
            child_link = child.get("link")
            if not parent_link or not child_link:
                continue

            origin = joint.find("origin")
            xyz = parse_xyz(origin.get("xyz") if origin is not None else None)
            rpy = parse_xyz(origin.get("rpy") if origin is not None else None)
            qx, qy, qz, qw = quaternion_from_rpy(*rpy)

            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = parent_link
            transform.child_frame_id = child_link
            transform.transform.translation.x = xyz[0]
            transform.transform.translation.y = xyz[1]
            transform.transform.translation.z = xyz[2]
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            transforms.append(transform)

        if skipped_moving_joints:
            self.get_logger().info(
                f"Skipped {skipped_moving_joints} moving joints; they should come from GO2 LowState TF"
            )

        return transforms

    def publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for transform in self.transforms:
            transform.header.stamp = stamp

        msg = String()
        msg.data = self.robot_description
        self.description_pub.publish(msg)
        self.static_broadcaster.sendTransform(self.transforms)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, help="Path to the Go2 URDF file.")
    parser.add_argument("--description-topic", default="/robot_description")
    parser.add_argument("--tf-root-frame", default="base_link")
    parser.add_argument("--republish-period", type=float, default=2.0)
    parser.add_argument(
        "--include-moving-joints",
        action="store_true",
        help="Also publish neutral transforms for revolute joints. Use only if no GO2 joint TF is available.",
    )
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = Go2RobotModelPublisher(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
