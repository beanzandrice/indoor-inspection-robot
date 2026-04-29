#!/usr/bin/env python3
"""Relay GO2 navigation topics into the laptop ROS graph for RViz2."""

import argparse
import base64
import json
import socket
import struct
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


TOPIC_TYPES = {
    "/tf": TFMessage,
    "/tf_static": TFMessage,
    "/map": OccupancyGrid,
    "/amcl_pose": PoseWithCovarianceStamped,
    "/odom": Odometry,
    "/scan": LaserScan,
    "/plan": Path,
    "/local_plan": Path,
    "/global_costmap/costmap": OccupancyGrid,
    "/local_costmap/costmap": OccupancyGrid,
}


def qos_for_topic(topic_name: str) -> QoSProfile:
    if topic_name in {"/map", "/tf_static", "/global_costmap/costmap", "/local_costmap/costmap"}:
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100 if topic_name == "/tf" else 20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class LaptopRvizReceiver(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("laptop_rviz_receiver")
        self.args = args
        self.topic_publishers = {
            topic_name: (self.create_publisher(msg_type, topic_name, qos_for_topic(topic_name)), msg_type)
            for topic_name, msg_type in TOPIC_TYPES.items()
        }

        command_qos = qos_for_topic("/goal_pose")
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initialpose_cb,
            command_qos,
        )
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_cb, command_qos)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((args.listen_host, args.recv_port))
        self.server.listen(1)
        self.server.setblocking(False)

        self.conn = None
        self.buffer = b""
        self.topic_counts = {topic_name: 0 for topic_name in TOPIC_TYPES}
        self.last_status = time.monotonic()
        self.last_send_warn = 0.0

        self.create_timer(0.01, self.poll_receive_socket)
        self.create_timer(5.0, self.log_status)

        stamp_mode = "preserving robot stamps" if args.preserve_robot_stamps else "restamping to laptop clock"
        self.get_logger().info(
            f"Listening for GO2 nav data on {args.listen_host}:{args.recv_port}; "
            f"return channel target {args.robot_host}:{args.send_port}; {stamp_mode}"
        )

    def send_packet(self, payload: dict) -> None:
        raw = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(self.args.connect_timeout)
            sock.connect((self.args.robot_host, self.args.send_port))
            sock.sendall(struct.pack(">I", len(raw)) + raw)

    def relay_command(self, topic_name: str, msg) -> None:
        try:
            payload = {
                "topic": topic_name,
                "stamp": time.time(),
                "data_b64": base64.b64encode(serialize_message(msg)).decode("ascii"),
            }
            self.send_packet(payload)
            self.get_logger().info(f"Relayed {topic_name} to GO2")
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_send_warn > 2.0:
                self.get_logger().error(f"Failed to relay {topic_name}: {exc}")
                self.last_send_warn = now

    def initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.relay_command("/initialpose", msg)

    def goal_pose_cb(self, msg: PoseStamped) -> None:
        self.relay_command("/goal_pose", msg)

    def close_receive_socket(self) -> None:
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass
        self.conn = None
        self.buffer = b""

    def poll_receive_socket(self) -> None:
        if self.conn is None:
            try:
                conn, addr = self.server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.setblocking(False)
                self.conn = conn
                self.buffer = b""
                self.get_logger().info(f"GO2 data channel connected from {addr}")
            except BlockingIOError:
                return
            except Exception as exc:
                self.get_logger().error(f"Accept failed: {exc}")
                return

        try:
            chunk = self.conn.recv(65536)
            if not chunk:
                self.get_logger().warn("GO2 data channel disconnected")
                self.close_receive_socket()
                return
            self.buffer += chunk
        except BlockingIOError:
            pass
        except Exception as exc:
            self.get_logger().error(f"Receive failed: {exc}")
            self.close_receive_socket()
            return

        while len(self.buffer) >= 4:
            msg_len = struct.unpack(">I", self.buffer[:4])[0]
            if len(self.buffer) < 4 + msg_len:
                return

            raw = self.buffer[4 : 4 + msg_len]
            self.buffer = self.buffer[4 + msg_len :]
            self.publish_relayed_payload(raw)

    def publish_relayed_payload(self, raw: bytes) -> None:
        try:
            payload = json.loads(raw.decode("utf-8"))
            topic_name = payload["topic"]
            if topic_name not in self.topic_publishers:
                self.get_logger().warn(f"Skipping unknown relayed topic: {topic_name}")
                return

            publisher, msg_type = self.topic_publishers[topic_name]
            msg = deserialize_message(base64.b64decode(payload["data_b64"]), msg_type)
            if not self.args.preserve_robot_stamps:
                self.restamp_for_laptop(topic_name, msg)
            publisher.publish(msg)
            self.topic_counts[topic_name] += 1
        except Exception as exc:
            self.get_logger().error(f"Failed to publish relayed message: {exc}")

    def restamp_for_laptop(self, topic_name: str, msg) -> None:
        stamp = self.get_clock().now().to_msg()

        if topic_name in {"/tf", "/tf_static"}:
            for transform in msg.transforms:
                transform.header.stamp = stamp
            return

        if hasattr(msg, "header"):
            msg.header.stamp = stamp

        if isinstance(msg, Path):
            for pose in msg.poses:
                pose.header.stamp = stamp

    def log_status(self) -> None:
        now = time.monotonic()
        elapsed = max(now - self.last_status, 0.001)
        active = [
            f"{topic}={count / elapsed:.1f}Hz"
            for topic, count in self.topic_counts.items()
            if count
        ]
        self.topic_counts = {topic_name: 0 for topic_name in TOPIC_TYPES}
        self.last_status = now
        data_state = "connected" if self.conn is not None else "waiting"
        command_state = f"on-demand:{self.args.robot_host}:{self.args.send_port}"
        if active:
            self.get_logger().info(
                f"data={data_state}, commands={command_state}, relayed: {', '.join(active)}"
            )
        else:
            self.get_logger().info(f"data={data_state}, commands={command_state}, no relayed samples yet")

    def close(self) -> None:
        self.close_receive_socket()
        try:
            self.server.close()
        except Exception:
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--recv-port", type=int, default=16000)
    parser.add_argument("--robot-host", default="127.0.0.1")
    parser.add_argument("--send-port", type=int, default=16001)
    parser.add_argument("--connect-timeout", type=float, default=2.0)
    parser.add_argument(
        "--preserve-robot-stamps",
        action="store_true",
        help="Do not rewrite relayed message stamps to the laptop clock.",
    )
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = LaptopRvizReceiver(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
