#!/usr/bin/env python3
"""Relay GO2 Nav2 topics to a laptop and convert RViz goals into Nav2 actions."""

import argparse
import base64
import json
import socket
import struct
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage


TOPIC_TYPES = {
    "/tf": TFMessage,
    "/tf_static": TFMessage,
    "/map": OccupancyGrid,
    "/amcl_pose": PoseWithCovarianceStamped,
    "/odom": Odometry,
    "/scan": LaserScan,
    "/trans_cloud": PointCloud2,
    "/camera/image_raw": Image,
    "/plan": Path,
    "/local_plan": Path,
    "/global_costmap/costmap": OccupancyGrid,
    "/local_costmap/costmap": OccupancyGrid,
}

LATCHED_TOPICS = {"/map", "/tf_static", "/global_costmap/costmap", "/local_costmap/costmap"}


def subscription_qos(topic_name: str) -> QoSProfile:
    if topic_name in LATCHED_TOPICS:
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100 if topic_name == "/tf" else 20,
        reliability=ReliabilityPolicy.BEST_EFFORT
        if topic_name in {"/tf", "/scan", "/trans_cloud", "/camera/image_raw"}
        else ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def command_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class Go2RvizTcpBridge(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("go2_rviz_tcp_bridge")
        self.args = args
        self.send_sock = None
        self.recv_conn = None
        self.recv_buffer = b""
        self.cached_messages = {}
        self.topic_counts = {topic_name: 0 for topic_name in TOPIC_TYPES}
        self.last_sent_by_topic = {topic_name: 0.0 for topic_name in TOPIC_TYPES}
        self.last_status = time.monotonic()
        self.last_connect_warn = 0.0

        self.topic_subscriptions = [
            self.create_subscription(
                msg_type,
                topic_name,
                self.make_topic_callback(topic_name),
                subscription_qos(topic_name),
            )
            for topic_name, msg_type in TOPIC_TYPES.items()
        ]

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            command_qos(),
        )
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", command_qos())
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((args.recv_host, args.recv_port))
        self.server.listen(1)
        self.server.setblocking(False)

        self.create_timer(1.0, self.ensure_send_connection)
        self.create_timer(0.01, self.poll_return_socket)
        self.create_timer(5.0, self.log_status)

        self.get_logger().info(
            f"GO2 RViz TCP bridge ready; sending to {args.send_host}:{args.send_port}, "
            f"receiving commands on {args.recv_host}:{args.recv_port}, goal_mode={args.goal_mode}"
        )

    def ensure_send_connection(self) -> None:
        if self.send_sock is not None:
            return

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(self.args.connect_timeout)
            sock.connect((self.args.send_host, self.args.send_port))
            sock.settimeout(None)
            self.send_sock = sock
            self.get_logger().info(f"Connected data channel to laptop at {self.args.send_host}:{self.args.send_port}")
            self.send_cached_messages()
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_connect_warn > 5.0:
                self.get_logger().warn(f"Waiting for laptop data receiver: {exc}")
                self.last_connect_warn = now

    def send_cached_messages(self) -> None:
        for topic_name, msg in self.cached_messages.items():
            self.send_ros_message(topic_name, msg)

    def send_packet(self, payload: dict) -> None:
        if self.send_sock is None:
            return

        try:
            raw = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_sock.sendall(struct.pack(">I", len(raw)) + raw)
        except Exception as exc:
            self.get_logger().error(f"Data channel send failed: {exc}")
            self.close_send_socket()

    def send_ros_message(self, topic_name: str, msg) -> None:
        if self.send_sock is None:
            return

        payload = {
            "topic": topic_name,
            "stamp": time.time(),
            "data_b64": base64.b64encode(serialize_message(msg)).decode("ascii"),
        }
        self.send_packet(payload)

    def make_topic_callback(self, topic_name: str):
        def cb(msg) -> None:
            if topic_name in LATCHED_TOPICS:
                self.cached_messages[topic_name] = msg
            if self.send_sock is not None and self.should_send_topic(topic_name):
                self.send_ros_message(topic_name, msg)
                self.topic_counts[topic_name] += 1

        return cb

    def topic_max_hz(self, topic_name: str) -> float:
        if topic_name == "/camera/image_raw":
            return self.args.image_max_hz
        if topic_name == "/trans_cloud":
            return self.args.pointcloud_max_hz
        return 0.0

    def should_send_topic(self, topic_name: str) -> bool:
        max_hz = self.topic_max_hz(topic_name)
        if max_hz <= 0.0:
            return True

        now = time.monotonic()
        min_period = 1.0 / max_hz
        if now - self.last_sent_by_topic[topic_name] < min_period:
            return False

        self.last_sent_by_topic[topic_name] = now
        return True

    def close_send_socket(self) -> None:
        if self.send_sock is not None:
            try:
                self.send_sock.close()
            except Exception:
                pass
        self.send_sock = None

    def close_return_socket(self) -> None:
        if self.recv_conn is not None:
            try:
                self.recv_conn.close()
            except Exception:
                pass
        self.recv_conn = None
        self.recv_buffer = b""

    def poll_return_socket(self) -> None:
        if self.recv_conn is None:
            try:
                conn, addr = self.server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.setblocking(False)
                self.recv_conn = conn
                self.recv_buffer = b""
                self.get_logger().info(f"Laptop command channel connected from {addr}")
            except BlockingIOError:
                return
            except Exception as exc:
                self.get_logger().error(f"Return accept failed: {exc}")
                return

        try:
            chunk = self.recv_conn.recv(65536)
            if not chunk:
                self.get_logger().warn("Laptop command channel disconnected")
                self.close_return_socket()
                return
            self.recv_buffer += chunk
        except BlockingIOError:
            pass
        except Exception as exc:
            self.get_logger().error(f"Return receive failed: {exc}")
            self.close_return_socket()
            return

        while len(self.recv_buffer) >= 4:
            msg_len = struct.unpack(">I", self.recv_buffer[:4])[0]
            if len(self.recv_buffer) < 4 + msg_len:
                return

            raw = self.recv_buffer[4 : 4 + msg_len]
            self.recv_buffer = self.recv_buffer[4 + msg_len :]
            self.handle_command_payload(raw)

    def handle_command_payload(self, raw: bytes) -> None:
        try:
            payload = json.loads(raw.decode("utf-8"))
            topic_name = payload["topic"]
            msg_bytes = base64.b64decode(payload["data_b64"])

            if topic_name == "/initialpose":
                msg = deserialize_message(msg_bytes, PoseWithCovarianceStamped)
                if not msg.header.frame_id:
                    msg.header.frame_id = "map"
                self.initialpose_pub.publish(msg)
                self.get_logger().info("Published RViz /initialpose on GO2")
                return

            if topic_name == "/goal_pose":
                msg = deserialize_message(msg_bytes, PoseStamped)
                if not msg.header.frame_id:
                    msg.header.frame_id = "map"
                if self.args.goal_mode in {"topic", "both"}:
                    self.goal_pose_pub.publish(msg)
                    self.get_logger().info("Published RViz /goal_pose on GO2")
                if self.args.goal_mode in {"action", "both"}:
                    self.send_nav2_goal(msg)
                return

            self.get_logger().warn(f"Ignoring unsupported command topic: {topic_name}")
        except Exception as exc:
            self.get_logger().error(f"Failed handling laptop command: {exc}")

    def send_nav2_goal(self, pose_msg: PoseStamped) -> None:
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=self.args.action_wait_timeout):
            self.get_logger().error("Cannot send RViz goal; Nav2 navigate_to_pose action server is unavailable")
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose_msg
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        future = self.nav_to_pose_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)
        self.get_logger().info(
            f"Sent RViz goal to Nav2 action: frame={goal.pose.header.frame_id}, "
            f"x={goal.pose.pose.position.x:.3f}, y={goal.pose.pose.position.y:.3f}"
        )

    def goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the RViz goal")
            return
        self.get_logger().info("Nav2 accepted the RViz goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future) -> None:
        result = future.result()
        self.get_logger().info(f"Nav2 RViz goal finished with status {result.status}")

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
        data_state = "connected" if self.send_sock is not None else "waiting"
        command_state = "connected" if self.recv_conn is not None else "waiting"
        if active:
            self.get_logger().info(
                f"data={data_state}, commands={command_state}, relayed: {', '.join(active)}"
            )
        else:
            self.get_logger().info(f"data={data_state}, commands={command_state}, no relayed samples yet")

    def close(self) -> None:
        self.close_send_socket()
        self.close_return_socket()
        try:
            self.server.close()
        except Exception:
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--send-host", default="127.0.0.1")
    parser.add_argument("--send-port", type=int, default=16000)
    parser.add_argument("--recv-host", default="127.0.0.1")
    parser.add_argument("--recv-port", type=int, default=16001)
    parser.add_argument("--connect-timeout", type=float, default=2.0)
    parser.add_argument("--action-wait-timeout", type=float, default=2.0)
    parser.add_argument("--goal-mode", choices=["action", "topic", "both"], default="action")
    parser.add_argument("--image-max-hz", type=float, default=5.0)
    parser.add_argument("--pointcloud-max-hz", type=float, default=2.0)
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = Go2RvizTcpBridge(args)
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
