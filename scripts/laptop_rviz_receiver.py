#!/usr/bin/env python3
"""Relay GO2 navigation topics into the laptop ROS graph for RViz2."""

import argparse
import math
import os
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Optional

os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

PROTOCOL_DIR = Path(__file__).resolve().parents[1] / "go2_navigation" / "go2_navigation"
if str(PROTOCOL_DIR) not in sys.path:
    sys.path.insert(0, str(PROTOCOL_DIR))

import rclpy  # noqa: E402
from bridge_protocol import (  # noqa: E402
    ALL_RELAY_TOPICS,
    DEFAULT_MAX_BUFFER_BYTES,
    DEFAULT_MAX_FRAME_BYTES,
    PROTOCOL_BINARY,
    SUPPORTED_PROTOCOLS,
    FrameParser,
    LatestPriorityQueue,
    ProtocolError,
    decode_payload,
    frame_message,
    topic_priority,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry, Path  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.serialization import deserialize_message, serialize_message  # noqa: E402
from sensor_msgs.msg import Image, LaserScan, PointCloud2  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

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
if set(TOPIC_TYPES) != set(ALL_RELAY_TOPICS):
    raise RuntimeError("Receiver topic type registry is out of sync with bridge_protocol")


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

        self.conn = None  # type: Optional[socket.socket]
        self.receive_parser = FrameParser(args.max_frame_bytes, args.max_buffer_bytes)
        self.command_queue = LatestPriorityQueue(args.command_queue_capacity)
        self.command_sock = None  # type: Optional[socket.socket]
        self.command_socket_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.topic_counts = {topic_name: 0 for topic_name in TOPIC_TYPES}
        self.last_status = time.monotonic()
        self.last_send_warn = 0.0

        self.command_writer_thread = threading.Thread(
            target=self.command_writer_loop,
            name="go2-command-writer",
            daemon=True,
        )
        self.command_writer_thread.start()

        self.create_timer(0.01, self.poll_receive_socket)
        self.create_timer(5.0, self.log_status)

        stamp_mode = (
            "preserving robot stamps"
            if args.preserve_robot_stamps
            else "restamping to laptop clock"
        )
        self.get_logger().info(
            "Listening for GO2 nav data on {}:{}; return channel target {}:{}; {}; "
            "protocol={}".format(
                args.listen_host,
                args.recv_port,
                args.robot_host,
                args.send_port,
                stamp_mode,
                args.protocol,
            )
        )

    def current_command_socket(self) -> Optional[socket.socket]:
        with self.command_socket_lock:
            return self.command_sock

    def connect_command_socket(self) -> Optional[socket.socket]:
        if self.stop_event.is_set():
            return None
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(self.args.connect_timeout)
            sock.connect((self.args.robot_host, self.args.send_port))
            sock.settimeout(self.args.write_timeout)
            with self.command_socket_lock:
                if self.stop_event.is_set():
                    sock.close()
                    return None
                self.command_sock = sock
            self.get_logger().info(
                "Connected command channel to GO2 at {}:{}".format(
                    self.args.robot_host, self.args.send_port
                )
            )
            return sock
        except Exception as exc:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
            now = time.monotonic()
            if now - self.last_send_warn > 2.0:
                self.get_logger().warn("Waiting for GO2 command channel: {}".format(exc))
                self.last_send_warn = now
            return None

    def close_command_socket(self, expected_socket: Optional[socket.socket] = None) -> None:
        sock = None
        with self.command_socket_lock:
            if expected_socket is not None and self.command_sock is not expected_socket:
                return
            sock = self.command_sock
            self.command_sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass

    def command_writer_loop(self) -> None:
        pending = None
        while not self.stop_event.is_set():
            sock = self.current_command_socket()
            if sock is None:
                sock = self.connect_command_socket()
                if sock is None:
                    self.stop_event.wait(self.args.reconnect_interval)
                    continue

            if pending is None:
                pending = self.command_queue.get(timeout=0.25)
                if pending is None:
                    continue

            message, queued_at = pending.value
            if time.monotonic() - queued_at > self.args.command_max_age:
                self.get_logger().warn(
                    "Dropping stale {} command after {:.1f}s".format(
                        pending.key, time.monotonic() - queued_at
                    )
                )
                pending = None
                continue

            try:
                cdr = serialize_message(message)
                framed = frame_message(
                    pending.key,
                    cdr,
                    protocol=self.args.protocol,
                    max_frame_bytes=self.args.max_frame_bytes,
                )
            except ProtocolError as exc:
                self.get_logger().error(
                    "Dropping {} command: {}".format(pending.key, exc)
                )
                pending = None
                continue
            except Exception as exc:
                self.get_logger().error(
                    "Could not serialize {} command: {}".format(pending.key, exc)
                )
                pending = None
                continue

            try:
                sock.sendall(framed)
                self.get_logger().info("Relayed {} to GO2".format(pending.key))
                pending = None
            except Exception as exc:
                now = time.monotonic()
                if now - self.last_send_warn > 2.0:
                    self.get_logger().error(
                        "Failed to relay {}: {}".format(pending.key, exc)
                    )
                    self.last_send_warn = now
                self.close_command_socket(sock)
                self.command_queue.put_if_absent(
                    pending.key,
                    pending.value,
                    pending.priority,
                )
                pending = None
                self.stop_event.wait(self.args.reconnect_interval)

    def relay_command(self, topic_name: str, msg) -> None:
        accepted = self.command_queue.put(
            topic_name,
            (msg, time.monotonic()),
            topic_priority(topic_name),
        )
        if not accepted:
            self.get_logger().warn("Command queue is closed; skipped {}".format(topic_name))

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
        self.receive_parser.clear()

    def poll_receive_socket(self) -> None:
        if self.conn is None:
            try:
                conn, addr = self.server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.setblocking(False)
                self.conn = conn
                self.receive_parser.clear()
                self.get_logger().info("GO2 data channel connected from {}".format(addr))
            except BlockingIOError:
                return
            except Exception as exc:
                self.get_logger().error("Accept failed: {}".format(exc))
                return

        try:
            buffered_frames = self.receive_parser.feed(
                b"", max_frames=self.args.max_frames_per_poll
            )
        except ProtocolError as exc:
            self.get_logger().error("Closing malformed data channel: {}".format(exc))
            self.close_receive_socket()
            return
        if buffered_frames:
            for raw in buffered_frames:
                self.publish_relayed_payload(raw)
            return

        chunk = b""
        try:
            chunk = self.conn.recv(65536)
            if not chunk:
                self.get_logger().warn("GO2 data channel disconnected")
                self.close_receive_socket()
                return
        except BlockingIOError:
            pass
        except Exception as exc:
            self.get_logger().error("Receive failed: {}".format(exc))
            self.close_receive_socket()
            return

        try:
            frames = self.receive_parser.feed(chunk, max_frames=self.args.max_frames_per_poll)
        except ProtocolError as exc:
            self.get_logger().error("Closing malformed data channel: {}".format(exc))
            self.close_receive_socket()
            return

        for raw in frames:
            self.publish_relayed_payload(raw)

    def publish_relayed_payload(self, raw: bytes) -> None:
        try:
            decoded = decode_payload(raw)
            topic_name = decoded.topic
            if topic_name not in self.topic_publishers:
                self.get_logger().warn("Skipping unknown relayed topic: {}".format(topic_name))
                return

            publisher, msg_type = self.topic_publishers[topic_name]
            msg = deserialize_message(decoded.cdr, msg_type)
            if not self.args.preserve_robot_stamps:
                self.restamp_for_laptop(topic_name, msg)
            publisher.publish(msg)
            self.topic_counts[topic_name] += 1
        except Exception as exc:
            self.get_logger().error("Failed to publish relayed message: {}".format(exc))

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
            "{}={:.1f}Hz".format(topic, count / elapsed)
            for topic, count in self.topic_counts.items()
            if count
        ]
        self.topic_counts = {topic_name: 0 for topic_name in TOPIC_TYPES}
        self.last_status = now
        data_state = "connected" if self.conn is not None else "waiting"
        command_state = (
            "connected" if self.current_command_socket() is not None else "waiting"
        )
        queue_state = "queued={}, replaced={}, dropped={}".format(
            len(self.command_queue),
            self.command_queue.replaced,
            self.command_queue.dropped,
        )
        if active:
            self.get_logger().info(
                "data={}, commands={}, {}, relayed: {}".format(
                    data_state, command_state, queue_state, ", ".join(active)
                )
            )
        else:
            self.get_logger().info(
                "data={}, commands={}, {}, no relayed samples yet".format(
                    data_state, command_state, queue_state
                )
            )

    def close(self) -> None:
        self.stop_event.set()
        self.command_queue.close()
        self.close_command_socket()
        self.close_receive_socket()
        try:
            self.server.close()
        except Exception:
            pass
        self.command_writer_thread.join(timeout=max(self.args.write_timeout, 0.1) + 0.5)


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--recv-port", type=int, default=16000)
    parser.add_argument("--robot-host", default="127.0.0.1")
    parser.add_argument("--send-port", type=int, default=16001)
    parser.add_argument("--connect-timeout", type=positive_float, default=2.0)
    parser.add_argument("--write-timeout", type=positive_float, default=2.0)
    parser.add_argument("--reconnect-interval", type=positive_float, default=1.0)
    parser.add_argument("--command-max-age", type=positive_float, default=5.0)
    parser.add_argument("--command-queue-capacity", type=positive_int, default=4)
    parser.add_argument("--protocol", choices=SUPPORTED_PROTOCOLS, default=PROTOCOL_BINARY)
    parser.add_argument("--max-frame-bytes", type=positive_int, default=DEFAULT_MAX_FRAME_BYTES)
    parser.add_argument("--max-buffer-bytes", type=positive_int, default=DEFAULT_MAX_BUFFER_BYTES)
    parser.add_argument("--max-frames-per-poll", type=positive_int, default=32)
    parser.add_argument(
        "--preserve-robot-stamps",
        action="store_true",
        help="Do not rewrite relayed message stamps to the laptop clock.",
    )
    args, _ = parser.parse_known_args()
    if args.max_buffer_bytes < args.max_frame_bytes + 4:
        parser.error("--max-buffer-bytes must be at least --max-frame-bytes + 4")
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
