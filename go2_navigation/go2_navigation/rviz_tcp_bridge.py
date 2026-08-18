#!/usr/bin/env python3
"""Relay GO2 Nav2 topics to a laptop and convert RViz goals into Nav2 actions."""

import argparse
import math
import socket
import threading
import time
from typing import Dict, Optional

import rclpy
from bridge_protocol import (
    ALL_RELAY_TOPICS,
    DEFAULT_MAX_BUFFER_BYTES,
    DEFAULT_MAX_FRAME_BYTES,
    DEFAULT_TOPIC_RATES_HZ,
    PROTOCOL_BINARY,
    SUPPORTED_PROTOCOLS,
    CommandValidationError,
    FrameParser,
    LatestPriorityQueue,
    ProtocolError,
    TopicRateLimiter,
    decode_payload,
    frame_message,
    parse_topic_allowlist,
    parse_topic_rates,
    prepare_pose_command,
    topic_priority,
)
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
if set(TOPIC_TYPES) != set(ALL_RELAY_TOPICS):
    raise RuntimeError("Bridge topic type registry is out of sync with bridge_protocol")

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
        reliability=(
            ReliabilityPolicy.BEST_EFFORT
            if topic_name in {"/tf", "/scan", "/trans_cloud", "/camera/image_raw"}
            else ReliabilityPolicy.RELIABLE
        ),
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
        self.allowed_topics = parse_topic_allowlist(
            args.topic_allowlist,
            debug_all_topics=args.debug_all_topics,
        )
        topic_rates = dict(DEFAULT_TOPIC_RATES_HZ)
        topic_rates["/tf"] = args.tf_max_hz
        topic_rates["/camera/image_raw"] = args.image_max_hz
        topic_rates["/trans_cloud"] = args.pointcloud_max_hz
        topic_rates.update(parse_topic_rates(args.topic_rate_limits))
        self.rate_limiter = TopicRateLimiter(topic_rates)

        self.send_queue = LatestPriorityQueue(args.queue_capacity)
        self.stop_event = threading.Event()
        self.send_socket_lock = threading.Lock()
        self.send_sock = None  # type: Optional[socket.socket]
        self.recv_conn = None  # type: Optional[socket.socket]
        self.recv_parser = FrameParser(args.max_frame_bytes, args.max_buffer_bytes)
        self.cached_messages: Dict[str, object] = {}
        self.cache_lock = threading.Lock()
        self.topic_counts = {topic_name: 0 for topic_name in self.allowed_topics}
        self.counts_lock = threading.Lock()
        self.last_status = time.monotonic()
        self.last_connect_warn = 0.0

        self.topic_subscriptions = [
            self.create_subscription(
                TOPIC_TYPES[topic_name],
                topic_name,
                self.make_topic_callback(topic_name),
                subscription_qos(topic_name),
            )
            for topic_name in self.allowed_topics
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

        self.writer_thread = threading.Thread(
            target=self.writer_loop,
            name="go2-rviz-bridge-writer",
            daemon=True,
        )
        self.writer_thread.start()

        self.create_timer(0.01, self.poll_return_socket)
        self.create_timer(5.0, self.log_status)

        self.get_logger().info(
            "GO2 RViz TCP bridge ready; sending to {}:{}, receiving commands on {}:{}, "
            "goal_mode={}, protocol={}, topics={}".format(
                args.send_host,
                args.send_port,
                args.recv_host,
                args.recv_port,
                args.goal_mode,
                args.protocol,
                ",".join(self.allowed_topics),
            )
        )

    def current_send_socket(self) -> Optional[socket.socket]:
        with self.send_socket_lock:
            return self.send_sock

    def connect_send_socket(self) -> Optional[socket.socket]:
        if self.stop_event.is_set():
            return None
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(self.args.connect_timeout)
            sock.connect((self.args.send_host, self.args.send_port))
            sock.settimeout(self.args.write_timeout)
            with self.send_socket_lock:
                if self.stop_event.is_set():
                    sock.close()
                    return None
                self.send_sock = sock
            self.get_logger().info(
                "Connected data channel to laptop at {}:{}".format(
                    self.args.send_host, self.args.send_port
                )
            )
            with self.cache_lock:
                cached = list(self.cached_messages.items())
            for topic_name, msg in cached:
                self.send_queue.put(topic_name, msg, topic_priority(topic_name))
            return sock
        except Exception as exc:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
            now = time.monotonic()
            if now - self.last_connect_warn > 5.0:
                self.get_logger().warn("Waiting for laptop data receiver: {}".format(exc))
                self.last_connect_warn = now
            return None

    def writer_loop(self) -> None:
        pending = None
        while not self.stop_event.is_set():
            sock = self.current_send_socket()
            if sock is None:
                sock = self.connect_send_socket()
                if sock is None:
                    self.stop_event.wait(self.args.reconnect_interval)
                    continue

            if pending is None:
                pending = self.send_queue.get(timeout=0.25)
                if pending is None:
                    continue

            try:
                cdr = serialize_message(pending.value)
                framed = frame_message(
                    pending.key,
                    cdr,
                    protocol=self.args.protocol,
                    max_frame_bytes=self.args.max_frame_bytes,
                )
            except ProtocolError as exc:
                self.get_logger().error(
                    "Dropping {} bridge sample: {}".format(pending.key, exc)
                )
                pending = None
                continue
            except Exception as exc:
                self.get_logger().error(
                    "Could not serialize {} bridge sample: {}".format(pending.key, exc)
                )
                pending = None
                continue

            try:
                sock.sendall(framed)
                with self.counts_lock:
                    self.topic_counts[pending.key] += 1
                pending = None
            except Exception as exc:
                self.get_logger().error("Data channel send failed: {}".format(exc))
                self.close_send_socket(sock)
                # A newer value for the same topic may already be queued. Drop
                # this failed sample instead of replaying stale stream data.
                pending = None
                self.stop_event.wait(self.args.reconnect_interval)

    def make_topic_callback(self, topic_name: str):
        def cb(msg) -> None:
            if topic_name in LATCHED_TOPICS:
                with self.cache_lock:
                    self.cached_messages[topic_name] = msg
            if self.rate_limiter.allow(topic_name):
                self.send_queue.put(topic_name, msg, topic_priority(topic_name))

        return cb

    def close_send_socket(self, expected_socket: Optional[socket.socket] = None) -> None:
        sock = None
        with self.send_socket_lock:
            if expected_socket is not None and self.send_sock is not expected_socket:
                return
            sock = self.send_sock
            self.send_sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass

    def close_return_socket(self) -> None:
        if self.recv_conn is not None:
            try:
                self.recv_conn.close()
            except Exception:
                pass
        self.recv_conn = None
        self.recv_parser.clear()

    def poll_return_socket(self) -> None:
        if self.recv_conn is None:
            try:
                conn, addr = self.server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.setblocking(False)
                self.recv_conn = conn
                self.recv_parser.clear()
                self.get_logger().info("Laptop command channel connected from {}".format(addr))
            except BlockingIOError:
                return
            except Exception as exc:
                self.get_logger().error("Return accept failed: {}".format(exc))
                return

        try:
            buffered_frames = self.recv_parser.feed(
                b"", max_frames=self.args.max_frames_per_poll
            )
        except ProtocolError as exc:
            self.get_logger().error("Closing malformed command channel: {}".format(exc))
            self.close_return_socket()
            return
        if buffered_frames:
            for raw in buffered_frames:
                self.handle_command_payload(raw)
            return

        chunk = b""
        try:
            chunk = self.recv_conn.recv(65536)
            if not chunk:
                self.get_logger().warn("Laptop command channel disconnected")
                self.close_return_socket()
                return
        except BlockingIOError:
            pass
        except Exception as exc:
            self.get_logger().error("Return receive failed: {}".format(exc))
            self.close_return_socket()
            return

        try:
            frames = self.recv_parser.feed(chunk, max_frames=self.args.max_frames_per_poll)
        except ProtocolError as exc:
            self.get_logger().error("Closing malformed command channel: {}".format(exc))
            self.close_return_socket()
            return

        for raw in frames:
            self.handle_command_payload(raw)

    def handle_command_payload(self, raw: bytes) -> None:
        try:
            decoded = decode_payload(raw)
            topic_name = decoded.topic

            if topic_name == "/initialpose":
                msg = deserialize_message(decoded.cdr, PoseWithCovarianceStamped)
                prepare_pose_command(
                    msg,
                    self.get_clock().now().to_msg(),
                    allowed_frame="map",
                    require_covariance=True,
                )
                self.initialpose_pub.publish(msg)
                self.get_logger().info("Published RViz /initialpose on GO2")
                return

            if topic_name == "/goal_pose":
                msg = deserialize_message(decoded.cdr, PoseStamped)
                prepare_pose_command(
                    msg,
                    self.get_clock().now().to_msg(),
                    allowed_frame="map",
                )
                if self.args.goal_mode in {"topic", "both"}:
                    self.goal_pose_pub.publish(msg)
                    self.get_logger().info("Published RViz /goal_pose on GO2")
                if self.args.goal_mode in {"action", "both"}:
                    self.send_nav2_goal(msg)
                return

            self.get_logger().warn("Ignoring unsupported command topic: {}".format(topic_name))
        except CommandValidationError as exc:
            self.get_logger().warn("Rejected unsafe laptop command: {}".format(exc))
        except Exception as exc:
            self.get_logger().error("Failed handling laptop command: {}".format(exc))

    def send_nav2_goal(self, pose_msg: PoseStamped) -> None:
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=self.args.action_wait_timeout):
            self.get_logger().error(
                "Cannot send RViz goal; Nav2 navigate_to_pose action server is unavailable"
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose_msg
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        future = self.nav_to_pose_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)
        self.get_logger().info(
            "Sent RViz goal to Nav2 action: frame={}, x={:.3f}, y={:.3f}".format(
                goal.pose.header.frame_id,
                goal.pose.pose.position.x,
                goal.pose.pose.position.y,
            )
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
        self.get_logger().info("Nav2 RViz goal finished with status {}".format(result.status))

    def log_status(self) -> None:
        now = time.monotonic()
        elapsed = max(now - self.last_status, 0.001)
        with self.counts_lock:
            active = [
                "{}={:.1f}Hz".format(topic, count / elapsed)
                for topic, count in self.topic_counts.items()
                if count
            ]
            self.topic_counts = {topic_name: 0 for topic_name in self.allowed_topics}
        self.last_status = now
        data_state = "connected" if self.current_send_socket() is not None else "waiting"
        command_state = "connected" if self.recv_conn is not None else "waiting"
        queue_state = "queued={}, replaced={}, dropped={}".format(
            len(self.send_queue), self.send_queue.replaced, self.send_queue.dropped
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
        self.send_queue.close()
        self.close_send_socket()
        self.close_return_socket()
        try:
            self.server.close()
        except Exception:
            pass
        self.writer_thread.join(timeout=max(self.args.write_timeout, 0.1) + 0.5)


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


def non_negative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--send-host", default="127.0.0.1")
    parser.add_argument("--send-port", type=int, default=16000)
    parser.add_argument("--recv-host", default="127.0.0.1")
    parser.add_argument("--recv-port", type=int, default=16001)
    parser.add_argument("--connect-timeout", type=positive_float, default=2.0)
    parser.add_argument("--write-timeout", type=positive_float, default=2.0)
    parser.add_argument("--reconnect-interval", type=positive_float, default=1.0)
    parser.add_argument("--action-wait-timeout", type=positive_float, default=2.0)
    parser.add_argument("--goal-mode", choices=["action", "topic", "both"], default="action")
    parser.add_argument("--protocol", choices=SUPPORTED_PROTOCOLS, default=PROTOCOL_BINARY)
    parser.add_argument("--max-frame-bytes", type=positive_int, default=DEFAULT_MAX_FRAME_BYTES)
    parser.add_argument("--max-buffer-bytes", type=positive_int, default=DEFAULT_MAX_BUFFER_BYTES)
    parser.add_argument("--max-frames-per-poll", type=positive_int, default=32)
    parser.add_argument("--queue-capacity", type=positive_int, default=32)
    parser.add_argument("--topic-allowlist", default="default")
    parser.add_argument(
        "--debug-all-topics",
        action="store_true",
        help="Relay every supported topic, including high-bandwidth costmaps.",
    )
    parser.add_argument(
        "--topic-rate-limits",
        default="",
        help="Comma-separated per-topic overrides such as /odom=10,/scan=5.",
    )
    parser.add_argument("--tf-max-hz", type=non_negative_float, default=20.0)
    parser.add_argument("--image-max-hz", type=non_negative_float, default=2.0)
    parser.add_argument("--pointcloud-max-hz", type=non_negative_float, default=0.5)
    args, _ = parser.parse_known_args()
    if args.max_buffer_bytes < args.max_frame_bytes + 4:
        parser.error("--max-buffer-bytes must be at least --max-frame-bytes + 4")
    try:
        parse_topic_allowlist(args.topic_allowlist, args.debug_all_topics)
        parse_topic_rates(args.topic_rate_limits)
    except ValueError as exc:
        parser.error(str(exc))
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
