#!/usr/bin/env python3
"""ROS-independent framing, throttling, and queue utilities for the bridge."""

import base64
import json
import math
import struct
import threading
import time
from collections import namedtuple
from typing import Dict, List, Mapping, Optional, Tuple

PROTOCOL_BINARY = "binary"
PROTOCOL_LEGACY_JSON = "legacy-json"
SUPPORTED_PROTOCOLS = (PROTOCOL_BINARY, PROTOCOL_LEGACY_JSON)

PROTOCOL_MAGIC = b"G2RB"
PROTOCOL_VERSION = 1
MESSAGE_KIND_ROS_CDR = 1
_BINARY_HEADER = struct.Struct(">4sBBHQ")
_LENGTH_PREFIX = struct.Struct(">I")

DEFAULT_MAX_FRAME_BYTES = 16 * 1024 * 1024
DEFAULT_MAX_BUFFER_BYTES = 20 * 1024 * 1024
MAX_TOPIC_BYTES = 512
MAX_UINT32 = (1 << 32) - 1
MAX_UINT64 = (1 << 64) - 1

ALL_RELAY_TOPICS = (
    "/tf",
    "/tf_static",
    "/map",
    "/amcl_pose",
    "/odom",
    "/scan",
    "/trans_cloud",
    "/camera/image_raw",
    "/plan",
    "/local_plan",
    "/global_costmap/costmap",
    "/local_costmap/costmap",
)

# Costmaps are intentionally opt-in because they duplicate data Nav2 already
# uses on the robot and can dominate a constrained SSH link.
DEFAULT_TOPIC_ALLOWLIST = tuple(
    topic
    for topic in ALL_RELAY_TOPICS
    if topic not in {"/global_costmap/costmap", "/local_costmap/costmap"}
)

DEFAULT_TOPIC_RATES_HZ = {
    "/tf": 20.0,
    "/tf_static": 0.0,
    "/map": 1.0,
    "/amcl_pose": 10.0,
    "/odom": 20.0,
    "/scan": 10.0,
    "/trans_cloud": 0.5,
    "/camera/image_raw": 2.0,
    "/plan": 5.0,
    "/local_plan": 10.0,
    "/global_costmap/costmap": 1.0,
    "/local_costmap/costmap": 2.0,
}

CRITICAL_TOPICS = {
    "/tf",
    "/tf_static",
    "/map",
    "/amcl_pose",
    "/odom",
    "/scan",
    "/plan",
    "/local_plan",
    "/initialpose",
    "/goal_pose",
}
BULK_TOPICS = {"/trans_cloud", "/camera/image_raw"}

DecodedMessage = namedtuple("DecodedMessage", "topic cdr sent_ns protocol")
QueueItem = namedtuple("QueueItem", "key value priority sequence")


class ProtocolError(ValueError):
    """Raised when a bridge frame is malformed or exceeds configured limits."""


class CommandValidationError(ValueError):
    """Raised when an inbound motion command is not semantically safe to relay."""


def prepare_pose_command(
    message,
    stamp,
    allowed_frame: str = "map",
    require_covariance: bool = False,
    quaternion_tolerance: float = 1e-3,
):
    """Validate an inbound pose command, canonicalize its frame, and restamp it."""

    frame_id = getattr(getattr(message, "header", None), "frame_id", "")
    canonical_frame = allowed_frame.strip("/")
    if not canonical_frame:
        raise CommandValidationError("Allowed command frame cannot be empty")
    if frame_id and frame_id.strip("/") != canonical_frame:
        raise CommandValidationError(
            "Command frame {!r} is not the allowed frame {!r}".format(
                frame_id, canonical_frame
            )
        )

    pose_container = getattr(message, "pose", None)
    pose = getattr(pose_container, "pose", None) if require_covariance else pose_container
    if pose is None:
        raise CommandValidationError("Command is missing a pose")

    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if position is None or orientation is None:
        raise CommandValidationError("Command pose is incomplete")

    position_values = (position.x, position.y, position.z)
    quaternion_values = (orientation.x, orientation.y, orientation.z, orientation.w)
    try:
        numeric_values = tuple(float(value) for value in position_values + quaternion_values)
    except (TypeError, ValueError) as exc:
        raise CommandValidationError("Command pose contains non-numeric fields") from exc
    if not all(math.isfinite(value) for value in numeric_values):
        raise CommandValidationError("Command pose contains NaN or infinite fields")

    quaternion_norm = math.sqrt(sum(value * value for value in numeric_values[3:]))
    if quaternion_norm <= 1e-9:
        raise CommandValidationError("Command quaternion must be nonzero")
    if abs(quaternion_norm - 1.0) > quaternion_tolerance:
        raise CommandValidationError(
            "Command quaternion norm {:.6f} is outside tolerance".format(quaternion_norm)
        )

    if require_covariance:
        covariance = getattr(pose_container, "covariance", None)
        if covariance is None or len(covariance) != 36:
            raise CommandValidationError("Initial pose covariance must contain 36 values")
        try:
            covariance_values = tuple(float(value) for value in covariance)
        except (TypeError, ValueError) as exc:
            raise CommandValidationError("Initial pose covariance is not numeric") from exc
        if not all(math.isfinite(value) for value in covariance_values):
            raise CommandValidationError("Initial pose covariance contains NaN or infinity")

    message.header.frame_id = canonical_frame
    message.header.stamp = stamp
    return message


def topic_priority(topic_name: str) -> int:
    """Return a lower numeric priority for latency-sensitive data."""

    if topic_name in CRITICAL_TOPICS:
        return 0
    if topic_name in BULK_TOPICS:
        return 2
    return 1


def parse_topic_allowlist(value: str, debug_all_topics: bool = False) -> Tuple[str, ...]:
    """Parse a comma-separated relay allowlist with named default/all profiles."""

    if debug_all_topics:
        return ALL_RELAY_TOPICS

    raw_items = [item.strip() for item in (value or "default").split(",") if item.strip()]
    selected = []  # type: List[str]
    for item in raw_items:
        if item == "default":
            candidates = DEFAULT_TOPIC_ALLOWLIST
        elif item in {"all", "debug"}:
            candidates = ALL_RELAY_TOPICS
        else:
            candidates = (item,)

        for topic_name in candidates:
            if topic_name not in ALL_RELAY_TOPICS:
                raise ValueError("Unsupported relay topic: {}".format(topic_name))
            if topic_name not in selected:
                selected.append(topic_name)

    if not selected:
        raise ValueError("The relay topic allowlist cannot be empty")
    return tuple(selected)


def parse_topic_rates(value: str) -> Dict[str, float]:
    """Parse ``/topic=hz`` entries and validate supported, finite rates."""

    rates = {}  # type: Dict[str, float]
    if not value:
        return rates

    for raw_item in value.split(","):
        item = raw_item.strip()
        if not item:
            continue
        if "=" not in item:
            raise ValueError("Expected /topic=hz, got {!r}".format(item))
        topic_name, raw_rate = (part.strip() for part in item.split("=", 1))
        if topic_name not in ALL_RELAY_TOPICS:
            raise ValueError("Unsupported relay topic in rate limit: {}".format(topic_name))
        try:
            rate = float(raw_rate)
        except ValueError as exc:
            raise ValueError("Invalid rate for {}: {!r}".format(topic_name, raw_rate)) from exc
        if not math.isfinite(rate) or rate < 0.0:
            raise ValueError("Rate for {} must be finite and non-negative".format(topic_name))
        rates[topic_name] = rate
    return rates


class TopicRateLimiter:
    """Monotonic per-topic limiter; a non-positive rate means unlimited."""

    def __init__(self, rates_hz: Mapping[str, float]):
        self._rates_hz = dict(rates_hz)
        self._last_allowed = {}  # type: Dict[str, float]

    def allow(self, topic_name: str, now: Optional[float] = None) -> bool:
        rate = self._rates_hz.get(topic_name, 0.0)
        if rate <= 0.0:
            return True

        current = time.monotonic() if now is None else now
        previous = self._last_allowed.get(topic_name)
        if previous is not None and current - previous < 1.0 / rate:
            return False
        self._last_allowed[topic_name] = current
        return True


class LatestPriorityQueue:
    """A bounded thread-safe queue retaining only the latest value per key."""

    def __init__(self, capacity: int):
        if capacity <= 0:
            raise ValueError("Queue capacity must be positive")
        self.capacity = capacity
        self._items = {}  # type: Dict[str, QueueItem]
        self._sequence = 0
        self._closed = False
        self._condition = threading.Condition()
        self.replaced = 0
        self.dropped = 0

    def put(self, key: str, value, priority: int = 1) -> bool:
        with self._condition:
            if self._closed:
                return False

            self._sequence += 1
            if key in self._items:
                self.replaced += 1
                self._items[key] = QueueItem(key, value, priority, self._sequence)
                self._condition.notify()
                return True

            return self._put_new_locked(key, value, priority)

    def put_if_absent(self, key: str, value, priority: int = 1) -> bool:
        """Requeue a failed item only when no newer value for its key exists."""

        with self._condition:
            if self._closed or key in self._items:
                return False
            self._sequence += 1
            return self._put_new_locked(key, value, priority)

    def _put_new_locked(self, key: str, value, priority: int) -> bool:
        if len(self._items) >= self.capacity:
            eviction_key, eviction_item = max(
                self._items.items(),
                key=lambda pair: (pair[1].priority, -pair[1].sequence),
            )
            if priority > eviction_item.priority:
                self.dropped += 1
                return False
            del self._items[eviction_key]
            self.dropped += 1

        self._items[key] = QueueItem(key, value, priority, self._sequence)
        self._condition.notify()
        return True

    def get(self, timeout: Optional[float] = None) -> Optional[QueueItem]:
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._condition:
            while not self._items and not self._closed:
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0.0:
                    return None
                self._condition.wait(remaining)

            if not self._items:
                return None

            selected_key, selected = min(
                self._items.items(),
                key=lambda pair: (pair[1].priority, pair[1].sequence),
            )
            del self._items[selected_key]
            return selected

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def __len__(self) -> int:
        with self._condition:
            return len(self._items)


def _validate_topic(topic_name: str) -> bytes:
    if not isinstance(topic_name, str) or not topic_name.startswith("/"):
        raise ProtocolError("Topic name must be an absolute ROS topic")
    topic_bytes = topic_name.encode("utf-8")
    if not topic_bytes or len(topic_bytes) > MAX_TOPIC_BYTES:
        raise ProtocolError("Topic name is empty or too long")
    return topic_bytes


def encode_payload(
    topic_name: str,
    cdr: bytes,
    protocol: str = PROTOCOL_BINARY,
    sent_ns: Optional[int] = None,
) -> bytes:
    """Encode one ROS CDR sample without the outer length prefix."""

    topic_bytes = _validate_topic(topic_name)
    cdr_bytes = bytes(cdr)
    timestamp_ns = time.time_ns() if sent_ns is None else int(sent_ns)
    if timestamp_ns < 0 or timestamp_ns > MAX_UINT64:
        raise ProtocolError("Timestamp is outside the unsigned 64-bit range")

    if protocol == PROTOCOL_BINARY:
        return (
            _BINARY_HEADER.pack(
                PROTOCOL_MAGIC,
                PROTOCOL_VERSION,
                MESSAGE_KIND_ROS_CDR,
                len(topic_bytes),
                timestamp_ns,
            )
            + topic_bytes
            + cdr_bytes
        )

    if protocol == PROTOCOL_LEGACY_JSON:
        payload = {
            "topic": topic_name,
            "stamp": timestamp_ns / 1_000_000_000.0,
            "data_b64": base64.b64encode(cdr_bytes).decode("ascii"),
        }
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")

    raise ProtocolError("Unsupported bridge protocol: {}".format(protocol))


def frame_message(
    topic_name: str,
    cdr: bytes,
    protocol: str = PROTOCOL_BINARY,
    sent_ns: Optional[int] = None,
    max_frame_bytes: int = DEFAULT_MAX_FRAME_BYTES,
) -> bytes:
    if max_frame_bytes <= 0 or max_frame_bytes > MAX_UINT32:
        raise ProtocolError("Maximum frame size must fit the 32-bit length prefix")
    payload = encode_payload(topic_name, cdr, protocol=protocol, sent_ns=sent_ns)
    if len(payload) > max_frame_bytes:
        raise ProtocolError(
            "Encoded frame is {} bytes; maximum is {}".format(len(payload), max_frame_bytes)
        )
    return _LENGTH_PREFIX.pack(len(payload)) + payload


def decode_payload(payload: bytes) -> DecodedMessage:
    """Decode binary v1 or the legacy JSON/Base64 rollout format."""

    raw = bytes(payload)
    if raw.startswith(PROTOCOL_MAGIC):
        if len(raw) < _BINARY_HEADER.size:
            raise ProtocolError("Truncated binary bridge header")
        magic, version, kind, topic_len, sent_ns = _BINARY_HEADER.unpack_from(raw)
        if magic != PROTOCOL_MAGIC:
            raise ProtocolError("Invalid bridge magic")
        if version != PROTOCOL_VERSION:
            raise ProtocolError("Unsupported bridge protocol version: {}".format(version))
        if kind != MESSAGE_KIND_ROS_CDR:
            raise ProtocolError("Unsupported bridge message kind: {}".format(kind))
        if topic_len <= 0 or topic_len > MAX_TOPIC_BYTES:
            raise ProtocolError("Invalid topic length: {}".format(topic_len))
        topic_end = _BINARY_HEADER.size + topic_len
        if topic_end > len(raw):
            raise ProtocolError("Truncated topic name")
        try:
            topic_name = raw[_BINARY_HEADER.size:topic_end].decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ProtocolError("Topic name is not valid UTF-8") from exc
        _validate_topic(topic_name)
        return DecodedMessage(topic_name, raw[topic_end:], sent_ns, PROTOCOL_BINARY)

    try:
        legacy = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("Payload is neither binary v1 nor valid legacy JSON") from exc
    if not isinstance(legacy, dict):
        raise ProtocolError("Legacy payload must be a JSON object")
    topic_name = legacy.get("topic")
    data_b64 = legacy.get("data_b64")
    _validate_topic(topic_name)
    if not isinstance(data_b64, str):
        raise ProtocolError("Legacy payload is missing data_b64")
    try:
        cdr = base64.b64decode(data_b64, validate=True)
    except (ValueError, base64.binascii.Error) as exc:
        raise ProtocolError("Legacy data_b64 is invalid") from exc

    sent_ns = None
    stamp = legacy.get("stamp")
    if stamp is not None:
        if isinstance(stamp, bool) or not isinstance(stamp, (int, float)):
            raise ProtocolError("Legacy stamp must be numeric")
        if not math.isfinite(stamp) or stamp < 0.0:
            raise ProtocolError("Legacy stamp must be finite and non-negative")
        scaled_stamp = stamp * 1_000_000_000
        if not math.isfinite(scaled_stamp) or scaled_stamp > MAX_UINT64:
            raise ProtocolError("Legacy stamp is outside the unsigned 64-bit range")
        sent_ns = int(scaled_stamp)
    return DecodedMessage(topic_name, cdr, sent_ns, PROTOCOL_LEGACY_JSON)


class FrameParser:
    """Incrementally parse length-prefixed payloads with strict memory bounds."""

    def __init__(
        self,
        max_frame_bytes: int = DEFAULT_MAX_FRAME_BYTES,
        max_buffer_bytes: int = DEFAULT_MAX_BUFFER_BYTES,
    ):
        if max_frame_bytes <= 0 or max_frame_bytes > MAX_UINT32:
            raise ValueError("Maximum frame size must fit the 32-bit length prefix")
        if max_buffer_bytes < max_frame_bytes + _LENGTH_PREFIX.size:
            raise ValueError("Maximum buffer must fit one maximum-size frame")
        self.max_frame_bytes = max_frame_bytes
        self.max_buffer_bytes = max_buffer_bytes
        self.buffer = bytearray()

    def feed(self, data: bytes, max_frames: Optional[int] = None) -> List[bytes]:
        if max_frames is not None and max_frames <= 0:
            raise ValueError("max_frames must be positive")
        if len(self.buffer) + len(data) > self.max_buffer_bytes:
            raise ProtocolError(
                "Receive buffer would exceed {} bytes".format(self.max_buffer_bytes)
            )
        self.buffer.extend(data)

        frames = []  # type: List[bytes]
        while len(self.buffer) >= _LENGTH_PREFIX.size:
            frame_len = _LENGTH_PREFIX.unpack_from(self.buffer)[0]
            if frame_len <= 0 or frame_len > self.max_frame_bytes:
                raise ProtocolError(
                    "Declared frame size {} is outside 1..{}".format(
                        frame_len, self.max_frame_bytes
                    )
                )
            total_len = _LENGTH_PREFIX.size + frame_len
            if len(self.buffer) < total_len:
                break
            frames.append(bytes(self.buffer[_LENGTH_PREFIX.size:total_len]))
            del self.buffer[:total_len]
            if max_frames is not None and len(frames) >= max_frames:
                break
        return frames

    def clear(self) -> None:
        self.buffer.clear()
