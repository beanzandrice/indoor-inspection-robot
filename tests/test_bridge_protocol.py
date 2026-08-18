import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

MODULE_DIR = Path(__file__).resolve().parents[1] / "go2_navigation" / "go2_navigation"
sys.path.insert(0, str(MODULE_DIR))

from bridge_protocol import (  # noqa: E402
    ALL_RELAY_TOPICS,
    DEFAULT_TOPIC_ALLOWLIST,
    PROTOCOL_BINARY,
    PROTOCOL_LEGACY_JSON,
    CommandValidationError,
    FrameParser,
    LatestPriorityQueue,
    ProtocolError,
    TopicRateLimiter,
    decode_payload,
    encode_payload,
    frame_message,
    parse_topic_allowlist,
    parse_topic_rates,
    prepare_pose_command,
)


@pytest.mark.parametrize("protocol", [PROTOCOL_BINARY, PROTOCOL_LEGACY_JSON])
def test_protocol_round_trip(protocol):
    cdr = b"\x00\x01\xffserialized-cdr"
    payload = encode_payload("/tf", cdr, protocol=protocol, sent_ns=1_234_567_890)

    decoded = decode_payload(payload)

    assert decoded.topic == "/tf"
    assert decoded.cdr == cdr
    assert decoded.sent_ns == 1_234_567_890
    assert decoded.protocol == protocol


def test_binary_protocol_avoids_base64_expansion():
    cdr = bytes(range(256)) * 100

    binary = encode_payload("/camera/image_raw", cdr, protocol=PROTOCOL_BINARY)
    legacy = encode_payload("/camera/image_raw", cdr, protocol=PROTOCOL_LEGACY_JSON)

    assert len(binary) < len(legacy)


def test_fragmented_and_coalesced_frames_are_parsed():
    first = frame_message("/tf", b"first")
    second = frame_message("/map", b"second")
    parser = FrameParser(max_frame_bytes=1024, max_buffer_bytes=2048)

    frames = []
    stream = first + second
    for byte in stream:
        frames.extend(parser.feed(bytes([byte])))

    assert [decode_payload(frame).cdr for frame in frames] == [b"first", b"second"]
    assert not parser.buffer


def test_parser_limits_work_per_poll_without_losing_buffered_frames():
    stream = b"".join(frame_message("/tf", str(index).encode()) for index in range(3))
    parser = FrameParser(max_frame_bytes=1024, max_buffer_bytes=4096)

    first_batch = parser.feed(stream, max_frames=2)
    second_batch = parser.feed(b"", max_frames=2)

    assert [decode_payload(frame).cdr for frame in first_batch] == [b"0", b"1"]
    assert [decode_payload(frame).cdr for frame in second_batch] == [b"2"]


def test_declared_oversized_frame_is_rejected_before_body_arrives():
    parser = FrameParser(max_frame_bytes=32, max_buffer_bytes=36)

    with pytest.raises(ProtocolError, match="Declared frame size"):
        parser.feed(struct.pack(">I", 33))


def test_receive_buffer_limit_is_enforced():
    parser = FrameParser(max_frame_bytes=16, max_buffer_bytes=20)

    with pytest.raises(ProtocolError, match="Receive buffer"):
        parser.feed(b"x" * 21)


@pytest.mark.parametrize(
    "payload",
    [
        b"not-json",
        b'{"topic":"/tf","data_b64":"!!!"}',
        b'{"topic":"relative","data_b64":""}',
        b"G2RB",
    ],
)
def test_malformed_payloads_are_rejected(payload):
    with pytest.raises(ProtocolError):
        decode_payload(payload)


def test_huge_legacy_timestamp_is_rejected_without_integer_overflow():
    with pytest.raises(ProtocolError, match="64-bit"):
        decode_payload(b'{"topic":"/tf","stamp":1e308,"data_b64":"Y2Ry"}')


def test_unknown_binary_version_is_rejected():
    payload = bytearray(encode_payload("/tf", b"cdr"))
    payload[4] = 99

    with pytest.raises(ProtocolError, match="version"):
        decode_payload(payload)


def test_binary_header_ranges_are_validated():
    with pytest.raises(ProtocolError, match="Timestamp"):
        encode_payload("/tf", b"cdr", sent_ns=-1)
    with pytest.raises(ProtocolError, match="32-bit"):
        frame_message("/tf", b"cdr", max_frame_bytes=1 << 32)


def pose_command(frame="map", quaternion=(0.0, 0.0, 0.0, 1.0), covariance=False):
    pose = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
        orientation=SimpleNamespace(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        ),
    )
    pose_field = (
        SimpleNamespace(pose=pose, covariance=[0.0] * 36)
        if covariance
        else pose
    )
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp="old"),
        pose=pose_field,
    )


def test_pose_command_is_validated_canonicalized_and_restamped():
    stamp = object()
    command = pose_command(frame="/map", covariance=True)

    prepared = prepare_pose_command(
        command,
        stamp,
        allowed_frame="map",
        require_covariance=True,
    )

    assert prepared is command
    assert command.header.frame_id == "map"
    assert command.header.stamp is stamp


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda msg: setattr(msg.header, "frame_id", "odom"), "allowed frame"),
        (lambda msg: setattr(msg.pose.position, "x", float("nan")), "NaN"),
        (
            lambda msg: setattr(msg.pose, "orientation", SimpleNamespace(x=0, y=0, z=0, w=0)),
            "nonzero",
        ),
        (
            lambda msg: setattr(msg.pose, "orientation", SimpleNamespace(x=0, y=0, z=0, w=2)),
            "norm",
        ),
    ],
)
def test_unsafe_goal_pose_commands_are_rejected(mutate, message):
    command = pose_command()
    mutate(command)

    with pytest.raises(CommandValidationError, match=message):
        prepare_pose_command(command, object())


def test_nonfinite_initial_pose_covariance_is_rejected():
    command = pose_command(covariance=True)
    command.pose.covariance[12] = float("inf")

    with pytest.raises(CommandValidationError, match="covariance"):
        prepare_pose_command(command, object(), require_covariance=True)


def test_rate_limiter_is_independent_per_topic():
    limiter = TopicRateLimiter({"/tf": 2.0, "/map": 1.0, "/scan": 0.0})

    assert limiter.allow("/tf", now=10.0)
    assert not limiter.allow("/tf", now=10.49)
    assert limiter.allow("/tf", now=10.5)
    assert limiter.allow("/map", now=10.1)
    assert limiter.allow("/scan", now=10.1)
    assert limiter.allow("/scan", now=10.1)


def test_topic_profiles_and_rate_overrides_are_validated():
    assert parse_topic_allowlist("default") == DEFAULT_TOPIC_ALLOWLIST
    assert parse_topic_allowlist("default", debug_all_topics=True) == ALL_RELAY_TOPICS
    assert parse_topic_allowlist("/tf,/map,/tf") == ("/tf", "/map")
    assert parse_topic_rates("/odom=10,/scan=2.5") == {"/odom": 10.0, "/scan": 2.5}

    with pytest.raises(ValueError):
        parse_topic_allowlist("/not-supported")
    with pytest.raises(ValueError):
        parse_topic_rates("/scan=-1")


def test_latest_priority_queue_replaces_same_topic():
    queue = LatestPriorityQueue(capacity=2)
    queue.put("/tf", "old", priority=0)
    queue.put("/tf", "new", priority=0)

    item = queue.get(timeout=0.0)

    assert item.key == "/tf"
    assert item.value == "new"
    assert queue.replaced == 1


def test_failed_item_does_not_overwrite_newer_queued_value():
    queue = LatestPriorityQueue(capacity=2)
    queue.put("/goal_pose", "old", priority=0)
    failed = queue.get(timeout=0.0)
    queue.put("/goal_pose", "new", priority=0)

    assert not queue.put_if_absent(failed.key, failed.value, failed.priority)
    assert queue.get(timeout=0.0).value == "new"


def test_critical_sample_evicts_bulk_and_is_dequeued_first():
    queue = LatestPriorityQueue(capacity=2)
    queue.put("/camera/image_raw", "image", priority=2)
    queue.put("/map", "map", priority=0)
    assert queue.put("/tf", "tf", priority=0)

    assert queue.get(timeout=0.0).key == "/map"
    assert queue.get(timeout=0.0).key == "/tf"
    assert queue.dropped == 1


def test_bulk_sample_cannot_displace_full_critical_queue():
    queue = LatestPriorityQueue(capacity=2)
    queue.put("/map", "map", priority=0)
    queue.put("/tf", "tf", priority=0)

    assert not queue.put("/camera/image_raw", "image", priority=2)
    assert len(queue) == 2
    assert queue.dropped == 1


def test_closed_queue_rejects_new_samples_and_unblocks_get():
    queue = LatestPriorityQueue(capacity=1)
    queue.close()

    assert not queue.put("/tf", "value", priority=0)
    assert queue.get(timeout=0.0) is None
