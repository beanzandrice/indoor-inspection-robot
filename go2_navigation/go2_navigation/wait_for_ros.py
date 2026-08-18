#!/usr/bin/env python3
"""Wait for ROS graph readiness, then exit so launch can start a dependent action."""

import argparse
import math
import re
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState


def non_negative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return parsed


def normalized_name(value: str) -> str:
    return value if value.startswith("/") else "/" + value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    readiness = parser.add_mutually_exclusive_group(required=True)
    readiness.add_argument("--topic", help="Wait until a topic appears in the ROS graph.")
    readiness.add_argument("--service", help="Wait until a service appears in the ROS graph.")
    readiness.add_argument(
        "--action",
        help="Wait until an action's send_goal service appears in the ROS graph.",
    )
    readiness.add_argument(
        "--lifecycle-node",
        help="Wait until a managed node reports the ACTIVE lifecycle state.",
    )
    parser.add_argument(
        "--timeout",
        type=non_negative_float,
        default=10.0,
        help="Fallback deadline; timeout still exits successfully so launch continues.",
    )
    parser.add_argument("--poll-period", type=non_negative_float, default=0.1)
    parser.add_argument("--label", default="dependency")
    args, _ = parser.parse_known_args()
    return args


def graph_contains(node, kind: str, target: str) -> bool:
    if kind == "topic":
        return normalized_name(target) in {
            name for name, _types in node.get_topic_names_and_types()
        }
    if kind == "service":
        return normalized_name(target) in {
            name for name, _types in node.get_service_names_and_types()
        }
    if kind == "action":
        send_goal_service = normalized_name(target).rstrip("/") + "/_action/send_goal"
        return send_goal_service in {
            name for name, _types in node.get_service_names_and_types()
        }
    raise ValueError("Unsupported graph readiness kind: {}".format(kind))


def wait_for_lifecycle_active(node, lifecycle_node: str, deadline: float, poll_period: float) -> bool:
    service_name = normalized_name(lifecycle_node).rstrip("/") + "/get_state"
    client = node.create_client(GetState, service_name)
    while rclpy.ok() and time.monotonic() < deadline:
        remaining = max(deadline - time.monotonic(), 0.0)
        if not client.wait_for_service(timeout_sec=min(poll_period, remaining)):
            continue
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=min(max(poll_period, 0.05), remaining),
        )
        if future.done() and future.exception() is None:
            response = future.result()
            if response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                return True
    return False


def main() -> None:
    args = parse_args()
    safe_label = re.sub(r"[^a-zA-Z0-9_]", "_", args.label).strip("_") or "dependency"
    rclpy.init()
    node = rclpy.create_node("wait_for_{}".format(safe_label.lower()))
    deadline = time.monotonic() + args.timeout
    ready = False
    target = ""
    kind = ""
    try:
        if args.lifecycle_node:
            kind = "lifecycle node"
            target = normalized_name(args.lifecycle_node)
            ready = wait_for_lifecycle_active(
                node,
                args.lifecycle_node,
                deadline,
                max(args.poll_period, 0.01),
            )
        else:
            for candidate_kind in ("topic", "service", "action"):
                candidate = getattr(args, candidate_kind)
                if candidate:
                    kind = candidate_kind
                    target = normalized_name(candidate)
                    break

            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(max(args.poll_period, 0.01), 0.25))
                if graph_contains(node, kind, target):
                    ready = True
                    break

        if ready:
            node.get_logger().info("{} {} is ready".format(kind, target))
        else:
            node.get_logger().warn(
                "Timed out after {:.1f}s waiting for {} {}; continuing with fallback startup".format(
                    args.timeout, kind, target
                )
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
