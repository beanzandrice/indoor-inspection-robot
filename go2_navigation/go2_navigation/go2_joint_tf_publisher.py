#!/usr/bin/env python3
"""Publish Go2 leg joint transforms from Unitree LowState motor positions."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from unitree_go.msg import LowState


JOINTS = (
    # Unitree motor index, joint name, parent, child, xyz, axis
    (0, "FR_hip_joint", "base", "FR_hip", (0.1934, -0.0465, 0.0), (1.0, 0.0, 0.0)),
    (1, "FR_thigh_joint", "FR_hip", "FR_thigh", (0.0, -0.0955, 0.0), (0.0, 1.0, 0.0)),
    (2, "FR_calf_joint", "FR_thigh", "FR_calf", (0.0, 0.0, -0.213), (0.0, 1.0, 0.0)),
    (3, "FL_hip_joint", "base", "FL_hip", (0.1934, 0.0465, 0.0), (1.0, 0.0, 0.0)),
    (4, "FL_thigh_joint", "FL_hip", "FL_thigh", (0.0, 0.0955, 0.0), (0.0, 1.0, 0.0)),
    (5, "FL_calf_joint", "FL_thigh", "FL_calf", (0.0, 0.0, -0.213), (0.0, 1.0, 0.0)),
    (6, "RR_hip_joint", "base", "RR_hip", (-0.1934, -0.0465, 0.0), (1.0, 0.0, 0.0)),
    (7, "RR_thigh_joint", "RR_hip", "RR_thigh", (0.0, -0.0955, 0.0), (0.0, 1.0, 0.0)),
    (8, "RR_calf_joint", "RR_thigh", "RR_calf", (0.0, 0.0, -0.213), (0.0, 1.0, 0.0)),
    (9, "RL_hip_joint", "base", "RL_hip", (-0.1934, 0.0465, 0.0), (1.0, 0.0, 0.0)),
    (10, "RL_thigh_joint", "RL_hip", "RL_thigh", (0.0, 0.0955, 0.0), (0.0, 1.0, 0.0)),
    (11, "RL_calf_joint", "RL_thigh", "RL_calf", (0.0, 0.0, -0.213), (0.0, 1.0, 0.0)),
)


def quaternion_about_axis(axis, angle):
    norm = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
    if norm == 0.0:
        return 0.0, 0.0, 0.0, 1.0

    scale = math.sin(angle * 0.5) / norm
    return axis[0] * scale, axis[1] * scale, axis[2] * scale, math.cos(angle * 0.5)


class Go2JointTfPublisher(Node):
    def __init__(self):
        super().__init__("go2_joint_tf_publisher")
        self.declare_parameter("lowstate_topics", "lowstate,lf/lowstate")
        self.declare_parameter("stale_switch_period", 1.0)
        self.declare_parameter("publish_hz", 20.0)

        topics_value = self.get_parameter("lowstate_topics").value
        self.lowstate_topics = [
            topic.strip()
            for topic in str(topics_value).split(",")
            if topic.strip()
        ]
        self.stale_switch_period = float(self.get_parameter("stale_switch_period").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.min_publish_period = 0.0 if self.publish_hz <= 0.0 else 1.0 / self.publish_hz
        self.active_topic = None
        self.last_message_time = 0.0
        self.last_publish_time = 0.0
        self.last_status = time.monotonic()
        self.message_count = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.lowstate_subscriptions = [
            self.create_subscription(
                LowState,
                topic,
                self.make_lowstate_callback(topic),
                qos,
            )
            for topic in self.lowstate_topics
        ]
        self.create_timer(1.0, self.publish_neutral_until_lowstate)
        self.create_timer(5.0, self.log_status)
        self.get_logger().info(
            "Listening for Unitree LowState on: %s" % ", ".join(self.lowstate_topics)
        )

    def make_lowstate_callback(self, topic):
        def callback(msg):
            now = time.monotonic()
            if self.active_topic and topic != self.active_topic:
                if now - self.last_message_time < self.stale_switch_period:
                    return

            if topic != self.active_topic:
                self.active_topic = topic
                self.get_logger().info("Using LowState topic for joint TF: %s" % topic)

            self.last_message_time = now
            if self.min_publish_period > 0.0 and now - self.last_publish_time < self.min_publish_period:
                return

            self.last_publish_time = now
            self.publish_joint_transforms(msg)
            self.message_count += 1

        return callback

    def publish_neutral_until_lowstate(self):
        if self.active_topic is None:
            self.publish_joint_transforms(None)

    def publish_joint_transforms(self, msg):
        stamp = self.get_clock().now().to_msg()
        transforms = []

        for motor_index, _joint_name, parent, child, xyz, axis in JOINTS:
            if msg is not None and motor_index >= len(msg.motor_state):
                continue

            angle = 0.0 if msg is None else float(msg.motor_state[motor_index].q)
            qx, qy, qz, qw = quaternion_about_axis(axis, angle)

            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.translation.x = xyz[0]
            transform.transform.translation.y = xyz[1]
            transform.transform.translation.z = xyz[2]
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            transforms.append(transform)

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    def log_status(self):
        now = time.monotonic()
        elapsed = max(now - self.last_status, 0.001)
        if self.active_topic:
            self.get_logger().info(
                "Publishing Go2 joint TF from %s at %.1f Hz"
                % (self.active_topic, self.message_count / elapsed)
            )
        else:
            self.get_logger().warn(
                "No LowState samples received yet; robot model joints will stay fixed"
            )
        self.message_count = 0
        self.last_status = now


def main():
    rclpy.init()
    node = Go2JointTfPublisher()
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
