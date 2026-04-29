#!/usr/bin/env python3
"""Publish the Go2 front camera as a ROS Image topic."""

import time

import gi
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


DEFAULT_GSTREAMER_PIPELINE = (
    "udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! "
    "application/x-rtp, media=video, encoding-name=H264 ! "
    "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
    "video/x-raw,width=1280,height=720,format=BGR ! "
    "appsink name=go2_camera_sink drop=true sync=false max-buffers=1"
)


class Go2CameraImagePublisher(Node):
    def __init__(self):
        super().__init__("go2_camera_image_publisher")
        Gst.init(None)

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_frame_id", "go2_front_camera")
        self.declare_parameter("target_fps", 10.0)
        self.declare_parameter("gstreamer_pipeline", DEFAULT_GSTREAMER_PIPELINE)
        self.declare_parameter("reconnect_period", 3.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.target_fps = float(self.get_parameter("target_fps").value)
        self.pipeline_text = self.get_parameter("gstreamer_pipeline").value
        self.reconnect_period = float(self.get_parameter("reconnect_period").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(Image, self.image_topic, qos)
        self.pipeline = None
        self.appsink = None
        self.bus = None
        self.last_open_attempt = 0.0
        self.last_warning = 0.0
        self.last_sample_time = 0.0
        self.frame_count = 0
        self.last_report = time.monotonic()

        timer_period = 1.0 / self.target_fps if self.target_fps > 0.0 else 0.1
        self.create_timer(timer_period, self.read_and_publish)
        self.get_logger().info(
            "Go2 camera publisher ready: topic=%s, fps=%.1f" % (self.image_topic, self.target_fps)
        )

    def open_pipeline(self):
        now = time.monotonic()
        if now - self.last_open_attempt < self.reconnect_period:
            return False

        self.last_open_attempt = now
        self.close_pipeline()

        try:
            self.pipeline = Gst.parse_launch(self.pipeline_text)
            self.appsink = self.pipeline.get_by_name("go2_camera_sink")
            if self.appsink is None:
                raise RuntimeError("GStreamer pipeline must include appsink name=go2_camera_sink")
            self.bus = self.pipeline.get_bus()
            result = self.pipeline.set_state(Gst.State.PLAYING)
            if result == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("GStreamer pipeline failed to enter PLAYING state")
        except Exception as exc:
            self.close_pipeline()
            self.throttled_warn("Could not open Go2 camera pipeline: %s" % exc)
            return False

        self.get_logger().info("Opened Go2 GStreamer camera stream")
        return True

    def close_pipeline(self):
        if self.pipeline is not None:
            try:
                self.pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass
        self.pipeline = None
        self.appsink = None
        self.bus = None

    def throttled_warn(self, message):
        now = time.monotonic()
        if now - self.last_warning >= 5.0:
            self.get_logger().warn(message)
            self.last_warning = now

    def poll_bus(self):
        if self.bus is None:
            return True

        while True:
            message = self.bus.pop_filtered(
                Gst.MessageType.ERROR | Gst.MessageType.EOS | Gst.MessageType.WARNING
            )
            if message is None:
                return True
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                self.throttled_warn("Go2 camera GStreamer error: %s (%s)" % (error, debug))
                self.close_pipeline()
                return False
            if message.type == Gst.MessageType.EOS:
                self.throttled_warn("Go2 camera GStreamer stream ended; reconnecting")
                self.close_pipeline()
                return False
            if message.type == Gst.MessageType.WARNING:
                warning, debug = message.parse_warning()
                self.throttled_warn("Go2 camera GStreamer warning: %s (%s)" % (warning, debug))

    def read_and_publish(self):
        if self.pipeline is None or self.appsink is None:
            self.open_pipeline()
            return

        if not self.poll_bus():
            return

        sample = self.appsink.emit("try-pull-sample", 0)
        if sample is None:
            if time.monotonic() - self.last_sample_time > 5.0:
                self.throttled_warn("Go2 camera stream is open but no frames have arrived yet")
            return

        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = int(structure.get_value("width"))
        height = int(structure.get_value("height"))
        encoding = str(structure.get_value("format")).lower()
        if encoding == "bgr":
            ros_encoding = "bgr8"
        elif encoding == "rgb":
            ros_encoding = "rgb8"
        else:
            self.throttled_warn("Unsupported Go2 camera format from GStreamer: %s" % encoding)
            return

        buffer = sample.get_buffer()
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            self.throttled_warn("Could not map Go2 camera frame buffer")
            return

        try:
            frame_data = bytes(map_info.data)
        finally:
            buffer.unmap(map_info)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame_id
        msg.height = height
        msg.width = width
        msg.encoding = ros_encoding
        msg.is_bigendian = 0
        msg.step = len(frame_data) // height if height else 0
        msg.data = frame_data
        self.publisher.publish(msg)

        self.last_sample_time = time.monotonic()
        self.frame_count += 1
        now = time.monotonic()
        if now - self.last_report >= 5.0:
            self.get_logger().info(
                "Publishing %s at %.1f Hz"
                % (self.image_topic, self.frame_count / (now - self.last_report))
            )
            self.frame_count = 0
            self.last_report = now

    def close(self):
        self.close_pipeline()


def main():
    rclpy.init()
    node = Go2CameraImagePublisher()
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
