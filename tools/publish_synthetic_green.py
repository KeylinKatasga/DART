#!/usr/bin/env python3
"""Publish a deterministic moving green circle for ROS end-to-end validation."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class SyntheticPublisher(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_green_light_publisher")
        self.image_publisher = self.create_publisher(
            Image, "/camera_node/image_raw", qos_profile_sensor_data
        )
        self.info_publisher = self.create_publisher(
            CameraInfo, "/camera_node/camera_info", qos_profile_sensor_data
        )
        self.frame = 0
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def publish_frame(self) -> None:
        width, height = 640, 480
        center_x = 220 + 2 * self.frame
        center_y = 180 + self.frame
        radius = 30
        pixels = bytearray(width * height * 3)
        for y in range(center_y - radius, center_y + radius + 1):
            for x in range(center_x - radius, center_x + radius + 1):
                if (x - center_x) ** 2 + (y - center_y) ** 2 <= radius**2:
                    pixels[(y * width + x) * 3 + 1] = 255
        stamp = self.get_clock().now().to_msg()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = "synthetic_camera_optical_frame"
        image.height = height
        image.width = width
        image.encoding = "bgr8"
        image.step = width * 3
        image.data = pixels
        self.image_publisher.publish(image)

        info = CameraInfo()
        info.header = image.header
        info.height = height
        info.width = width
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_publisher.publish(info)
        self.frame = (self.frame + 1) % 100


def main() -> int:
    rclpy.init()
    node = SyntheticPublisher()
    deadline = time.monotonic() + 8.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
