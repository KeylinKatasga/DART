#!/usr/bin/env python3
"""Offline PTY smoke test for MCU RX decoding and estimate-node handoff."""

import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import time
import tty

import rclpy
from ament_index_python.packages import get_package_prefix
from dart_interfaces.msg import GreenLight
from estimate_node.msg import TargetEstimate
from mcu_node.msg import Rx


def crc16(data: bytes) -> bytes:
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xA001 if value & 1 else 0)
    return struct.pack("<H", value)


def executable(package: str, name: str) -> str:
    return str(Path(get_package_prefix(package)) / "lib" / package / name)


def stop(process: subprocess.Popen) -> None:
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main() -> int:
    os.environ.setdefault("ROS_DOMAIN_ID", "74")
    rclpy.init()
    node = rclpy.create_node("mcu_estimate_smoke")
    received = {"rx": None, "estimate": None}
    subscriptions = [
        node.create_subscription(
            Rx, "/mcu_node/message/rx", lambda message: received.__setitem__("rx", message), 100
        ),
        node.create_subscription(
            TargetEstimate,
            "/estimate_node/target",
            lambda message: received.__setitem__("estimate", message),
            100,
        ),
    ]
    detection_publisher = node.create_publisher(
        GreenLight, "/detect_node/detection", 10
    )
    master, slave = pty.openpty()
    tty.setraw(slave)
    payload = struct.pack(
        "<13fhhbBQ",
        12.5,
        -3.0,
        1.0,
        0.2,
        0.3,
        0.4,
        0.5,
        0.6,
        0.7,
        1.0,
        0.0,
        0.0,
        0.0,
        2,
        15,
        3,
        1,
        123456,
    )
    frame = bytes([0x3E, 66]) + payload + crc16(payload)
    processes = [
        subprocess.Popen(
            [
                executable("mcu_node", "mcu_node"),
                "--ros-args",
                "-p",
                f"device:={os.ttyname(slave)}",
            ]
        ),
        subprocess.Popen([executable("estimate_node", "estimate_node")]),
    ]
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and received["estimate"] is None:
            if any(process.poll() is not None for process in processes):
                raise RuntimeError("mcu_node or estimate_node exited early")
            os.write(master, frame)
            rclpy.spin_once(node, timeout_sec=0.02)
            if received["rx"] is not None and detection_publisher.get_subscription_count():
                detection = GreenLight()
                detection.header.stamp = node.get_clock().now().to_msg()
                detection.header.frame_id = "synthetic_camera_optical_frame"
                detection.sequence = 7
                detection.detected = True
                detection.delta_valid = True
                detection.center_x_px = 320.0
                detection.center_y_px = 240.0
                detection.radius_px = 30.0
                detection.bearing_x_rad = 0.01
                detection.bearing_y_rad = -0.02
                detection.velocity_bearing_x_rad_s = 0.1
                detection.velocity_bearing_y_rad_s = -0.2
                detection_publisher.publish(detection)
            time.sleep(0.01)

        rx = received["rx"]
        estimate = received["estimate"]
        assert rx is not None, "no decoded MCU Rx message"
        assert estimate is not None, "no TargetEstimate message"
        assert rx.imu_yaw == 12.5 and rx.device_sample_time_us == 123456
        assert estimate.valid and estimate.imu_valid and estimate.target_detected
        assert estimate.imu_yaw_raw == 12.5
        assert estimate.imu_quaternion_w == 1.0
        assert estimate.detection_sequence == 7
        assert not estimate.imu_units_verified
        print(
            "PASS: MCU frame decoded and estimate published; "
            f"rx_sequence={rx.sequence} imu_yaw_raw={estimate.imu_yaw_raw} "
            f"imu_age_s={estimate.imu_age_s:.6f}"
        )
        return 0
    finally:
        for process in reversed(processes):
            stop(process)
        del subscriptions
        os.close(master)
        os.close(slave)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
