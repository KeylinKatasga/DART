#!/usr/bin/env python3
"""PTY smoke test for dart_fin_v1 RX and four-angle TX."""

import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import time
import tty

import rclpy
from ament_index_python.packages import get_package_prefix
from dart_interfaces.msg import FinCommand
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
    os.environ.setdefault("ROS_DOMAIN_ID", "75")
    rclpy.init()
    node = rclpy.create_node("dart_fin_mcu_smoke")
    received = {"rx": None}
    subscription = node.create_subscription(
        Rx, "/mcu_node/message/rx", lambda message: received.__setitem__("rx", message), 100
    )
    command_publisher = node.create_publisher(
        FinCommand, "/mcu_node/fin_command", 100
    )
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    process = subprocess.Popen(
        [
            executable("mcu_node", "mcu_node"),
            "--ros-args",
            "-p",
            f"device:={os.ttyname(slave)}",
            "-p",
            "protocol_mode:=dart_fin_v1",
            "-p",
            "actuator_output_enabled:=true",
        ]
    )
    rx_values = (1.0, 0.1, 0.2, 0.3, 12.0, -0.5, 0.01, 0.02, 0.03, 0.04)
    rx_payload = struct.pack("<10f", *rx_values)
    rx_frame = bytes([0x3E, len(rx_payload)]) + rx_payload + crc16(rx_payload)
    tx_bytes = bytearray()
    try:
        deadline = time.monotonic() + 8.0
        command_sent = False
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("mcu_node exited early")
            if received["rx"] is None:
                os.write(master, rx_frame)
            if received["rx"] is not None and command_publisher.get_subscription_count():
                command = FinCommand()
                command.valid = True
                command.armed = True
                command.neutral = False
                command.deflection_rad = [0.11, 0.22, 0.33, 0.44]
                command_publisher.publish(command)
                command_sent = True
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([master], [], [], 0.0)
            if readable:
                try:
                    tx_bytes.extend(os.read(master, 1024))
                except BlockingIOError:
                    pass
            if command_sent and len(tx_bytes) >= 20:
                break
            time.sleep(0.01)

        rx = received["rx"]
        assert rx is not None, "no dart_fin_v1 RX message"
        assert rx.imu_quat_w == 1.0 and rx.vehicle_x_vel == 12.0
        assert abs(rx.angle1 - 0.01) < 1e-6 and abs(rx.angle4 - 0.04) < 1e-6
        start = tx_bytes.find(b"\x3e\x10")
        assert start >= 0 and len(tx_bytes) >= start + 20, "no complete dart_fin_v1 TX frame"
        tx_frame = bytes(tx_bytes[start : start + 20])
        assert tx_frame[-2:] == crc16(tx_frame[2:-2]), "TX CRC mismatch"
        tx_angles = struct.unpack("<4f", tx_frame[2:-2])
        expected = (0.11, 0.22, 0.33, 0.44)
        assert all(abs(actual - wanted) < 1e-6 for actual, wanted in zip(tx_angles, expected))
        print(
            "PASS: dart_fin_v1 RX/TX; "
            f"velocity=({rx.vehicle_x_vel:.2f},{rx.vehicle_y_vel:.2f}) "
            f"feedback=({rx.angle1:.2f},{rx.angle2:.2f},{rx.angle3:.2f},{rx.angle4:.2f}) "
            f"tx={tx_angles}"
        )
        return 0
    finally:
        stop(process)
        del subscription
        os.close(master)
        os.close(slave)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
