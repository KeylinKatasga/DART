#!/usr/bin/env python3
"""Synthetic ROS smoke test for PnP, both filters and disabled fin output."""

import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from dart_interfaces.msg import FinCommand, GreenLight, SelfState, TargetState
from mcu_node.msg import Rx
from sensor_msgs.msg import CameraInfo, Imu


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
    os.environ.setdefault("ROS_DOMAIN_ID", "76")
    rclpy.init()
    node = rclpy.create_node("filter_chain_smoke")
    received = {"target": None, "self": None, "fin": None}
    subscriptions = [
        node.create_subscription(
            TargetState,
            "/dart_estimate_node/target_state",
            lambda message: received.__setitem__("target", message),
            20,
        ),
        node.create_subscription(
            SelfState,
            "/dart_self_estimate_node/state",
            lambda message: received.__setitem__("self", message),
            20,
        ),
        node.create_subscription(
            FinCommand,
            "/mcu_node/fin_command",
            lambda message: received.__setitem__("fin", message),
            20,
        ),
    ]
    detection_pub = node.create_publisher(GreenLight, "/detect_node/detection", 20)
    camera_info_pub = node.create_publisher(CameraInfo, "/camera_node/camera_info", 20)
    mcu_pub = node.create_publisher(Rx, "/mcu_node/message/rx", 20)
    imu_pub = node.create_publisher(Imu, "/imu/data", 20)
    processes = [
        subprocess.Popen(
            [
                executable("dart_coordinate_node", "dart_coordinate_node"),
                "--ros-args",
                "-p",
                "solution_mode:=known_circle_pnp",
                "-p",
                "target_diameter_m:=0.1",
            ]
        ),
        subprocess.Popen(
            [
                executable("dart_estimate_node", "dart_estimate_node"),
                "--ros-args",
                "-p",
                "prediction.model:=singer",
                "-p",
                "prediction.horizon_s:=0.1",
            ]
        ),
        subprocess.Popen(
            [executable("dart_self_estimate_node", "dart_self_estimate_node")]
        ),
        subprocess.Popen([executable("dart_calculate_node", "dart_calculate_node")]),
    ]
    try:
        deadline = time.monotonic() + 10.0
        sequence = 0
        while time.monotonic() < deadline:
            if any(process.poll() is not None for process in processes):
                raise RuntimeError("one of the filter-chain nodes exited early")
            stamp = node.get_clock().now().to_msg()
            info = CameraInfo()
            info.header.stamp = stamp
            info.width = 1280
            info.height = 1024
            info.d = [0.0] * 5
            info.k = [1000.0, 0.0, 640.0, 0.0, 1000.0, 512.0, 0.0, 0.0, 1.0]
            camera_info_pub.publish(info)

            mcu = Rx()
            mcu.stamp = stamp
            mcu.imu_quat_w = 1.0
            mcu_pub.publish(mcu)

            imu = Imu()
            imu.header.stamp = stamp
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = -9.80665
            imu_pub.publish(imu)

            detection = GreenLight()
            detection.header.stamp = stamp
            detection.sequence = sequence
            detection.detected = True
            detection.center_x_px = 650.0 + 0.1 * sequence
            detection.center_y_px = 507.0
            detection.radius_px = 50.0
            detection.bearing_x_rad = 0.01
            detection.bearing_y_rad = 0.005
            detection_pub.publish(detection)
            sequence += 1
            rclpy.spin_once(node, timeout_sec=0.03)
            target = received["target"]
            self_state = received["self"]
            fin = received["fin"]
            if (
                target is not None
                and target.position_valid
                and self_state is not None
                and self_state.valid
                and fin is not None
                and fin.valid
            ):
                assert 0.8 < target.position_world_m[0] < 1.2
                assert abs(sum(value * value for value in [
                    self_state.quaternion_w,
                    self_state.quaternion_x,
                    self_state.quaternion_y,
                    self_state.quaternion_z,
                ]) - 1.0) < 1e-3
                assert not fin.armed and fin.neutral
                print(
                    "PASS: circle PnP -> Singer -> self EKF -> safe fin command; "
                    f"range={target.position_world_m[0]:.3f}m "
                    f"mahalanobis={self_state.innovation_mahalanobis:.3f}"
                )
                return 0
            time.sleep(0.01)
        raise AssertionError("filter chain did not produce a complete valid output")
    finally:
        for process in reversed(processes):
            stop(process)
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
