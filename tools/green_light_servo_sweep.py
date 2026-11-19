#!/usr/bin/env python3
"""Bounded synthetic left/right green-light sweep for the real MCU fin servos."""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from dart_interfaces.msg import FinCommand, GreenLight, GuidanceCommand, TargetState
from mcu_node.msg import Rx


def executable(package: str, name: str) -> str:
    return str(Path(get_package_prefix(package)) / "lib" / package / name)


def stop(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", action="store_true", help="send bounded commands to real servos")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--amplitude-rad", type=float, default=0.025)
    parser.add_argument("--frequency-hz", type=float, default=0.20)
    parser.add_argument("--foxglove-port", type=int, default=8765)
    parser.add_argument(
        "--config", default=str(Path.cwd() / "config" / "standard3.yaml")
    )
    args = parser.parse_args()
    if not 1.0 <= args.duration <= 30.0:
        parser.error("duration must be within [1, 30] seconds")
    if not 0.0 < args.amplitude_rad <= 0.05:
        parser.error("amplitude-rad must be within (0, 0.05] rad")
    if not 0.05 <= args.frequency_hz <= 0.5:
        parser.error("frequency-hz must be within [0.05, 0.5] Hz")
    if not Path(args.config).is_file():
        parser.error(f"config does not exist: {args.config}")
    return args


def main() -> int:
    args = parse_args()
    os.environ.setdefault("ROS_DOMAIN_ID", "77")
    rclpy.init()
    node = rclpy.create_node("green_light_servo_sweep")
    received = {"imu": None, "guidance": None, "fin": None, "fin_tx": None}
    qos = 20
    subscriptions = [
        node.create_subscription(
            Rx, "/mcu_node/message/rx", lambda msg: received.__setitem__("imu", msg), qos
        ),
        node.create_subscription(
            GuidanceCommand,
            "/dart_calculate_node/guidance",
            lambda msg: received.__setitem__("guidance", msg),
            qos,
        ),
        node.create_subscription(
            FinCommand,
            "/mcu_node/fin_command",
            lambda msg: received.__setitem__("fin", msg),
            qos,
        ),
        node.create_subscription(
            FinCommand,
            "/mcu_node/message/fin_tx",
            lambda msg: received.__setitem__("fin_tx", msg),
            qos,
        ),
    ]
    target_pub = node.create_publisher(TargetState, "/dart_estimate_node/target_state", qos)
    light_pub = node.create_publisher(GreenLight, "/test/green_light", qos)

    enabled = "true" if args.arm else "false"
    processes = [
        subprocess.Popen(
            [
                executable("mcu_node", "mcu_node"),
                "--ros-args",
                "--params-file",
                args.config,
                "-p",
                f"actuator_output_enabled:={enabled}",
            ]
        ),
        subprocess.Popen(
            [
                executable("dart_calculate_node", "dart_calculate_node"),
                "--ros-args",
                "--params-file",
                args.config,
                "-p",
                f"safety.output_enabled:={enabled}",
                "-p",
                "attitude.attack_control_limit:=0.05",
                "-p",
                "attitude.winding_control_limit:=0.05",
            ]
        ),
        subprocess.Popen(
            [
                executable("foxglove_bridge", "foxglove_bridge"),
                "--ros-args",
                "-p",
                f"port:={args.foxglove_port}",
            ]
        ),
    ]

    def publish_target(elapsed: float, valid: bool = True) -> tuple[float, float]:
        imu = received["imu"]
        phase = 2.0 * math.pi * args.frequency_hz * elapsed
        bearing = args.amplitude_rad * math.sin(phase) if valid else 0.0
        bearing_rate = (
            2.0 * math.pi * args.frequency_hz * args.amplitude_rad * math.cos(phase)
            if valid
            else 0.0
        )
        stamp = node.get_clock().now().to_msg()
        target = TargetState()
        target.header.stamp = stamp
        target.header.frame_id = "synthetic_green_light"
        target.sequence = int(elapsed * 100.0)
        target.valid = valid
        target.target_detected = valid
        target.imu_valid = imu is not None
        target.imu_quaternion_valid = imu is not None
        target.motion_model = TargetState.MOTION_SINUSOIDAL_VELOCITY
        target.bearing_x_rad = bearing
        target.bearing_rate_x_rad_s = bearing_rate
        target.predicted_bearing_x_rad = bearing
        target.predicted_bearing_rate_x_rad_s = bearing_rate
        target.radius_px = 24.0
        if imu is not None:
            target.imu_quaternion_w = imu.imu_quat_w
            target.imu_quaternion_x = imu.imu_quat_x
            target.imu_quaternion_y = imu.imu_quat_y
            target.imu_quaternion_z = imu.imu_quat_z
        target_pub.publish(target)

        light = GreenLight()
        light.header = target.header
        light.sequence = target.sequence
        light.detected = valid
        light.delta_valid = valid
        light.center_x_px = 672.48 + 1785.49 * math.tan(bearing)
        light.center_y_px = 559.90
        light.radius_px = target.radius_px
        light.velocity_x_px_s = 1785.49 * bearing_rate / (math.cos(bearing) ** 2)
        light.bearing_x_rad = bearing
        light.velocity_bearing_x_rad_s = bearing_rate
        light.score = 1.0 if valid else 0.0
        light_pub.publish(light)
        return bearing, light.center_x_px

    try:
        print(f"Foxglove: ws://localhost:{args.foxglove_port}")
        print(f"mode={'ARMED' if args.arm else 'DRY-RUN'} duration={args.duration:.1f}s")
        imu_deadline = time.monotonic() + 5.0
        while received["imu"] is None and time.monotonic() < imu_deadline:
            if any(process.poll() is not None for process in processes):
                raise RuntimeError("MCU, calculate node, or Foxglove bridge exited early")
            rclpy.spin_once(node, timeout_sec=0.05)
        if received["imu"] is None:
            raise RuntimeError("no MCU IMU quaternion received within 5 seconds")

        print("t[s]  x[px]  bearing[deg]  attack[deg]  winding[deg]  fins[deg]  tx")
        started = time.monotonic()
        next_publish = started
        next_print = started
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            if any(process.poll() is not None for process in processes):
                raise RuntimeError("MCU, calculate node, or Foxglove bridge exited early")
            elapsed = now - started
            if now >= next_publish:
                bearing, center_x = publish_target(elapsed)
                next_publish += 0.01
            rclpy.spin_once(node, timeout_sec=0.005)
            if now >= next_print and received["guidance"] and received["fin"]:
                guidance = received["guidance"]
                fin = received["fin"]
                fins_deg = [math.degrees(value) for value in fin.deflection_rad]
                transmitted = received["fin_tx"] is not None
                print(
                    f"{elapsed:4.1f} {center_x:7.1f} {math.degrees(bearing):12.3f} "
                    f"{math.degrees(guidance.attack_control):11.3f} "
                    f"{math.degrees(guidance.roll_control):12.3f} "
                    f"{str([round(value, 2) for value in fins_deg]):>26} "
                    f"{'yes' if transmitted else 'no'}"
                )
                next_print += 0.1
            time.sleep(0.001)

        received["fin_tx"] = None
        publish_target(args.duration, valid=False)
        neutral_deadline = time.monotonic() + 0.4
        while time.monotonic() < neutral_deadline and not (
            received["fin_tx"] is not None and received["fin_tx"].neutral
        ):
            rclpy.spin_once(node, timeout_sec=0.02)
        if args.arm and not (
            received["fin_tx"] is not None and received["fin_tx"].neutral
        ):
            raise RuntimeError("neutral servo command was not confirmed after serial write")
        print("PASS: sweep completed and neutral command confirmed")
        return 0
    finally:
        try:
            publish_target(0.0, valid=False)
            neutral_deadline = time.monotonic() + 0.25
            while rclpy.ok() and time.monotonic() < neutral_deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
        except Exception:
            pass
        for process in reversed(processes):
            stop(process)
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
