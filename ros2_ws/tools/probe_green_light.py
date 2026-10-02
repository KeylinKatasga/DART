#!/usr/bin/env python3
"""Wait for one positive GreenLight message during live hardware validation."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node

from dart_interfaces.msg import GreenLight


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("green_light_validation_probe")
        self.result = None
        self.subscription = self.create_subscription(
            GreenLight, "/detect_node/detection", self.receive, 10
        )

    def receive(self, message: GreenLight) -> None:
        if message.detected and self.result is None:
            self.result = {
                "sequence": message.sequence,
                "center_px": [message.center_x_px, message.center_y_px],
                "radius_px": message.radius_px,
                "delta_valid": message.delta_valid,
                "delta_px": [message.delta_x_px, message.delta_y_px],
                "velocity_px_s": [
                    message.velocity_x_px_s,
                    message.velocity_y_px_s,
                ],
                "bearing_rad": [message.bearing_x_rad, message.bearing_y_rad],
                "delta_bearing_rad": [
                    message.delta_bearing_x_rad,
                    message.delta_bearing_y_rad,
                ],
                "bearing_velocity_rad_s": [
                    message.velocity_bearing_x_rad_s,
                    message.velocity_bearing_y_rad_s,
                ],
                "circularity": message.circularity,
                "fill_ratio": message.fill_ratio,
                "score": message.score,
                "processing_ms": message.processing_ms,
            }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--require-delta", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.result is not None and (
                not args.require_delta or node.result["delta_valid"]
            ):
                break
            if node.result is not None:
                node.result = None
        if node.result is None:
            print("no positive detection before timeout")
            return 2
        print(json.dumps(node.result, ensure_ascii=False, indent=2))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
