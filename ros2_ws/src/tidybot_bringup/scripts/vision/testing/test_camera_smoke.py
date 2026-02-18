#!/usr/bin/env python3
"""
Finite camera smoke test for TidyBot hardware.

What it checks:
- Required topics are publishing.
- RGB/depth stream rates are stable enough.
- Pan-tilt state topic is present.

No infinite loops. Script exits with a clear PASS/FAIL summary.
"""

import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, JointState


@dataclass
class StreamStats:
    count: int = 0
    first_ts: float | None = None
    last_ts: float | None = None
    last_arrival: float | None = None
    max_gap_s: float = 0.0

    def on_msg(self, arrival_s: float) -> None:
        self.count += 1
        if self.first_ts is None:
            self.first_ts = arrival_s
        self.last_ts = arrival_s
        if self.last_arrival is not None:
            gap = arrival_s - self.last_arrival
            if gap > self.max_gap_s:
                self.max_gap_s = gap
        self.last_arrival = arrival_s

    def fps(self) -> float:
        if self.count <= 1 or self.first_ts is None or self.last_ts is None:
            return 0.0
        dt = self.last_ts - self.first_ts
        if dt <= 0:
            return 0.0
        return (self.count - 1) / dt


class CameraSmokeTest(Node):
    def __init__(self) -> None:
        super().__init__("camera_smoke_test")

        self.rgb_stats = StreamStats()
        self.depth_stats = StreamStats()
        self.pan_tilt_stats = StreamStats()
        self.got_color_info = False
        self.got_depth_info = False

        self.create_subscription(Image, "/camera/color/image_raw", self._on_rgb, 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, 10)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._on_color_info, 10)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info", self._on_depth_info, 10)
        self.create_subscription(JointState, "/camera/pan_tilt_state", self._on_pan_tilt_state, 10)

    def _on_rgb(self, _: Image) -> None:
        self.rgb_stats.on_msg(time.time())

    def _on_depth(self, _: Image) -> None:
        self.depth_stats.on_msg(time.time())

    def _on_color_info(self, _: CameraInfo) -> None:
        self.got_color_info = True

    def _on_depth_info(self, _: CameraInfo) -> None:
        self.got_depth_info = True

    def _on_pan_tilt_state(self, _: JointState) -> None:
        self.pan_tilt_stats.on_msg(time.time())


def print_summary(node: CameraSmokeTest, duration_s: float, min_fps: float, max_gap_s: float) -> bool:
    rgb_fps = node.rgb_stats.fps()
    depth_fps = node.depth_stats.fps()
    pt_fps = node.pan_tilt_stats.fps()

    rgb_ok = node.rgb_stats.count > 0 and rgb_fps >= min_fps and node.rgb_stats.max_gap_s <= max_gap_s
    depth_ok = node.depth_stats.count > 0 and depth_fps >= min_fps and node.depth_stats.max_gap_s <= max_gap_s
    info_ok = node.got_color_info and node.got_depth_info
    pan_tilt_ok = node.pan_tilt_stats.count > 0
    passed = rgb_ok and depth_ok and info_ok and pan_tilt_ok

    print("\n===== Camera Smoke Test Summary =====")
    print(f"Duration: {duration_s:.1f}s")
    print(f"RGB     : count={node.rgb_stats.count:4d}, fps={rgb_fps:6.2f}, max_gap={node.rgb_stats.max_gap_s:5.3f}s")
    print(f"Depth   : count={node.depth_stats.count:4d}, fps={depth_fps:6.2f}, max_gap={node.depth_stats.max_gap_s:5.3f}s")
    print(f"PanTilt : count={node.pan_tilt_stats.count:4d}, fps={pt_fps:6.2f}")
    print(f"CameraInfo received: color={node.got_color_info}, depth={node.got_depth_info}")
    print(f"Thresholds: min_fps={min_fps:.2f}, max_gap_s={max_gap_s:.3f}")
    print(f"RESULT: {'PASS' if passed else 'FAIL'}")

    if not passed:
        reasons = []
        if not rgb_ok:
            reasons.append("RGB stream unhealthy")
        if not depth_ok:
            reasons.append("Depth stream unhealthy")
        if not info_ok:
            reasons.append("CameraInfo missing")
        if not pan_tilt_ok:
            reasons.append("Pan-tilt state missing")
        print("Reasons:", "; ".join(reasons))
    print("====================================\n")
    return passed


def main() -> None:
    # ---------------------- Editable hyperparameters ---------------------- #
    TEST_DURATION_S = 10.0
    SPIN_TIMEOUT_S = 0.05
    MIN_EXPECTED_FPS = 5.0
    MAX_ALLOWED_GAP_S = 0.5
    # --------------------------------------------------------------------- #

    rclpy.init()
    node = CameraSmokeTest()
    start = time.time()
    end = start + TEST_DURATION_S

    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)
    finally:
        print_summary(
            node=node,
            duration_s=(time.time() - start),
            min_fps=MIN_EXPECTED_FPS,
            max_gap_s=MAX_ALLOWED_GAP_S,
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

