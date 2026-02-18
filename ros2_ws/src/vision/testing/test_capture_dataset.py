#!/usr/bin/env python3
"""
Finite RGB-D capture script with optional fixed pan/tilt setpoint.

Captures synchronized-ish RGB/depth frame pairs and saves:
- RGB image
- Raw depth (16-bit PNG, mm)
- Colorized depth preview
- Per-frame JSON metadata

No infinite loops. Exits after NUM_FRAMES or DURATION_S.
"""

import json
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray

try:
    import cv2
    from cv_bridge import CvBridge

    HAS_CV = True
except ImportError:
    HAS_CV = False


@dataclass
class FramePacket:
    stamp_ros: tuple[int, int]
    arrival_wall_s: float
    msg: Image


class CaptureDatasetNode(Node):
    def __init__(self) -> None:
        super().__init__("capture_dataset_test")
        self.cv_bridge = CvBridge()

        self.latest_rgb: Optional[FramePacket] = None
        self.latest_depth: Optional[FramePacket] = None
        self.latest_color_info: Optional[CameraInfo] = None
        self.latest_depth_info: Optional[CameraInfo] = None
        self.latest_pan_tilt: list[float] = [float("nan"), float("nan")]

        self.rgb_count = 0
        self.depth_count = 0
        self.pairs_saved = 0
        self.depth_invalid_ratio_samples: list[float] = []
        self.depth_age_ms_samples: list[float] = []

        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, "/camera/pan_tilt_cmd", 10)

        self.create_subscription(Image, "/camera/color/image_raw", self._on_rgb, 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, 10)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._on_color_info, 10)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info", self._on_depth_info, 10)
        self.create_subscription(JointState, "/camera/pan_tilt_state", self._on_pan_tilt_state, 10)

    def _on_rgb(self, msg: Image) -> None:
        self.rgb_count += 1
        self.latest_rgb = FramePacket(
            stamp_ros=(msg.header.stamp.sec, msg.header.stamp.nanosec),
            arrival_wall_s=time.time(),
            msg=msg,
        )

    def _on_depth(self, msg: Image) -> None:
        self.depth_count += 1
        self.latest_depth = FramePacket(
            stamp_ros=(msg.header.stamp.sec, msg.header.stamp.nanosec),
            arrival_wall_s=time.time(),
            msg=msg,
        )

    def _on_color_info(self, msg: CameraInfo) -> None:
        self.latest_color_info = msg

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self.latest_depth_info = msg

    def _on_pan_tilt_state(self, msg: JointState) -> None:
        pan = float("nan")
        tilt = float("nan")
        if "camera_pan" in msg.name:
            pan = msg.position[msg.name.index("camera_pan")]
        if "camera_tilt" in msg.name:
            tilt = msg.position[msg.name.index("camera_tilt")]
        if np.isnan(pan) and len(msg.position) >= 1:
            pan = msg.position[0]
        if np.isnan(tilt) and len(msg.position) >= 2:
            tilt = msg.position[1]
        self.latest_pan_tilt = [float(pan), float(tilt)]

    def publish_pan_tilt(self, pan: float, tilt: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(pan), float(tilt)]
        self.pan_tilt_pub.publish(msg)


def ensure_output_dir(base_path: str) -> Path:
    ts = time.strftime("%Y%m%d_%H%M%S")
    out = Path(base_path).expanduser().resolve() / ts
    out.mkdir(parents=True, exist_ok=True)
    (out / "rgb").mkdir(exist_ok=True)
    (out / "depth_raw").mkdir(exist_ok=True)
    (out / "depth_viz").mkdir(exist_ok=True)
    (out / "meta").mkdir(exist_ok=True)
    return out


def depth_to_u16_mm(node: CaptureDatasetNode, depth_msg: Image) -> np.ndarray:
    if depth_msg.encoding == "16UC1":
        depth_u16 = node.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        return depth_u16
    if depth_msg.encoding == "32FC1":
        depth_f = node.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        depth_mm = np.nan_to_num(depth_f, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
        depth_u16 = np.clip(depth_mm, 0, 65535).astype(np.uint16)
        return depth_u16
    depth_any = node.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    if depth_any.dtype != np.uint16:
        depth_any = np.clip(depth_any, 0, 65535).astype(np.uint16)
    return depth_any


def save_pair(node: CaptureDatasetNode, out_dir: Path, idx: int) -> bool:
    if node.latest_rgb is None or node.latest_depth is None:
        return False

    rgb_pkt = node.latest_rgb
    depth_pkt = node.latest_depth

    rgb = node.cv_bridge.imgmsg_to_cv2(rgb_pkt.msg, desired_encoding="bgr8")
    depth_u16 = depth_to_u16_mm(node, depth_pkt.msg)

    rgb_name = out_dir / "rgb" / f"rgb_{idx:04d}.png"
    depth_raw_name = out_dir / "depth_raw" / f"depth_{idx:04d}.png"
    depth_viz_name = out_dir / "depth_viz" / f"depth_viz_{idx:04d}.png"
    meta_name = out_dir / "meta" / f"meta_{idx:04d}.json"

    cv2.imwrite(str(rgb_name), rgb)
    cv2.imwrite(str(depth_raw_name), depth_u16)

    depth_norm = cv2.normalize(depth_u16, None, 0, 255, cv2.NORM_MINMAX)
    depth_u8 = depth_norm.astype(np.uint8)
    depth_viz = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
    cv2.imwrite(str(depth_viz_name), depth_viz)

    valid_mask = depth_u16 > 0
    invalid_ratio = 1.0 - float(np.count_nonzero(valid_mask)) / float(depth_u16.size)
    depth_age_ms = abs(rgb_pkt.arrival_wall_s - depth_pkt.arrival_wall_s) * 1000.0
    node.depth_invalid_ratio_samples.append(invalid_ratio)
    node.depth_age_ms_samples.append(depth_age_ms)

    color_info = node.latest_color_info
    depth_info = node.latest_depth_info
    meta = {
        "index": idx,
        "wall_time_s": time.time(),
        "rgb_stamp": {"sec": rgb_pkt.stamp_ros[0], "nanosec": rgb_pkt.stamp_ros[1]},
        "depth_stamp": {"sec": depth_pkt.stamp_ros[0], "nanosec": depth_pkt.stamp_ros[1]},
        "rgb_frame_id": rgb_pkt.msg.header.frame_id,
        "depth_frame_id": depth_pkt.msg.header.frame_id,
        "rgb_encoding": rgb_pkt.msg.encoding,
        "depth_encoding": depth_pkt.msg.encoding,
        "rgb_size": {"width": rgb_pkt.msg.width, "height": rgb_pkt.msg.height},
        "depth_size": {"width": depth_pkt.msg.width, "height": depth_pkt.msg.height},
        "pan_tilt_state": {"pan": node.latest_pan_tilt[0], "tilt": node.latest_pan_tilt[1]},
        "depth_age_ms": depth_age_ms,
        "depth_invalid_ratio": invalid_ratio,
        "color_camera_info": (
            {"k": list(color_info.k), "d": list(color_info.d), "distortion_model": color_info.distortion_model}
            if color_info is not None
            else None
        ),
        "depth_camera_info": (
            {"k": list(depth_info.k), "d": list(depth_info.d), "distortion_model": depth_info.distortion_model}
            if depth_info is not None
            else None
        ),
    }
    with open(meta_name, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    node.pairs_saved += 1
    return True


def summarize(node: CaptureDatasetNode, out_dir: Path, elapsed_s: float) -> None:
    rgb_fps = node.rgb_count / elapsed_s if elapsed_s > 0 else 0.0
    depth_fps = node.depth_count / elapsed_s if elapsed_s > 0 else 0.0
    pair_fps = node.pairs_saved / elapsed_s if elapsed_s > 0 else 0.0

    invalid_mean = float(np.mean(node.depth_invalid_ratio_samples)) if node.depth_invalid_ratio_samples else float("nan")
    age_mean = float(np.mean(node.depth_age_ms_samples)) if node.depth_age_ms_samples else float("nan")
    age_max = float(np.max(node.depth_age_ms_samples)) if node.depth_age_ms_samples else float("nan")

    passed = node.pairs_saved > 0 and node.rgb_count > 0 and node.depth_count > 0

    print("\n===== Capture Dataset Summary =====")
    print(f"Output dir: {out_dir}")
    print(f"Elapsed: {elapsed_s:.2f}s")
    print(f"RGB frames received   : {node.rgb_count} ({rgb_fps:.2f} fps)")
    print(f"Depth frames received : {node.depth_count} ({depth_fps:.2f} fps)")
    print(f"Pairs saved           : {node.pairs_saved} ({pair_fps:.2f} fps)")
    print(f"Depth invalid ratio   : mean={invalid_mean:.3f}")
    print(f"RGB-depth age (ms)    : mean={age_mean:.1f}, max={age_max:.1f}")
    print(f"RESULT: {'PASS' if passed else 'FAIL'}")
    print("===================================\n")


def main() -> None:
    # ---------------------- Editable hyperparameters ---------------------- #
    PAN = 0.0
    TILT = 0.0
    SETTLE_TIME_S = 2.0

    NUM_FRAMES = 30       # If > 0, stops when this many pairs are saved
    DURATION_S = 0.0      # If > 0 and NUM_FRAMES <= 0, capture for this duration

    OUTPUT_PATH = "/home/sukeerth/Courses/ME326/Project/collaborative-robotics-2026/ros2_ws/src/vision/output"
    PUTPUT_PATH = OUTPUT_PATH  # Kept as requested for quick manual edits

    SPIN_TIMEOUT_S = 0.02
    SAVE_MIN_PERIOD_S = 0.05   # Avoid writing too fast
    # --------------------------------------------------------------------- #

    if not HAS_CV:
        raise RuntimeError("cv2/cv_bridge not available. Install OpenCV and ROS cv_bridge first.")

    if NUM_FRAMES <= 0 and DURATION_S <= 0:
        raise ValueError("Set NUM_FRAMES > 0 or DURATION_S > 0.")

    rclpy.init()
    node = CaptureDatasetNode()
    out_dir = ensure_output_dir(PUTPUT_PATH)
    last_save_t = 0.0
    start = time.time()

    try:
        # Move pan/tilt to requested fixed pose before capture.
        settle_end = time.time() + SETTLE_TIME_S
        while rclpy.ok() and time.time() < settle_end:
            node.publish_pan_tilt(PAN, TILT)
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)

        capture_start = time.time()
        idx = 0
        while rclpy.ok():
            now = time.time()
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)

            if now - last_save_t < SAVE_MIN_PERIOD_S:
                continue

            if save_pair(node, out_dir, idx):
                idx += 1
                last_save_t = now

            if NUM_FRAMES > 0 and node.pairs_saved >= NUM_FRAMES:
                break
            if NUM_FRAMES <= 0 and DURATION_S > 0 and (now - capture_start) >= DURATION_S:
                break
    finally:
        elapsed = time.time() - start
        summarize(node, out_dir, elapsed)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

