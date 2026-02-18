#!/usr/bin/env python3
"""
Finite pan-tilt sweep test with RGB-D capture per pose.

For each commanded pose:
- Send /camera/pan_tilt_cmd
- Wait for settle time
- Read /camera/pan_tilt_state
- Save RGB + depth + metadata
- Compute command-vs-state error

No infinite loops. Exits with PASS/FAIL and stats.
"""

import json
import math
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

try:
    import cv2
    from cv_bridge import CvBridge

    HAS_CV = True
except ImportError:
    HAS_CV = False


class PanTiltSweepNode(Node):
    def __init__(self) -> None:
        super().__init__("pan_tilt_sweep_test")
        self.cv_bridge = CvBridge()
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, "/camera/pan_tilt_cmd", 10)

        self.latest_rgb: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.latest_pan_tilt: list[float] = [float("nan"), float("nan")]

        self.rgb_count = 0
        self.depth_count = 0
        self.state_count = 0

        self.create_subscription(Image, "/camera/color/image_raw", self._on_rgb, 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, 10)
        self.create_subscription(JointState, "/camera/pan_tilt_state", self._on_pan_tilt_state, 10)

    def _on_rgb(self, msg: Image) -> None:
        self.rgb_count += 1
        self.latest_rgb = msg

    def _on_depth(self, msg: Image) -> None:
        self.depth_count += 1
        self.latest_depth = msg

    def _on_pan_tilt_state(self, msg: JointState) -> None:
        self.state_count += 1
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
    out = Path(base_path).expanduser().resolve() / f"sweep_{ts}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "rgb").mkdir(exist_ok=True)
    (out / "depth_raw").mkdir(exist_ok=True)
    (out / "depth_viz").mkdir(exist_ok=True)
    (out / "meta").mkdir(exist_ok=True)
    return out


def depth_to_u16_mm(node: PanTiltSweepNode, depth_msg: Image) -> np.ndarray:
    if depth_msg.encoding == "16UC1":
        return node.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
    if depth_msg.encoding == "32FC1":
        depth_f = node.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        depth_mm = np.nan_to_num(depth_f, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
        return np.clip(depth_mm, 0, 65535).astype(np.uint16)
    depth_any = node.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    if depth_any.dtype != np.uint16:
        depth_any = np.clip(depth_any, 0, 65535).astype(np.uint16)
    return depth_any


def save_pose_capture(
    node: PanTiltSweepNode,
    out_dir: Path,
    pose_idx: int,
    cmd_pan: float,
    cmd_tilt: float,
    settle_s: float,
) -> dict:
    if node.latest_rgb is None or node.latest_depth is None:
        return {
            "pose_idx": pose_idx,
            "cmd_pan": cmd_pan,
            "cmd_tilt": cmd_tilt,
            "success": False,
            "reason": "missing_rgb_or_depth",
        }

    rgb = node.cv_bridge.imgmsg_to_cv2(node.latest_rgb, desired_encoding="bgr8")
    depth_u16 = depth_to_u16_mm(node, node.latest_depth)
    depth_norm = cv2.normalize(depth_u16, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    depth_viz = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)

    rgb_path = out_dir / "rgb" / f"pose_{pose_idx:03d}.png"
    depth_raw_path = out_dir / "depth_raw" / f"pose_{pose_idx:03d}.png"
    depth_viz_path = out_dir / "depth_viz" / f"pose_{pose_idx:03d}.png"
    meta_path = out_dir / "meta" / f"pose_{pose_idx:03d}.json"

    cv2.imwrite(str(rgb_path), rgb)
    cv2.imwrite(str(depth_raw_path), depth_u16)
    cv2.imwrite(str(depth_viz_path), depth_viz)

    st_pan, st_tilt = node.latest_pan_tilt
    pan_err = abs(cmd_pan - st_pan) if not math.isnan(st_pan) else float("nan")
    tilt_err = abs(cmd_tilt - st_tilt) if not math.isnan(st_tilt) else float("nan")

    meta = {
        "pose_idx": pose_idx,
        "wall_time_s": time.time(),
        "command": {"pan": cmd_pan, "tilt": cmd_tilt},
        "state": {"pan": st_pan, "tilt": st_tilt},
        "abs_error": {"pan": pan_err, "tilt": tilt_err},
        "settle_time_s": settle_s,
        "rgb_frame": {
            "stamp": {"sec": node.latest_rgb.header.stamp.sec, "nanosec": node.latest_rgb.header.stamp.nanosec},
            "frame_id": node.latest_rgb.header.frame_id,
            "encoding": node.latest_rgb.encoding,
            "width": node.latest_rgb.width,
            "height": node.latest_rgb.height,
        },
        "depth_frame": {
            "stamp": {"sec": node.latest_depth.header.stamp.sec, "nanosec": node.latest_depth.header.stamp.nanosec},
            "frame_id": node.latest_depth.header.frame_id,
            "encoding": node.latest_depth.encoding,
            "width": node.latest_depth.width,
            "height": node.latest_depth.height,
        },
        "success": True,
    }

    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    return meta


def main() -> None:
    # ---------------------- Editable hyperparameters ---------------------- #
    CENTER_PAN = 0.0
    CENTER_TILT = 0.0
    PAN_DELTA = 0.5
    TILT_DELTA = 0.35

    # Pose sequence for sweep. Edit as needed.
    SWEEP_POSES = [
        (CENTER_PAN, CENTER_TILT),
        (CENTER_PAN - PAN_DELTA, CENTER_TILT),
        (CENTER_PAN, CENTER_TILT),
        (CENTER_PAN + PAN_DELTA, CENTER_TILT),
        (CENTER_PAN, CENTER_TILT),
        (CENTER_PAN, CENTER_TILT - TILT_DELTA),
        (CENTER_PAN, CENTER_TILT),
        (CENTER_PAN, CENTER_TILT + TILT_DELTA),
        (CENTER_PAN, CENTER_TILT),
    ]

    SETTLE_TIME_S = 1.5
    SPIN_TIMEOUT_S = 0.02
    POSE_ERROR_TOL_RAD = 0.20

    OUTPUT_PATH = "/home/sukeerth/Courses/ME326/Project/collaborative-robotics-2026/ros2_ws/src/vision/output"
    PUTPUT_PATH = OUTPUT_PATH  # Kept as requested for quick manual edits
    # --------------------------------------------------------------------- #

    if not HAS_CV:
        raise RuntimeError("cv2/cv_bridge not available. Install OpenCV and ROS cv_bridge first.")

    rclpy.init()
    node = PanTiltSweepNode()
    out_dir = ensure_output_dir(PUTPUT_PATH)
    pose_results: list[dict] = []
    start_t = time.time()

    try:
        # Warm-up a bit to receive first frames/state.
        warmup_end = time.time() + 2.0
        while rclpy.ok() and time.time() < warmup_end:
            node.publish_pan_tilt(CENTER_PAN, CENTER_TILT)
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)

        for i, (pan_cmd, tilt_cmd) in enumerate(SWEEP_POSES):
            settle_end = time.time() + SETTLE_TIME_S
            while rclpy.ok() and time.time() < settle_end:
                node.publish_pan_tilt(pan_cmd, tilt_cmd)
                rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)

            result = save_pose_capture(
                node=node,
                out_dir=out_dir,
                pose_idx=i,
                cmd_pan=pan_cmd,
                cmd_tilt=tilt_cmd,
                settle_s=SETTLE_TIME_S,
            )
            pose_results.append(result)
    finally:
        elapsed = time.time() - start_t
        valid_results = [r for r in pose_results if r.get("success", False)]
        fail_count = len(pose_results) - len(valid_results)
        pan_errs = [r["abs_error"]["pan"] for r in valid_results if not math.isnan(r["abs_error"]["pan"])]
        tilt_errs = [r["abs_error"]["tilt"] for r in valid_results if not math.isnan(r["abs_error"]["tilt"])]

        pan_mean = float(np.mean(pan_errs)) if pan_errs else float("nan")
        pan_max = float(np.max(pan_errs)) if pan_errs else float("nan")
        tilt_mean = float(np.mean(tilt_errs)) if tilt_errs else float("nan")
        tilt_max = float(np.max(tilt_errs)) if tilt_errs else float("nan")

        error_ok = (
            bool(pan_errs)
            and bool(tilt_errs)
            and pan_max <= POSE_ERROR_TOL_RAD
            and tilt_max <= POSE_ERROR_TOL_RAD
        )
        stream_ok = node.rgb_count > 0 and node.depth_count > 0 and node.state_count > 0
        pass_ok = fail_count == 0 and error_ok and stream_ok

        print("\n===== Pan-Tilt Sweep Summary =====")
        print(f"Output dir: {out_dir}")
        print(f"Elapsed: {elapsed:.2f}s")
        print(f"Poses attempted: {len(pose_results)}")
        print(f"Poses successful: {len(valid_results)}")
        print(f"Poses failed: {fail_count}")
        print(f"RGB frames seen: {node.rgb_count}, Depth frames seen: {node.depth_count}, State msgs: {node.state_count}")
        print(f"Pan error  : mean={pan_mean:.3f} rad, max={pan_max:.3f} rad")
        print(f"Tilt error : mean={tilt_mean:.3f} rad, max={tilt_max:.3f} rad")
        print(f"Error tolerance: {POSE_ERROR_TOL_RAD:.3f} rad")
        print(f"RESULT: {'PASS' if pass_ok else 'FAIL'}")
        print("==================================\n")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

