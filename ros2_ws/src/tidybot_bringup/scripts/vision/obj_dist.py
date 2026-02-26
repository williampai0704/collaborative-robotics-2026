#!/usr/bin/env python3
"""
Object Distance Node - Computes average depth of masked object.

Supports two mask sources:
1. Color-based mask (default) - uses HSV thresholding
2. SAM3 mask (optional) - from external segmentation

Debug mode loads images from test_data folder instead of ROS topics.

Usage:
    # Normal mode (from topics)
    ros2 run tidybot_bringup obj_dist.py

    # With live visualization
    ros2 run tidybot_bringup obj_dist.py --ros-args -p visualize:=true

    # Debug mode (from test_data)
    ros2 run tidybot_bringup obj_dist.py --ros-args -p debug:=true -p debug_pair:=0

    # Change target color (r, g, b, y)
    ros2 run tidybot_bringup obj_dist.py --ros-args -p target_color:=b

    # Use SAM3 mask instead of color mask
    ros2 run tidybot_bringup obj_dist.py --ros-args -p use_sam3_mask:=true
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import sys

try:
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False

# Add script directory to path for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from color_mask import color_mask

# Test data path for debug mode
TEST_DATA_PATH = '/home/cdc/collaborative-robotics-2026/test_data'



class ObjectDistanceNode(Node):
    def __init__(self) -> None:
        super().__init__("object_distance_node")

        # Declare parameters
        self.declare_parameter('debug', False)
        self.declare_parameter('debug_pair', 0)
        self.declare_parameter('target_color', 'r')
        self.declare_parameter('use_sam3_mask', False)
        self.declare_parameter('visualize', False)  # Enable live visualization

        # Get parameters
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.debug_pair = self.get_parameter('debug_pair').get_parameter_value().integer_value
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value
        self.use_sam3_mask = self.get_parameter('use_sam3_mask').get_parameter_value().bool_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value

        self.cv_bridge = CvBridge()
        self.distance_pub = self.create_publisher(Float32, "/vision/object_distance", 10)

        self.latest_depth = None
        self.latest_rgb = None
        self.latest_rgb_bgr = None  # For visualization
        self.latest_mask = None  # For SAM3 mask mode

        if self.debug:
            # Debug mode: load from test_data
            self.get_logger().info(f'DEBUG MODE: Loading from test_data/pair_{self.debug_pair:04d}')
            self.get_logger().info(f'Target color: {self.target_color}')
            self.timer = self.create_timer(1.0, self.debug_process)
        else:
            # Normal mode: subscribe to topics
            self.create_subscription(Image, "/camera/depth/image_raw", self._depth_callback, 10)

            if self.use_sam3_mask:
                # SAM3 mask mode
                self.get_logger().info('Using SAM3 mask from /sam3/mask')
                self.create_subscription(Image, "/sam3/mask", self._sam3_mask_callback, 10)
            else:
                # Color mask mode
                self.get_logger().info(f'Using color mask (target: {self.target_color})')
                self.create_subscription(Image, "/camera/color/image_raw", self._rgb_callback, 10)

        if self.visualize:
            self.get_logger().info('Visualization ENABLED - Press "q" to quit')

        self.get_logger().info('object_distance_node started')

    def debug_process(self):
        """Process test data in debug mode with visualization."""
        self.timer.cancel()

        pair_dir = os.path.join(TEST_DATA_PATH, f'pair_{self.debug_pair:04d}')
        rgb_path = os.path.join(pair_dir, 'rgb.png')
        depth_path = os.path.join(pair_dir, 'depth.png')
        depth_npy_path = os.path.join(pair_dir, 'depth.npy')

        if not os.path.exists(rgb_path):
            self.get_logger().error(f'RGB image not found: {rgb_path}')
            return

        # Load RGB image
        rgb_bgr = cv2.imread(rgb_path)
        rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
        self.get_logger().info(f'Loaded RGB image: {rgb.shape}')

        # Load depth (prefer .npy if available)
        if os.path.exists(depth_npy_path):
            depth = np.load(depth_npy_path)
            self.get_logger().info(f'Loaded depth from npy: {depth.shape}, dtype: {depth.dtype}')
        elif os.path.exists(depth_path):
            depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            self.get_logger().info(f'Loaded depth from png: {depth.shape}, dtype: {depth.dtype}')
        else:
            self.get_logger().error(f'Depth image not found in {pair_dir}')
            return

        # Generate color mask
        mask = color_mask(rgb, self.target_color)
        self.get_logger().info(f'Generated color mask, non-zero pixels: {np.count_nonzero(mask)}')

        # Compute distance and get stats
        distance_m, stats = self.compute_distance(depth, mask, rgb_bgr=rgb_bgr, return_stats=True)

        # Visualize results (blocking for debug mode)
        self.visualize_frame(rgb_bgr, depth, mask, distance_m, stats, blocking=True)

    def visualize_frame(self, rgb_bgr, depth, mask, distance_m, stats, blocking=False):
        """Visualize results with OpenCV windows.
        
        Args:
            rgb_bgr: Original RGB image in BGR format
            depth: Depth image
            mask: Binary mask
            distance_m: Computed distance in meters (None if failed)
            stats: Dictionary with statistics
            blocking: If True, wait for key press. If False, use waitKey(1) for streaming.
        """
        h, w = mask.shape[:2]
        
        # 1. Create depth visualization (colormap)
        depth_valid = depth.copy().astype(np.float32)
        depth_valid[depth_valid == 0] = np.nan
        vmin = np.nanpercentile(depth_valid, 5) if np.any(~np.isnan(depth_valid)) else 0
        vmax = np.nanpercentile(depth_valid, 95) if np.any(~np.isnan(depth_valid)) else 1
        depth_normalized = np.clip((depth.astype(np.float32) - vmin) / (vmax - vmin + 1e-6), 0, 1)
        depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)
        depth_colored[depth == 0] = 0  # Black for invalid depth
        
        # 2. Get color tint
        color_tints = {
            'red': (0, 0, 255), 'r': (0, 0, 255),
            'green': (0, 255, 0), 'g': (0, 255, 0),
            'blue': (255, 0, 0), 'b': (255, 0, 0),
            'yellow': (0, 255, 255), 'y': (0, 255, 255)
        }
        tint = color_tints.get(self.target_color.lower(), (255, 255, 255))
        
        # 3. Overlay: RGB with mask
        mask_colored = np.zeros_like(rgb_bgr)
        mask_colored[mask > 0] = tint
        overlay = cv2.addWeighted(rgb_bgr.copy(), 0.7, mask_colored, 0.3, 0)
        
        # Draw mask contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, tint, 2)
        
        # Add distance text
        if distance_m is not None:
            text = f'Dist: {distance_m:.3f}m'
            cv2.putText(overlay, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(overlay, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)
        else:
            cv2.putText(overlay, 'No depth', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Add stats
        stats_text = f'Pixels: {stats.get("valid_pixels", 0)}'
        cv2.putText(overlay, stats_text, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 4. Masked depth
        masked_depth = depth_colored.copy()
        masked_depth[mask == 0] = 0
        
        # 5. Create combined view: RGB | Depth | Mask | Overlay
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([rgb_bgr, depth_colored, mask_bgr, overlay])
        
        # Add labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(combined, 'RGB', (10, 20), font, 0.5, (255, 255, 255), 1)
        cv2.putText(combined, 'Depth', (w + 10, 20), font, 0.5, (255, 255, 255), 1)
        cv2.putText(combined, 'Mask', (2*w + 10, 20), font, 0.5, (255, 255, 255), 1)
        cv2.putText(combined, 'Overlay', (3*w + 10, 20), font, 0.5, (255, 255, 255), 1)
        
        # Resize if too large
        max_width = 1920
        if combined.shape[1] > max_width:
            scale = max_width / combined.shape[1]
            combined = cv2.resize(combined, None, fx=scale, fy=scale)

        cv2.imshow('Object Distance: RGB | Depth | Mask | Overlay', combined)
        
        if blocking:
            self.get_logger().info('Visualization displayed. Press any key to close.')
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit key pressed, disabling visualization')
                self.visualize = False
                cv2.destroyAllWindows()

    def _depth_callback(self, msg: Image) -> None:
        if msg.encoding == "16UC1":
            depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        elif msg.encoding == "32FC1":
            depth_f = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            depth = np.nan_to_num(depth_f, nan=0.0) * 1000.0
            depth = depth.astype(np.uint16)
        else:
            self.get_logger().warn(f"Unknown depth encoding: {msg.encoding}")
            return

        self.latest_depth = depth
        self.try_compute_distance()

    def _rgb_callback(self, msg: Image) -> None:
        """Handle RGB image for color mask mode."""
        try:
            rgb = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self.latest_rgb = rgb
            self.latest_rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            self.try_compute_distance()
        except Exception as e:
            self.get_logger().warn(f'Failed to convert RGB image: {e}')

    def _sam3_mask_callback(self, msg: Image) -> None:
        """Handle SAM3 segmentation mask (kept for future use)."""
        try:
            mask = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            self.latest_mask = mask
            self.try_compute_distance()
        except Exception as e:
            self.get_logger().warn(f'Failed to convert mask: {e}')

    def try_compute_distance(self) -> None:
        if self.latest_depth is None:
            return

        if self.use_sam3_mask:
            if self.latest_mask is None:
                return
            mask = self.latest_mask
        else:
            if self.latest_rgb is None:
                return
            mask = color_mask(self.latest_rgb, self.target_color)

        self.compute_distance(self.latest_depth, mask, rgb_bgr=self.latest_rgb_bgr)

    def compute_distance(self, depth, mask, rgb_bgr=None, return_stats=False):
        """Compute and publish average distance of masked pixels.
        
        Args:
            depth: Depth image (uint16 in mm or float32 in m)
            mask: Binary mask
            rgb_bgr: Optional BGR image for visualization
            return_stats: If True, return (distance, stats_dict)
            
        Returns:
            If return_stats: (distance_m, stats_dict) or (None, stats_dict) if failed
            Otherwise: None
        """
        stats = {
            'mask_pixels': np.count_nonzero(mask),
            'valid_pixels': 0,
            'min_depth': 0,
            'max_depth': 0,
            'std_depth': 0,
        }
        
        distance_m = None
        
        if depth.shape[:2] != mask.shape[:2]:
            self.get_logger().warn(
                f"Shape mismatch. Depth: {depth.shape}, Mask: {mask.shape}")
        else:
            object_pixels = depth[mask > 0]
            object_pixels = object_pixels[object_pixels > 0]
            stats['valid_pixels'] = len(object_pixels)

            if object_pixels.size == 0:
                self.get_logger().info("Object not in image (no valid depth pixels).")
            else:
                avg_depth_mm = np.mean(object_pixels)
                distance_m = avg_depth_mm / 1000.0
                
                stats['min_depth'] = np.min(object_pixels) / 1000.0
                stats['max_depth'] = np.max(object_pixels) / 1000.0
                stats['std_depth'] = np.std(object_pixels) / 1000.0

                msg = Float32()
                msg.data = float(distance_m)
                self.distance_pub.publish(msg)

                self.get_logger().info(f"Object distance: {distance_m:.3f} m ({len(object_pixels)} pixels)")
        
        # Live visualization (non-blocking)
        if self.visualize and rgb_bgr is not None:
            self.visualize_frame(rgb_bgr, depth, mask, distance_m, stats, blocking=False)
        
        if return_stats:
            return distance_m, stats


def main() -> None:
    if not HAS_CV:
        raise RuntimeError("cv2 not available. Install OpenCV first.")

    rclpy.init()
    node = ObjectDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
