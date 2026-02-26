#!/usr/bin/env python3
"""
ROS publisher code for finding the center coord from a segmentation mask.

Supports two mask sources:
1. Color-based mask (default) - uses HSV thresholding
2. SAM3 mask (commented out) - from external segmentation

Debug mode loads images from test_data folder instead of ROS topics.

Usage:
    # Normal mode (from topics)
    ros2 run tidybot_bringup find_center.py

    # Debug mode (from test_data)
    ros2 run tidybot_bringup find_center.py --ros-args -p debug:=true -p debug_pair:=0

    # Change target color (r, g, b, y)
    ros2 run tidybot_bringup find_center.py --ros-args -p target_color:=yellow
"""

import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Add vision scripts directory to path for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from color_mask import color_mask

# Test data path for debug mode
TEST_DATA_PATH = '/home/cdc/collaborative-robotics-2026/test_data'


class FindCenter(Node):
    def __init__(self):
        super().__init__('find_center')

        # Declare parameters
        self.declare_parameter('debug', False)
        self.declare_parameter('debug_pair', 0)
        self.declare_parameter('target_color', 'red')  # Use full name to avoid YAML 'y'=true issue
        self.declare_parameter('use_sam3_mask', False)  # Set True to use SAM3 mask instead

        # Get parameters
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.debug_pair = self.get_parameter('debug_pair').get_parameter_value().integer_value
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value
        self.use_sam3_mask = self.get_parameter('use_sam3_mask').get_parameter_value().bool_value

        self.bridge = CvBridge()

        # Publisher for mask center
        self.center_pub = self.create_publisher(Point, '/mask_center', 10)

        if self.debug:
            # Debug mode: load from test_data
            self.get_logger().info(f'DEBUG MODE: Loading from test_data/pair_{self.debug_pair:04d}')
            self.get_logger().info(f'Target color: {self.target_color}')
            # Timer to process once after startup
            self.timer = self.create_timer(1.0, self.debug_process)
        else:
            # Normal mode: subscribe to topics
            if self.use_sam3_mask:
                # SAM3 mask mode
                self.get_logger().info('Using SAM3 mask from /sam3/mask')
                self.sub = self.create_subscription(Image, '/sam3/mask', self.sam3_mask_cb, 10)
            else:
                # Color mask mode: need RGB image
                self.get_logger().info(f'Using color mask (target: {self.target_color})')
                self.sub = self.create_subscription(
                    Image, '/camera/color/image_raw', self.rgb_cb, 10)

        self.get_logger().info('find_center node started')

    def debug_process(self):
        """Process test data in debug mode with visualization."""
        self.timer.cancel()  # Only run once

        pair_dir = os.path.join(TEST_DATA_PATH, f'pair_{self.debug_pair:04d}')
        rgb_path = os.path.join(pair_dir, 'rgb.png')

        if not os.path.exists(rgb_path):
            self.get_logger().error(f'RGB image not found: {rgb_path}')
            return

        # Load RGB image
        rgb = cv2.imread(rgb_path)
        rgb_display = rgb.copy()  # Keep BGR for display
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        self.get_logger().info(f'Loaded RGB image: {rgb.shape}')

        # Generate color mask
        mask = color_mask(rgb, self.target_color)
        self.get_logger().info(f'Generated color mask, non-zero pixels: {np.count_nonzero(mask)}')

        # Find center and get coordinates
        cx, cy = self.process_mask(mask, return_center=True)

        # Visualize results
        self.visualize_debug(rgb_display, mask, cx, cy)

    def rgb_cb(self, msg):
        """Handle RGB image and generate color mask."""
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert RGB image: {e}')
            return

        # Generate color mask
        mask = color_mask(rgb, self.target_color)
        self.process_mask(mask)

    def sam3_mask_cb(self, msg):
        """Handle SAM3 segmentation mask (kept for future use)."""
        try:
            # Handle different encodings
            if msg.encoding in ['mono8', '8UC1']:
                mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            elif msg.encoding in ['mono16', '16UC1']:
                mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            elif msg.encoding in ['32FC1']:
                mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                mask = (mask > 0).astype(np.uint8) * 255
            else:
                mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if len(mask.shape) == 3:
                    mask = np.any(mask > 0, axis=2).astype(np.uint8) * 255
        except Exception as e:
            self.get_logger().error(f'Failed to convert mask: {e}')
            return

        self.process_mask(mask)

    def process_mask(self, mask, return_center=False):
        """Calculate and publish mask centroid.
        
        Args:
            mask: Binary mask image
            return_center: If True, return (cx, cy) tuple
            
        Returns:
            If return_center is True, returns (cx, cy) or (None, None) if no mask pixels
        """
        # Find all non-zero pixel coordinates
        ys, xs = np.where(mask > 0)

        if len(xs) == 0:
            self.get_logger().debug('No mask pixels found.')
            if return_center:
                return None, None
            return

        # Calculate centroid
        cx = np.mean(xs)
        cy = np.mean(ys)

        out = Point()
        out.x = float(cx)
        out.y = float(cy)
        out.z = 0.0

        self.center_pub.publish(out)
        self.get_logger().info(f'Mask center: ({out.x:.2f}, {out.y:.2f}), pixels: {len(xs)}')

        if return_center:
            return cx, cy

    def visualize_debug(self, rgb_bgr, mask, cx, cy):
        """Visualize debug results with OpenCV windows.
        
        Args:
            rgb_bgr: Original RGB image in BGR format for OpenCV display
            mask: Binary mask image
            cx, cy: Center coordinates (can be None if no mask found)
        """
        # Create visualization images
        h, w = mask.shape[:2]
        
        # 1. Original image with center marker
        img_with_center = rgb_bgr.copy()
        if cx is not None and cy is not None:
            cx_int, cy_int = int(cx), int(cy)
            # Draw crosshair at center
            cv2.drawMarker(img_with_center, (cx_int, cy_int), (0, 255, 0), 
                          cv2.MARKER_CROSS, markerSize=30, thickness=2)
            # Draw circle at center
            cv2.circle(img_with_center, (cx_int, cy_int), 10, (0, 255, 0), 2)
            # Add text with coordinates
            text = f'Center: ({cx:.1f}, {cy:.1f})'
            cv2.putText(img_with_center, text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(img_with_center, 'No mask detected', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 2. Mask visualization (colorized)
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # Tint the mask with the target color
        color_tints = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255)
        }
        # Normalize target_color
        tc = self.target_color.lower()
        if tc in ['r', 'red']:
            tc = 'red'
        elif tc in ['g', 'green']:
            tc = 'green'
        elif tc in ['b', 'blue']:
            tc = 'blue'
        elif tc in ['y', 'yellow']:
            tc = 'yellow'
        tint = color_tints.get(tc, (255, 255, 255))
        mask_tinted = np.zeros_like(rgb_bgr)
        mask_tinted[mask > 0] = tint
        
        # 3. Overlay: original with mask overlay
        overlay = rgb_bgr.copy()
        overlay = cv2.addWeighted(overlay, 0.7, mask_tinted, 0.3, 0)
        if cx is not None and cy is not None:
            cx_int, cy_int = int(cx), int(cy)
            cv2.drawMarker(overlay, (cx_int, cy_int), (255, 255, 255),
                          cv2.MARKER_CROSS, markerSize=30, thickness=2)
            cv2.circle(overlay, (cx_int, cy_int), 10, (255, 255, 255), 2)

        # 4. Create combined visualization (2x2 grid)
        # Top row: original, mask
        # Bottom row: overlay, info panel
        top_row = np.hstack([rgb_bgr, mask_colored])
        
        # Info panel
        info_panel = np.zeros_like(rgb_bgr)
        info_lines = [
            f'Target Color: {self.target_color}',
            f'Image Size: {w} x {h}',
            f'Mask Pixels: {np.count_nonzero(mask)}',
        ]
        if cx is not None:
            info_lines.extend([
                f'Center X: {cx:.2f}',
                f'Center Y: {cy:.2f}',
            ])
        else:
            info_lines.append('Center: NOT FOUND')
        
        for i, line in enumerate(info_lines):
            cv2.putText(info_panel, line, (10, 30 + i * 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        bottom_row = np.hstack([overlay, info_panel])
        combined = np.vstack([top_row, bottom_row])

        # Show windows
        cv2.imshow('Debug: Combined View', combined)
        cv2.imshow('Debug: Original + Center', img_with_center)
        cv2.imshow('Debug: Mask', mask)
        cv2.imshow('Debug: Overlay', overlay)

        self.get_logger().info('Debug visualization displayed. Press any key to close.')
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = FindCenter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
