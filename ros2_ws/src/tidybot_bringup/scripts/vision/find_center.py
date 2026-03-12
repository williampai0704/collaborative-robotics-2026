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

    # With live visualization
    ros2 run tidybot_bringup find_center.py --ros-args -p visualize:=true

    # Debug mode (from test_data)
    ros2 run tidybot_bringup find_center.py --ros-args -p debug:=true
    ros2 run tidybot_bringup find_center.py --ros-args -p debug:=true -p debug_dataset:=yellow_block_1 -p debug_frame:=0

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
        self.declare_parameter('debug_dataset', 'yellow_block_1')  # Dataset folder name
        self.declare_parameter('debug_frame', 0)  # Frame index within dataset
        self.declare_parameter('target_color', 'yellow')  # Use full name to avoid YAML 'y'=true issue
        self.declare_parameter('use_sam3_mask', False)  # Set True to use SAM3 mask instead
        self.declare_parameter('visualize', False)  # Enable live visualization

        # Get parameters
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.debug_dataset = self.get_parameter('debug_dataset').get_parameter_value().string_value
        self.debug_frame = self.get_parameter('debug_frame').get_parameter_value().integer_value
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value
        self.use_sam3_mask = self.get_parameter('use_sam3_mask').get_parameter_value().bool_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value

        self.bridge = CvBridge()

        # Store latest RGB for visualization
        self.latest_rgb_bgr = None

        # Publisher for mask center
        self.center_pub = self.create_publisher(Point, '/mask_center', 10)

        if self.debug:
            # Debug mode: load from test_data
            self.get_logger().info(f'DEBUG MODE: Loading from test_data/{self.debug_dataset}/rgb/rgb_{self.debug_frame:04d}.png')
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

        if self.visualize:
            self.get_logger().info('Visualization ENABLED - Press "q" to quit')

        self.get_logger().info('find_center node started')

    def debug_process(self):
        """Process test data in debug mode with visualization."""
        self.timer.cancel()  # Only run once

        dataset_dir = os.path.join(TEST_DATA_PATH, self.debug_dataset)
        rgb_path = os.path.join(dataset_dir, 'rgb', f'rgb_{self.debug_frame:04d}.png')

        if not os.path.exists(rgb_path):
            self.get_logger().error(f'RGB image not found: {rgb_path}')
            # List available datasets
            if os.path.exists(TEST_DATA_PATH):
                datasets = [d for d in os.listdir(TEST_DATA_PATH) 
                           if os.path.isdir(os.path.join(TEST_DATA_PATH, d))]
                self.get_logger().info(f'Available datasets: {datasets}')
            return

        # Load RGB image
        rgb = cv2.imread(rgb_path)
        if rgb is None:
            self.get_logger().error(f'Failed to load RGB image: {rgb_path}')
            return
            
        rgb_display = rgb.copy()  # Keep BGR for display
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        self.get_logger().info(f'Loaded RGB image: {rgb.shape}')

        # Generate color mask
        mask = color_mask(rgb, self.target_color)
        self.get_logger().info(f'Generated color mask, non-zero pixels: {np.count_nonzero(mask)}')

        # Find center and get coordinates
        cx, cy = self.process_mask(mask, rgb_bgr=rgb_display, return_center=True)

        # Visualize results (blocking for debug mode)
        self.visualize_frame(rgb_display, mask, cx, cy, blocking=True)

    def rgb_cb(self, msg):
        """Handle RGB image and generate color mask."""
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.latest_rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f'Failed to convert RGB image: {e}')
            return

        # Generate color mask
        mask = color_mask(rgb, self.target_color)
        self.process_mask(mask, rgb_bgr=self.latest_rgb_bgr)

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

        self.process_mask(mask, rgb_bgr=self.latest_rgb_bgr)

    def process_mask(self, mask, rgb_bgr=None, return_center=False):
        """Calculate and publish mask centroid.
        
        Args:
            mask: Binary mask image
            rgb_bgr: Optional BGR image for visualization
            return_center: If True, return (cx, cy) tuple
            
        Returns:
            If return_center is True, returns (cx, cy) or (None, None) if no mask pixels
        """
        # Find all non-zero pixel coordinates
        ys, xs = np.where(mask > 0)

        cx, cy = None, None
        if len(xs) == 0:
            self.get_logger().debug('No mask pixels found.')
        else:
            # Calculate centroid
            cx = np.mean(xs)
            cy = np.mean(ys)

            out = Point()
            out.x = float(cx)
            out.y = float(cy)
            out.z = 0.0

            self.center_pub.publish(out)
            self.get_logger().info(f'Mask center: ({out.x:.2f}, {out.y:.2f}), pixels: {len(xs)}')

        # Live visualization (non-blocking)
        if self.visualize and rgb_bgr is not None:
            self.visualize_frame(rgb_bgr, mask, cx, cy, blocking=False)

        if return_center:
            return cx, cy

    def visualize_frame(self, rgb_bgr, mask, cx, cy, blocking=False):
        """Visualize results with OpenCV windows.
        
        Args:
            rgb_bgr: Original RGB image in BGR format for OpenCV display
            mask: Binary mask image
            cx, cy: Center coordinates (can be None if no mask found)
            blocking: If True, wait for key press. If False, use waitKey(1) for streaming.
        """
        h, w = mask.shape[:2]
        
        # Get color tint
        color_tints = {
            'red':    (0,   0,   255), 'r': (0,   0,   255),
            'green':  (0,   255, 0),   'g': (0,   255, 0),
            'blue':   (255, 0,   0),   'b': (255, 0,   0),
            'yellow': (0,   255, 255), 'y': (0,   255, 255),
            'orange': (0,   165, 255), 'o': (0,   165, 255),
            'purple': (128, 0,   128), 'p': (128, 0,   128),
        }
        tint = color_tints.get(self.target_color.lower(), (255, 255, 255))
        
        # Create overlay with mask
        mask_tinted = np.zeros_like(rgb_bgr)
        mask_tinted[mask > 0] = tint
        overlay = cv2.addWeighted(rgb_bgr.copy(), 0.7, mask_tinted, 0.3, 0)
        
        # Draw center marker
        if cx is not None and cy is not None:
            cx_int, cy_int = int(cx), int(cy)
            cv2.drawMarker(overlay, (cx_int, cy_int), (255, 255, 255),
                          cv2.MARKER_CROSS, markerSize=30, thickness=2)
            cv2.circle(overlay, (cx_int, cy_int), 10, (255, 255, 255), 2)
            text = f'Center: ({cx:.1f}, {cy:.1f})'
            cv2.putText(overlay, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(overlay, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)
        else:
            cv2.putText(overlay, 'No mask detected', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Add info text
        info_text = f'Color: {self.target_color} | Pixels: {np.count_nonzero(mask)}'
        cv2.putText(overlay, info_text, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Create side-by-side view: RGB | Mask | Overlay
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([rgb_bgr, mask_colored, overlay])
        
        # Resize if too large
        max_width = 1800
        if combined.shape[1] > max_width:
            scale = max_width / combined.shape[1]
            combined = cv2.resize(combined, None, fx=scale, fy=scale)

        cv2.imshow('Find Center: RGB | Mask | Overlay', combined)
        
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


def main():
    rclpy.init()
    node = FindCenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
