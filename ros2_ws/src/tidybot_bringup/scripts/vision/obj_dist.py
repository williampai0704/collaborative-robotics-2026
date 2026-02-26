#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

try:
    import cv2
    from cv_bridge import CvBridge

    HAS_CV = True
except ImportError:
    HAS_CV = False

class ObjectDistanceNode(Node):
    def __init__(self) -> None:
        super().__init__("object_distance_node")
        self.cv_bridge = CvBridge()
        self.distance_pub = self.create_publisher(Float32, "/vision/object_distance", 10)

        self.latest_depth = None
        self.latest_mask = None
        
        self.create_subscription(Image, "/camera/depth/image_raw", self._depth_callback, 10)
        self.create_subscription(Image, "/sam3/mask", self._mask_callback, 10)

    def _depth_callback(self, msg: Image) -> None:
        if msg.encoding == "16UC1":
            depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        elif msg.encoding == "32FC1":
            depth_f = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            depth = np.nan_to_num(depth_f, nan=0.0) * 1000.0
            depth = depth.astype(np.uint16)
        else:
            print("Warning!!! No depth encoding.")
            return

        self.latest_depth = depth
        self.try_compute_distance()

    def _mask_callback(self, msg: Image) -> None:
        mask = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.latest_mask = mask
        self.try_compute_distance()

    def try_compute_distance(self) -> None:
        if self.latest_depth is None or self.latest_mask is None:
            return

        depth = self.latest_depth
        mask = self.latest_mask

        if depth.shape != mask.shape:
            print(f"Warning!!! Shape mismatch. Depth shape: {depth.shape}, Mask shape: {mask.shape}")
            return

        object_pixels = depth[mask > 0]
        object_pixels = object_pixels[object_pixels > 0]
        if object_pixels.size == 0:
            print(f"Object not in image.")
            return

        avg_depth_mm = np.mean(object_pixels)
        avg_depth_m = avg_depth_mm / 1000.0

        msg = Float32()
        msg.data = float(avg_depth_m)
        self.distance_pub.publish(msg)

        print(f"Object distance: {avg_depth_m:.3f} m.")

def main() -> None:
    if not HAS_CV:
        raise RuntimeError("cv2/cv_bridge not available. Install OpenCV and ROS cv_bridge first.")

    rclpy.init()
    node = ObjectDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
