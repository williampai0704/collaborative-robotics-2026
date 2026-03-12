#!/usr/bin/env python3
"""
Visualize RGB Point Cloud using Open3D.

Supports two modes:
1. Standalone mode: Loads RGB and depth images from test_data folder
2. ROS2 node mode: Subscribes to camera topics and streams live point cloud

Handles unaligned RGB-D data by projecting depth points to RGB for color lookup.

Usage (Standalone):
    python3 visualize_pc.py                     # Default: pair_0000
    python3 visualize_pc.py --pair 1            # Use pair_0001
    python3 visualize_pc.py --pair 0 --mask red # Show only red-masked points

Usage (ROS2 node):
    ros2 run tidybot_bringup visualize_pc.py

    # With color mask
    ros2 run tidybot_bringup visualize_pc.py --ros-args -p target_color:=red

    # With live visualization (OpenCV window)
    ros2 run tidybot_bringup visualize_pc.py --ros-args -p visualize:=true

Controls in Open3D viewer:
    - Left mouse: Rotate
    - Right mouse: Pan
    - Scroll: Zoom
    - Q: Quit
"""

import argparse
import os
import sys
import numpy as np
import cv2
import threading

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("WARNING: Open3D not installed. Install with: pip install open3d")

# Add script directory to path for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from color_mask import color_mask

# Test data path
TEST_DATA_PATH = '/home/cdc/collaborative-robotics-2026/test_data'

# Default camera intrinsics (RealSense D435 at 640x480)
DEFAULT_DEPTH_FX = 384.0
DEFAULT_DEPTH_FY = 384.0
DEFAULT_DEPTH_CX = 320.0
DEFAULT_DEPTH_CY = 240.0

DEFAULT_RGB_FX = 615.0
DEFAULT_RGB_FY = 615.0
DEFAULT_RGB_CX = 320.0
DEFAULT_RGB_CY = 240.0

# Depth-to-RGB translation (RealSense D435: ~15mm offset)
DEPTH_TO_RGB_TX = 0.015  # meters


def load_camera_intrinsics(meta_path):
    """Load RGB camera intrinsics from meta.mat file if available."""
    try:
        import scipy.io
        meta = scipy.io.loadmat(meta_path)
        if 'K' in meta:
            K = meta['K']
            return K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        elif 'intrinsic_matrix' in meta:
            K = meta['intrinsic_matrix']
            return K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    except Exception as e:
        print(f"Could not load intrinsics from meta.mat: {e}")
    
    return DEFAULT_RGB_FX, DEFAULT_RGB_FY, DEFAULT_RGB_CX, DEFAULT_RGB_CY


def depth_to_pointcloud(rgb, depth, 
                        depth_fx, depth_fy, depth_cx, depth_cy,
                        rgb_fx, rgb_fy, rgb_cx, rgb_cy,
                        mask=None, depth_scale=1000.0):
    """Convert unaligned RGB-D images to colored point cloud.
    
    Creates geometry from depth camera frame and projects each 3D point
    to RGB image to get its color.
    
    Args:
        rgb: RGB image (H, W, 3) uint8
        depth: Depth image (H, W) uint16 or float
        depth_fx, depth_fy, depth_cx, depth_cy: Depth camera intrinsics
        rgb_fx, rgb_fy, rgb_cx, rgb_cy: RGB camera intrinsics
        mask: Optional binary mask (in RGB frame) to filter points
        depth_scale: Scale factor (1000 for mm to m conversion)
    
    Returns:
        Open3D PointCloud with colors
    """
    h, w = depth.shape
    rgb_h, rgb_w = rgb.shape[:2]
    
    # Convert depth to meters
    if depth.dtype == np.uint16:
        depth_m = depth.astype(np.float32) / depth_scale
    else:
        depth_m = depth.astype(np.float32)
    
    # Create pixel coordinate grids for depth image
    u_d, v_d = np.meshgrid(np.arange(w), np.arange(h))
    
    # Back-project depth pixels to 3D points in depth camera frame
    z = depth_m
    x = (u_d - depth_cx) * z / depth_fx
    y = (v_d - depth_cy) * z / depth_fy
    
    # Stack into (H*W, 3) points
    points_depth = np.stack([x.flatten(), y.flatten(), z.flatten()], axis=1)
    
    # Transform points from depth frame to RGB frame
    points_rgb = points_depth + [DEPTH_TO_RGB_TX, 0.0, 0.0]
    
    # Project 3D points to RGB image plane
    z_rgb = points_rgb[:, 2]
    z_rgb_safe = np.where(z_rgb > 0.001, z_rgb, 0.001)
    u_rgb = (points_rgb[:, 0] * rgb_fx / z_rgb_safe + rgb_cx).astype(np.int32)
    v_rgb = (points_rgb[:, 1] * rgb_fy / z_rgb_safe + rgb_cy).astype(np.int32)
    
    # Valid points: positive depth, within RGB image bounds
    valid = (z.flatten() > 0.001) & (z.flatten() < 10.0)
    valid = valid & (u_rgb >= 0) & (u_rgb < rgb_w) & (v_rgb >= 0) & (v_rgb < rgb_h)
    
    # Get colors from RGB image
    colors = np.zeros((len(points_depth), 3), dtype=np.float32)
    valid_indices = np.where(valid)[0]
    colors[valid_indices] = rgb[v_rgb[valid_indices], u_rgb[valid_indices]].astype(np.float32) / 255.0
    
    # Apply mask if provided (mask is in RGB frame)
    if mask is not None:
        mask_valid = np.zeros(len(points_depth), dtype=bool)
        mask_valid[valid_indices] = mask[v_rgb[valid_indices], u_rgb[valid_indices]] > 0
        valid = valid & mask_valid
    
    # Filter and create point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_depth[valid])
    pcd.colors = o3d.utility.Vector3dVector(colors[valid])
    
    return pcd


# ============================================================================
# ROS2 Node
# ============================================================================

def run_ros_node():
    """Run as ROS2 node with live point cloud visualization."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge
    
    class PointCloudVisualizerNode(Node):
        def __init__(self):
            super().__init__('visualize_pc_node')
            
            # Declare parameters
            self.declare_parameter('target_color', '')  # Empty = no mask
            self.declare_parameter('visualize', False)  # Enable OpenCV 2D preview
            self.declare_parameter('voxel_size', 0.005)  # Voxel downsampling
            self.declare_parameter('depth_scale', 1000.0)
            
            # Get parameters
            self.target_color = self.get_parameter('target_color').get_parameter_value().string_value
            self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
            self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
            self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
            
            self.bridge = CvBridge()
            self.lock = threading.Lock()
            
            # Data storage
            self.latest_rgb = None
            self.latest_depth = None
            self.latest_rgb_bgr = None
            self.K_rgb = None
            
            # Subscriptions
            self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
            self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
            self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
            
            # Open3D visualizer (non-blocking)
            self.vis = None
            self.pcd = None
            self.coord_frame = None
            self.vis_initialized = False
            
            # Timer for updating visualization
            self.timer = self.create_timer(0.1, self.update_visualization)  # 10 Hz
            
            self.get_logger().info('=' * 50)
            self.get_logger().info('Point Cloud Visualizer Node')
            self.get_logger().info(f'  Target color: {self.target_color if self.target_color else "None (full cloud)"}')
            self.get_logger().info(f'  Voxel size: {self.voxel_size}m')
            self.get_logger().info(f'  2D preview: {"ENABLED" if self.visualize else "disabled"}')
            self.get_logger().info('  Waiting for RGB-D images...')
            self.get_logger().info('=' * 50)
        
        def rgb_callback(self, msg):
            try:
                with self.lock:
                    self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
                    self.latest_rgb_bgr = cv2.cvtColor(self.latest_rgb, cv2.COLOR_RGB2BGR)
            except Exception as e:
                self.get_logger().warn(f'Failed to convert RGB: {e}')
        
        def depth_callback(self, msg):
            try:
                with self.lock:
                    if msg.encoding == '16UC1':
                        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
                    elif msg.encoding == '32FC1':
                        depth_f = self.bridge.imgmsg_to_cv2(msg, '32FC1')
                        self.latest_depth = (np.nan_to_num(depth_f, nan=0.0) * 1000).astype(np.uint16)
                    else:
                        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            except Exception as e:
                self.get_logger().warn(f'Failed to convert depth: {e}')
        
        def camera_info_callback(self, msg):
            with self.lock:
                self.K_rgb = np.array(msg.k).reshape(3, 3)
        
        def update_visualization(self):
            """Timer callback to update point cloud visualization."""
            with self.lock:
                rgb = self.latest_rgb
                depth = self.latest_depth
                rgb_bgr = self.latest_rgb_bgr
                K = self.K_rgb
            
            if rgb is None or depth is None:
                return
            
            # Get RGB intrinsics
            if K is not None:
                rgb_fx, rgb_fy = K[0, 0], K[1, 1]
                rgb_cx, rgb_cy = K[0, 2], K[1, 2]
            else:
                rgb_fx, rgb_fy = DEFAULT_RGB_FX, DEFAULT_RGB_FY
                rgb_cx, rgb_cy = DEFAULT_RGB_CX, DEFAULT_RGB_CY
            
            # Generate mask if target color specified
            mask = None
            if self.target_color:
                mask = color_mask(rgb, self.target_color)
            
            # Create point cloud
            pcd = depth_to_pointcloud(
                rgb, depth,
                DEFAULT_DEPTH_FX, DEFAULT_DEPTH_FY, DEFAULT_DEPTH_CX, DEFAULT_DEPTH_CY,
                rgb_fx, rgb_fy, rgb_cx, rgb_cy,
                mask, self.depth_scale
            )
            
            # Downsample if requested
            if self.voxel_size > 0:
                pcd = pcd.voxel_down_sample(self.voxel_size)
            
            num_points = len(pcd.points)
            
            # Initialize Open3D visualizer on first run
            if not self.vis_initialized and HAS_OPEN3D:
                self.get_logger().info('Initializing Open3D visualizer...')
                self.get_logger().info('Controls: Left-drag=Rotate, Right-drag=Pan, Scroll=Zoom')
                self.vis = o3d.visualization.Visualizer()
                self.vis.create_window('Live Point Cloud', width=1280, height=720)
                
                # Add initial point cloud
                self.pcd = pcd
                self.vis.add_geometry(self.pcd)
                
                # Add coordinate frame
                self.coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
                self.vis.add_geometry(self.coord_frame)
                
                # Set initial view
                ctr = self.vis.get_view_control()
                ctr.set_zoom(0.5)
                
                self.vis_initialized = True
            elif self.vis_initialized and HAS_OPEN3D:
                # Update point cloud
                self.pcd.points = pcd.points
                self.pcd.colors = pcd.colors
                self.vis.update_geometry(self.pcd)
                self.vis.poll_events()
                self.vis.update_renderer()
            
            # 2D OpenCV preview (optional)
            if self.visualize and rgb_bgr is not None:
                self.show_2d_preview(rgb_bgr, depth, mask, num_points)
        
        def show_2d_preview(self, rgb_bgr, depth, mask, num_points):
            """Show 2D preview of RGB, depth, and mask."""
            h, w = depth.shape[:2]
            
            # Depth colormap
            depth_vis = depth.astype(np.float32)
            depth_vis[depth_vis == 0] = np.nan
            vmin = np.nanpercentile(depth_vis, 5) if np.any(~np.isnan(depth_vis)) else 0
            vmax = np.nanpercentile(depth_vis, 95) if np.any(~np.isnan(depth_vis)) else 1
            depth_norm = np.clip((depth.astype(np.float32) - vmin) / (vmax - vmin + 1e-6), 0, 1)
            depth_colored = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
            depth_colored[depth == 0] = 0
            
            # Mask visualization
            if mask is not None:
                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            else:
                mask_bgr = np.zeros_like(rgb_bgr)
                cv2.putText(mask_bgr, 'No mask', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            
            # Info overlay on RGB
            info_img = rgb_bgr.copy()
            cv2.putText(info_img, f'Points: {num_points}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if self.target_color:
                cv2.putText(info_img, f'Color: {self.target_color}', (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            
            # Combined view
            combined = np.hstack([info_img, depth_colored, mask_bgr])
            
            # Add labels
            cv2.putText(combined, 'RGB', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(combined, 'Depth', (w + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(combined, 'Mask', (2*w + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Resize if needed
            max_width = 1600
            if combined.shape[1] > max_width:
                scale = max_width / combined.shape[1]
                combined = cv2.resize(combined, None, fx=scale, fy=scale)
            
            cv2.imshow('Point Cloud Preview: RGB | Depth | Mask', combined)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit key pressed')
                self.visualize = False
                cv2.destroyAllWindows()
        
        def destroy_node(self):
            if self.vis_initialized and self.vis is not None:
                self.vis.destroy_window()
            cv2.destroyAllWindows()
            super().destroy_node()
    
    # Initialize and run
    rclpy.init()
    node = PointCloudVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ============================================================================
# Standalone mode
# ============================================================================

def run_standalone():
    """Run standalone visualization from test data."""
    parser = argparse.ArgumentParser(description='Visualize RGB point cloud from test data')
    parser.add_argument('--pair', type=int, default=0, help='Test data pair index (default: 0)')
    parser.add_argument('--mask', type=str, default=None, 
                       help='Color mask: red, green, blue, yellow, orange, purple')
    parser.add_argument('--voxel', type=float, default=0.0,
                       help='Voxel size for downsampling (default: 0 = no downsampling)')
    parser.add_argument('--depth-scale', type=float, default=1000.0,
                       help='Depth scale factor (default: 1000 for mm to m)')
    args = parser.parse_args()
    
    if not HAS_OPEN3D:
        print("ERROR: Open3D required for visualization")
        return
    
    # Load test data
    pair_dir = os.path.join(TEST_DATA_PATH, f'pair_{args.pair:04d}')
    rgb_path = os.path.join(pair_dir, 'rgb.png')
    depth_path = os.path.join(pair_dir, 'depth.png')
    depth_npy_path = os.path.join(pair_dir, 'depth.npy')
    meta_path = os.path.join(pair_dir, 'meta.mat')
    
    if not os.path.exists(rgb_path):
        print(f"ERROR: RGB image not found: {rgb_path}")
        return
    
    # Load RGB
    print(f"Loading RGB: {rgb_path}")
    rgb = cv2.imread(rgb_path)
    rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    
    # Load depth (prefer .npy if available)
    if os.path.exists(depth_path):
        print(f"Loading depth: {depth_path}")
        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    else:
        print(f"ERROR: Depth image not found")
        return
    
    # Load intrinsics
    rgb_fx, rgb_fy, rgb_cx, rgb_cy = load_camera_intrinsics(meta_path)
    depth_fx, depth_fy = DEFAULT_DEPTH_FX, DEFAULT_DEPTH_FY
    depth_cx, depth_cy = DEFAULT_DEPTH_CX, DEFAULT_DEPTH_CY

    
    # Generate color mask if requested
    mask = None
    if args.mask:
        print(f"Applying color mask: {args.mask}")
        mask = color_mask(rgb, args.mask)
        print(f"  Mask pixels: {np.count_nonzero(mask)}")
    
    # Create point cloud
    print("Creating point cloud...")
    pcd = depth_to_pointcloud(
        rgb, depth,
        depth_fx, depth_fy, depth_cx, depth_cy,
        rgb_fx, rgb_fy, rgb_cx, rgb_cy,
        mask, args.depth_scale
    )
    print(f"  Points: {len(pcd.points)}")
    
    # Optional downsampling
    if args.voxel > 0:
        pcd = pcd.voxel_down_sample(args.voxel)
        print(f"  After downsampling: {len(pcd.points)}")
    
    # Estimate normals for better visualization
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
    
    # Create coordinate frame for reference
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    
    # Visualize
    print("\nControls: Left-drag=Rotate, Right-drag=Pan, Scroll=Zoom, Q=Quit")
    o3d.visualization.draw_geometries(
        [pcd, coord_frame],
        window_name=f'RGB Point Cloud - pair_{args.pair:04d}',
        width=1280,
        height=720
    )


def main():
    """Main entry point - detect if running as ROS2 node or standalone."""
    # Check if running with ROS2 arguments
    if '--ros-args' in sys.argv or 'ROS_DISTRO' in os.environ:
        # Check if rclpy is available
        try:
            import rclpy
            run_ros_node()
        except ImportError:
            print("ROS2 not available, running in standalone mode")
            run_standalone()
    else:
        run_standalone()


if __name__ == '__main__':
    main()
