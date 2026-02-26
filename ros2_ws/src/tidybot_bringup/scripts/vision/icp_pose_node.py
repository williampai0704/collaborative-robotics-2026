#!/usr/bin/env python3
"""
ICP-based Object Pose Estimation Node

A simpler alternative to FoundationPose that uses ICP (Iterative Closest Point)
for 6-DOF pose estimation from RGB-D images and object masks.

Supports two mask sources:
1. Color-based mask (default) - uses HSV thresholding
2. SAM3 mask (optional) - from external segmentation

Debug mode loads images from test_data folder instead of ROS topics.

Pipeline:
1. Receive RGB image, depth image, object mask, and camera intrinsics
2. Back-project masked depth pixels to 3D point cloud (scene cloud)
3. Sample points from object mesh (model cloud)
4. Initial alignment via centroid matching + PCA
5. Refine with ICP

Topics:
- Subscribes to:
  - /camera/color/image_raw (sensor_msgs/Image) - RGB image
  - /camera/depth/image_raw (sensor_msgs/Image) - Depth image
  - /camera/color/camera_info (sensor_msgs/CameraInfo) - Camera intrinsics
  - /object_mask (sensor_msgs/Image) - Object segmentation mask (if use_sam3_mask=true)

- Publishes to:
  - /object_pose (geometry_msgs/PoseStamped) - Estimated 6-DOF pose in camera frame
  - /object_pose_base (geometry_msgs/PoseStamped) - Estimated 6-DOF pose in robot base frame
  - /icp_pose/debug_image (sensor_msgs/Image) - Debug visualization
  - /icp_pose/scene_cloud (sensor_msgs/PointCloud2) - Scene point cloud
  - /icp_pose/model_cloud (sensor_msgs/PointCloud2) - Transformed model cloud

Parameters:
- mesh_file: Path to object mesh (.obj, .stl, .ply)
- debug: Enable debug mode (load from test_data)
- debug_pair: Which pair to load (0-5)
- target_color: Color for mask (r, g, b, y)
- use_sam3_mask: Use SAM3 mask instead of color mask
- num_mesh_samples: Number of points to sample from mesh (default: 5000)
- icp_max_iterations: Max ICP iterations (default: 50)
- icp_threshold: ICP convergence threshold (default: 1e-6)
- voxel_size: Voxel size for downsampling (default: 0.005m)
- depth_scale: Depth image scale factor (default: 1000.0 for mm to m)
- camera_frame: Camera optical frame name (default: camera_color_optical_frame)
- base_frame: Robot base frame name (default: base_link)

Usage:
    # Normal mode
    ros2 run tidybot_bringup icp_pose_node.py --ros-args \
        -p mesh_file:=/path/to/object.obj

    # With live visualization
    ros2 run tidybot_bringup icp_pose_node.py --ros-args \
        -p mesh_file:=/path/to/object.obj -p visualize:=true

    # Debug mode (from test_data)
    ros2 run tidybot_bringup icp_pose_node.py --ros-args \
        -p mesh_file:=/path/to/object.obj \
        -p debug:=true -p debug_pair:=0 -p target_color:=r
"""

import os
import sys
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy.time
import rclpy.duration
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
import struct

# TF2 for frame transformations
from tf2_ros import Buffer, TransformListener, TransformException

# Add script directory to path for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# Also add the source directory (for when running from install location)
SOURCE_VISION_DIR = '/home/cdc/collaborative-robotics-2026/ros2_ws/src/tidybot_bringup/scripts/vision'
if SOURCE_VISION_DIR not in sys.path:
    sys.path.insert(0, SOURCE_VISION_DIR)

from color_mask import color_mask

# Default depth camera intrinsics (RealSense D435 at 640x480)
DEFAULT_DEPTH_FX = 384.0
DEFAULT_DEPTH_FY = 384.0
DEFAULT_DEPTH_CX = 320.0
DEFAULT_DEPTH_CY = 240.0

# Default RGB camera intrinsics
DEFAULT_RGB_FX = 615.0
DEFAULT_RGB_FY = 615.0
DEFAULT_RGB_CX = 320.0
DEFAULT_RGB_CY = 240.0

# Default depth-to-RGB translation (RealSense D435: ~15mm offset)
DEFAULT_DEPTH_TO_RGB_TX = 0.015  # meters


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
    except Exception:
        pass
    return DEFAULT_RGB_FX, DEFAULT_RGB_FY, DEFAULT_RGB_CX, DEFAULT_RGB_CY


def depth_to_pointcloud_unaligned(rgb, depth, 
                                   depth_fx, depth_fy, depth_cx, depth_cy,
                                   rgb_fx, rgb_fy, rgb_cx, rgb_cy,
                                   mask=None, depth_scale=1.0, min_depth=0.001, max_depth=10.0):
    """Convert unaligned RGB-D images to colored point cloud.
    
    Creates geometry from depth camera frame and projects each 3D point
    to RGB image to get its color. This handles the case where depth and
    RGB cameras have different intrinsics and a physical offset.
    
    Args:
        rgb: RGB image (H, W, 3) uint8
        depth: Depth image (H, W) - can be uint16 (mm) or float (m)
        depth_fx, depth_fy, depth_cx, depth_cy: Depth camera intrinsics
        rgb_fx, rgb_fy, rgb_cx, rgb_cy: RGB camera intrinsics
        mask: Optional binary mask (in RGB frame) to filter points
        depth_scale: Scale factor (1.0 if already in meters, 1000.0 for mm to m)
        min_depth: Minimum valid depth in meters
        max_depth: Maximum valid depth in meters
    
    Returns:
        Open3D PointCloud with colors, or None if not enough points
    """
    h, w = depth.shape
    rgb_h, rgb_w = rgb.shape[:2]
    
    # Convert depth to meters if needed
    if depth.dtype == np.uint16:
        depth_m = depth.astype(np.float32) / depth_scale
    elif depth_scale != 1.0:
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
    
    # Transform points from depth frame to RGB frame (15mm X offset for D435)
    points_rgb = points_depth + np.array([DEFAULT_DEPTH_TO_RGB_TX, 0.0, 0.0])
    
    # Project 3D points to RGB image plane
    z_rgb = points_rgb[:, 2]
    # Avoid division by zero
    z_rgb_safe = np.where(z_rgb > 0.001, z_rgb, 0.001)
    u_rgb = (points_rgb[:, 0] * rgb_fx / z_rgb_safe + rgb_cx).astype(np.int32)
    v_rgb = (points_rgb[:, 1] * rgb_fy / z_rgb_safe + rgb_cy).astype(np.int32)
    
    # Valid points: positive depth, within RGB image bounds
    valid = (z.flatten() > min_depth) & (z.flatten() < max_depth)
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

# Optional imports for point cloud processing
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False

try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False

# Test data path for debug mode
TEST_DATA_PATH = '/home/cdc/collaborative-robotics-2026/test_data'


class ICPPoseNode(Node):
    """ROS2 node for ICP-based 6-DOF pose estimation."""

    def __init__(self):
        super().__init__('icp_pose_node')

        # Check dependencies
        if not OPEN3D_AVAILABLE:
            self.get_logger().error('Open3D not available. Install with: pip install open3d')
            return
        if not TRIMESH_AVAILABLE:
            self.get_logger().error('Trimesh not available. Install with: pip install trimesh')
            return

        # Declare parameters
        self.declare_parameter('mesh_file', '')
        self.declare_parameter('debug', False)
        self.declare_parameter('debug_pair', 0)
        self.declare_parameter('target_color', 'r')
        self.declare_parameter('use_sam3_mask', False)
        self.declare_parameter('num_mesh_samples', 5000)
        self.declare_parameter('icp_max_iterations', 50)
        self.declare_parameter('icp_threshold', 1e-6)
        self.declare_parameter('voxel_size', 0.005)  # 5mm
        self.declare_parameter('depth_scale', 1000.0)  # mm to m
        self.declare_parameter('mesh_scale', 0.01)  # Scale factor for mesh (0.01 for cm->m, 0.001 for mm->m)
        self.declare_parameter('min_points', 100)  # Minimum points for ICP
        self.declare_parameter('use_color_icp', False)  # Use color information in ICP
        self.declare_parameter('use_pca_alignment', False)  # Use PCA for initial rotation (disable for symmetric objects)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('visualize', False)  # Enable live visualization

        # Get parameters
        mesh_file = self.get_parameter('mesh_file').get_parameter_value().string_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.debug_pair = self.get_parameter('debug_pair').get_parameter_value().integer_value
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value
        self.use_sam3_mask = self.get_parameter('use_sam3_mask').get_parameter_value().bool_value
        self.num_mesh_samples = self.get_parameter('num_mesh_samples').get_parameter_value().integer_value
        self.icp_max_iterations = self.get_parameter('icp_max_iterations').get_parameter_value().integer_value
        self.icp_threshold = self.get_parameter('icp_threshold').get_parameter_value().double_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
        self.min_points = self.get_parameter('min_points').get_parameter_value().integer_value
        self.use_color_icp = self.get_parameter('use_color_icp').get_parameter_value().bool_value
        self.use_pca_alignment = self.get_parameter('use_pca_alignment').get_parameter_value().bool_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.mesh_scale = self.get_parameter('mesh_scale').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        
        # Store for visualization
        self.latest_rgb_bgr = None
        self.latest_depth_raw = None

        if not mesh_file:
            self.get_logger().error('mesh_file parameter is required!')
            return

        if not os.path.exists(mesh_file):
            self.get_logger().error(f'Mesh file not found: {mesh_file}')
            return

        # Load and prepare model point cloud
        self.get_logger().info(f'Loading mesh: {mesh_file}')
        self.model_cloud = self._load_mesh_as_pointcloud(mesh_file)
        if self.model_cloud is None:
            return

        # CV Bridge
        self.bridge = CvBridge()

        # Thread lock
        self.lock = threading.Lock()

        # Data storage
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        self.object_mask = None  # For SAM3 mask mode
        self.K = None  # Camera intrinsic matrix
        self.new_data_received = False

        # Last estimated pose (for tracking/smoothing)
        self.last_pose = None

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.pose_base_pub = self.create_publisher(PoseStamped, '/object_pose_base', 10)
        self.debug_pub = self.create_publisher(Image, '/icp_pose/debug_image', 10)
        self.scene_cloud_pub = self.create_publisher(PointCloud2, '/icp_pose/scene_cloud', 10)
        self.model_cloud_pub = self.create_publisher(PointCloud2, '/icp_pose/model_cloud', 10)

        # TF2 buffer for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.debug:
            # Debug mode: load from test_data
            self.get_logger().info(f'DEBUG MODE: Loading from test_data/pair_{self.debug_pair:04d}')
            self.get_logger().info(f'Target color: {self.target_color}')
            # Set default camera intrinsics for debug mode (typical RealSense D435)
            self.K = np.array([
                [615.0, 0.0, 320.0],
                [0.0, 615.0, 240.0],
                [0.0, 0.0, 1.0]
            ])
            self.timer = self.create_timer(1.0, self.debug_process)
        else:
            # Normal mode: subscribe to topics
            image_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=1
            )

            self.rgb_sub = self.create_subscription(
                Image, '/camera/color/image_raw', self.rgb_callback, image_qos)
            self.depth_sub = self.create_subscription(
                Image, '/camera/depth/image_raw', self.depth_callback, image_qos)
            self.camera_info_sub = self.create_subscription(
                CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

            if self.use_sam3_mask:
                self.get_logger().info('Using SAM3 mask from /object_mask')
                self.mask_sub = self.create_subscription(
                    Image, '/object_mask', self.mask_callback, 10)
            else:
                self.get_logger().info(f'Using color mask (target: {self.target_color})')

            # Timer for pose estimation (10 Hz)
            self.timer = self.create_timer(0.1, self.estimate_pose_callback)

        self.get_logger().info('=' * 50)
        self.get_logger().info('ICP Pose Estimation Node Initialized')
        self.get_logger().info(f'  Mesh: {mesh_file}')
        self.get_logger().info(f'  Mesh scale: {self.mesh_scale} (0.01 for cm->m, 0.001 for mm->m)')
        self.get_logger().info(f'  Model points: {len(self.model_cloud.points)}')
        self.get_logger().info(f'  Voxel size: {self.voxel_size}m')
        self.get_logger().info(f'  ICP iterations: {self.icp_max_iterations}')
        self.get_logger().info(f'  Debug mode: {self.debug}')
        self.get_logger().info(f'  Mask mode: {"SAM3" if self.use_sam3_mask else f"color ({self.target_color})"}')
        if self.visualize:
            self.get_logger().info('  Visualization: ENABLED - Press "q" to quit')
        self.get_logger().info('=' * 50)

    def debug_process(self):
        """Process test data in debug mode with visualization."""
        self.timer.cancel()

        pair_dir = os.path.join(TEST_DATA_PATH, f'pair_{self.debug_pair:04d}')
        rgb_path = os.path.join(pair_dir, 'rgb.png')
        depth_path = os.path.join(pair_dir, 'depth.png')
        depth_npy_path = os.path.join(pair_dir, 'depth.npy')
        meta_path = os.path.join(pair_dir, 'meta.mat')

        if not os.path.exists(rgb_path):
            self.get_logger().error(f'RGB image not found: {rgb_path}')
            return

        # Load RGB image
        rgb_bgr = cv2.imread(rgb_path)
        rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
        self.get_logger().info(f'Loaded RGB image: {rgb.shape}')

        # Load depth (prefer .npy if available)
        if os.path.exists(depth_npy_path):
            depth_raw = np.load(depth_npy_path)
            self.get_logger().info(f'Loaded depth from npy: {depth_raw.shape}, dtype: {depth_raw.dtype}')
        elif os.path.exists(depth_path):
            depth_raw = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            self.get_logger().info(f'Loaded depth from png: {depth_raw.shape}, dtype: {depth_raw.dtype}')
        else:
            self.get_logger().error(f'Depth image not found in {pair_dir}')
            return

        # Convert depth to meters
        if depth_raw.dtype == np.uint16:
            depth = depth_raw.astype(np.float32) / self.depth_scale
        elif depth_raw.max() > 100:  # Likely in mm
            depth = depth_raw.astype(np.float32) / self.depth_scale
        else:
            depth = depth_raw.astype(np.float32)

        # Load camera intrinsics
        rgb_fx, rgb_fy, rgb_cx, rgb_cy = load_camera_intrinsics(meta_path)
        self.K = np.array([
            [rgb_fx, 0.0, rgb_cx],
            [0.0, rgb_fy, rgb_cy],
            [0.0, 0.0, 1.0]
        ])

        # Generate color mask
        mask = color_mask(rgb, self.target_color)
        self.get_logger().info(f'Generated color mask, non-zero pixels: {np.count_nonzero(mask)}')

        # Run pose estimation and get results
        result = self.run_pose_estimation(rgb, depth, mask, return_result=True)

        # Visualize results
        self.visualize_debug(rgb_bgr, depth_raw, mask, rgb, result)

    def visualize_debug(self, rgb_bgr, depth_raw, mask, rgb, result):
        """Visualize debug results with OpenCV and Open3D.
        
        Args:
            rgb_bgr: Original RGB image in BGR format
            depth_raw: Raw depth image (for visualization)
            mask: Binary mask
            rgb: RGB image (for point cloud)
            result: Dictionary with pose estimation results or None
        """
        h, w = mask.shape[:2]
        
        # === 2D Visualization with OpenCV ===
        
        # 1. Depth colormap
        depth_vis = depth_raw.astype(np.float32)
        depth_vis[depth_vis == 0] = np.nan
        vmin = np.nanpercentile(depth_vis, 5) if np.any(~np.isnan(depth_vis)) else 0
        vmax = np.nanpercentile(depth_vis, 95) if np.any(~np.isnan(depth_vis)) else 1
        depth_normalized = np.clip((depth_raw.astype(np.float32) - vmin) / (vmax - vmin + 1e-6), 0, 1)
        depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)
        depth_colored[depth_raw == 0] = 0
        
        # 2. Mask visualization
        color_tints = {
            'red': (0, 0, 255), 'r': (0, 0, 255),
            'green': (0, 255, 0), 'g': (0, 255, 0),
            'blue': (255, 0, 0), 'b': (255, 0, 0),
            'yellow': (0, 255, 255), 'y': (0, 255, 255)
        }
        tint = color_tints.get(self.target_color.lower(), (255, 255, 255))
        mask_colored = np.zeros_like(rgb_bgr)
        mask_colored[mask > 0] = tint
        
        # 3. Overlay with pose visualization
        overlay = rgb_bgr.copy()
        overlay = cv2.addWeighted(overlay, 0.7, mask_colored, 0.3, 0)
        
        # Draw mask contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, tint, 2)
        
        # Add pose info if available
        if result and result.get('transform') is not None:
            transform = result['transform']
            pos = transform[:3, 3]
            fitness = result.get('fitness', 0)
            rmse = result.get('rmse', 0)
            
            # Draw projected bounding box
            self._draw_bbox_on_image(overlay, transform, self.K)
            
            # Add text info
            info_text = [
                f'Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m',
                f'Fitness: {fitness:.4f}',
                f'RMSE: {rmse:.6f}',
            ]
            for i, text in enumerate(info_text):
                cv2.putText(overlay, text, (10, 25 + i * 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(overlay, text, (10, 25 + i * 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        else:
            cv2.putText(overlay, 'Pose estimation failed', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # 4. Info panel
        info_panel = np.zeros_like(rgb_bgr)
        info_lines = [
            f'Target Color: {self.target_color}',
            f'Image Size: {w} x {h}',
            f'Mask Pixels: {np.count_nonzero(mask)}',
            f'Mesh: {os.path.basename(self.get_parameter("mesh_file").get_parameter_value().string_value)}',
        ]
        if result:
            info_lines.extend([
                f'Scene Points: {result.get("scene_points", 0)}',
                f'Model Points: {len(self.model_cloud.points)}',
            ])
        for i, line in enumerate(info_lines):
            cv2.putText(info_panel, line, (10, 25 + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        
        # 5. Create combined 2D visualization (2x2 grid)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        row1 = np.hstack([rgb_bgr, depth_colored])
        row2 = np.hstack([overlay, mask_bgr])
        combined = np.vstack([row1, row2])
        
        # Add labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(combined, 'RGB', (10, 20), font, 0.6, (255, 255, 255), 1)
        cv2.putText(combined, 'Depth', (w + 10, 20), font, 0.6, (255, 255, 255), 1)
        cv2.putText(combined, 'Pose Overlay', (10, h + 20), font, 0.6, (255, 255, 255), 1)
        cv2.putText(combined, 'Mask', (w + 10, h + 20), font, 0.6, (255, 255, 255), 1)
        
        cv2.imshow('ICP Pose Debug - 2D', combined)
        
        # === 3D Visualization with Open3D ===
        if result and result.get('scene_cloud') is not None:
            self.get_logger().info('Opening 3D visualization...')
            self.get_logger().info('Controls: Left-drag=Rotate, Right-drag=Pan, Scroll=Zoom, Q=Quit')
            self.get_logger().info('Legend:')
            self.get_logger().info('  - Colored points: Scene point cloud (from depth + RGB)')
            self.get_logger().info('  - RED points: Model transformed by ICP result')
            self.get_logger().info('  - Small coord frame: At estimated object pose')
            self.get_logger().info('  - Large coord frame: Camera origin (0,0,0)')
            
            geometries = []
            
            # Scene point cloud (colored from RGB)
            scene_cloud = result['scene_cloud']
            geometries.append(scene_cloud)
            
            # Model point cloud with INITIAL alignment (blue) - to see if initial guess is reasonable
            if result.get('initial_transform') is not None:
                init_transform = result['initial_transform']
                model_initial = o3d.geometry.PointCloud(self.model_cloud)
                model_initial.transform(init_transform)
                model_initial.paint_uniform_color([0.0, 0.0, 1.0])  # Blue
                geometries.append(model_initial)
                self.get_logger().info('BLUE points: Model with initial alignment (before ICP)')
            
            # Model point cloud with FINAL transform (red) - ICP result
            if result.get('transform') is not None:
                transform = result['transform']
                model_transformed = o3d.geometry.PointCloud(self.model_cloud)
                model_transformed.transform(transform)
                model_transformed.paint_uniform_color([1.0, 0.0, 0.0])  # Red
                geometries.append(model_transformed)
                self.get_logger().info('RED points: Model with ICP result (final)')
                
                # Log transform details
                pos = transform[:3, 3]
                self.get_logger().info(f'ICP Transform position: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m')
                
                # Coordinate frame at the CENTROID of transformed model (not mesh origin)
                # This shows where the object center actually is
                transformed_centroid = np.asarray(model_transformed.points).mean(axis=0)
                coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                    size=0.03, origin=transformed_centroid)
                geometries.append(coord_frame)
                self.get_logger().info(f'Object centroid (red frame): ({transformed_centroid[0]:.4f}, {transformed_centroid[1]:.4f}, {transformed_centroid[2]:.4f}) m')
            
            # Camera origin coordinate frame (larger, for reference)
            origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.05, origin=[0, 0, 0])
            geometries.append(origin_frame)
            
            # Print scene cloud stats for debugging
            scene_pts = np.asarray(scene_cloud.points)
            self.get_logger().info(f'Scene cloud centroid: ({scene_pts.mean(axis=0)[0]:.4f}, {scene_pts.mean(axis=0)[1]:.4f}, {scene_pts.mean(axis=0)[2]:.4f}) m')
            self.get_logger().info(f'Scene cloud Z range: {scene_pts[:, 2].min():.4f} - {scene_pts[:, 2].max():.4f} m')
            
            model_pts = np.asarray(self.model_cloud.points)
            self.get_logger().info(f'Model cloud (untransformed) centroid: ({model_pts.mean(axis=0)[0]:.4f}, {model_pts.mean(axis=0)[1]:.4f}, {model_pts.mean(axis=0)[2]:.4f}) m')
            self.get_logger().info(f'Model cloud extents: {model_pts.max(axis=0) - model_pts.min(axis=0)} m')
            
            # Show 3D visualization
            o3d.visualization.draw_geometries(
                geometries,
                window_name='ICP Pose Debug - 3D Point Clouds',
                width=1280,
                height=720
            )
        
        self.get_logger().info('Press any key on 2D window to close...')
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def visualize_live(self, rgb_bgr, depth_raw, mask, result):
        """Live visualization during streaming (non-blocking).
        
        Args:
            rgb_bgr: Original RGB image in BGR format
            depth_raw: Raw depth image
            mask: Binary mask
            result: Dictionary with pose estimation results
        """
        h, w = mask.shape[:2]
        
        # 1. Depth colormap
        depth_vis = depth_raw.astype(np.float32)
        depth_vis[depth_vis == 0] = np.nan
        vmin = np.nanpercentile(depth_vis, 5) if np.any(~np.isnan(depth_vis)) else 0
        vmax = np.nanpercentile(depth_vis, 95) if np.any(~np.isnan(depth_vis)) else 1
        depth_normalized = np.clip((depth_raw.astype(np.float32) - vmin) / (vmax - vmin + 1e-6), 0, 1)
        depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)
        depth_colored[depth_raw == 0] = 0
        
        # 2. Get color tint
        color_tints = {
            'red': (0, 0, 255), 'r': (0, 0, 255),
            'green': (0, 255, 0), 'g': (0, 255, 0),
            'blue': (255, 0, 0), 'b': (255, 0, 0),
            'yellow': (0, 255, 255), 'y': (0, 255, 255)
        }
        tint = color_tints.get(self.target_color.lower(), (255, 255, 255))
        
        # 3. Overlay with mask and pose
        mask_colored = np.zeros_like(rgb_bgr)
        mask_colored[mask > 0] = tint
        overlay = cv2.addWeighted(rgb_bgr.copy(), 0.7, mask_colored, 0.3, 0)
        
        # Draw mask contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, tint, 2)
        
        # Add pose info if available
        if result and result.get('transform') is not None:
            transform = result['transform']
            pos = transform[:3, 3]
            fitness = result.get('fitness', 0)
            
            # Draw projected bounding box
            self._draw_bbox_on_image(overlay, transform, self.K)
            
            # Add text
            text = f'Pos: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})m  Fit: {fitness:.3f}'
            cv2.putText(overlay, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(overlay, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        else:
            cv2.putText(overlay, 'Pose estimation failed', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Stats
        stats_text = f'Color: {self.target_color} | Mask: {np.count_nonzero(mask)} | Scene: {result.get("scene_points", 0)}'
        cv2.putText(overlay, stats_text, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 4. Create combined view: RGB | Depth | Overlay
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([rgb_bgr, depth_colored, mask_bgr, overlay])
        
        # Add labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(combined, 'RGB', (10, 20), font, 0.5, (255, 255, 255), 1)
        cv2.putText(combined, 'Depth', (w + 10, 20), font, 0.5, (255, 255, 255), 1)
        cv2.putText(combined, 'Mask', (2*w + 10, 20), font, 0.5, (255, 255, 255), 1)
        cv2.putText(combined, 'Pose', (3*w + 10, 20), font, 0.5, (255, 255, 255), 1)
        
        # Resize if too large
        max_width = 1920
        if combined.shape[1] > max_width:
            scale = max_width / combined.shape[1]
            combined = cv2.resize(combined, None, fx=scale, fy=scale)
        
        cv2.imshow('ICP Pose: RGB | Depth | Mask | Pose', combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quit key pressed, disabling visualization')
            self.visualize = False
            cv2.destroyAllWindows()

    def _draw_bbox_on_image(self, image, transform, K):
        """Draw projected 3D bounding box on image."""
        try:
            corners_3d = self._get_bbox_corners()
            corners_transformed = (transform[:3, :3] @ corners_3d.T + transform[:3, 3:4]).T
            
            # Project to 2D
            corners_2d = (K @ corners_transformed.T).T
            corners_2d = corners_2d[:, :2] / corners_2d[:, 2:3]
            corners_2d = corners_2d.astype(int)
            
            # Draw edges
            edges = [
                (0, 1), (1, 3), (3, 2), (2, 0),  # Bottom face
                (4, 5), (5, 7), (7, 6), (6, 4),  # Top face
                (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical edges
            ]
            for i, j in edges:
                pt1 = tuple(corners_2d[i])
                pt2 = tuple(corners_2d[j])
                cv2.line(image, pt1, pt2, (0, 255, 0), 2)
            
            # Draw coordinate axes at object origin
            origin = transform[:3, 3]
            axis_length = 0.05
            axes = np.eye(3) * axis_length
            axes_transformed = (transform[:3, :3] @ axes.T).T + origin
            
            origin_2d = (K @ origin)
            origin_2d = (int(origin_2d[0] / origin_2d[2]), int(origin_2d[1] / origin_2d[2]))
            
            colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # BGR for XYZ
            for i, color in enumerate(colors):
                end = axes_transformed[i]
                end_2d = (K @ end)
                end_2d = (int(end_2d[0] / end_2d[2]), int(end_2d[1] / end_2d[2]))
                cv2.arrowedLine(image, origin_2d, end_2d, color, 2, tipLength=0.2)
        except Exception as e:
            self.get_logger().warn(f'Failed to draw bbox: {e}')

    def _load_mesh_as_pointcloud(self, mesh_file: str) -> o3d.geometry.PointCloud:
        """Load mesh and sample points to create model point cloud.
        
        The mesh is scaled by mesh_scale parameter and CENTERED at the origin.
        - mesh_scale=0.01 (default): mesh is in centimeters
        - mesh_scale=0.001: mesh is in millimeters
        - mesh_scale=1.0: mesh is already in meters
        """
        try:
            # Load mesh with trimesh (handles more formats)
            mesh = trimesh.load(mesh_file)
            self.get_logger().info(f'Loaded mesh: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces')
            
            # Apply scale if needed (e.g., convert mm to m)
            if self.mesh_scale != 1.0:
                mesh.apply_scale(self.mesh_scale)
                self.get_logger().info(f'Applied mesh scale: {self.mesh_scale} (mesh units -> meters)')
            
            # Center mesh at origin (important for proper ICP alignment!)
            # This ensures the model centroid is at (0,0,0)
            mesh_centroid = mesh.centroid
            mesh.vertices -= mesh_centroid
            self.get_logger().info(f'Centered mesh at origin (shifted by {mesh_centroid})')

            # Sample points uniformly from mesh surface
            points, face_indices = trimesh.sample.sample_surface(mesh, self.num_mesh_samples)
            self.get_logger().info(f'Sampled {len(points)} points from mesh surface')
            self.get_logger().info(f'Mesh extents after scaling: {mesh.extents} meters')

            # Get colors if available (from vertex colors or face colors)
            colors = None
            if mesh.visual.kind == 'vertex' and hasattr(mesh.visual, 'vertex_colors'):
                # Interpolate vertex colors to sampled points
                colors = mesh.visual.vertex_colors[mesh.faces[face_indices]].mean(axis=1)[:, :3] / 255.0
            elif mesh.visual.kind == 'face' and hasattr(mesh.visual, 'face_colors'):
                colors = mesh.visual.face_colors[face_indices][:, :3] / 255.0

            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            if colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(colors)

            # Estimate normals
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 2, max_nn=30))

            # Store mesh bounds for visualization
            self.mesh_bounds = mesh.bounds
            self.mesh_extents = mesh.extents

            return pcd

        except Exception as e:
            self.get_logger().error(f'Failed to load mesh: {e}')
            return None

    def rgb_callback(self, msg: Image):
        """Handle RGB image."""
        try:
            with self.lock:
                self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
                self.latest_rgb_bgr = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
                self.new_data_received = True
        except Exception as e:
            self.get_logger().warn(f'Failed to convert RGB image: {e}')

    def depth_callback(self, msg: Image):
        """Handle depth image."""
        try:
            with self.lock:
                depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                self.latest_depth_raw = depth.copy()  # Store raw for visualization
                # Convert to meters
                if depth.dtype == np.uint16:
                    self.depth_image = depth.astype(np.float32) / self.depth_scale
                else:
                    self.depth_image = depth.astype(np.float32)
                self.new_data_received = True
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """Handle camera info."""
        with self.lock:
            self.camera_info = msg
            self.K = np.array(msg.k).reshape(3, 3)

    def mask_callback(self, msg: Image):
        """Handle SAM3 object segmentation mask (kept for future use)."""
        try:
            with self.lock:
                mask = self.bridge.imgmsg_to_cv2(msg, 'mono8')
                self.object_mask = (mask > 127).astype(np.uint8) * 255
                self.new_data_received = True
                self.get_logger().info('New SAM3 mask received')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert mask: {e}')

    def estimate_pose_callback(self):
        """Timer callback to run pose estimation."""
        with self.lock:
            rgb = self.rgb_image
            depth = self.depth_image
            depth_raw = self.latest_depth_raw
            rgb_bgr = self.latest_rgb_bgr
            K = self.K
            sam3_mask = self.object_mask
            new_data = self.new_data_received
            self.new_data_received = False

        # Check if we have required data
        if depth is None or K is None or rgb is None:
            return

        if not new_data:
            return

        # Generate mask
        if self.use_sam3_mask:
            if sam3_mask is None:
                return
            mask = sam3_mask
        else:
            mask = color_mask(rgb, self.target_color)

        self.run_pose_estimation(rgb, depth, mask, rgb_bgr=rgb_bgr, depth_raw=depth_raw)

    def run_pose_estimation(self, rgb, depth, mask, return_result=False, rgb_bgr=None, depth_raw=None):
        """Run the full pose estimation pipeline.
        
        Args:
            rgb: RGB image
            depth: Depth image in meters
            mask: Binary mask
            return_result: If True, return dict with results for visualization
            rgb_bgr: BGR image for visualization (optional)
            depth_raw: Raw depth image for visualization (optional)
            
        Returns:
            If return_result: dict with 'transform', 'fitness', 'rmse', 'scene_cloud', 'scene_points'
            Otherwise: None
        """
        result = {
            'transform': None,
            'initial_transform': None,
            'fitness': 0,
            'rmse': 0,
            'scene_cloud': None,
            'scene_points': 0,
        }
        
        try:
            # Step 1: Back-project masked depth to 3D point cloud
            self.get_logger().info('Back-projecting depth to point cloud...')
            scene_cloud = self.backproject_to_pointcloud(depth, mask, self.K, rgb)

            if scene_cloud is None:
                if return_result:
                    return result
                return

            self.get_logger().info(f'Scene cloud: {len(scene_cloud.points)} points')
            result['scene_points'] = len(scene_cloud.points)

            # Downsample scene cloud
            scene_cloud_down = scene_cloud.voxel_down_sample(self.voxel_size)
            self.get_logger().info(f'Downsampled scene cloud: {len(scene_cloud_down.points)} points')
            result['scene_cloud'] = scene_cloud_down

            # Step 2: Initial alignment (centroid matching, optionally with PCA rotation)
            alignment_type = "centroid + PCA" if self.use_pca_alignment else "centroid only (no rotation)"
            self.get_logger().info(f'Computing initial alignment ({alignment_type})...')
            initial_transform = self.compute_initial_alignment(
                self.model_cloud, scene_cloud_down, use_pca=self.use_pca_alignment)
            result['initial_transform'] = initial_transform
            
            init_pos = initial_transform[:3, 3]
            self.get_logger().info(f'Initial alignment position: ({init_pos[0]:.4f}, {init_pos[1]:.4f}, {init_pos[2]:.4f}) m')

            # Step 3: ICP refinement
            self.get_logger().info('Running ICP refinement...')
            final_transform, fitness, rmse = self.run_icp(
                self.model_cloud, scene_cloud_down, initial_transform)

            self.get_logger().info(f'ICP result: fitness={fitness:.4f}, RMSE={rmse:.6f}')
            final_pos = final_transform[:3, 3]
            self.get_logger().info(f'Final position: ({final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f}) m')
            
            result['transform'] = final_transform
            result['fitness'] = fitness
            result['rmse'] = rmse

            # Store last pose
            self.last_pose = final_transform

            # Publish pose
            self._publish_pose(final_transform)

            # Publish debug visualization
            if rgb is not None:
                self._publish_debug_image(rgb, final_transform, self.K, mask)

            # Publish point clouds for RViz
            self._publish_pointclouds(scene_cloud_down, final_transform)

        except Exception as e:
            self.get_logger().error(f'Pose estimation failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        # Live visualization (non-blocking)
        if self.visualize and rgb_bgr is not None and depth_raw is not None:
            self.visualize_live(rgb_bgr, depth_raw, mask, result)
        
        if return_result:
            return result

    def backproject_to_pointcloud(self, depth: np.ndarray, mask: np.ndarray,
                                   K: np.ndarray, rgb: np.ndarray = None) -> o3d.geometry.PointCloud:
        """
        Back-project masked depth pixels to 3D point cloud.
        
        Uses the unaligned depth-to-pointcloud function that handles the physical
        offset between depth and RGB cameras on RealSense D435.

        Args:
            depth: Depth image (H, W) in meters
            mask: Binary mask (H, W)
            K: Camera intrinsic matrix (3, 3) - RGB camera intrinsics
            rgb: Optional RGB image for coloring points

        Returns:
            Open3D PointCloud in depth camera frame, or None if not enough points
        """
        if rgb is None:
            self.get_logger().warn('RGB image required for point cloud generation')
            return None
        
        # Extract RGB intrinsics from K matrix
        rgb_fx, rgb_fy = K[0, 0], K[1, 1]
        rgb_cx, rgb_cy = K[0, 2], K[1, 2]
        
        # Use the unaligned depth-to-pointcloud function
        # depth_scale=1.0 because depth is already in meters
        pcd = depth_to_pointcloud_unaligned(
            rgb=rgb,
            depth=depth,
            depth_fx=DEFAULT_DEPTH_FX,
            depth_fy=DEFAULT_DEPTH_FY,
            depth_cx=DEFAULT_DEPTH_CX,
            depth_cy=DEFAULT_DEPTH_CY,
            rgb_fx=rgb_fx,
            rgb_fy=rgb_fy,
            rgb_cx=rgb_cx,
            rgb_cy=rgb_cy,
            mask=mask,
            depth_scale=1.0,  # Already in meters
            min_depth=0.001,
            max_depth=10.0
        )
        
        if pcd is None or len(pcd.points) < self.min_points:
            self.get_logger().warn(f'Not enough valid points: {len(pcd.points) if pcd else 0} < {self.min_points}')
            return None
        
        # Estimate normals
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=self.voxel_size * 2, max_nn=30))

        return pcd

    def compute_initial_alignment(self, source: o3d.geometry.PointCloud,
                                   target: o3d.geometry.PointCloud,
                                   use_pca: bool = False) -> np.ndarray:
        """
        Compute initial alignment using centroid matching (and optionally PCA).

        Args:
            source: Model point cloud (to be transformed)
            target: Scene point cloud (reference)
            use_pca: If True, also align principal axes. If False, only translate
                     centroids (more robust for symmetric objects or shape mismatch)

        Returns:
            4x4 transformation matrix
        """
        # Get points as numpy arrays
        source_pts = np.asarray(source.points)
        target_pts = np.asarray(target.points)

        # Step 1: Centroid alignment (always done)
        source_centroid = source_pts.mean(axis=0)
        target_centroid = target_pts.mean(axis=0)

        if not use_pca:
            # Simple translation-only alignment (no rotation)
            # This is more robust for:
            # - Symmetric objects (cubes, spheres)
            # - Shape mismatches
            # - Partial views
            T = np.eye(4)
            T[:3, 3] = target_centroid - source_centroid
            return T

        # Step 2: PCA for orientation alignment (optional)
        # Center the point clouds
        source_centered = source_pts - source_centroid
        target_centered = target_pts - target_centroid

        # Compute covariance matrices
        source_cov = np.cov(source_centered.T)
        target_cov = np.cov(target_centered.T)

        # Compute eigenvectors (principal axes)
        _, source_eigvec = np.linalg.eigh(source_cov)
        _, target_eigvec = np.linalg.eigh(target_cov)

        # Sort by eigenvalues (descending) - eigh returns ascending order
        source_eigvec = source_eigvec[:, ::-1]
        target_eigvec = target_eigvec[:, ::-1]

        # Ensure right-handed coordinate system
        if np.linalg.det(source_eigvec) < 0:
            source_eigvec[:, 2] *= -1
        if np.linalg.det(target_eigvec) < 0:
            target_eigvec[:, 2] *= -1

        # Rotation to align principal axes
        R = target_eigvec @ source_eigvec.T

        # Build transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = target_centroid - R @ source_centroid

        return T

    def run_icp(self, source: o3d.geometry.PointCloud,
                target: o3d.geometry.PointCloud,
                initial_transform: np.ndarray) -> tuple:
        """
        Run ICP refinement.

        Args:
            source: Model point cloud
            target: Scene point cloud
            initial_transform: Initial alignment transform

        Returns:
            (final_transform, fitness_score, inlier_rmse)
        """
        # Downsample for faster ICP
        source_down = source.voxel_down_sample(self.voxel_size)
        target_down = target.voxel_down_sample(self.voxel_size)

        # ICP parameters
        threshold = self.voxel_size * 2  # Max correspondence distance

        # Run point-to-plane ICP (more robust than point-to-point)
        result = o3d.pipelines.registration.registration_icp(
            source_down, target_down,
            threshold,
            initial_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.icp_max_iterations,
                relative_fitness=self.icp_threshold,
                relative_rmse=self.icp_threshold
            )
        )

        return result.transformation, result.fitness, result.inlier_rmse

    def _publish_pose(self, transform: np.ndarray):
        """Publish pose as PoseStamped message in both camera and base frames."""
        stamp = self.get_clock().now().to_msg()

        # Extract rotation as quaternion
        rotation = Rotation.from_matrix(transform[:3, :3])
        quat = rotation.as_quat()  # [x, y, z, w]

        # Create pose message in camera frame
        msg_camera = PoseStamped()
        msg_camera.header = Header()
        msg_camera.header.stamp = stamp
        msg_camera.header.frame_id = self.camera_frame

        msg_camera.pose.position.x = float(transform[0, 3])
        msg_camera.pose.position.y = float(transform[1, 3])
        msg_camera.pose.position.z = float(transform[2, 3])
        msg_camera.pose.orientation.x = float(quat[0])
        msg_camera.pose.orientation.y = float(quat[1])
        msg_camera.pose.orientation.z = float(quat[2])
        msg_camera.pose.orientation.w = float(quat[3])

        # Publish pose in camera frame
        self.pose_pub.publish(msg_camera)
        self.get_logger().info(
            f'Pose (camera): pos=({transform[0,3]:.3f}, {transform[1,3]:.3f}, {transform[2,3]:.3f})')

        # Transform and publish pose in base frame
        self._publish_pose_in_base_frame(msg_camera)

    def _publish_pose_in_base_frame(self, pose_camera: PoseStamped):
        """Transform pose from camera frame to base frame and publish."""
        try:
            # Look up transform from camera frame to base frame
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),  # Get latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Convert pose to 4x4 matrix
            pose_mat = self._pose_msg_to_matrix(pose_camera.pose)

            # Convert TF transform to 4x4 matrix
            tf_mat = self._transform_msg_to_matrix(transform.transform)

            # Transform pose: pose_base = T_base_camera @ pose_camera
            pose_base_mat = tf_mat @ pose_mat

            # Extract position and orientation
            pos = pose_base_mat[:3, 3]
            rot = Rotation.from_matrix(pose_base_mat[:3, :3])
            quat = rot.as_quat()  # [x, y, z, w]

            # Create pose message in base frame
            msg_base = PoseStamped()
            msg_base.header.stamp = pose_camera.header.stamp
            msg_base.header.frame_id = self.base_frame

            msg_base.pose.position.x = float(pos[0])
            msg_base.pose.position.y = float(pos[1])
            msg_base.pose.position.z = float(pos[2])
            msg_base.pose.orientation.x = float(quat[0])
            msg_base.pose.orientation.y = float(quat[1])
            msg_base.pose.orientation.z = float(quat[2])
            msg_base.pose.orientation.w = float(quat[3])

            # Publish pose in base frame
            self.pose_base_pub.publish(msg_base)
            self.get_logger().info(
                f'Pose (base): pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})')

        except TransformException as e:
            self.get_logger().warn(f'Could not transform pose to base frame: {e}')

    def _pose_msg_to_matrix(self, pose) -> np.ndarray:
        """Convert geometry_msgs/Pose to 4x4 transformation matrix."""
        mat = np.eye(4)

        # Position
        mat[0, 3] = pose.position.x
        mat[1, 3] = pose.position.y
        mat[2, 3] = pose.position.z

        # Orientation (quaternion to rotation matrix)
        quat = [pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w]
        rot = Rotation.from_quat(quat)
        mat[:3, :3] = rot.as_matrix()

        return mat

    def _transform_msg_to_matrix(self, transform) -> np.ndarray:
        """Convert geometry_msgs/Transform to 4x4 transformation matrix."""
        mat = np.eye(4)

        # Translation
        mat[0, 3] = transform.translation.x
        mat[1, 3] = transform.translation.y
        mat[2, 3] = transform.translation.z

        # Rotation (quaternion to rotation matrix)
        quat = [transform.rotation.x, transform.rotation.y,
                transform.rotation.z, transform.rotation.w]
        rot = Rotation.from_quat(quat)
        mat[:3, :3] = rot.as_matrix()

        return mat

    def _publish_debug_image(self, rgb: np.ndarray, transform: np.ndarray,
                             K: np.ndarray, mask: np.ndarray):
        """Publish debug visualization with projected bounding box."""
        try:
            vis = rgb.copy()

            # Draw mask overlay
            mask_overlay = np.zeros_like(vis)
            mask_overlay[:, :, 1] = mask.astype(np.uint8)  # Green tint
            vis = cv2.addWeighted(vis, 0.7, mask_overlay, 0.3, 0)

            # Project mesh bounding box corners to image
            corners_3d = self._get_bbox_corners()
            corners_transformed = (transform[:3, :3] @ corners_3d.T + transform[:3, 3:4]).T

            # Project to 2D
            corners_2d = (K @ corners_transformed.T).T
            corners_2d = corners_2d[:, :2] / corners_2d[:, 2:3]
            corners_2d = corners_2d.astype(int)

            # Draw bounding box edges
            edges = [
                (0, 1), (1, 3), (3, 2), (2, 0),  # Bottom face
                (4, 5), (5, 7), (7, 6), (6, 4),  # Top face
                (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical edges
            ]
            for i, j in edges:
                pt1 = tuple(corners_2d[i])
                pt2 = tuple(corners_2d[j])
                cv2.line(vis, pt1, pt2, (0, 255, 0), 2)

            # Draw coordinate axes
            origin = transform[:3, 3]
            axis_length = 0.05  # 5cm axes

            axes = np.eye(3) * axis_length
            axes_transformed = (transform[:3, :3] @ axes.T).T + origin

            origin_2d_raw = K @ origin
            if origin_2d_raw[2] > 0.001:  # Only draw if in front of camera
                origin_2d = (int(origin_2d_raw[0] / origin_2d_raw[2]), 
                             int(origin_2d_raw[1] / origin_2d_raw[2]))

                colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]  # RGB for XYZ
                for i, color in enumerate(colors):
                    end = axes_transformed[i]
                    end_2d_raw = K @ end
                    if end_2d_raw[2] > 0.001:
                        end_2d = (int(end_2d_raw[0] / end_2d_raw[2]), 
                                  int(end_2d_raw[1] / end_2d_raw[2]))
                        cv2.arrowedLine(vis, origin_2d, end_2d, color, 2, tipLength=0.2)

            # Convert to ROS message
            debug_msg = self.bridge.cv2_to_imgmsg(vis, 'rgb8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = 'camera_color_optical_frame'
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().warn(f'Failed to publish debug image: {e}')

    def _get_bbox_corners(self) -> np.ndarray:
        """Get 8 corners of the mesh bounding box."""
        min_bound = self.mesh_bounds[0]
        max_bound = self.mesh_bounds[1]

        corners = np.array([
            [min_bound[0], min_bound[1], min_bound[2]],
            [max_bound[0], min_bound[1], min_bound[2]],
            [min_bound[0], max_bound[1], min_bound[2]],
            [max_bound[0], max_bound[1], min_bound[2]],
            [min_bound[0], min_bound[1], max_bound[2]],
            [max_bound[0], min_bound[1], max_bound[2]],
            [min_bound[0], max_bound[1], max_bound[2]],
            [max_bound[0], max_bound[1], max_bound[2]],
        ])
        return corners

    def _publish_pointclouds(self, scene_cloud: o3d.geometry.PointCloud,
                              transform: np.ndarray):
        """Publish point clouds for RViz visualization."""
        stamp = self.get_clock().now().to_msg()
        frame_id = 'camera_color_optical_frame'

        # Publish scene cloud
        scene_msg = self._pointcloud_to_ros(scene_cloud, stamp, frame_id)
        self.scene_cloud_pub.publish(scene_msg)

        # Transform and publish model cloud (copy first to avoid modifying original!)
        model_transformed = o3d.geometry.PointCloud(self.model_cloud)
        model_transformed.transform(transform)
        model_msg = self._pointcloud_to_ros(model_transformed, stamp, frame_id)
        self.model_cloud_pub.publish(model_msg)

    def _pointcloud_to_ros(self, pcd: o3d.geometry.PointCloud,
                           stamp, frame_id: str) -> PointCloud2:
        """Convert Open3D PointCloud to ROS PointCloud2."""
        points = np.asarray(pcd.points)
        has_colors = len(pcd.colors) > 0

        if has_colors:
            colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.is_dense = True

        if has_colors:
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            msg.point_step = 16
            msg.row_step = msg.point_step * msg.width

            data = []
            for i in range(len(points)):
                rgb = (int(colors[i, 0]) << 16) | (int(colors[i, 1]) << 8) | int(colors[i, 2])
                data.append(struct.pack('fffI', points[i, 0], points[i, 1], points[i, 2], rgb))
            msg.data = b''.join(data)
        else:
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 12
            msg.row_step = msg.point_step * msg.width
            msg.data = points.astype(np.float32).tobytes()

        return msg


def main(args=None):
    rclpy.init(args=args)

    node = ICPPoseNode()

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
