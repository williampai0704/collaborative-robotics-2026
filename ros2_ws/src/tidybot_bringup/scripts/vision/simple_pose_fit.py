#!/usr/bin/env python3
"""
Simple Heuristic Pose Estimation Node

Uses a geometric heuristic approach to find the pose of objects (cubes, bins)
without requiring ICP or mesh models. The approach:

1. Isolate object using color mask
2. Find table plane via RANSAC to establish "up" direction  
3. Extract top surface using Z-threshold
4. Compute 2D Oriented Bounding Box (OBB) to get position and yaw
5. Construct 4x4 transformation matrix

The OUTPUT POSE represents:
- Position: Center of the object's top surface in the DEPTH CAMERA frame
- Orientation: Object's rotation, where:
  - Z-axis points UP from the table surface (away from table)
  - X-axis points along the object's primary direction (longer edge for rectangles)
  - Y-axis completes the right-handed frame

This pose is suitable for top-down grasping where the gripper approaches
along the -Z direction (down towards the table).

Topics:
- Subscribes to:
  - /camera/color/image_raw (sensor_msgs/Image) - RGB image
  - /camera/depth/image_raw (sensor_msgs/Image) - Depth image  
  - /camera/color/camera_info (sensor_msgs/CameraInfo) - Camera intrinsics

- Publishes to:
  - /object_pose (geometry_msgs/PoseStamped) - Estimated pose in depth camera frame

Parameters:
- debug: Enable debug mode with step-by-step 3D visualization
- debug_dataset: Dataset folder name (e.g., 'yellow_block_1')
- debug_frame: Frame index within the dataset
- target_color: Color to detect (red, green, blue, yellow)
- visualize: Enable live 2D OpenCV visualization
- top_surface_threshold: Height threshold for top surface extraction (meters)
- table_distance_threshold: RANSAC distance threshold for table plane (meters)
- min_object_points: Minimum points required for pose estimation

Usage:
    # Normal mode (from camera topics)
    ros2 run tidybot_bringup simple_pose_fit.py --ros-args -p target_color:=yellow

    # With live visualization
    ros2 run tidybot_bringup simple_pose_fit.py --ros-args \\
        -p target_color:=yellow -p visualize:=true

    # Debug mode with step-by-step 3D visualization
    ros2 run tidybot_bringup simple_pose_fit.py --ros-args \\
        -p debug:=true -p debug_dataset:=yellow_block_1 -p target_color:=yellow
"""

import os
import sys
import json
import numpy as np
import threading
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

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

# =============================================================================
# Constants
# =============================================================================

TEST_DATA_PATH = '/home/cdc/collaborative-robotics-2026/test_data'
DEFAULT_DEBUG_DATASET = 'yellow_block_1'

# Default camera intrinsics (RealSense D435 at 640x480)
DEFAULT_DEPTH_FX = 384.0
DEFAULT_DEPTH_FY = 384.0
DEFAULT_DEPTH_CX = 320.0
DEFAULT_DEPTH_CY = 240.0

DEFAULT_RGB_FX = 615.0
DEFAULT_RGB_FY = 615.0
DEFAULT_RGB_CX = 320.0
DEFAULT_RGB_CY = 240.0

# Depth-to-RGB camera translation (RealSense D435)
DEPTH_TO_RGB_TX = 0.015  # meters

# =============================================================================
# Utility Functions
# =============================================================================

def depth_to_pointcloud_full(depth, fx, fy, cx, cy, scale=1000.0, 
                              min_depth=0.001, max_depth=10.0):
    """Convert depth image to full point cloud.
    
    Returns:
        points: (H, W, 3) array of 3D points in depth camera frame
        valid: (H, W) boolean mask of valid pixels
    """
    h, w = depth.shape
    depth_m = depth.astype(np.float32) / scale if depth.dtype == np.uint16 else depth.astype(np.float32)
    
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_m
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    
    valid = (z > min_depth) & (z < max_depth)
    points = np.stack([x, y, z], axis=-1)
    
    return points, valid


def depth_to_pointcloud_masked(rgb, depth, mask, 
                                depth_fx, depth_fy, depth_cx, depth_cy,
                                rgb_fx, rgb_fy, rgb_cx, rgb_cy,
                                scale=1000.0, min_depth=0.001, max_depth=10.0):
    """Convert depth to point cloud, keeping only masked pixels.
    
    Handles unaligned RGB-D by projecting depth points to RGB image.
    
    Returns:
        points: (N, 3) masked 3D points in depth camera frame
        colors: (N, 3) RGB colors normalized to [0, 1]
    """
    h, w = depth.shape
    rgb_h, rgb_w = rgb.shape[:2]
    
    depth_m = depth.astype(np.float32) / scale if depth.dtype == np.uint16 else depth.astype(np.float32)
    
    u_d, v_d = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_m
    x = (u_d - depth_cx) * z / depth_fx
    y = (v_d - depth_cy) * z / depth_fy
    
    points = np.stack([x.flatten(), y.flatten(), z.flatten()], axis=1)
    
    # Transform to RGB frame and project
    points_rgb = points + [DEPTH_TO_RGB_TX, 0.0, 0.0]
    z_rgb = np.maximum(points_rgb[:, 2], 0.001)
    u_rgb = (points_rgb[:, 0] * rgb_fx / z_rgb + rgb_cx).astype(np.int32)
    v_rgb = (points_rgb[:, 1] * rgb_fy / z_rgb + rgb_cy).astype(np.int32)
    
    # Valid points
    valid = (z.flatten() > min_depth) & (z.flatten() < max_depth)
    valid &= (u_rgb >= 0) & (u_rgb < rgb_w) & (v_rgb >= 0) & (v_rgb < rgb_h)
    
    # Apply mask
    valid_idx = np.where(valid)[0]
    mask_check = np.zeros(len(points), dtype=bool)
    mask_check[valid_idx] = mask[v_rgb[valid_idx], u_rgb[valid_idx]] > 0
    valid &= mask_check
    
    # Get colors
    colors = np.zeros((len(points), 3), dtype=np.float32)
    colors[valid_idx] = rgb[v_rgb[valid_idx], u_rgb[valid_idx]] / 255.0
    
    return points[valid], colors[valid]


def fit_plane_ransac(points, distance_threshold=0.01, num_iterations=1000):
    """Fit plane using RANSAC. Returns (plane_model, inlier_indices) or (None, None)."""
    if not HAS_OPEN3D or len(points) < 3:
        return None, None
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    plane, inliers = pcd.segment_plane(distance_threshold, 3, num_iterations)
    return plane, inliers


def compute_rotation_to_align(source, target):
    """Compute rotation matrix to align source vector to target vector."""
    source = np.array(source) / np.linalg.norm(source)
    target = np.array(target) / np.linalg.norm(target)
    
    dot = np.dot(source, target)
    if abs(dot - 1.0) < 1e-6:
        return np.eye(3)
    if abs(dot + 1.0) < 1e-6:
        perp = np.array([1, 0, 0]) if abs(source[0]) < 0.9 else np.array([0, 1, 0])
        return Rotation.from_rotvec(np.pi * perp).as_matrix()
    
    v = np.cross(source, target)
    s = np.linalg.norm(v)
    c = dot
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1 - c) / (s * s)


def extract_top_surface(points, threshold_m=0.01):
    """Extract points near the top (max Z)."""
    z = points[:, 2]
    z_max = np.max(z)
    mask = z > (z_max - threshold_m)
    return points[mask], np.where(mask)[0]


def compute_2d_obb(points_2d):
    """Compute 2D Oriented Bounding Box.
    
    Returns:
        center: (x, y)
        size: (width, height) where width >= height
        angle: yaw angle in radians
        corners: (4, 2) corner points
    """
    rect = cv2.minAreaRect(points_2d.astype(np.float32))
    center = rect[0]
    width, height = rect[1]
    angle_deg = rect[2]
    
    # Normalize so width >= height
    if width < height:
        width, height = height, width
        angle_deg += 90
    
    # Normalize angle to [-90, 90)
    angle_rad = np.deg2rad(angle_deg)
    while angle_rad > np.pi / 2:
        angle_rad -= np.pi
    while angle_rad < -np.pi / 2:
        angle_rad += np.pi
    
    corners = cv2.boxPoints(rect)
    return center, (width, height), angle_rad, corners


# =============================================================================
# ROS2 Node
# =============================================================================

class SimplePoseFitNode(Node):
    """ROS2 node for heuristic-based pose estimation."""
    
    def __init__(self):
        super().__init__('simple_pose_fit_node')
        
        if not HAS_OPEN3D:
            self.get_logger().error('Open3D required. Install: pip install open3d')
            return
        
        # Parameters
        self.declare_parameter('debug', False)
        self.declare_parameter('debug_dataset', DEFAULT_DEBUG_DATASET)
        self.declare_parameter('debug_frame', 0)
        self.declare_parameter('target_color', 'yellow')
        self.declare_parameter('visualize', False)
        self.declare_parameter('top_surface_threshold', 0.01)
        self.declare_parameter('table_distance_threshold', 0.01)
        self.declare_parameter('min_object_points', 50)
        self.declare_parameter('depth_scale', 1000.0)
        
        self.debug = self.get_parameter('debug').value
        self.debug_dataset = self.get_parameter('debug_dataset').value
        self.debug_frame = self.get_parameter('debug_frame').value
        self.target_color = self.get_parameter('target_color').value
        self.visualize = self.get_parameter('visualize').value
        self.top_threshold = self.get_parameter('top_surface_threshold').value
        self.table_threshold = self.get_parameter('table_distance_threshold').value
        self.min_points = self.get_parameter('min_object_points').value
        self.depth_scale = self.get_parameter('depth_scale').value
        
        # State
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.rgb = None
        self.rgb_bgr = None
        self.depth = None
        self.K_rgb = None
        self.K_depth = None
        self.new_data = False
        
        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/object_pose', 10)
        
        if self.debug:
            self.get_logger().info(f'DEBUG MODE: {self.debug_dataset} frame {self.debug_frame}')
            self.create_timer(1.0, self._run_debug)
        else:
            self.create_subscription(Image, '/camera/color/image_raw', self._rgb_cb, 10)
            self.create_subscription(Image, '/camera/depth/image_raw', self._depth_cb, 10)
            self.create_subscription(CameraInfo, '/camera/color/camera_info', self._info_cb, 10)
            self.create_timer(0.2, self._run_live)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Simple Pose Fit Node')
        self.get_logger().info(f'  Target: {self.target_color}')
        self.get_logger().info(f'  Debug: {self.debug}, Visualize: {self.visualize}')
        self.get_logger().info('=' * 50)
    
    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    
    def _rgb_cb(self, msg):
        with self.lock:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            self.rgb_bgr = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR)
            self.new_data = True
    
    def _depth_cb(self, msg):
        with self.lock:
            if msg.encoding == '16UC1':
                self.depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            elif msg.encoding == '32FC1':
                d = self.bridge.imgmsg_to_cv2(msg, '32FC1')
                self.depth = (np.nan_to_num(d) * 1000).astype(np.uint16)
            else:
                self.depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            self.new_data = True
    
    def _info_cb(self, msg):
        with self.lock:
            self.K_rgb = np.array(msg.k).reshape(3, 3)
    
    # -------------------------------------------------------------------------
    # Main Processing
    # -------------------------------------------------------------------------
    
    def _run_debug(self):
        """Run once in debug mode with step-by-step visualization."""
        self.destroy_timer(self._run_debug)
        
        # Load data
        base = os.path.join(TEST_DATA_PATH, self.debug_dataset)
        rgb_path = os.path.join(base, 'rgb', f'rgb_{self.debug_frame:04d}.png')
        depth_path = os.path.join(base, 'depth_raw', f'depth_{self.debug_frame:04d}.png')
        meta_path = os.path.join(base, 'meta', f'meta_{self.debug_frame:04d}.json')
        
        if not os.path.exists(rgb_path):
            self.get_logger().error(f'Not found: {rgb_path}')
            return
        
        rgb_bgr = cv2.imread(rgb_path)
        rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        
        # Load intrinsics
        if os.path.exists(meta_path):
            with open(meta_path) as f:
                meta = json.load(f)
            rgb_k = meta.get('color_camera_info', {}).get('k', [])
            depth_k = meta.get('depth_camera_info', {}).get('k', [])
            if len(rgb_k) >= 9:
                self.K_rgb = np.array([[rgb_k[0], 0, rgb_k[2]], 
                                       [0, rgb_k[4], rgb_k[5]], 
                                       [0, 0, 1]])
            if len(depth_k) >= 9:
                self.K_depth = np.array([[depth_k[0], 0, depth_k[2]], 
                                         [0, depth_k[4], depth_k[5]], 
                                         [0, 0, 1]])
        
        self.get_logger().info(f'Loaded: RGB {rgb.shape}, Depth {depth.shape}')
        
        # Run with visualization
        result = self._estimate_pose(rgb, depth, debug_viz=True)
        
        # Show final 2D result
        if result:
            self._visualize_2d(rgb_bgr, result, blocking=True)
    
    def _run_live(self):
        """Run continuously from camera topics."""
        with self.lock:
            if not self.new_data or self.rgb is None or self.depth is None:
                return
            rgb, depth, rgb_bgr = self.rgb.copy(), self.depth.copy(), self.rgb_bgr.copy()
            self.new_data = False
        
        result = self._estimate_pose(rgb, depth, debug_viz=False)
        
        if result and result.get('pose') is not None:
            self._publish_pose(result)
        
        if self.visualize:
            self._visualize_2d(rgb_bgr, result, blocking=False)
    
    def _estimate_pose(self, rgb, depth, debug_viz=False):
        """Core pose estimation pipeline.
        
        Returns dict with:
            pose: 4x4 transformation matrix (object frame in depth camera frame)
            obb_center: (x, y) center in aligned frame
            obb_size: (width, height) in meters
            yaw: rotation angle in radians
            z_max, z_min: Z bounds in aligned frame
            R_align: rotation from depth camera to aligned frame
            ... and other debug info
        """
        # Get intrinsics
        K_rgb = self.K_rgb if self.K_rgb is not None else np.array([
            [DEFAULT_RGB_FX, 0, DEFAULT_RGB_CX],
            [0, DEFAULT_RGB_FY, DEFAULT_RGB_CY],
            [0, 0, 1]
        ])
        K_depth = self.K_depth if self.K_depth is not None else np.array([
            [DEFAULT_DEPTH_FX, 0, DEFAULT_DEPTH_CX],
            [0, DEFAULT_DEPTH_FY, DEFAULT_DEPTH_CY],
            [0, 0, 1]
        ])
        
        depth_fx, depth_fy = K_depth[0, 0], K_depth[1, 1]
        depth_cx, depth_cy = K_depth[0, 2], K_depth[1, 2]
        rgb_fx, rgb_fy = K_rgb[0, 0], K_rgb[1, 1]
        rgb_cx, rgb_cy = K_rgb[0, 2], K_rgb[1, 2]
        
        result = {'pose': None, 'R_align': None, 'K_rgb': K_rgb}
        
        # ---------------------------------------------------------------------
        # STEP 0: Show inputs (debug only)
        # ---------------------------------------------------------------------
        if debug_viz:
            self._debug_step0(rgb, depth)
        
        # ---------------------------------------------------------------------
        # STEP 1: Generate mask and extract object point cloud
        # ---------------------------------------------------------------------
        self.get_logger().info(f'Step 1: Color mask ({self.target_color})...')
        mask = color_mask(rgb, self.target_color)
        
        if np.count_nonzero(mask) < self.min_points:
            self.get_logger().warn('Not enough mask pixels')
            return result
        
        obj_pts, obj_colors = depth_to_pointcloud_masked(
            rgb, depth, mask,
            depth_fx, depth_fy, depth_cx, depth_cy,
            rgb_fx, rgb_fy, rgb_cx, rgb_cy,
            self.depth_scale
        )
        result['object_points'] = len(obj_pts)
        self.get_logger().info(f'  Object points: {len(obj_pts)}')
        
        if len(obj_pts) < self.min_points:
            self.get_logger().warn('Not enough object points')
            return result
        
        if debug_viz:
            self._debug_step1(obj_pts, obj_colors)
        
        # ---------------------------------------------------------------------
        # STEP 2: Find table plane via RANSAC
        # ---------------------------------------------------------------------
        self.get_logger().info('Step 2: Table plane (RANSAC)...')
        full_pts, valid = depth_to_pointcloud_full(
            depth, depth_fx, depth_fy, depth_cx, depth_cy, self.depth_scale
        )
        full_pts_flat = full_pts[valid]
        
        plane, inliers = fit_plane_ransac(full_pts_flat, self.table_threshold)
        if plane is None:
            self.get_logger().warn('Failed to find table plane')
            return result
        
        table_normal = np.array(plane[:3])
        if table_normal[2] > 0:
            table_normal = -table_normal
        
        result['table_normal'] = table_normal
        result['table_inliers'] = len(inliers)
        self.get_logger().info(f'  Table normal: {table_normal}')
        
        if debug_viz:
            self._debug_step2(full_pts_flat, inliers, obj_pts)
        
        # ---------------------------------------------------------------------
        # STEP 3: Align to table normal (Z-up)
        # ---------------------------------------------------------------------
        self.get_logger().info('Step 3: Align to Z-up...')
        R_align = compute_rotation_to_align(table_normal, [0, 0, 1])
        result['R_align'] = R_align
        
        obj_pts_aligned = (R_align @ obj_pts.T).T
        
        z_max = np.max(obj_pts_aligned[:, 2])
        z_min = np.min(obj_pts_aligned[:, 2])
        result['z_max'] = z_max
        result['z_min'] = z_min
        result['object_height'] = z_max - z_min
        self.get_logger().info(f'  Height: {result["object_height"]:.4f}m')
        
        if debug_viz:
            self._debug_step3(obj_pts_aligned, obj_colors)
        
        # ---------------------------------------------------------------------
        # STEP 4: Extract top surface
        # ---------------------------------------------------------------------
        self.get_logger().info('Step 4: Extract top surface...')
        top_pts, _ = extract_top_surface(obj_pts_aligned, self.top_threshold)
        result['top_points'] = len(top_pts)
        self.get_logger().info(f'  Top points: {len(top_pts)}')
        
        if len(top_pts) < 10:
            self.get_logger().warn('Not enough top surface points')
            return result
        
        if debug_viz:
            self._debug_step4(obj_pts_aligned, top_pts)
        
        # ---------------------------------------------------------------------
        # STEP 5: Compute 2D OBB
        # ---------------------------------------------------------------------
        self.get_logger().info('Step 5: 2D OBB...')
        center, size, yaw, corners = compute_2d_obb(top_pts[:, :2])
        
        result['obb_center'] = center
        result['obb_size'] = size
        result['obb_corners'] = corners
        result['yaw'] = yaw
        self.get_logger().info(f'  Center: ({center[0]:.4f}, {center[1]:.4f})')
        self.get_logger().info(f'  Size: {size[0]*100:.1f}cm x {size[1]*100:.1f}cm')
        self.get_logger().info(f'  Yaw: {np.rad2deg(yaw):.1f} deg')
        
        if debug_viz:
            self._debug_step5(top_pts, center, size, yaw, corners)
        
        # ---------------------------------------------------------------------
        # STEP 6: Build pose matrix
        # ---------------------------------------------------------------------
        self.get_logger().info('Step 6: Build pose...')
        
        # Pose in aligned frame: center of top surface, Z-up, rotated by yaw
        R_yaw = Rotation.from_euler('z', yaw).as_matrix()
        t_aligned = np.array([center[0], center[1], z_max])
        
        # Transform to depth camera frame
        R_cam = R_align.T @ R_yaw
        t_cam = R_align.T @ t_aligned
        
        pose = np.eye(4)
        pose[:3, :3] = R_cam
        pose[:3, 3] = t_cam
        
        result['pose'] = pose
        self.get_logger().info(f'  Position (cam): ({t_cam[0]:.4f}, {t_cam[1]:.4f}, {t_cam[2]:.4f})')
        
        if debug_viz:
            self._debug_step6(obj_pts_aligned, center, size, yaw, z_max, z_min)
        
        return result
    
    # -------------------------------------------------------------------------
    # Debug Visualization Steps
    # -------------------------------------------------------------------------
    
    def _wait_for_window_close(self, window_name):
        """Wait until the specified OpenCV window is closed."""
        while True:
            # Check if window still exists
            try:
                # getWindowProperty returns -1 if window doesn't exist
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
            
            # Small delay to not hog CPU
            key = cv2.waitKey(100)
            if key == ord('q') or key == 27:  # q or ESC
                cv2.destroyWindow(window_name)
                break
    
    def _debug_step0(self, rgb, depth):
        """Show input images."""
        h, w = depth.shape
        depth_viz = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_viz = cv2.applyColorMap(depth_viz, cv2.COLORMAP_JET)
        
        mask = color_mask(rgb, self.target_color)
        mask_viz = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        overlay = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
        overlay[mask > 0] = [0, 255, 255]
        
        top = np.hstack([cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), depth_viz])
        bot = np.hstack([mask_viz, overlay])
        combined = np.vstack([top, bot])
        combined = cv2.resize(combined, (combined.shape[1]//2, combined.shape[0]//2))
        
        cv2.putText(combined, 'Step 0: RGB | Depth | Mask | Overlay (close window)', (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        window_name = 'Step 0: Input'
        cv2.imshow(window_name, combined)
        self.get_logger().info('Step 0: Close window to continue...')
        self._wait_for_window_close(window_name)
    
    def _debug_step1(self, pts, colors):
        """Show object point cloud."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05)
        
        self.get_logger().info('Step 1: Object point cloud (close window to continue)')
        o3d.visualization.draw_geometries([pcd, frame], 
            window_name='Step 1: Object Point Cloud', width=800, height=600)
    
    def _debug_step2(self, full_pts, inliers, obj_pts):
        """Show table segmentation."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(full_pts)
        colors = np.ones((len(full_pts), 3)) * 0.5
        colors[inliers] = [0, 0, 1]
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        pcd_obj = o3d.geometry.PointCloud()
        pcd_obj.points = o3d.utility.Vector3dVector(obj_pts)
        pcd_obj.paint_uniform_color([1, 0, 0])
        
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05)
        
        self.get_logger().info('Step 2: Blue=Table, Red=Object, Gray=Other')
        o3d.visualization.draw_geometries([pcd, pcd_obj, frame],
            window_name='Step 2: Table + Object', width=800, height=600)
    
    def _debug_step3(self, pts_aligned, colors):
        """Show aligned point cloud."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_aligned)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05)
        
        self.get_logger().info('Step 3: Aligned (Z-up from table)')
        o3d.visualization.draw_geometries([pcd, frame],
            window_name='Step 3: Aligned Object', width=800, height=600)
    
    def _debug_step4(self, pts_aligned, top_pts):
        """Show top surface extraction."""
        pcd_full = o3d.geometry.PointCloud()
        pcd_full.points = o3d.utility.Vector3dVector(pts_aligned)
        pcd_full.paint_uniform_color([0.5, 0.5, 0.5])
        
        pcd_top = o3d.geometry.PointCloud()
        pcd_top.points = o3d.utility.Vector3dVector(top_pts)
        pcd_top.paint_uniform_color([0, 1, 0])
        
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05)
        
        self.get_logger().info('Step 4: Gray=Full, Green=Top surface')
        o3d.visualization.draw_geometries([pcd_full, pcd_top, frame],
            window_name='Step 4: Top Surface', width=800, height=600)
    
    def _debug_step5(self, top_pts, center, size, yaw, corners):
        """Show 2D OBB."""
        fig_size = 600
        margin = 50
        
        xy = top_pts[:, :2]
        xy_min, xy_max = xy.min(axis=0), xy.max(axis=0)
        scale = (fig_size - 2*margin) / max(xy_max[0] - xy_min[0], xy_max[1] - xy_min[1], 0.001)
        
        img = np.zeros((fig_size, fig_size, 3), dtype=np.uint8)
        
        for pt in xy:
            px = int((pt[0] - xy_min[0]) * scale + margin)
            py = fig_size - int((pt[1] - xy_min[1]) * scale + margin)
            cv2.circle(img, (px, py), 2, (0, 255, 0), -1)
        
        corners_px = []
        for c in corners:
            px = int((c[0] - xy_min[0]) * scale + margin)
            py = fig_size - int((c[1] - xy_min[1]) * scale + margin)
            corners_px.append((px, py))
        
        for i in range(4):
            cv2.line(img, corners_px[i], corners_px[(i+1)%4], (255, 0, 0), 2)
        
        cx = int((center[0] - xy_min[0]) * scale + margin)
        cy = fig_size - int((center[1] - xy_min[1]) * scale + margin)
        cv2.drawMarker(img, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
        
        cv2.putText(img, f'Size: {size[0]*100:.1f}x{size[1]*100:.1f}cm, Yaw: {np.rad2deg(yaw):.1f}deg (close window)',
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        window_name = 'Step 5: 2D OBB'
        cv2.imshow(window_name, img)
        self.get_logger().info('Step 5: Close window to continue...')
        self._wait_for_window_close(window_name)
    
    def _debug_step6(self, pts_aligned, center, size, yaw, z_max, z_min):
        """Show final pose in 3D."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_aligned)
        pcd.paint_uniform_color([0.7, 0.7, 0.7])
        
        # Build box
        w, h = size
        if w / max(h, 0.001) < 2:
            w = h = min(w, h)
        d = z_max - z_min
        
        half_w, half_h = w/2, h/2
        cx, cy = center
        
        corners = np.array([
            [cx-half_w, cy-half_h, z_max], [cx+half_w, cy-half_h, z_max],
            [cx+half_w, cy+half_h, z_max], [cx-half_w, cy+half_h, z_max],
            [cx-half_w, cy-half_h, z_min], [cx+half_w, cy-half_h, z_min],
            [cx+half_w, cy+half_h, z_min], [cx-half_w, cy+half_h, z_min],
        ])
        
        R_yaw = Rotation.from_euler('z', yaw).as_matrix()
        center_3d = np.array([cx, cy, (z_max+z_min)/2])
        corners = np.array([R_yaw @ (c - center_3d) + center_3d for c in corners])
        
        lines = [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]]
        lineset = o3d.geometry.LineSet()
        lineset.points = o3d.utility.Vector3dVector(corners)
        lineset.lines = o3d.utility.Vector2iVector(lines)
        lineset.colors = o3d.utility.Vector3dVector([[0,1,0]]*len(lines))
        
        pose_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.03)
        pose_frame.translate([cx, cy, z_max])
        pose_frame.rotate(R_yaw)
        
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05)
        
        self.get_logger().info('Step 6: Final pose (green box, RGB axes)')
        o3d.visualization.draw_geometries([pcd, lineset, pose_frame, frame],
            window_name='Step 6: Final Pose', width=800, height=600)
        
        cv2.destroyAllWindows()
    
    # -------------------------------------------------------------------------
    # 2D Visualization
    # -------------------------------------------------------------------------
    
    def _visualize_2d(self, rgb_bgr, result, blocking=False):
        """Draw 2D visualization with projected bounding box."""
        output = rgb_bgr.copy()
        h, w = output.shape[:2]
        
        if result.get('R_align') is not None and result.get('obb_center') is not None:
            self._draw_bbox_2d(output, result)
            
            pos = result['pose'][:3, 3] if result.get('pose') is not None else [0,0,0]
            yaw = np.rad2deg(result.get('yaw', 0))
            
            texts = [
                f'Pos: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})m',
                f'Yaw: {yaw:.1f}deg | Pts: {result.get("object_points", 0)}',
            ]
            for i, t in enumerate(texts):
                cv2.putText(output, t, (10, 25+i*25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            
            cv2.putText(output, 'POSE OK', (w-120, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        else:
            cv2.putText(output, 'No pose', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        
        cv2.imshow('Simple Pose Fit', output)
        
        if blocking:
            self.get_logger().info('Press any key to close')
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.visualize = False
                cv2.destroyAllWindows()
    
    def _draw_bbox_2d(self, image, result):
        """Project 3D bounding box onto 2D image."""
        R_align = result['R_align']
        center = result['obb_center']
        size = result['obb_size']
        yaw = result.get('yaw', 0)
        z_max = result['z_max']
        z_min = result['z_min']
        K = result['K_rgb']
        
        # Box dimensions (square for cubes)
        w, h = size
        if w / max(h, 0.001) < 2:
            w = h = min(w, h)
        
        half_w, half_h = w/2, h/2
        cx, cy = center
        
        # Corners in aligned frame
        corners = np.array([
            [cx-half_w, cy-half_h, z_max], [cx+half_w, cy-half_h, z_max],
            [cx+half_w, cy+half_h, z_max], [cx-half_w, cy+half_h, z_max],
            [cx-half_w, cy-half_h, z_min], [cx+half_w, cy-half_h, z_min],
            [cx+half_w, cy+half_h, z_min], [cx-half_w, cy+half_h, z_min],
        ])
        
        # Apply yaw
        R_yaw = Rotation.from_euler('z', yaw).as_matrix()
        center_3d = np.array([cx, cy, (z_max+z_min)/2])
        corners = np.array([R_yaw @ (c - center_3d) + center_3d for c in corners])
        
        # Transform to depth camera frame, then to RGB frame
        corners_cam = (R_align.T @ corners.T).T
        corners_rgb = corners_cam + [DEPTH_TO_RGB_TX, 0, 0]
        
        # Project
        corners_2d = []
        for pt in corners_rgb:
            if pt[2] > 0.001:
                u = int(pt[0] * K[0,0] / pt[2] + K[0,2])
                v = int(pt[1] * K[1,1] / pt[2] + K[1,2])
                corners_2d.append((u, v))
            else:
                corners_2d.append(None)
        
        # Draw
        for i, j in [(0,1),(1,2),(2,3),(3,0)]:
            if corners_2d[i] and corners_2d[j]:
                cv2.line(image, corners_2d[i], corners_2d[j], (0,255,0), 2)
        for i, j in [(4,5),(5,6),(6,7),(7,4)]:
            if corners_2d[i] and corners_2d[j]:
                cv2.line(image, corners_2d[i], corners_2d[j], (0,180,0), 2)
        for i, j in [(0,4),(1,5),(2,6),(3,7)]:
            if corners_2d[i] and corners_2d[j]:
                cv2.line(image, corners_2d[i], corners_2d[j], (255,255,0), 2)
        
        for i in range(4):
            if corners_2d[i]:
                cv2.circle(image, corners_2d[i], 4, (255,0,0), -1)
        
        # Center marker
        center_aligned = np.array([cx, cy, z_max])
        center_cam = R_align.T @ center_aligned
        center_rgb = center_cam + [DEPTH_TO_RGB_TX, 0, 0]
        if center_rgb[2] > 0.001:
            cu = int(center_rgb[0] * K[0,0] / center_rgb[2] + K[0,2])
            cv = int(center_rgb[1] * K[1,1] / center_rgb[2] + K[1,2])
            cv2.drawMarker(image, (cu, cv), (0,0,255), cv2.MARKER_CROSS, 15, 2)
    
    # -------------------------------------------------------------------------
    # Publishing
    # -------------------------------------------------------------------------
    
    def _publish_pose(self, result):
        """Publish pose as PoseStamped."""
        pose = result['pose']
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_depth_optical_frame'
        
        msg.pose.position.x = float(pose[0, 3])
        msg.pose.position.y = float(pose[1, 3])
        msg.pose.position.z = float(pose[2, 3])
        
        quat = Rotation.from_matrix(pose[:3, :3]).as_quat()
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePoseFitNode()
    
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
