#!/usr/bin/env python3
"""
Visualize RGB Point Cloud using Open3D.

Loads RGB and depth images from test_data folder and creates a colored 3D point cloud.
Handles unaligned RGB-D data by projecting depth points to RGB for color lookup.

Usage:
    python3 visualize_pc.py                     # Default: pair_0000
    python3 visualize_pc.py --pair 1            # Use pair_0001
    python3 visualize_pc.py --pair 0 --mask red # Show only red-masked points
    python3 visualize_pc.py --tx 20             # Adjust depth-to-RGB offset

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

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("ERROR: Open3D not installed. Install with: pip install open3d")

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
    
    return 615.0, 615.0, 320.0, 240.0  # Default RGB intrinsics


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
        depth_to_rgb_t: Translation from depth to RGB frame in meters [tx, ty, tz]
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
    
    # Transform points from depth frame to RGB frame (rotation is identity)
    points_rgb = points_depth + [15.0/ 1000.0, 0.0, 0.0] # depth_to_rgb_t
    
    # Project 3D points to RGB image plane
    u_rgb = (points_rgb[:, 0] * rgb_fx / points_rgb[:, 2] + rgb_cx).astype(np.int32)
    v_rgb = (points_rgb[:, 1] * rgb_fy / points_rgb[:, 2] + rgb_cy).astype(np.int32)
    
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


def main():
    parser = argparse.ArgumentParser(description='Visualize RGB point cloud from test data')
    parser.add_argument('--pair', type=int, default=0, help='Test data pair index (default: 0)')
    parser.add_argument('--mask', type=str, default=None, 
                       help='Color mask: red, green, blue, yellow')
    parser.add_argument('--voxel', type=float, default=0.0,
                       help='Voxel size for downsampling (default: 0 = no downsampling)')
    parser.add_argument('--depth-scale', type=float, default=1000.0,
                       help='Depth scale factor (default: 1000 for mm to m)')
    args = parser.parse_args()
    
    if not HAS_OPEN3D:
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
    if os.path.exists(depth_npy_path):
        print(f"Loading depth: {depth_npy_path}")
        depth = np.load(depth_npy_path)
    elif os.path.exists(depth_path):
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


if __name__ == '__main__':
    main()
