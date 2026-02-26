#!/usr/bin/env python3
"""
ICP-based Object Pose Estimation Node

A simpler alternative to FoundationPose that uses ICP (Iterative Closest Point)
for 6-DOF pose estimation from RGB-D images and object masks.

Pipeline:
1. Receive RGB image, depth image, object mask, and camera intrinsics
2. Back-project masked depth pixels to 3D point cloud (scene cloud)
3. Sample points from object mesh (model cloud)
4. Initial alignment via centroid matching + PCA
5. Refine with ICP

Topics:
- Subscribes to:
  - /camera/color/image_raw (sensor_msgs/Image) - RGB image (for visualization)
  - /camera/depth/image_raw (sensor_msgs/Image) - Depth image
  - /camera/color/camera_info (sensor_msgs/CameraInfo) - Camera intrinsics
  - /object_mask (sensor_msgs/Image) - Object segmentation mask

- Publishes to:
  - /object_pose (geometry_msgs/PoseStamped) - Estimated 6-DOF pose in camera frame
  - /object_pose_base (geometry_msgs/PoseStamped) - Estimated 6-DOF pose in robot base frame
  - /icp_pose/debug_image (sensor_msgs/Image) - Debug visualization
  - /icp_pose/scene_cloud (sensor_msgs/PointCloud2) - Scene point cloud
  - /icp_pose/model_cloud (sensor_msgs/PointCloud2) - Transformed model cloud

Parameters:
- mesh_file: Path to object mesh (.obj, .stl, .ply)
- num_mesh_samples: Number of points to sample from mesh (default: 5000)
- icp_max_iterations: Max ICP iterations (default: 50)
- icp_threshold: ICP convergence threshold (default: 1e-6)
- voxel_size: Voxel size for downsampling (default: 0.005m)
- depth_scale: Depth image scale factor (default: 1000.0 for mm to m)
- camera_frame: Camera optical frame name (default: camera_color_optical_frame)
- base_frame: Robot base frame name (default: base_link)

Usage:
    ros2 run tidybot_bringup icp_pose_node.py --ros-args \
        -p mesh_file:=/path/to/object.obj
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
        self.declare_parameter('num_mesh_samples', 5000)
        self.declare_parameter('icp_max_iterations', 50)
        self.declare_parameter('icp_threshold', 1e-6)
        self.declare_parameter('voxel_size', 0.005)  # 5mm
        self.declare_parameter('depth_scale', 1000.0)  # mm to m
        self.declare_parameter('min_points', 100)  # Minimum points for ICP
        self.declare_parameter('use_color_icp', False)  # Use color information in ICP
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')

        # Get parameters
        mesh_file = self.get_parameter('mesh_file').get_parameter_value().string_value
        self.num_mesh_samples = self.get_parameter('num_mesh_samples').get_parameter_value().integer_value
        self.icp_max_iterations = self.get_parameter('icp_max_iterations').get_parameter_value().integer_value
        self.icp_threshold = self.get_parameter('icp_threshold').get_parameter_value().double_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
        self.min_points = self.get_parameter('min_points').get_parameter_value().integer_value
        self.use_color_icp = self.get_parameter('use_color_icp').get_parameter_value().bool_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

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
        self.object_mask = None
        self.K = None  # Camera intrinsic matrix
        self.new_mask_received = False

        # Last estimated pose (for tracking/smoothing)
        self.last_pose = None

        # QoS for image topics
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, image_qos)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, image_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.mask_sub = self.create_subscription(
            Image, '/object_mask', self.mask_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.pose_base_pub = self.create_publisher(PoseStamped, '/object_pose_base', 10)
        self.debug_pub = self.create_publisher(Image, '/icp_pose/debug_image', 10)
        self.scene_cloud_pub = self.create_publisher(PointCloud2, '/icp_pose/scene_cloud', 10)
        self.model_cloud_pub = self.create_publisher(PointCloud2, '/icp_pose/model_cloud', 10)

        # TF2 buffer for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for pose estimation (10 Hz)
        self.timer = self.create_timer(0.1, self.estimate_pose_callback)

        self.get_logger().info('=' * 50)
        self.get_logger().info('ICP Pose Estimation Node Initialized')
        self.get_logger().info(f'  Mesh: {mesh_file}')
        self.get_logger().info(f'  Model points: {len(self.model_cloud.points)}')
        self.get_logger().info(f'  Voxel size: {self.voxel_size}m')
        self.get_logger().info(f'  ICP iterations: {self.icp_max_iterations}')
        self.get_logger().info(f'  Camera frame: {self.camera_frame}')
        self.get_logger().info(f'  Base frame: {self.base_frame}')
        self.get_logger().info('  Publishing:')
        self.get_logger().info('    /object_pose (camera frame)')
        self.get_logger().info('    /object_pose_base (robot base frame)')
        self.get_logger().info('  Waiting for images and object mask...')
        self.get_logger().info('=' * 50)

    def _load_mesh_as_pointcloud(self, mesh_file: str) -> o3d.geometry.PointCloud:
        """Load mesh and sample points to create model point cloud."""
        try:
            # Load mesh with trimesh (handles more formats)
            mesh = trimesh.load(mesh_file)
            self.get_logger().info(f'Loaded mesh: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces')

            # Sample points uniformly from mesh surface
            points, face_indices = trimesh.sample.sample_surface(mesh, self.num_mesh_samples)
            self.get_logger().info(f'Sampled {len(points)} points from mesh surface')

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
        except Exception as e:
            self.get_logger().warn(f'Failed to convert RGB image: {e}')

    def depth_callback(self, msg: Image):
        """Handle depth image."""
        try:
            with self.lock:
                depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                # Convert to meters
                if depth.dtype == np.uint16:
                    self.depth_image = depth.astype(np.float32) / self.depth_scale
                else:
                    self.depth_image = depth.astype(np.float32)
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """Handle camera info."""
        with self.lock:
            self.camera_info = msg
            self.K = np.array(msg.k).reshape(3, 3)

    def mask_callback(self, msg: Image):
        """Handle object segmentation mask."""
        try:
            with self.lock:
                mask = self.bridge.imgmsg_to_cv2(msg, 'mono8')
                self.object_mask = (mask > 127).astype(np.uint8)
                self.new_mask_received = True
                self.get_logger().info('New object mask received')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert mask: {e}')

    def backproject_to_pointcloud(self, depth: np.ndarray, mask: np.ndarray,
                                   K: np.ndarray, rgb: np.ndarray = None) -> o3d.geometry.PointCloud:
        """
        Back-project masked depth pixels to 3D point cloud.

        Args:
            depth: Depth image (H, W) in meters
            mask: Binary mask (H, W)
            K: Camera intrinsic matrix (3, 3)
            rgb: Optional RGB image for coloring points

        Returns:
            Open3D PointCloud in camera frame
        """
        H, W = depth.shape

        # Get valid pixels (masked and valid depth)
        valid = (mask > 0) & (depth > 0.001) & (depth < 10.0)  # 1mm to 10m
        vs, us = np.where(valid)

        if len(us) < self.min_points:
            self.get_logger().warn(f'Not enough valid points: {len(us)} < {self.min_points}')
            return None

        # Get depth values
        zs = depth[vs, us]

        # Back-project to 3D
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        xs = (us - cx) * zs / fx
        ys = (vs - cy) * zs / fy

        points = np.stack([xs, ys, zs], axis=1)

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Add colors if available
        if rgb is not None:
            colors = rgb[vs, us, :] / 255.0
            pcd.colors = o3d.utility.Vector3dVector(colors)

        # Estimate normals
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=self.voxel_size * 2, max_nn=30))

        return pcd

    def compute_initial_alignment(self, source: o3d.geometry.PointCloud,
                                   target: o3d.geometry.PointCloud) -> np.ndarray:
        """
        Compute initial alignment using centroid matching and PCA.

        Args:
            source: Model point cloud (to be transformed)
            target: Scene point cloud (reference)

        Returns:
            4x4 transformation matrix
        """
        # Get points as numpy arrays
        source_pts = np.asarray(source.points)
        target_pts = np.asarray(target.points)

        # Step 1: Centroid alignment
        source_centroid = source_pts.mean(axis=0)
        target_centroid = target_pts.mean(axis=0)

        # Center the point clouds
        source_centered = source_pts - source_centroid
        target_centered = target_pts - target_centroid

        # Step 2: PCA for orientation alignment
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

    def estimate_pose_callback(self):
        """Timer callback to run pose estimation."""
        with self.lock:
            rgb = self.rgb_image
            depth = self.depth_image
            K = self.K
            mask = self.object_mask
            new_mask = self.new_mask_received
            self.new_mask_received = False

        # Check if we have all required data
        if depth is None or K is None or mask is None:
            return

        # Only run when we have a new mask
        if not new_mask:
            return

        try:
            # Step 1: Back-project masked depth to 3D point cloud
            self.get_logger().info('Back-projecting depth to point cloud...')
            scene_cloud = self.backproject_to_pointcloud(depth, mask, K, rgb)

            if scene_cloud is None:
                return

            self.get_logger().info(f'Scene cloud: {len(scene_cloud.points)} points')

            # Downsample scene cloud
            scene_cloud_down = scene_cloud.voxel_down_sample(self.voxel_size)
            self.get_logger().info(f'Downsampled scene cloud: {len(scene_cloud_down.points)} points')

            # Step 2: Initial alignment (centroid + PCA)
            self.get_logger().info('Computing initial alignment...')
            initial_transform = self.compute_initial_alignment(self.model_cloud, scene_cloud_down)

            # Step 3: ICP refinement
            self.get_logger().info('Running ICP refinement...')
            final_transform, fitness, rmse = self.run_icp(
                self.model_cloud, scene_cloud_down, initial_transform)

            self.get_logger().info(f'ICP result: fitness={fitness:.4f}, RMSE={rmse:.6f}')

            # Store last pose
            self.last_pose = final_transform

            # Publish pose
            self._publish_pose(final_transform)

            # Publish debug visualization
            if rgb is not None:
                self._publish_debug_image(rgb, final_transform, K, mask)

            # Publish point clouds for RViz
            self._publish_pointclouds(scene_cloud_down, final_transform)

        except Exception as e:
            self.get_logger().error(f'Pose estimation failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

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
            mask_overlay[:, :, 1] = mask * 100  # Green tint
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

            origin_2d = (K @ origin).astype(int)
            origin_2d = (origin_2d[0] // origin_2d[2], origin_2d[1] // origin_2d[2])

            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]  # RGB for XYZ
            for i, color in enumerate(colors):
                end = axes_transformed[i]
                end_2d = (K @ end).astype(int)
                end_2d = (end_2d[0] // end_2d[2], end_2d[1] // end_2d[2])
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

        # Transform and publish model cloud
        model_transformed = self.model_cloud.transform(transform)
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
