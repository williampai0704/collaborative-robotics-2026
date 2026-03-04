#!/usr/bin/env python3
"""
TidyBot2 Motion Planner Node for Real Hardware

Provides trajectory planning with collision checking and singularity avoidance.
Uses Pinocchio for inverse kinematics (no MuJoCo dependency) with expanded
safety validations ported from the simulation node.

Services:
- /plan_to_target (PlanToTarget): Plan and optionally execute arm motion
"""

import numpy as np
from pathlib import Path
from threading import Lock
import threading  # [MODIFIED] Added threading module to prevent ROS 2 executor blocking
import subprocess
import time
from ament_index_python.packages import get_package_share_directory

import pinocchio as pin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

from interbotix_xs_msgs.msg import JointGroupCommand
from tidybot_msgs.srv import PlanToTarget


class MotionPlannerRealNode(Node):
    """Motion planner with IK using Pinocchio for real hardware."""

    # Joint limits (from WX250s specs)
    JOINT_LIMITS = {
        'waist': (-3.14159, 3.14159),
        'shoulder': (-1.8849, 1.9897),
        'elbow': (-2.1468, 1.6057),
        'forearm_roll': (-3.14159, 3.14159),
        'wrist_angle': (-1.7453, 2.1468),
        'wrist_rotate': (-3.14159, 3.14159),
    }

    # Default seed position (non-singular)
    DEFAULT_SEED = np.array([0.0, -1.0, 0.8, 0.0, 0.5, 0.0])

    # Additional seed configurations to help the local IK solver avoid local minima
    EXTRA_SEEDS = [
        np.array([0.0, -0.3, 0.7, 0.0, -1.0, 0.0]),
        np.array([0.0, -0.3, 0.7, np.pi, -1.0, 0.0]),
        np.array([0.0, -0.3, 0.7, np.pi/2, -1.0, 0.0]),
        np.array([0.0, -0.3, 0.7, -np.pi/2, -1.0, 0.0]),
        np.array([0.0, -0.8, 1.0, 0.0, 0.3, 0.0]),
        np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0]),
    ]

    def __init__(self):
        super().__init__('motion_planner_real')

        # Declare parameters for IK solver, tolerances, and safety margins
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('ik_dt', 0.3)
        self.declare_parameter('ik_max_iterations', 200)
        self.declare_parameter('position_tolerance', 0.03)
        self.declare_parameter('orientation_tolerance', 0.1)
        self.declare_parameter('min_collision_distance', 0.05)  # Safe distance boundary (5cm)
        self.declare_parameter('ik_damping', 1e-5)
        self.declare_parameter('max_ik_seeds', 7)
        self.declare_parameter('workspace_min', [-0.2, -0.5, 0.0])
        self.declare_parameter('workspace_max', [0.8, 0.5, 0.8])
        self.declare_parameter('workspace_frame', 'base_link')

        # Retrieve parameters
        urdf_path_param = self.get_parameter('urdf_path').get_parameter_value().string_value
        self.ik_dt = self.get_parameter('ik_dt').get_parameter_value().double_value
        self.ik_max_iterations = self.get_parameter('ik_max_iterations').get_parameter_value().integer_value
        self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').get_parameter_value().double_value
        self.min_collision_distance = self.get_parameter('min_collision_distance').get_parameter_value().double_value
        self.ik_damping = self.get_parameter('ik_damping').get_parameter_value().double_value
        self.max_ik_seeds = self.get_parameter('max_ik_seeds').get_parameter_value().integer_value

        self.workspace_min = np.array(self.get_parameter('workspace_min').get_parameter_value().double_array_value)
        self.workspace_max = np.array(self.get_parameter('workspace_max').get_parameter_value().double_array_value)
        self.workspace_frame = self.get_parameter('workspace_frame').get_parameter_value().string_value

        # Resolve URDF path and load Pinocchio model
        if urdf_path_param:
            urdf_path = Path(urdf_path_param)
        else:
            try:
                pkg_share = get_package_share_directory('tidybot_description')
                urdf_path = Path(pkg_share) / 'urdf' / 'tidybot_wx250s.urdf.xacro'
            except Exception:
                urdf_path = Path(__file__).resolve().parents[4] / 'src' / 'tidybot_description' / 'urdf' / 'tidybot_wx250s.urdf.xacro'

        if not urdf_path.exists():
            self.get_logger().error(f'URDF not found: {urdf_path}')
            raise FileNotFoundError(f'URDF not found: {urdf_path}')

        urdf_string = self._process_xacro(urdf_path)

        self.get_logger().info(f'Loading URDF: {urdf_path}')
        self.model = pin.buildModelFromXML(urdf_string)
        self.data = self.model.createData()

        # Define joint names mapping for left and right arms
        self.arm_joints = {
            'right': ['right_waist', 'right_shoulder', 'right_elbow',
                      'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate'],
            'left': ['left_waist', 'left_shoulder', 'left_elbow',
                     'left_forearm_roll', 'left_wrist_angle', 'left_wrist_rotate'],
        }

        # Cache joint IDs for faster access during IK loops
        self.joint_ids = {}
        for arm in ['right', 'left']:
            for jname in self.arm_joints[arm]:
                if self.model.existJointName(jname):
                    self.joint_ids[jname] = self.model.getJointId(jname)

        # Define end-effector frames
        self.ee_frames = {
            'right': 'right_ee_arm_link',
            'left': 'left_ee_arm_link',
        }

        # Cache end-effector frame IDs
        self.ee_frame_ids = {}
        for arm, frame_name in self.ee_frames.items():
            if self.model.existFrame(frame_name):
                self.ee_frame_ids[arm] = self.model.getFrameId(frame_name)
            else:
                alt_name = f'{arm}_pinch_site'
                if self.model.existFrame(alt_name):
                    self.ee_frame_ids[arm] = self.model.getFrameId(alt_name)

        # =========================================================================
        # [PORTED FROM SIMULATION] 1. Expanded Collision Bodies
        # Originally, the real node only checked a few arm links.
        # This has been updated to include gripper fingers, gripper bars, 
        # and the robot base/camera mounts for comprehensive collision checking.
        # =========================================================================
        self.collision_frames = {
            'right': [
                'right_upper_arm_link', 'right_upper_forearm_link', 'right_lower_forearm_link', 
                'right_wrist_link', 'right_gripper_link', 'right_gripper_bar_link', 
                'right_left_finger_link', 'right_right_finger_link'
            ],
            'left': [
                'left_upper_arm_link', 'left_upper_forearm_link', 'left_lower_forearm_link', 
                'left_wrist_link', 'left_gripper_link', 'left_gripper_bar_link', 
                'left_left_finger_link', 'left_right_finger_link'
            ],
            'base': [
                'base_link', 'camera_mount', 'pan_link', 'tilt_link', 'camera_link'
            ]
        }

        # Cache collision frame IDs
        self.collision_frame_ids = {}
        for group in ['right', 'left', 'base']:
            self.collision_frame_ids[group] = []
            for fname in self.collision_frames[group]:
                if self.model.existFrame(fname):
                    self.collision_frame_ids[group].append(self.model.getFrameId(fname))
                else:
                    self.get_logger().warn(f"Collision frame '{fname}' not found in URDF. Checking might be limited.")

        # Thread-safe storage for current joint positions
        self.current_joint_positions = {}
        self.joint_lock = Lock()

        # Publishers to send execution commands directly to xs_sdk
        self.arm_cmd_pubs = {
            'right': self.create_publisher(JointGroupCommand, '/right_arm/commands/joint_group', 10),
            'left': self.create_publisher(JointGroupCommand, '/left_arm/commands/joint_group', 10),
        }

        # Publisher and timer for RViz workspace visualization and real-time bounds checking
        self.workspace_marker_pub = self.create_publisher(Marker, 'workspace_marker', 10)
        self.create_timer(0.5, self.safety_timer_callback)

        # Subscribers for joint states
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(JointState, '/right_arm/joint_states', self.joint_state_callback, 10)
        self.create_subscription(JointState, '/left_arm/joint_states', self.joint_state_callback, 10)

        # Service server for the manipulation pipeline
        self.plan_service = self.create_service(PlanToTarget, '/plan_to_target', self.plan_to_target_callback)
        self.get_logger().info('Motion planner (real hardware) initialized with advanced safety checks.')

    def _process_xacro(self, xacro_path: Path) -> str:
        """Helper method to parse xacro files into standard URDF string."""
        if xacro_path.suffix == '.xacro':
            result = subprocess.run(['xacro', str(xacro_path)], capture_output=True, text=True, check=True)
            return result.stdout
        return xacro_path.read_text()

    def joint_state_callback(self, msg: JointState):
        """Callback to update the internal representation of the robot's current joint states."""
        with self.joint_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.current_joint_positions[name] = msg.position[i]

    def get_arm_joint_positions(self, arm_name: str, use_default_if_zero: bool = True) -> np.ndarray:
        """Retrieves the current joint positions for a specific arm, avoiding singular zero-states."""
        with self.joint_lock:
            positions = np.zeros(6)
            for i, jname in enumerate(self.arm_joints[arm_name]):
                positions[i] = self.current_joint_positions.get(jname, 0.0)
            if use_default_if_zero and np.allclose(positions, 0.0, atol=0.01):
                return self.DEFAULT_SEED.copy()
            return positions

    def set_arm_configuration(self, q: np.ndarray, arm_name: str, positions: np.ndarray):
        """Helper to inject specific arm joint positions into the full robot configuration vector 'q'."""
        for i, jname in enumerate(self.arm_joints[arm_name]):
            if jname in self.joint_ids:
                jid = self.joint_ids[jname]
                idx = self.model.joints[jid].idx_q
                q[idx] = positions[i]

    def numerical_jacobian(self, q: np.ndarray, arm_name: str, ee_frame_id: int, use_orientation: bool = False, eps: float = 1e-4) -> np.ndarray:
        """Computes the numerical Jacobian matrix for IK, bypassing issues with floating base analytical Jacobians."""
        arm_idx_q = [self.model.joints[self.joint_ids[jname]].idx_q for jname in self.arm_joints[arm_name] if jname in self.joint_ids]
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pos0 = self.data.oMf[ee_frame_id].translation.copy()
        R0 = self.data.oMf[ee_frame_id].rotation.copy() if use_orientation else None

        rows = 6 if use_orientation else 3
        J = np.zeros((rows, len(arm_idx_q)))
        for i, idx_q in enumerate(arm_idx_q):
            q_plus = q.copy()
            q_plus[idx_q] += eps
            pin.forwardKinematics(self.model, self.data, q_plus)
            pin.updateFramePlacements(self.model, self.data)
            pos_plus = self.data.oMf[ee_frame_id].translation.copy()
            J[:3, i] = (pos_plus - pos0) / eps
            if use_orientation:
                R_plus = self.data.oMf[ee_frame_id].rotation.copy()
                J[3:, i] = pin.log3(R0.T @ R_plus) / eps
        return J

    def get_arm_from_configuration(self, q: np.ndarray, arm_name: str) -> np.ndarray:
        """Extracts the joint positions of a specific arm from the full configuration vector 'q'."""
        positions = np.zeros(6)
        for i, jname in enumerate(self.arm_joints[arm_name]):
            if jname in self.joint_ids:
                jid = self.joint_ids[jname]
                idx = self.model.joints[jid].idx_q
                positions[i] = q[idx]
        return positions

    def pose_to_se3(self, pose: Pose) -> pin.SE3:
        """Converts ROS geometry_msgs/Pose to Pinocchio SE3 format."""
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = pin.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        return pin.SE3(quat.matrix(), position)

    def is_in_workspace(self, position: np.ndarray) -> bool:
        """Checks if a given 3D coordinate falls within the allowed workspace bounding box."""
        pos = np.asarray(position)
        return bool(np.all(pos >= self.workspace_min) and np.all(pos <= self.workspace_max))

    def solve_ik(self, arm_name: str, target_pose: pin.SE3, use_orientation: bool, seed: np.ndarray) -> tuple:
        """Solves Inverse Kinematics using Damped Least Squares (Closed-Loop IK) via Pinocchio."""
        q = pin.neutral(self.model)
        self.set_arm_configuration(q, arm_name, seed)
        other_arm = 'left' if arm_name == 'right' else 'right'
        other_positions = self.get_arm_joint_positions(other_arm)
        self.set_arm_configuration(q, other_arm, other_positions)

        ee_frame_id = self.ee_frame_ids.get(arm_name)
        if ee_frame_id is None:
            return False, seed, float('inf'), float('inf')

        arm_idx_q = [self.model.joints[self.joint_ids[jname]].idx_q for jname in self.arm_joints[arm_name] if jname in self.joint_ids]
        target_position, target_rotation = target_pose.translation, target_pose.rotation
        limits = list(self.JOINT_LIMITS.values())

        for iteration in range(self.ik_max_iterations):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current_pose = self.data.oMf[ee_frame_id]
            pos_error_vec = target_position - current_pose.translation
            pos_error = np.linalg.norm(pos_error_vec)

            if use_orientation:
                ori_error_vec = pin.log3(target_rotation.T @ current_pose.rotation)
                ori_error = np.linalg.norm(ori_error_vec)
                if pos_error < self.position_tolerance and ori_error < self.orientation_tolerance: break
                error_vec, J, dim = np.concatenate([pos_error_vec, -ori_error_vec]), self.numerical_jacobian(q, arm_name, ee_frame_id, True), 6
            else:
                if pos_error < self.position_tolerance: break
                error_vec, J, dim = pos_error_vec, self.numerical_jacobian(q, arm_name, ee_frame_id, False), 3

            JJT = J @ J.T + self.ik_damping * np.eye(dim)
            try: v = J.T @ np.linalg.solve(JJT, error_vec)
            except np.linalg.LinAlgError: break

            for i, idx_q in enumerate(arm_idx_q):
                q[idx_q] = np.clip(q[idx_q] + self.ik_dt * v[i], limits[i][0], limits[i][1])

        solution = self.get_arm_from_configuration(q, arm_name)
        for i, jname in enumerate(list(self.JOINT_LIMITS.keys())):
            solution[i] = np.clip(solution[i], self.JOINT_LIMITS[jname][0], self.JOINT_LIMITS[jname][1])

        self.set_arm_configuration(q, arm_name, solution)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        current_pose = self.data.oMf[ee_frame_id]

        position_error = np.linalg.norm(current_pose.translation - target_pose.translation)
        orientation_error = 0.0
        if use_orientation:
            cos_angle = np.clip((np.trace(target_pose.rotation.T @ current_pose.rotation) - 1) / 2, -1, 1)
            orientation_error = np.arccos(cos_angle)

        success = (position_error < self.position_tolerance and (not use_orientation or orientation_error < self.orientation_tolerance))
        return success, solution, position_error, orientation_error

    def compute_jacobian_condition(self, arm_name: str, joint_positions: np.ndarray) -> float:
        """Calculates the condition number of the Jacobian matrix to detect and avoid kinematic singularities."""
        q = pin.neutral(self.model)
        self.set_arm_configuration(q, arm_name, joint_positions)
        ee_frame_id = self.ee_frame_ids.get(arm_name)
        if ee_frame_id is None: return float('inf')

        J = self.numerical_jacobian(q, arm_name, ee_frame_id, use_orientation=True)
        try: return np.linalg.cond(J)
        except: return float('inf')

    def check_arm_collision(self, joint_positions_right: np.ndarray, joint_positions_left: np.ndarray) -> tuple:
        """
        =========================================================================
        [PORTED FROM SIMULATION] 2. Comprehensive Collision Check
        Calculates the minimum Euclidean distance between the center points of 
        all predefined collision frames. Checks:
        1. Right Arm vs Left Arm
        2. Right Arm vs Base/Mounts
        3. Left Arm vs Base/Mounts
        =========================================================================
        """
        q = pin.neutral(self.model)
        self.set_arm_configuration(q, 'right', joint_positions_right)
        self.set_arm_configuration(q, 'left', joint_positions_left)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        min_distance = float('inf')

        # Extract real-time cartesian positions for all defined frames
        pos_right = [self.data.oMf[fid].translation for fid in self.collision_frame_ids['right']]
        pos_left = [self.data.oMf[fid].translation for fid in self.collision_frame_ids['left']]
        pos_base = [self.data.oMf[fid].translation for fid in self.collision_frame_ids['base']]

        # 1. Right Arm vs Left Arm
        for pr in pos_right:
            for pl in pos_left:
                min_distance = min(min_distance, np.linalg.norm(pr - pl))

        # 2. Right Arm vs Base/Mounts
        for pr in pos_right:
            for pb in pos_base:
                min_distance = min(min_distance, np.linalg.norm(pr - pb))

        # 3. Left Arm vs Base/Mounts
        for pl in pos_left:
            for pb in pos_base:
                min_distance = min(min_distance, np.linalg.norm(pl - pb))

        collision_free = min_distance >= self.min_collision_distance
        return collision_free, min_distance

    def plan_to_target_callback(self, request, response):
        """Primary service callback to process movement requests, solve IK, validate safety, and execute."""
        arm_name = request.arm_name.lower()
        if arm_name not in ['right', 'left']:
            response.success, response.message = False, f"Invalid arm_name '{request.arm_name}'."
            return response

        mode = 'pos+orient' if request.use_orientation else 'pos-only'
        self.get_logger().info(f'Planning for {arm_name} arm ({mode})...')

        primary_seed = self.get_arm_joint_positions(arm_name)
        other_arm = 'left' if arm_name == 'right' else 'right'
        other_arm_positions = self.get_arm_joint_positions(other_arm)

        target_se3 = self.pose_to_se3(request.target_pose)
        
        # Abort if the target falls outside the safe workspace box
        if not self.is_in_workspace(target_se3.translation):
            response.success, response.message = False, 'Target out of workspace'
            self.get_logger().warn(response.message)
            return response

        # Prepare multi-seed list to avoid IK local minima
        seeds = [primary_seed]
        for extra in self.EXTRA_SEEDS:
            if arm_name == 'left':
                mirrored = extra.copy()
                mirrored[0] = -mirrored[0]
                mirrored[3] = -mirrored[3]
                seeds.append(mirrored)
            else:
                seeds.append(extra.copy())

        # Attempt to solve IK using multiple seeds, keep the result with the lowest error
        best_result, seeds_tried = None, 0
        for seed in seeds[:self.max_ik_seeds]:
            seeds_tried += 1
            ik_success, solution, pos_error, ori_error = self.solve_ik(arm_name, target_se3, request.use_orientation, seed)
            if ik_success:
                total_err = pos_error + ori_error
                if best_result is None or total_err < (best_result[1] + best_result[2]):
                    best_result = (solution, pos_error, ori_error)
                # Break early if the solution is exceptionally good
                if pos_error < self.position_tolerance * 0.5 and (not request.use_orientation or ori_error < self.orientation_tolerance * 0.5):
                    break

        if best_result is not None:
            solution, pos_error, ori_error = best_result
        else:
            # Fallback to primary seed if all fail to meet strict tolerances
            _, solution, pos_error, ori_error = self.solve_ik(arm_name, target_se3, request.use_orientation, primary_seed)

        response.position_error, response.orientation_error, response.joint_positions = pos_error, ori_error, solution.tolist()

        if best_result is None:
            response.success, response.message = False, f"IK failed: pos_err={pos_error:.4f}m, ori_err={ori_error:.4f}rad"
            self.get_logger().warn(response.message)
            return response

        self.get_logger().info('IK solution found. Checking singularity and collisions...')
        
        # Validate that the final configuration is far from kinematic singularities
        condition_number = self.compute_jacobian_condition(arm_name, solution)
        response.condition_number = condition_number

        max_cond = request.max_condition_number if hasattr(request, 'max_condition_number') and request.max_condition_number > 0 else 100.0
        if condition_number > max_cond:
            response.success, response.message = False, f"Near singularity: condition number={condition_number:.1f} > {max_cond}"
            self.get_logger().warn(response.message)
            return response

        self.get_logger().info('Configuration is not near singularity. Checking collisions...')
        
        # Static collision check at the final goal configuration
        if arm_name == 'right': collision_free, min_dist = self.check_arm_collision(solution, other_arm_positions)
        else: collision_free, min_dist = self.check_arm_collision(other_arm_positions, solution)

        if not collision_free:
            response.success, response.message = False, f"Arm collision detected: min distance={min_dist:.3f}m < {self.min_collision_distance}m"
            self.get_logger().warn(response.message)
            return response

        self.get_logger().info('No collisions at solution configuration. Validating path for collisions...')
        
        # =========================================================================
        # [PORTED FROM SIMULATION] 3. Path Validation
        # Slices the trajectory from current position to goal position into 20 steps.
        # Checks for collisions at every interpolated waypoint to prevent the arm
        # from swinging through an obstacle (or the other arm) to reach a safe goal.
        # =========================================================================
        steps = 20
        start_joints = self.get_arm_joint_positions(arm_name)
        goal_joints = solution.copy()
        
        for i in range(1, steps + 1):
            alpha = i / steps
            interp_joints = (1 - alpha) * start_joints + alpha * goal_joints
            
            if arm_name == 'right':
                path_collision_free, path_min_dist = self.check_arm_collision(interp_joints, other_arm_positions)
            else:
                path_collision_free, path_min_dist = self.check_arm_collision(other_arm_positions, interp_joints)

            if not path_collision_free:
                response.success = False
                response.message = f"Collision detected along path at step {i}/{steps}: min distance={path_min_dist:.3f}m < {self.min_collision_distance}m"
                self.get_logger().warn(response.message)
                return response

        self.get_logger().info('Path is collision-free. Planning successful...')

        response.success = True
        response.message = f"Planning succeeded: pos_err={pos_error:.4f}m, cond={condition_number:.1f}, min_dist={min_dist:.3f}m"
        
        # Dispatch hardware commands if requested
        if request.execute:
            # [MODIFIED] Spawning a background thread to prevent ROS 2 executor blocking during time.sleep()
            exec_thread = threading.Thread(
                target=self.execute_trajectory,
                args=(arm_name, solution, request.duration),
                daemon=True
            )
            exec_thread.start()
            response.executed = True
            self.get_logger().info(f'Started executing motion over {request.duration}s in background thread.')
        else:
            response.executed = False

        return response

    def execute_trajectory(self, arm_name: str, target: np.ndarray, duration: float):
        """Generates a smooth cosine-interpolated trajectory and streams commands to the real hardware."""
        start = self.get_arm_joint_positions(arm_name, use_default_if_zero=False)
        rate_hz, dt = 50.0, 1.0 / 50.0
        num_steps = max(int(duration * rate_hz), 1)

        self.get_logger().info(f'Executing {arm_name} trajectory: {num_steps} steps over {duration}s')
        for i in range(num_steps + 1):
            t = i / num_steps
            # Apply smooth start/stop easing (cosine profile)
            alpha = 0.5 * (1 - np.cos(np.pi * t))
            q = start + alpha * (target - start)

            cmd = JointGroupCommand()
            # [KEPT ORIGINAL] Retained your original custom namespace structure for the real hardware
            cmd.name = f'{arm_name}_arm'  
            cmd.cmd = q.tolist()
            self.arm_cmd_pubs[arm_name].publish(cmd)
            
            if i < num_steps: 
                time.sleep(dt)

    def publish_workspace_marker(self):
        """Publishes an RViz marker to visually display the safe workspace bounding box."""
        marker = Marker()
        marker.header.frame_id = self.workspace_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = 'workspace', 0, Marker.CUBE, Marker.ADD
        center = (self.workspace_min + self.workspace_max) / 2.0
        size = (self.workspace_max - self.workspace_min)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(center[0]), float(center[1]), float(center[2])
        marker.scale.x, marker.scale.y, marker.scale.z = float(size[0]), float(size[1]), float(size[2])
        marker.color.g, marker.color.a = 1.0, 0.3
        self.workspace_marker_pub.publish(marker)

    def safety_timer_callback(self):
        """Continuous background loop that monitors if either end-effector escapes the safe workspace."""
        for arm in ['right', 'left']:
            ee_frame_id = self.ee_frame_ids.get(arm)
            if ee_frame_id is None: continue
            q = pin.neutral(self.model)
            self.set_arm_configuration(q, arm, self.get_arm_joint_positions(arm, use_default_if_zero=False))
            other_arm = 'left' if arm == 'right' else 'right'
            self.set_arm_configuration(q, other_arm, self.get_arm_joint_positions(other_arm, use_default_if_zero=False))
            try:
                pin.forwardKinematics(self.model, self.data, q)
                pin.updateFramePlacements(self.model, self.data)
            except Exception: continue

            ee_pos = self.data.oMf[ee_frame_id].translation.copy()
            if not self.is_in_workspace(ee_pos):
                self.get_logger().warn(f'{arm.capitalize()} end-effector out of workspace: {ee_pos}')
                
        # [MODIFIED] Corrected indentation for try-except block
        try: 
            self.publish_workspace_marker()
        except Exception as e: 
            self.get_logger().error(f'Failed to publish workspace marker: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerRealNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()