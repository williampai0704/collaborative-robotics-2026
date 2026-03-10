#!/usr/bin/env python3
"""
Task 1: Object Search, Centre, and Pose Hand-off

Orchestrates the existing motion and vision modules via ROS2 topics:

  Vision nodes expected to be running:
    find_center.py       → publishes /mask_center (geometry_msgs/Point)
    simple_pose_fit.py   → publishes /object_pose (geometry_msgs/PoseStamped)

  Motion node expected to be running:
    test-merge.py        → subscribes /motion_command, /stop_motion
                         → publishes  /cmd_vel

  Manipulation node expected to be running:
    motion_planner_node  → serves /plan_to_target (tidybot_msgs/PlanToTarget)

  This node:
    Subscribes : /mask_center       — pixel centroid of the detected object
                 /object_pose       — 6-DOF pose in camera_depth_optical_frame
    Publishes  : /motion_command    — "search" to spin, used with /stop_motion
                 /stop_motion       — Bool True to halt the motion module
                 /camera/pan_tilt_cmd — [pan, tilt] to centre object vertically
    Calls      : /plan_to_target    — motion planner service (PlanToTarget)

Pipeline (state machine):
  INIT → SPINNING → WAIT_POSE → SEND_TO_MANIPULATION → DONE

Usage:
    # Terminal 1 – motion module
    ros2 run tidybot_bringup test-merge.py

    # Terminal 2 – vision: object centre detector
    ros2 run tidybot_bringup find_center.py --ros-args -p target_color:=yellow

    # Terminal 3 – vision: pose estimator
    ros2 run tidybot_bringup simple_pose_fit.py --ros-args -p target_color:=yellow

    # Terminal 4 – motion planner
    ros2 run tidybot_ik motion_planner_node

    # Terminal 5 – this orchestrator
    ros2 run tidybot_bringup task_1.py
"""

import time
from enum import Enum, auto
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros

from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, String, Float64MultiArray

from scipy.spatial.transform import Rotation as ScipyRotation

from tidybot_msgs.srv import PlanToTarget

# =============================================================================
# Configuration
# =============================================================================

IMAGE_WIDTH    = 640
IMAGE_HEIGHT   = 480
CENTER_X       = IMAGE_WIDTH  / 2.0   # 320 px
CENTER_Y       = IMAGE_HEIGHT / 2.0   # 240 px

# Tolerances for "centred" decision
HORIZONTAL_TOL = 50    # px — stop spinning when |cx − CENTER_X| < this
VERTICAL_TOL   = 60    # px — stop tilting  when |cy − CENTER_Y| < this

# Camera tilt parameters
TILT_GAIN = 0.001      # rad / px
TILT_MIN  = -1.2       # rad  (tilt down)
TILT_MAX  =  0.5       # rad  (tilt up)

# How long to wait for a fresh /object_pose after stopping
POSE_WAIT_TIMEOUT = 8.0   # s
SETTLE_TIME       = 2.0   # s — wait for robot to stop moving before accepting poses
NUM_POSE_SAMPLES  = 10   # number of pose samples to average

# How long to wait for /plan_to_target service result (includes execution time)
RESULT_TIMEOUT = 30.0   # s

MAX_GRASP_RETRIES = 3   # re-attempt pick if grasped=False
GRASP_Z = 0.07          # fixed grasp height in base_link frame (m)

# =============================================================================
# States
# =============================================================================

class State(Enum):
    INIT                 = auto()  # wait for first /mask_center message
    SPINNING             = auto()  # command motion module to spin; watch mask centre
    WAIT_POSE            = auto()  # wait for a fresh /object_pose from simple_pose_fit
    SEND_TO_MANIPULATION = auto()  # call /plan_to_target with pose in base_link
    DONE                 = auto()

# =============================================================================
# Task 1 Orchestrator
# =============================================================================

class Task1Node(Node):

    def __init__(self):
        super().__init__('task_1')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('arm_name', 'left')
        self.arm_name: str = self.get_parameter('arm_name').value

        # ── Callback groups ──────────────────────────────────────────────────
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        # ── Publishers ──────────────────────────────────────────────────────
        # Motion module interface
        self.motion_cmd_pub = self.create_publisher(
            String, '/motion_command', 10)
        self.stop_motion_pub = self.create_publisher(
            Bool, '/stop_motion', 10)

        # Camera pan-tilt (direct hardware topic)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # ── Service client ───────────────────────────────────────────────────
        self.plan_client = self.create_client(
            PlanToTarget, '/plan_to_target',
            callback_group=self.client_cb_group)

        # ── Subscribers ─────────────────────────────────────────────────────
        # From find_center node – pixel centroid of colour-masked object
        self.create_subscription(
            Point, '/mask_center', self._mask_center_cb, 10)

        # From simple_pose_fit node – 6-DOF object pose in camera frame
        self.create_subscription(
            PoseStamped, '/object_pose', self._object_pose_cb, 10)

        # ── TF2 ─────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Shared state ─────────────────────────────────────────────────────
        self.mask_center: Optional[Point]      = None  # latest from find_center
        self.object_pose: Optional[PoseStamped] = None  # latest from simple_pose_fit
        self.pose_timestamp: Optional[float]   = None  # wall-clock of last pose

        self._cmd_pan:  float = 0.0
        self._cmd_tilt: float = 0.0

        self._pending_future = None   # async service call future
        self._grasp_retries  = 0
        self._pose_samples: list = []  # collected pose samples for averaging

        # ── State machine ────────────────────────────────────────────────────
        self.state       = State.SPINNING
        self.state_start = time.time()

        self.create_timer(0.05, self._loop)   # 20 Hz control loop

        self.get_logger().info('=' * 55)
        self.get_logger().info('Task 1: Spin → Find → Centre → Pose → Manipulate')
        self.get_logger().info(f'Arm: {self.arm_name}')
        self.get_logger().info('Waiting for /mask_center from find_center node …')
        self.get_logger().info('=' * 55)

    # =========================================================================
    # Topic callbacks
    # =========================================================================

    def _mask_center_cb(self, msg: Point):
        """Receive pixel centroid of the detected object from find_center."""
        self.mask_center = msg

    def _object_pose_cb(self, msg: PoseStamped):
        """Receive 6-DOF pose in camera_depth_optical_frame from simple_pose_fit."""
        self.object_pose   = msg
        self.pose_timestamp = time.time()

    # =========================================================================
    # Helpers
    # =========================================================================

    def _send_motion_command(self, command: str):
        msg = String()
        msg.data = command
        self.motion_cmd_pub.publish(msg)

    def _stop_motion(self):
        msg = Bool()
        msg.data = True
        self.stop_motion_pub.publish(msg)

    def _set_pan_tilt(self, pan: float, tilt: float):
        tilt = float(np.clip(tilt, TILT_MIN, TILT_MAX))
        self._cmd_pan  = float(pan)
        self._cmd_tilt = tilt
        msg = Float64MultiArray()
        msg.data = [self._cmd_pan, self._cmd_tilt]
        self.pan_tilt_pub.publish(msg)

    def _transition(self, new_state: State):
        self.get_logger().info(f'[State] {self.state.name} → {new_state.name}')
        self.state       = new_state
        self.state_start = time.time()

    def _elapsed(self) -> float:
        return time.time() - self.state_start

    def _average_poses(self, poses: list) -> PoseStamped:
        """Average multiple PoseStamped messages (position mean + quaternion mean)."""
        positions = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                              for p in poses])
        quats = np.array([[p.pose.orientation.x, p.pose.orientation.y,
                           p.pose.orientation.z, p.pose.orientation.w]
                          for p in poses])

        mean_pos = positions.mean(axis=0)

        # Flip quaternions to same hemisphere before averaging
        for i in range(1, len(quats)):
            if np.dot(quats[i], quats[0]) < 0:
                quats[i] = -quats[i]
        mean_quat = quats.mean(axis=0)
        mean_quat /= np.linalg.norm(mean_quat)

        out = PoseStamped()
        out.header = poses[-1].header
        out.pose.position.x = float(mean_pos[0])
        out.pose.position.y = float(mean_pos[1])
        out.pose.position.z = float(mean_pos[2])
        out.pose.orientation.x = float(mean_quat[0])
        out.pose.orientation.y = float(mean_quat[1])
        out.pose.orientation.z = float(mean_quat[2])
        out.pose.orientation.w = float(mean_quat[3])
        return out

    def _transform_to_base(self, pose_cam: PoseStamped) -> Optional[PoseStamped]:
        """Transform PoseStamped from camera_depth_optical_frame to base_link."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_color_optical_frame',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None

        # Build 4×4 T_cam→base from the TF transform
        tr = tf.transform.translation
        ro = tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = ScipyRotation.from_quat([ro.x, ro.y, ro.z, ro.w]).as_matrix()
        T[:3, 3]  = [tr.x, tr.y, tr.z]

        # Build 4×4 pose in camera frame from the incoming PoseStamped
        p = pose_cam.pose.position
        q = pose_cam.pose.orientation
        P_cam = np.eye(4)
        P_cam[:3, :3] = ScipyRotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        P_cam[:3, 3]  = [p.x, p.y, p.z]

        P_base = T @ P_cam

        out = PoseStamped()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.pose.position.x = float(P_base[0, 3]) -0.015
        out.pose.position.y = float(P_base[1, 3]) -0.025
        out.pose.position.z = float(P_base[2, 3])
        qb = ScipyRotation.from_matrix(P_base[:3, :3]).as_quat()
        out.pose.orientation.x = float(qb[0])
        out.pose.orientation.y = float(qb[1])
        out.pose.orientation.z = float(qb[2])
        out.pose.orientation.w = float(qb[3])
        return out

    # =========================================================================
    # State machine
    # =========================================================================

    def _loop(self):
        elapsed = self._elapsed()

        # ── INIT ─────────────────────────────────────────────────────────────
        # Wait until find_center is live (we're receiving /mask_center)
        if self.state == State.INIT:
            if self.mask_center is not None:
                self._set_pan_tilt(0.0, 0.0)   # camera centred
                self.get_logger().info('Vision module online – starting spin search')
                self._transition(State.SPINNING)
            elif elapsed > 15.0:
                self.get_logger().error(
                    'Timed out waiting for /mask_center. '
                    'Is find_center.py running?')

        # ── SPINNING ─────────────────────────────────────────────────────────
        # Ask the motion module to spin; stop when object centred horizontally.
        elif self.state == State.SPINNING:
            # Keep issuing "search" so the motion module doesn't time out
            self._send_motion_command('search')

            mc = self.mask_center
            if mc is None:
                return   # object not yet visible

            h_err = mc.x - CENTER_X
            self.get_logger().info(
                f'[SPIN] mask_center=({mc.x:.0f},{mc.y:.0f})  '
                f'h_err={h_err:+.0f} px',
                throttle_duration_sec=0.5,
            )

            if abs(h_err) < HORIZONTAL_TOL:
                self._stop_motion()
                self.get_logger().info(
                    f'Horizontally centred (cx={mc.x:.0f}, err={h_err:+.0f} px)')
                self.pose_timestamp = None
                self._pose_samples.clear()
                self._transition(State.WAIT_POSE)

        # ── WAIT_POSE ─────────────────────────────────────────────────────────
        # Wait for robot to settle, then collect multiple pose samples and
        # average them for a more robust estimate.
        elif self.state == State.WAIT_POSE:
            # Phase 1: let the robot settle after stopping
            if elapsed < SETTLE_TIME:
                self.get_logger().info(
                    f'Settling … ({elapsed:.1f} / {SETTLE_TIME:.0f} s)',
                    throttle_duration_sec=0.5)
                return

            # Phase 2: collect pose samples
            if self.pose_timestamp is not None and \
               self.pose_timestamp >= self.state_start + SETTLE_TIME:
                # New pose arrived after settle period — record it
                if len(self._pose_samples) == 0 or \
                   self.pose_timestamp > self._pose_samples[-1][0]:
                    self._pose_samples.append(
                        (self.pose_timestamp, self.object_pose))
                    self.get_logger().info(
                        f'Pose sample {len(self._pose_samples)}/{NUM_POSE_SAMPLES}')

            if len(self._pose_samples) >= NUM_POSE_SAMPLES:
                self.object_pose = self._average_poses(
                    [s[1] for s in self._pose_samples])
                self._pose_samples.clear()
                self.get_logger().info('Averaged pose computed')
                self._transition(State.SEND_TO_MANIPULATION)
                return

            if elapsed > POSE_WAIT_TIMEOUT + SETTLE_TIME:
                if len(self._pose_samples) > 0:
                    self.get_logger().warn(
                        f'Timed out with {len(self._pose_samples)} samples '
                        f'– using partial average')
                    self.object_pose = self._average_poses(
                        [s[1] for s in self._pose_samples])
                    self._pose_samples.clear()
                    self._transition(State.SEND_TO_MANIPULATION)
                else:
                    self.get_logger().error(
                        'Timed out waiting for /object_pose. '
                        'Is simple_pose_fit.py running?')
                    self._transition(State.DONE)
            elif int(elapsed * 2) % 2 == 0:
                self.get_logger().info(
                    f'Waiting for pose … ({elapsed:.1f} / '
                    f'{POSE_WAIT_TIMEOUT + SETTLE_TIME:.0f} s, '
                    f'samples: {len(self._pose_samples)}/{NUM_POSE_SAMPLES})',
                    throttle_duration_sec=1.0)

        # ── SEND_TO_MANIPULATION ──────────────────────────────────────────────
        # Transform pose from camera frame to base_link and call /plan_to_target.
        elif self.state == State.SEND_TO_MANIPULATION:

            # ── Step 1: send service request once on entry ────────────────────
            if self._pending_future is None and elapsed < 0.2:
                pose_base = self._transform_to_base(self.object_pose)
                if pose_base is None:
                    self.get_logger().error('TF transform failed – aborting')
                    self._transition(State.DONE)
                    return

                if not self.plan_client.service_is_ready():
                    self.get_logger().warn(
                        'Waiting for /plan_to_target service …',
                        throttle_duration_sec=1.0)
                    return

                req = PlanToTarget.Request()
                req.arm_name             = self.arm_name
                req.mode                 = 'pick'
                req.target_pose          = pose_base.pose
                req.target_pose.position.z = GRASP_Z
                req.use_orientation      = False
                req.max_condition_number = 500.0
                req.execute              = True
                req.duration             = 5.0

                self._pending_future = self.plan_client.call_async(req)

                p = pose_base.pose.position
                q = pose_base.pose.orientation
                self.get_logger().info('=' * 55)
                self.get_logger().info('Calling /plan_to_target')
                self.get_logger().info(f'  Arm        : {self.arm_name}')
                self.get_logger().info(f'  Frame      : base_link')
                self.get_logger().info(
                    f'  Position   : ({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) m')
                self.get_logger().info(
                    f'  Orientation: ({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})')
                self.get_logger().info('=' * 55)
                return

            # ── Step 2: poll future until done ────────────────────────────────
            if self._pending_future is not None:
                if not self._pending_future.done():
                    if elapsed > RESULT_TIMEOUT:
                        self.get_logger().error(
                            '/plan_to_target service timed out')
                        self._pending_future = None
                        self._transition(State.DONE)
                    return

                result = self._pending_future.result()
                self._pending_future = None

                if result.success and result.grasped:
                    self.get_logger().info(
                        f'Pick succeeded: {result.message}')
                    self._grasp_retries = 0
                    self._transition(State.DONE)
                elif result.success and not result.grasped:
                    self._grasp_retries += 1
                    if self._grasp_retries < MAX_GRASP_RETRIES:
                        self.get_logger().warn(
                            f'Grasp failed (attempt {self._grasp_retries}/'
                            f'{MAX_GRASP_RETRIES}) – retrying')
                        self._transition(State.SEND_TO_MANIPULATION)
                    else:
                        self.get_logger().error(
                            f'Grasp failed after {MAX_GRASP_RETRIES} attempts')
                        self._grasp_retries = 0
                        self._transition(State.DONE)
                else:
                    self.get_logger().error(
                        f'Motion planning failed: {result.message}')
                    self._transition(State.DONE)

        # ── DONE ─────────────────────────────────────────────────────────────
        elif self.state == State.DONE:
            if elapsed < 0.1:
                self.get_logger().info('Task 1 complete.')


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = Task1Node()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
