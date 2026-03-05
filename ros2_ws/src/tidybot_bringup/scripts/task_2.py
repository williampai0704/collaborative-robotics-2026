#!/usr/bin/env python3
"""
Task 2: Pick up a colored block and place it in a colored bin.

This script is SELF-CONTAINED — it automatically launches and manages all
vision nodes internally. You do NOT need to start find_center, obj_dist, or
simple_pose_fit manually.

─── Prerequisites (must already be running) ───────────────────────────────────
1. Real robot hardware:
       ros2 launch tidybot_bringup real.launch.py

2. Motion planner:
       ros2 run tidybot_ik motion_planner_real_node

─── Run task_2 ────────────────────────────────────────────────────────────────
Default (yellow block → blue bin, left arm):
    ros2 run tidybot_bringup task_2.py

Custom colors / arm:
    ros2 run tidybot_bringup task_2.py \
        --ros-args -p block_color:=yellow -p bin_color:=blue -p arm_name:=left

For Speech:
    ros2 run tidybot_bringup task_2.py \
        --ros-args -p use_speech:=true

Available colors:  red, green, blue, yellow
Available arms:    left, right

─── What this script does automatically ───────────────────────────────────────
Phase 1 — Find & Pick block (block_color):
  • Launches find_center, obj_dist, simple_pose_fit with target_color=block_color
  • Spins robot until block is centred, approaches, estimates pose
  • Calls /plan_to_target (mode=pick) — arm opens gripper, descends, grasps,
    returns home; retries up to 3 times if grasp fails

Phase 2 — Find & Place in bin (bin_color):
  • Kills Phase 1 vision nodes, relaunches with target_color=bin_color
  • Spins robot until bin is centred, approaches, estimates pose
  • Calls /plan_to_target (mode=place) — arm descends with block, opens
    gripper to release, returns home

─── State machine ─────────────────────────────────────────────────────────────
  INIT
    ↓ (vision nodes online)
  FIND_BLOCK       spin until block centred horizontally
    ↓ (h_err < HORIZONTAL_TOL)
  APPROACH_BLOCK   drive forward with heading correction; stop at APPROACH_DIST
    ↑ (drift > RECENTER_TOL)  → back to FIND_BLOCK
    ↓ (distance ≤ 0.50 m)
  TILT_BLOCK       adjust camera tilt for vertical centering
    ↓ (v_err < VERTICAL_TOL)
  WAIT_BLOCK_POSE  wait for fresh /object_pose from simple_pose_fit
    ↓
  GRASP            call /plan_to_target (mode=pick); retry up to 3x if grasped=False
    ↓ (success + grasped)
  FIND_BIN         relaunch vision nodes with bin_color; spin to find bin
    ↓
  APPROACH_BIN     same approach logic as block
    ↑ (drift)      → back to FIND_BIN
    ↓
  TILT_BIN         centre bin vertically
    ↓
  WAIT_BIN_POSE    wait for fresh /object_pose
    ↓
  PLACE            call /plan_to_target (mode=place)
    ↓
  DONE / ERROR
"""

import time
import subprocess
from enum import Enum, auto
from typing import Optional, List

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros

from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Float32, String, Float64MultiArray

from scipy.spatial.transform import Rotation as ScipyRotation

from tidybot_msgs.srv import PlanToTarget

# =============================================================================
# Configuration
# =============================================================================

IMAGE_WIDTH   = 640
IMAGE_HEIGHT  = 480
CENTER_X      = IMAGE_WIDTH  / 2.0
CENTER_Y      = IMAGE_HEIGHT / 2.0

HORIZONTAL_TOL  = 50    # px  — stop spinning: |cx − CENTER_X| < this
RECENTER_TOL    = 120   # px  — stop approach and re-center if drift > this
VERTICAL_TOL    = 60    # px  — stop tilting: |cy − CENTER_Y| < this

APPROACH_DIST   = 0.50  # m   — stop driving when object is this close
APPROACH_SPEED  = 0.06  # m/s — forward speed during approach
SPIN_SPEED      = 0.3   # rad/s
ANGULAR_GAIN    = 0.003 # rad/s per px of horizontal error (heading correction)

TILT_GAIN       = 0.001 # rad / px
TILT_MIN        = -1.2  # rad
TILT_MAX        =  0.5  # rad

POSE_WAIT_TIMEOUT  = 8.0   # s — additional wait after settle for pose samples
SETTLE_TIME        = 2.0   # s — wait for robot to stop moving before accepting poses
NUM_POSE_SAMPLES   = 10    # number of pose samples to average for robust estimate
RESULT_TIMEOUT     = 30.0  # s — max wait for /plan_to_target result (includes execution)
MAX_GRASP_RETRIES  = 3
GRASP_Z            = 0.07  # fixed grasp/place height in base_link frame (m)

# =============================================================================
# States
# =============================================================================

class State(Enum):
    WAIT_SPEECH    = auto()   # (optional) wait for /speech_extraction colors
    # ── Block ─────────────────────────────
    FIND_BLOCK     = auto()   # spin to centre block horizontally
    APPROACH_BLOCK = auto()   # drive forward; stop when close enough
    TILT_BLOCK     = auto()   # centre block vertically in camera
    WAIT_BLOCK_POSE= auto()   # wait for fresh /object_pose
    GRASP          = auto()   # call /plan_to_target for grasp; wait for result
    # ── Bin ───────────────────────────────
    FIND_BIN       = auto()   # spin to centre bin horizontally
    APPROACH_BIN   = auto()   # drive forward; stop when close enough
    TILT_BIN       = auto()   # centre bin vertically in camera
    WAIT_BIN_POSE  = auto()   # wait for fresh /object_pose
    PLACE          = auto()   # call /plan_to_target for place; wait for result
    # ─────────────────────────────────────
    DONE           = auto()
    ERROR          = auto()

# =============================================================================
# Task 2 Node
# =============================================================================

class Task2Node(Node):

    def __init__(self):
        super().__init__('task_2')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('block_color', 'yellow')
        self.declare_parameter('bin_color',   'blue')
        self.declare_parameter('arm_name',    'left')
        self.declare_parameter('use_speech',  False)

        self.block_color: str  = self.get_parameter('block_color').value
        self.bin_color:   str  = self.get_parameter('bin_color').value
        self.arm_name:    str  = self.get_parameter('arm_name').value
        self.use_speech:  bool = self.get_parameter('use_speech').value

        # ── Callback groups ──────────────────────────────────────────────────
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        # ── Publishers ──────────────────────────────────────────────────────
        self.cmd_vel_pub  = self.create_publisher(Twist,             '/cmd_vel',              10)
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd',  10)
        self.color_pub    = self.create_publisher(String,            '/vision/target_color',  10)

        # ── Service client ───────────────────────────────────────────────────
        self.plan_client = self.create_client(
            PlanToTarget, '/plan_to_target',
            callback_group=self.client_cb_group)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Point,       '/mask_center',            self._mask_center_cb, 10)
        self.create_subscription(Float32,     '/vision/object_distance', self._obj_dist_cb,    10)
        self.create_subscription(PoseStamped, '/object_pose',            self._object_pose_cb, 10)

        # ── TF2 ─────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Shared state ─────────────────────────────────────────────────────
        self.mask_center:     Optional[Point]       = None
        self.object_distance: Optional[float]       = None
        self.object_pose:     Optional[PoseStamped] = None
        self.pose_timestamp:  Optional[float]       = None

        self._cmd_pan:  float = 0.0
        self._cmd_tilt: float = 0.0
        self._grasp_retries:  int = 0

        self._pending_future = None   # async /plan_to_target future
        self._pose_samples: list = []  # collected pose samples for averaging
        self._vision_procs: List[subprocess.Popen] = []
        self._speech_proc:  Optional[subprocess.Popen] = None

        # ── Optional speech subscription ─────────────────────────────────────
        if self.use_speech:
            from tidybot_msgs.msg import SpeechExtraction
            self.create_subscription(
                SpeechExtraction, '/speech_extraction', self._speech_cb, 10)

        # ── State machine ────────────────────────────────────────────────────
        if self.use_speech:
            self.state = State.WAIT_SPEECH
            # Launch speech extraction node (mic mode)
            self._speech_proc = subprocess.Popen([
                'ros2', 'run', 'tidybot_bringup', 'speech_extraction_node.py',
            ])
        else:
            self.state = State.FIND_BLOCK
            self._start_vision_nodes(self.block_color)

        self.state_start = time.time()
        self.create_timer(0.05, self._loop)  # 20 Hz

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Task 2: Pick [{self.block_color}] block → [{self.bin_color}] bin')
        self.get_logger().info(f'Arm: {self.arm_name}  |  use_speech: {self.use_speech}')
        if self.use_speech:
            self.get_logger().info('Waiting for voice command...')
        self.get_logger().info('=' * 60)

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _speech_cb(self, msg):
        """Receive extracted colors from speech_extraction_node."""
        if self.state != State.WAIT_SPEECH:
            return
        if not msg.success:
            self.get_logger().warn(
                f'Speech extraction failed: {msg.error_message} — still waiting...')
            return
        self.block_color = msg.object_color
        self.bin_color   = msg.bin_color
        self.get_logger().info(
            f'[Speech] block_color={self.block_color}  bin_color={self.bin_color}')
        # Kill speech node and start vision nodes for block detection
        if self._speech_proc is not None:
            self._speech_proc.terminate()
            self._speech_proc = None
        self._start_vision_nodes(self.block_color)
        self._transition(State.FIND_BLOCK)

    def _mask_center_cb(self, msg: Point):
        self.mask_center = msg

    def _obj_dist_cb(self, msg: Float32):
        self.object_distance = float(msg.data)

    def _object_pose_cb(self, msg: PoseStamped):
        self.object_pose   = msg
        self.pose_timestamp = time.time()

    # =========================================================================
    # Helpers
    # =========================================================================

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())

    def _spin(self):
        t = Twist()
        t.angular.z = SPIN_SPEED
        self.cmd_vel_pub.publish(t)

    def _drive(self, h_err: float):
        """Move forward with proportional heading correction from horizontal error."""
        t = Twist()
        t.linear.x  = APPROACH_SPEED
        t.angular.z = float(np.clip(-h_err * ANGULAR_GAIN, -0.5, 0.5))
        self.cmd_vel_pub.publish(t)

    def _set_pan_tilt(self, pan: float, tilt: float):
        tilt = float(np.clip(tilt, TILT_MIN, TILT_MAX))
        self._cmd_pan  = float(pan)
        self._cmd_tilt = tilt
        msg = Float64MultiArray()
        msg.data = [self._cmd_pan, self._cmd_tilt]
        self.pan_tilt_pub.publish(msg)

    def _start_vision_nodes(self, color: str):
        """Stop any running vision nodes and relaunch with new target color."""
        self._stop_vision_nodes()
        scripts = ['find_center.py', 'obj_dist.py', 'simple_pose_fit.py']
        for script in scripts:
            proc = subprocess.Popen([
                'ros2', 'run', 'tidybot_bringup', script,
                '--ros-args', '-p', f'target_color:={color}', '-p', 'visualize:=true',
            ])
            self._vision_procs.append(proc)
        self.get_logger().info(f'[Vision] Launched {len(scripts)} nodes with target_color={color}')

    def _stop_vision_nodes(self):
        """Terminate all managed vision node processes."""
        for proc in self._vision_procs:
            try:
                proc.terminate()
                proc.wait(timeout=3.0)
            except Exception:
                proc.kill()
        self._vision_procs = []

    def _set_target_color(self, color: str):
        """Restart vision nodes with new target color and notify via topic."""
        msg = String()
        msg.data = color
        self.color_pub.publish(msg)
        # Clear stale detection data from the previous color
        self.mask_center    = None
        self.object_pose    = None
        self.pose_timestamp = None
        self._start_vision_nodes(color)

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

    def _transition(self, new_state: State):
        self.get_logger().info(f'[State] {self.state.name} → {new_state.name}')
        self.state       = new_state
        self.state_start = time.time()
        self._pending_future = None   # clear any pending service call on transition
        self._pose_samples.clear()   # clear stale samples from previous wait

    def _elapsed(self) -> float:
        return time.time() - self.state_start

    def _transform_to_base(self, pose_cam: PoseStamped) -> Optional[PoseStamped]:
        """Transform PoseStamped from camera_depth_optical_frame → base_link."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'camera_depth_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None

        tr, ro = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = ScipyRotation.from_quat([ro.x, ro.y, ro.z, ro.w]).as_matrix()
        T[:3, 3]  = [tr.x, tr.y, tr.z]

        p, q = pose_cam.pose.position, pose_cam.pose.orientation
        P = np.eye(4)
        P[:3, :3] = ScipyRotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        P[:3, 3]  = [p.x, p.y, p.z]

        R = T @ P
        out = PoseStamped()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.pose.position.x = float(R[0, 3])
        out.pose.position.y = float(R[1, 3])
        out.pose.position.z = float(R[2, 3])
        qb = ScipyRotation.from_matrix(R[:3, :3]).as_quat()
        out.pose.orientation.x = float(qb[0])
        out.pose.orientation.y = float(qb[1])
        out.pose.orientation.z = float(qb[2])
        out.pose.orientation.w = float(qb[3])
        return out

    def _call_plan_to_target(self, pose_base: PoseStamped,
                              mode: str = 'pick',
                              duration: float = 5.0) -> bool:
        """
        Build and send an async /plan_to_target request.
        Returns True if the request was sent, False if the service isn't ready.
        """
        if not self.plan_client.service_is_ready():
            self.get_logger().warn(
                'Waiting for /plan_to_target service …',
                throttle_duration_sec=1.0)
            return False

        req = PlanToTarget.Request()
        req.arm_name                = self.arm_name
        req.mode                    = mode
        req.target_pose             = pose_base.pose
        req.target_pose.position.z  = GRASP_Z
        req.use_orientation         = True
        req.max_condition_number    = 100.0
        req.execute                 = True
        req.duration                = duration

        self._pending_future = self.plan_client.call_async(req)
        return True

    # =========================================================================
    # Shared sub-behaviours
    # =========================================================================

    def _do_find(self, next_state: State):
        """Generic spin-until-centred logic used by both FIND_BLOCK and FIND_BIN."""
        mc = self.mask_center
        if mc is None:
            self._spin()
            return

        h_err = mc.x - CENTER_X
        self.get_logger().info(
            f'[{self.state.name}] cx={mc.x:.0f}  h_err={h_err:+.0f} px',
            throttle_duration_sec=0.5)

        if abs(h_err) < HORIZONTAL_TOL:
            self._stop()
            self.get_logger().info(
                f'Centred (h_err={h_err:+.0f} px) → {next_state.name}')
            self._transition(next_state)
        else:
            self._spin()

    def _do_approach(self, find_state: State, tilt_state: State):
        """Generic approach logic: drive forward while correcting heading."""
        mc   = self.mask_center
        dist = self.object_distance

        if mc is None:
            self._stop()
            self.get_logger().warn('Object lost during approach – re-searching')
            self._transition(find_state)
            return

        h_err = mc.x - CENTER_X

        if abs(h_err) > RECENTER_TOL:
            self._stop()
            self.get_logger().warn(
                f'Heading drift ({h_err:+.0f} px) – re-centring')
            self._transition(find_state)
            return

        if dist is not None:
            self.get_logger().info(
                f'[{self.state.name}] dist={dist:.3f} m  h_err={h_err:+.0f} px',
                throttle_duration_sec=0.5)
            if dist <= APPROACH_DIST:
                self._stop()
                self.get_logger().info(
                    f'Close enough ({dist:.3f} m ≤ {APPROACH_DIST} m)')
                self._transition(tilt_state)
                return

        self._drive(h_err)

    def _do_tilt(self, find_state: State, next_state: State):
        """Generic tilt-adjust logic used by both TILT_BLOCK and TILT_BIN."""
        mc = self.mask_center
        if mc is None:
            self.get_logger().warn('Object lost during tilt adjust – re-searching')
            self._transition(find_state)
            return

        h_err = mc.x - CENTER_X
        v_err = mc.y - CENTER_Y

        self.get_logger().info(
            f'[{self.state.name}] h_err={h_err:+.0f}  v_err={v_err:+.0f}  '
            f'tilt={self._cmd_tilt:.3f} rad',
            throttle_duration_sec=0.3)

        if abs(h_err) > RECENTER_TOL:
            self.get_logger().warn(
                f'Horizontal drift ({h_err:+.0f} px) during tilt – re-searching')
            self._transition(find_state)
            return

        if abs(v_err) < VERTICAL_TOL:
            self.get_logger().info(
                f'Object centred vertically (v_err={v_err:+.0f} px)')
            self.pose_timestamp = None
            self._transition(next_state)
        else:
            new_tilt = self._cmd_tilt - v_err * TILT_GAIN
            self._set_pan_tilt(self._cmd_pan, new_tilt)

    def _do_wait_pose(self, next_state: State):
        """Wait for robot to settle, then collect and average pose samples.

        Phase 1 (0 → SETTLE_TIME): let vibrations die down after stopping.
        Phase 2 (SETTLE_TIME → SETTLE_TIME+POSE_WAIT_TIMEOUT): collect
          NUM_POSE_SAMPLES fresh poses from simple_pose_fit and average them
          for a more robust estimate before handing off to manipulation.
        """
        elapsed = self._elapsed()

        # Phase 1: settle
        if elapsed < SETTLE_TIME:
            self.get_logger().info(
                f'[{self.state.name}] Settling … ({elapsed:.1f} / {SETTLE_TIME:.0f} s)',
                throttle_duration_sec=0.5)
            return

        # Phase 2: collect samples
        if self.pose_timestamp is not None and \
                self.pose_timestamp >= self.state_start + SETTLE_TIME:
            if len(self._pose_samples) == 0 or \
                    self.pose_timestamp > self._pose_samples[-1][0]:
                self._pose_samples.append((self.pose_timestamp, self.object_pose))
                self.get_logger().info(
                    f'[{self.state.name}] Pose sample '
                    f'{len(self._pose_samples)}/{NUM_POSE_SAMPLES}')

        if len(self._pose_samples) >= NUM_POSE_SAMPLES:
            self.object_pose = self._average_poses(
                [s[1] for s in self._pose_samples])
            self._pose_samples.clear()
            self.get_logger().info(f'[{self.state.name}] Averaged pose computed')
            self._transition(next_state)
            return

        total_timeout = SETTLE_TIME + POSE_WAIT_TIMEOUT
        if elapsed > total_timeout:
            if len(self._pose_samples) > 0:
                self.get_logger().warn(
                    f'[{self.state.name}] Timed out with {len(self._pose_samples)} '
                    f'samples – using partial average')
                self.object_pose = self._average_poses(
                    [s[1] for s in self._pose_samples])
                self._pose_samples.clear()
                self._transition(next_state)
            else:
                self.get_logger().error(
                    'Timed out waiting for /object_pose. '
                    'Is simple_pose_fit.py running with the correct color?')
                self._transition(State.ERROR)
        else:
            self.get_logger().info(
                f'[{self.state.name}] Waiting for pose … '
                f'({elapsed:.1f} / {total_timeout:.0f} s, '
                f'samples: {len(self._pose_samples)}/{NUM_POSE_SAMPLES})',
                throttle_duration_sec=1.0)

    def _do_plan_to_target(self, success_state: State, fail_state: State,
                            mode: str = 'pick',
                            retry_state: Optional[State] = None,
                            max_retries: int = 0,
                            retry_count_attr: str = ''):
        """
        Generic async /plan_to_target handler used by GRASP and PLACE.

        On entry (pending_future is None): transform current object pose and
        send service request.
        While waiting: poll future.done(); enforce timeout.
        On result: transition to success_state or fail_state.
        """
        elapsed = self._elapsed()

        # ── Send request once on entry ────────────────────────────────────────
        if self._pending_future is None and elapsed < 0.2:
            pose_base = self._transform_to_base(self.object_pose)
            if pose_base is None:
                self.get_logger().error('TF transform failed')
                self._transition(State.ERROR)
                return

            sent = self._call_plan_to_target(pose_base, mode=mode, duration=5.0)
            if not sent:
                return  # retry next tick

            p = pose_base.pose.position
            retries_str = ''
            if retry_count_attr:
                n = getattr(self, retry_count_attr, 0)
                retries_str = f' (attempt {n + 1}/{max_retries})'
            self.get_logger().info(
                f'[{self.state.name}] Calling /plan_to_target{retries_str}: '
                f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m')
            return

        # ── Poll future ───────────────────────────────────────────────────────
        if self._pending_future is not None:
            if not self._pending_future.done():
                if elapsed > RESULT_TIMEOUT:
                    self.get_logger().error(
                        f'[{self.state.name}] /plan_to_target timed out')
                    self._pending_future = None
                    self._transition(State.ERROR)
                return

            result = self._pending_future.result()
            self._pending_future = None

            # For pick: treat grasped=False as a failure worth retrying
            pick_failed = mode == 'pick' and result.success and not result.grasped

            if result.success and not pick_failed:
                self.get_logger().info(
                    f'[{self.state.name}] Planning/execution succeeded: '
                    f'{result.message}')
                self._transition(success_state)
            else:
                if pick_failed:
                    self.get_logger().warn(
                        f'[{self.state.name}] Arm reached target but grasp failed')
                else:
                    self.get_logger().error(
                        f'[{self.state.name}] Planning failed: {result.message}')
                if retry_state is not None and retry_count_attr:
                    retries = getattr(self, retry_count_attr, 0) + 1
                    setattr(self, retry_count_attr, retries)
                    if retries < max_retries:
                        self.get_logger().warn(
                            f'Retrying ({retries}/{max_retries}) …')
                        self.pose_timestamp = None
                        self._transition(retry_state)
                        return
                    self.get_logger().error(
                        f'Failed after {max_retries} attempts')
                self._transition(fail_state)

    # =========================================================================
    # State machine
    # =========================================================================

    def _loop(self):  # noqa: C901
        elapsed = self._elapsed()

        # ── WAIT_SPEECH ──────────────────────────────────────────────────────
        if self.state == State.WAIT_SPEECH:
            if elapsed > 60.0:
                self.get_logger().error(
                    'Timed out waiting for speech. '
                    'Falling back to parameter colors: '
                    f'block={self.block_color}, bin={self.bin_color}')
                if self._speech_proc is not None:
                    self._speech_proc.terminate()
                    self._speech_proc = None
                self._start_vision_nodes(self.block_color)
                self._transition(State.FIND_BLOCK)
            else:
                self.get_logger().info(
                    f'Listening for voice command... ({elapsed:.0f}s / 60s)',
                    throttle_duration_sec=5.0)

        # ── FIND_BLOCK ────────────────────────────────────────────────────────
        elif self.state == State.FIND_BLOCK:
            self._do_find(next_state=State.APPROACH_BLOCK)

        # ── APPROACH_BLOCK ────────────────────────────────────────────────────
        elif self.state == State.APPROACH_BLOCK:
            self._do_approach(find_state=State.FIND_BLOCK,
                              tilt_state=State.TILT_BLOCK)

        # ── TILT_BLOCK ────────────────────────────────────────────────────────
        elif self.state == State.TILT_BLOCK:
            self._do_tilt(find_state=State.FIND_BLOCK,
                          next_state=State.WAIT_BLOCK_POSE)

        # ── WAIT_BLOCK_POSE ───────────────────────────────────────────────────
        elif self.state == State.WAIT_BLOCK_POSE:
            self._do_wait_pose(next_state=State.GRASP)

        # ── GRASP ─────────────────────────────────────────────────────────────
        elif self.state == State.GRASP:
            self._do_plan_to_target(
                success_state=State.FIND_BIN,
                fail_state=State.ERROR,
                mode='pick',
                retry_state=State.WAIT_BLOCK_POSE,
                max_retries=MAX_GRASP_RETRIES,
                retry_count_attr='_grasp_retries',
            )
            # On grasp success, switch vision to bin color and reset camera
            if self.state == State.FIND_BIN:
                self._grasp_retries = 0
                self._set_pan_tilt(0.0, 0.0)
                self._set_target_color(self.bin_color)

        # ── FIND_BIN ──────────────────────────────────────────────────────────
        elif self.state == State.FIND_BIN:
            self._do_find(next_state=State.APPROACH_BIN)

        # ── APPROACH_BIN ──────────────────────────────────────────────────────
        elif self.state == State.APPROACH_BIN:
            self._do_approach(find_state=State.FIND_BIN,
                              tilt_state=State.TILT_BIN)

        # ── TILT_BIN ──────────────────────────────────────────────────────────
        elif self.state == State.TILT_BIN:
            self._do_tilt(find_state=State.FIND_BIN,
                          next_state=State.WAIT_BIN_POSE)

        # ── WAIT_BIN_POSE ─────────────────────────────────────────────────────
        elif self.state == State.WAIT_BIN_POSE:
            self._do_wait_pose(next_state=State.PLACE)

        # ── PLACE ─────────────────────────────────────────────────────────────
        elif self.state == State.PLACE:
            self._do_plan_to_target(
                success_state=State.DONE,
                fail_state=State.ERROR,
                mode='place',
            )

        # ── DONE ─────────────────────────────────────────────────────────────
        elif self.state == State.DONE:
            if elapsed < 0.1:
                self.get_logger().info('=' * 60)
                self.get_logger().info(
                    f'Task 2 complete: [{self.block_color}] block → '
                    f'[{self.bin_color}] bin')
                self.get_logger().info('=' * 60)

        # ── ERROR ─────────────────────────────────────────────────────────────
        elif self.state == State.ERROR:
            if elapsed < 0.1:
                self.get_logger().error('Task 2 ended in ERROR state')


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = Task2Node()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node._stop_vision_nodes()
        if node._speech_proc is not None:
            node._speech_proc.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
