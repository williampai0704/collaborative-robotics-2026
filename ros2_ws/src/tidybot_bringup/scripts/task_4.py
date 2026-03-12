#!/usr/bin/env python3
"""
Task 4: Repeatedly listen for voice commands, each specifying ONE block → ONE bin.

Unlike task_3 (multiple blocks → one shared bin, one speech command), task_4 re-listens
after every successful place cycle so each block-bin pair is commanded individually.

─── Prerequisites (must already be running) ───────────────────────────────────
1. Real robot hardware:
       ros2 launch tidybot_bringup real.launch.py

2. Motion planner:
       ros2 run tidybot_ik motion_planner_real_node

─── Run task_4 ────────────────────────────────────────────────────────────────
    ros2 run tidybot_bringup task_4.py

Custom arm:
    ros2 run tidybot_bringup task_4.py --ros-args -p arm_name:=right

─── What this script does ─────────────────────────────────────────────────────
Loop:
  1. WAIT_SPEECH — listen for "put the <block> block in the <bin> bin"
     • Say "done" / "stop" / "finish" to end the loop.
     • If SPEECH_TIMEOUT elapses with no command → DONE (if ≥1 task completed)
       or ERROR (if nothing was ever placed).
  2. Find & Pick the specified block (same pipeline as task_3).
  3. Find & Place in the specified bin.
  4. Go back to step 1.

─── State machine ─────────────────────────────────────────────────────────────
  WAIT_SPEECH  listen; "done" or timeout→DONE; valid pair→FIND_BLOCK
    ↓
  FIND_BLOCK   spin to centre block horizontally
    ↓
  APPROACH_BLOCK drive forward; stop at APPROACH_DIST
    ↓
  WAIT_BLOCK_POSE  wait for averaged /object_pose
    ↓
  GRASP        pick block; switch vision to bin_color → FIND_BIN
    ↓
  FIND_BIN     spin to centre bin horizontally
    ↓
  APPROACH_BIN drive forward; stop at APPROACH_DIST
    ↓
  WAIT_BIN_POSE  wait for averaged /object_pose
    ↓
  PLACE        place block → re-launch speech → WAIT_SPEECH
  DONE / ERROR

─── Available colors ──────────────────────────────────────────────────────────
  red, green, blue, yellow, orange, purple

─── Example speech commands ───────────────────────────────────────────────────
  "Put the red block in the green bin."
  "Place the blue block into the yellow bin."
  "Done." / "Stop." — ends the session.
"""

import os
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
from tidybot_msgs.msg import GripperCommand

from scipy.spatial.transform import Rotation as ScipyRotation

from tidybot_msgs.srv import PlanToTarget

# =============================================================================
# Configuration  (keep in sync with task_3.py / task_5.py)
# =============================================================================

IMAGE_WIDTH   = 640
IMAGE_HEIGHT  = 480
CENTER_X      = IMAGE_WIDTH  / 2.0
CENTER_Y      = IMAGE_HEIGHT / 2.0

HORIZONTAL_TOL   = 50    # px — stop spinning: |cx − CENTER_X| < this
RECENTER_TOL     = 120   # px — stop approach and re-center if drift > this

APPROACH_DIST    = 0.35  # m   — stop driving when object is this close
APPROACH_SPEED   = 0.06  # m/s — forward speed during approach
SPIN_SPEED       = 0.2   # rad/s
ANGULAR_GAIN     = 0.003 # rad/s per px of horizontal error

POSE_WAIT_TIMEOUT = 8.0   # s — additional wait after settle for pose samples
SETTLE_TIME       = 2.0   # s — wait for robot to stop moving before accepting poses
NUM_POSE_SAMPLES  = 10    # number of pose samples to average for robust estimate
RESULT_TIMEOUT    = 30.0  # s — max wait for /plan_to_target (includes execution)
MAX_GRASP_RETRIES = 3
GRASP_Z           = 0.07  # m — fixed grasp/place height in base_link frame
MOTION_DURATION   = 3.0   # s — time for each arm motion segment

# Calibration offsets applied to every IK target (metres, base_link frame).
GRASP_OFFSET_X    = 0.03
GRASP_OFFSET_Y    = -0.045
GRASP_OFFSET_Z    = 0.0

SPEECH_TIMEOUT    = 60.0  # s

# Valid color names accepted by color_mask.py
VALID_COLORS = {'red', 'green', 'blue', 'yellow', 'orange', 'purple'}

# Words Gemini may return to signal "I'm done"
DONE_SIGNALS = {'done', 'stop', 'finish', 'finished', 'exit', 'quit', 'end'}

# Gemini prompt: extract ONE block color and ONE bin color per command.
# Returns {"object_color": "done", "bin_color": "done"} for stop signals.
TASK4_EXTRACTION_PROMPT = (
    "Given the instruction, extract two things: "
    "1) 'object_color': the single block color to pick (one word). "
    "2) 'bin_color': the single bin color to place into (one word). "
    "Only use these exact color names: red, green, blue, yellow, orange, purple. "
    "If the user says 'done', 'stop', 'finish', 'that\\'s all', or any similar "
    "phrase indicating they are finished, return "
    "{\"object_color\": \"done\", \"bin_color\": \"done\"}. "
    "Return only a JSON object. If a value cannot be determined, use 'unknown'. "
    "Example output: {\"object_color\": \"red\", \"bin_color\": \"green\"} "
    "Instruction: "
)

# =============================================================================
# States
# =============================================================================

class State(Enum):
    WAIT_SPEECH    = auto()
    FIND_BLOCK     = auto()
    APPROACH_BLOCK = auto()
    WAIT_BLOCK_POSE= auto()
    GRASP          = auto()
    FIND_BIN       = auto()
    APPROACH_BIN   = auto()
    WAIT_BIN_POSE  = auto()
    PLACE          = auto()
    DONE           = auto()
    ERROR          = auto()

# =============================================================================
# Task 4 Node
# =============================================================================

class Task4Node(Node):

    def __init__(self):
        super().__init__('task_4')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('arm_name', 'left')

        self.arm_name: str = self.get_parameter('arm_name').value

        # ── Per-command state (filled by _speech_cb each iteration) ─────────
        self.current_block_color: str = ''
        self.current_bin_color:   str = ''
        self._tasks_completed:    int = 0

        # ── Callback groups ──────────────────────────────────────────────────
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        # ── Publishers ──────────────────────────────────────────────────────
        self.cmd_vel_pub       = self.create_publisher(Twist,             '/cmd_vel',                  10)
        self.pan_tilt_pub      = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd',      10)
        self.color_pub         = self.create_publisher(String,            '/vision/target_color',      10)
        self.left_gripper_pub  = self.create_publisher(GripperCommand,    '/left_gripper/command',     10)
        self.right_gripper_pub = self.create_publisher(GripperCommand,    '/right_gripper/command',    10)

        # ── Service client ───────────────────────────────────────────────────
        self.plan_client = self.create_client(
            PlanToTarget, '/plan_to_target',
            callback_group=self.client_cb_group)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Point,       '/mask_center',            self._mask_center_cb, 10)
        self.create_subscription(Float32,     '/vision/object_distance', self._obj_dist_cb,    10)
        self.create_subscription(PoseStamped, '/object_pose',            self._object_pose_cb, 10)

        from tidybot_msgs.msg import SpeechExtraction
        self.create_subscription(
            SpeechExtraction, '/speech_extraction', self._speech_cb, 10)

        # ── TF2 ─────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Shared sensor state ──────────────────────────────────────────────
        self.mask_center:     Optional[Point]       = None
        self.object_distance: Optional[float]       = None
        self.object_pose:     Optional[PoseStamped] = None
        self.pose_timestamp:  Optional[float]       = None

        self._cmd_pan:        float = 0.0
        self._spin_sign:      float = 1.0   # flips to -1.0 after grasp to search opposite
        self._grasp_retries:  int   = 0

        self._pending_future = None
        self._pose_samples: list = []
        self._vision_procs: List[subprocess.Popen] = []
        self._speech_proc:  Optional[subprocess.Popen] = None

        # ── Initial state: always starts in WAIT_SPEECH ──────────────────────
        self.state       = State.WAIT_SPEECH
        self.state_start = time.time()
        self._launch_speech_node()

        self.create_timer(0.05, self._loop)  # 20 Hz

        self.get_logger().info('=' * 65)
        self.get_logger().info('Task 4: Repeated speech-commanded pick-and-place')
        self.get_logger().info(f'  Arm: {self.arm_name}')
        self.get_logger().info('  Say: "put the <block> block in the <bin> bin"')
        self.get_logger().info('  Say: "done" / "stop" to finish.')
        self.get_logger().info('=' * 65)

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _speech_cb(self, msg):
        """Receive one block→bin command (or stop signal)."""
        if self.state != State.WAIT_SPEECH:
            return
        if not msg.success:
            self.get_logger().warn(
                f'Speech extraction failed: {msg.error_message} — still waiting...')
            return

        block_color = msg.object_color.strip().lower()
        bin_color   = msg.bin_color.strip().lower()

        # Explicit stop signal
        if block_color in DONE_SIGNALS or bin_color in DONE_SIGNALS:
            self.get_logger().info(
                f'[Speech] Stop signal received after {self._tasks_completed} task(s) → DONE')
            if self._speech_proc is not None:
                self._speech_proc.terminate()
                self._speech_proc = None
            self._transition(State.DONE)
            return

        # Validate colors
        if block_color not in VALID_COLORS or bin_color not in VALID_COLORS:
            self.get_logger().warn(
                f'[Speech] Unrecognized colors: block="{block_color}" bin="{bin_color}" '
                f'— still waiting...')
            return

        self.current_block_color = block_color
        self.current_bin_color   = bin_color
        self.get_logger().info(
            f'[Speech] Command: [{block_color}] block → [{bin_color}] bin')

        if self._speech_proc is not None:
            self._speech_proc.terminate()
            self._speech_proc = None

        self._spin_sign = 1.0
        self._switch_vision(block_color)
        self._transition(State.FIND_BLOCK)

    def _mask_center_cb(self, msg: Point):
        self.mask_center = msg

    def _obj_dist_cb(self, msg: Float32):
        self.object_distance = float(msg.data)

    def _object_pose_cb(self, msg: PoseStamped):
        self.object_pose    = msg
        self.pose_timestamp = time.time()

    # =========================================================================
    # Helpers
    # =========================================================================

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())

    def _spin(self):
        t = Twist()
        t.angular.z = self._spin_sign * SPIN_SPEED
        self.cmd_vel_pub.publish(t)

    def _open_gripper(self):
        """Publish an open-gripper command for the active arm."""
        cmd = GripperCommand()
        cmd.position = 0.0
        cmd.effort   = 0.5
        if self.arm_name == 'left':
            self.left_gripper_pub.publish(cmd)
        else:
            self.right_gripper_pub.publish(cmd)
        self.get_logger().info(f'[Gripper] Opened {self.arm_name} gripper')

    def _drive(self, h_err: float):
        t = Twist()
        t.linear.x  = APPROACH_SPEED
        t.angular.z = float(np.clip(-h_err * ANGULAR_GAIN, -0.5, 0.5))
        self.cmd_vel_pub.publish(t)

    def _set_pan_tilt(self, pan: float, tilt: float):
        self._cmd_pan = float(pan)
        msg = Float64MultiArray()
        msg.data = [self._cmd_pan, float(tilt)]
        self.pan_tilt_pub.publish(msg)

    def _start_vision_nodes(self, color: str):
        self._stop_vision_nodes()
        for script in ['find_center.py', 'obj_dist.py', 'simple_pose_fit.py']:
            proc = subprocess.Popen([
                'ros2', 'run', 'tidybot_bringup', script,
                '--ros-args', '-p', f'target_color:={color}', '-p', 'visualize:=true',
            ])
            self._vision_procs.append(proc)
        self.get_logger().info(f'[Vision] Launched 3 nodes with target_color={color}')

    def _stop_vision_nodes(self):
        for proc in self._vision_procs:
            try:
                proc.terminate()
                proc.wait(timeout=3.0)
            except Exception:
                proc.kill()
        self._vision_procs = []

    def _switch_vision(self, color: str):
        msg = String()
        msg.data = color
        self.color_pub.publish(msg)
        self.mask_center    = None
        self.object_pose    = None
        self.pose_timestamp = None
        self._start_vision_nodes(color)

    def _launch_speech_node(self):
        """(Re-)launch the speech extraction subprocess via env var prompt."""
        if self._speech_proc is not None:
            self._speech_proc.terminate()
            self._speech_proc = None
        env = os.environ.copy()
        env['EXTRACTION_PROMPT'] = TASK4_EXTRACTION_PROMPT
        self._speech_proc = subprocess.Popen([
            'ros2', 'run', 'tidybot_bringup', 'speech_extraction_node.py',
        ], env=env)
        self.get_logger().info('[Speech] Listening for next command...')

    def _average_poses(self, poses: list) -> PoseStamped:
        """Average multiple PoseStamped messages (position mean + quaternion mean)."""
        positions = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                              for p in poses])
        quats = np.array([[p.pose.orientation.x, p.pose.orientation.y,
                           p.pose.orientation.z, p.pose.orientation.w]
                          for p in poses])
        mean_pos = positions.mean(axis=0)
        for i in range(1, len(quats)):
            if np.dot(quats[i], quats[0]) < 0:
                quats[i] = -quats[i]
        mean_quat = quats.mean(axis=0)
        mean_quat /= np.linalg.norm(mean_quat)
        out = PoseStamped()
        out.header = poses[-1].header
        out.pose.position.x = float(mean_pos[0]) + 0.01
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
        self._pending_future = None
        self._pose_samples.clear()

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
                              duration: float = MOTION_DURATION) -> bool:
        if not self.plan_client.service_is_ready():
            self.get_logger().warn(
                'Waiting for /plan_to_target service …',
                throttle_duration_sec=1.0)
            return False

        req = PlanToTarget.Request()
        req.arm_name               = self.arm_name
        req.mode                   = mode
        req.target_pose            = pose_base.pose
        req.target_pose.position.x += GRASP_OFFSET_X
        req.target_pose.position.y += GRASP_OFFSET_Y
        req.target_pose.position.z  = GRASP_Z + GRASP_OFFSET_Z
        req.use_orientation        = False
        req.max_condition_number   = 500.0
        req.execute                = True
        req.duration               = duration

        self._pending_future = self.plan_client.call_async(req)
        return True

    # =========================================================================
    # Shared sub-behaviours (identical to task_3)
    # =========================================================================

    def _do_find(self, next_state: State):
        mc = self.mask_center
        if mc is None:
            # Target not visible — spin only
            self._spin()
            return
        h_err = mc.x - CENTER_X
        self.get_logger().info(
            f'[{self.state.name}] cx={mc.x:.0f}  h_err={h_err:+.0f} px',
            throttle_duration_sec=0.5)
        if abs(h_err) < HORIZONTAL_TOL:
            self._stop()
            self._transition(next_state)
        else:
            self._spin()

    def _do_approach(self, find_state: State, next_state: State):
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
            self.get_logger().warn(f'Heading drift ({h_err:+.0f} px) – re-centring')
            self._transition(find_state)
            return
        if dist is not None:
            self.get_logger().info(
                f'[{self.state.name}] dist={dist:.3f} m  h_err={h_err:+.0f} px',
                throttle_duration_sec=0.5)
            if dist <= APPROACH_DIST:
                self._stop()
                self._transition(next_state)
                return
        self._drive(h_err)

    def _do_wait_pose(self, next_state: State):
        """Wait for robot to settle, then collect and average pose samples.

        Phase 1 (0 → SETTLE_TIME): let vibrations die down after stopping.
        Phase 2 (SETTLE_TIME → SETTLE_TIME+POSE_WAIT_TIMEOUT): collect
          NUM_POSE_SAMPLES fresh poses from simple_pose_fit and average them.
        """
        elapsed = self._elapsed()

        if elapsed < SETTLE_TIME:
            self.get_logger().info(
                f'[{self.state.name}] Settling … ({elapsed:.1f} / {SETTLE_TIME:.0f} s)',
                throttle_duration_sec=0.5)
            return

        if self.pose_timestamp is not None and \
                self.pose_timestamp >= self.state_start + SETTLE_TIME:
            if len(self._pose_samples) == 0 or \
                    self.pose_timestamp > self._pose_samples[-1][0]:
                self._pose_samples.append((self.pose_timestamp, self.object_pose))
                self.get_logger().info(
                    f'[{self.state.name}] Pose sample '
                    f'{len(self._pose_samples)}/{NUM_POSE_SAMPLES}')

        if len(self._pose_samples) >= NUM_POSE_SAMPLES:
            self.object_pose = self._average_poses([s[1] for s in self._pose_samples])
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
                self.object_pose = self._average_poses([s[1] for s in self._pose_samples])
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
        elapsed = self._elapsed()

        if self._pending_future is None and elapsed < 0.2:
            pose_base = self._transform_to_base(self.object_pose)
            if pose_base is None:
                self.get_logger().error('TF transform failed')
                self._transition(State.ERROR)
                return
            sent = self._call_plan_to_target(pose_base, mode=mode, duration=MOTION_DURATION)
            if not sent:
                return
            p = pose_base.pose.position
            retries_str = ''
            if retry_count_attr:
                n = getattr(self, retry_count_attr, 0)
                retries_str = f' (attempt {n + 1}/{max_retries})'
            self.get_logger().info(
                f'[{self.state.name}] Calling /plan_to_target{retries_str}: '
                f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m')
            return

        if self._pending_future is not None:
            if not self._pending_future.done():
                if elapsed > RESULT_TIMEOUT:
                    self.get_logger().error(f'[{self.state.name}] /plan_to_target timed out')
                    self._pending_future = None
                    self._transition(State.ERROR)
                return

            result = self._pending_future.result()
            self._pending_future = None

            pick_failed = mode == 'pick' and result.success and not result.grasped

            if result.success and not pick_failed:
                self.get_logger().info(f'[{self.state.name}] Succeeded: {result.message}')
                self._transition(success_state)
            else:
                if pick_failed:
                    self.get_logger().warn(f'[{self.state.name}] Grasp failed')
                else:
                    self.get_logger().error(
                        f'[{self.state.name}] Planning failed: {result.message}')
                if retry_state is not None and retry_count_attr:
                    retries = getattr(self, retry_count_attr, 0) + 1
                    setattr(self, retry_count_attr, retries)
                    if retries < max_retries:
                        self.get_logger().warn(f'Retrying ({retries}/{max_retries}) …')
                        if mode == 'pick':
                            self._open_gripper()
                        self.pose_timestamp = None
                        self._transition(retry_state)
                        return
                    self.get_logger().error(f'Failed after {max_retries} attempts')
                self._transition(fail_state)

    # =========================================================================
    # State machine
    # =========================================================================

    def _loop(self):  # noqa: C901
        elapsed = self._elapsed()

        # ── WAIT_SPEECH ──────────────────────────────────────────────────────
        if self.state == State.WAIT_SPEECH:
            if elapsed > SPEECH_TIMEOUT:
                if self._tasks_completed > 0:
                    self.get_logger().info(
                        f'Speech timeout — {self._tasks_completed} task(s) completed → DONE')
                else:
                    self.get_logger().warn('Speech timeout with no tasks completed → DONE')
                if self._speech_proc is not None:
                    self._speech_proc.terminate()
                    self._speech_proc = None
                self._transition(State.DONE)
            else:
                self.get_logger().info(
                    f'Listening... ({elapsed:.0f}s / {SPEECH_TIMEOUT:.0f}s)',
                    throttle_duration_sec=5.0)

        # ── FIND_BLOCK ────────────────────────────────────────────────────────
        elif self.state == State.FIND_BLOCK:
            self.get_logger().info(
                f'[Task {self._tasks_completed + 1}] Searching for [{self.current_block_color}]',
                throttle_duration_sec=2.0)
            self._do_find(next_state=State.APPROACH_BLOCK)

        # ── APPROACH_BLOCK ────────────────────────────────────────────────────
        elif self.state == State.APPROACH_BLOCK:
            self._do_approach(find_state=State.FIND_BLOCK,
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
            if self.state == State.FIND_BIN:
                self._grasp_retries = 0
                self._spin_sign = -1.0  # spin opposite to find bin efficiently
                self._set_pan_tilt(0.0, 0.0)
                self._switch_vision(self.current_bin_color)

        # ── FIND_BIN ──────────────────────────────────────────────────────────
        elif self.state == State.FIND_BIN:
            self.get_logger().info(
                f'Searching for [{self.current_bin_color}] bin',
                throttle_duration_sec=2.0)
            self._do_find(next_state=State.APPROACH_BIN)

        # ── APPROACH_BIN ──────────────────────────────────────────────────────
        elif self.state == State.APPROACH_BIN:
            self._do_approach(find_state=State.FIND_BIN,
                              next_state=State.WAIT_BIN_POSE)

        # ── WAIT_BIN_POSE ─────────────────────────────────────────────────────
        elif self.state == State.WAIT_BIN_POSE:
            self._do_wait_pose(next_state=State.PLACE)

        # ── PLACE ─────────────────────────────────────────────────────────────
        elif self.state == State.PLACE:
            self._do_plan_to_target(
                success_state=State.WAIT_SPEECH,
                fail_state=State.ERROR,
                mode='place',
            )
            if self.state == State.WAIT_SPEECH:
                self._tasks_completed += 1
                self.get_logger().info(
                    f'[Task {self._tasks_completed}] '
                    f'[{self.current_block_color}] → [{self.current_bin_color}] complete!')
                self._stop_vision_nodes()
                self._set_pan_tilt(0.0, 0.0)
                self._launch_speech_node()

        # ── DONE ─────────────────────────────────────────────────────────────
        elif self.state == State.DONE:
            if elapsed < 0.1:
                self.get_logger().info('=' * 65)
                self.get_logger().info(
                    f'Task 4 complete — {self._tasks_completed} block(s) placed.')
                self.get_logger().info('=' * 65)

        # ── ERROR ─────────────────────────────────────────────────────────────
        elif self.state == State.ERROR:
            if elapsed < 0.1:
                self.get_logger().error(
                    f'Task 4 ended in ERROR after {self._tasks_completed} task(s). '
                    f'Last command: [{self.current_block_color}] → [{self.current_bin_color}]')


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = Task4Node()
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
