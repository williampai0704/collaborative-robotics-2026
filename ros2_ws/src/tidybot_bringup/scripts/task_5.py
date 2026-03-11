#!/usr/bin/env python3
"""
Task 5: One speech command specifying multiple (block → bin) pairs; execute all sequentially.

Unlike task_3 (many blocks → one shared bin) and task_4 (one block per speech command),
task_5 accepts a single upfront command like
  "put the red block in the green bin and the blue block in the yellow bin"
and then autonomously executes every pair in the stated order.

─── Prerequisites (must already be running) ───────────────────────────────────
1. Real robot hardware:
       ros2 launch tidybot_bringup real.launch.py

2. Motion planner:
       ros2 run tidybot_ik motion_planner_real_node

─── Run task_5 ────────────────────────────────────────────────────────────────
    ros2 run tidybot_bringup task_5.py

Custom arm / fallback pairs (used when use_speech:=false):
    ros2 run tidybot_bringup task_5.py \
        --ros-args -p arm_name:=left \
                   -p block_bin_pairs:=red:green,blue:yellow \
                   -p use_speech:=false

─── Speech format ─────────────────────────────────────────────────────────────
Say something like:
  "Put the red block in the green bin and the blue block in the yellow bin."
  "Place orange in purple bin, then red in green bin."

Gemini extracts ordered (block:bin) pairs returned in object_color as
  "red:green,blue:yellow,orange:purple"
(colon separates block from bin; comma separates pairs).

─── State machine ─────────────────────────────────────────────────────────────
  WAIT_SPEECH  listen; valid pairs → FIND_BLOCK; timeout (no pairs received) → ERROR
    ↓
  [for each (block_color, bin_color) pair in queue:]
  FIND_BLOCK   spin to centre block
    ↓
  APPROACH_BLOCK
    ↓
  TILT_BLOCK
    ↓
  WAIT_BLOCK_POSE
    ↓
  GRASP        pick block; switch vision to this pair's bin_color → FIND_BIN
    ↓
  FIND_BIN
    ↓
  APPROACH_BIN
    ↓
  TILT_BIN
    ↓
  WAIT_BIN_POSE
    ↓
  PLACE        place block → advance to next pair → FIND_BLOCK (or DONE)
  DONE / ERROR
"""

import time
import subprocess
from enum import Enum, auto
from typing import Optional, List, Tuple

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
# Configuration  (keep in sync with task_3.py / task_4.py)
# =============================================================================

IMAGE_WIDTH   = 640
IMAGE_HEIGHT  = 480
CENTER_X      = IMAGE_WIDTH  / 2.0
CENTER_Y      = IMAGE_HEIGHT / 2.0

HORIZONTAL_TOL   = 50    # px
RECENTER_TOL     = 120   # px
VERTICAL_TOL     = 60    # px

APPROACH_DIST    = 0.50  # m
APPROACH_SPEED   = 0.06  # m/s
SPIN_SPEED       = 0.3   # rad/s
ANGULAR_GAIN     = 0.003

TILT_GAIN        = 0.001
TILT_MIN         = -0.5  # rad
TILT_MAX         =  0.3  # rad
TILT_SWEEP_SPEED = 0.012

POSE_WAIT_TIMEOUT = 4.0   # s
SETTLE_TIME       = 1.0   # s
NUM_POSE_SAMPLES  = 5
RESULT_TIMEOUT    = 30.0  # s
MAX_GRASP_RETRIES = 3
GRASP_Z           = 0.07  # m
MOTION_DURATION   = 3.0   # s

GRASP_OFFSET_X    = 0.0
GRASP_OFFSET_Y    = 0.0
GRASP_OFFSET_Z    = 0.0

SPEECH_TIMEOUT    = 60.0  # s

VALID_COLORS = {'red', 'green', 'blue', 'yellow', 'orange', 'purple'}

# Gemini prompt: extract ordered (block:bin) pairs from one spoken instruction.
# object_color encodes all pairs as "block1:bin1,block2:bin2,..." — colon separates
# block from bin, comma separates pairs. bin_color is left empty.
TASK5_EXTRACTION_PROMPT = (
    "Given the instruction, extract ordered block-to-bin placement pairs. "
    "Return a JSON object with: "
    "1) 'object_color': a comma-separated string of 'block:bin' pairs with no spaces "
    "(e.g. 'red:green,blue:yellow,orange:purple'). "
    "2) 'bin_color': an empty string. "
    "Only use these exact color names: red, green, blue, yellow, orange, purple. "
    "Preserve the order as stated in the instruction. "
    "If a value cannot be determined, use 'unknown'. "
    "Example output: {\"object_color\": \"red:green,blue:yellow\", \"bin_color\": \"\"} "
    "Instruction: "
)

# =============================================================================
# States
# =============================================================================

class State(Enum):
    WAIT_SPEECH    = auto()
    FIND_BLOCK     = auto()
    APPROACH_BLOCK = auto()
    TILT_BLOCK     = auto()
    WAIT_BLOCK_POSE= auto()
    GRASP          = auto()
    FIND_BIN       = auto()
    APPROACH_BIN   = auto()
    TILT_BIN       = auto()
    WAIT_BIN_POSE  = auto()
    PLACE          = auto()
    DONE           = auto()
    ERROR          = auto()

# =============================================================================
# Task 5 Node
# =============================================================================

class Task5Node(Node):

    def __init__(self):
        super().__init__('task_5')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('arm_name',       'left')
        self.declare_parameter('use_speech',     True)
        # Fallback pairs when use_speech:=false, format: "red:green,blue:yellow"
        self.declare_parameter('block_bin_pairs', 'red:green,blue:yellow')

        self.arm_name:    str  = self.get_parameter('arm_name').value
        self.use_speech:  bool = self.get_parameter('use_speech').value
        pairs_param:      str  = self.get_parameter('block_bin_pairs').value

        # Queue of (block_color, bin_color) tuples
        self._block_queue: List[Tuple[str, str]] = []
        self._block_idx:   int = 0

        if not self.use_speech:
            self._block_queue = self._parse_pairs(pairs_param)
            if not self._block_queue:
                raise ValueError(
                    f'block_bin_pairs param "{pairs_param}" produced no valid pairs. '
                    f'Expected format: "red:green,blue:yellow". '
                    f'Valid colors: {sorted(VALID_COLORS)}')

        # ── Callback groups ──────────────────────────────────────────────────
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        # ── Publishers ──────────────────────────────────────────────────────
        self.cmd_vel_pub  = self.create_publisher(Twist,             '/cmd_vel',             10)
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.color_pub    = self.create_publisher(String,            '/vision/target_color', 10)

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

        # ── Shared sensor state ──────────────────────────────────────────────
        self.mask_center:     Optional[Point]       = None
        self.object_distance: Optional[float]       = None
        self.object_pose:     Optional[PoseStamped] = None
        self.pose_timestamp:  Optional[float]       = None

        self._cmd_pan:        float = 0.0
        self._cmd_tilt:       float = 0.0
        self._tilt_sweep_dir: float = -1.0
        self._grasp_retries:  int   = 0

        self._pending_future = None
        self._pose_samples: list = []
        self._vision_procs: List[subprocess.Popen] = []
        self._speech_proc:  Optional[subprocess.Popen] = None

        # ── Optional speech subscription ─────────────────────────────────────
        if self.use_speech:
            from tidybot_msgs.msg import SpeechExtraction
            self.create_subscription(
                SpeechExtraction, '/speech_extraction', self._speech_cb, 10)

        # ── Initial state ────────────────────────────────────────────────────
        if self.use_speech:
            self.state = State.WAIT_SPEECH
            self._speech_proc = subprocess.Popen([
                'ros2', 'run', 'tidybot_bringup', 'speech_extraction_node.py',
                '--ros-args', '-p', f'extraction_prompt:={TASK5_EXTRACTION_PROMPT}',
            ])
        else:
            self.state = State.FIND_BLOCK
            self._start_vision_nodes(self._current_block_color)

        self.state_start = time.time()
        self.create_timer(0.05, self._loop)  # 20 Hz

        self.get_logger().info('=' * 65)
        self.get_logger().info('Task 5: Multi-pair pick-and-place from one speech command')
        self.get_logger().info(f'  Arm        : {self.arm_name}')
        self.get_logger().info(f'  use_speech : {self.use_speech}')
        if not self.use_speech:
            self.get_logger().info(f'  Pairs      : {self._block_queue}')
        self.get_logger().info('=' * 65)

    # =========================================================================
    # Properties
    # =========================================================================

    @property
    def _current_block_color(self) -> str:
        return self._block_queue[self._block_idx][0]

    @property
    def _current_bin_color(self) -> str:
        return self._block_queue[self._block_idx][1]

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _speech_cb(self, msg):
        """Receive the full ordered list of (block:bin) pairs from speech."""
        if self.state != State.WAIT_SPEECH:
            return
        if not msg.success:
            self.get_logger().warn(
                f'Speech extraction failed: {msg.error_message} — still waiting...')
            return

        pairs = self._parse_pairs(msg.object_color.strip())
        if not pairs:
            self.get_logger().warn(
                f'No valid pairs extracted from: "{msg.object_color}" — still waiting...')
            return

        self._block_queue = pairs
        self._block_idx   = 0
        self.get_logger().info(f'[Speech] Pairs: {self._block_queue}')

        if self._speech_proc is not None:
            self._speech_proc.terminate()
            self._speech_proc = None

        self._switch_vision(self._current_block_color)
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

    @staticmethod
    def _parse_pairs(raw: str) -> List[Tuple[str, str]]:
        """Parse 'red:green,blue:yellow' into [(red, green), (blue, yellow)].
        Silently drops any pair where either color is not in VALID_COLORS.
        """
        pairs = []
        for token in raw.split(','):
            token = token.strip()
            if ':' not in token:
                continue
            block_c, bin_c = token.split(':', 1)
            block_c = block_c.strip().lower()
            bin_c   = bin_c.strip().lower()
            if block_c in VALID_COLORS and bin_c in VALID_COLORS:
                pairs.append((block_c, bin_c))
        return pairs

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())

    def _spin(self):
        t = Twist()
        t.angular.z = SPIN_SPEED
        self.cmd_vel_pub.publish(t)

    def _drive(self, h_err: float):
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

    def _advance_to_next_pair(self):
        """Move to next (block, bin) pair or finish if queue exhausted."""
        self._block_idx    += 1
        self._grasp_retries = 0
        if self._block_idx < len(self._block_queue):
            self.get_logger().info(
                f'Pair {self._block_idx + 1}/{len(self._block_queue)}: '
                f'[{self._current_block_color}] → [{self._current_bin_color}]')
            self._set_pan_tilt(0.0, 0.0)
            self._switch_vision(self._current_block_color)
            self._transition(State.FIND_BLOCK)
        else:
            self._transition(State.DONE)

    def _average_poses(self, poses: list) -> PoseStamped:
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
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame',
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
        req.use_orientation        = True
        req.max_condition_number   = 500.0
        req.execute                = True
        req.duration               = duration

        self._pending_future = self.plan_client.call_async(req)
        return True

    # =========================================================================
    # Shared sub-behaviours (identical to task_3 / task_4)
    # =========================================================================

    def _do_find(self, next_state: State):
        mc = self.mask_center
        if mc is None:
            self._spin()
            new_tilt = self._cmd_tilt + self._tilt_sweep_dir * TILT_SWEEP_SPEED
            if new_tilt <= TILT_MIN:
                new_tilt = TILT_MIN
                self._tilt_sweep_dir = 1.0
            elif new_tilt >= TILT_MAX:
                new_tilt = TILT_MAX
                self._tilt_sweep_dir = -1.0
            self._set_pan_tilt(self._cmd_pan, new_tilt)
            return
        h_err = mc.x - CENTER_X
        v_err = mc.y - CENTER_Y
        self._set_pan_tilt(self._cmd_pan, self._cmd_tilt - v_err * TILT_GAIN)
        self.get_logger().info(
            f'[{self.state.name}] cx={mc.x:.0f}  h_err={h_err:+.0f}  v_err={v_err:+.0f} px',
            throttle_duration_sec=0.5)
        if abs(h_err) < HORIZONTAL_TOL:
            self._stop()
            self._transition(next_state)
        else:
            self._spin()

    def _do_approach(self, find_state: State, tilt_state: State):
        mc   = self.mask_center
        dist = self.object_distance
        if mc is None:
            self._stop()
            self.get_logger().warn('Object lost during approach – re-searching')
            self._transition(find_state)
            return
        h_err = mc.x - CENTER_X
        v_err = mc.y - CENTER_Y
        self._set_pan_tilt(self._cmd_pan, self._cmd_tilt - v_err * TILT_GAIN)
        if abs(h_err) > RECENTER_TOL:
            self._stop()
            self.get_logger().warn(f'Heading drift ({h_err:+.0f} px) – re-centring')
            self._transition(find_state)
            return
        if dist is not None:
            self.get_logger().info(
                f'[{self.state.name}] dist={dist:.3f} m  h_err={h_err:+.0f}  v_err={v_err:+.0f} px',
                throttle_duration_sec=0.5)
            if dist <= APPROACH_DIST:
                self._stop()
                self._transition(tilt_state)
                return
        self._drive(h_err)

    def _do_tilt(self, find_state: State, next_state: State):
        mc = self.mask_center
        if mc is None:
            self.get_logger().warn('Object lost during tilt – re-searching')
            self._transition(find_state)
            return
        h_err = mc.x - CENTER_X
        v_err = mc.y - CENTER_Y
        self.get_logger().info(
            f'[{self.state.name}] h_err={h_err:+.0f}  v_err={v_err:+.0f}  '
            f'tilt={self._cmd_tilt:.3f} rad',
            throttle_duration_sec=0.3)
        if abs(h_err) > RECENTER_TOL:
            self.get_logger().warn(f'Horizontal drift ({h_err:+.0f} px) – re-searching')
            self._transition(find_state)
            return
        if abs(v_err) < VERTICAL_TOL:
            self.pose_timestamp = None
            self._transition(next_state)
        else:
            self._set_pan_tilt(self._cmd_pan, self._cmd_tilt - v_err * TILT_GAIN)

    def _do_wait_pose(self, next_state: State):
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
                self.get_logger().error('Timed out waiting for /object_pose.')
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
                self.get_logger().error(
                    f'Speech timeout — no valid pairs received.')
                if self._speech_proc is not None:
                    self._speech_proc.terminate()
                    self._speech_proc = None
                self._transition(State.ERROR)
            else:
                self.get_logger().info(
                    f'Listening for placement pairs... ({elapsed:.0f}s / {SPEECH_TIMEOUT:.0f}s)',
                    throttle_duration_sec=5.0)

        # ── FIND_BLOCK ────────────────────────────────────────────────────────
        elif self.state == State.FIND_BLOCK:
            self.get_logger().info(
                f'[Pair {self._block_idx + 1}/{len(self._block_queue)}] '
                f'Searching for [{self._current_block_color}]',
                throttle_duration_sec=2.0)
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
            # On grasp success: switch vision to this pair's specific bin color.
            if self.state == State.FIND_BIN:
                self._grasp_retries = 0
                self._set_pan_tilt(0.0, 0.0)
                self._switch_vision(self._current_bin_color)

        # ── FIND_BIN ──────────────────────────────────────────────────────────
        elif self.state == State.FIND_BIN:
            self.get_logger().info(
                f'Searching for [{self._current_bin_color}] bin',
                throttle_duration_sec=2.0)
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
                success_state=State.FIND_BLOCK,
                fail_state=State.ERROR,
                mode='place',
            )
            # On success: advance to next pair (→ FIND_BLOCK) or finish (→ DONE).
            # Both transitions happen within this single _loop tick.
            if self.state == State.FIND_BLOCK:
                self._advance_to_next_pair()

        # ── DONE ─────────────────────────────────────────────────────────────
        elif self.state == State.DONE:
            if elapsed < 0.1:
                self.get_logger().info('=' * 65)
                self.get_logger().info('Task 5 complete!')
                self.get_logger().info(
                    f'  Placed {len(self._block_queue)} pair(s): {self._block_queue}')
                self.get_logger().info('=' * 65)

        # ── ERROR ─────────────────────────────────────────────────────────────
        elif self.state == State.ERROR:
            if elapsed < 0.1:
                idx = self._block_idx
                pair_str = (str(self._block_queue[idx])
                            if idx < len(self._block_queue) else 'n/a')
                self.get_logger().error(
                    f'Task 5 ended in ERROR at pair '
                    f'{idx + 1}/{len(self._block_queue)} {pair_str}')


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = Task5Node()
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
