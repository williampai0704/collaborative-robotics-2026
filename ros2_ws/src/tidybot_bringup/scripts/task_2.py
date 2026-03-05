#!/usr/bin/env python3
"""
Task 2: Pick up {block_color} block and put it in {bin_color} bin

Orchestrates the existing motion and vision modules via ROS2 topics.

─── Vision nodes to run ───────────────────────────────────────────────────────
Phase 1 (block):
    ros2 run tidybot_bringup find_center.py     --ros-args -p target_color:=yellow
    ros2 run tidybot_bringup obj_dist.py        --ros-args -p target_color:=yellow
    ros2 run tidybot_bringup simple_pose_fit.py --ros-args -p target_color:=yellow

Phase 2 (bin) — restart vision nodes with new color when task_2 publishes
                /vision/target_color, OR run a second namespaced set:
    ros2 run tidybot_bringup find_center.py     --ros-args -p target_color:=blue
    ros2 run tidybot_bringup obj_dist.py        --ros-args -p target_color:=blue
    ros2 run tidybot_bringup simple_pose_fit.py --ros-args -p target_color:=blue

─── Topics ────────────────────────────────────────────────────────────────────
Subscribes:
  /mask_center                  geometry_msgs/Point    ← find_center
  /vision/object_distance       std_msgs/Float32       ← obj_dist
  /object_pose                  geometry_msgs/PoseStamped ← simple_pose_fit
  /manipulation/grasp_result    std_msgs/Bool          ← manipulation module
  /manipulation/place_result    std_msgs/Bool          ← manipulation module

Publishes:
  /cmd_vel                      geometry_msgs/Twist    → base motion
  /camera/pan_tilt_cmd          std_msgs/Float64MultiArray → camera tilt
  /vision/target_color          std_msgs/String        → signal vision nodes to switch color
  /manipulation/grasp_target    geometry_msgs/PoseStamped → pseudo manipulation
  /manipulation/place_target    geometry_msgs/PoseStamped → pseudo manipulation

─── State machine ─────────────────────────────────────────────────────────────
  INIT
    ↓ (vision online)
  FIND_BLOCK       spin until block centred horizontally
    ↓ (h_err < HORIZONTAL_TOL)
  APPROACH_BLOCK   drive forward; correct heading; stop at APPROACH_DIST
    ↑ (drift > RECENTER_TOL)  → back to FIND_BLOCK
    ↓ (distance ≤ 0.35 m)
  TILT_BLOCK       adjust camera tilt for vertical centering
    ↓ (v_err < VERTICAL_TOL)
  WAIT_BLOCK_POSE  wait for fresh /object_pose from simple_pose_fit
    ↓
  GRASP            send to /manipulation/grasp_target; wait for result
    ↑ (failed, retries left) → back to WAIT_BLOCK_POSE
    ↓ (success)
  FIND_BIN         publish /vision/target_color=bin_color; spin to find bin
    ↓
  APPROACH_BIN     same approach logic for bin
    ↑ (drift)      → back to FIND_BIN
    ↓
  TILT_BIN         centre bin vertically
    ↓
  WAIT_BIN_POSE    wait for fresh /object_pose
    ↓
  PLACE            send to /manipulation/place_target; wait for result
    ↓
  DONE / ERROR

Usage:
    ros2 run tidybot_bringup task_2.py
    ros2 run tidybot_bringup task_2.py \\
        --ros-args -p block_color:=yellow -p bin_color:=blue
"""

import time
from enum import Enum, auto
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros

from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Bool, Float32, String, Float64MultiArray

from scipy.spatial.transform import Rotation as ScipyRotation

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

APPROACH_DIST   = 0.35  # m   — stop driving when object is this close
APPROACH_SPEED  = 0.12  # m/s — forward speed during approach
SPIN_SPEED      = 0.3   # rad/s
ANGULAR_GAIN    = 0.003 # rad/s per px of horizontal error (heading correction)

TILT_GAIN       = 0.001 # rad / px
TILT_MIN        = -1.2  # rad
TILT_MAX        =  0.5  # rad

POSE_WAIT_TIMEOUT  = 8.0  # s
RESULT_TIMEOUT     = 15.0 # s — max wait for manipulation result
MAX_GRASP_RETRIES  = 3

# =============================================================================
# States
# =============================================================================

class State(Enum):
    INIT           = auto()
    # ── Block ─────────────────────────────
    FIND_BLOCK     = auto()   # spin to centre block horizontally
    APPROACH_BLOCK = auto()   # drive forward; stop when close enough
    TILT_BLOCK     = auto()   # centre block vertically in camera
    WAIT_BLOCK_POSE= auto()   # wait for fresh /object_pose
    GRASP          = auto()   # send grasp target; wait for result
    # ── Bin ───────────────────────────────
    FIND_BIN       = auto()   # spin to centre bin horizontally
    APPROACH_BIN   = auto()   # drive forward; stop when close enough
    TILT_BIN       = auto()   # centre bin vertically in camera
    WAIT_BIN_POSE  = auto()   # wait for fresh /object_pose
    PLACE          = auto()   # send place target; wait for result
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

        self.block_color: str = self.get_parameter('block_color').value
        self.bin_color:   str = self.get_parameter('bin_color').value

        # ── Publishers ──────────────────────────────────────────────────────
        self.cmd_vel_pub    = self.create_publisher(Twist,             '/cmd_vel',                   10)
        self.pan_tilt_pub   = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd',       10)
        self.color_pub      = self.create_publisher(String,            '/vision/target_color',       10)
        self.grasp_pub      = self.create_publisher(PoseStamped,       '/manipulation/grasp_target', 10)
        self.place_pub      = self.create_publisher(PoseStamped,       '/manipulation/place_target', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Point,       '/mask_center',               self._mask_center_cb,    10)
        self.create_subscription(Float32,     '/vision/object_distance',    self._obj_dist_cb,       10)
        self.create_subscription(PoseStamped, '/object_pose',               self._object_pose_cb,    10)
        self.create_subscription(Bool,        '/manipulation/grasp_result', self._grasp_result_cb,   10)
        self.create_subscription(Bool,        '/manipulation/place_result', self._place_result_cb,   10)

        # ── TF2 ─────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Shared state ─────────────────────────────────────────────────────
        self.mask_center:      Optional[Point]       = None
        self.object_distance:  Optional[float]       = None
        self.object_pose:      Optional[PoseStamped] = None
        self.pose_timestamp:   Optional[float]       = None

        self.grasp_result:     Optional[bool]        = None
        self.place_result:     Optional[bool]        = None

        self._cmd_pan:  float = 0.0
        self._cmd_tilt: float = 0.0
        self._grasp_retries:  int = 0

        # ── State machine ────────────────────────────────────────────────────
        self.state       = State.INIT
        self.state_start = time.time()

        self.create_timer(0.05, self._loop)  # 20 Hz

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Task 2: Pick [{self.block_color}] block → [{self.bin_color}] bin')
        self.get_logger().info('Waiting for vision nodes …')
        self.get_logger().info('=' * 60)

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _mask_center_cb(self, msg: Point):
        self.mask_center = msg

    def _obj_dist_cb(self, msg: Float32):
        self.object_distance = float(msg.data)

    def _object_pose_cb(self, msg: PoseStamped):
        self.object_pose   = msg
        self.pose_timestamp = time.time()

    def _grasp_result_cb(self, msg: Bool):
        self.grasp_result = msg.data

    def _place_result_cb(self, msg: Bool):
        self.place_result = msg.data

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

    def _set_target_color(self, color: str):
        """Signal vision nodes to switch to a new target color."""
        msg = String()
        msg.data = color
        self.color_pub.publish(msg)
        self.get_logger().info(f'[Vision] target_color → {color}  '
                               f'(restart vision nodes with this color if needed)')

    def _transition(self, new_state: State):
        self.get_logger().info(f'[State] {self.state.name} → {new_state.name}')
        self.state       = new_state
        self.state_start = time.time()

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
        """Generic approach logic: drive forward while correcting heading.

        Returns to find_state if drift is too large.
        Advances to tilt_state when close enough.
        """
        mc   = self.mask_center
        dist = self.object_distance

        if mc is None:
            # Lost the object — stop and re-search
            self._stop()
            self.get_logger().warn('Object lost during approach – re-searching')
            self._transition(find_state)
            return

        h_err = mc.x - CENTER_X

        # Re-centre if heading has drifted
        if abs(h_err) > RECENTER_TOL:
            self._stop()
            self.get_logger().warn(
                f'Heading drift ({h_err:+.0f} px) – re-centring')
            self._transition(find_state)
            return

        # Check distance
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
        v_err = mc.y - CENTER_Y   # positive → object below image centre

        self.get_logger().info(
            f'[{self.state.name}] h_err={h_err:+.0f}  v_err={v_err:+.0f}  '
            f'tilt={self._cmd_tilt:.3f} rad',
            throttle_duration_sec=0.3)

        # If the robot has drifted horizontally during tilt, go back to find
        if abs(h_err) > RECENTER_TOL:
            self.get_logger().warn(
                f'Horizontal drift ({h_err:+.0f} px) during tilt – re-searching')
            self._transition(find_state)
            return

        if abs(v_err) < VERTICAL_TOL:
            self.get_logger().info(
                f'Object centred vertically (v_err={v_err:+.0f} px)')
            self.pose_timestamp = None   # invalidate stale pose
            self._transition(next_state)
        else:
            # Object below centre (v_err>0) → tilt down (negative)
            new_tilt = self._cmd_tilt - v_err * TILT_GAIN
            self._set_pan_tilt(self._cmd_pan, new_tilt)

    def _do_wait_pose(self, next_state: State):
        """Wait for simple_pose_fit to publish a pose after the robot is positioned."""
        if self._elapsed() < 0.3:
            return  # stabilisation pause

        if (self.pose_timestamp is not None and
                self.pose_timestamp >= self.state_start):
            self.get_logger().info('Fresh pose received from simple_pose_fit')
            self._transition(next_state)
            return

        if self._elapsed() > POSE_WAIT_TIMEOUT:
            self.get_logger().error(
                'Timed out waiting for /object_pose. '
                'Is simple_pose_fit.py running with the correct color?')
            self._transition(State.ERROR)
        else:
            self.get_logger().info(
                f'Waiting for pose … ({self._elapsed():.1f} / {POSE_WAIT_TIMEOUT:.0f} s)',
                throttle_duration_sec=1.0)

    # =========================================================================
    # State machine
    # =========================================================================

    def _loop(self):  # noqa: C901
        elapsed = self._elapsed()

        # ── INIT ─────────────────────────────────────────────────────────────
        if self.state == State.INIT:
            if self.mask_center is not None:
                self._set_pan_tilt(0.0, 0.0)
                self._set_target_color(self.block_color)
                self.get_logger().info(
                    f'Vision online – searching for [{self.block_color}] block')
                self._transition(State.FIND_BLOCK)
            elif elapsed > 15.0:
                self.get_logger().error(
                    'Timed out waiting for /mask_center. '
                    'Is find_center.py running?')

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
            if elapsed < 0.1:
                # Transform and publish grasp target once on entry
                pose_base = self._transform_to_base(self.object_pose)
                if pose_base is None:
                    self.get_logger().error('TF transform failed for grasp pose')
                    self._transition(State.ERROR)
                    return

                self.grasp_result = None   # clear previous result
                self.grasp_pub.publish(pose_base)

                p = pose_base.pose.position
                self.get_logger().info(
                    f'Grasp target sent (attempt {self._grasp_retries + 1}/'
                    f'{MAX_GRASP_RETRIES}): '
                    f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m')

            # Wait for manipulation result
            if self.grasp_result is None:
                if elapsed > RESULT_TIMEOUT:
                    self.get_logger().error('Grasp result timed out')
                    self._transition(State.ERROR)
                return

            if self.grasp_result:
                self.get_logger().info('Grasp SUCCESS → searching for bin')
                self._grasp_retries = 0
                self._set_pan_tilt(0.0, 0.0)          # reset camera
                self._set_target_color(self.bin_color) # switch vision to bin color
                self._transition(State.FIND_BIN)
            else:
                self._grasp_retries += 1
                if self._grasp_retries >= MAX_GRASP_RETRIES:
                    self.get_logger().error(
                        f'Grasp failed after {MAX_GRASP_RETRIES} attempts')
                    self._transition(State.ERROR)
                else:
                    self.get_logger().warn(
                        f'Grasp FAILED – retrying pose estimation '
                        f'({self._grasp_retries}/{MAX_GRASP_RETRIES})')
                    self.pose_timestamp = None  # force fresh pose
                    self._transition(State.WAIT_BLOCK_POSE)

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
            if elapsed < 0.1:
                pose_base = self._transform_to_base(self.object_pose)
                if pose_base is None:
                    self.get_logger().error('TF transform failed for place pose')
                    self._transition(State.ERROR)
                    return

                self.place_result = None
                self.place_pub.publish(pose_base)

                p = pose_base.pose.position
                self.get_logger().info(
                    f'Place target sent: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m')

            if self.place_result is None:
                if elapsed > RESULT_TIMEOUT:
                    self.get_logger().error('Place result timed out')
                    self._transition(State.ERROR)
                return

            if self.place_result:
                self.get_logger().info('Place SUCCESS')
                self._transition(State.DONE)
            else:
                self.get_logger().error('Place FAILED')
                self._transition(State.ERROR)

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
