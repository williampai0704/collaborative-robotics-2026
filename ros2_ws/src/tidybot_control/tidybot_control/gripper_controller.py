#!/usr/bin/env python3
"""
Gripper Controller - Importable gripper control for TidyBot2.

This module provides a GripperController class that can be imported by any
test script or node to control the grippers.

Two modes are supported:
    - 'sim': Publishes to /right_gripper/cmd and /left_gripper/cmd
             (Float64MultiArray, 0-1 normalized). Requires gripper_wrapper_node
             for real hardware, or works directly with MuJoCo simulation.
    - 'sdk': Publishes directly to Interbotix SDK topics using PWM control.
             Works with real hardware without gripper_wrapper_node.

Usage:
    from tidybot_control.gripper_controller import GripperController

    # For simulation or when gripper_wrapper_node is running:
    gripper = GripperController(self, mode='sim')

    # For direct real hardware control:
    gripper = GripperController(self, mode='sdk')

    # Control grippers:
    gripper.open('right')
    gripper.close('left')
    gripper.open_both()
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointSingleCommand


# PWM pressure limits (from Interbotix SDK)
GRIPPER_PRESSURE_LOWER = 150   # Minimum PWM for movement
GRIPPER_PRESSURE_UPPER = 350   # Maximum PWM (avoid motor overload)

# Finger position limits (meters, from MuJoCo bridge)
FINGER_OPEN_POS = 0.037    # Fully open finger position
FINGER_CLOSED_POS = 0.015  # Fully closed finger position

# Finger joint names per side
FINGER_JOINT_NAMES = {
    'right': ['right_left_finger', 'right_right_finger'],
    'left': ['left_left_finger', 'left_right_finger'],
}


class GripperController:
    """
    Gripper controller that can be used by any ROS2 node.

    Supports both simulation-compatible mode and direct SDK mode.
    """

    def __init__(self, node: Node, mode: str = 'sdk', pressure: float = 1.0):
        """
        Initialize the gripper controller.

        Args:
            node: The ROS2 node instance to create publishers on.
            mode: 'sim' for simulation-compatible topics, 'sdk' for direct Interbotix SDK.
            pressure: Gripper pressure from 0.0 (gentle) to 1.0 (strong). Only used in SDK mode.
        """
        self.node = node
        self.mode = mode
        self.pressure = pressure

        # Calculate PWM value for SDK mode
        self.pwm_value = GRIPPER_PRESSURE_LOWER + pressure * (
            GRIPPER_PRESSURE_UPPER - GRIPPER_PRESSURE_LOWER
        )

        if mode == 'sim':
            # Simulation-compatible publishers
            self.right_pub = node.create_publisher(
                Float64MultiArray, '/right_gripper/cmd', 10
            )
            self.left_pub = node.create_publisher(
                Float64MultiArray, '/left_gripper/cmd', 10
            )
        else:
            # Direct Interbotix SDK publishers
            self.right_pub = node.create_publisher(
                JointSingleCommand, '/right_arm/commands/joint_single', 10
            )
            self.left_pub = node.create_publisher(
                JointSingleCommand, '/left_arm/commands/joint_single', 10
            )

        # Subscribe to /joint_states for grasp detection
        self.finger_positions = {}
        self.joint_state_sub = node.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )

        self.node.get_logger().debug(f'GripperController initialized (mode={mode})')

    def _publish_sim(self, side: str, position: float):
        """Publish to simulation-compatible topic."""
        msg = Float64MultiArray()
        msg.data = [float(position)]

        if side == 'right':
            self.right_pub.publish(msg)
        else:
            self.left_pub.publish(msg)

    def _publish_sdk(self, side: str, pwm: float):
        """Publish to Interbotix SDK topic."""
        cmd = JointSingleCommand()
        cmd.cmd = float(pwm)

        if side == 'right':
            cmd.name = 'right_gripper'
            self.right_pub.publish(cmd)
        else:
            cmd.name = 'left_gripper'
            self.left_pub.publish(cmd)

    def set_position(self, side: str, position: float, duration: float = 2.0):
        """
        Set gripper to a specific position.

        Args:
            side: 'right' or 'left'
            position: 0.0 (fully open) to 1.0 (fully closed)
            duration: Time to publish the command (seconds)
        """
        position = max(0.0, min(1.0, position))

        if self.mode == 'sim':
            # Publish normalized position
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self._publish_sim(side, position)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)
        else:
            # Convert to PWM: 0.0 -> +pwm (open), 1.0 -> -pwm (close)
            pwm = self.pwm_value - position * (2 * self.pwm_value)
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self._publish_sdk(side, pwm)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)

            # Stop gripper
            self._publish_sdk(side, 0.0)

    def open(self, side: str, duration: float = 2.0):
        """
        Open the gripper.

        Args:
            side: 'right' or 'left'
            duration: Time to publish the command (seconds)
        """
        self.set_position(side, 0.0, duration)

    def close(self, side: str, duration: float = 2.0):
        """
        Close the gripper.

        Args:
            side: 'right' or 'left'
            duration: Time to publish the command (seconds)
        """
        self.set_position(side, 1.0, duration)

    def open_both(self, duration: float = 2.0):
        """Open both grippers simultaneously."""
        if self.mode == 'sim':
            msg = Float64MultiArray()
            msg.data = [0.0]
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self.right_pub.publish(msg)
                self.left_pub.publish(msg)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)
        else:
            pwm = self.pwm_value  # Positive = open
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self._publish_sdk('right', pwm)
                self._publish_sdk('left', pwm)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)
            # Stop grippers
            self._publish_sdk('right', 0.0)
            self._publish_sdk('left', 0.0)

    def close_both(self, duration: float = 2.0):
        """Close both grippers simultaneously."""
        if self.mode == 'sim':
            msg = Float64MultiArray()
            msg.data = [1.0]
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self.right_pub.publish(msg)
                self.left_pub.publish(msg)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)
        else:
            pwm = -self.pwm_value  # Negative = close
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self._publish_sdk('right', pwm)
                self._publish_sdk('left', pwm)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)
            # Stop grippers
            self._publish_sdk('right', 0.0)
            self._publish_sdk('left', 0.0)

    def _joint_state_callback(self, msg: JointState):
        """Store latest finger joint positions from /joint_states."""
        for i, name in enumerate(msg.name):
            if 'finger' in name and i < len(msg.position):
                self.finger_positions[name] = msg.position[i]

    def check_grasp(self, side: str, threshold: float = 0.003) -> bool:
        """
        Check if the gripper successfully grasped an object.

        Compares actual finger position against the fully-closed position.
        If the fingers stopped short of fully closed (blocked by an object),
        the grasp is considered successful.

        Call this after close() or set_position() with position=1.0.

        Args:
            side: 'right' or 'left'
            threshold: Minimum distance (meters) above closed position to
                       count as a successful grasp. Default 0.003m (3mm).

        Returns:
            True if an object is detected between the fingers, False otherwise.
        """
        finger_names = FINGER_JOINT_NAMES.get(side)
        if finger_names is None:
            self.node.get_logger().warn(f"Invalid side '{side}' for check_grasp")
            return False

        # Spin briefly to ensure we have fresh joint state data
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.01)

        # Read actual finger positions
        positions = []
        for fname in finger_names:
            pos = self.finger_positions.get(fname)
            if pos is None:
                self.node.get_logger().warn(
                    f"No joint state for '{fname}' — is /joint_states being published?"
                )
                return False
            positions.append(pos)

        # Average of both finger positions
        avg_pos = sum(positions) / len(positions)

        # If fingers stopped above (closed + threshold), something is between them
        grasped = avg_pos > (FINGER_CLOSED_POS + threshold)

        self.node.get_logger().info(
            f'Grasp check ({side}): finger_pos={avg_pos:.4f}m, '
            f'closed={FINGER_CLOSED_POS}m, threshold={threshold}m, '
            f'grasped={grasped}'
        )
        return grasped

    def close_and_check(self, side: str, duration: float = 2.0,
                        threshold: float = 0.003) -> bool:
        """
        Close the gripper and check if an object was grasped.

        Args:
            side: 'right' or 'left'
            duration: Time to publish close command (seconds).
            threshold: Grasp detection threshold in meters.

        Returns:
            True if an object is detected between the fingers, False otherwise.
        """
        self.close(side, duration)
        return self.check_grasp(side, threshold)
