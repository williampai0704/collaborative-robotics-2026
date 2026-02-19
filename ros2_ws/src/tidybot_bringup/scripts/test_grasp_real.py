#!/usr/bin/env python3
"""
Test TidyBot2 Grippers + Grasp Detection using GripperController.

Uses the reusable GripperController from tidybot_control, which publishes
directly to Interbotix SDK topics (SDK mode) or via gripper_wrapper_node
(sim mode).

Usage:
    # First, launch the real robot:
    ros2 launch tidybot_bringup real.launch.py

    # Then run this test (SDK mode, default):
    ros2 run tidybot_bringup test_grasp_real.py

    # Or use sim mode (requires gripper_wrapper_node):
    ros2 run tidybot_bringup test_grasp_real.py --ros-args -p mode:=sim
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tidybot_control.gripper_controller import GripperController


class TestGraspReal(Node):
    """Test grippers and grasp detection on real hardware."""

    def __init__(self):
        super().__init__('test_grasp_real')

        # Declare parameters
        self.declare_parameter('mode', 'sdk')
        self.declare_parameter('pressure', 1.0)

        mode = self.get_parameter('mode').get_parameter_value().string_value
        pressure = self.get_parameter('pressure').get_parameter_value().double_value

        # Create gripper controller
        self.gripper = GripperController(self, mode=mode, pressure=pressure)

        # Track arm connectivity via joint states
        self.right_connected = False
        self.left_connected = False

        self.right_joint_sub = self.create_subscription(
            JointState, '/right_arm/joint_states', self._right_cb, 10
        )
        self.left_joint_sub = self.create_subscription(
            JointState, '/left_arm/joint_states', self._left_cb, 10
        )

        self.get_logger().info(f'GripperController mode={mode}, pressure={pressure}')
        self.get_logger().info('Waiting for joint states...')

    def _right_cb(self, msg):
        if not self.right_connected:
            self.right_connected = True
            self.get_logger().info('Connected to right_arm!')

    def _left_cb(self, msg):
        if not self.left_connected:
            self.left_connected = True
            self.get_logger().info('Connected to left_arm!')

    def wait_for_connection(self, timeout=5.0):
        """Wait for at least one arm to be connected."""
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.right_connected or self.left_connected:
                return True
        return False

    def run_test(self):
        if not self.wait_for_connection():
            self.get_logger().error('No joint states received!')
            self.get_logger().error('Make sure real.launch.py is running:')
            self.get_logger().error('  ros2 launch tidybot_bringup real.launch.py')
            return False

        sides = []
        if self.right_connected:
            sides.append('right')
        if self.left_connected:
            sides.append('left')

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('GRIPPER + GRASP DETECTION TEST')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Right arm: {"Connected" if self.right_connected else "Not found"}')
        self.get_logger().info(f'Left arm:  {"Connected" if self.left_connected else "Not found"}')
        self.get_logger().info('')

        # --- Step 1: Open all connected grippers ---
        self.get_logger().info('[Step 1/4] Opening grippers...')
        if len(sides) == 2:
            self.gripper.open_both(duration=2.5)
        else:
            self.gripper.open(sides[0], duration=2.5)
        input('\n>>> Grippers should be OPEN. Press Enter to continue...\n')

        # --- Step 2: Close grippers (empty) and check grasp ---
        self.get_logger().info('[Step 2/4] Closing grippers (empty — no object)...')
        for side in sides:
            grasped = self.gripper.close_and_check(side, duration=2.5)
            self.get_logger().info(f'  {side}: grasped={grasped}  (expected: False)')
        input('\n>>> Grippers should be CLOSED with nothing held. Press Enter to continue...\n')

        # --- Step 3: Open grippers, let user place an object ---
        self.get_logger().info('[Step 3/4] Opening grippers — place an object between the fingers...')
        if len(sides) == 2:
            self.gripper.open_both(duration=2.5)
        else:
            self.gripper.open(sides[0], duration=2.5)
        input('\n>>> Place an object between the gripper fingers, then press Enter to CLOSE...\n')

        # --- Step 4: Close grippers on the object and check grasp ---
        self.get_logger().info('[Step 4/4] Closing grippers on object...')
        for side in sides:
            grasped = self.gripper.close_and_check(side, duration=2.5)
            self.get_logger().info(f'  {side}: grasped={grasped}  (expected: True)')

        # Final open
        self.get_logger().info('')
        self.get_logger().info('Opening grippers to release...')
        if len(sides) == 2:
            self.gripper.open_both(duration=2.5)
        else:
            self.gripper.open(sides[0], duration=2.5)

        self.get_logger().info('')
        self.get_logger().info('Grasp test complete!')
        return True


def main():
    rclpy.init()
    node = TestGraspReal()
    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
