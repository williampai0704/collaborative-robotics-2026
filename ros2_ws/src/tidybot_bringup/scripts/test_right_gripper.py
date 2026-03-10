#!/usr/bin/env python3
"""
Test right gripper with interactive open/close via Enter key.
Continuously prints gripper joint state.

Usage:
    ros2 launch tidybot_bringup real.launch.py
    ros2 run tidybot_bringup test_right_gripper.py
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tidybot_control.gripper_controller import GripperController


class TestRightGripper(Node):
    def __init__(self):
        super().__init__('test_right_gripper')

        self.declare_parameter('mode', 'sdk')
        self.declare_parameter('pressure', 1.0)

        mode = self.get_parameter('mode').get_parameter_value().string_value
        pressure = self.get_parameter('pressure').get_parameter_value().double_value

        self.gripper = GripperController(self, mode=mode, pressure=pressure)
        self.is_open = True  # start open
        self.gripper_positions = {}

        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10
        )

        self.get_logger().info(f'Mode={mode}, pressure={pressure}')
        self.get_logger().info('Waiting for right gripper joint states...')

    def _joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if 'right' in name and 'finger' in name:
                self.gripper_positions[name] = pos

        if self.gripper_positions:
            parts = [f'{n}: {p:.4f}' for n, p in sorted(self.gripper_positions.items())]
            print(f'\r  Gripper joints: {" | ".join(parts)}', end='', flush=True)

    def run(self):
        # Open gripper to start
        self.get_logger().info('Opening right gripper...')
        self.gripper.open('right', duration=3.0)
        self.is_open = True

        print('\n')
        print('=' * 50)
        print('  RIGHT GRIPPER INTERACTIVE TEST')
        print('  Press Enter to toggle open/close')
        print('  Press Ctrl+C to exit')
        print('=' * 50)
        print()

        # Spin in background so joint states keep updating
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()

        try:
            while True:
                state = 'OPEN' if self.is_open else 'CLOSED'
                next_action = 'CLOSE' if self.is_open else 'OPEN'
                input(f'\n  [{state}] Press Enter to {next_action} >>> ')

                if self.is_open:
                    print('  Closing gripper...')
                    self.gripper.close('right', duration=3.0)
                    self.is_open = False
                else:
                    print('  Opening gripper...')
                    self.gripper.open('right', duration=3.0)
                    self.is_open = True
        except (KeyboardInterrupt, EOFError):
            print('\n\nExiting.')


def main():
    rclpy.init()
    node = TestRightGripper()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
