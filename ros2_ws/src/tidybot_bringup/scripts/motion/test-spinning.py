#!/usr/bin/env python3
"""
Spin the TidyBot in place until a stop command is received.

Usage:
    # Terminal 1: Start spinning
    ros2 run tidybot_bringup spin_robot.py --ros-args -p angular_speed:=0.5

    # Optionally disable MuJoCo coordinate frame offset correction
    ros2 run tidybot_bringup spin_robot.py --ros-args -p angular_speed:=0.5 -p use_frame_offset:=false

    # Terminal 2: Stop spinning
    ros2 topic pub /stop_spin std_msgs/msg/Bool "{data: true}" --once
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import time

robot_angular_speed = 0.3
use_mujoco_frame_offset_default = True

class SpinRobot(Node):
    def __init__(self):
        super().__init__('spin_robot')

        self.declare_parameter('angular_speed', robot_angular_speed)
        self.declare_parameter('use_frame_offset', use_mujoco_frame_offset_default)

        self.should_stop = False
        self.odom_received = False

        self.current_theta = 0.0
        self.start_theta = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.stop_sub = self.create_subscription(
            Bool,
            '/stop_spin',
            self.stop_callback,
            10
        )

        self.get_logger().info('Waiting for odometry...')

    def odom_callback(self, msg):
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        odom_theta = 2.0 * np.arctan2(qz, qw)

        use_frame_offset = self.get_parameter('use_frame_offset').value
        if use_frame_offset:
            self.current_theta = odom_theta - np.pi / 2.0
        else:
            self.current_theta = odom_theta

        self.current_theta = np.arctan2(
            np.sin(self.current_theta),
            np.cos(self.current_theta)
        )

        if not self.odom_received:
            self.odom_received = True
            self.start_theta = self.current_theta

            self.get_logger().info(
                f'Odometry connected! Initial heading: {self.current_theta:.3f} rad '
                f'({np.degrees(self.current_theta):.1f} deg)'
            )

    def stop_callback(self, msg):
        if msg.data:
            self.get_logger().info('Stop signal received!')
            self.should_stop = True

    def run_spin(self):
        odom_timeout = 5.0
        start_wait = time.time()

        while not self.odom_received and (time.time() - start_wait) < odom_timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.odom_received:
            self.get_logger().error('No odometry received! Make sure the base node is running?')
            return False

        angular_speed = self.get_parameter('angular_speed').value
        use_frame_offset = self.get_parameter('use_frame_offset').value

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Starting to spin at {angular_speed:.2f} rad/s')
        self.get_logger().info(f'                    ({np.degrees(angular_speed):.1f} deg/s)')
        self.get_logger().info(f'Frame offset correction enabled: {use_frame_offset}')
        self.get_logger().info('Publish to /stop_spin to stop')
        self.get_logger().info('=' * 60)

        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = angular_speed

        start_theta = self.current_theta
        last_log_time = time.time()
        start_time = time.time()

        rate = self.create_rate(20)
        while rclpy.ok() and not self.should_stop:
            self.cmd_vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.01)

            if time.time() - last_log_time > 1.0:
                delta_theta = self.current_theta - start_theta
                delta_theta = np.arctan2(
                    np.sin(delta_theta),
                    np.cos(delta_theta)
                )

                elapsed = time.time() - start_time

                self.get_logger().info(
                    f'Time: {elapsed:.1f}s | '
                    f'Heading: {np.degrees(self.current_theta):.0f} deg | '
                    f'Rotated: {np.degrees(delta_theta):.0f} deg'
                )
                last_log_time = time.time()

        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)

        total_rotation = self.current_theta - start_theta
        total_rotation = np.arctan2(
            np.sin(total_rotation),
            np.cos(total_rotation)
        )

        total_time = time.time() - start_time
        avg_rate = 0.0
        if total_time > 1.0e-9:
            avg_rate = total_rotation / total_time

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Robot stopped!')
        self.get_logger().info(f'Total rotation: {np.degrees(total_rotation):.1f} deg')
        self.get_logger().info(f'Total time: {total_time:.1f} s')
        self.get_logger().info(f'Average rotation rate: {avg_rate:.3f} rad/s')
        self.get_logger().info('=' * 60)

        return True

def main():
    rclpy.init()
    node = SpinRobot()
    try:
        node.run_spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
