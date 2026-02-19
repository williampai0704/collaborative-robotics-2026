#!/usr/bin/env python3
"""
Spin the TidyBot in a circle until a stop command is received.

Usage:
    # Terminal 1: Start spinning
    ros2 run tidybot_bringup spin_robot.py --ros-args -p angular_speed:=0.5
    
    # Terminal 2: Stop spinning
    ros2 topic pub /stop_spin std_msgs/msg/Bool "{data: true}" --once
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np  # ✨ NEW: For angle normalization


class SpinRobot(Node):
    def __init__(self):
        super().__init__('spin_robot')

        self.declare_parameter('angular_speed', 0.3)
        
        self.should_stop = False
        self.odom_received = False
        
        # ✨ NEW: Track actual heading (corrected for frame offset)
        self.current_theta = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        self.stop_sub = self.create_subscription(
            Bool, '/stop_spin', self.stop_callback, 10
        )

        self.get_logger().info('Waiting for odometry...')

    def odom_callback(self, msg):
        # ✨ UPDATED: Extract orientation and correct for frame offset
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to yaw
        odom_theta = 2.0 * np.arctan2(qz, qw)
        
        # ✨ CRITICAL: Correct for MuJoCo coordinate frame offset
        # (same as trajectory_tracking.py)
        self.current_theta = odom_theta - np.pi / 2
        
        # ✨ NEW: Normalize to [-π, π]
        self.current_theta = np.arctan2(
            np.sin(self.current_theta), 
            np.cos(self.current_theta)
        )
        
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f'Odometry connected! Initial heading: {self.current_theta:.3f} rad '
                f'({np.degrees(self.current_theta):.1f} deg)'
            )

    def stop_callback(self, msg):
        if msg.data:
            self.get_logger().info('Stop signal received!')
            self.should_stop = True

    def run_spin(self):
        # Wait for connection
        timeout = 5.0
        import time
        start = time.time()
        while not self.odom_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.odom_received:
            self.get_logger().error('No odometry received! Is the base node running?')
            return False

        angular_speed = self.get_parameter('angular_speed').value
        
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Starting to spin at {angular_speed:.2f} rad/s')
        self.get_logger().info(f'                    ({np.degrees(angular_speed):.1f} deg/s)')
        self.get_logger().info('Publish to /stop_spin to stop')
        self.get_logger().info('=' * 50)

        vel = Twist()
        vel.angular.z = angular_speed

        # ✨ NEW: Track rotation for logging
        start_theta = self.current_theta
        last_log_time = time.time()

        rate = self.create_rate(20)  # 20 Hz
        while rclpy.ok() and not self.should_stop:
            self.cmd_vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # ✨ NEW: Log progress every second
            if time.time() - last_log_time > 1.0:
                # Calculate total rotation (handling wrap-around)
                delta_theta = self.current_theta - start_theta
                delta_theta = np.arctan2(np.sin(delta_theta), np.cos(delta_theta))
                
                self.get_logger().info(
                    f'Current heading: {self.current_theta:.2f} rad '
                    f'({np.degrees(self.current_theta):.0f}°) | '
                    f'Rotated: {np.degrees(delta_theta):.0f}°'
                )
                last_log_time = time.time()
            
        # Stop the robot
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)
        
        # ✨ NEW: Calculate total rotation
        total_rotation = self.current_theta - start_theta
        total_rotation = np.arctan2(np.sin(total_rotation), np.cos(total_rotation))
        
        self.get_logger().info('')
        self.get_logger().info(f'Robot stopped! Total rotation: {np.degrees(total_rotation):.1f}°')
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