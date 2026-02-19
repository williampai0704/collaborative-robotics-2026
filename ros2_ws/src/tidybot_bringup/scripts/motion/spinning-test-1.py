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
# ✨ NEW: Import Bool message for stop signal
from std_msgs.msg import Bool


class SpinRobot(Node):
    def __init__(self):
        super().__init__('spin_robot')

        # ✨ NEW: Declare parameter for angular speed
        self.declare_parameter('angular_speed', 0.3)  # rad/s (default ~17 deg/s)
        
        # ✨ NEW: State variables
        self.should_stop = False
        self.odom_received = False

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # ✨ NEW: Subscriber for stop signal
        self.stop_sub = self.create_subscription(
            Bool, '/stop_spin', self.stop_callback, 10
        )

        self.get_logger().info('Waiting for odometry...')

    def odom_callback(self, msg):
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('Odometry connected!')

    # ✨ NEW: Callback to handle stop signal
    def stop_callback(self, msg):
        if msg.data:
            self.get_logger().info('Stop signal received!')
            self.should_stop = True

    # ✨ NEW: Main spinning loop
    def run_spin(self):
        # Wait for connection
        timeout = 5.0
        import time
        start = time.time()
        while not self.odom_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.odom_received:
            self.get_logger().error('No odometry received! Is phoenix6_base_node running?')
            return False

        # Get the angular speed parameter
        angular_speed = self.get_parameter('angular_speed').value
        
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Starting to spin at {angular_speed:.2f} rad/s')
        self.get_logger().info('Publish to /stop_spin to stop')
        self.get_logger().info('=' * 50)

        vel = Twist()
        vel.angular.z = angular_speed  # Positive = counter-clockwise

        # Keep spinning until stop signal
        rate = self.create_rate(20)  # 20 Hz
        while rclpy.ok() and not self.should_stop:
            self.cmd_vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.01)
            
        # Stop the robot
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)
        
        self.get_logger().info('Robot stopped!')
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