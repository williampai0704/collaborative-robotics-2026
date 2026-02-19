#!/usr/bin/env python3
"""
Move the TidyBot in a straight line until a stop command is received.

Usage:
    # Terminal 1: Start moving forward
    ros2 run tidybot_bringup move_straight.py --ros-args -p linear_speed:=0.2
    
    # Move backward
    ros2 run tidybot_bringup move_straight.py --ros-args -p linear_speed:=-0.2
    
    # Terminal 2: Stop moving
    ros2 topic pub /stop_move std_msgs/msg/Bool "{data: true}" --once
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import time


class MoveStraight(Node):
    def __init__(self):
        super().__init__('move_straight')

        # Declare parameters
        self.declare_parameter('linear_speed', 0.2)  # m/s (positive = forward)
        
        # State variables
        self.should_stop = False
        self.odom_received = False
        
        # Position tracking
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Subscriber for stop signal
        self.stop_sub = self.create_subscription(
            Bool, '/stop_move', self.stop_callback, 10
        )

        self.get_logger().info('Waiting for odometry...')

    def odom_callback(self, msg):
        """Extract position and orientation from odometry."""
        # Get position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Get orientation and correct for frame offset
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        odom_theta = 2.0 * np.arctan2(qz, qw)
        
        # Correct for MuJoCo coordinate frame offset
        self.current_theta = odom_theta - np.pi / 2
        
        # Normalize to [-π, π]
        self.current_theta = np.arctan2(
            np.sin(self.current_theta), 
            np.cos(self.current_theta)
        )
        
        if not self.odom_received:
            self.odom_received = True
            # Store starting position
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(
                f'Odometry connected! Starting at: ({self.start_x:.3f}, {self.start_y:.3f}), '
                f'heading: {np.degrees(self.current_theta):.1f}°'
            )

    def stop_callback(self, msg):
        """Handle stop signal."""
        if msg.data:
            self.get_logger().info('Stop signal received!')
            self.should_stop = True

    def run_straight(self):
        """Main control loop for straight line movement."""
        # Wait for connection
        timeout = 5.0
        start = time.time()
        while not self.odom_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.odom_received:
            self.get_logger().error('No odometry received! Is the base node running?')
            return False

        linear_speed = self.get_parameter('linear_speed').value
        
        direction = "forward" if linear_speed >= 0 else "backward"
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Starting to move {direction} at {abs(linear_speed):.2f} m/s')
        self.get_logger().info('Publish to /stop_move to stop')
        self.get_logger().info('=' * 60)

        vel = Twist()
        vel.linear.x = linear_speed
        vel.angular.z = 0.0  # Keep angular velocity at zero for straight line

        # Tracking
        last_log_time = time.time()
        start_time = time.time()

        rate = self.create_rate(20)  # 20 Hz
        while rclpy.ok() and not self.should_stop:
            self.cmd_vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Log progress every second
            if time.time() - last_log_time > 1.0:
                # Calculate distance traveled
                dx = self.current_x - self.start_x
                dy = self.current_y - self.start_y
                distance = np.sqrt(dx*dx + dy*dy)
                
                elapsed = time.time() - start_time
                
                self.get_logger().info(
                    f'Time: {elapsed:.1f}s | '
                    f'Position: ({self.current_x:.3f}, {self.current_y:.3f}) | '
                    f'Distance: {distance:.3f}m | '
                    f'Heading: {np.degrees(self.current_theta):.0f}°'
                )
                last_log_time = time.time()
            
        # Stop the robot
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)
        
        # Calculate final statistics
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        total_distance = np.sqrt(dx*dx + dy*dy)
        total_time = time.time() - start_time
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Robot stopped!')
        self.get_logger().info(f'Total distance traveled: {total_distance:.3f} m')
        self.get_logger().info(f'Total time: {total_time:.1f} s')
        self.get_logger().info(f'Average speed: {total_distance/total_time:.3f} m/s')
        self.get_logger().info(f'Final position: ({self.current_x:.3f}, {self.current_y:.3f})')
        self.get_logger().info('=' * 60)
        
        return True


def main():
    rclpy.init()
    node = MoveStraight()
    try:
        node.run_straight()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()