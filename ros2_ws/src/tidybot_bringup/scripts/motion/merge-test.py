#!/usr/bin/env python3
"""
Control TidyBot to move in a straight line or spin in place based on commands.

Usage:

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import numpy as np
import time

# Default motion parameters
DEFAULT_LINEAR_SPEED = 0.02  # m/s
DEFAULT_ANGULAR_SPEED = 0.3  # rad/s
DEFAULT_TIMEOUT = 10.0  # seconds
USE_MUJOCO_FRAME_OFFSET = True


class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller")

        # Declare parameters
        self.declare_parameter("linear_speed", DEFAULT_LINEAR_SPEED)
        self.declare_parameter("angular_speed", DEFAULT_ANGULAR_SPEED)
        self.declare_parameter("timeout", DEFAULT_TIMEOUT)
        self.declare_parameter("use_frame_offset", USE_MUJOCO_FRAME_OFFSET)

        # State variables
        self.current_mode = "idle"  # 'idle', 'linear', 'search'
        self.should_stop = False
        self.odom_received = False

        # Position and orientation tracking
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.start_theta = 0.0

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # Subscriber for stop signal
        self.stop_sub = self.create_subscription(
            Bool, "/stop_motion", self.stop_callback, 10
        )

        # Subscriber for motion commands
        self.command_sub = self.create_subscription(
            String, "/motion_command", self.command_callback, 10
        )

        self.get_logger().info(
            "Motion controller started. Waiting for odometry..."
        )

    def odom_callback(self, msg):
        """Extract position and orientation from odometry."""
        # Get position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Get orientation
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        odom_theta = 2.0 * np.arctan2(qz, qw)

        # Apply MuJoCo coordinate frame offset correction if enabled
        use_frame_offset = self.get_parameter("use_frame_offset").value
        if use_frame_offset:
            self.current_theta = odom_theta - np.pi / 2.0
        else:
            self.current_theta = odom_theta

        # Normalize to [-π, π]
        self.current_theta = np.arctan2(
            np.sin(self.current_theta), np.cos(self.current_theta)
        )

        if not self.odom_received:
            self.odom_received = True
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_theta = self.current_theta
            self.get_logger().info(
                f"Odometry connected! Position: ({self.start_x:.3f}, {self.start_y:.3f}), "
                f"Heading: {np.degrees(self.current_theta):.1f}°"
            )
            self.get_logger().info("")
            self.get_logger().info("Ready for commands!")
            self.get_logger().info(
                '  - Publish "linear" to /motion_command for straight motion'
            )
            self.get_logger().info(
                '  - Publish "search" to /motion_command for spinning motion'
            )
            self.get_logger().info("  - Publish true to /stop_motion to stop")

    def stop_callback(self, msg):
        """Handle stop signal."""
        if msg.data and self.current_mode != "idle":
            self.get_logger().info("Stop signal received!")
            self.should_stop = True

    def command_callback(self, msg):
        """Handle motion command."""
        command = msg.data.lower().strip()

        if command not in ["linear", "search"]:
            self.get_logger().warn(
                f'Unknown command: "{command}". Use "linear" or "search".'
            )
            return

        if self.current_mode != "idle":
            self.get_logger().warn(
                f'Already executing "{self.current_mode}" motion. '
                "Send stop command first."
            )
            return

        if not self.odom_received:
            self.get_logger().error(
                "No odometry received yet! Cannot execute motion."
            )
            return

        # Reset stop flag and update starting position/orientation
        self.should_stop = False
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_theta = self.current_theta
        self.current_mode = command

        # Execute the requested motion
        if command == "linear":
            self.execute_linear_motion()
        elif command == "search":
            self.execute_search_motion()

        # Return to idle state
        self.current_mode = "idle"

    def execute_linear_motion(self):
        """Execute straight line motion."""
        linear_speed = self.get_parameter("linear_speed").value
        timeout = self.get_parameter("timeout").value

        direction = "forward" if linear_speed >= 0 else "backward"

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"LINEAR MOTION: Moving {direction} at {abs(linear_speed):.2f} m/s"
        )
        self.get_logger().info(f"Timeout: {timeout:.1f}s")
        self.get_logger().info("=" * 60)

        vel = Twist()
        vel.linear.x = linear_speed
        vel.angular.z = 0.0

        last_log_time = time.time()
        start_time = time.time()

        rate = self.create_rate(20)  # 20 Hz
        while rclpy.ok() and not self.should_stop:
            elapsed = time.time() - start_time
            if elapsed >= timeout:
                self.get_logger().info("Timeout reached!")
                break

            self.cmd_vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.01)

            # Log progress every second
            if time.time() - last_log_time > 1.0:
                dx = self.current_x - self.start_x
                dy = self.current_y - self.start_y
                distance = np.sqrt(dx * dx + dy * dy)

                self.get_logger().info(
                    f"Time: {elapsed:.1f}s | "
                    f"Position: ({self.current_x:.3f}, {self.current_y:.3f}) | "
                    f"Distance: {distance:.3f}m | "
                    f"Heading: {np.degrees(self.current_theta):.0f}°"
                )
                last_log_time = time.time()

        # Stop the robot
        self.stop_robot()

        # Calculate final statistics
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        total_distance = np.sqrt(dx * dx + dy * dy)
        total_time = time.time() - start_time

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("LINEAR MOTION COMPLETE")
        self.get_logger().info(f"Distance traveled: {total_distance:.3f} m")
        self.get_logger().info(f"Time: {total_time:.1f} s")
        self.get_logger().info(
            f"Average speed: {total_distance / total_time:.3f} m/s"
        )
        self.get_logger().info(
            f"Final position: ({self.current_x:.3f}, {self.current_y:.3f})"
        )
        self.get_logger().info("=" * 60)

    def execute_search_motion(self):
        """Execute circular search motion (spinning in place)."""
        angular_speed = self.get_parameter("angular_speed").value
        timeout = self.get_parameter("timeout").value

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"SEARCH MOTION: Spinning at {angular_speed:.2f} rad/s"
        )
        self.get_logger().info(
            f"               ({np.degrees(angular_speed):.1f} deg/s)"
        )
        self.get_logger().info(f"Timeout: {timeout:.1f}s")
        self.get_logger().info("=" * 60)

        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = angular_speed

        last_log_time = time.time()
        start_time = time.time()

        rate = self.create_rate(20)  # 20 Hz
        while rclpy.ok() and not self.should_stop:
            elapsed = time.time() - start_time
            if elapsed >= timeout:
                self.get_logger().info("Timeout reached!")
                break

            self.cmd_vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.01)

            # Log progress every second
            if time.time() - last_log_time > 1.0:
                delta_theta = self.current_theta - self.start_theta
                delta_theta = np.arctan2(
                    np.sin(delta_theta), np.cos(delta_theta)
                )

                self.get_logger().info(
                    f"Time: {elapsed:.1f}s | "
                    f"Heading: {np.degrees(self.current_theta):.0f} deg | "
                    f"Rotated: {np.degrees(delta_theta):.0f} deg"
                )
                last_log_time = time.time()

        # Stop the robot
        self.stop_robot()

        # Calculate final statistics
        total_rotation = self.current_theta - self.start_theta
        total_rotation = np.arctan2(
            np.sin(total_rotation), np.cos(total_rotation)
        )
        total_time = time.time() - start_time
        avg_rate = total_rotation / total_time if total_time > 1e-9 else 0.0

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("SEARCH MOTION COMPLETE")
        self.get_logger().info(
            f"Total rotation: {np.degrees(total_rotation):.1f} deg"
        )
        self.get_logger().info(f"Time: {total_time:.1f} s")
        self.get_logger().info(f"Average rotation rate: {avg_rate:.3f} rad/s")
        self.get_logger().info("=" * 60)

    def stop_robot(self):
        """Send zero velocity command to stop the robot."""
        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)

    def run(self):
        """Main run loop - just spin and wait for commands."""
        # Wait for initial odometry
        odom_timeout = 5.0
        start_wait = time.time()

        while (
            not self.odom_received
            and (time.time() - start_wait) < odom_timeout
        ):
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.odom_received:
            self.get_logger().error(
                "No odometry received! Make sure the base node is running."
            )
            return False

        # Just spin and wait for commands
        self.get_logger().info(
            "Controller ready - waiting for motion commands..."
        )
        rclpy.spin(self)

        return True


def main():
    rclpy.init()
    node = MotionController()
    try:
        node.run()
    finally:
        node.stop_robot()  # Ensure robot stops on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
