#!/usr/bin/env python3
"""
Control TidyBot to move forward/backward or spin in place based on commands.
Allows for adjustment to speed and timeout from ros topic pub


Setup:
ros2 run tidybot_bringup test-merge.py --ros-args -p linear_speed:=### -p angular_speed:=### -p timeout:=### -p use_frame_offset:=###

Topics:
- Publishes:  /cmd_vel (geometry_msgs/Twist)
- Subscribes: /odom (nav_msgs/Odometry)
- Subscribes: /motion_command (std_msgs/String)
        args:
          "forward"
          "backward"
          "spin"
        optional overrides (in the same string):
          v=0.15      (linear speed magnitude, m/s)
          w=0.60      (angular speed, rad/s)
          timeout=3.0 (seconds)

Examples:
    ros2 topic pub /motion_command std_msgs/msg/String "{data: 'forward'}" --once
    ros2 topic pub /motion_command std_msgs/msg/String "{data: 'backward v=0.10 timeout=2.5'}" --once
    ros2 topic pub /motion_command std_msgs/msg/String "{data: 'spin w=0.6 timeout=4.0'}" --once
    ros2 topic pub /stop_motion std_msgs/msg/Bool "{data: true}" --once
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import numpy as np


DEFAULT_LINEAR_SPEED = 0.02  # m/s (magnitude used for forward/backward)
DEFAULT_ANGULAR_SPEED = 0.3  # rad/s
DEFAULT_TIMEOUT = 10.0  # s
USE_MUJOCO_FRAME_OFFSET = True

CONTROL_HZ = 20.0
STOP_PUBLISH_COUNT = 5  # publish zero cmd_vel a few times for robustness


class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller")

        self.declare_parameter("linear_speed", DEFAULT_LINEAR_SPEED)
        self.declare_parameter("angular_speed", DEFAULT_ANGULAR_SPEED)
        self.declare_parameter("timeout", DEFAULT_TIMEOUT)
        self.declare_parameter("use_frame_offset", USE_MUJOCO_FRAME_OFFSET)

        # State
        self.odom_received = False
        self.active_mode = "idle"  # "idle", "forward", "backward", "spin"
        self.should_stop = False

        # Per-motion overrides (set when a command is received; cleared on finish)
        self.override_linear_speed = None
        self.override_angular_speed = None
        self.override_timeout = None

        # Pose tracking
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Motion bookkeeping
        self.motion_start_time = None
        self.last_log_time = None
        self.stop_publish_remaining = 0

        # ROS I/O
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.stop_sub = self.create_subscription(
            Bool, "/stop_motion", self.stop_callback, 10
        )

        self.command_sub = self.create_subscription(
            String, "/motion_command", self.command_callback, 10
        )

        self.control_timer = self.create_timer(
            1.0 / CONTROL_HZ, self.control_tick
        )

        self.get_logger().info("Motion controller started. Waiting for odometry...")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        odom_theta = 2.0 * np.arctan2(qz, qw)

        use_frame_offset = bool(self.get_parameter("use_frame_offset").value)
        if use_frame_offset:
            theta = odom_theta - np.pi / 2.0
        else:
            theta = odom_theta

        self.current_theta = np.arctan2(np.sin(theta), np.cos(theta))

        if not self.odom_received:
            self.odom_received = True
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_theta = self.current_theta

            self.get_logger().info(
                f"Odometry connected! Position: ({self.start_x:.3f}, {self.start_y:.3f}), "
                f"Heading: {np.degrees(self.current_theta):.1f}°"
            )
            self.get_logger().info(
                'Ready. Commands on /motion_command: "forward", "backward", "spin".'
            )
            self.get_logger().info(
                'Optional overrides: v=<m/s> w=<rad/s> timeout=<s> (e.g. "forward v=0.1 timeout=2").'
            )
            self.get_logger().info("Stop: publish true on /stop_motion.")

    def stop_callback(self, msg: Bool):
        if msg.data:
            if self.active_mode != "idle":
                self.get_logger().info("Stop signal received!")
            self.should_stop = True

    def parse_motion_command(self, text: str):
        """
        Parses commands like:
          "forward v=0.15 timeout=3.0"
          "backward v=0.10"
          "spin w=0.6 timeout=4"
        Returns: (mode, v_override, w_override, timeout_override)
        """
        tokens = text.lower().strip().split()
        if len(tokens) == 0:
            return None, None, None, None

        mode = tokens[0]
        v_override = None
        w_override = None
        timeout_override = None

        for t in tokens[1:]:
            if "=" not in t:
                continue
            k, v = t.split("=", 1)
            k = k.strip()
            v = v.strip()
            try:
                val = float(v)
            except ValueError:
                continue

            if k in ["v", "linear", "linear_speed"]:
                v_override = val
            elif k in ["w", "angular", "angular_speed"]:
                w_override = val
            elif k in ["timeout", "t"]:
                timeout_override = val

        return mode, v_override, w_override, timeout_override

    def command_callback(self, msg: String):
        raw = msg.data

        mode, v_override, w_override, timeout_override = self.parse_motion_command(raw)

        if mode not in ["forward", "backward", "spin"]:
            self.get_logger().warn(
                f'Unknown command: "{raw}". Use "forward", "backward", or "spin" '
                '(optionally add v=, w=, timeout=).'
            )
            return

        if not self.odom_received:
            self.get_logger().error("No odometry received yet! Cannot execute motion.")
            return

        if self.active_mode != "idle":
            self.get_logger().warn(
                f'Already executing "{self.active_mode}". '
                "Publish /stop_motion true to stop, then send a new command."
            )
            return

        self.override_linear_speed = v_override
        self.override_angular_speed = w_override
        self.override_timeout = timeout_override

        self.start_motion(mode)

    def get_timeout(self):
        timeout = float(self.get_parameter("timeout").value)
        if self.override_timeout is not None:
            timeout = float(self.override_timeout)
        return timeout

    def get_linear_speed_mag(self):
        vmag = float(self.get_parameter("linear_speed").value)
        if self.override_linear_speed is not None:
            vmag = float(self.override_linear_speed)
        vmag = abs(vmag)
        return vmag

    def get_angular_speed(self):
        w = float(self.get_parameter("angular_speed").value)
        if self.override_angular_speed is not None:
            w = float(self.override_angular_speed)
        return w

    def start_motion(self, mode: str):
        self.should_stop = False
        self.active_mode = mode

        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_theta = self.current_theta

        now = self.get_clock().now()
        self.motion_start_time = now
        self.last_log_time = now

        timeout = self.get_timeout()

        self.get_logger().info("")
        self.get_logger().info("=" * 60)

        if mode == "forward":
            vmag = self.get_linear_speed_mag()
            self.get_logger().info(f"FORWARD: {vmag:.2f} m/s")
        elif mode == "backward":
            vmag = self.get_linear_speed_mag()
            self.get_logger().info(f"BACKWARD: {vmag:.2f} m/s")
        else:
            w = self.get_angular_speed()
            self.get_logger().info(
                f"SPIN: {w:.2f} rad/s ({np.degrees(w):.1f} deg/s)"
            )

        if self.override_linear_speed is not None:
            self.get_logger().info(f"Override: v={float(self.override_linear_speed):.3f} m/s")
        if self.override_angular_speed is not None:
            self.get_logger().info(f"Override: w={float(self.override_angular_speed):.3f} rad/s")
        if self.override_timeout is not None:
            self.get_logger().info(f"Override: timeout={float(self.override_timeout):.3f} s")

        self.get_logger().info(f"Timeout: {timeout:.1f}s | Stop: /stop_motion true")
        self.get_logger().info("=" * 60)

    def finish_motion(self, reason: str):
        self.active_mode = "idle"
        self.motion_start_time = None
        self.last_log_time = None
        self.should_stop = False

        self.override_linear_speed = None
        self.override_angular_speed = None
        self.override_timeout = None

        self.stop_publish_remaining = STOP_PUBLISH_COUNT

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"MOTION STOPPED ({reason})")
        self.get_logger().info("=" * 60)

    def publish_zero(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)

    def control_tick(self):
        if self.stop_publish_remaining > 0:
            self.publish_zero()
            self.stop_publish_remaining -= 1
            return

        if self.active_mode == "idle":
            return

        timeout = self.get_timeout()
        now = self.get_clock().now()
        elapsed = (now - self.motion_start_time).nanoseconds * 1e-9

        if self.should_stop:
            self.finish_motion("stop command")
            return

        if elapsed >= timeout:
            self.finish_motion("timeout")
            return

        vel = Twist()

        if self.active_mode == "forward":
            vel.linear.x = self.get_linear_speed_mag()
            vel.angular.z = 0.0
        elif self.active_mode == "backward":
            vel.linear.x = -self.get_linear_speed_mag()
            vel.angular.z = 0.0
        elif self.active_mode == "spin":
            vel.linear.x = 0.0
            vel.angular.z = self.get_angular_speed()

        self.cmd_vel_pub.publish(vel)

        if (now - self.last_log_time).nanoseconds * 1e-9 >= 1.0:
            if self.active_mode in ["forward", "backward"]:
                dx = self.current_x - self.start_x
                dy = self.current_y - self.start_y
                distance = float(np.sqrt(dx * dx + dy * dy))
                self.get_logger().info(
                    f"Time: {elapsed:.1f}s | "
                    f"Pos: ({self.current_x:.3f}, {self.current_y:.3f}) | "
                    f"Dist: {distance:.3f}m | "
                    f"Heading: {np.degrees(self.current_theta):.0f}°"
                )
            else:
                dtheta = self.current_theta - self.start_theta
                dtheta = float(np.arctan2(np.sin(dtheta), np.cos(dtheta)))
                self.get_logger().info(
                    f"Time: {elapsed:.1f}s | "
                    f"Heading: {np.degrees(self.current_theta):.0f}° | "
                    f"Rotated: {np.degrees(dtheta):.0f}°"
                )

            self.last_log_time = now


def main():
    rclpy.init()
    node = MotionController()
    try:
        rclpy.spin(node)
    finally:
        for _ in range(STOP_PUBLISH_COUNT):
            node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()