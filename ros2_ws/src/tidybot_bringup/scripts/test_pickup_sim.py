#!/usr/bin/env python3
"""
TidyBot2 Block Pickup Demo (Simulation)

Picks up a red block from the table using the right arm with IK-based
motion planning and grasp detection.

State machine:
    INIT → OPEN_GRIPPER → APPROACH → DESCEND → CLOSE_GRIPPER → LIFT → DONE

Usage:
    # Terminal 1: Start simulation with block scene
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_pickup.xml

    # Terminal 2: Run this demo
    ros2 run tidybot_bringup test_pickup_sim.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tidybot_msgs.srv import PlanToTarget
from tidybot_control.gripper_controller import GripperController
import time
import numpy as np


# Block position in base_link frame
# World position (0.55, -0.15, 0.025), robot at origin with base_th=π/2
BLOCK_X = -0.15
BLOCK_Y = -0.55
BLOCK_Z = 0.025

# Waypoint heights (above block origin)
APPROACH_HEIGHT = 0.175   # 15cm above block
GRASP_HEIGHT = 0.085      # at block top
LIFT_HEIGHT = 0.225       # 20cm above block

# Top-down grasp orientation in base_link frame (quaternion wxyz)
# Fingers point straight down (-Z)
ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)


class TestPickup(Node):
    """Pick up a block using IK planning and grasp detection."""

    def __init__(self):
        super().__init__('test_pickup')

        # Create client for planning service
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        # Gripper controller in sim mode
        self.gripper = GripperController(self, mode='sim')

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 Block Pickup Demo')
        self.get_logger().info('=' * 50)

        # Wait for planning service
        self.get_logger().info('Waiting for /plan_to_target service...')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('Service connected!')
        self.get_logger().info('')

    def create_pose(self, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
        """Create a Pose message."""
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = float(qw)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        return pose

    def call_service_sync(self, request, timeout_sec=15.0):
        """Call service synchronously."""
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error('Service call timed out!')
            return None

        if future.exception() is not None:
            self.get_logger().error(f'Service call exception: {future.exception()}')
            return None

        return future.result()

    def plan_and_execute(self, pose, use_orientation=True, duration=2.0):
        """Plan and execute a right arm motion to target pose."""
        request = PlanToTarget.Request()
        request.arm_name = 'right'
        request.target_pose = pose
        request.use_orientation = use_orientation
        request.execute = True
        request.duration = duration
        request.max_condition_number = 100.0

        pos = pose.position
        self.get_logger().info(
            f'Planning right arm to: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})')

        result = self.call_service_sync(request)
        if result is None:
            return False

        if result.success:
            self.get_logger().info(f'  SUCCESS: {result.message}')
            if use_orientation:
                self.get_logger().info(
                    f'  Errors: pos={result.position_error:.4f}m, '
                    f'ori={result.orientation_error:.4f}rad '
                    f'({np.degrees(result.orientation_error):.1f}deg)')
            if result.executed:
                self.get_logger().info(f'  Executing over {duration}s...')
            return True
        else:
            self.get_logger().warn(f'  FAILED: {result.message}')
            return False

    def run_pickup(self):
        """Execute the block pickup sequence."""
        qw, qx, qy, qz = ORIENT_FINGERS_DOWN

        # --- OPEN GRIPPER ---
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 1: Opening right gripper')
        self.gripper.open('right', duration=1.0)
        self.get_logger().info('  Gripper opened')

        # --- APPROACH ---
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 2: Moving above block (approach)')
        pose_approach = self.create_pose(
            BLOCK_X, BLOCK_Y, APPROACH_HEIGHT, qw, qx, qy, qz)
        success = self.plan_and_execute(pose_approach, use_orientation=True, duration=2.0)
        if not success:
            self.get_logger().warn('Approach failed, retrying without orientation...')
            success = self.plan_and_execute(pose_approach, use_orientation=False, duration=2.0)
            if not success:
                self.get_logger().error('Approach failed! Aborting.')
                return
        time.sleep(2.5)

        # --- DESCEND ---
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 3: Descending to grasp height')
        pose_grasp = self.create_pose(
            BLOCK_X, BLOCK_Y, GRASP_HEIGHT, qw, qx, qy, qz)
        success = self.plan_and_execute(pose_grasp, use_orientation=True, duration=2.0)
        if not success:
            self.get_logger().warn('Descend failed, retrying without orientation...')
            success = self.plan_and_execute(pose_grasp, use_orientation=False, duration=2.0)
            if not success:
                self.get_logger().error('Descend failed! Aborting.')
                return
        time.sleep(2.5)

        # --- CLOSE GRIPPER + CHECK GRASP ---
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 4: Closing gripper and checking grasp')
        grasped = self.gripper.close_and_check('right', duration=1.5)
        self.get_logger().info(f'  Grasp detected: {grasped}')

        # --- LIFT ---
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 5: Lifting block')
        pose_lift = self.create_pose(
            BLOCK_X, BLOCK_Y, LIFT_HEIGHT, qw, qx, qy, qz)
        success = self.plan_and_execute(pose_lift, use_orientation=True, duration=2.0)
        if not success:
            self.get_logger().warn('Lift failed, retrying without orientation...')
            self.plan_and_execute(pose_lift, use_orientation=False, duration=2.0)
        time.sleep(2.5)

        # --- DONE ---
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 6: Final grasp check')
        final_grasp = self.gripper.check_grasp('right')

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        if final_grasp:
            self.get_logger().info('SUCCESS: Block picked up!')
        else:
            self.get_logger().info('Block not detected in gripper after lift.')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = TestPickup()

    try:
        node.run_pickup()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
