#!/usr/bin/env python3
"""
Combined Arm & Gripper Controller Node for TidyBot2 (Real Hardware Version).

This node acts as an integration layer to test the manipulation pipeline 
(motion planning + gripper control) independently from the terminal.
It uses separated callback groups and async service calls to guarantee 
no deadlocks occur during execution.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tidybot_msgs.srv import PlanToTarget
from tidybot_control.gripper_controller import GripperController

class Combined_Arm_Gripper_Controller_Real(Node):
    def __init__(self):
        super().__init__('combined_arm_gripper_controller')

        # [MODIFIED FOR REAL ROBOT]: Default gripper mode changed to 'real'
        self.declare_parameter('gripper_mode', 'real')
        gripper_mode = self.get_parameter('gripper_mode').get_parameter_value().string_value

        # Separate callback groups for Server and Client to avoid starvation
        self.server_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        # 1. Initialize Gripper Controller
        self.get_logger().info(f'[INIT] Initializing GripperController in {gripper_mode} mode...')
        self.gripper = GripperController(self, mode=gripper_mode, pressure=1.0)

        # 2. Client setup
        self.plan_client = self.create_client(
            PlanToTarget,
            '/plan_to_target',
            callback_group=self.client_cb_group
        )

        self.get_logger().info('[INIT] Waiting for /plan_to_target service...')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('        Still waiting for /plan_to_target...')

        # 3. Server setup
        self.grasp_server = self.create_service(
            PlanToTarget,
            '/perform_grasp',
            self.grasp_callback,
            callback_group=self.server_cb_group
        )

        self.get_logger().info('[READY] Manipulation Coordinator ready. Call /perform_grasp to test.')

    def grasp_callback(self, request, response):
        arm_name = request.arm_name.lower()
        self.get_logger().info(f"\n========== GRASP SEQUENCE STARTED FOR {arm_name.upper()} ARM ==========")
        
        timeout_sec = 20.0

        # ==========================================
        # Step 1: Open Gripper
        # ==========================================
        self.get_logger().info("STEP 1: Opening gripper...")
        self.gripper.open(arm_name, duration=1.0)
        
        # [MODIFIED FOR REAL ROBOT]: Added small hardware settling time
        time.sleep(0.5)
        
        # ==========================================
        # Step 2: Pre-grasp Hover (Target X, Y with Z=0.4)
        # ==========================================
        self.get_logger().info(f"STEP 2: Planning pre-grasp hover motion (x={request.target_pose.position.x}, y={request.target_pose.position.y}, z=0.4)...")
        
        hover_req = PlanToTarget.Request()
        hover_req.arm_name = arm_name
        hover_req.target_pose.position.x = request.target_pose.position.x
        hover_req.target_pose.position.y = request.target_pose.position.y
        hover_req.target_pose.position.z = 0.4  #
        hover_req.target_pose.orientation = request.target_pose.orientation
        hover_req.use_orientation = request.use_orientation
        hover_req.execute = request.execute
        hover_req.duration = 5.0  # 

        self.get_logger().info("STEP 2: Calling motion planner to generate IK solution for hover position...")
        hover_future = self.plan_client.call_async(hover_req)

        wait_start = time.time()
        while not hover_future.done():
            if time.time() - wait_start > timeout_sec:
                self.get_logger().error(f"STEP 2 ERROR: Pre-grasp hover planner timeout!")
                response.success = False
                response.message = "Hover motion planner service timeout."
                return response
            time.sleep(0.05)

        hover_response = hover_future.result()

        if not hover_response.success:
            self.get_logger().error(f"STEP 2 ERROR: Hover planning failed -> {hover_response.message}")
            response.success = False
            response.message = f"Failed at pre-grasp hover: {hover_response.message}"
            return response

        self.get_logger().info("STEP 2: Pre-grasp hover IK solution found successfully!")

        if request.execute:
            self.get_logger().info(f"STEP 2-1: Executing physical arm movement to hover position... Waiting {hover_req.duration}s")
            time.sleep(hover_req.duration)
            # [MODIFIED FOR REAL ROBOT]: Wait for arm to physically settle
            time.sleep(0.5)


        # ==========================================
        # Step 3: Final Reach to Requested Target
        # ==========================================
        self.get_logger().info(f"STEP 3: Planning final descent to target (z={request.target_pose.position.z})...")
        self.get_logger().info("STEP 3: Calling motion planner to generate IK solution for final grasp position...")
        
        reach_future = self.plan_client.call_async(request)

        wait_start = time.time()
        while not reach_future.done():
            if time.time() - wait_start > timeout_sec:
                self.get_logger().error(f"STEP 3 ERROR: Final reach planner timeout!")
                response.success = False
                response.message = "Final reach motion planner service timeout."
                return response
            time.sleep(0.05)

        plan_response = reach_future.result()

        if not plan_response.success:
            self.get_logger().error(f"STEP 3 ERROR: Final reach planning failed -> {plan_response.message}")
            response.success = False
            response.message = f"Failed at final reach: {plan_response.message}"
            return response

        self.get_logger().info("STEP 3: Final reach IK solution found successfully!")

        if request.execute:
            self.get_logger().info(f"STEP 3-1: Executing physical arm descent to target... Waiting {request.duration}s")
            time.sleep(request.duration)
            # [MODIFIED FOR REAL ROBOT]: Wait for arm to physically settle before grasping
            time.sleep(0.5)

            # ==========================================
            # Step 4: Close Gripper
            # ==========================================
            self.get_logger().info("STEP 4: Target reached. Closing gripper to grasp...")
            self.gripper.close(arm_name, duration=2.0)
            
            # ==========================================
            # Step 5: Wait 1 second after grasping
            # ==========================================
            self.get_logger().info("STEP 5: Grasp complete. Waiting 1 second for steady grip...")
            time.sleep(1.0)
            
            # ==========================================
            # Step 6: Lift the object to new position
            # ==========================================
            self.get_logger().info("STEP 6: Planning vertical lift motion...")
            lift_req = PlanToTarget.Request()
            lift_req.arm_name = arm_name
            
            lift_req.target_pose.position.x = 0.0
            lift_req.target_pose.position.y = -0.3
            lift_req.target_pose.position.z = 0.3
            
            # Keep the same orientation used for grasping
            lift_req.target_pose.orientation = request.target_pose.orientation
            
            lift_req.use_orientation = request.use_orientation
            lift_req.execute = True
            lift_req.duration = 4.0  # lift time
            
            self.get_logger().info("STEP 6: Calling motion planner to generate IK solution for lift...")
            lift_future = self.plan_client.call_async(lift_req)
            
            lift_wait_start = time.time()
            while not lift_future.done():
                if time.time() - lift_wait_start > timeout_sec:
                    self.get_logger().error(f"STEP 6 ERROR: Lift motion planner timeout!")
                    response.success = False
                    response.message = "Lift motion planner service timeout."
                    return response
                time.sleep(0.05)
                
            lift_response = lift_future.result()
            
            if not lift_response.success:
                self.get_logger().error(f"STEP 6 ERROR: Lift planning failed -> {lift_response.message}")
                response.success = False
                response.message = f"Failed at lift planning: {lift_response.message}"
                return response
                
            self.get_logger().info(f"STEP 6-1: Lift IK solution found. Executing movement for {lift_req.duration}s...")
            time.sleep(lift_req.duration)
            
            response.message = "Pre-grasp hover, final reach, grasp, and lift execution completed successfully."
        else:
            response.message = "Planning for hover and final reach completed (execution skipped)."

        response.success = True
        response.position_error = plan_response.position_error
        response.orientation_error = plan_response.orientation_error
        response.joint_positions = plan_response.joint_positions
        
        self.get_logger().info(f"========== GRASP SEQUENCE COMPLETED ==========\n")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Combined_Arm_Gripper_Controller_Real()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()