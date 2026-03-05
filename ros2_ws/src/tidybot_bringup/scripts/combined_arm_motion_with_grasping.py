#!/usr/bin/env python3
"""
Combined Arm & Gripper Controller Node for TidyBot2.

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

from sensor_msgs.msg import JointState  # JointState 메시지 임포트 추가
from tidybot_msgs.srv import PlanToTarget
from tidybot_msgs.msg import ArmCommand # [MODIFIED] Added for Step 0 synchronization
from tidybot_control.gripper_controller import GripperController

class Combined_Arm_Gripper_Controller(Node):
    def __init__(self):
        super().__init__('combined_arm_gripper_controller')

        self.declare_parameter('gripper_mode', 'sim')
        gripper_mode = self.get_parameter('gripper_mode').get_parameter_value().string_value

        # Separate callback groups for Server, Client, and Subscriber to avoid starvation
        self.server_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub_cb_group = MutuallyExclusiveCallbackGroup() # Subscriber용 그룹 추가

        # Joint States 저장용 딕셔너리
        self.current_joint_states = {}

        # ==========================================
        # [MODIFIED] Added publishers to send direct sync commands to the low-level controllers
        # ==========================================
        self.arm_cmd_pubs = {
            'right': self.create_publisher(ArmCommand, '/right_arm/cmd', 10, callback_group=self.client_cb_group),
            'left': self.create_publisher(ArmCommand, '/left_arm/cmd', 10, callback_group=self.client_cb_group)
        }

        # 1. Initialize Gripper Controller
        self.get_logger().info(f'[INIT] Initializing GripperController in {gripper_mode} mode...')
        self.gripper = GripperController(self, mode=gripper_mode, pressure=1.0)

        # 1-1. Initialize Joint State Subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.sub_cb_group
        )

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

    def joint_state_callback(self, msg: JointState):
        """계속해서 들어오는 조인트 상태를 딕셔너리에 업데이트합니다."""
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_states[name] = pos

    def wait_for_joint_convergence(self, arm_name: str, target_joints: list, timeout: float, tolerance: float = 0.01) -> bool:
        """
        현재 관절 각도가 목표 관절 각도(target_joints)에 오차(tolerance) 내로 도달할 때까지 대기합니다.
        """
        # arm_name에 맞는 조인트 이름 리스트 생성
        joint_names = [
            f'{arm_name}_waist', f'{arm_name}_shoulder', f'{arm_name}_elbow',
            f'{arm_name}_forearm_roll', f'{arm_name}_wrist_angle', f'{arm_name}_wrist_rotate'
        ]

        start_time = time.time()
        while time.time() - start_time < timeout:
            # 필요한 조인트 데이터가 모두 수신되었는지 확인
            if all(j in self.current_joint_states for j in joint_names):
                current_joints = [self.current_joint_states[j] for j in joint_names]
                
                # 각 관절별 오차 중 가장 큰 값을 확인
                max_error = max(abs(c - t) for c, t in zip(current_joints, target_joints))
                
                if max_error <= tolerance:
                    self.get_logger().info(f" -> Joints converged! (Max error: {max_error:.4f} rad)")
                    return True
            
            # 짧게 대기 후 다시 확인 (while문 폭주 방지)
            time.sleep(0.1)

        self.get_logger().warn(f" -> Timeout reached before joints fully converged (Tolerance: {tolerance} rad).")
        return False

    def grasp_callback(self, request, response):
        arm_name = request.arm_name.lower()
        self.get_logger().info(f"\n========== GRASP SEQUENCE STARTED FOR {arm_name.upper()} ARM ==========")
        
        timeout_sec = 20.0
        joint_names = [
            f'{arm_name}_waist', f'{arm_name}_shoulder', f'{arm_name}_elbow',
            f'{arm_name}_forearm_roll', f'{arm_name}_wrist_angle', f'{arm_name}_wrist_rotate'
        ]

        # ==========================================
        # Step 0: Synchronize Low-Level Controller (Ghost Data 방지)
        # ==========================================
        self.get_logger().info("STEP 0: Waiting for valid (non-zero) physical joint states...")
        valid_state_received = False
        
        # 0.0 유령 데이터를 걸러내기 위해 최대 2초 대기
        for _ in range(20):
            if all(j in self.current_joint_states for j in joint_names):
                # 모든 관절이 0.001 미만(사실상 0.0)인지 검사
                is_all_zeros = all(abs(self.current_joint_states[j]) < 0.001 for j in joint_names)
                if not is_all_zeros:
                    valid_state_received = True
                    break
            time.sleep(0.1)

        if valid_state_received:
            self.get_logger().info("STEP 0: Valid states received. Synchronizing controller...")
            sync_cmd = ArmCommand()
            sync_cmd.joint_positions = [self.current_joint_states[j] for j in joint_names]
            sync_cmd.duration = 0.5 
            self.arm_cmd_pubs[arm_name].publish(sync_cmd)
            time.sleep(1.0)
        else:
            self.get_logger().warn("STEP 0: Failed to get valid states. Robot might jerk to 0.0!")

        # ==========================================
        # Step 1: Open Gripper
        # ==========================================
        self.get_logger().info("STEP 1: Opening gripper...")
        self.gripper.open(arm_name, duration=1.0)
        
        # ==========================================
        # Step 2: Pre-grasp Hover (z=0.4)
        # ==========================================
        self.get_logger().info(f"STEP 2: Planning pre-grasp hover motion (z=0.4)...")
        hover_req = PlanToTarget.Request()
        hover_req.arm_name = arm_name
        hover_req.target_pose.position.x = request.target_pose.position.x
        hover_req.target_pose.position.y = request.target_pose.position.y
        hover_req.target_pose.position.z = 0.4
        hover_req.target_pose.orientation = request.target_pose.orientation
        hover_req.use_orientation = request.use_orientation
        hover_req.execute = request.execute
        hover_req.duration = 10.0 

        hover_future = self.plan_client.call_async(hover_req)
        while not hover_future.done():
            time.sleep(0.05)
        hover_response = hover_future.result()

        if request.execute:
            self.wait_for_joint_convergence(arm_name, hover_response.joint_positions, timeout=15.0)

        # ==========================================
        # Step 3-A: Mid-descent Waypoint (z=0.15) - 튐 방지
        # ==========================================
        self.get_logger().info("STEP 3-A: Mid-descent to z=0.15 to prevent joint sweeping arc...")
        reach_req_a = PlanToTarget.Request()
        reach_req_a.arm_name = arm_name
        reach_req_a.target_pose.position.x = request.target_pose.position.x 
        reach_req_a.target_pose.position.y = request.target_pose.position.y
        reach_req_a.target_pose.position.z = 0.15 
        reach_req_a.target_pose.orientation = request.target_pose.orientation
        reach_req_a.use_orientation = request.use_orientation
        reach_req_a.execute = True
        reach_req_a.duration = 5.0 
        
        future_3a = self.plan_client.call_async(reach_req_a)
        while not future_3a.done():
            time.sleep(0.05)
        self.wait_for_joint_convergence(arm_name, future_3a.result().joint_positions, timeout=10.0)

        # ==========================================
        # Step 3-B: Final Reach (z=0.02)
        # ==========================================
        self.get_logger().info(f"STEP 3-B: Final descent to target (z={request.target_pose.position.z})...")
        reach_future = self.plan_client.call_async(request)
        while not reach_future.done():
            time.sleep(0.05)
        plan_response = reach_future.result()
        
        if request.execute:
            self.wait_for_joint_convergence(arm_name, plan_response.joint_positions, timeout=10.0)

            # ==========================================
            # Step 4: Close Gripper & Step 5: Wait
            # ==========================================
            self.get_logger().info("STEP 4: Target reached. Closing gripper...")
            time.sleep(2.0)
            self.gripper.close(arm_name, duration=2.0)
            self.get_logger().info("STEP 5: Grasp complete. Waiting 1 second...")
            time.sleep(1.0)
            
            # ==========================================
            # Step 6-A: Mid-lift Waypoint (z=0.15) - 튐 방지
            # ==========================================
            self.get_logger().info("STEP 6-A: Mid-lift to z=0.15 to prevent joint sweeping arc...")
            lift_req_a = PlanToTarget.Request()
            lift_req_a.arm_name = arm_name
            lift_req_a.target_pose.position.x = request.target_pose.position.x 
            lift_req_a.target_pose.position.y = request.target_pose.position.y
            lift_req_a.target_pose.position.z = 0.15 
            lift_req_a.target_pose.orientation = request.target_pose.orientation
            lift_req_a.use_orientation = request.use_orientation
            lift_req_a.execute = True
            lift_req_a.duration = 5.0 
            
            future_6a = self.plan_client.call_async(lift_req_a)
            while not future_6a.done():
                time.sleep(0.05)
            self.wait_for_joint_convergence(arm_name, future_6a.result().joint_positions, timeout=10.0)

            # ==========================================
            # Step 6-B: Final Lift (z=0.40)
            # ==========================================
            self.get_logger().info("STEP 6-B: Final lift to hover height (z=0.40)...")
            lift_req_b = PlanToTarget.Request()
            lift_req_b.arm_name = arm_name
            lift_req_b.target_pose.position.x = request.target_pose.position.x 
            lift_req_b.target_pose.position.y = request.target_pose.position.y
            lift_req_b.target_pose.position.z = 0.4 
            lift_req_b.target_pose.orientation = request.target_pose.orientation
            lift_req_b.use_orientation = request.use_orientation
            lift_req_b.execute = True
            lift_req_b.duration = 8.0 
            
            future_6b = self.plan_client.call_async(lift_req_b)
            while not future_6b.done():
                time.sleep(0.05)
            self.wait_for_joint_convergence(arm_name, future_6b.result().joint_positions, timeout=12.0)
            
            response.message = "Multi-step grasp and lift execution completed perfectly."

        response.success = True
        response.position_error = plan_response.position_error
        response.orientation_error = plan_response.orientation_error
        response.joint_positions = plan_response.joint_positions
        
        self.get_logger().info(f"========== GRASP SEQUENCE COMPLETED ==========\n")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Combined_Arm_Gripper_Controller()
    
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