#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

from tidybot_msgs.srv import PlanToTarget
from tidybot_msgs.action import SequenceTask

class TaskSequencerActionServer(Node):
    def __init__(self):
        super().__init__('task_sequencer_action')
        self.cb_group = ReentrantCallbackGroup()
        
        # 하위 모션 플래너 호출용 클라이언트
        self.cli = self.create_client(PlanToTarget, '/plan_to_target', callback_group=self.cb_group)
        
        # 상위 명령 수신용 Action 서버
        self._action_server = ActionServer(
            self,
            SequenceTask,
            '/sequence_task',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )
        self.get_logger().info('✅ Action Server ready: /sequence_task')

    def cancel_callback(self, goal_handle):
        """클라이언트(터미널)에서 Ctrl+C를 누르면 호출됨"""
        self.get_logger().warn('🚨 Cancel request received! Stopping sequence.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """실제 시퀀스 실행 루프"""
        req = goal_handle.request
        feedback = SequenceTask.Feedback()
        result = SequenceTask.Result()

        if not self.cli.wait_for_service(timeout_sec=3.0):
            result.success, result.message = False, "Motion planner service not available"
            goal_handle.abort()
            return result

        # 공통 서비스 요청 템플릿
        srv_req = PlanToTarget.Request()
        srv_req.arm_name = req.arm_name
        srv_req.target_pose.position.x = req.target_x
        srv_req.target_pose.position.y = req.target_y
        srv_req.target_pose.orientation.w = 0.7071
        srv_req.target_pose.orientation.y = 0.7071
        srv_req.use_orientation = True
        srv_req.execute = True
        srv_req.duration = req.duration

        # =======================================================
        # Step 1: Hover (z = 0.5)
        # =======================================================
        self.get_logger().info(f"Step 1: Hovering at z=0.5")
        srv_req.target_pose.position.z = 0.5
        future = self.cli.call(srv_req)
        
        if not future.success:
            result.success, result.message = False, f"Hover failed: {future.message}"
            goal_handle.abort()
            return result

        # 로봇 이동(duration) + 대기(5초) 동안 실시간 피드백 및 취소 감지
        total_wait = req.duration + 5.0
        steps = int(total_wait * 10)  # 0.1초 단위로 쪼개서 검사
        
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success, result.message = False, "Canceled during Hover phase"
                return result
            
            time_left = total_wait - (i / 10.0)
            feedback.current_phase = f"Hovering... {time_left:.1f}s left"
            feedback.time_remaining = time_left
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        # =======================================================
        # Step 2: Descend (z = 사용자가 입력한 값)
        # =======================================================
        self.get_logger().info(f"Step 2: Descending to z={req.target_z}")
        srv_req.target_pose.position.z = req.target_z
        future = self.cli.call(srv_req)

        if not future.success:
            result.success, result.message = False, f"Descend failed: {future.message}"
            goal_handle.abort()
            return result

        # 내려가는 동안 실시간 피드백 및 취소 감지
        steps = int(req.duration * 10)
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success, result.message = False, "Canceled during Descend phase"
                return result
            
            time_left = req.duration - (i / 10.0)
            feedback.current_phase = f"Descending... {time_left:.1f}s left"
            feedback.time_remaining = time_left
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        goal_handle.succeed()
        result.success = True
        result.message = "Sequence fully completed!"
        return result

def main(args=None):
    rclpy.init(args=args)
    node = TaskSequencerActionServer()
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