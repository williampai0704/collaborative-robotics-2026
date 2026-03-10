#!/usr/bin/env python3
"""
End-Effector Trajectory Tracker and 3D Plotter.
Strictly tailored to user requirements:
[LOG] 1. End-effector coordinate log (Exactly every 1 second).
[PLOT] 1. Target waypoints (Red X).
[PLOT] 2. Actual trajectory sampled exactly every 1 second (Dots + Line).
[PLOT] 3. Planned trajectory from IK (Blue dashed line).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tidybot_msgs.msg import ArmCommand
from sensor_msgs.msg import JointState

import mujoco
from pathlib import Path
import os
import numpy as np
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import traceback

class EETrajectoryTracker(Node):
    def __init__(self):
        super().__init__('ee_trajectory_tracker')
        
        # --- Data Storage Arrays ---
        self.actual_x, self.actual_y, self.actual_z, self.time_data = [], [], [], []
        self.target_wp_x, self.target_wp_y, self.target_wp_z = [], [], []
        self.planned_traj_x, self.planned_traj_y, self.planned_traj_z = [], [], []
        
        self.start_time = None
        self.current_joints = {}  
        self.last_log_time = None
        
        # Load MuJoCo model and setup isolated calculators
        self.setup_mujoco_for_fk()

        # --- Subscribers ---
        self.create_subscription(Pose, '/target_marker_pose', self.target_marker_callback, 10)
        self.create_subscription(ArmCommand, '/right_arm/cmd', self.arm_cmd_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        
        # 타이머를 사용하여 1.0초마다 로깅을 시도합니다. (Wall Time 기준)
        self.create_timer(1.0, self.timer_log_callback)
        
        self.get_logger().info("✅ Tracker initialized (Ghost Publisher Filter Active).")
        self.get_logger().info("1. Logging EE coordinates exactly every 1 second.")
        self.get_logger().info("2. Tracking Target, Actual (1s intervals), and Planned trajectories.")
        self.get_logger().info("Run your grasp sequence now. Press Ctrl+C to view the 3D plot!")

    def setup_mujoco_for_fk(self):
        sim_path = os.environ.get('TIDYBOT_SIMULATION_PATH')
        if not sim_path:
            self.get_logger().error("TIDYBOT_SIMULATION_PATH not set!")
            return
            
        model_path = Path(sim_path) / 'assets/mujoco/tidybot_wx250s_bimanual.xml'
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        
        # Independent calculators
        self.data_actual = mujoco.MjData(self.model)
        self.data_planned = mujoco.MjData(self.model)
        
        # Target only the right arm joints
        self.joint_names = [
            'right_waist', 'right_shoulder', 'right_elbow',
            'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate'
        ]
        self.joint_addrs = [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in self.joint_names]
        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'right_pinch_site')
        self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')

    def target_marker_callback(self, msg):
        """Save target coordinates for each step."""
        base_x = msg.position.y
        base_y = -msg.position.x
        base_z = msg.position.z
        
        self.get_logger().info(f"🎯 Target Waypoint Received: X={base_x:.3f}, Y={base_y:.3f}, Z={base_z:.3f}")
        self.target_wp_x.append(base_x)
        self.target_wp_y.append(base_y)
        self.target_wp_z.append(base_z)

    def arm_cmd_callback(self, msg):
        """Save planned trajectory for the end effector."""
        if not hasattr(self, 'model'): return
        
        for addr, pos in zip(self.joint_addrs, msg.joint_positions):
            self.data_planned.qpos[addr] = pos
            
        mujoco.mj_forward(self.model, self.data_planned)
        
        base_pos = self.data_planned.xpos[self.base_id]
        base_mat = self.data_planned.xmat[self.base_id].reshape(3, 3)
        site_pos = self.data_planned.site_xpos[self.site_id]
        
        local_pos = base_mat.T @ (site_pos - base_pos)
        self.planned_traj_x.append(local_pos[0])
        self.planned_traj_y.append(local_pos[1])
        self.planned_traj_z.append(local_pos[2])

    def joint_states_callback(self, msg):
        """
        [핵심 필터] 유령 퍼블리셔가 보내는 가짜 데이터(0.0)를 무시하고 
        진짜 로봇 관절 데이터만 저장합니다.
        """
        temp_joints = {}
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                temp_joints[name] = pos
                
        # 오른팔 데이터가 아예 없으면 무시
        if not temp_joints:
            return
            
        # 모든 값이 0.0 근처인지 확인 (유령 퍼블리셔의 특징)
        # 만약 전부 0.0 이라면 이 데이터는 버립니다.
        positions = list(temp_joints.values())
        if all(abs(p) < 1e-4 for p in positions):
            return 
            
        # 정상적인 데이터라면 현재 상태를 업데이트합니다.
        for name, pos in temp_joints.items():
            self.current_joints[name] = pos

    def timer_log_callback(self):
        """1초마다 최신 관절 상태를 바탕으로 좌표를 계산하고 저장합니다."""
        if not hasattr(self, 'model'): return
        
        # 오른팔 6개 조인트 데이터가 모두 모일 때까지 대기
        if not all(j in self.current_joints for j in self.joint_names):
            return

        # ROS의 get_clock()을 사용하여 시간을 측정합니다.
        now = self.get_clock().now().nanoseconds / 1e9
        
        if self.start_time is None:
            self.start_time = now
            
        elapsed_total = now - self.start_time

        # 현재 저장된 조인트 값을 MuJoCo에 입력하여 Forward Kinematics 계산
        for idx, jname in enumerate(self.joint_names):
            self.data_actual.qpos[self.joint_addrs[idx]] = self.current_joints[jname]
                
        mujoco.mj_forward(self.model, self.data_actual)
        
        base_pos = self.data_actual.xpos[self.base_id]
        base_mat = self.data_actual.xmat[self.base_id].reshape(3, 3)
        site_pos = self.data_actual.site_xpos[self.site_id]
        
        # Base 좌표계 기준 EE 위치 계산
        local_pos = base_mat.T @ (site_pos - base_pos)
        x, y, z = local_pos[0], local_pos[1], local_pos[2]

        # 1.0초마다 터미널에 로그 출력
        self.get_logger().info(f"⏱️ Actual EE Pose -> X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f} (Time: {elapsed_total:.1f}s)")
        
        # 그래프를 그리기 위해 데이터 저장
        self.actual_x.append(x)
        self.actual_y.append(y)
        self.actual_z.append(z)
        self.time_data.append(elapsed_total)

    def plot_trajectory(self):
        if not self.actual_x and not self.target_wp_x:
            self.get_logger().error("🚨 No data collected! Plot aborted.")
            return
            
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        
        if self.actual_x:
            scatter = ax.scatter(self.actual_x, self.actual_y, self.actual_z, 
                                 c=self.time_data, cmap='viridis', 
                                 marker='o', s=40, alpha=0.9, label='1. Actual Trajectory (1s points)')
            ax.plot(self.actual_x, self.actual_y, self.actual_z, color='gray', linestyle='-', linewidth=1, alpha=0.5)
            ax.scatter(self.actual_x[0], self.actual_y[0], self.actual_z[0], color='green', s=150, marker='^', label='Start Position')
            cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
            cbar.set_label('Elapsed Time (s)')
        
        if self.target_wp_x:
            ax.scatter(self.target_wp_x, self.target_wp_y, self.target_wp_z, 
                       color='red', s=200, marker='X', label='2. Target Waypoints')
                       
        if self.planned_traj_x and self.actual_x:
            px = [self.actual_x[0]] + self.planned_traj_x
            py = [self.actual_y[0]] + self.planned_traj_y
            pz = [self.actual_z[0]] + self.planned_traj_z
            ax.plot(px, py, pz, color='blue', linestyle='--', linewidth=2, label='3. Planned Trajectory (IK)')
        
        ax.set_xlabel('X axis (m)')
        ax.set_ylabel('Y axis (m)')
        ax.set_zlabel('Z axis (m)')
        ax.set_title('Trajectory Analysis: Actual (1s) vs Planned vs Targets')
        ax.legend()
        
        all_x = self.actual_x + self.target_wp_x + self.planned_traj_x
        all_y = self.actual_y + self.target_wp_y + self.planned_traj_y
        all_z = self.actual_z + self.target_wp_z + self.planned_traj_z
        
        if all_x:
            max_range = np.array([max(all_x)-min(all_x), max(all_y)-min(all_y), max(all_z)-min(all_z)]).max() / 2.0
            mid_x, mid_y, mid_z = (max(all_x)+min(all_x))*0.5, (max(all_y)+min(all_y))*0.5, (max(all_z)+min(all_z))*0.5
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.show()


def main(args=None):
    import traceback
    try:
        rclpy.init(args=args)
        node = EETrajectoryTracker()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.plot_trajectory()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except Exception as e:
        print("\n🚨 [치명적 에러 발생] 프로그램이 중단되었습니다! 아래 로그를 확인하세요:")
        traceback.print_exc()


if __name__ == '__main__':
    main()