import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # ========================================================================
    # 1. Declare Launch Arguments
    # ========================================================================
    # Argument: 'mode'
    # Options: 'sim' (MuJoCo Simulation) or 'real' (WX250s Real Hardware)
    # Default: 'real'
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description='Operation mode: "sim" for simulation, "real" for physical hardware.'
    )
    
    # Capture the launch configuration value
    mode = LaunchConfiguration('mode')

    # ========================================================================
    # 2. Define Nodes
    # ========================================================================
    
    # [A] Motion Planner Node (Simulation Version)
    # This node uses MuJoCo for IK and collision checking.
    # Condition: Only launches if mode == 'sim'
    planner_sim_node = Node(
        package='tidybot_ik',           
        executable='motion_planner_node',
        name='motion_planner_sim',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'sim'"])
        )
    )

    # [B] Motion Planner Node (Real Hardware Version)
    # This node uses Pinocchio/xs_sdk for IK and real-world safety checks.
    # Condition: Only launches if mode == 'real'
    planner_real_node = Node(
        package='tidybot_ik',           
        executable='motion_planner_real_node',
        name='motion_planner_real',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'real'"])
        )
    )

    # [C] Combined Arm & Gripper Controller (Client)
    # This node orchestrates the manipulation sequence (Hover -> Reach -> Grasp -> Lift).
    # It runs in BOTH modes.
    # Parameter: 'gripper_mode' is set to the value of the 'mode' launch argument.
    controller_node = Node(
        package='tidybot_bringup',      
        executable='combined_arm_gripper_controller',
        name='combined_arm_gripper_controller',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'gripper_mode': mode}       # Pass 'sim' or 'real' dynamically
        ]
    )

    # ========================================================================
    # 3. Return Launch Description
    # ========================================================================
    return LaunchDescription([
        mode_arg,
        planner_sim_node,
        planner_real_node,
        controller_node
    ])