cmake_minimum_required(VERSION 3.8)
project(tidybot_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz config files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/test_base_sim.py
  scripts/test_camera_sim.py
  scripts/test_arms_sim.py
  scripts/test_planner_sim.py
  scripts/test_pickup_sim.py
  scripts/test_real_hardware.py
  scripts/test_base_real.py
  scripts/test_arms_real.py
  scripts/test_grippers_real.py
  scripts/test_grasp_real.py
  scripts/test_right_gripper.py
  scripts/test_pan_tilt_real.py
  scripts/test_torque_hold.py
  scripts/test_full_robot.py
  scripts/test_tf.py
  scripts/test_planner_real.py
  scripts/record_joint_config.py
  scripts/diagnose_dynamixel.py
  scripts/example_state_machine.py
  scripts/trajectory_tracking.py
  scripts/speech/speech_extraction_node.py
  scripts/speech/gemini.py
  scripts/combined_arm_motion_with_grasping.py
  scripts/combined_arm_motion_with_grasping_real.py
  scripts/ee_trajectory_plotter.py
  scripts/task_1.py
  scripts/task_2.py
  scripts/task_3.py
  scripts/task_4.py
  scripts/task_5.py
  # Vision scripts
  scripts/vision/find_center.py
  scripts/vision/color_mask.py
  scripts/vision/obj_dist.py
  scripts/vision/simple_pose_fit.py
  scripts/vision/testing/test_camera_smoke.py
  scripts/vision/testing/test_capture_dataset.py
  scripts/vision/testing/test_pan_tilt_sweep.py
  scripts/motion/test-linear.py
  scripts/motion/test-spinning.py
  scripts/motion/test-merge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/combined_arm_motion_with_grasping.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME combined_arm_gripper_controller
)

ament_package()
