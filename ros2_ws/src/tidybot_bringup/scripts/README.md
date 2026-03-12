# Collborative Robotics Task Scripts

This directory contains the main task orchestration scripts and supporting modules for vision, motion, and speech.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Task Scripts](#task-scripts)
  - [Task 1: Find & Pick](#task-1-find--pick)
  - [Task 2: Pick & Place](#task-2-pick--place)
  - [Task 3: Multi-Block Placement](#task-3-multi-block-placement)
- [Vision Modules](#vision-modules)
- [Motion Modules](#motion-modules)
- [Speech Modules](#speech-modules)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

### 1. Source the ROS2 Environment

```bash
cd /home/cdc/collaborative-robotics-2026/ros2_ws
source setup_env.bash
```

### 2. Start the Robot Hardware

**Real robot:**
```bash
ros2 launch tidybot_bringup real.launch.py
```

**Simulation:**
```bash
ros2 launch tidybot_bringup sim.launch.py
```

### 3. Start the Motion Planner

**For real robot:**
```bash
ros2 run tidybot_ik motion_planner_real_node
```

**For simulation:**
```bash
ros2 run tidybot_ik motion_planner_node
```

---

## Quick Start

```bash
# Terminal 1: Launch robot
ros2 launch tidybot_bringup real.launch.py

# Terminal 2: Start motion planner
ros2 run tidybot_ik motion_planner_real_node

# Terminal 3: Run a task
ros2 run tidybot_bringup task_2.py
```

---

## Task Scripts

### Task 1: Find & Pick

**Location:** `task_1.py`

**Description:** Spins to find a colored object, centers it, estimates pose, and picks it up.

**Dependencies:**
- Vision nodes: `find_center.py`, `simple_pose_fit.py` (must be started manually)
- Motion module: `test-merge.py` (must be started manually)
- Motion planner: `motion_planner_real_node`

#### Running Task 1

```bash
# Terminal 1: Start motion module
ros2 run tidybot_bringup test-merge.py

# Terminal 2: Start vision - find_center
ros2 run tidybot_bringup find_center.py --ros-args -p target_color:=yellow

# Terminal 3: Start vision - pose estimator
ros2 run tidybot_bringup simple_pose_fit.py --ros-args -p target_color:=yellow

# Terminal 4: Run task_1
ros2 run tidybot_bringup task_1.py
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `arm_name` | `left` | Which arm to use (`left` or `right`) |

```bash
ros2 run tidybot_bringup task_1.py --ros-args -p arm_name:=right
```

---

### Task 2: Pick & Place

**Location:** `task_2.py`

**Description:** Picks up a colored block and places it in a colored bin. **Self-contained** — automatically launches and manages vision nodes.

**Dependencies:**
- Motion planner: `motion_planner_real_node`
- (Optional) Google Cloud credentials for speech input

#### Running Task 2

```bash
# Default: yellow block → blue bin, left arm
ros2 run tidybot_bringup task_2.py

# Custom colors
ros2 run tidybot_bringup task_2.py \
    --ros-args -p block_color:=red -p bin_color:=green -p arm_name:=left

# With speech input (say "put the red block in the blue bin")
ros2 run tidybot_bringup task_2.py --ros-args -p use_speech:=true
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `block_color` | `yellow` | Color of block to pick (`red`, `green`, `blue`, `yellow`) |
| `bin_color` | `blue` | Color of target bin |
| `arm_name` | `left` | Which arm to use |
| `use_speech` | `true` | Enable voice command input |

---

### Task 3: Multi-Block Placement

**Location:** `task_3.py`

**Description:** Places multiple colored blocks into one bin in a specified order. Extends Task 2 with a block queue and camera tilt control.

**Dependencies:**
- Motion planner: `motion_planner_real_node`
- (Optional) Google Cloud credentials for speech input

#### Running Task 3

```bash
# Default: red, blue, yellow blocks → green bin
ros2 run tidybot_bringup task_3.py

# Custom order
ros2 run tidybot_bringup task_3.py \
    --ros-args -p block_colors:=red,blue,purple -p bin_color:=green

# With speech (say "place the red, blue, and purple blocks in the green bin")
ros2 run tidybot_bringup task_3.py --ros-args -p use_speech:=true
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `block_colors` | `red,blue,yellow` | Comma-separated ordered list of block colors |
| `bin_color` | `green` | Target bin color |
| `arm_name` | `left` | Which arm to use |
| `use_speech` | `false` | Enable voice command input |

---

## Vision Modules

Located in `vision/`

### find_center.py

Detects colored objects and publishes the pixel centroid.

**Topics:**
- Subscribes: `/camera/color/image_raw`
- Publishes: `/mask_center` (geometry_msgs/Point)

```bash
# Basic usage
ros2 run tidybot_bringup find_center.py --ros-args -p target_color:=yellow

# With visualization
ros2 run tidybot_bringup find_center.py --ros-args -p target_color:=red -p visualize:=true

# Debug mode (load from test_data folder)
ros2 run tidybot_bringup find_center.py --ros-args -p debug:=true -p debug_pair:=0 -p target_color:=yellow
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_color` | `red` | Color to detect (`red`, `green`, `blue`, `yellow`) |
| `visualize` | `false` | Show OpenCV window with mask overlay |
| `debug` | `false` | Load from test_data folder instead of camera |
| `debug_pair` | `0` | Which test pair to load (e.g., `pair_0000`) |

---

### obj_dist.py

Computes average depth distance to the masked object.

**Topics:**
- Subscribes: `/camera/color/image_raw`, `/camera/depth/image_raw`
- Publishes: `/vision/object_distance` (std_msgs/Float32)

```bash
# Basic usage
ros2 run tidybot_bringup obj_dist.py --ros-args -p target_color:=blue

# With visualization
ros2 run tidybot_bringup obj_dist.py --ros-args -p target_color:=blue -p visualize:=true

# Debug mode
ros2 run tidybot_bringup obj_dist.py --ros-args -p debug:=true -p debug_pair:=0
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_color` | `r` | Color to detect |
| `visualize` | `false` | Show depth visualization |
| `debug` | `false` | Load from test_data folder |

---

### simple_pose_fit.py

Estimates 6DOF pose using geometric heuristics (RANSAC table plane + 2D OBB).

**Topics:**
- Subscribes: `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`
- Publishes: `/object_pose` (geometry_msgs/PoseStamped)

```bash
# Basic usage
ros2 run tidybot_bringup simple_pose_fit.py --ros-args -p target_color:=yellow

# With 2D visualization
ros2 run tidybot_bringup simple_pose_fit.py --ros-args -p target_color:=yellow -p visualize:=true

# Debug mode with 3D Open3D visualization
ros2 run tidybot_bringup simple_pose_fit.py --ros-args \
    -p debug:=true -p debug_dataset:=yellow_block_1 -p target_color:=yellow
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_color` | `yellow` | Color to detect |
| `visualize` | `false` | Show 2D OpenCV visualization |
| `debug` | `false` | Debug mode with 3D visualization |
| `debug_dataset` | `yellow_block_1` | Dataset folder in test_data |
| `top_surface_threshold` | `0.01` | Height threshold for top surface (m) |
| `min_object_points` | `100` | Minimum points for pose estimation |

---

### visualize_pc.py

Visualizes RGB point cloud using Open3D.

```bash
# Standalone mode (from test_data)
python3 vision/visualize_pc.py --pair 0
python3 vision/visualize_pc.py --pair 1 --mask red

# ROS2 node mode (from camera)
ros2 run tidybot_bringup visualize_pc.py
ros2 run tidybot_bringup visualize_pc.py --ros-args -p target_color:=red -p visualize:=true
```

---

### color_mask.py

Utility module for HSV color thresholding. Used by other vision nodes.

```python
from color_mask import color_mask

# Returns binary mask (255 where color detected, 0 elsewhere)
mask = color_mask(rgb_image, 'red')    # or 'r'
mask = color_mask(rgb_image, 'green')  # or 'g'
mask = color_mask(rgb_image, 'blue')   # or 'b'
mask = color_mask(rgb_image, 'yellow') # or 'y'
```

---

## Motion Modules

Located in `motion/`

### test-merge.py

Combined motion controller supporting both linear movement and spinning.

**Topics:**
- Publishes: `/cmd_vel`
- Subscribes: `/odom`, `/motion_command`, `/stop_motion`

```bash
# Start the motion controller
ros2 run tidybot_bringup test-merge.py

# Send commands (from another terminal)
ros2 topic pub /motion_command std_msgs/msg/String "{data: 'search'}" --once   # Start spinning
ros2 topic pub /motion_command std_msgs/msg/String "{data: 'linear'}" --once   # Move forward
ros2 topic pub /stop_motion std_msgs/msg/Bool "{data: true}" --once            # Stop

# With custom speeds
ros2 run tidybot_bringup test-merge.py --ros-args \
    -p linear_speed:=0.05 -p angular_speed:=0.5 -p timeout:=15.0
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `linear_speed` | `0.02` | Forward speed (m/s) |
| `angular_speed` | `0.3` | Rotation speed (rad/s) |
| `timeout` | `10.0` | Auto-stop timeout (s) |

---

### test-spinning.py

Spin the robot in place until stopped.

```bash
# Start spinning
ros2 run tidybot_bringup test-spinning.py --ros-args -p angular_speed:=0.5

# Stop (from another terminal)
ros2 topic pub /stop_spin std_msgs/msg/Bool "{data: true}" --once
```

---

### test-linear.py

Move the robot in a straight line.

```bash
# Move forward
ros2 run tidybot_bringup test-linear.py --ros-args -p linear_speed:=0.1

# Move backward
ros2 run tidybot_bringup test-linear.py --ros-args -p linear_speed:=-0.1

# Stop (from another terminal)
ros2 topic pub /stop_move std_msgs/msg/Bool "{data: true}" --once
```

---

## Speech Modules

Located in `speech/`

### speech_extraction_node.py

Records audio, transcribes with Google Cloud STT, and extracts colors using Gemini.

**Prerequisites:**
```bash
# Set Google Cloud credentials (one of these methods)
export GOOGLE_API_KEY="your-api-key"
# OR
export GOOGLE_APPLICATION_CREDENTIALS="/path/to/service-account.json"
# OR place key at:
# ros2_ws/src/tidybot_bringup/config/credentials/google_cloud_key.json
```

**Topics:**
- Publishes: `/speech_extraction` (tidybot_msgs/SpeechExtraction)
- Calls: `/microphone/record` (service)

```bash
# Mic mode (default, 5-second recording)
ros2 run tidybot_bringup speech_extraction_node.py

# Longer recording
ros2 run tidybot_bringup speech_extraction_node.py --ros-args -p record_duration:=8.0

# File mode (process pre-recorded audio)
ros2 run tidybot_bringup speech_extraction_node.py \
    --ros-args -p use_mic:=false -p audio_file_path:=/path/to/audio.m4a
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_mic` | `true` | Use microphone (false = file mode) |
| `record_duration` | `5.0` | Recording duration in seconds |
| `audio_file_path` | `''` | Path to audio file (file mode) |

---

### test_speech_detection.py

Standalone test script for speech transcription and extraction (no ROS2).

```bash
# Set credentials first
export GOOGLE_API_KEY="your-api-key"

# Run test
python3 speech/test_speech_detection.py /path/to/audio.m4a
```

---

### gemini.py

Utility class for Gemini LLM integration. Used by speech_extraction_node.

```python
from gemini import GeminiClass

gemini = GeminiClass(
    prompt="Extract object_color and bin_color from: ",
    api_key="your-api-key"
)
response = gemini.generate_content("put the red block in the blue bin", use_prompt=True)
# Returns: {"object_color": "red", "bin_color": "blue"}
```

---

## Troubleshooting

### Vision nodes not detecting objects

1. Check camera is publishing:
   ```bash
   ros2 topic hz /camera/color/image_raw
   ```

2. Test with visualization:
   ```bash
   ros2 run tidybot_bringup find_center.py --ros-args -p target_color:=red -p visualize:=true
   ```

3. Adjust HSV thresholds in `color_mask.py` if lighting conditions differ.

### Task node not receiving pose

1. Verify simple_pose_fit is publishing:
   ```bash
   ros2 topic echo /object_pose
   ```

2. Check TF is available:
   ```bash
   ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
   ```

### Speech not working

1. Check microphone node is running:
   ```bash
   ros2 service list | grep microphone
   ```

2. Verify Google credentials:
   ```bash
   echo $GOOGLE_APPLICATION_CREDENTIALS
   # or
   echo $GOOGLE_API_KEY
   ```

3. Test standalone:
   ```bash
   python3 speech/test_speech_detection.py /path/to/test_audio.m4a
   ```

### Motion planner service not available

```bash
# Check service exists
ros2 service list | grep plan_to_target

# If not, start the planner
ros2 run tidybot_ik motion_planner_real_node
```

---

## Architecture Overview

```
task_1/2/3.py (State Machine)
    │
    ├── Vision Pipeline
    │   ├── find_center.py      → /mask_center (Point)
    │   ├── obj_dist.py         → /vision/object_distance (Float32)
    │   └── simple_pose_fit.py  → /object_pose (PoseStamped)
    │
    ├── Motion Control
    │   ├── test-merge.py       ← /motion_command (String)
    │   └── /cmd_vel            → base_driver
    │
    ├── Speech (optional)
    │   └── speech_extraction_node.py → /speech_extraction
    │
    └── Arm Planning
        └── /plan_to_target (service) → motion_planner_real_node
```

See `docs/system_architecture.html` for detailed diagrams.
