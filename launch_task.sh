#!/bin/bash
# launch_task.sh - Launch all nodes required for Task 1 or Task 2
#
# Usage:
#   ./launch_task.sh 1 [block_color]
#   ./launch_task.sh 2 [block_color] [bin_color] [arm_name]
#
# Examples:
#   ./launch_task.sh 1
#   ./launch_task.sh 1 yellow
#   ./launch_task.sh 2
#   ./launch_task.sh 2 yellow blue right
#
# Task 1 nodes:
#   - real.launch.py (hardware + motion planner)
#   - test-merge.py  (motion module: /motion_command -> /cmd_vel)
#   - find_center.py (vision: pixel centroid of object)
#   - simple_pose_fit.py (vision: 6-DOF pose estimation)
#   - task_1.py (orchestrator)
#
# Task 2 nodes:
#   - real.launch.py (hardware + motion planner)
#   - find_center.py (vision: pixel centroid)
#   - obj_dist.py    (vision: object distance)
#   - simple_pose_fit.py (vision: 6-DOF pose estimation)
#   - task_2.py (orchestrator)
#
# Requires tmux. Install with: sudo apt install tmux

set -e

# ── Arguments ──────────────────────────────────────────────────────────────────
TASK="${1:-}"
if [[ -z "$TASK" || ( "$TASK" != "1" && "$TASK" != "2" ) ]]; then
    echo "Usage: $0 <1|2> [options]"
    echo "  Task 1: $0 1 [block_color]"
    echo "  Task 2: $0 2 [block_color] [bin_color] [arm_name]"
    exit 1
fi

BLOCK_COLOR="${2:-yellow}"
BIN_COLOR="${3:-blue}"
ARM_NAME="${4:-right}"

# ── Environment ────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="$SCRIPT_DIR/ros2_ws"
SETUP_ENV="$ROS2_WS/setup_env.bash"

if [[ ! -f "$SETUP_ENV" ]]; then
    echo "ERROR: setup_env.bash not found at $SETUP_ENV"
    exit 1
fi

# ── tmux check ─────────────────────────────────────────────────────────────────
if ! command -v tmux &>/dev/null; then
    echo "ERROR: tmux is not installed. Install with: sudo apt install tmux"
    exit 1
fi

SESSION="tidybot_task${TASK}"

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "Launching Task $TASK nodes in tmux session: $SESSION"
echo "  Block color : $BLOCK_COLOR"
if [[ "$TASK" == "2" ]]; then
    echo "  Bin color   : $BIN_COLOR"
    echo "  Arm name    : $ARM_NAME"
fi
echo ""
echo "Attach with:  tmux attach -t $SESSION"
echo "Kill all with: tmux kill-session -t $SESSION"
echo ""

# Shared preamble to source environment in each pane
SOURCE_CMD="source $SETUP_ENV"

# ── Helper: run a command in a new tmux window ─────────────────────────────────
new_window() {
    local name="$1"
    local cmd="$2"
    local delay="${3:-0}"  # optional startup delay in seconds

    tmux new-window -t "$SESSION" -n "$name"
    if [[ "$delay" -gt 0 ]]; then
        tmux send-keys -t "$SESSION:$name" \
            "$SOURCE_CMD && sleep $delay && $cmd" Enter
    else
        tmux send-keys -t "$SESSION:$name" \
            "$SOURCE_CMD && $cmd" Enter
    fi
}

# ── Create tmux session ────────────────────────────────────────────────────────
# Window 0: hardware launch (real.launch.py)
tmux new-session -d -s "$SESSION" -n "hardware"
tmux send-keys -t "$SESSION:hardware" \
    "$SOURCE_CMD && ros2 launch tidybot_bringup real.launch.py" Enter

# Wait for hardware to be ready before launching other nodes.
# We use sleep in each subsequent window to stagger startup.
HARDWARE_WAIT=15  # seconds for hardware + motion planner to initialize

if [[ "$TASK" == "1" ]]; then
    # ── Task 1 ──────────────────────────────────────────────────────────────────
    # Motion module: listens to /motion_command and /stop_motion, publishes /cmd_vel
    new_window "motion" \
        "ros2 run tidybot_bringup test-merge.py" \
        "$HARDWARE_WAIT"

    # Vision: pixel centroid of target object
    new_window "find_center" \
        "ros2 run tidybot_bringup find_center.py --ros-args -p target_color:=$BLOCK_COLOR" \
        "$HARDWARE_WAIT"

    # Vision: 6-DOF pose estimation of target object
    new_window "pose_fit" \
        "ros2 run tidybot_bringup simple_pose_fit.py --ros-args -p target_color:=$BLOCK_COLOR" \
        "$HARDWARE_WAIT"

    # Orchestrator (start a bit later to ensure all nodes are up)
    new_window "task_1" \
        "ros2 run tidybot_bringup task_1.py --ros-args -p arm_name:=$ARM_NAME" \
        "$((HARDWARE_WAIT + 3))"

elif [[ "$TASK" == "2" ]]; then
    # ── Task 2 ──────────────────────────────────────────────────────────────────
    # Vision: pixel centroid of target object
    new_window "find_center" \
        "ros2 run tidybot_bringup find_center.py --ros-args -p target_color:=$BLOCK_COLOR" \
        "$HARDWARE_WAIT"

    # Vision: object distance from depth camera
    new_window "obj_dist" \
        "ros2 run tidybot_bringup obj_dist.py --ros-args -p target_color:=$BLOCK_COLOR" \
        "$HARDWARE_WAIT"

    # Vision: 6-DOF pose estimation
    new_window "pose_fit" \
        "ros2 run tidybot_bringup simple_pose_fit.py --ros-args -p target_color:=$BLOCK_COLOR" \
        "$HARDWARE_WAIT"

    # Orchestrator
    new_window "task_2" \
        "ros2 run tidybot_bringup task_2.py --ros-args -p block_color:=$BLOCK_COLOR -p bin_color:=$BIN_COLOR -p arm_name:=$ARM_NAME" \
        "$((HARDWARE_WAIT + 3))"
fi

# Focus on the orchestrator window
tmux select-window -t "$SESSION:$([ "$TASK" == "1" ] && echo task_1 || echo task_2)"

# Attach to the session
tmux attach -t "$SESSION"
