#!/bin/bash
# Helper script to run FoundationPose node with correct environment
#
# Usage:
#   ./run_foundation_pose.sh --mesh_file /path/to/object.obj
#
# This script activates the FoundationPose venv and runs the ROS2 node

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${TIDYBOT_REPO_ROOT:-$HOME/collaborative-robotics-2026}"
FOUNDATION_POSE_VENV="$REPO_ROOT/deps/FoundationPose/.venv"

# Check if FoundationPose venv exists
if [ ! -d "$FOUNDATION_POSE_VENV" ]; then
    echo "ERROR: FoundationPose venv not found at $FOUNDATION_POSE_VENV"
    echo "Please install FoundationPose first:"
    echo "  cd $REPO_ROOT/deps/FoundationPose"
    echo "  python3 -m venv .venv"
    echo "  source .venv/bin/activate"
    echo "  pip install -r requirements.txt"
    exit 1
fi

# Activate FoundationPose venv
source "$FOUNDATION_POSE_VENV/bin/activate"

# Run the node with all passed arguments
python3 "$SCRIPT_DIR/foundation_pose_node.py" "$@"
