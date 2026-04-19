#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS setup file not found at $ROS_SETUP"
  echo "Install ROS 2 Jazzy or update this path in quick_run.sh"
  exit 1
fi

# ROS setup scripts may reference unset vars internally; temporarily relax nounset.
set +u
source "$ROS_SETUP"
set -u

if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
  echo "ERROR: Workspace is not built yet."
  echo "Run ./full_run.sh once, then use ./quick_run.sh for fast startup."
  exit 1
fi

set +u
source "$ROOT_DIR/install/setup.bash"
set -u
export SAR_RUN_MODE=quick

echo "Starting SAR Autonomous Drone in QUICK mode..."
exec ros2 launch drone_sim drone_sim.launch.py
