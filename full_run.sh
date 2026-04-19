#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS setup file not found at $ROS_SETUP"
  echo "Install ROS 2 Jazzy or update this path in full_run.sh"
  exit 1
fi

# ROS setup scripts may reference unset vars internally; temporarily relax nounset.
set +u
source "$ROS_SETUP"
set -u

if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: colcon is not installed or not on PATH."
  exit 1
fi

echo "Building drone_sim package..."
colcon build --symlink-install --packages-select drone_sim

set +u
source "$ROOT_DIR/install/setup.bash"
set -u
export SAR_RUN_MODE=full

echo "Starting SAR Autonomous Drone in FULL mode..."
exec ros2 launch drone_sim drone_sim.launch.py
