#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WORKSPACE_ROOT}/install/setup.bash"

if [[ ! -f "${SETUP_FILE}" ]]; then
    echo "Missing workspace overlay: ${SETUP_FILE}"
    echo "Please build the workspace first, for example:"
    echo "  cd ${WORKSPACE_ROOT}"
    echo "  colcon build"
    exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is required by nav.sh but was not found in PATH."
    exit 1
fi

cmds=(
#      "ros2 launch navi slam_launch.py"
      "ros2 launch icp_registration icp.launch.py"
      "ros2 launch navi localization_launch.py"
      "ros2 launch navi navigation_launch.py"
#      "ros2 topic echo /tf"
#      "ros2 topic echo /cmd_vel"
      "ros2 launch navi rviz_launch.py"
)

for cmd in "${cmds[@]}"; do
    echo "Current CMD : ${cmd}"
    gnome-terminal -- bash -lc "cd \"${SCRIPT_DIR}\"; source \"${SETUP_FILE}\"; ${cmd}; exec bash"
    sleep 0.4
done
