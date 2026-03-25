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
    echo "gnome-terminal is required by pre.sh but was not found in PATH."
    exit 1
fi

cmds=(
      "ros2 launch livox_ros_driver2 msg_MID360_cloud_launch.py"
      "ros2 launch rm_description model.launch.py"
      "ros2 launch point_lio mapping_mid360.launch.py"
      "ros2 launch bubble_protocol sentry_launch.py"

      # ================== 障碍物分割 (A/B 方案切换) ==================

      # 【方案A：原本的 Linefit 方案】
      "ros2 launch linefit_ground_segmentation_ros segmentation.launch.py"

      # 【方案B：现在的 Terrain Analysis 方案】
      # "ros2 launch terrain_analysis terrain_analysis.launch"

      # ===============================================================

      "ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
)

for cmd in "${cmds[@]}"; do
    echo "Current CMD : ${cmd}"
    gnome-terminal -- bash -lc "cd \"${SCRIPT_DIR}\"; source \"${SETUP_FILE}\"; ${cmd}; exec bash"
    sleep 0.2
done
