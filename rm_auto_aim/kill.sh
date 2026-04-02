#!/bin/bash

# 定义需要终止的节点
declare -A NODES=(
    ["hik_camera"]="ros2 launch hik_camera hik_camera.launch.py"
    ["rm_description"]="ros2 launch rm_description model.launch.py"
    ["armor_detector"]="ros2 run armor_detector armor_detector_node"
    ["armor_solver"]="ros2 run armor_solver armor_solver_node"
    ["sentry_up_serial"]="ros2 launch bubble_protocol sentry_up_serial_launch.py"
)

# 终止所有节点
for NODE_NAME in "${!NODES[@]}"; do
    PIDS=$(ps -ef | grep "${NODES[$NODE_NAME]}" | grep -v grep | awk '{print $2}')
    if [ "$PIDS" != "" ]; then
        echo "Killing $NODE_NAME (PIDs: $PIDS)..."
        kill -9 $PIDS
    else
        echo "$NODE_NAME is not running."
    fi
done

# 终止所有 ROS 2 相关进程
echo "Killing all remaining ROS 2 processes..."
pkill -f ros2

# 终止 autostart.sh 脚本
AUTOSTART_PIDS=$(ps -ef | grep "autostart.sh" | grep -v grep | awk '{print $2}')
if [ "$AUTOSTART_PIDS" != "" ]; then
    echo "Killing autostart.sh (PIDs: $AUTOSTART_PIDS)..."
    kill -9 $AUTOSTART_PIDS
else
    echo "autostart.sh is not running."
fi

echo "All nodes, ROS 2 processes, and autostart.sh have been terminated."