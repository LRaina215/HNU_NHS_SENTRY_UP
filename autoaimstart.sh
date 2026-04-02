#!/bin/bash

# 指定ROS的日志文件夹并赋予权限，否则会运行失败
export ROS_LOG_DIR=/home/robomaster/shaobing/log

# 激活ROS环境 (这里是父终端的环境)
source /opt/ros/galactic/setup.bash
source /home/robomaster/shaobing/install/setup.bash

# 设置 OpenCV 库路径 (这是我们刚才修正的正确路径)
export LD_LIBRARY_PATH=/home/robomaster/opencv_4.5.4_local/lib:$LD_LIBRARY_PATH

# 定义需要监控的节点和启动命令
declare -A NODES=(
    ["hik_camera"]="ros2 launch hik_camera hik_camera.launch.py"
    ["rm_description"]="ros2 launch rm_description model.launch.py"
    ["auto_aim"]="ros2 launch auto_aim_bringup auto_aim.launch.py"
    ["sentry_up_serial"]="ros2 launch bubble_protocol sentry_launch.py"
)

# ANSI 颜色代码
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # 重置颜色

# 全局标志文件
FLAG_FILE="/tmp/autostart_enabled"
# 创建标志文件
touch "$FLAG_FILE"

echo "Autostart monitoring started. Check /tmp/autostart_enabled to stop."

# 监控节点状态
while [ -f "$FLAG_FILE" ]; do
    # ↑↑↑ 注意：这里改成了 do
    
    for NODE_NAME in "${!NODES[@]}"; do
        # 检查节点是否启动 (这里用 grep 检查命令字符串)
        # 注意：如果命令很长，ps -ef 可能会截断导致误判，但暂时先这样用
        PIDS=$(ps -ef | grep "${NODES[$NODE_NAME]}" | grep -v grep | awk '{print $2}')
        
        if [ "$PIDS" != "" ]; then
            echo -e "${GREEN}$NODE_NAME is running! (PID: $PIDS)${NC}"
        else
            # 如果节点没有启动则启动
            echo -e "${RED}Start $NODE_NAME ! ! ! ! !${NC}"
            
            # 在新的终端中启动节点
            # 关键修改：我们在新终端里显式 source 了 setup.bash，确保环境变量正确
            gnome-terminal -- bash -c "source /home/robomaster/shaobing/install/setup.bash; export LD_LIBRARY_PATH=$LD_LIBRARY_PATH; echo 'Starting $NODE_NAME...'; ${NODES[$NODE_NAME]}; exec bash"
        fi
    done
    
    # 建议注释掉 clear，否则屏幕会一直闪烁，看不到报错信息
    # clear 
    echo "-----------------------------------------------------"
    sleep 5

done

echo "Autostart script stopped."
