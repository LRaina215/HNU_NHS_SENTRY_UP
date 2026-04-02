#!/bin/bash
cmds=(
      "ros2 launch navi slam_launch.py"  # 【关键】启动 SLAM Toolbox 进行建图
#      "ros2 launch navi localization_launch.py"  # 【关键】建图时必须注释掉，不读旧地图，不跑AMCL
#      "ros2 launch navi navigation_launch.py"  # 建图时建议注释掉，用键盘/手柄遥控底盘建图更稳
      "ros2 launch navi rviz_launch.py"
     )

for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source ../install/setup.bash;$cmd;exec bash;"
     sleep 0.4
done