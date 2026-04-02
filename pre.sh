cmds=(
      "ros2 launch livox_ros_driver2 msg_MID360_cloud_launch.py"
     #  "ros2 launch rm_description model.launch.py"
      "ros2 launch point_lio mapping_mid360.launch.py"
            "python3 ~/shaobing/src/rm_navi/rm_navigation/navi/launch/odom_to_base_node.py"

      # ================== 障碍物分割 (A/B 方案切换) ==================
      
      # 【方案A：原本的 Linefit 方案】(如果新方案出问题，解除这两行的注释即可回退)
      # "ros2 run lidar_filter lidar_filter_node"
      "ros2 launch linefit_ground_segmentation_ros segmentation.launch.py"

      # 【方案B：现在的 Terrain Analysis 方案】(自带车身过滤与动态滤除，XML直接启动)
     #  "ros2 launch terrain_analysis terrain_analysis.launch"

      # ===============================================================

      "ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py" # 建图时启动
      "ros2 launch bubble_decision decision_launch.py" # 决策树
     )
     
for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source ../install/setup.bash;$cmd;exec bash;"
     sleep 0.2
done