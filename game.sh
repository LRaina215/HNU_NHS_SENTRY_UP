cmds=(
      "ros2 launch livox_ros_driver2 msg_MID360_cloud_launch.py"
#      "ros2 launch rm_description model.launch.py" # tf模型发布放进自瞄中
      "ros2 launch linefit_ground_segmentation_ros segmentation.launch.py"
      "ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
      "ros2 launch point_lio mapping_mid360.launch.py"
      
#      "ros2 launch bubble_protocol sentry_launch.py" # 通信启动放进自瞄中
      "ros2 launch bubble_decision decision_launch.py"
      "ros2 launch icp_registration icp.launch.py"
           
      "ros2 launch navi slam_launch.py"
      "ros2 launch navi localization_launch.py"
      "ros2 launch navi navigation_launch.py"

      "ros2 launch navi rviz_launch.py"
     )

#source /opt/ros/foxy/setup.bash
for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source /home/robomaster/shaobing/install/setup.bash;$cmd;exec bash;"
     sleep 0.2
done
