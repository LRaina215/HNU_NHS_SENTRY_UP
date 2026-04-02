cmds=(
      "ros2 launch livox_ros_driver2 msg_MID360_cloud_launch.py"
      "ros2 launch point_lio mapping_mid360.launch.py "
     )
     
for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source ../install/setup.bash;$cmd;exec bash;"
     sleep 0.4
done
