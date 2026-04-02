cmds=(
#      "ros2 launch navi slam_launch.py"
      # "ros2 launch icp_registration icp.launch.py"
      "ros2 launch navi localization_launch.py"
      "ros2 launch navi navigation_launch.py"
#      "ros2 topic echo /tf"
#      "ros2 topic echo /cmd_vel"
      "ros2 launch navi rviz_launch.py"
      # "ros2 launch fake_vel_transform fake_vel_transform_launch.py "
     )

for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source ../install/setup.bash;$cmd;exec bash;"
     sleep 0.4
done
