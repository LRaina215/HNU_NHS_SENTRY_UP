cmds=(
    "ros2 run bubble_decision rmuc_body"
    "ros2 run rmuc_bt_brain bt_brain_node"
     )

#source /opt/ros/foxy/setup.bash
for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source /home/robomaster/shaobing/install/setup.bash;$cmd;exec bash;"
     sleep 0.2
done
