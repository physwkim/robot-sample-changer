#!/bin/bash
printf '\033]0;[2] MoveIt - ur_moveit.launch.py\007'
if pgrep -f "[u]r_moveit.launch.py" > /dev/null; then
    echo "MoveIt is already running."
    read -p "Press Enter to close..."
    exit 0
fi
source /opt/ros/humble/setup.bash
source /home/bl9b/ws/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e \
    description_package:=ur3e_hande_robot_description \
    description_file:=ur_with_hande.xacro \
    moveit_config_package:=ur3e_hande_moveit_config \
    moveit_config_file:=ur.srdf \
    launch_rviz:=false
exec bash
