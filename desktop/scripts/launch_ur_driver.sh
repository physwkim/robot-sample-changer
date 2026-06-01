#!/bin/bash
printf '\033]0;[1] UR Driver - ur_control.launch.py\007'
if pgrep -f "[u]r_control.launch.py" > /dev/null; then
    echo "UR Driver is already running."
    read -p "Press Enter to close..."
    exit 0
fi
source /opt/ros/humble/setup.bash
source /home/bl9b/ws/install/setup.bash
ros2 launch ur3e_hande_robot_description ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.192.10 \
    description_file:=ur_with_hande.xacro \
    runtime_config_package:=ur3e_hande_robot_description \
    controllers_file:=ur_hande_controllers.yaml \
    gripper_use_fake_hardware:=false \
    use_tool_communication:=true \
    tool_voltage:=24 \
    launch_rviz:=false
exec bash
