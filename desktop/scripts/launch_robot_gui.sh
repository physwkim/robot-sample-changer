#!/bin/bash
printf '\033]0;[4] Robot GUI - robot_control_gui\007'
if pgrep -f "[r]obot_control_gui" > /dev/null; then
    echo "Robot GUI is already running."
    read -p "Press Enter to close..."
    exit 0
fi
source /opt/ros/humble/setup.bash
source /home/bl9b/ws/install/setup.bash
ros2 run robot_gui robot_control_gui
exec bash
