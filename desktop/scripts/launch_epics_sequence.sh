#!/bin/bash
printf '\033]0;[3] EPICS Sequence - epics_triggered_sequence\007'
if pgrep -f "[e]pics_triggered_sequence" > /dev/null; then
    echo "EPICS Sequence is already running."
    read -p "Press Enter to close..."
    exit 0
fi
source /opt/ros/humble/setup.bash
source /home/bl9b/ws/install/setup.bash
ros2 run epics_robot epics_triggered_sequence \
    --ros-args \
    -p arm_group:=ur_manipulator \
    -p waypoints_yaml_path:=/home/bl9b/ws/src/epics_robot/config/taught_waypoints.yaml \
    -p epics_trigger_pv:=Robot:Trigger \
    -p epics_start_step_pv:=Robot:StartStep \
    -p epics_wait_pv:=Robot:Wait \
    -p epics_holder_pv:=Robot:Holder \
    -p epics_stop_pv:=Robot:Stop \
    -p epics_current_step_pv:=Robot:CurrentStep \
    -p epics_gripper_pv:=Robot:Gripper \
    -p gripper_open_threshold:=0.02 \
    -p epics_pause_step_pv:=Robot:PauseStep
exec bash
