#!/bin/bash
printf '\033]0;[1] Robot Sequencer - robot-sequencer\007'
if pgrep -f "[r]obot-sequencer" > /dev/null; then
    echo "Robot Sequencer is already running."
    read -p "Press Enter to close..."
    exit 0
fi
# ROS-free daemon: talks CA to robot_ioc (systemd) and drives the UR +
# Hand-E directly. Build: cd ~/ws/src/robot_sequencer && cargo build --release
/home/bl9b/ws/src/robot_sequencer/target/release/robot-sequencer \
    /home/bl9b/ws/config/sequencer.yaml
exec bash
