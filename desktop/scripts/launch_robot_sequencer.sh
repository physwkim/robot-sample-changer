#!/bin/bash
printf '\033]0;[1] Robot Sequencer - robot-sequencer\007'
if pgrep -x "robot-sequencer" > /dev/null; then
    echo "Robot Sequencer is already running."
    read -p "Press Enter to close..."
    exit 0
fi
# ROS-free daemon: talks CA to robot_ioc (systemd --user, started by
# [0] Robot IOC) and drives the UR +
# Hand-E directly. Build: cd src/robot_sequencer && cargo build --release
REPO=/home/bl9b/work/robot-sample-changer
"$REPO/src/robot_sequencer/target/release/robot-sequencer" \
    "$REPO/config/sequencer.yaml"
exec bash
