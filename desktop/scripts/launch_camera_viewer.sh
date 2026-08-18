#!/bin/bash
printf '\033]0;[3] Camera Viewer - D405\007'
REPO=/home/bl9b/work/robot-sample-changer
BIN="$REPO/src/robot_gui_rs/target/release/robot-gui"

# The D405 IOC runs under procServ (console: telnet 20003), started by
# run-d405-ioc.sh — there is no systemd unit for it. Bring it up if gone.
if ! pgrep -f 'release/d435i-ioc' > /dev/null; then
    echo "D405 IOC is not running — starting it (procServ, console 20003)..."
    setsid nohup /home/bl9b/work/run-d405-ioc.sh > /dev/null 2>&1 &
    sleep 5
fi

# Same binary as the Robot GUI, opened on the Camera tab. Images are
# pvAccess by default (see launch_robot_gui.sh for the CA/PVA notes).
"$BIN" --camera "$REPO/config/taught_waypoints.yaml"
exec bash
