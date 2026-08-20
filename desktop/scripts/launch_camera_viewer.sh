#!/bin/bash
printf '\033]0;[4] Camera Viewer - D405\007'
REPO=/home/bl9b/work/robot-sample-changer
BIN="$REPO/src/robot_gui_rs/target/release/robot-gui"

# The D405 IOC is a systemd user unit (console: telnet 20003). Ask systemd
# rather than starting a copy by hand: a hand-started IOC lives outside the
# unit, so starting the service afterwards leaves two IOCs on the RS405:
# prefix. Both answer CA name searches, only one can hold the camera, and the
# other looks exactly like dead hardware.
if ! systemctl --user is-active --quiet d405-ioc.service; then
    echo "D405 IOC is not running — starting it (systemd, console 20003)..."
    systemctl --user start d405-ioc.service
    sleep 5
fi

# Same binary as the Robot GUI, opened on the Camera tab. Images are
# pvAccess by default (see launch_robot_gui.sh for the CA/PVA notes).
"$BIN" --camera "$REPO/config/taught_waypoints.yaml"
exec bash
