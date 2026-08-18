#!/bin/bash
printf '\033]0;[2] Robot GUI - rsdm\007'
# One control panel at a time; camera-viewer instances (--camera) don't count.
if pgrep -af '/robot-gui' | grep -v -- --camera | grep -q robot-gui; then
    echo "Robot GUI is already running."
    read -p "Press Enter to close..."
    exit 0
fi

REPO=/home/bl9b/work/robot-sample-changer
BIN="$REPO/src/robot_gui_rs/target/release/robot-gui"

# RsDM GUI (Rust). Robot:* and the camera's RS405:* both resolve over CA
# broadcast search — set neither EPICS_CA_NAME_SERVERS nor ADDR_LIST here,
# the process talks to two different IOCs (see CLAUDE.md). Images arrive
# over pvAccess through a direct TCP connection (ROBOT_GUI_PVA_SERVER,
# default 127.0.0.1:5085).
"$BIN" "$REPO/config/taught_waypoints.yaml"
exec bash
