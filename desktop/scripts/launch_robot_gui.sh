#!/bin/bash
printf '\033]0;[2] Robot GUI - robot_control_gui\007'
if pgrep -f "[r]obot_gui.main" > /dev/null; then
    echo "Robot GUI is already running."
    read -p "Press Enter to close..."
    exit 0
fi

# robot_gui is a pure-Python EPICS CA client (silx/PyQt6/pyepics) — no ROS.
# It runs from a dedicated conda env that provides silx, since the system
# Python does not have it. See CLAUDE.md for env setup.
source /home/bl9b/miniconda3/etc/profile.d/conda.sh
conda activate robot_gui

# Resolve Robot:* over TCP at the local IOC. This host is multihomed and
# shares 5064 with other CA servers; NAME_SERVERS pins the lookup without
# breaking broadcast search for anyone else (never use EPICS_CA_ADDR_LIST
# here — see CLAUDE.md).
export EPICS_CA_NAME_SERVERS=127.0.0.1:5064

unset PYTHONPATH
cd /home/bl9b/work/robot-sample-changer/src/robot_gui
python -m robot_gui.main
exec bash
