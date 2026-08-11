#!/bin/bash
printf '\033]0;[2] Robot GUI - robot_control_gui\007'
if pgrep -f "[r]obot_gui.main" > /dev/null; then
    echo "Robot GUI is already running."
    read -p "Press Enter to close..."
    exit 0
fi

# robot_gui is a pure-Python EPICS CA client (silx/PyQt6/pyepics) — no ROS needed.
# It runs from a dedicated conda env that provides silx, since the system/base
# Python does not have it. See CLAUDE.md for env setup.
source /home/bl9b/miniconda3/etc/profile.d/conda.sh
conda activate robot_gui

# Clear ROS PYTHONPATH so the stale colcon build copy of robot_gui
# (~/ws/build/robot_gui) doesn't shadow the source package, and run from
# the package dir so `robot_gui` resolves to the source tree.
unset PYTHONPATH
cd /home/bl9b/ws/src/robot_gui
python -m robot_gui.main
exec bash
