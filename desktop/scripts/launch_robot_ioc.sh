#!/bin/bash
printf '\033]0;[0] Robot IOC - robot_ioc\007'
# Brings up the robot EPICS soft IOC, so the Robot Sequencer has PVs to
# connect to. Run this before [1] Robot Sequencer -- the daemon exits five
# seconds after start with "PV 'Robot:Trigger' is not connected" otherwise.
#
# The IOC is a systemd *user* unit (robot-ioc.service) running the binary
# under procServ, console on telnet 20001. Do not start it by hand as well:
# two IOCs on the same prefix both answer CA name searches and the second
# one's PVs win at random.
PREFIX=Robot:
UNIT=robot-ioc.service
export PATH=$PATH:/home/bl9b/epics/bin/linux-x86_64
# Never pin a unicast EPICS_CA_ADDR_LIST here: several CA servers on this host
# share UDP 5064 and a unicast search reaches only whichever the kernel picks.
unset EPICS_CA_ADDR_LIST

if systemctl --user is-active --quiet "$UNIT"; then
    echo "IOC already running (systemd: $UNIT)."
else
    echo "Starting $UNIT ..."
    systemctl --user start "$UNIT" || { echo "FAILED to start $UNIT"; read -p "Press Enter to close..."; exit 1; }
fi

echo -n "Waiting for PVs "
for _ in $(seq 1 30); do
    caget -w 2 "${PREFIX}Trigger" >/dev/null 2>&1 && break
    echo -n "."; sleep 2
done
echo

if ! caget -w 3 "${PREFIX}Trigger" >/dev/null 2>&1; then
    echo "IOC is up but PVs do not answer. Console: telnet localhost 20001"
    read -p "Press Enter to close..."
    exit 1
fi

# Which db actually loaded. The binary's compile default is ~/ws/db, an older
# copy with no Robot:MapSource record and no CalibMode 3-7 labels, so grip
# null (6) and holder transfer (7) would lose the PVs they steer by. The unit
# sets ROBOT_DB to this repository's db; this is the check that it took.
if ! caget -w 3 "${PREFIX}MapSource" >/dev/null 2>&1; then
    echo
    echo "WARNING: ${PREFIX}MapSource is missing — the IOC loaded the wrong db."
    echo "         Expected ROBOT_DB=/home/bl9b/work/robot-sample-changer/db;"
    echo "         check 'systemctl --user cat $UNIT'."
fi

echo
echo "  trigger   : $(caget -w 3 -t "${PREFIX}Trigger")"
echo "  step      : $(caget -w 3 -t "${PREFIX}CurrentStep")  (>0 = interrupted sequence)"
echo "  holder    : $(caget -w 3 -t "${PREFIX}Holder")   source: $(caget -w 3 -t "${PREFIX}MapSource" 2>/dev/null || echo '?')"
echo "  mode      : $(caget -w 3 -t "${PREFIX}CalibMode")"
echo "  loaded    : $(caget -w 3 -t "${PREFIX}Loaded")"
echo "  grip null : $(caget -w 3 -t "${PREFIX}Null:State")  $(caget -w 3 -t "${PREFIX}Null:Msg")"
echo
echo "  (state above is autosave-restored; the sequencer overwrites it on the"
echo "   next trigger. CurrentStep > 0 means resume per CLAUDE.md, not idle.)"
echo
echo "Console : telnet localhost 20001   (Ctrl-] then 'quit' to leave it running)"
echo "Next    : [1] Robot Sequencer"
echo "Stop    : systemctl --user stop $UNIT"
exec bash
