#!/bin/bash
printf '\033]0;[4] Camera IOC - D405\007'
# Brings up the D405 areaDetector IOC and starts acquisition, so the Camera
# Viewer has images to show.
#
# The IOC is a systemd *user* unit (d405-ioc.service) running the binary under
# procServ, console on telnet 20003. Do not start it by hand as well: two IOCs
# on the same prefix both answer CA name searches and the second cannot open
# the camera, which looks exactly like a dead camera.
PREFIX=RS405:
UNIT=d405-ioc.service
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
    caget -w 2 "${PREFIX}cam1:Acquire_RBV" >/dev/null 2>&1 && break
    echo -n "."; sleep 2
done
echo

if ! caget -w 3 "${PREFIX}cam1:Acquire_RBV" >/dev/null 2>&1; then
    echo "IOC is up but PVs do not answer. Console: telnet localhost 20003"
    read -p "Press Enter to close..."
    exit 1
fi

# The unit restores stream mode and the PVA plugins from autosave, but leaves
# acquisition off — start it here so the viewer has frames.
[ "$(caget -w 3 -t "${PREFIX}cam1:Acquire_RBV")" = "Acquiring" ] \
    || caput -w 10 "${PREFIX}cam1:Acquire" 1 >/dev/null 2>&1
sleep 6

echo
echo "  mode      : $(caget -w 3 -t "${PREFIX}cam1:RSStreamMode_RBV")"
echo "  acquire   : $(caget -w 3 -t "${PREFIX}cam1:Acquire_RBV")"
a=$(caget -w 3 -t "${PREFIX}cam1:ArrayCounter_RBV"); sleep 4
b=$(caget -w 3 -t "${PREFIX}cam1:ArrayCounter_RBV")
echo "  rate      : $(echo "scale=1;($b-$a)/4" | bc) fps"
echo "  errors    : $(caget -w 3 -t "${PREFIX}cam1:RSErrorCount_RBV")"
echo "  PVA images: ${PREFIX}Pva1:Image (colour), ${PREFIX}depthPva1:Image (depth)"
echo
echo "Console : telnet localhost 20003"
echo "Stop    : systemctl --user stop $UNIT"
echo "          (the unit stops acquisition first — killing the IOC mid-stream"
echo "           wedges the camera firmware and needs a physical replug)"
exec bash
