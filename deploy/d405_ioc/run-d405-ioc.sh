#!/usr/bin/env bash
# Start the RealSense D405 areaDetector IOC.
#
#   PV prefix : RS405:      asyn ports : RS405 / RS405_DEPTH / RS405_PC
#   camera    : serial 315122272475   PVA port : 5085
#
# Same binary and same driver as the D435i script -- the driver reads the
# camera's capabilities instead of assuming a model, so on this camera
# RSHasIMU_RBV and RSHasEmitter_RBV come up 0 and RSDepthUnits_RBV is 0.0001 m
# (the D435i reports 0.001).
#
# The PVA port differs from the D435i's because two IOCs on one host cannot
# both bind 5075. CA needs no such split.
set -uo pipefail

WS="/home/bl9b/work/epics-rs-iocs"
BIN="$WS/target/release/d435i-ioc"
CMD="$WS/iocs/d435i-ioc/st.d405.cmd"
PREFIX="RS405:"
export PATH="$PATH:/home/bl9b/epics/bin/linux-x86_64"

# Do NOT set EPICS_CA_ADDR_LIST to a unicast address here. This host runs
# several CA servers sharing UDP 5064, and a unicast search reaches only
# whichever socket the kernel picks -- PVs then resolve at random. Broadcast
# (the default) reaches all of them.
unset EPICS_CA_ADDR_LIST

if [ ! -x "$BIN" ]; then
    echo "Binary not built yet. Run:" >&2
    echo "  cargo build -p d435i-ioc --release --manifest-path $WS/Cargo.toml" >&2
    exit 1
fi

# --- Clean shutdown ----------------------------------------------------------
# Killing the IOC mid-stream leaves the RealSense firmware hung: it still
# enumerates over USB but delivers no frames, and a USB bus reset will not
# clear it (a bus reset leaves VBUS on, so the camera's ASIC keeps its state).
# This camera is the one that had to be physically unplugged to come back.
#
# Best effort only -- Ctrl-C reaches the IOC at the same time as this trap.
# The reliable way is to type this at the iocsh prompt before exiting:
#     dbpf ${PREFIX}cam1:Acquire 0
cleanup() {
    echo
    echo "Stopping acquisition on $PREFIX ..."
    caput -w 5 "${PREFIX}cam1:Acquire" 0 >/dev/null 2>&1 || true
    sleep 2
}
trap cleanup INT TERM

# --- Finish the autosave restore --------------------------------------------
# pass1 restore writes the saved VAL into RSStreamMode but never processes the
# record, so the mode does not reach the driver: RSStreamMode reads back the
# saved 15fps while RSStreamMode_RBV stays on the driver default of 30 and the
# camera streams at 30. This cannot be done inside st.cmd -- the framework runs
# the restore after the script finishes. So wait for the IOC to come up and
# force one process. Harmless when nothing was restored: it re-asserts whatever
# the record already holds.
(
    for _ in $(seq 1 40); do
        sleep 2
        caget -w 2 "${PREFIX}cam1:RSStreamMode" >/dev/null 2>&1 && break
    done
    sleep 2
    want=$(caget -w 3 -t -n "${PREFIX}cam1:RSStreamMode" 2>/dev/null)
    have=$(caget -w 3 -t -n "${PREFIX}cam1:RSStreamMode_RBV" 2>/dev/null)
    if [ -n "$want" ] && [ "$want" != "$have" ]; then
        caput -w 5 "${PREFIX}cam1:RSStreamMode.PROC" 1 >/dev/null 2>&1
        echo "[restore] stream mode pushed to driver: index $want"
    fi
) &

echo "=== D405 IOC ================================================"
echo "  prefix   : $PREFIX"
echo "  st.cmd   : $CMD"
echo "  PVA port : 5085"
echo
echo "  start acquisition : dbpf ${PREFIX}cam1:Acquire 1"
echo "  frame counter     : dbgf ${PREFIX}cam1:ArrayCounter_RBV"
echo "  BEFORE EXITING    : dbpf ${PREFIX}cam1:Acquire 0"
echo "============================================================="
echo

cd "$WS" || exit 1
"$BIN" "$CMD"
