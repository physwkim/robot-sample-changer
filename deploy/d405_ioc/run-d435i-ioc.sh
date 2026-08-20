#!/usr/bin/env bash
# Start the RealSense D435i areaDetector IOC.
#
#   PV prefix : RS435:      asyn ports : RS435 / RS435_DEPTH / RS435_PC
#   camera    : serial 348122070162   PVA port : 5075
#
# Runs in the foreground and drops into the iocsh prompt, so `dbl`, `dbpf`,
# `asynReport` all work as usual. Ctrl-C stops acquisition before the IOC
# exits -- see the note below.
#
# The D405 has its own script: run-d405-ioc.sh. The two are separate IOC
# processes (one camera each) and are fine to run at the same time; verified
# at a sustained 30.01 fps on both with zero drops.
set -uo pipefail

WS="/home/bl9b/work/epics-rs-iocs"
BIN="$WS/target/release/d435i-ioc"
CMD="$WS/iocs/d435i-ioc/st.d435i.cmd"
PREFIX="RS435:"
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
# This matters more than it looks. Killing the IOC while the camera is
# streaming leaves the RealSense firmware hung: it keeps answering USB
# enumeration but stops delivering frames, and a USB bus reset does not clear
# it (a bus reset does not remove VBUS, so the camera's ASIC keeps its state).
# Recovering it took an xHCI controller reset, and on the other camera a
# physical replug. So: stop acquisition first, then let the IOC go.
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
# saved mode while RSStreamMode_RBV stays on the driver default and the camera
# streams at that default. This cannot be done inside st.cmd -- the framework
# runs the restore after the script finishes. So wait for the IOC to come up
# and force one process. Harmless when nothing was restored: it re-asserts
# whatever the record already holds.
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

echo "=== D435i IOC ==============================================="
echo "  prefix   : $PREFIX"
echo "  st.cmd   : $CMD"
echo "  PVA port : 5075"
echo
echo "  start acquisition : dbpf ${PREFIX}cam1:Acquire 1"
echo "  frame counter     : dbgf ${PREFIX}cam1:ArrayCounter_RBV"
echo "  BEFORE EXITING    : dbpf ${PREFIX}cam1:Acquire 0"
echo "============================================================="
echo

cd "$WS" || exit 1
"$BIN" "$CMD"
