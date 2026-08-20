#!/usr/bin/env bash
# Launch the PyDM viewer(s) for the RealSense IOCs.
#
#   ./run-camera-viewer.sh                 both cameras, dual (colour+depth) view
#   ./run-camera-viewer.sh d435i           D435i only
#   ./run-camera-viewer.sh d405 main       D405, full control screen
#
# The cameras are owned exclusively by the IOCs, so the viewer reads images
# over Channel Access rather than talking to librealsense. Start the IOCs
# first (run-d435i-ioc.sh / run-d405-ioc.sh) and put them in Acquire.
set -uo pipefail

ENV_NAME="pydm"
MICROMAMBA="/home/bl9b/.local/bin/micromamba"
DISPLAY_DIR="/home/bl9b/work/epics-rs-iocs/display"
export PATH="$PATH:/home/bl9b/epics/bin/linux-x86_64"

# Never pin this to a unicast address: this host runs several CA servers
# sharing UDP 5064, and a unicast search reaches only whichever socket the
# kernel happens to pick, so PVs resolve at random. Broadcast reaches all.
unset EPICS_CA_ADDR_LIST

TARGET="${1:-both}"
VIEW="${2:-dual}"

case "$VIEW" in
    dual) SCREEN="d435i_dual_view.py" ;;
    main) SCREEN="d435i_main.py" ;;
    *) echo "usage: $0 [d435i|d405|both] [dual|main]" >&2; exit 1 ;;
esac

case "$TARGET" in
    d435i) PREFIXES="RS435:" ;;
    d405)  PREFIXES="RS405:" ;;
    both)  PREFIXES="RS435: RS405:" ;;
    *) echo "usage: $0 [d435i|d405|both] [dual|main]" >&2; exit 1 ;;
esac

[ -x "$MICROMAMBA" ] || { echo "micromamba not found at $MICROMAMBA" >&2; exit 1; }
[ -f "$DISPLAY_DIR/$SCREEN" ] || { echo "screen not found: $DISPLAY_DIR/$SCREEN" >&2; exit 1; }

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "No DISPLAY set -- this is a GUI. Over SSH, connect with 'ssh -X'." >&2
    exit 1
fi

# DISPLAY being set is not the same as it being reachable. An SSH session can
# export DISPLAY=localhost:10.0 while the forwarding channel or the xauth
# cookie belongs to a different connection -- Qt then fails with a wall of
# "no Qt platform plugin could be initialized" that says nothing about the
# real cause ("could not connect to display"). Probe it and say so plainly.
if [ -n "${DISPLAY:-}" ] && command -v xset >/dev/null 2>&1; then
    if ! xset -display "$DISPLAY" q >/dev/null 2>&1; then
        echo "DISPLAY=$DISPLAY is set but cannot be connected to." >&2
        echo >&2
        echo "  - over SSH: reconnect with 'ssh -X' (or -Y) and retry" >&2
        echo "  - on the machine itself: run this from a logged-in desktop" >&2
        echo "    session, not from a console/tty" >&2
        exit 1
    fi
fi
case "${DISPLAY:-}" in
    localhost:*)
        echo "NOTE: DISPLAY=$DISPLAY looks like SSH X11 forwarding. Full-rate"
        echo "      image streaming over a forwarded X connection is slow; if it"
        echo "      crawls, run the viewer on the console instead."
        echo
        ;;
esac

# --- Per-prefix preflight ----------------------------------------------------
# The screen enables its own plugins now (CC1 + image1 colour routing and
# image2 for depth), so there is nothing to set up here -- just check the IOC
# is actually there, and say so if it is not acquiring, which otherwise looks
# like a broken viewer rather than an idle camera.
prepare() {
    local p="$1"
    if ! caget -w 3 "${p}cam1:Acquire_RBV" >/dev/null 2>&1; then
        echo "  ${p} IOC not reachable -- start it first (run-*-ioc.sh). Skipping."
        return 1
    fi
    local acq
    acq=$(caget -w 3 -t "${p}cam1:Acquire_RBV" 2>/dev/null)
    if [ "$acq" = "Acquiring" ]; then
        echo "  ${p} acquiring"
    else
        echo "  ${p} reachable but NOT acquiring -- 'caput ${p}cam1:Acquire 1' to see frames"
    fi
    return 0
}

echo "=== RealSense viewer ($VIEW) ==="
READY=""
for p in $PREFIXES; do
    prepare "$p" && READY="$READY $p"
done
[ -n "$READY" ] || { echo "No IOC reachable. Nothing to show." >&2; exit 1; }

# --- Launch ------------------------------------------------------------------
PIDS=""
cleanup() {
    echo
    echo "Closing viewer(s) ..."
    for pid in $PIDS; do kill "$pid" 2>/dev/null; done
    wait 2>/dev/null
}
trap cleanup INT TERM

cd "$DISPLAY_DIR" || exit 1
for p in $READY; do
    echo "  launching $SCREEN for $p"
    # -m MUST precede the display file. PyDM treats everything after the
    # displayfile as display_args and passes it through, so `pydm screen.py -m
    # '{...}'` drops the macro silently and every channel binds to the screen's
    # default prefix (RS1:) instead of this camera's.
    "$MICROMAMBA" run -n "$ENV_NAME" pydm -m "{\"P\":\"$p\"}" "$SCREEN" &
    PIDS="$PIDS $!"
done

echo
echo "Viewer(s) running. Ctrl-C here closes them."
echo "Closing the viewer does NOT stop acquisition -- use the IOC's iocsh"
echo "prompt ('dbpf <P>cam1:Acquire 0') before shutting an IOC down."
wait
