#!/usr/bin/env bash
# Reset the xHCI host controller to recover wedged RealSense cameras.
#
# WHY THIS EXISTS
# ---------------
# Two distinct failure states showed up after an unclean stream teardown:
#
#   D405  (port 2-1) : kernel cannot enumerate it at all --
#                      "device not accepting address, error -71",
#                      "unable to enumerate USB device"
#   D435i (port 2-2) : enumerates fine at 5000M, but no isochronous frames
#                      arrive. Reproducible with plain librealsense C code,
#                      so it is not an IOC or driver problem.
#
# A USB *bus* reset (USBDEVFS_RESET) was already tried and did not help. That
# re-runs link training and enumeration but leaves VBUS on, so the camera's
# D4 vision ASIC keeps its hung state. The kernel also tried its own port
# power cycle ("attempt power cycle" in dmesg) and still failed: root-hub
# ports on PCH-integrated xHCI generally have no per-port VBUS switch, so the
# request is a no-op in hardware.
#
# Resetting the whole host controller is the one remaining software lever. It
# forces re-enumeration of every port on the controller and often clears the
# error -71 state. It does NOT guarantee VBUS removal, so a truly hung ASIC
# may still need a physical unplug -- this is worth trying first, not a
# guaranteed fix.
#
# WHY THIS IS SAFE ON THIS MACHINE
# --------------------------------
# Normally a controller reset would drop the keyboard and mouse too. Here the
# machine has exactly one USB controller (00:14.0) and the RealSense cameras
# are the only things attached to it -- no input devices, no storage. The
# script re-checks this before touching anything and refuses if that changes.
#
# Run with: sudo /home/bl9b/work/robot-sample-changer/deploy/d405_ioc/reset-usb-controller.sh
set -euo pipefail

PCI_ID="0000:00:14.0"
DRV="/sys/bus/pci/drivers/xhci_hcd"
LIST_TOOL="/home/bl9b/work/librealsense-debs/rs-list-devices"

if [ "$(id -u)" -ne 0 ]; then
    echo "This script needs root: sudo $0" >&2
    exit 1
fi

if [ ! -e "$DRV/$PCI_ID" ]; then
    echo "$PCI_ID is not bound to xhci_hcd -- nothing to reset." >&2
    echo "Bound controllers:" >&2
    ls -1 "$DRV" | grep '^0000:' >&2 || true
    exit 1
fi

# --- Refuse if anything other than a RealSense camera is on this controller ---
# The whole reason this is safe is that only the cameras are attached. If a
# keyboard or a disk has since been plugged into it, resetting would take that
# down too, so stop rather than surprise the operator.
strays=0
for d in /sys/devices/pci0000:00/$PCI_ID/usb*/*-*; do
    [ -f "$d/idVendor" ] || continue
    vid=$(cat "$d/idVendor")
    name=$(cat "$d/product" 2>/dev/null || echo "unknown")
    if [ "$vid" != "8086" ]; then
        echo "Non-RealSense device on this controller: $(basename "$d") $name [$vid]" >&2
        strays=1
    else
        echo "Will reset: $(basename "$d") $name"
    fi
done
if [ "$strays" -ne 0 ]; then
    echo >&2
    echo "Refusing to reset -- something other than a camera would be dropped." >&2
    exit 1
fi

# --- Reset -------------------------------------------------------------------
# The trap guarantees the rebind even if the unbind half throws, so the
# controller is never left detached.
rebind() {
    if [ ! -e "$DRV/$PCI_ID" ]; then
        echo "Rebinding $PCI_ID ..."
        echo "$PCI_ID" > "$DRV/bind" || {
            echo "REBIND FAILED -- USB is down on $PCI_ID. Reboot to recover." >&2
            return 1
        }
    fi
}
trap rebind EXIT

echo "Unbinding $PCI_ID from xhci_hcd ..."
echo "$PCI_ID" > "$DRV/unbind"
sleep 3
rebind
trap - EXIT

# --- Wait for re-enumeration and report --------------------------------------
# Be patient here. A port stuck in the error -71 retry loop holds up the hub
# thread for a long time -- the kernel walks several addresses and its own
# power-cycle attempt on the bad port before it gets to the good one. An
# earlier version of this script sampled after ~23s, reported "cameras did not
# come back", and was simply wrong: the working camera turned up shortly after.
# So: wait for the bus to go quiet rather than for the first device to appear,
# and do not call it a failure until the full window has elapsed.
echo "Waiting for re-enumeration (a failing port stretches this out) ..."
stable=0
for _ in $(seq 1 90); do
    sleep 1
    count=$(ls -d /sys/devices/pci0000:00/$PCI_ID/usb*/*-* 2>/dev/null | wc -l)
    if [ "$count" -gt 0 ]; then
        stable=$((stable + 1))
        # Two consecutive quiet seconds with something attached: settled.
        [ "$stable" -ge 2 ] && break
    else
        stable=0
    fi
done
sleep 2

echo
echo "=== USB devices on $PCI_ID ==="
found=0
for d in /sys/devices/pci0000:00/$PCI_ID/usb*/*-*; do
    [ -f "$d/product" ] || continue
    echo "  $(basename "$d"): $(cat "$d/product") @ $(cat "$d/speed" 2>/dev/null)M"
    found=1
done
[ "$found" -eq 1 ] || echo "  (none attached)"

# A port that never enumerates is the one case this reset cannot fix: the
# controller comes back, the device on that port does not. Name it explicitly
# instead of leaving the operator to read the kernel log.
for p in /sys/bus/usb/devices/usb*/; do :; done
if journalctl -k -n 60 --no-pager 2>/dev/null | grep -q "unable to enumerate USB device"; then
    echo
    echo "NOTE: a port still fails to enumerate (error -71 in the log below)."
    echo "      A controller reset cannot cut VBUS, so a device whose own USB"
    echo "      PHY is wedged needs a physical unplug/replug. Try a different"
    echo "      USB3 port while you are at it."
fi

echo
echo "=== librealsense enumeration ==="
# librealsense2-utils was deliberately not installed (it pulls a GTK/GL stack),
# so this is the small helper built against the SDK instead.
if [ -x "$LIST_TOOL" ]; then
    "$LIST_TOOL" || echo "  enumeration failed"
else
    echo "  $LIST_TOOL not found"
fi

echo
echo "Recent kernel USB messages:"
journalctl -k -n 40 --no-pager 2>/dev/null | grep -iE "usb|xhci" | tail -8 || true
