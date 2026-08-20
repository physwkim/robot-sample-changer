#!/usr/bin/env bash
# Install the librealsense2 SDK that the d435i driver crate links against.
#
# WHY NOT AN APT REPO
# -------------------
# The vendor repo (librealsense.realsenseai.com, formerly librealsense.intel.com)
# serves an InRelease whose clearsigned signature does not verify -- gpgv, which
# is what apt itself uses, calls it BAD. The detached Release.gpg over the same
# Release file verifies GOOD, so the repo contents are authentic and it is the
# Artifactory-generated InRelease that is broken. apt reads InRelease first and
# refuses the repo on that failure, so adding it would only reproduce the error
# on every `apt update`.
#
# The packages here were therefore fetched and verified by hand, rooted in the
# same vendor key apt would have used:
#
#   vendor key  ->  Release.gpg  ->  Release  ->  Packages  ->  *.deb
#
#   - key FB0B24895113F120 (fpr 5381411D24E659FB18195FA5FB0B24895113F120),
#     "RealSense Debian Archive Automatic Signing Key", obtained from BOTH
#     https://librealsense.realsenseai.com/Debian/librealsenseai.asc and
#     keyserver.ubuntu.com -- identical fingerprints. Kept alongside the debs
#     as vendor-key.asc so this is re-checkable.
#   - Release.gpg over Release: gpgv "Good signature"
#   - Packages SHA256 matches the entry in the signed Release
#   - each .deb SHA256 matches its entry in Packages (re-checked below)
#
# VERSION
# -------
# Pinned to 2.56.5 to match `realsense-sys 2.56.5` in Cargo.lock. Its build.rs
# only enforces major version 2, but the FFI structs are generated against the
# 2.56.5 headers, so a newer runtime lib is an ABI gamble for no gain. Held so
# an unattended upgrade cannot drift it.
#
# librealsense2-utils (rs-enumerate-devices, realsense-viewer) is deliberately
# NOT installed: it pulls librealsense2-gl, libgtk-3-dev and a GL stack for
# tooling the IOC does not need. To add it later, fetch it the same way from
# pool/jammy/main/ and verify against Packages.
#
# Run with: sudo /home/bl9b/work/robot-sample-changer/deploy/d405_ioc/install-librealsense.sh
set -euo pipefail

DEB_DIR="/home/bl9b/work/librealsense-debs"
RS_VER="2.56.5-0~realsense.17054"

if [ "$(id -u)" -ne 0 ]; then
    echo "This script needs root: sudo $0" >&2
    exit 1
fi

# --- Undo the apt repo the first attempt added -------------------------------
# It carried the superseded 2018 Intel key (C8B3A55A6F3EFCDE); the repo now
# signs with FB0B24895113F120, which is what produced the NO_PUBKEY error.
# Removing both rather than updating the key: see the InRelease note above --
# the repo cannot be used by apt regardless of which key is installed.
rm -f /etc/apt/sources.list.d/librealsense.list
rm -f /etc/apt/keyrings/librealsense.pgp

# --- Re-verify the debs before installing anything ---------------------------
declare -A SHA=(
  ["librealsense2_${RS_VER}_amd64.deb"]=ef8a31d45159be620b46af412b2c4e42410899f4b3181e7881f5e131f607d8d5
  ["librealsense2-dev_${RS_VER}_amd64.deb"]=c602d5c21c74980d26f8e5072fd7034555f869fd32c4744b981ae6e74c60af8d
  ["librealsense2-udev-rules_${RS_VER}_amd64.deb"]=249bf2d4cb1e0ad62118ac0a58e27122fb036ddb33a396ff8f34d5937313ea21
)
for f in "${!SHA[@]}"; do
    actual=$(sha256sum "$DEB_DIR/$f" | cut -d' ' -f1)
    if [ "$actual" != "${SHA[$f]}" ]; then
        echo "SHA256 mismatch on $f -- refusing to install" >&2
        exit 1
    fi
done
echo "All three packages match the signed Release chain."

# --- Dependencies from the Ubuntu archive ------------------------------------
# librealsense2-udev-rules needs `at` and `v4l-utils`, neither installed here.
apt-get update
apt-get install -y at v4l-utils

# --- Install -----------------------------------------------------------------
# -dev ships realsense2.pc, which is what realsense-sys's build.rs looks for.
# -udev-rules is what makes the cameras openable as a non-root user: without it
# /dev/video* carries only a gdm ACL, so bl9b (already in plugdev) cannot open
# them.
dpkg -i \
    "$DEB_DIR/librealsense2_${RS_VER}_amd64.deb" \
    "$DEB_DIR/librealsense2-dev_${RS_VER}_amd64.deb" \
    "$DEB_DIR/librealsense2-udev-rules_${RS_VER}_amd64.deb"

apt-get install -f -y   # settle anything dpkg left half-configured

apt-mark hold librealsense2 librealsense2-dev librealsense2-udev-rules

# Apply the new rules to the already-plugged cameras without a replug.
udevadm control --reload-rules
udevadm trigger

echo
echo "librealsense2: $(pkg-config --modversion realsense2)"
echo "Camera nodes now readable by plugdev:"
ls -l /dev/video0 || true
