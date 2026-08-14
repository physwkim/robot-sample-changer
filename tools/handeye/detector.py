#!/usr/bin/env python3
"""AprilTag detector for handeye_collect, driven over stdin/stdout.

On startup it writes one unsolicited readiness line, then one JSON reply
per JSON request:

    (startup)         -> {"ok":true,"cmd":"hello","message":"..."}
    {"cmd":"detect"}  -> {"ok":true,"cmd":"detect","t":[...],"R":[9],...}
                      -> {"ok":false,"cmd":"detect","reason":"..."}
    {"cmd":"quit"}    -> exits, no reply

Every reply echoes the command it answers so the parent can tell that it
is reading the answer to the request it just sent. Without that echo an
extra or missing line shifts the whole stream by one and the parent pairs
each robot pose with the previous pose's tag observation -- every sample
plausible, every sample wrong.

It runs as a child of the Rust capture tool rather than as a library
because the detection stack (cv2) has no Rust equivalent here, and as a
persistent process rather than one invocation per pose because importing
cv2 and numpy costs more than the detection itself.

Intrinsics come from the IOC, not from a constant: the old ROS node
hardcoded a D405's matrix, which is wrong for any other unit and silently
scales every distance it reports.
"""

import json
import os
import sys
import time

os.environ.setdefault("EPICS_CA_MAX_ARRAY_BYTES", "20000000")

import cv2
import numpy as np
from epics import PV, caget

PREFIX = os.environ.get("HANDEYE_CAM_PREFIX", "RS405:")
# The size the sheet was DRAWN at, deliberately, and not the 0.098585 the
# calibration measured (doc/handeye_calibration.md). Seeding the capture
# with a previous run's answer would be wrong in exactly the case someone
# recalibrates for -- a resized reprint, a sheet remounted flatter -- and
# the joint solve does not need the seed to be right: it refits the size
# from corners_px and reached 98.575 mm starting from this 100. What this
# value does reach is cam_t_tag, whose range is short by the same 1.4 %,
# so anything reading that field directly wants the measured size instead.
# Set HANDEYE_TAG_SIZE_M to override for a sheet known to differ.
TAG_SIZE_M = float(os.environ.get("HANDEYE_TAG_SIZE_M", "0.100"))
# The stream carrying depth, alongside image1's mono frame.
DEPTH_PLUGIN = os.environ.get("HANDEYE_DEPTH_PLUGIN", "image2")
# Trimmed off the tag polygon before the plane is fitted, so the fit sees
# only the sheet: stereo smears a few pixels either side of the tag's
# border, and the depth map is not pixel-identical to image1 (see
# `tag_plane`).
PLANE_ERODE_PX = 15
# Below this the patch is too small or too broken to call a plane.
MIN_PLANE_PX = 500
# The 100 mm sheet is id 0 (doc/apriltags/). Anything else in frame is a
# holder tag and must not be mistaken for the calibration target.
TAG_ID = int(os.environ.get("HANDEYE_TAG_ID", "0"))
# RSDistModel reports BrownConradyInverse, which reads like the opposite
# of the forward Brown-Conrady model solvePnP assumes. It is not. Asking
# librealsense2.so itself, rs2_project_point_to_pixel on these same
# coefficients reproduces cv2.projectPoints exactly (0.0000 px over the
# frame) under RS2_DISTORTION_BROWN_CONRADY, and to 0.0223 px under
# INVERSE_BROWN_CONRADY -- which librealsense projects identically to
# MODIFIED_BROWN_CONRADY, so that residue is a branch difference inside
# librealsense and not a disagreement with OpenCV. "Inverse" names the
# direction relative to MODIFIED_BROWN_CONRADY's deprojection, not
# relative to OpenCV. So the coefficients go in as-is; dropping them
# would move a corner by 5.8 px, 4.3 mm at 290 mm.
# tools/handeye/check_distortion_model.py prints all of this.


def reply(**kw):
    sys.stdout.write(json.dumps(kw) + "\n")
    sys.stdout.flush()


def fresh_frame(data, counter, timeout=5.0, advance=2):
    """A frame the camera exposed after this call started.

    Two things conspire to hand back a stale image, and both did: pyepics
    returns its cached value from `get()` unless `use_monitor=False`, and
    even a genuine CA read can land on the frame that was in flight while
    the arm was still moving. Either one silently records the previous
    pose's tag position at the new pose -- every sample consistent, every
    sample wrong, and `calibrateHandEye` converges on it without
    complaint. So: wait for the counter to advance past where it was, and
    only then read the pixels.
    """
    start = counter.get(use_monitor=False)
    if start is None:
        return None
    deadline = time.time() + timeout
    while time.time() < deadline:
        now = counter.get(use_monitor=False)
        if now is not None and now >= start + advance:
            return data.get(timeout=timeout, use_monitor=False)
        time.sleep(0.01)
    return None


def connect_depth(width, height):
    """The depth stream, its own intrinsics and its scale, or None.

    None is a capture without depth, not a failure: the tag pose comes
    from the corners either way, and refusing to calibrate because a
    cross-check was unavailable would be the wrong trade.
    """
    data = PV(PREFIX + DEPTH_PLUGIN + ":ArrayData", auto_monitor=False)
    counter = PV(PREFIX + DEPTH_PLUGIN + ":ArrayCounter_RBV", auto_monitor=False)
    if not (data.wait_for_connection(3) and counter.wait_for_connection(3)):
        return None
    try:
        fx = float(caget(PREFIX + "cam1:RSDepthFx_RBV"))
        fy = float(caget(PREFIX + "cam1:RSDepthFy_RBV"))
        ppx = float(caget(PREFIX + "cam1:RSDepthPPx_RBV"))
        ppy = float(caget(PREFIX + "cam1:RSDepthPPy_RBV"))
        unit = float(caget(PREFIX + "cam1:RSDepthUnits_RBV"))
    except (TypeError, ValueError):
        return None
    return {
        "data": data,
        "counter": counter,
        "unit": unit,
        "K": np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]], float),
        "size": (width, height),
    }


def tag_plane(depth, pts):
    """The tag's plane in the depth camera's frame, as `n . X = d` (m).

    A plane, and not a depth per corner, because of what the alignment
    check could and could not settle. The two streams share an optical
    centre: a stereo baseline between them would put the tag 31 px out at
    this working distance, and every landmark measured under 5 px. What
    the residual few pixels are could not be pinned down -- the
    plate-boundary landmark carries 2.8 px of noise of its own, more than
    the candidate mappings differ by. Sliding the sampled patch a few
    pixels along the same flat sheet leaves the plane exactly where it
    was, so the plane is the part of this measurement that does not rest
    on what could not be measured. A per-corner depth would take those
    pixels straight into the corner.

    The range this anchors is the one thing the corners cannot supply:
    fx and the tag's true size trade against each other in the image and
    nothing in a picture separates them.
    """
    width, height = depth["size"]
    raw = depth["data"].get(timeout=5, use_monitor=False)
    if raw is None:
        return None
    z = np.asarray(raw)[: width * height].reshape(height, width).astype(np.float64)
    z *= depth["unit"]

    mask = np.zeros((height, width), np.uint8)
    cv2.fillConvexPoly(mask, pts.astype(np.int32), 1)
    mask = cv2.erode(mask, np.ones((PLANE_ERODE_PX, PLANE_ERODE_PX), np.uint8))
    ys, xs = np.nonzero(mask & (z > 0))
    if len(ys) < MIN_PLANE_PX:
        return None

    K = depth["K"]
    zv = z[ys, xs]
    X = (xs - K[0, 2]) * zv / K[0, 0]
    Y = (ys - K[1, 2]) * zv / K[1, 1]

    def fit(X, Y, zv):
        A = np.c_[X, Y, np.ones(len(zv))]
        coef, *_ = np.linalg.lstsq(A, zv, rcond=None)
        return coef, zv - A @ coef

    # Trimmed twice: the tape at the tag's border and stereo speckle both
    # sit far off the surface, and either one tilts a least-squares plane.
    coef, resid = fit(X, Y, zv)
    for _ in range(2):
        keep = np.abs(resid) < 3.0 * resid.std()
        if keep.all() or keep.sum() < MIN_PLANE_PX:
            break
        X, Y, zv = X[keep], Y[keep], zv[keep]
        coef, resid = fit(X, Y, zv)

    n = np.array([-coef[0], -coef[1], 1.0])
    n /= np.linalg.norm(n)
    d = coef[2] * n[2]
    # Where the plane cuts the ray through the tag's centre. For the log
    # and for comparison with solvePnP; the solver wants n and d, which
    # do not depend on picking a pixel.
    cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
    ray = np.array([(cx - K[0, 2]) / K[0, 0], (cy - K[1, 2]) / K[1, 1], 1.0])
    return {
        "plane": [float(v) for v in n] + [float(d)],
        "plane_range": float(d / (n @ ray) * np.linalg.norm(ray)),
        "plane_rms": float(resid.std()),
        "plane_px": int(len(zv)),
    }


def load_intrinsics():
    def g(name):
        v = caget(PREFIX + "cam1:" + name)
        if v is None:
            raise RuntimeError(f"{PREFIX}cam1:{name} did not connect")
        return float(v)

    fx, fy, cx, cy = g("RSFx_RBV"), g("RSFy_RBV"), g("RSPPx_RBV"), g("RSPPy_RBV")
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], float)
    dist = np.array([g(f"RSDistCoeff{i}_RBV") for i in range(1, 6)], float)
    return K, dist


def main():
    K, dist = load_intrinsics()
    width = int(caget(PREFIX + "image1:ArraySize0_RBV"))
    height = int(caget(PREFIX + "image1:ArraySize1_RBV"))
    data = PV(PREFIX + "image1:ArrayData", auto_monitor=False)
    if not data.wait_for_connection(5):
        raise RuntimeError(f"{PREFIX}image1:ArrayData did not connect")
    counter = PV(PREFIX + "image1:ArrayCounter_RBV", auto_monitor=False)
    counter.wait_for_connection(5)
    depth = connect_depth(width, height)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    # Default is CORNER_REFINE_NONE, which returns contour corners rounded
    # to whole pixels: a static scene then yields bit-identical poses frame
    # after frame, which reads as excellent repeatability and is really
    # quantisation at ~0.74 mm per pixel at this working distance. That
    # error lands directly in T_ee_cam.
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 50
    params.cornerRefinementMinAccuracy = 0.01
    detector = cv2.aruco.ArucoDetector(dictionary, params)

    half = TAG_SIZE_M / 2.0
    obj = np.array(
        [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
        float,
    )

    # The readiness line, not a greeting: it is written only once the
    # intrinsics PVs have answered and the image channel has connected,
    # so the parent that reads it knows detection can be asked for.
    reply(
        ok=True,
        cmd="hello",
        message=(
            f"{PREFIX} {width}x{height} fx={K[0, 0]:.3f} "
            f"tag id={TAG_ID} size={TAG_SIZE_M * 1000:.0f}mm "
            f"k1={dist[0]:+.4f} "
            + (
                f"depth {DEPTH_PLUGIN} fx={depth['K'][0, 0]:.3f}"
                if depth
                else "no depth stream"
            )
        ),
        # The depth stream's own intrinsics, which are not image1's: the
        # plane is fitted in this frame and a re-solve has to deproject
        # in the same one.
        depth_K=[float(v) for v in depth["K"].ravel()] if depth else None,
        # Recorded alongside the samples: the corners are only re-solvable
        # against the model that produced them, and these come from the
        # camera's own PVs, which change with the stream profile.
        K=[float(v) for v in K.ravel()],
        dist=[float(v) for v in dist.ravel()],
        tag_size_m=TAG_SIZE_M,
        image_size=[width, height],
    )

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        cmd = json.loads(line).get("cmd")
        if cmd == "quit":
            return
        if cmd != "detect":
            reply(ok=False, cmd=cmd, reason=f"unknown cmd {cmd}")
            continue

        # Averaging N frames would cut detector noise, but a pose captured
        # while the arm is still settling would average in the motion. One
        # fresh frame, and the caller decides when to ask.
        raw = fresh_frame(data, counter)
        if raw is None:
            reply(ok=False, cmd="detect", reason="no frame newer than the request")
            continue
        img = np.asarray(raw, dtype=np.uint8)[: width * height].reshape(height, width)

        corners, ids, _ = detector.detectMarkers(img)
        if ids is None:
            reply(ok=False, cmd="detect", reason="no tag in frame")
            continue
        found = {int(i): c for c, i in zip(corners, ids.ravel())}
        if TAG_ID not in found:
            reply(ok=False, cmd="detect", reason=f"tag {TAG_ID} not among {sorted(found)}")
            continue

        pts = found[TAG_ID].reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(
            obj, pts, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            reply(ok=False, cmd="detect", reason="solvePnP failed")
            continue
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = float(np.linalg.norm(proj.reshape(4, 2) - pts, axis=1).mean())
        R, _ = cv2.Rodrigues(rvec)
        # Read after the mono frame and from the same driver: the two
        # counters advance in lockstep (image2 constantly one ahead), and
        # the arm is parked while this runs, so a one-frame slip changes
        # nothing. `plane_px` and `plane_rms` are what say whether the
        # fit is worth believing.
        plane = tag_plane(depth, pts) if depth else None
        reply(
            ok=True,
            cmd="detect",
            id=TAG_ID,
            **(plane or {}),
            t=[float(v) for v in tvec.ravel()],
            R=[float(v) for v in R.ravel()],
            reproj=err,
            side_px=float(np.linalg.norm(pts[0] - pts[1])),
            center=[float(pts[:, 0].mean()), float(pts[:, 1].mean())],
            # The corners are the measurement; the pose above is already an
            # interpretation of them under this K and dist. Recalibrating
            # the lens invalidates the pose but not the corners, so passing
            # them up is what makes a better model a re-solve of the file
            # already on disk instead of another hour on the robot.
            # Order matches `obj`, so a solver can pair them without
            # guessing which corner is which.
            corners=[float(v) for v in pts.ravel()],
        )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # the parent only sees stderr, so say why
        print(f"detector: {exc}", file=sys.stderr)
        sys.exit(1)
