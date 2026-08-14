"""The depth-gated ECC estimator, shared by the measurement tool and the node.

`doc/vision_correction_plan.md` §12 settled how this system measures: it does
not recognise the holder. Depth splits the stack from the room (§12.1 — useless
as metrology on this metal, decisive as segmentation across a 525 mm gap), and
`cv2.findTransformECC(MOTION_TRANSLATION)` recovers how far the scene inside
that window has slid since a reference frame. Specular metal breaks model-based
detection; it does not break the correlation of two frames of the same scene.

`reapproach.py` measured this estimator's floor at sigma (0.0010, 0.0009) mm
over 20 parked frames, against an arm that re-approaches to (0.022, 0.007) mm
(§14). Both numbers came from the code in this file, which is why the node and
the instrument import it rather than each keeping a copy — a node measuring
with a different estimator than the one the deadband was derived from would
make §14 a statement about nothing.
"""

import time

import cv2
import numpy as np
from epics import PV

# Everything the D405 shows at a standby pose is either the holder stack
# (~75 mm) or the room (675 mm and beyond). The gate keeps the near side.
DEFAULT_GATE = (0.050, 0.250)  # m
# The IOC reports depth in units of 0.0001 m (D405-specific).
DEPTH_UNIT_M = 1e-4


def fresh(data, counter, timeout=5.0, advance=2):
    """A frame the camera produced after this call, not one from the cache.

    `advance=2` because the plugin's counter moves in twos here. Waiting for a
    strict advance is not defensive padding: in this repo "perfect
    repeatability" has twice been the mask on a bug (a pyepics cache, then
    `CORNER_REFINE_NONE`), and a stale frame reads exactly like a still arm.
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


class Camera:
    """The mono frame and the depth map behind `image1:` / `image2:`.

    The two are not pixel-identical — `tools/handeye/detector.py` says so of
    the same pair, and the depth stream carries its own intrinsics. Nothing
    here needs them to be: depth is used only to grow a mask, and the mask is
    closed 9x9 and dilated 5x5 afterwards, which is far wider than the
    disagreement. Depth is never used as a coordinate.
    """

    def __init__(self, prefix="RS405:", width=640, height=480, connect_timeout=5.0):
        self.width, self.height = width, height
        self.mono = PV(prefix + "image1:ArrayData", auto_monitor=False)
        self.mono_c = PV(prefix + "image1:ArrayCounter_RBV", auto_monitor=False)
        self.depth = PV(prefix + "image2:ArrayData", auto_monitor=False)
        self.depth_c = PV(prefix + "image2:ArrayCounter_RBV", auto_monitor=False)
        for pv in (self.mono, self.mono_c, self.depth, self.depth_c):
            if not pv.wait_for_connection(connect_timeout):
                raise SystemExit(f"{pv.pvname} did not connect")

    def grab(self):
        """One mono frame and its depth map in metres, or `None` if stalled."""
        m = fresh(self.mono, self.mono_c)
        d = fresh(self.depth, self.depth_c)
        if m is None or d is None:
            return None
        n = self.width * self.height
        img = np.asarray(m, np.uint8)[:n].reshape(self.height, self.width).astype(np.float32)
        z = np.asarray(d, np.float64)[:n].reshape(self.height, self.width) * DEPTH_UNIT_M
        return img, z


def gate_mask(z, gate=DEFAULT_GATE):
    """`(raw, grown)`: the depth gate, and the same gate closed 9x9 then dilated 5x5."""
    raw = ((z > gate[0]) & (z < gate[1])).astype(np.uint8)
    k9, k5 = np.ones((9, 9), np.uint8), np.ones((5, 5), np.uint8)
    return raw, cv2.dilate(cv2.morphologyEx(raw, cv2.MORPH_CLOSE, k9), k5)


def build_roi(z, window, gate=DEFAULT_GATE):
    """`(roi, area)` for `window`: the dilated gate, and the raw gated pixels.

    `area` is the honest count — pre-dilation — because it is what says whether
    the target is in view at all. An empty gate means the arm is not at the
    observation pose, or the window is wrong; both are answers, not noise.
    """
    y0, y1, x0, x1 = window
    raw, grown = gate_mask(z, gate)
    return grown[y0:y1, x0:x1], int(raw[y0:y1, x0:x1].sum())


def gate_depth_m(z, window, gate=DEFAULT_GATE):
    """Median range to the gated pixels, in metres, or None if the gate is empty.

    This is the Z that converts a pixel shift to millimetres. Depth on this
    metal is biased 1.83 mm and spread 8.48 mm (§12.1), which would be fatal if
    it were the measurement — here it is only the scale factor on one. At the
    ~136 mm standby range, 8 mm of error is 6 %, so it costs 0.18 mm on the
    3 mm correction the sequence will ever accept.
    """
    y0, y1, x0, x1 = window
    patch = z[y0:y1, x0:x1]
    inside = patch[(patch > gate[0]) & (patch < gate[1])]
    return float(np.median(inside)) if inside.size else None


def ecc_shift(ref, cur, roi, iterations=200, eps=1e-7, gauss=5):
    """Translation of `cur` relative to `ref` in pixels, with the correlation.

    `findTransformECC(template, input, W)` solves `template(x) ~= input(W(x))`,
    so for a pure translation `W(x) = x + t` the returned `t` is where a
    feature of the reference now sits in the current frame. Returns
    `(dx_px, dy_px, cc)` or `None` if ECC did not converge.
    """
    warp = np.eye(2, 3, dtype=np.float32)
    try:
        cc, warp = cv2.findTransformECC(
            ref,
            cur,
            warp,
            cv2.MOTION_TRANSLATION,
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, iterations, eps),
            roi,
            gauss,
        )
    except cv2.error:
        return None
    return float(warp[0, 2]), float(warp[1, 2]), float(cc)
