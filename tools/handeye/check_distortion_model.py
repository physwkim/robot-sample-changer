#!/usr/bin/env python3
"""Which convention are RS405's RSDistCoeff_RBV in?

The IOC reports RSDistModel_RBV = BrownConradyInverse while cv2.solvePnP
assumes the forward Brown-Conrady model. Rather than argue from the name,
call librealsense2.so itself: project the same 3D points with
rs2_project_point_to_pixel under each rs2_distortion model, and compare
against cv2.projectPoints with the identical coefficients. Whichever
librealsense model matches OpenCV tells us what OpenCV is being handed.
"""

import ctypes
import numpy as np
import cv2

LIB = ctypes.CDLL("librealsense2.so")

RS2_NONE, RS2_MODIFIED_BC, RS2_INVERSE_BC, RS2_FTHETA, RS2_BC, RS2_KB4 = range(6)
NAMES = {
    RS2_NONE: "NONE",
    RS2_MODIFIED_BC: "MODIFIED_BROWN_CONRADY",
    RS2_INVERSE_BC: "INVERSE_BROWN_CONRADY  <- IOC가 보고하는 모델",
    RS2_BC: "BROWN_CONRADY",
}


class RS2Intrinsics(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("ppx", ctypes.c_float),
        ("ppy", ctypes.c_float),
        ("fx", ctypes.c_float),
        ("fy", ctypes.c_float),
        ("model", ctypes.c_int),
        ("coeffs", ctypes.c_float * 5),
    ]


LIB.rs2_project_point_to_pixel.argtypes = [
    ctypes.POINTER(ctypes.c_float * 2),
    ctypes.POINTER(RS2Intrinsics),
    ctypes.POINTER(ctypes.c_float * 3),
]
LIB.rs2_project_point_to_pixel.restype = None

# Measured on this unit (RS405:cam1:RS*_RBV), colour stream.
W, H = 640, 480
FX, FY, PPX, PPY = 393.284, 392.673, 321.745, 246.323
COEFFS = [-0.0503777, 0.0602241, 0.00047613, 0.00129567, -0.0205373]

K = np.array([[FX, 0, PPX], [0, FY, PPY], [0, 0, 1]], float)
DIST = np.array(COEFFS, float)


def rs_project(model, pts):
    intr = RS2Intrinsics(W, H, PPX, PPY, FX, FY, model, (ctypes.c_float * 5)(*COEFFS))
    out = []
    for p in pts:
        px = (ctypes.c_float * 2)()
        pt = (ctypes.c_float * 3)(*[float(v) for v in p])
        LIB.rs2_project_point_to_pixel(
            ctypes.byref(px), ctypes.byref(intr), ctypes.byref(pt)
        )
        out.append([px[0], px[1]])
    return np.array(out)


def cv_project(pts):
    proj, _ = cv2.projectPoints(
        np.array(pts, float).reshape(-1, 1, 3),
        np.zeros(3),
        np.zeros(3),
        K,
        DIST,
    )
    return proj.reshape(-1, 2)


def pixel_to_point(u, v, z):
    """An ideal (pinhole) ray through pixel (u, v) at depth z."""
    return [(u - PPX) / FX * z, (v - PPY) / FY * z, z]


# Points spread from the centre out to the corners, at the tag's measured
# working distance. The models only separate away from the centre, which
# is exactly why one centred tag could not decide this.
Z = 0.290
GRID = []
LABELS = []
for name, (u, v) in [
    ("centre", (PPX, PPY)),
    ("edge  x", (W - 1, PPY)),
    ("edge -x", (0, PPY)),
    ("edge  y", (PPX, H - 1)),
    ("corner+", (W - 1, H - 1)),
    ("corner-", (0, 0)),
]:
    GRID.append(pixel_to_point(u, v, Z))
    LABELS.append(name)

cvp = cv_project(GRID)
print(f"intrinsics fx={FX} fy={FY} cx={PPX} cy={PPY}, z={Z * 1000:.0f} mm")
print(f"coeffs     {COEFFS}\n")

for model in (RS2_NONE, RS2_MODIFIED_BC, RS2_INVERSE_BC, RS2_BC):
    rsp = rs_project(model, GRID)
    d = np.linalg.norm(rsp - cvp, axis=1)
    print(f"{NAMES[model]}")
    for lab, a, b, e in zip(LABELS, rsp, cvp, d):
        print(
            f"   {lab}: rs=({a[0]:8.3f},{a[1]:8.3f})  "
            f"cv=({b[0]:8.3f},{b[1]:8.3f})  diff {e:8.4f} px"
        )
    print(f"   -> max diff vs cv2.projectPoints: {d.max():.4f} px\n")

# What it costs to get it wrong: the same undistorted ray, projected under
# the two candidate conventions, differs by this much in pixels -- and a
# tag corner misplaced by that lands directly in solvePnP's answer.
none_px = rs_project(RS2_NONE, GRID)
inv_px = rs_project(RS2_INVERSE_BC, GRID)
gap = np.linalg.norm(none_px - inv_px, axis=1)
print("왜곡 무시 vs 적용 (같은 광선, 픽셀 차):")
for lab, e in zip(LABELS, gap):
    print(f"   {lab}: {e:7.3f} px  ({e * 0.737:6.3f} mm @ 290 mm)")
