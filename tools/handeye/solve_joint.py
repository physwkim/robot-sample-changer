#!/usr/bin/env python3
"""Hand-eye, lens model and tag size from one solve over the whole capture.

`cv2.calibrateCamera` treats every view's tag pose as six free unknowns:
42 views is 261 unknowns against 336 corner coordinates, 75 net, and the
fit comes out undetermined -- two lens models that agree on the captured
corners to 0.08 px disagree 9 % on fx and put the tag 27 mm apart. The
robot already knows how it moved between those views. Holding its FK
fixed collapses the 252 per-view pose unknowns into two poses every view
shares -- the camera on the flange, and the tag in the base frame -- and
the same 336 numbers now answer 22.

Depth enters as the plane's offset from the camera origin, and nothing
else. That offset is invariant both to a rotation between the depth and
colour frames and to the few-pixel mapping between them that the
alignment check could bound (under 5 px, against the 31 px a stereo
baseline would show) but not identify; the plane's normal is invariant to
neither. It is also the only measurement here that is not an angle: in an
image, focal length and the tag's true edge length trade against each
other almost exactly, and every purely photographic estimate of the pair
inherits that. Depth is what separates them.

Usage: solve_joint.py <samples.yaml> [--sigma-depth MM] [--no-depth]
                      [--rational] [--loo] [--out FILE]
"""

import os
import sys

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares

# The corner detector's own repeatability, and what the residuals are
# divided by so the two kinds of observation can be added up.
SIGMA_PX = 0.30
# The plane offset's uncertainty. Not the per-pixel depth noise (2-3 mm):
# that averages down over the ~15000 pixels a plane is fitted to. Chosen
# conservatively, and --sigma-depth exists because the answer's movement
# with it is the honest measure of how much depth is really deciding.
SIGMA_DEPTH_MM = 1.0
# Ceiling for a per-axis t_ee_cam sigma before `gate` calls that axis
# undetermined; `vision.max_correction` in config/sequencer.yaml, the
# largest correction the sequencer will apply. Override with
# --max-sigma-mm.
MAX_SIGMA_MM = 3.0


def rt(rvec, t):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, float))[0]
    T[:3, 3] = t
    return T


def quat_T(v):
    x, y, z, qi, qj, qk, qw = v
    T = np.eye(4)
    n = np.sqrt(qi * qi + qj * qj + qk * qk + qw * qw)
    qi, qj, qk, qw = qi / n, qj / n, qk / n, qw / n
    T[:3, :3] = np.array(
        [
            [1 - 2 * (qj * qj + qk * qk), 2 * (qi * qj - qk * qw), 2 * (qi * qk + qj * qw)],
            [2 * (qi * qj + qk * qw), 1 - 2 * (qi * qi + qk * qk), 2 * (qj * qk - qi * qw)],
            [2 * (qi * qk - qj * qw), 2 * (qj * qk + qi * qw), 1 - 2 * (qi * qi + qj * qj)],
        ]
    )
    T[:3, 3] = [x, y, z]
    return T


def load(path):
    doc = yaml.safe_load(open(path))
    intr = doc["intrinsics"]
    views = []
    for s in doc["samples"]:
        views.append(
            {
                "label": s["label"],
                "base_T_ee": quat_T(s["base_t_ee"]),
                "corners": np.array(s["corners_px"], float).reshape(4, 2),
                "cam_T_tag": quat_T(s["cam_t_tag"]),
                # The offset only. See the module docstring.
                "depth_d": s["depth"]["plane"][3] if "depth" in s else None,
            }
        )
    K = np.array(intr["camera_matrix"], float).reshape(3, 3)
    return views, K, np.array(intr["dist_coeffs"], float), intr["tag_size_m"]


def pack(ee_T_cam, base_T_tag, K, dist, half, ndist=5):
    """Layout: rvec|t camera, rvec|t tag, fx fy cx cy, ndist coeffs, half.

    Everything except the lens coefficients is at a fixed index, and the
    tag half-size is last, so the block can grow from the 5-coefficient
    polynomial model to the 8-coefficient rational one without moving any
    other parameter.
    """
    d = np.zeros(ndist)
    d[: min(ndist, len(dist))] = np.asarray(dist).ravel()[:ndist]
    return np.concatenate(
        [
            cv2.Rodrigues(ee_T_cam[:3, :3])[0].ravel(),
            ee_T_cam[:3, 3],
            cv2.Rodrigues(base_T_tag[:3, :3])[0].ravel(),
            base_T_tag[:3, 3],
            [K[0, 0], K[1, 1], K[0, 2], K[1, 2]],
            d,
            [half],
        ]
    )


def unpack(p):
    ee_T_cam = rt(p[0:3], p[3:6])
    base_T_tag = rt(p[6:9], p[9:12])
    fx, fy, cx, cy = p[12:16]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    return ee_T_cam, base_T_tag, K, p[16:-1], p[-1]


def residuals(p, views, sigma_depth_m, use_depth):
    ee_T_cam, base_T_tag, K, dist, half = unpack(p)
    obj = np.array(
        [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]]
    )
    out = []
    for v in views:
        cam_T_tag = np.linalg.inv(v["base_T_ee"] @ ee_T_cam) @ base_T_tag
        rvec = cv2.Rodrigues(cam_T_tag[:3, :3])[0]
        proj, _ = cv2.projectPoints(obj, rvec, cam_T_tag[:3, 3], K, dist)
        out.append(((proj.reshape(4, 2) - v["corners"]) / SIGMA_PX).ravel())
        if use_depth and v["depth_d"] is not None:
            # Distance from the camera's origin to the tag's plane: the
            # tag's own z axis dotted with where its origin sits.
            n = cam_T_tag[:3, 2]
            d = abs(n @ cam_T_tag[:3, 3])
            out.append([(d - v["depth_d"]) / sigma_depth_m])
    return np.concatenate(out)


def initial(views, K, dist, tag_size, ndist=5):
    """calibrateHandEye for the camera pose, then the tag from any view.

    Rotation views only for the seed: a pure translation makes R_a = I,
    which leaves AX = XB with nothing to say about the translation, and
    the four methods then disagree by millimetres. The joint solve has no
    such trouble with them -- a translation is an ordinary observation to
    a bundle adjustment -- so they rejoin for the solve itself.
    """
    rot = [v for v in views if not v["label"].startswith("f")]
    R, t = cv2.calibrateHandEye(
        [v["base_T_ee"][:3, :3] for v in rot],
        [v["base_T_ee"][:3, 3] for v in rot],
        [v["cam_T_tag"][:3, :3] for v in rot],
        [v["cam_T_tag"][:3, 3] for v in rot],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    ee_T_cam = np.eye(4)
    ee_T_cam[:3, :3], ee_T_cam[:3, 3] = R, t.ravel()
    v = views[0]
    base_T_tag = v["base_T_ee"] @ ee_T_cam @ v["cam_T_tag"]
    return pack(ee_T_cam, base_T_tag, K, dist, tag_size / 2, ndist)


def solve(views, p0, sigma_depth_m, use_depth):
    return least_squares(
        residuals,
        p0,
        args=(views, sigma_depth_m, use_depth),
        x_scale="jac",
        max_nfev=20000,
    )


def reproj_rms(p, views):
    """RMS of the per-corner distance, in pixels."""
    r = (residuals(p, views, 1.0, False) * SIGMA_PX).reshape(-1, 2)
    return float(np.sqrt(np.mean((r**2).sum(axis=1))))


def stderrs(result):
    """One-sigma on each parameter, from the Jacobian at the solution.

    This is the question the earlier probing kept circling: not "do two
    arbitrary fits agree" but "does the data determine this number".

    Two numbers come back per parameter. The first takes SIGMA_PX and
    SIGMA_DEPTH at face value. The second multiplies by sqrt(chi2/dof) --
    what the fit itself says the observations scatter by. They differ
    here: the corners land ~0.9 px from the model where SIGMA_PX claims
    0.3, so something outside the model (the arm's own pose error, a lens
    term this one lacks, the sheet's flatness) is moving them, and the
    face-value sigma understates by exactly that factor. Reported as the
    honest one.
    """
    J = result.jac
    dof = J.shape[0] - J.shape[1]
    chi2_red = 2 * result.cost / dof
    try:
        se = np.sqrt(np.diag(np.linalg.inv(J.T @ J)))
    except np.linalg.LinAlgError:
        se = np.full(J.shape[1], np.nan)
    return se * np.sqrt(chi2_red), chi2_red


def report(name, p, views, use_depth, sigma_depth_m, fit=None):
    ee_T_cam, base_T_tag, K, dist, half = unpack(p)
    t = ee_T_cam[:3, 3] * 1000
    rv = cv2.Rodrigues(ee_T_cam[:3, :3])[0].ravel()
    dres = []
    for v in views:
        if v["depth_d"] is None:
            continue
        cam_T_tag = np.linalg.inv(v["base_T_ee"] @ ee_T_cam) @ base_T_tag
        dres.append(abs(cam_T_tag[:3, 2] @ cam_T_tag[:3, 3]) - v["depth_d"])
    print(f"{name}")
    print(
        f"  T_ee_cam  t = ({t[0]:7.2f}, {t[1]:7.2f}, {t[2]:7.2f}) mm   "
        f"rot = ({np.degrees(rv[0]):6.2f}, {np.degrees(rv[1]):6.2f}, {np.degrees(rv[2]):7.2f}) deg"
    )
    print(
        f"  lens      fx {K[0,0]:7.3f}  fy {K[1,1]:7.3f}  cx {K[0,2]:7.3f}  cy {K[1,2]:7.3f}"
    )
    print(f"            dist [{', '.join(f'{c:+.5f}' for c in dist)}]")
    print(f"  tag side  {half*2000:.3f} mm")
    print(
        f"  residual  corners {reproj_rms(p, views):.4f} px"
        + (
            f"   depth {np.std(dres)*1000:.3f} mm sd, {np.mean(dres)*1000:+.3f} mm mean"
            if dres
            else ""
        )
    )
    if fit is not None:
        se, chi2_red = fit
        print(
            f"  1-sigma   t_ee_cam ({se[3]*1000:.2f}, {se[4]*1000:.2f}, {se[5]*1000:.2f}) mm   "
            f"fx {se[12]:.3f} px   tag side {se[-1]*2000:.3f} mm"
            f"   [chi2/dof {chi2_red:.1f}]"
        )


def main():
    args = sys.argv[1:]
    path = next(a for a in args if not a.startswith("-"))
    use_depth = "--no-depth" not in args
    sigma = SIGMA_DEPTH_MM
    if "--sigma-depth" in args:
        sigma = float(args[args.index("--sigma-depth") + 1])
    # The 5-coefficient polynomial model runs out at the frame corners:
    # fitted on centred views it mispredicts a corner view by 3.2 px while
    # fitting its own to 0.6. --rational adds k4..k6, the denominator that
    # lets the radial curve bend back at large r instead of running away.
    ndist = 8 if "--rational" in args else 5
    views, K, dist, tag_size = load(path)
    have_depth = sum(v["depth_d"] is not None for v in views)
    print(
        f"{len(views)} views, {len(views)*8} corner coordinates, "
        f"{have_depth} depth planes, {17+ndist} unknowns\n"
    )
    p0 = initial(views, K, dist, tag_size, ndist)
    report("seed (calibrateHandEye + the camera's own lens model)", p0, views, False, sigma)

    print()
    runs = [("corners only", False)]
    if use_depth and have_depth:
        runs.append((f"corners + depth (sigma {sigma:.1f} mm)", True))
    best = None
    for name, ud in runs:
        r = solve(views, p0, sigma / 1000.0, ud)
        fit = stderrs(r)
        report(name, r.x, views, ud, sigma / 1000.0, fit)
        print()
        best = (r.x, fit, ud)

    if use_depth and have_depth:
        print("how much is depth actually deciding?")
        print(f"  {'sigma_depth':>12s}{'fx':>10s}{'tag mm':>10s}{'t_z mm':>10s}{'corners px':>12s}")
        for sg in (0.2, 0.5, 1.0, 2.0, 5.0, 1e6):
            r = solve(views, p0, sg / 1000.0, True)
            _, _, Kf, _, half = unpack(r.x)
            lbl = "off" if sg > 1e5 else f"{sg:.1f} mm"
            print(
                f"  {lbl:>12s}{Kf[0,0]:10.3f}{half*2000:10.3f}"
                f"{r.x[5]*1000:10.2f}{reproj_rms(r.x, views):12.4f}"
            )

    loo = None
    if "--loo" in args:
        print("\nleave-one-out: refit without each view, then predict its corners")
        errs = []
        for i in range(len(views)):
            keep = views[:i] + views[i + 1 :]
            r = solve(keep, p0, sigma / 1000.0, use_depth)
            errs.append(reproj_rms(r.x, [views[i]]))
        errs = np.array(errs)
        loo = float(errs.mean())
        print(
            f"  held-out corner error: mean {errs.mean():.4f} px, "
            f"max {errs.max():.4f} px ({views[int(errs.argmax())]['label']})"
        )

    out = args[args.index("--out") + 1] if "--out" in args else "T_ee_cam.yaml"
    write_result(out, best, views, loo, path)
    print(f"\nwrote {out}")
    return gate(best, args)


def gate(best, args):
    """Per-axis verdict on `t_ee_cam`, as an exit code.

    A low corner residual does not mean the camera's position was
    determined: AX = XB says nothing about `t_X` along an axis every
    rotation shares, and pure translations say nothing about it at all --
    the sweep views on their own put the camera at +-3000 mm while
    fitting their own corners to 0.63 px. That failure is per-axis and
    invisible in any whole-fit number, so it needs its own check.

    The limit is the largest correction the sequencer will ever apply
    (`vision.max_correction`): an axis the fit cannot pin down to better
    than that cannot inform a correction along it. Absolute rather than a
    worst/best ratio, because an anisotropic fit that is small on every
    axis is still usable and a uniformly bad one still is not. A NaN
    sigma -- a singular J'J, which is degeneracy in its exact form --
    fails the comparison and so fails the gate.
    """
    limit = MAX_SIGMA_MM
    if "--max-sigma-mm" in args:
        limit = float(args[args.index("--max-sigma-mm") + 1])
    se, _ = best[1]
    sigmas = [float(v * 1000) for v in se[3:6]]
    print(f"\nper-axis gate on t_ee_cam (limit {limit:.2f} mm):")
    ok = True
    for axis, s in zip("xyz", sigmas):
        passed = s < limit  # NaN compares False, which is the answer here
        ok &= passed
        note = "" if passed else "  <- not determined; add rotations about another axis"
        print(f"  t_{axis}  sigma {s:8.3f} mm   {'ok' if passed else 'DEGENERATE'}{note}")
    if not ok:
        print("VERDICT: NOT usable — one or more axes are undetermined")
    return 0 if ok else 2


def write_result(path, best, views, loo, path_of_samples):
    """The camera pose, and everything the solve had to pin down to get it.

    The lens model and the tag size are written alongside because they
    came out of the same fit: using this T_ee_cam with the camera's
    factory intrinsics, or with 100 mm for the tag, would reintroduce
    exactly the trade the joint solve resolved.

    tag_size_m is the recovered corner-to-corner chord, and it is the
    right number to use with these corners, but it is not evidence that
    the sheet was printed at that size -- paper does not stretch, so a
    sheet that bulges between its corners measures short through the air.
    Which of the two it was is recorded as a note rather than decided
    here, because the measurement cannot separate them.
    """
    p, (se, chi2_red) = best[0], best[1]
    ee_T_cam, base_T_tag, K, dist, half = unpack(p)
    nominal = 0.100
    yaml.safe_dump(
        {
            "T_ee_cam": {
                "translation_m": [float(v) for v in ee_T_cam[:3, 3]],
                "rotation_matrix": [[float(v) for v in r] for r in ee_T_cam[:3, :3]],
                "rotation_vector": [float(v) for v in cv2.Rodrigues(ee_T_cam[:3, :3])[0].ravel()],
                "translation_sigma_mm": [float(v * 1000) for v in se[3:6]],
            },
            "camera_matrix": [float(v) for v in K.ravel()],
            "dist_coeffs": [float(v) for v in dist],
            "fx_sigma_px": float(se[12]),
            "tag_size_m": float(half * 2),
            "tag_size_sigma_mm": float(se[-1] * 2000),
            "tag_size_note": (
                f"recovered chord, {100*(half*2/nominal - 1):+.2f} % against the "
                f"{nominal*1000:.0f} mm the sheet was drawn at. A scaled print and a "
                "sheet that is not flat both read short here and this fit cannot "
                "tell them apart; measure the sheet's own 100 mm rule to decide."
            ),
            "base_T_tag": {
                "translation_m": [float(v) for v in base_T_tag[:3, 3]],
                "rotation_vector": [float(v) for v in cv2.Rodrigues(base_T_tag[:3, :3])[0].ravel()],
            },
            "samples": os.path.abspath(path_of_samples),
            "views": len(views),
            "used_depth": bool(best[2]),
            "corner_rms_px": float(reproj_rms(p, views)),
            "chi2_per_dof": float(chi2_red),
            "leave_one_out_mean_px": loo,
        },
        open(path, "w"),
        sort_keys=False,
    )


if __name__ == "__main__":
    sys.exit(main())
