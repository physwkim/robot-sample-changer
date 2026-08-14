#!/usr/bin/env python3
"""Solve eye-in-hand calibration from handeye_collect's samples.yaml.

    solve.py samples.yaml [--out T_ee_cam_axxb.yaml]

Kept as a cross-check on solve_joint.py, which is the calibration of
record. This one solves AX = XB per the classical methods, which needs a
tag pose per view and so inherits whatever the lens model got wrong; the
joint solve holds the arm's FK fixed instead and fits the lens with it.

Runs every cv2.calibrateHandEye method and reports the spread between
them, then leave-one-out cross-validation. Both exist because
calibrateHandEye returns an answer for bad input as readily as for good:
a low residual on the poses it was fitted to proves nothing, so the
numbers that decide whether to trust the result are the disagreement
between methods and the error on poses held out of the fit.

The tag's pose in the base frame is never used. AX = XB eliminates it —
the tag only had to stay still during capture.

Reading this as a cross-check, mind what it cannot see. It consumes
cam_t_tag as given, so the detector's assumed tag size rides straight
through: the sheet measures 98.585 mm where the detector assumes 100, so
every cam_t_tag range is long by 1.4 % and that error enters t_X through
the least squares rather than as a clean scaling
(doc/handeye_calibration.md). Nor does it read the depth planes, which
are what anchor the scale at all. The two answers on record sit 16.5 mm
apart, almost all of it along the camera axis — the axis with no metric
handle once the tag size is wrong. None of that shows in the diagnostics
below: method spread and leave-one-out stay small while every method is
wrong together, which is exactly what they cannot detect.
"""

import argparse
import sys

import cv2
import numpy as np
import yaml

METHODS = {
    "Tsai": cv2.CALIB_HAND_EYE_TSAI,
    "Park": cv2.CALIB_HAND_EYE_PARK,
    "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def quat_to_R(x, y, z, w):
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def load(path):
    doc = yaml.safe_load(open(path))
    Rg, tg, Rt, tt, labels = [], [], [], [], []
    for s in doc["samples"]:
        bx, by, bz, qi, qj, qk, qw = s["base_t_ee"]
        cx, cy, cz, ci, cj, ck, cw = s["cam_t_tag"]
        Rg.append(quat_to_R(qi, qj, qk, qw))
        tg.append(np.array([[bx], [by], [bz]]))
        Rt.append(quat_to_R(ci, cj, ck, cw))
        tt.append(np.array([[cx], [cy], [cz]]))
        labels.append(s["label"])
    return Rg, tg, Rt, tt, labels


def solve(Rg, tg, Rt, tt, method):
    R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=method)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t.ravel()
    return T


def rotation_angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def held_out_error(Rg, tg, Rt, tt, method, i):
    """Fit without sample i, then predict where the tag should have been.

    With X = T_ee_cam fixed, base_T_tag = base_T_ee @ X @ cam_T_tag is the
    same constant for every pose. Fit on the rest, evaluate that constant
    from the held-out pose, and compare against the mean of the fitted
    ones -- the residual is the error a real measurement would carry.
    """
    keep = [j for j in range(len(Rg)) if j != i]
    if len(keep) < 3:
        return None
    X = solve([Rg[j] for j in keep], [tg[j] for j in keep],
              [Rt[j] for j in keep], [tt[j] for j in keep], method)

    def base_T_tag(j):
        A = np.eye(4)
        A[:3, :3], A[:3, 3] = Rg[j], tg[j].ravel()
        B = np.eye(4)
        B[:3, :3], B[:3, 3] = Rt[j], tt[j].ravel()
        return A @ X @ B

    fitted = [base_T_tag(j) for j in keep]
    ref = np.mean([T[:3, 3] for T in fitted], axis=0)
    got = base_T_tag(i)[:3, 3]
    return float(np.linalg.norm(got - ref) * 1000.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("samples")
    # Not T_ee_cam.yaml: solve_joint.py owns that name. Both scripts writing
    # it left whichever ran last as the answer, and they disagree by 16.5 mm.
    ap.add_argument("--out", default="T_ee_cam_axxb.yaml")
    args = ap.parse_args()

    Rg, tg, Rt, tt, labels = load(args.samples)
    n = len(Rg)
    print(f"{n} poses: {' '.join(labels)}")
    if n < 5:
        print(f"ERROR: {n} poses is too few; capture more", file=sys.stderr)
        return 1

    results = {}
    for name, flag in METHODS.items():
        try:
            results[name] = solve(Rg, tg, Rt, tt, flag)
        except cv2.error as exc:
            print(f"  {name:11s} FAILED: {exc}")
    if not results:
        print("ERROR: every method failed", file=sys.stderr)
        return 1

    print("\nT_ee_cam per method (translation mm, rotation deg from first):")
    ref = next(iter(results.values()))
    for name, T in results.items():
        t_mm = T[:3, 3] * 1000
        dR = rotation_angle_deg(ref[:3, :3].T @ T[:3, :3])
        dt = np.linalg.norm(T[:3, 3] - ref[:3, 3]) * 1000
        print(
            f"  {name:11s} t=({t_mm[0]:8.2f},{t_mm[1]:8.2f},{t_mm[2]:8.2f})  "
            f"vs first: {dt:5.2f} mm {dR:5.2f} deg"
        )

    spread_t = max(
        np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1000
        for a in results.values()
        for b in results.values()
    )
    spread_R = max(
        rotation_angle_deg(a[:3, :3].T @ b[:3, :3])
        for a in results.values()
        for b in results.values()
    )
    print(f"\nmethod spread: {spread_t:.2f} mm, {spread_R:.2f} deg")

    best = "Daniilidis" if "Daniilidis" in results else next(iter(results))
    errs = [e for i in range(n)
            if (e := held_out_error(Rg, tg, Rt, tt, METHODS[best], i)) is not None]
    if errs:
        print(
            f"leave-one-out ({best}): mean {np.mean(errs):.2f} mm, "
            f"max {np.max(errs):.2f} mm"
        )

    verdict_ok = spread_t < 2.0 and spread_R < 1.0
    print("\nVERDICT:", "usable" if verdict_ok else "NOT usable -- recapture")
    if not verdict_ok:
        print("  methods disagree by more than 2 mm / 1 deg, which means the")
        print("  poses lack rotational diversity or some detections are bad.")

    T = results[best]
    q = cv2.Rodrigues(T[:3, :3])[0].ravel()
    with open(args.out, "w") as fh:
        yaml.safe_dump(
            {
                "T_ee_cam": {
                    "translation_m": [float(v) for v in T[:3, 3]],
                    "rotation_matrix": [[float(v) for v in row] for row in T[:3, :3]],
                    "rotation_vector": [float(v) for v in q],
                },
                "method": best,
                "poses": n,
                "method_spread_mm": float(spread_t),
                "method_spread_deg": float(spread_R),
                "leave_one_out_mean_mm": float(np.mean(errs)) if errs else None,
                "usable": bool(verdict_ok),
            },
            fh,
            sort_keys=False,
        )
    print(f"wrote {args.out}")
    return 0 if verdict_ok else 2


if __name__ == "__main__":
    sys.exit(main())
