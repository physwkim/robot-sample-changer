#!/usr/bin/env python3
"""Re-approach repeatability at the observation pose — what `min_correction` has to filter.

`doc/vision_correction_plan.md` §12.3 measured two things at holder 3's
standby. The estimator's own noise floor, with the arm parked: σ (0.0050,
0.0036) px = (0.0017, 0.0012) mm over 20 frames. And, once, the shift after
the arm left and came back: **0.070 mm** (dx 0.013, dy 0.069, ECC 0.980) —
forty times the noise floor, and larger than the shipped deadband
`vision.min_correction: 0.05` mm. That single number is why the deadband is
currently wrong: it filters the sensor, when what needs filtering is the arm.

n = 1 is not a σ. This tool collects the rest of them.

    reapproach.py ref                  reference frame at the pose the arm is in now
    reapproach.py sample               one re-approach sample against that reference
    reapproach.py watch --step 1       sample automatically on every arrival at a step
    reapproach.py stats                σ, peak-to-peak, and what they say about the deadband
    reapproach.py stationary -n 20     the estimator's noise floor, arm parked (the §12.3 control)

The estimator is §12.3's: a depth gate isolates the stack from the room, and
`cv2.findTransformECC(MOTION_TRANSLATION)` recovers the shift of the mono
frame inside that window. Nothing here moves the arm — the sequencer daemon
is the only writer that may (see CLAUDE.md, "읽기는 다중, 쓰기는 하나"), so
`watch` reads `Robot:CurrentStep` and takes its pictures when the daemon
parks there.

`--step 1` is the pick@rack observation pose: `step_epilogue` publishes
`CurrentStep` after the move completes, so the transition into 1 is the arm
standing still at `w.standby`. Give the daemon a reason to dwell there —
`Robot:PauseStep = 1` holds until PauseStep changes — or the frame grab
races the departure to step 2.
"""

import argparse
import csv
import os
import sys
import time

os.environ.setdefault("EPICS_CA_MAX_ARRAY_BYTES", "20000000")

import cv2
import numpy as np
from epics import PV

PREFIX = os.environ.get("VISION_CAM_PREFIX", "RS405:")
WIDTH, HEIGHT = 640, 480

# Holder 3's window in the standby view (h3_overlay.png) and the depth band
# that isolates the stack: everything else in the frame sits beyond 675 mm.
DEFAULT_WINDOW = (285, 380, 260, 360)  # y0, y1, x0, x1
DEFAULT_GATE = (0.050, 0.250)  # m
# 136.0 mm optical working distance at standby / fx 393.284.
MM_PER_PX = 0.346


def fresh(data, counter, timeout=5.0, advance=2):
    """A frame the camera produced after this call, not one from the cache.

    `advance=2` because the areaDetector plugin counter moves in twos here;
    waiting for a strict advance is what caught the pyepics cache bug that
    once made this measurement look perfectly repeatable.
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
    def __init__(self):
        self.mono = PV(PREFIX + "image1:ArrayData", auto_monitor=False)
        self.mono_c = PV(PREFIX + "image1:ArrayCounter_RBV", auto_monitor=False)
        self.depth = PV(PREFIX + "image2:ArrayData", auto_monitor=False)
        self.depth_c = PV(PREFIX + "image2:ArrayCounter_RBV", auto_monitor=False)
        for pv in (self.mono, self.mono_c, self.depth, self.depth_c):
            if not pv.wait_for_connection(5):
                raise SystemExit(f"{pv.pvname} did not connect")

    def grab(self):
        """One mono frame and its depth map, or `None` if the stream stalled."""
        m = fresh(self.mono, self.mono_c)
        d = fresh(self.depth, self.depth_c)
        if m is None or d is None:
            return None
        n = WIDTH * HEIGHT
        img = np.asarray(m, np.uint8)[:n].reshape(HEIGHT, WIDTH).astype(np.float32)
        z = np.asarray(d, np.float64)[:n].reshape(HEIGHT, WIDTH) * 1e-4
        return img, z


def build_roi(z, window, gate):
    """The depth gate, closed and dilated into a mask over the window."""
    y0, y1, x0, x1 = window
    near, far = gate
    mask = ((z > near) & (z < far)).astype(np.uint8)
    k9, k5 = np.ones((9, 9), np.uint8), np.ones((5, 5), np.uint8)
    grown = cv2.dilate(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k9), k5)
    return grown[y0:y1, x0:x1], int(mask[y0:y1, x0:x1].sum())


def ecc_shift(ref, cur, roi):
    """Translation of `cur` relative to `ref`, in pixels, or None if ECC failed."""
    warp = np.eye(2, 3, dtype=np.float32)
    try:
        cc, warp = cv2.findTransformECC(
            ref,
            cur,
            warp,
            cv2.MOTION_TRANSLATION,
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 200, 1e-7),
            roi,
            5,
        )
    except cv2.error as exc:
        print(f"  ECC did not converge: {exc}", file=sys.stderr)
        return None
    return float(warp[0, 2]), float(warp[1, 2]), float(cc)


def ref_path(out):
    return os.path.join(out, "reference.npz")


def csv_path(out):
    return os.path.join(out, "samples.csv")


def cmd_ref(args):
    cam = Camera()
    frame = cam.grab()
    if frame is None:
        raise SystemExit("no fresh frame; is the camera IOC streaming?")
    img, z = frame
    y0, y1, x0, x1 = args.window
    roi, area = build_roi(z, args.window, args.gate)
    if area == 0:
        raise SystemExit(
            f"the depth gate {args.gate} is empty in window {args.window}: "
            "the arm is not at the observation pose, or the window is wrong"
        )
    os.makedirs(args.out, exist_ok=True)
    np.savez(
        ref_path(args.out),
        ref=img[y0:y1, x0:x1],
        roi=roi,
        window=np.array(args.window),
        gate=np.array(args.gate),
    )
    print(f"reference written to {ref_path(args.out)}")
    print(f"depth-gate area in the window: {area} px")


def load_ref(out):
    if not os.path.exists(ref_path(out)):
        raise SystemExit(f"no reference at {ref_path(out)} — run `ref` first")
    z = np.load(ref_path(out))
    return z["ref"], z["roi"], tuple(z["window"]), tuple(z["gate"])


def append_sample(out, row):
    new = not os.path.exists(csv_path(out))
    with open(csv_path(out), "a", newline="") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(
                ["n", "unix_time", "dx_px", "dy_px", "dx_mm", "dy_mm", "d_mm", "ecc", "gate_px"]
            )
        w.writerow(row)


def count_samples(out):
    if not os.path.exists(csv_path(out)):
        return 0
    with open(csv_path(out)) as fh:
        return max(0, sum(1 for _ in fh) - 1)


def take_sample(cam, out, ref, roi, window, gate, stamp):
    y0, y1, x0, x1 = window
    frame = cam.grab()
    if frame is None:
        print("  no fresh frame", file=sys.stderr)
        return None
    img, z = frame
    _, area = build_roi(z, window, gate)
    shift = ecc_shift(ref, img[y0:y1, x0:x1], roi)
    if shift is None:
        return None
    dx, dy, cc = shift
    dx_mm, dy_mm = dx * MM_PER_PX, dy * MM_PER_PX
    d_mm = float(np.hypot(dx_mm, dy_mm))
    n = count_samples(out) + 1
    append_sample(
        out,
        [
            n,
            f"{stamp:.3f}",
            f"{dx:+.4f}",
            f"{dy:+.4f}",
            f"{dx_mm:+.4f}",
            f"{dy_mm:+.4f}",
            f"{d_mm:.4f}",
            f"{cc:.4f}",
            area,
        ],
    )
    print(
        f"  sample {n:2d}: dx {dx:+7.4f} dy {dy:+7.4f} px "
        f"= {d_mm:.4f} mm, ECC {cc:.4f}, gate {area} px"
    )
    return d_mm


def cmd_sample(args):
    cam = Camera()
    ref, roi, window, gate = load_ref(args.out)
    take_sample(cam, args.out, ref, roi, window, gate, time.time())


def cmd_watch(args):
    """Sample once per arrival at `--step`, until `--n` samples or Ctrl-C."""
    cam = Camera()
    ref, roi, window, gate = load_ref(args.out)
    step = PV("Robot:CurrentStep", auto_monitor=False)
    if not step.wait_for_connection(5):
        raise SystemExit("Robot:CurrentStep did not connect")

    print(
        f"watching Robot:CurrentStep for arrivals at step {args.step}; "
        f"{args.n} samples wanted, Ctrl-C to stop"
    )
    print("  (the daemon must dwell there — set Robot:PauseStep to the step)")
    taken = 0
    previous = step.get(use_monitor=False)
    while taken < args.n:
        time.sleep(0.05)
        now = step.get(use_monitor=False)
        if now is None or now == previous:
            continue
        previous = now
        if int(now) != args.step:
            continue
        print(f"arrival at step {args.step}:")
        if take_sample(cam, args.out, ref, roi, window, gate, time.time()) is not None:
            taken += 1
    print(f"\n{taken} samples collected; run `stats` next")


def cmd_stationary(args):
    """The §12.3 control: the arm does not move, so this is the estimator alone."""
    cam = Camera()
    frames, gates = [], []
    for i in range(args.n):
        frame = cam.grab()
        if frame is None:
            print(f"frame {i}: no fresh frame", file=sys.stderr)
            continue
        img, z = frame
        frames.append(img)
        gates.append(z)
    if len(frames) < 2:
        raise SystemExit("need at least two frames")
    print(f"captured {len(frames)} frames")

    y0, y1, x0, x1 = args.window
    roi, area = build_roi(gates[0], args.window, args.gate)
    areas = [
        int(((z > args.gate[0]) & (z < args.gate[1]))[y0:y1, x0:x1].sum()) for z in gates
    ]
    print(
        f"depth-gate area in the window: mean {np.mean(areas):.0f} px, "
        f"sigma {np.std(areas):.0f} px ({100 * np.std(areas) / max(np.mean(areas), 1):.1f}%)"
    )
    if area == 0:
        raise SystemExit(
            f"the depth gate {args.gate} is empty in window {args.window}: "
            "the arm is not at the observation pose, or the window is wrong"
        )

    ref = frames[0][y0:y1, x0:x1]
    shifts = []
    for f in frames[1:]:
        s = ecc_shift(ref, f[y0:y1, x0:x1], roi)
        if s is not None:
            shifts.append(s[:2])
    report("estimator noise floor, arm stationary", np.array(shifts))


def cmd_stats(args):
    if not os.path.exists(csv_path(args.out)):
        raise SystemExit(f"no samples at {csv_path(args.out)}")
    rows = list(csv.DictReader(open(csv_path(args.out))))
    if not rows:
        raise SystemExit("samples.csv is empty")
    s = np.array([[float(r["dx_px"]), float(r["dy_px"])] for r in rows])
    report("re-approach, arm left and returned", s)

    d = np.array([float(r["d_mm"]) for r in rows])
    print(f"\n|d| mean {d.mean():.4f} mm, max {d.max():.4f} mm, n = {len(d)}")
    print(f"ECC correlation min {min(float(r['ecc']) for r in rows):.4f}")
    print("\nagainst the shipped thresholds:")
    for name, value in (("min_correction", 0.05), ("max_correction", 3.0)):
        over = int((d > value).sum())
        print(f"  {name} = {value} mm: {over}/{len(d)} samples exceed it")
    print(
        "\nA deadband below the re-approach spread makes the arm chase its own\n"
        "repeatability every cycle. Set min_correction above it — 3σ of |d| is\n"
        f"{3 * d.std():.4f} mm here."
    )


def report(title, s):
    if s.size == 0:
        raise SystemExit("no usable pairs")
    print(f"\nECC translation, {len(s)} pairs — {title}:")
    print(
        f"mean  ({s[:, 0].mean():+.4f}, {s[:, 1].mean():+.4f}) px"
        f"\nsigma ({s[:, 0].std():.4f}, {s[:, 1].std():.4f}) px"
        f"  = ({s[:, 0].std() * MM_PER_PX:.4f}, {s[:, 1].std() * MM_PER_PX:.4f}) mm"
        f"\npeak-to-peak ({np.ptp(s[:, 0]):.4f}, {np.ptp(s[:, 1]):.4f}) px"
        f"  = ({np.ptp(s[:, 0]) * MM_PER_PX:.4f}, {np.ptp(s[:, 1]) * MM_PER_PX:.4f}) mm"
    )


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", default="reapproach", help="output directory")
    p.add_argument(
        "--window",
        type=int,
        nargs=4,
        metavar=("Y0", "Y1", "X0", "X1"),
        default=DEFAULT_WINDOW,
        help="ROI window in the frame",
    )
    p.add_argument(
        "--gate",
        type=float,
        nargs=2,
        metavar=("NEAR", "FAR"),
        default=DEFAULT_GATE,
        help="depth gate in metres",
    )
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ref").set_defaults(func=cmd_ref)
    sub.add_parser("sample").set_defaults(func=cmd_sample)
    w = sub.add_parser("watch")
    w.add_argument("--step", type=int, default=1, help="CurrentStep value to sample at")
    w.add_argument("-n", type=int, default=20, help="samples wanted")
    w.set_defaults(func=cmd_watch)
    st = sub.add_parser("stationary")
    st.add_argument("-n", type=int, default=20, help="frames")
    st.set_defaults(func=cmd_stationary)
    sub.add_parser("stats").set_defaults(func=cmd_stats)
    args = p.parse_args()
    args.window = tuple(args.window)
    args.gate = tuple(args.gate)
    args.func(args)


if __name__ == "__main__":
    main()
