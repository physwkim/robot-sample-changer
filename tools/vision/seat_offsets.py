"""How far each holder's seat sits from its neighbours', in the tool frame.

`compute_run_waypoints` places holder N by stepping `holder_offset` along the
tool y axis and then adding `holder_multi_x_offsets[N-1]` /
`holder_multi_z_offsets` — a table hand-taught one holder at a time. Holder 1
has its own entry like every other seat, and the rack-wide part lives in
`holder_rack_*_offset`; what this measures is the residual still left after
the table has been applied.

**Only like against like.** A rack stop differs from another rack stop in two
ways that have nothing to do with where the seat is: whether the seat holds a
puck, and whether the fingers do. Step 1 is a loaded seat and empty fingers,
step 18 an empty seat and loaded fingers, and comparing across that boundary
costs most of the correlation — measured at ECC 0.79..0.81 against 0.977..0.985
within a step. So the frames are grouped by step and never mixed, which also
buys a second, independent estimate of the same offset: step 18 measures the
machined seat, step 1 measures the puck sitting in it. They should agree, and
where they do not the difference is how the puck seats in that cup rather than
where the cup is.

Frames are correlated over one fixed window rather than each holder's own, so
the recovered shift is in the same pixels for all of them.

    seat_offsets.py

Reads `vision_refs/` and every `--dump` frame in `vision_survey/`, so repeats
accumulate without any change here.
"""

import glob
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from estimator import build_roi, ecc_shift, gate_depth_m  # noqa: E402
from vision_node import REPO, load_transform  # noqa: E402

REFS = os.path.join(REPO, "vision_refs")
SURVEY = os.path.join(REPO, "vision_survey")
RACK_STEPS = (1, 18)
SEAT = {1: "loaded seat, empty fingers", 18: "empty seat, loaded fingers"}
GATE = (0.090, 0.250)


def rack_frames():
    """`{step: {holder: [(label, frame, depth)]}}` for every rack stop on disk."""
    out = {s: defaultdict(list) for s in RACK_STEPS}
    for path in sorted(glob.glob(os.path.join(REFS, "k*_s*_h*.npz"))):
        name = os.path.basename(path)
        step = int(name.split("_s")[1].split("_h")[0])
        holder = int(name.split("_h")[1].split(".")[0])
        d = np.load(path)
        out[step][holder].append((name, d["frame"], d["depth_map"]))
    for path in sorted(glob.glob(os.path.join(SURVEY, "h*_s*.npz"))):
        d = np.load(path)
        step = int(d["step"])
        if step in RACK_STEPS:
            out[step][int(d["holder"])].append(
                (os.path.basename(path), d["frame"], d["depth_map"])
            )
    return out


def common_target():
    """The intersection of the taught target windows — the seat and little else."""
    boxes = [
        tuple(int(v) for v in np.load(p)["target"])
        for p in glob.glob(os.path.join(REFS, "k3_s18_h*.npz"))
    ]
    return (max(b[0] for b in boxes), min(b[1] for b in boxes),
            max(b[2] for b in boxes), min(b[3] for b in boxes))


def offset(ref_frame, frame, z, target, rot, fx, fy):
    """`(d_mm, cc)`: where the reference's seat sits in this frame, in the tool frame."""
    ty0, ty1, tx0, tx1 = target
    troi, _ = build_roi(z, target, GATE)
    got = ecc_shift(
        ref_frame[ty0:ty1, tx0:tx1].astype(np.float32),
        frame[ty0:ty1, tx0:tx1].astype(np.float32),
        troi,
    )
    if got is None:
        return None, -1.0
    du, dv, cc = got
    depth = gate_depth_m(z, target, GATE)
    d_cam = np.array([du * depth / fx, dv * depth / fy, 0.0])
    return (rot @ d_cam) * 1000.0, cc


def report(step, holders, target, rot, fx, fy):
    """One step's table, anchored on its lowest holder."""
    usable = {h: f for h, f in holders.items() if h != 1}
    if len(usable) < 2:
        return
    anchor_h = min(usable)
    anchor_label, anchor, _ = usable[anchor_h][0]
    print(f"\n== step {step} ({SEAT[step]}), anchored on holder {anchor_h} ==")
    print(f"   anchor frame {anchor_label}")
    means, sigmas, ccs = {}, {}, {}
    for h in sorted(usable):
        got = []
        for label, frame, z in usable[h]:
            if label == anchor_label:
                continue
            d, cc = offset(anchor, frame, z.astype(float), target, rot, fx, fy)
            if d is not None:
                got.append((d, cc))
        if not got:
            continue
        a = np.array([g[0] for g in got])
        means[h] = a.mean(axis=0)
        sigmas[h] = a.std(axis=0, ddof=1) if len(a) > 1 else np.full(3, np.nan)
        ccs[h] = np.mean([g[1] for g in got])
    print(f"{'holder':<7} {'n':>3} {'dx mm':>9} {'dy mm':>9} {'dz mm':>9}  "
          f"{'sx':>7} {'sy':>7} {'sz':>7} {'ECC':>7}")
    for h in sorted(means):
        m, s = means[h], sigmas[h]
        n = len(usable[h]) - (1 if h == anchor_h else 0)
        print(f"h{h:<6} {n:>3} {m[0]:>9.4f} {m[1]:>9.4f} {m[2]:>9.4f}  "
              f"{s[0]:>7.4f} {s[1]:>7.4f} {s[2]:>7.4f} {ccs[h]:>7.4f}")
    order = sorted(means)
    if len(order) >= 2:
        print("  per-slot step in dy:", ", ".join(
            f"h{a}->h{b} {means[b][1] - means[a][1]:+.4f}"
            for a, b in zip(order, order[1:])))
    return means


rot, fx, fy = load_transform(os.path.join(REPO, "T_ee_cam.yaml"))
target = common_target()
frames = rack_frames()
print(f"common target window {target}, gate {GATE}")
for step in RACK_STEPS:
    counts = ", ".join(f"h{h}:{len(v)}" for h, v in sorted(frames[step].items()))
    print(f"  step {step:<2} ({SEAT[step]}): {counts}")

tables = {s: report(s, frames[s], target, rot, fx, fy) for s in RACK_STEPS}

if all(tables.get(s) for s in RACK_STEPS):
    a, b = tables[18], tables[1]
    shared = sorted(set(a) & set(b))
    print("\n== seat against puck: the two steps' tables, differenced ==")
    print("   (agreement means the offset is where the cup is, not how the puck sits)")
    for h in shared:
        d = a[h] - b[h]
        print(f"h{h}  {d[0]:>9.4f} {d[1]:>9.4f} {d[2]:>9.4f}   |d| {np.linalg.norm(d):.4f}")

if 1 in frames[18]:
    print("\nholder 1 is measured by neither: its seat was empty at both rack stops,")
    print("so it matches step 18's seat state but not its loaded fingers, and step 1's")
    print("fingers but not its loaded seat. Put a puck in it and the anchor moves there.")
