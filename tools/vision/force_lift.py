"""Can the arm feel the puck? Measured at the poses it visits loaded and empty.

The question force has to answer before any of it is worth building: UR3e
estimates `actual_TCP_force` from joint currents rather than a wrist sensor, so
a small payload may sit under the noise. Nothing in the datasheet settles it
for this puck on this arm.

The sequence answers it for free. Each cycle visits the rack standby twice —
step 1 with empty fingers, step 18 with the puck in them — and the sample
holder standby twice the other way round, step 7 loaded and step 12 empty.
Same pose, same joints, same gravity torque from the arm and gripper. What
differs between the two visits is the puck, so the difference in Fz *is* the
puck's weight, with every modelling error common to both sides.

Finding those visits needs no step numbers: the arm is held still at each of
them by `PauseStep`, so they are the long stationary segments of the trace,
and the joint vector says which pose each one was.

    force_lift.py force_125hz.csv
"""

import sys

import numpy as np

# Held still at an observation stop for seconds; a move between them is never
# this quiet for this long.
STILL_RAD = 1e-4     # max joint travel within a segment to call it stationary
MIN_STILL_S = 1.0
RATE_HZ = 125.0
# Two stationary segments are the same pose if every joint agrees this well.
SAME_POSE_RAD = 0.01


def segments(t, q):
    """Stationary stretches as `(start, end)` index pairs."""
    moving = np.abs(np.diff(q, axis=0)).max(axis=1) > STILL_RAD
    out, i = [], 0
    while i < len(moving):
        if moving[i]:
            i += 1
            continue
        j = i
        while j < len(moving) and not moving[j]:
            j += 1
        if (t[j] - t[i]) >= MIN_STILL_S:
            out.append((i, j))
        i = j + 1
    return out


d = np.genfromtxt(sys.argv[1] if len(sys.argv) > 1 else "force.csv",
                  delimiter=",", names=True)
t = d["t"]
q = np.column_stack([d[f"q{i}"] for i in range(6)])
f = np.column_stack([d[c] for c in ("fx", "fy", "fz")])
tau = np.column_stack([d[c] for c in ("tx", "ty", "tz")])
z = d["z"]

segs = segments(t, q)
print(f"{len(t)} samples over {t[-1] - t[0]:.1f} s, {len(segs)} stationary stops\n")
if not segs:
    raise SystemExit("no stationary segment found — was the arm running?")

# Group the stops by pose. The rack standby and the sample-holder standby each
# come round twice a cycle, so a pose visited many times is an observation stop.
poses, groups = [], []
for a, b in segs:
    mid = q[(a + b) // 2]
    for k, p in enumerate(poses):
        if np.abs(mid - p).max() < SAME_POSE_RAD:
            groups[k].append((a, b))
            break
    else:
        poses.append(mid)
        groups.append([(a, b)])

print(f"{'pose':<5} {'visits':>6} {'z mm':>8} {'Fz N':>9} {'sigma':>7} "
      f"{'Fx N':>9} {'Fy N':>9} {'|tau| Nm':>9}")
stats = []
for k, g in enumerate(groups):
    fz = np.array([f[a:b, 2].mean() for a, b in g])
    noise = np.mean([f[a:b, 2].std(ddof=1) for a, b in g if b - a > 2])
    fx = np.mean([f[a:b, 0].mean() for a, b in g])
    fy = np.mean([f[a:b, 1].mean() for a, b in g])
    tm = np.mean([np.linalg.norm(tau[a:b].mean(axis=0)) for a, b in g])
    zz = np.mean([z[a:b].mean() for a, b in g])
    stats.append((k, g, fz, noise))
    print(f"p{k:<4} {len(g):>6} {zz:>8.1f} {fz.mean():>9.3f} {fz.std():>7.3f} "
          f"{fx:>9.3f} {fy:>9.3f} {tm:>9.4f}")

print("\nwithin-stop sample noise (sigma of Fz while standing still):")
for k, g, fz, noise in stats:
    print(f"  p{k}: {noise:.4f} N over {len(g)} visit(s)")

print("\nsplit of each pose's visits — a pose visited loaded and empty should")
print("show two clusters separated by the puck's weight:")
for k, g, fz, noise in stats:
    if len(fz) < 2:
        continue
    order = np.sort(fz)
    gaps = np.diff(order)
    split = int(np.argmax(gaps))
    lo, hi = order[: split + 1], order[split + 1:]
    print(f"  p{k}: {len(fz)} visits, values " + " ".join(f"{v:.3f}" for v in order))
    if len(lo) and len(hi):
        sep = hi.mean() - lo.mean()
        spread = max(lo.std(), hi.std(), noise)
        print(f"       largest gap {gaps[split]:.3f} N -> {lo.mean():.3f} vs "
              f"{hi.mean():.3f} N, separation {sep:.3f} N against spread {spread:.4f} N")
