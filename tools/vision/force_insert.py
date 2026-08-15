"""Lateral force through the vertical insert and extract — the rub, if there is one.

The criterion this looks for is not image agreement. A puck that goes into the
cup off-centre rubs the wall on the way down, and the sample takes that as a
shake; the offsets are right when the descent is quiet, which is a statement
about force, not pixels. Image alignment is a proxy for it — a good one, but
still a proxy, and one that cannot see a cup whose bore is not where its rim
looks like it is.

So this finds the vertical segments of a cycle — the ones where the TCP moves
in z with x and y held — and reports what the lateral force did across each.
A clean insert holds Fx/Fy where they were in free air. A rub shows up as
lateral force that builds with depth and lets go at the bottom, and as a wrist
torque with it.

Fx/Fy are in the base frame, and the whole rack is approached from one side, so
they are not rotated into anything: what matters is the *change* across the
segment against the same arm's noise standing still.

    force_insert.py force_125hz.csv
"""

import sys

import numpy as np

RATE_HZ = 125.0
# A vertical move: z travelling while x and y hold. The insert is a few mm, so
# the threshold is well under that and well over the pose noise.
MIN_DZ_M = 0.002
MAX_LATERAL_M = 0.0015
MIN_SAMPLES = 8


def vertical_runs(x, y, z):
    """Index pairs where z moves and x/y do not."""
    moving = np.abs(np.diff(z)) > 1e-6
    runs, i = [], 0
    while i < len(moving):
        if not moving[i]:
            i += 1
            continue
        j = i
        while j < len(moving) and moving[j]:
            j += 1
        if (j - i) >= MIN_SAMPLES:
            dz = z[j] - z[i]
            lat = np.hypot(x[i:j] - x[i], y[i:j] - y[i]).max()
            if abs(dz) >= MIN_DZ_M and lat <= MAX_LATERAL_M:
                runs.append((i, j, dz))
        i = j + 1
    return runs


d = np.genfromtxt(sys.argv[1] if len(sys.argv) > 1 else "force.csv",
                  delimiter=",", names=True)
t, x, y, z = d["t"], d["x"], d["y"], d["z"]
fx, fy, fz = d["fx"], d["fy"], d["fz"]
tx, ty = d["tx"], d["ty"]

runs = vertical_runs(x, y, z)
print(f"{len(t)} samples over {t[-1] - t[0]:.1f} s, {len(runs)} vertical segments\n")
# Which station a segment belongs to is in its own x/y — the rack and the
# sample holder are hundreds of millimetres apart, and each rack slot is 30 mm
# from the next. No step number needs to be logged to tell them apart.
stations = []
for i, j, dz in runs:
    p = np.array([x[i], y[i]])
    for k, (c, n) in enumerate(stations):
        if np.linalg.norm(p - c) < 0.010:
            stations[k] = ((c * n + p) / (n + 1), n + 1)
            break
    else:
        stations.append((p, 1))


def station_of(i):
    p = np.array([x[i], y[i]])
    return int(np.argmin([np.linalg.norm(p - c) for c, _ in stations]))


print(f"{'seg':<4} {'stn':>3} {'dz mm':>8} {'dur s':>6} "
      f"{'dFx N':>8} {'dFy N':>8} {'|dF_lat|':>9} {'dTx Nm':>8} {'dTy Nm':>8} {'dFz N':>8}")
for k, (i, j, dz) in enumerate(runs):
    # Against the segment's own start, which is free air a millimetre above the
    # cup: the arm's static bias and the payload both cancel there.
    dfx = fx[i:j] - fx[i]
    dfy = fy[i:j] - fy[i]
    lat = np.hypot(dfx, dfy)
    peak = int(np.argmax(lat))
    print(f"{k:<4} {station_of(i):>3} {dz * 1000:>8.2f} {(t[j] - t[i]):>6.2f} "
          f"{dfx[peak]:>8.3f} {dfy[peak]:>8.3f} {lat[peak]:>9.3f} "
          f"{ty[i:j][peak] - ty[i]:>8.4f} {tx[i:j][peak] - tx[i]:>8.4f} "
          f"{fz[i:j].max() - fz[i]:>8.3f}")

if runs:
    lat_peaks = []
    for i, j, dz in runs:
        lat_peaks.append(np.hypot(fx[i:j] - fx[i], fy[i:j] - fy[i]).max())
    lat_peaks = np.array(lat_peaks)
    down = [p for (i, j, dz), p in zip(runs, lat_peaks) if dz < 0]
    up = [p for (i, j, dz), p in zip(runs, lat_peaks) if dz > 0]
    print(f"\npeak lateral excursion: descents {np.mean(down) if down else float('nan'):.3f} N "
          f"(n={len(down)}), ascents {np.mean(up) if up else float('nan'):.3f} N (n={len(up)})")
    print("compare against the still-arm noise printed by force_lift.py; a peak")
    print("inside that noise means this cycle did not rub hard enough to feel.")

    print(f"\n{'stn':<4} {'x mm':>9} {'y mm':>9} {'segs':>5} {'down N':>8} {'up N':>8}  "
          "(the number a per-holder offset would be tuned against)")
    for s, (c, _) in enumerate(stations):
        d_ = [p for (i, j, dz), p in zip(runs, lat_peaks) if station_of(i) == s and dz < 0]
        u_ = [p for (i, j, dz), p in zip(runs, lat_peaks) if station_of(i) == s and dz > 0]
        print(f"{s:<4} {c[0] * 1000:>9.1f} {c[1] * 1000:>9.1f} {len(d_) + len(u_):>5} "
              f"{np.mean(d_) if d_ else float('nan'):>8.3f} "
              f"{np.mean(u_) if u_ else float('nan'):>8.3f}")
