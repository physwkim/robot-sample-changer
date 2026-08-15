"""The smallest step the arm actually takes, measured rather than assumed.

The force probe advances by stepping and reads force between steps, so its
resolution is bounded by the smallest move the motion stack will really
execute. That bound is not the arm's repeatability: a trajectory is
TOTG-resampled at 0.1 s and streamed as spline points, and a move short
enough to fall inside one resample interval can come back `Ok` having
produced fewer than two waypoints — that is, having not moved at all
(`motion/execute.rs`, `n < 2`).

So this drives the daemon's own jog at a sweep of commanded sizes and
measures what arrived. It does *not* try to detect the individual steps:
at 0.05 mm a single step sits at three times the 0.016 mm span the pose
reads over a 0.1 s window while standing still, which is not a margin to
build a measurement on. The sweep instead walks five steps out and five
back at each size, so each group is a triangle whose amplitude is five
steps — fifteen times the noise — and the achieved step is that amplitude
over five. Nothing has to be detected but the turning points.

    step_floor.py step_sweep.csv 0.05,0.10,0.20,0.30,0.50,1.00 5
"""

import sys

import numpy as np

RATE_HZ = 125.0
# The pose is smoothed over about a fifth of a second before the turning
# points are taken. The jogs are 1.5 s apart, so this cannot blur two of
# them together, and it puts the single-sample scatter well below the
# smallest amplitude being measured.
SMOOTH = 25


def smooth(v, n):
    return np.convolve(v, np.ones(n) / n, mode="same")


d = np.genfromtxt(sys.argv[1], delimiter=",", names=True)
commanded = [float(s) for s in sys.argv[2].split(",")]
per_leg = int(sys.argv[3]) if len(sys.argv) > 3 else 5

# The jog runs along the tool axis; which base axis carries it is whichever
# moved, and asking the data is safer than assuming the mount.
xyz = np.column_stack([d["x"], d["y"], d["z"]]) * 1000.0
axis = int(np.argmax(xyz.max(axis=0) - xyz.min(axis=0)))
v = smooth(xyz[:, axis], SMOOTH)
edge = SMOOTH  # the convolution's ends are not usable
v = v[edge:-edge]
print(f"{len(v)} usable samples, motion along base {'xyz'[axis]}, "
      f"total span {np.ptp(v):.4f} mm")

# Groups are equal in duration by construction: the sweep spends the same
# number of jogs at the same cadence on every size. Splitting by time and
# taking each group's own peak-to-peak needs no turning point found by
# name, and no clock shared with the daemon's log.
n_groups = len(commanded)
bounds = np.linspace(0, len(v), n_groups + 1).astype(int)

print(f"\n{'commanded':>10} {'amplitude':>10} {'achieved':>9} {'delivered':>10}")
achieved = []
for k, want in enumerate(commanded):
    seg = v[bounds[k]:bounds[k + 1]]
    amp = np.ptp(seg)
    got = amp / per_leg
    achieved.append(got)
    print(f"{want:>10.3f} {amp:>10.4f} {got:>9.4f} {got / want:>9.1%}")

print("\nA size that delivers well under 100% is one the motion stack is")
print("swallowing, not one the arm cannot resolve: the probe must step at")
print("or above the smallest size that arrives in full.")
