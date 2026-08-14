"""Offline confusion matrix for the occupancy decision, over the shipped code.

Every taught reference is a labelled frame: a step-1/12 capture is a loaded
seat, a step-7/18 capture is an empty one, both at the same pose and both
carrying the same target window. That makes the decision testable without the
arm — replay each frame at each stop and see what the node answers.

It calls `Node.observe` itself rather than re-deriving the comparison, because
a test that paraphrases the code under test stops testing it the first time
either side is edited. `Node` is built without `__init__` so that nothing here
opens a camera or a PV; `observe` reads only `args.refs`, `cam.grab`, and the
hand-eye numbers, and the last of those cannot change a verdict.

The holder-1 dumps in `vision_survey/` are the case that matters: they were
taken at a seat that was empty while the cycle believed it loaded.
"""

import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vision_node import KINDS, STEP_KIND, Node, ref_key, seat_state  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REFS = os.path.join(REPO, "vision_refs")
SURVEY = os.path.join(REPO, "vision_survey")


class Replay:
    """A camera that hands out one stored frame."""

    def __init__(self, npz):
        self.frame = npz["frame"].astype(np.float32)
        self.z = npz["depth_map"].astype(float)

    def grab(self):
        return self.frame, self.z


class Args:
    refs = REFS


def node_for(npz):
    n = Node.__new__(Node)
    n.args = Args()
    n.cam = Replay(npz)
    n.rot, n.fx, n.fy = np.eye(3), 1.0, 1.0
    return n


def replay(npz, step, holder):
    """`(d, quality, note)` for this frame presented at this stop."""
    return node_for(npz).observe(STEP_KIND[step], step, holder)


def taught(step, holder):
    path = os.path.join(REFS, ref_key(STEP_KIND[step], step, holder) + ".npz")
    return np.load(path) if os.path.exists(path) else None


def show(label, step, holder, npz, want_answer):
    """Print one replay, and say whether it matched what the seat really holds."""
    d, quality, note = replay(npz, step, holder)
    got = "answered" if d is not None else "refused"
    ok = got == want_answer
    print(f"  {label:<34} {got:<9} q={quality:.4f}  {'ok' if ok else '<-- WRONG'}")
    if d is None:
        print(f"      {note}")
    return ok


bad = 0
print("== each taught reference replayed at its own stop (seat state is right) ==")
for holder in (2, 3, 4):
    for step in (1, 18):
        d = taught(step, holder)
        if d is not None:
            bad += not show(f"h{holder} s{step} {seat_state(STEP_KIND[step])} seat",
                            step, holder, d, "answered")
for step in (7, 12):
    d = taught(step, 0)
    bad += not show(f"sample holder s{step} {seat_state(STEP_KIND[step])} seat",
                    step, 0, d, "answered")

print("\n== the collision case: a loaded seat replayed at the step that places ==")
for holder in (2, 3, 4):
    d = taught(1, holder)
    if d is not None:
        bad += not show(f"h{holder} loaded seat at s18", 18, holder, d, "refused")
d = taught(12, 0)
bad += not show("sample holder loaded at s7", 7, 0, d, "refused")

print("\n== the empty-pick case: an empty seat replayed at the step that picks ==")
for holder in (2, 3, 4):
    d = taught(18, holder)
    if d is not None:
        bad += not show(f"h{holder} empty seat at s1", 1, holder, d, "refused")
d = taught(7, 0)
bad += not show("sample holder empty at s12", 12, 0, d, "refused")

print("\n== holder 1's real cycle, the one that picked nothing ==")
print("   Every stop of it should refuse, and for four different reasons: the")
print("   rack seat was empty, the fingers were empty, the sample holder never")
print("   received a puck, and holder 1 has no reference of its own — so the")
print("   rack stops are scored against 2/3/4, which is also a wrong-holder test.")
for path in sorted(glob.glob(os.path.join(SURVEY, "h1_s*.npz"))):
    d = np.load(path)
    step = int(d["step"])
    if step in (7, 12):
        bad += not show(f"s{step} {KINDS[STEP_KIND[step]]}", step, 0, d, "refused")
        continue
    for holder in (2, 3, 4):
        bad += not show(f"s{step} {KINDS[STEP_KIND[step]]} vs h{holder}", step, holder,
                        d, "refused")

print(f"\n{bad} replay(s) decided wrongly")
