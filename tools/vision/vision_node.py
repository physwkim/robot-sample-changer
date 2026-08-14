#!/usr/bin/env python3
"""The vision node — the last piece §10 lists as missing.

The sequencer has been wired for this since before the camera worked: it
raises `Robot:Vision:Req` with a `Kind`, waits `vision.timeout` seconds, and
reads back `DX/DY/DZ` as millimetres in the tool frame of the pose it made the
request from. `vision_sim` has stood in for this program during rehearsals and
is the contract: answer the payload first and `Done` last, because the
sequencer treats the `Done` echo as "results ready".

What this node measures, and does not:

- **Pick Align / Place Align** (kinds 1, 3). Grounded. The arm is parked at a
  standby pose (§13), the holder stack fills the depth gate, and the estimator
  in `estimator.py` recovers how far the scene has slid since a taught
  reference. Its floor is 0.0010 mm against an arm that repeats to 0.022 mm
  (§14) — the measurement is not what limits this. The answer comes out of two
  correlations rather than one: the whole column says *whether this is the
  right scene*, the target alone says *how far it moved*. §15.9 measured why
  neither window can do both.
- **Grip Offset** (kind 2) and **Seating** (kind 4). Not measured, and this
  node says so: it answers `Valid = 0` rather than zeros. Zeros would read as
  "perfectly aligned" and "seated", pass the deadband, and be indistinguishable
  from a real answer. §13.2 explains why grip offset is hard here — the puck
  rides 33 deg off the optical axis with only its near rim in frame — and
  seating needs a tilt that this metal's depth (1.83 mm bias, 8.48 mm spread,
  §12.1) cannot support. Both hooks ship off in `sequencer.yaml`; turning one
  on without teaching this node how to answer it stops the sequence, which is
  the correct failure.

The geometry, for the two kinds it does answer. A feature that has moved
`(du, dv)` pixels since the reference has moved, in the camera frame,

    d_cam = (du * Z / fx,  dv * Z / fy,  0)

with `Z` the gate's median range. The third component is zero because a
translation in the image plane says nothing along the optical axis — that is
the axis this node does not measure, not an axis it claims is zero in the
tool. In the tool frame,

    d_ee = R_ee_cam @ d_cam

which generally has all three components non-zero: `R_ee_cam` tilts 12.4 deg,
so an image-plane shift genuinely moves the tool in z. `T_ee_cam.yaml` carries
`R_ee_cam` and the intrinsics it was solved under, and those intrinsics — not
the IOC's — are the ones to use, because the hand-eye fit refined them.

    vision_node.py teach-run      drive one cycle and teach the missing references
    vision_node.py check-run      drive one cycle and answer at each, teaching nothing
    vision_node.py teach          teach the reference for the pose the arm is at
    vision_node.py probe          answer once from the current frame, write nothing
    vision_node.py run            serve the handshake

References are what make the rest work, and `teach-run` is the way to get
them: the four observation poses have to be referenced in the states a real
run finds them in, and those differ — the rack is full at step 1 and empty at
18, the sample holder empty at 7 and full at 12. One cycle passes through all
four in order. `teach-run` moves the robot; every other mode only reads.

A reference is keyed by `(Kind, CurrentStep)`, plus `Holder` at the two rack
poses, because `Kind` alone cannot tell step 1 from step 12. Every holder the
sequence will use therefore needs its own `teach-run`, and those runs are
additive: a reference already taught is kept unless `--force` says otherwise.
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import yaml
from epics import PV

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from estimator import DEFAULT_GATE, Camera, build_roi, ecc_shift, gate_depth_m, gate_mask
from sequencer import Robot

PREFIX = os.environ.get("VISION_CAM_PREFIX", "RS405:")
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

KINDS = {1: "PickAlign", 2: "GripOffset", 3: "PlaceAlign", 4: "Seating"}
# The kinds this node can measure. The rest answer Valid=0 — see the module
# docstring for why silence is better than zeros.
ANSWERABLE = (1, 3)
# The two observation poses that depend on which holder the run is using;
# the sample-holder poses (steps 7 and 12) do not.
RACK_STEPS = (1, 18)
# `(CurrentStep, Kind)` for every reference a run needs, in the order one
# cycle visits them. Each is a different scene, not just a different pose:
# the rack is full at 1 and empty at 18, the sample holder empty at 7 and
# full at 12.
TEACH_STOPS = ((1, 1), (7, 3), (12, 1), (18, 3))

# Below this the target is not in the gate at all, and the answer is not a
# small correction but "the arm is not where the reference was taught".
MIN_GATE_PX = 500
# Grown around the target's bounding box so the correlation has context and
# a shifted feature does not leave the crop.
CROP_MARGIN_PX = 24

# The two correlation floors, one per stage (§15.9). They are different
# numbers because the stages answer different questions of the same frame.
#
# Identity, over the whole column: is this the scene the reference was taught
# in? Wrong holder correlates at 0.481..0.681 — the holders are nominally
# identical, and it is the *neighbours* that tell them apart, so this stage
# needs the wide window. One neighbouring slot changing its load costs 0.868,
# and 0.78 sits in that valley: a rack whose loadout moved on still passes,
# a reference from another holder does not.
MIN_IDENTITY_ECC = 0.78
# Measurement, over the target alone: the arm's own re-approach ran
# 0.9723..0.9997 with the arm doing real work, so anything near the identity
# floor here means the target itself is not what was taught.
MIN_TARGET_ECC = 0.90

# How the target window is found. Not a tuned rectangle — the pose pairs
# below visit the same standby twice, once with the puck present and once
# without, so the target is simply what changes between them.
POSE_PAIRS = ((1, 18), (7, 12))
STEP_KIND = {step: kind for step, kind in TEACH_STOPS}
# A frame-to-frame difference in DN that is the puck leaving rather than the
# sensor's 1.41 DN pixel noise (§12.3), and the smallest blob worth calling
# part of the target.
TARGET_DIFF_DN = 40
MIN_TARGET_PX = 100
# Room for the correlation to slide without pulling a neighbour in. The
# measurement stage wants the target and little else — that immunity to the
# neighbours is the whole point of splitting the stages.
TARGET_MARGIN_PX = 8


def load_transform(path):
    """`(R_ee_cam, fx, fy)` from the hand-eye solve, as `solve_joint.py` wrote it."""
    if not os.path.exists(path):
        raise SystemExit(
            f"no hand-eye transform at {path} — run tools/handeye/solve_joint.py first"
        )
    d = yaml.safe_load(open(path))
    r = np.array(d["T_ee_cam"]["rotation_matrix"], dtype=float)
    k = np.array(d["camera_matrix"], dtype=float).reshape(3, 3)
    return r, float(k[0, 0]), float(k[1, 1])


def target_crop(z, gate=DEFAULT_GATE):
    """The bounding box of the nearest object in the gate, or None.

    The gate keeps more than the target: §12.1 measured a 1505 px blob in one
    frame's corner that belongs to nothing. Taking the largest connected
    component drops it without a hand-tuned window, which is what lets one
    reference procedure serve all four observation poses.
    """
    raw, grown = gate_mask(z, gate)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(grown, connectivity=8)
    if count < 2:
        return None
    biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, w, h = (int(stats[biggest, i]) for i in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                                                   cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    m = CROP_MARGIN_PX
    y0, y1 = max(0, y - m), min(z.shape[0], y + h + m)
    x0, x1 = max(0, x - m), min(z.shape[1], x + w + m)
    if (y1 - y0) < 16 or (x1 - x0) < 16:
        return None
    return (y0, y1, x0, x1)


def ref_key(kind, step, holder):
    """`(Kind, CurrentStep)`, and `Holder` where the pose depends on it.

    Kind alone is ambiguous: Pick Align fires at step 1 and again at step 12,
    Place Align at 7 and 18, and those are different scenes.
    """
    if step in RACK_STEPS:
        return f"k{kind}_s{step}_h{holder}"
    return f"k{kind}_s{step}"


def target_box(fa, za, fb, zb, gate):
    """Where the puck is, from a full frame and an empty one of the same pose.

    Both frames have to be gated in for a pixel to count. The puck the gripper
    is carrying is out of the gate in one of them and world-fixed geometry
    stands behind it in the other, so without that intersection the biggest
    "change" in the frame is the occlusion rather than the target.
    """
    _, ga = gate_mask(za, gate)
    _, gb = gate_mask(zb, gate)
    both = ((ga > 0) & (gb > 0)).astype(np.uint8)
    diff = np.abs(fa.astype(np.int16) - fb.astype(np.int16)).astype(np.uint8)
    diff = cv2.GaussianBlur(diff, (9, 9), 0) * both
    mask = cv2.morphologyEx(
        (diff > TARGET_DIFF_DN).astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)
    )
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    keep = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= MIN_TARGET_PX]
    if not keep:
        return None, 0
    # The union, not the largest: the rim and the seat come out as separate
    # blobs on some holders and both belong to the same target.
    y0 = min(int(stats[i, cv2.CC_STAT_TOP]) for i in keep)
    y1 = max(int(stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT]) for i in keep)
    x0 = min(int(stats[i, cv2.CC_STAT_LEFT]) for i in keep)
    x1 = max(int(stats[i, cv2.CC_STAT_LEFT] + stats[i, cv2.CC_STAT_WIDTH]) for i in keep)
    m = TARGET_MARGIN_PX
    box = (
        max(0, y0 - m),
        min(fa.shape[0], y1 + m),
        max(0, x0 - m),
        min(fa.shape[1], x1 + m),
    )
    return box, sum(int(stats[i, cv2.CC_STAT_AREA]) for i in keep)


class Node:
    NAMES = (
        "Vision:Req",
        "Vision:Kind",
        "Vision:Done",
        "Vision:Valid",
        "Vision:DX",
        "Vision:DY",
        "Vision:DZ",
        "Vision:Quality",
        "Vision:Seated",
        "Vision:Tilt",
        "CurrentStep",
        "Holder",
    )

    def __init__(self, args):
        self.args = args
        self.cam = Camera(prefix=PREFIX)
        self.rot, self.fx, self.fy = load_transform(args.transform)
        self.pv = {n: PV("Robot:" + n, auto_monitor=False) for n in self.NAMES}
        for pv in self.pv.values():
            if not pv.wait_for_connection(5):
                raise SystemExit(f"{pv.pvname} did not connect")

    def get(self, name):
        return self.pv[name].get(use_monitor=False)

    def put(self, name, value):
        self.pv[name].put(value, wait=True)

    # ---- measurement ---------------------------------------------------

    def observe(self, key):
        """Measure against the stored reference for `key`, in two stages.

        The stages exist because one window cannot do both jobs (§15.9).
        Identity needs the whole column — the holders are nominally identical
        and only their neighbours distinguish them, so a target-sized window
        correlates 0.92..0.96 against the wrong holder and would answer it.
        Measurement wants the opposite: the target alone, so that a neighbour
        being loaded or emptied between teaching and running does not move the
        number. Widen the measurement and it picks up the neighbours;
        narrow the identity check and it stops being one.

        Returns `(d_mm, quality, note)` with `d_mm = None` when there is no
        answer to give. Every `None` carries a note, because the sequencer
        turns an invalid answer into a stopped run and the log is what tells
        the operator which of the several reasons it was.
        """
        path = os.path.join(self.args.refs, key + ".npz")
        if not os.path.exists(path):
            return None, 0.0, f"no reference taught for {key}"
        stored = np.load(path)
        if "target" not in stored:
            return None, 0.0, (
                f"{key} carries no target window — it was taught before its pose pair "
                "existed; re-run teach-run"
            )
        window = tuple(int(v) for v in stored["window"])
        target = tuple(int(v) for v in stored["target"])
        gate = tuple(float(v) for v in stored["gate"])

        frame = self.cam.grab()
        if frame is None:
            return None, 0.0, "no fresh frame from the camera"
        img, z = frame

        # Stage 1 — identity, over the wide window. Nothing is measured here;
        # the shift it recovers is thrown away.
        roi, area = build_roi(z, window, gate)
        if area < MIN_GATE_PX:
            return None, 0.0, f"gate holds {area} px in the taught window (< {MIN_GATE_PX})"
        y0, y1, x0, x1 = window
        ident = ecc_shift(stored["ref"], img[y0:y1, x0:x1], roi)
        if ident is None:
            return None, 0.0, "the identity correlation did not converge"
        cc_id = ident[2]
        if cc_id < MIN_IDENTITY_ECC:
            return None, float(cc_id), (
                f"identity correlation {cc_id:.4f} below {MIN_IDENTITY_ECC} — this is not "
                "the scene the reference was taught in"
            )

        # Stage 2 — the measurement, over the target alone.
        troi, tarea = build_roi(z, target, gate)
        if tarea < MIN_GATE_PX:
            return None, float(cc_id), (
                f"gate holds {tarea} px on the target (< {MIN_GATE_PX})"
            )
        depth = gate_depth_m(z, target, gate)
        if depth is None:
            return None, float(cc_id), "the target's gate is empty; no range to scale by"
        ty0, ty1, tx0, tx1 = target
        shift = ecc_shift(
            stored["frame"][ty0:ty1, tx0:tx1].astype(np.float32),
            img[ty0:ty1, tx0:tx1],
            troi,
        )
        if shift is None:
            return None, float(cc_id), "the target correlation did not converge"
        du, dv, cc = shift
        if cc < MIN_TARGET_ECC:
            return None, float(cc), (
                f"target correlation {cc:.4f} below {MIN_TARGET_ECC} (identity "
                f"{cc_id:.4f} passed) — the scene is right but the target is not"
            )

        d_cam = np.array([du * depth / self.fx, dv * depth / self.fy, 0.0])
        d_mm = (self.rot @ d_cam) * 1000.0
        note = (
            f"du={du:+.4f} dv={dv:+.4f} px, Z={depth * 1000:.1f} mm, "
            f"ECC={cc:.4f} on {tarea} px (identity {cc_id:.4f} on {area} px)"
        )
        return d_mm, float(cc), note

    # ---- commands ------------------------------------------------------

    def cmd_teach(self):
        if self.args.kind is None:
            raise SystemExit("--kind is required: 1=PickAlign, 3=PlaceAlign")
        holder = int(self.get("Holder"))
        self.teach(self.args.kind, int(self.get("CurrentStep")), holder)
        # A single pose is half of a pair; this completes the target window if
        # the other half is already on disk, and says so plainly if it is not.
        self.pair_targets(holder)

    def teach(self, kind, step, holder):
        """Capture the reference for one observation pose, from the frame now.

        An existing reference is kept, not replaced. Teaching a second holder
        drives a whole cycle and therefore passes through steps 7 and 12, whose
        references do not depend on the holder — replacing those would throw
        away captures that a `check-run` has already stood behind, for nothing.
        Re-teaching is a decision (an aged reference, §14.2), so it is `--force`.
        """
        key = ref_key(kind, step, holder)
        path = os.path.join(self.args.refs, key + ".npz")
        if os.path.exists(path) and not self.args.force:
            print(f"keeping {key} — already taught; --force to replace")
            return

        frame = self.cam.grab()
        if frame is None:
            raise SystemExit("no fresh frame from the camera")
        img, z = frame
        window = target_crop(z, DEFAULT_GATE)
        if window is None:
            raise SystemExit(
                "nothing in the 50-250 mm gate to reference: the arm is not at an "
                "observation pose, or nothing is loaded there"
            )
        roi, area = build_roi(z, window, DEFAULT_GATE)
        if area < MIN_GATE_PX:
            raise SystemExit(f"the gate holds only {area} px here (< {MIN_GATE_PX})")
        depth = gate_depth_m(z, window, DEFAULT_GATE)

        os.makedirs(self.args.refs, exist_ok=True)
        y0, y1, x0, x1 = window
        # The whole frame and its depth ride along with the crop. Every capture
        # costs a cycle of the arm, so a change to the window rule — the gate
        # moving, the component test changing — is then re-derived from the
        # frames that were already taught instead of driving the robot again.
        np.savez(
            path,
            ref=img[y0:y1, x0:x1],
            roi=roi,
            window=np.array(window),
            gate=np.array(DEFAULT_GATE),
            depth_m=depth,
            frame=img.astype(np.uint8),
            depth_map=z.astype(np.float32),
        )
        print(f"taught {key} -> {path}")
        print(
            f"  {KINDS.get(kind, kind)} at step {step}"
            + (f", holder {holder}" if step in RACK_STEPS else "")
        )
        print(f"  window {window}, gate {area} px, range {depth * 1000:.1f} mm")

    def pair_targets(self, holder):
        """Give every reference its target window, from the pose it shares.

        A pose is visited twice per cycle with the puck on either side of the
        gripper — the rack at steps 1 and 18, the sample holder at 7 and 12 —
        so the target does not have to be described. It is what changed. This
        runs over the stored frames once the cycle is done; it needs neither
        the camera nor the arm, and running it again is harmless.
        """
        for a_step, b_step in POSE_PAIRS:
            keys = [ref_key(STEP_KIND[s], s, holder) for s in (a_step, b_step)]
            paths = [os.path.join(self.args.refs, k + ".npz") for k in keys]
            missing = [k for k, p in zip(keys, paths) if not os.path.exists(p)]
            if missing:
                print(f"cannot pair steps {a_step}/{b_step}: {', '.join(missing)} missing")
                continue
            a, b = (np.load(p) for p in paths)
            box, changed = target_box(
                a["frame"],
                a["depth_map"].astype(float),
                b["frame"],
                b["depth_map"].astype(float),
                tuple(float(v) for v in a["gate"]),
            )
            if box is None:
                print(
                    f"cannot pair steps {a_step}/{b_step}: nothing changed between the "
                    "two frames — was the puck actually moved?"
                )
                continue
            for key, path in zip(keys, paths):
                stored = dict(np.load(path))
                stored["target"] = np.array(box)
                np.savez(path, **stored)
            _, area = build_roi(a["depth_map"].astype(float), box,
                                tuple(float(v) for v in a["gate"]))
            print(
                f"paired {keys[0]} + {keys[1]}: target {box}, "
                f"{changed} px changed, gate {area} px"
            )

    def cmd_check_run(self):
        """Drive one cycle and answer at each stop without teaching.

        What a taught reference is worth is not visible at teach time — the
        capture always succeeds against itself. This runs the same four stops
        one cycle later and prints what the node would have answered. Every
        reading is the arm's re-approach plus whatever the scene did, so with
        good references they land near §14's 0.02 mm and well under
        `min_correction`. A large one, or a fallen ECC, is the reference
        having aged (§15.5) rather than the holder having moved.
        """
        self.run_stops(teach=False)

    def cmd_teach_run(self):
        """Teach the references this holder is missing, in one production cycle.

        Each observation pose has to be referenced in the state the run will
        actually find it in, and those states differ: at step 1 the puck is in
        the rack, at step 7 the sample holder is **empty** and the puck is in
        the fingers, at step 12 the puck is in the sample holder, and at step
        18 the rack slot is **empty** again. A cycle visits all four in that
        order, which is why this drives one rather than asking an operator to
        park the arm four times.

        `PauseStep` does the holding, and writing the next stop releases the
        current hold and arms the next one in the same write. A capture that
        fails does not abort the cycle — the arm is mid-run with the puck in
        its fingers, and stopping there is worse than finishing without one
        reference. The failures are collected and printed at the end.
        """
        self.run_stops(teach=True)

    def run_stops(self, teach):
        """One cycle, stopping at each observation pose to teach or to answer."""
        bot = Robot()
        bot.check_idle()
        holder = int(bot.get("Holder"))
        verb = "teaching" if teach else "checking"
        print(f"{verb} references for holder {holder} over one cycle")

        done, failed = [], []
        try:
            bot.put("PauseStep", TEACH_STOPS[0][0])
            bot.put("Trigger", 1)
            for i, (step, kind) in enumerate(TEACH_STOPS):
                bot.advance_to(step, self.args.arrive_timeout, f"step {step}")
                key = ref_key(kind, step, holder)
                print(f"\nstep {step} — {KINDS[kind]} ({key}):")
                try:
                    if teach:
                        self.teach(kind, step, holder)
                    else:
                        d, quality, note = self.observe(key)
                        if d is None:
                            raise SystemExit(note)
                        print(
                            f"  dx={d[0]:+.4f} dy={d[1]:+.4f} dz={d[2]:+.4f} mm, "
                            f"|d|={float(np.linalg.norm(d)):.4f} mm"
                        )
                        print(f"  {note}")
                    done.append(key)
                except SystemExit as e:
                    print(f"  FAILED: {e}", file=sys.stderr)
                    failed.append(f"{key}: {e}")
                if self.args.dump:
                    self.dump_frame(holder, step)
                # Release this hold and arm the next in one write; 0 after the
                # last one leaves nothing armed.
                bot.put("PauseStep", TEACH_STOPS[i + 1][0] if i + 1 < len(TEACH_STOPS) else 0)
            bot.wait_cycle_end(self.args.cycle_timeout)
        except KeyboardInterrupt:
            print("\ninterrupted — releasing the hold, letting the cycle finish", file=sys.stderr)
            bot.put("PauseStep", 0)
            bot.put("Wait", 1)
            raise SystemExit(f"{len(done)} done before the interrupt")
        bot.put("PauseStep", 0)
        bot.put("Wait", 0)

        if teach:
            print()
            self.pair_targets(holder)

        print(f"\n{len(done)}/{len(TEACH_STOPS)} ok: {', '.join(done)}")
        if failed:
            print("failed:", file=sys.stderr)
            for f in failed:
                print(f"  {f}", file=sys.stderr)
            raise SystemExit(1)

    def dump_frame(self, holder, step):
        """Keep the frame this stop was standing in, for work done afterwards.

        A stop at an observation pose costs a cycle of the arm whether or not
        anything is kept, so a campaign that needs n of them — the per-holder
        seat offsets, say — is n cycles either way. Keeping the frame is what
        makes those cycles reusable by an analysis written later.
        """
        frame = self.cam.grab()
        if frame is None:
            print("  no fresh frame to dump", file=sys.stderr)
            return
        img, z = frame
        os.makedirs(self.args.dump, exist_ok=True)
        path = os.path.join(
            self.args.dump, f"h{holder}_s{step}_{int(time.time())}.npz"
        )
        np.savez(
            path,
            frame=img.astype(np.uint8),
            depth_map=z.astype(np.float32),
            gate=np.array(DEFAULT_GATE),
            holder=holder,
            step=step,
        )
        print(f"  dumped {os.path.basename(path)}")

    def cmd_probe(self):
        step = int(self.get("CurrentStep"))
        holder = int(self.get("Holder"))
        kind = self.args.kind if self.args.kind is not None else int(self.get("Vision:Kind"))
        key = ref_key(kind, step, holder)
        print(f"would answer {KINDS.get(kind, kind)} at step {step} as {key}:")
        d, quality, note = self.observe(key)
        if d is None:
            print(f"  INVALID — {note}")
            return
        print(f"  dx={d[0]:+.4f} dy={d[1]:+.4f} dz={d[2]:+.4f} mm, quality {quality:.4f}")
        print(f"  {note}")

    def answer(self, req, kind, step, holder):
        """Answer one request. Payload first, `Done` last."""
        label = KINDS.get(kind, str(kind))
        if kind not in ANSWERABLE:
            print(
                f"request {req} ({label}) at step {step}: this node does not measure "
                f"{label} — answering invalid",
                flush=True,
            )
            d, quality = None, 0.0
        else:
            d, quality, note = self.observe(ref_key(kind, step, holder))
            verdict = "INVALID — " + note if d is None else note
            print(f"request {req} ({label}) at step {step}: {verdict}", flush=True)

        self.put("Vision:DX", float(d[0]) if d is not None else 0.0)
        self.put("Vision:DY", float(d[1]) if d is not None else 0.0)
        self.put("Vision:DZ", float(d[2]) if d is not None else 0.0)
        self.put("Vision:Quality", quality)
        self.put("Vision:Tilt", 0.0)
        # Seating is not measured; the flag never claims a seat this node
        # cannot see. The Valid=0 beside it is what the sequencer acts on.
        self.put("Vision:Seated", 0)
        self.put("Vision:Valid", 1 if d is not None else 0)
        self.put("Vision:Done", req)
        if d is not None:
            print(
                f"  answered dx={d[0]:+.4f} dy={d[1]:+.4f} dz={d[2]:+.4f} mm "
                f"quality={quality:.4f}",
                flush=True,
            )

    def cmd_run(self):
        # Start from the last ANSWERED id, not zero: the PVs persist across
        # restarts and a stale Req would otherwise look like a new request.
        last = int(self.get("Vision:Done"))
        print(f"vision node ready, refs in {self.args.refs}, last answered id {last}")
        print(f"  measuring: {', '.join(KINDS[k] for k in ANSWERABLE)}")
        print(f"  answering invalid: {', '.join(KINDS[k] for k in KINDS if k not in ANSWERABLE)}")
        while True:
            req = self.get("Vision:Req")
            if req is not None and int(req) != last:
                req = int(req)
                started = time.time()
                self.answer(
                    req,
                    int(self.get("Vision:Kind")),
                    int(self.get("CurrentStep")),
                    int(self.get("Holder")),
                )
                print(f"  {time.time() - started:.2f}s", flush=True)
                last = req
            time.sleep(0.02)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--refs",
        default=os.path.join(REPO, "vision_refs"),
        help="directory of taught reference frames",
    )
    p.add_argument(
        "--transform",
        default=os.path.join(REPO, "T_ee_cam.yaml"),
        help="hand-eye result written by tools/handeye/solve_joint.py",
    )
    p.add_argument("--kind", type=int, help="1=PickAlign, 3=PlaceAlign (teach/probe)")
    p.add_argument(
        "--force",
        action="store_true",
        help="replace references that are already taught instead of keeping them",
    )
    p.add_argument(
        "--dump",
        help="directory to keep every stop's frame in, for analyses written later",
    )
    p.add_argument("--arrive-timeout", type=float, default=180.0, help="seconds to a hold")
    p.add_argument("--cycle-timeout", type=float, default=900.0, help="seconds to cycle end")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("teach").set_defaults(fn="cmd_teach")
    sub.add_parser("teach-run").set_defaults(fn="cmd_teach_run")
    sub.add_parser("check-run").set_defaults(fn="cmd_check_run")
    sub.add_parser("probe").set_defaults(fn="cmd_probe")
    sub.add_parser("run").set_defaults(fn="cmd_run")
    args = p.parse_args()
    getattr(Node(args), args.fn)()


if __name__ == "__main__":
    main()
