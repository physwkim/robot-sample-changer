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
  (§14) — the measurement is not what limits this.
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

    vision_node.py teach          teach the reference for the pose the arm is at
    vision_node.py probe          answer once from the current frame, write nothing
    vision_node.py run            serve the handshake

`teach` is the step that makes the rest work, and it is a robot-free operation:
drive the sequencer to the observation step by any means (`PauseStep` holds it
there — see `reapproach.py cycles`), then run `teach`. It keys the reference by
`(Kind, CurrentStep)`, plus `Holder` at the two rack poses, because `Kind`
alone cannot tell step 1 from step 12.
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

PREFIX = os.environ.get("VISION_CAM_PREFIX", "RS405:")
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

KINDS = {1: "PickAlign", 2: "GripOffset", 3: "PlaceAlign", 4: "Seating"}
# The kinds this node can measure. The rest answer Valid=0 — see the module
# docstring for why silence is better than zeros.
ANSWERABLE = (1, 3)
# The two observation poses that depend on which holder the run is using;
# the sample-holder poses (steps 7 and 12) do not.
RACK_STEPS = (1, 18)

# Below this the target is not in the gate at all, and the answer is not a
# small correction but "the arm is not where the reference was taught".
MIN_GATE_PX = 500
# ECC correlation below this is not a shifted view of the reference scene.
# The 20-cycle campaign ran 0.9723..0.9981 with the arm doing real work.
MIN_ECC = 0.90
# Grown around the target's bounding box so the correlation has context and
# a shifted feature does not leave the crop.
CROP_MARGIN_PX = 24


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
        """Measure against the stored reference for `key`.

        Returns `(d_mm, quality, note)` with `d_mm = None` when there is no
        answer to give. Every `None` carries a note, because the sequencer
        turns an invalid answer into a stopped run and the log is what tells
        the operator which of the several reasons it was.
        """
        path = os.path.join(self.args.refs, key + ".npz")
        if not os.path.exists(path):
            return None, 0.0, f"no reference taught for {key}"
        stored = np.load(path)
        window = tuple(int(v) for v in stored["window"])
        gate = tuple(float(v) for v in stored["gate"])

        frame = self.cam.grab()
        if frame is None:
            return None, 0.0, "no fresh frame from the camera"
        img, z = frame
        roi, area = build_roi(z, window, gate)
        if area < MIN_GATE_PX:
            return None, 0.0, f"gate holds {area} px in the taught window (< {MIN_GATE_PX})"
        depth = gate_depth_m(z, window, gate)
        if depth is None:
            return None, 0.0, "the gate is empty; no range to scale the shift by"

        y0, y1, x0, x1 = window
        shift = ecc_shift(stored["ref"], img[y0:y1, x0:x1], roi)
        if shift is None:
            return None, 0.0, "ECC did not converge"
        du, dv, cc = shift
        if cc < MIN_ECC:
            return None, float(cc), f"ECC correlation {cc:.4f} below {MIN_ECC}"

        d_cam = np.array([du * depth / self.fx, dv * depth / self.fy, 0.0])
        d_mm = (self.rot @ d_cam) * 1000.0
        note = (
            f"du={du:+.4f} dv={dv:+.4f} px, Z={depth * 1000:.1f} mm, "
            f"ECC={cc:.4f}, gate={area} px"
        )
        return d_mm, float(cc), note

    # ---- commands ------------------------------------------------------

    def cmd_teach(self):
        step = int(self.get("CurrentStep"))
        holder = int(self.get("Holder"))
        kind = self.args.kind
        if kind is None:
            raise SystemExit("--kind is required: 1=PickAlign, 3=PlaceAlign")
        key = ref_key(kind, step, holder)

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
        path = os.path.join(self.args.refs, key + ".npz")
        np.savez(
            path,
            ref=img[y0:y1, x0:x1],
            roi=roi,
            window=np.array(window),
            gate=np.array(DEFAULT_GATE),
            depth_m=depth,
        )
        print(f"taught {key} -> {path}")
        print(
            f"  {KINDS.get(kind, kind)} at step {step}"
            + (f", holder {holder}" if step in RACK_STEPS else "")
        )
        print(f"  window {window}, gate {area} px, range {depth * 1000:.1f} mm")

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
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("teach").set_defaults(fn="cmd_teach")
    sub.add_parser("probe").set_defaults(fn="cmd_probe")
    sub.add_parser("run").set_defaults(fn="cmd_run")
    args = p.parse_args()
    getattr(Node(args), args.fn)()


if __name__ == "__main__":
    main()
