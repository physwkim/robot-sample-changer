"""Driving the sequencer from outside, for tools that need the arm somewhere.

The daemon owns the arm — it holds the program slot, and CLAUDE.md's
"읽기는 다중, 쓰기는 하나" is about exactly this. So no tool here commands
motion. They drive the daemon the way an operator does, through `Robot:Trigger`,
and the only reason a measurement is possible at all is `PauseStep`:
`step_epilogue` publishes `CurrentStep` and *then* holds while `PauseStep`
equals it, so the arm stands still at the observation pose for as long as a
frame grab or a reference capture needs.

Releasing means writing a `PauseStep` the run will not match again. Zero
disables the hold outright (`wait_for_pause_step_change` returns early on
zero), which makes zero both the release value and the parking value — and
writing the *next* step number releases the current hold and arms the next one
in a single write.
"""

import time

from epics import PV


class Robot:
    """The sequencer's control PVs. Everything that writes to the arm is here."""

    NAMES = (
        "Trigger",
        "CurrentStep",
        "PauseStep",
        "StartStep",
        "Holder",
        "CalibMode",
        "Stop",
        "Wait",
        "Loaded",
    )

    def __init__(self, connect_timeout=5.0):
        self.pv = {n: PV("Robot:" + n, auto_monitor=False) for n in self.NAMES}
        for pv in self.pv.values():
            if not pv.wait_for_connection(connect_timeout):
                raise SystemExit(f"{pv.pvname} did not connect")

    def get(self, name):
        return self.pv[name].get(use_monitor=False)

    def put(self, name, value):
        self.pv[name].put(value, wait=True)

    def until(self, name, want, timeout, what):
        """Wait for a PV to read `want`. Does not drive the run — see `advance_to`."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.get(name) == want:
                return
            time.sleep(0.05)
        raise SystemExit(f"timed out after {timeout:.0f}s waiting for {what}")

    def answer_measurement(self):
        """Answer the beamline's question, if it is being asked. Idempotent.

        `Loaded` going high at step 12 *is* the question, and the run does not
        move again until something answers it. `Wait` cannot be pre-answered:
        `run_sequence` clears it (`write_wait(0)`) at the top of every run.
        """
        if self.get("Loaded") == 1 and self.get("Wait") != 1:
            self.put("Wait", 1)

    def advance_to(self, step, timeout, what=None):
        """Wait for the run to reach `step`, answering the measurement wait on the way.

        Every wait on the run's *progress* goes through here, and not through
        `until`, because no run gets past step 12 unanswered. Waiting for a
        later step with a bare `until` looks like it works — right up to the
        first stop beyond 12, where it burns the whole timeout while the arm
        stands there holding the puck.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.get("CurrentStep") == step:
                return
            self.answer_measurement()
            time.sleep(0.05)
        raise SystemExit(f"timed out after {timeout:.0f}s waiting for {what or f'step {step}'}")

    def check_idle(self, expect_calib=0):
        """Refuse to trigger over a run in progress, or into the wrong mode.

        A non-zero `CurrentStep` is an interrupted run's resume point and
        triggering over it destroys that. A non-zero `StartStep` skips steps,
        and a calibration mode runs a different sequence entirely — in both
        cases the next trigger does something other than the cycle being
        measured.
        """
        for name, want, why in (
            ("CurrentStep", 0, "a run is in progress or interrupted"),
            ("CalibMode", expect_calib, "not in the expected mode"),
            ("StartStep", 0, "steps would be skipped"),
            ("Stop", 0, "the sequence is paused"),
        ):
            got = self.get(name)
            if got != want:
                raise SystemExit(
                    f"refusing to start: Robot:{name} is {got}, not {want} — {why}"
                )

    def wait_cycle_end(self, timeout):
        """Wait for the run to return to idle, answering the beamline on the way."""
        self.advance_to(0, timeout, "the cycle to finish")
