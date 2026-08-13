//! The trigger-driven sequence state machine, a faithful port of
//! `epics_triggered_sequence.cpp`'s main loop.
//!
//! Failure semantics are the resume design and must not be "improved":
//! any step failure exits the daemon, leaving `CurrentStep` at the last
//! completed step so the operator can resume via `StartStep` +
//! `Trigger` after clearing the fault (the IOC preserves the PVs; see
//! CLAUDE.md "충돌/크래시 후 재개"). Only a completed or skipped run
//! resets `CurrentStep` and `StartStep` to 0.

use std::time::Duration;

use crate::config::Config;
use crate::epics::{CalibMode, Epics, VisionKind, WaitStatus};
use crate::error::SequencerError;
use crate::gripper::Gripper;
use crate::log;
use crate::model::{JointMap, Model};
use crate::motion::Motion;
use crate::waypoints::WaypointData;

const POLL: Duration = Duration::from_millis(100);

/// Base waypoints recomputed from the YAML before every sequence.
/// The C++ also derived holder-above/retreat and sample-holder-retreat
/// bases here, but the sequence never read them (the per-trigger
/// above/retreat chain starts from the offset on-position instead);
/// those dead computations are dropped.
struct BaseWaypoints {
    holder_standby: JointMap,
    holder_on: JointMap,
    sample_holder_standby: JointMap,
    sample_holder_on: JointMap,
    sample_holder_above: JointMap,
}

/// The per-trigger waypoint set the step tables index into.
struct RunWaypoints {
    standby: JointMap,
    on_pos: JointMap,
    above: JointMap,
    retreat: JointMap,
    sh_standby: JointMap,
    sh_above: JointMap,
    sh_on_pos: JointMap,
}

pub struct Sequencer<'a> {
    epics: Epics,
    motion: Motion<'a>,
    gripper: Gripper,
    model: &'a Model,
    config: &'a Config,
    /// Last seen `Robot:Gripper` command; -1 = unknown (first successful
    /// read initializes without executing).
    last_gripper_cmd: i32,
    sequence_count: u32,
    /// Monotonic id for the vision handshake (`Robot:Vision:Req`/`Done`).
    vision_req_id: i32,
}

/// A measured correction gated against the configured deadband/limit.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Gate {
    /// Below the deadband: noise, skip the move.
    Below(f64),
    /// Within limits: apply this TCP-local correction (mm).
    Apply([f64; 3]),
    /// Over the limit: a mis-detection, wrong slot, or moved rack —
    /// never auto-applied.
    TooLarge(f64),
}

fn gate_correction(d: [f64; 3], min_mm: f64, max_mm: f64) -> Gate {
    let mag = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if mag < min_mm {
        Gate::Below(mag)
    } else if mag <= max_mm {
        Gate::Apply(d)
    } else {
        Gate::TooLarge(mag)
    }
}

impl<'a> Sequencer<'a> {
    pub fn new(
        epics: Epics,
        motion: Motion<'a>,
        gripper: Gripper,
        model: &'a Model,
        config: &'a Config,
    ) -> Self {
        // Seed the request ids above the persisted Done echo — see
        // `vision_last_done` for why starting at 0 aliases stale answers.
        let vision_req_id = epics.vision_last_done();
        Self {
            epics,
            motion,
            gripper,
            model,
            config,
            last_gripper_cmd: -1,
            sequence_count: 0,
            vision_req_id,
        }
    }

    /// The main trigger loop. Returns only on error (step failure or a
    /// structural fault), which exits the daemon with `CurrentStep`
    /// preserved.
    pub fn run(&mut self) -> Result<(), SequencerError> {
        loop {
            let start_from_step = self.wait_for_trigger(false);
            self.sequence_count += 1;

            let holder_number = self.epics.read_holder();
            let calib_mode = self.epics.read_calib_mode();
            let mode_name = match calib_mode {
                CalibMode::Holder => "Holder",
                CalibMode::SampleHolder => "SampleHolder",
                CalibMode::Normal => "Normal",
            };
            log::info("========================================");
            log::info(&format!(
                "Starting sequence #{} (from step {start_from_step}, holder {holder_number}, mode={mode_name})",
                self.sequence_count
            ));
            log::info("========================================");

            self.epics.write_wait(0);

            log::info("Reloading waypoints from YAML...");
            let waypoints = match WaypointData::load(&self.config.sequence.waypoints_yaml) {
                Ok(data) => data,
                Err(e) => {
                    log::error(&format!(
                        "Failed to reload waypoints, skipping sequence: {e}"
                    ));
                    continue;
                }
            };
            let base = self.compute_base_waypoints(&waypoints)?;
            let run = self.compute_run_waypoints(&waypoints, &base, holder_number)?;

            let outcome = match calib_mode {
                CalibMode::Holder => self.run_calib_holder(&run, start_from_step)?,
                CalibMode::SampleHolder => self.run_calib_sample_holder(&run, start_from_step)?,
                CalibMode::Normal => self.run_normal(&run, start_from_step)?,
            };

            log::info("========================================");
            match (calib_mode, outcome) {
                (CalibMode::Holder | CalibMode::SampleHolder, _) => log::info(&format!(
                    "Calibration sequence #{} completed ({mode_name} mode)",
                    self.sequence_count
                )),
                (CalibMode::Normal, true) => log::info(&format!(
                    "Sequence #{}: Steps 13-23 skipped (Wait PV = 2)",
                    self.sequence_count
                )),
                (CalibMode::Normal, false) => log::info(&format!(
                    "Sequence #{} completed successfully!",
                    self.sequence_count
                )),
            }
            log::info("========================================");

            self.epics.write_current_step(0);
            // StartStep is a one-shot resume override; a completed (or
            // skipped) run clears it so the next trigger starts from the
            // top. A failed run never gets here — the daemon has exited
            // with CurrentStep preserved.
            if self.epics.write_start_step(0) {
                log::info("Reset StartStep to 0 (next run starts from the beginning)");
            }
        }
    }

    /// Normal mode, steps 0-23 with the measurement wait after step 12.
    /// Returns whether steps 13-23 were skipped (`Wait` = 2).
    ///
    /// The vision hooks (all no-ops unless `vision.enabled`) hang off
    /// the four above→on descents: measure at the taught above pose,
    /// fold the gated correction into the descent and the matching
    /// ascent. Each `measure` condition requires that THIS run executed
    /// the above step — after a resume that skipped it the arm's pose
    /// is not the calibrated measurement pose, so no measurement is
    /// taken. Calibration modes get no hooks: they exist to measure the
    /// taught error, which a correction would mask.
    fn run_normal(&mut self, w: &RunWaypoints, start: i32) -> Result<bool, SequencerError> {
        self.hand(0, "open_hand", true, start)?;
        self.arm(1, "holder_standby", &w.standby, start)?;
        self.cartesian(2, "holder_above", &w.above, start)?;

        let d_pick = self.vision_correction(start <= 2, VisionKind::PickAlign, "pick@rack")?;
        let above_c = self.corrected(&w.above, d_pick, "holder_above+vision")?;
        let on_c = self.corrected(&w.on_pos, d_pick, "holder_on_position+vision")?;
        self.cartesian_via(
            3,
            "holder_on_position",
            d_pick.is_some().then_some(&above_c),
            &on_c,
            start,
        )?;
        self.hand(4, "close_gripper", false, start)?;
        self.cartesian(5, "holder_above_return", &above_c, start)?;
        let d_grip = self.vision_correction(start <= 5, VisionKind::GripOffset, "grip@rack")?;
        self.cartesian(6, "holder_retreat", &w.retreat, start)?;
        self.arm(7, "sample_holder_standby", &w.sh_standby, start)?;
        self.cartesian(8, "sample_holder_above", &w.sh_above, start)?;

        let d_slot =
            self.vision_correction(start <= 8, VisionKind::PlaceAlign, "place@sample_holder")?;
        let d_place = self.combine_corrections(d_slot, d_grip, "place@sample_holder")?;
        let sh_above_c = self.corrected(&w.sh_above, d_place, "sample_holder_above+vision")?;
        let sh_on_c = self.corrected(&w.sh_on_pos, d_place, "sample_holder_on+vision")?;
        self.cartesian_via(
            9,
            "sample_holder_on_position",
            d_place.is_some().then_some(&sh_above_c),
            &sh_on_c,
            start,
        )?;
        self.hand(10, "open_gripper", true, start)?;
        self.cartesian(11, "sample_holder_above_return", &sh_above_c, start)?;
        self.vision_seating_check(start <= 11, "seating@sample_holder")?;
        self.cartesian(12, "sample_holder_standby_return", &w.sh_standby, start)?;

        let mut skip_remaining = false;
        if start <= 12 {
            // Loaded=1 while the beamline measures the sample the arm just
            // delivered; the measurement program flips Wait to continue.
            self.epics.write_loaded(1);
            let wait_result = self.wait_for_measurement();
            self.epics.write_loaded(0);
            if wait_result == WaitStatus::Skip {
                log::info("Skip requested - skipping remaining steps (13-23)");
                skip_remaining = true;
            }
        }

        if !skip_remaining {
            self.cartesian(13, "sample_holder_above_2nd", &w.sh_above, start)?;
            let d_pick2 =
                self.vision_correction(start <= 13, VisionKind::PickAlign, "pick@sample_holder")?;
            let sh_above2_c = self.corrected(&w.sh_above, d_pick2, "sh_above_2nd+vision")?;
            let sh_on2_c = self.corrected(&w.sh_on_pos, d_pick2, "sh_on_2nd+vision")?;
            self.cartesian_via(
                14,
                "sample_holder_on_position_2nd",
                d_pick2.is_some().then_some(&sh_above2_c),
                &sh_on2_c,
                start,
            )?;
            self.hand(15, "close_gripper_2nd", false, start)?;
            self.cartesian(16, "sample_holder_above_2nd_return", &sh_above2_c, start)?;
            let d_grip2 =
                self.vision_correction(start <= 16, VisionKind::GripOffset, "grip@sample_holder")?;
            self.cartesian(17, "sample_holder_standby_2nd", &w.sh_standby, start)?;
            self.arm(18, "holder_standby_return", &w.standby, start)?;
            self.cartesian(19, "holder_above_final", &w.above, start)?;
            let d_slot2 =
                self.vision_correction(start <= 19, VisionKind::PlaceAlign, "place@rack")?;
            let d_place2 = self.combine_corrections(d_slot2, d_grip2, "place@rack")?;
            let above_f_c = self.corrected(&w.above, d_place2, "holder_above_final+vision")?;
            let on_f_c = self.corrected(&w.on_pos, d_place2, "holder_on_final+vision")?;
            self.cartesian_via(
                20,
                "holder_on_position_final",
                d_place2.is_some().then_some(&above_f_c),
                &on_f_c,
                start,
            )?;
            self.hand(21, "open_gripper_final", true, start)?;
            self.cartesian(22, "holder_above_final_return", &above_f_c, start)?;
            self.vision_seating_check(start <= 22, "seating@rack")?;
            self.cartesian(23, "holder_standby_final", &w.standby, start)?;
        }
        Ok(skip_remaining)
    }

    /// Holder calibration: pick and hold above the holder (0-5), let the
    /// operator jog, then return the sample (20-23) on the next trigger.
    fn run_calib_holder(&mut self, w: &RunWaypoints, start: i32) -> Result<bool, SequencerError> {
        log::info(">>> HOLDER CALIBRATION MODE: Steps 0-5, wait, 20-23 <<<");
        self.hand(0, "open_hand", true, start)?;
        self.arm(1, "holder_standby", &w.standby, start)?;
        self.cartesian(2, "holder_above", &w.above, start)?;
        self.cartesian(3, "holder_on_position", &w.on_pos, start)?;
        self.hand(4, "close_gripper", false, start)?;
        self.cartesian(5, "holder_above_return", &w.above, start)?;

        self.calibration_hold("HOLDER CALIBRATION: Holding at above position");

        log::info(">>> Returning sample to holder (steps 20-23) <<<");
        self.cartesian(20, "holder_on_position_final", &w.on_pos, start)?;
        self.hand(21, "open_gripper_final", true, start)?;
        self.cartesian(22, "holder_above_final_return", &w.above, start)?;
        self.cartesian(23, "holder_standby_final", &w.standby, start)?;
        Ok(false)
    }

    /// Sample-holder calibration: carry to the sample holder above pose
    /// (0-8), hold for jogging, then return everything (16-23).
    fn run_calib_sample_holder(
        &mut self,
        w: &RunWaypoints,
        start: i32,
    ) -> Result<bool, SequencerError> {
        log::info(">>> SAMPLE HOLDER CALIBRATION MODE: Steps 0-8, wait, 16-23 <<<");
        self.hand(0, "open_hand", true, start)?;
        self.arm(1, "holder_standby", &w.standby, start)?;
        self.cartesian(2, "holder_above", &w.above, start)?;
        self.cartesian(3, "holder_on_position", &w.on_pos, start)?;
        self.hand(4, "close_gripper", false, start)?;
        self.cartesian(5, "holder_above_return", &w.above, start)?;
        self.cartesian(6, "holder_retreat", &w.retreat, start)?;
        self.arm(7, "sample_holder_standby", &w.sh_standby, start)?;
        self.cartesian(8, "sample_holder_above", &w.sh_above, start)?;

        self.calibration_hold("SAMPLE HOLDER CALIBRATION: Holding at sample holder above");

        log::info(">>> Returning sample to holder (steps 16-23) <<<");
        self.cartesian(16, "sample_holder_above_2nd_return", &w.sh_above, start)?;
        self.cartesian(17, "sample_holder_standby_2nd", &w.sh_standby, start)?;
        self.arm(18, "holder_standby_return", &w.standby, start)?;
        self.cartesian(19, "holder_above_final", &w.above, start)?;
        self.cartesian(20, "holder_on_position_final", &w.on_pos, start)?;
        self.hand(21, "open_gripper_final", true, start)?;
        self.cartesian(22, "holder_above_final_return", &w.above, start)?;
        self.cartesian(23, "holder_standby_final", &w.standby, start)?;
        Ok(false)
    }

    fn calibration_hold(&mut self, banner: &str) {
        log::info("========================================");
        log::info(banner);
        log::info("  Use JogX/Y/Z PVs to adjust TCP position");
        log::info("  Set Trigger=1 to return sample");
        log::info("========================================");
        // The hold's StartStep read is discarded: the return phase keeps
        // the original trigger's start_from_step for its skip logic, as
        // the C++ did.
        let _ = self.wait_for_trigger(true);
    }

    // ---- vision correction ---------------------------------------------

    /// One correction measurement (the "look" of look-then-move).
    /// Returns the TCP-local correction in mm to fold into the descent,
    /// or `None` when vision/the hook is off, this run never reached the
    /// measurement pose (`measure` false — a resume skipped the above
    /// step, so a measurement would be taken from an unknown pose), the
    /// correction is below the deadband, or observe_only.
    ///
    /// Outside observe_only a timeout, an invalid verdict, and an
    /// over-limit correction are all errors: never guess, never
    /// auto-apply a large jump. In observe_only every failure is logged
    /// and swallowed — Phase C observation must not affect operation.
    fn vision_correction(
        &mut self,
        measure: bool,
        kind: VisionKind,
        label: &str,
    ) -> Result<Option<[f64; 3]>, SequencerError> {
        let v = &self.config.vision;
        let hook_on = match kind {
            VisionKind::PickAlign => v.pick_align,
            VisionKind::GripOffset => v.grip_offset,
            VisionKind::PlaceAlign => v.place_align,
            VisionKind::Seating => v.seating_check,
        };
        if !v.enabled || !hook_on || !measure {
            return Ok(None);
        }
        self.vision_req_id += 1;
        let answer =
            self.epics
                .vision_request(kind, self.vision_req_id, Duration::from_secs_f64(v.timeout));
        if v.observe_only {
            match answer {
                Ok(r) => log::info(&format!(
                    "{label}: vision observed dx={:.3} dy={:.3} dz={:.3} mm, valid={}, quality={:.2} (observe_only, not applied)",
                    r.dx, r.dy, r.dz, r.valid as i32, r.quality
                )),
                Err(e) => log::warn(&format!("{label}: vision observation failed: {e}")),
            }
            return Ok(None);
        }
        let r = answer?;
        if !r.valid {
            return Err(SequencerError(format!(
                "{label}: vision verdict UNKNOWN (quality {:.2})",
                r.quality
            )));
        }
        match gate_correction([r.dx, r.dy, r.dz], v.min_correction, v.max_correction) {
            Gate::Below(mag) => {
                log::info(&format!(
                    "{label}: vision correction {mag:.3} mm below deadband, skipping"
                ));
                Ok(None)
            }
            Gate::Apply(d) => {
                log::info(&format!(
                    "{label}: vision correction dx={:.3} dy={:.3} dz={:.3} mm (quality {:.2})",
                    d[0], d[1], d[2], r.quality
                ));
                Ok(Some(d))
            }
            Gate::TooLarge(mag) => Err(SequencerError(format!(
                "{label}: vision correction {mag:.2} mm exceeds the {:.2} mm limit — not applied",
                v.max_correction
            ))),
        }
    }

    /// Post-place seating verdict. Errors stop the sequence before the
    /// arm leaves a badly seated puck behind; observe_only only logs.
    fn vision_seating_check(&mut self, measure: bool, label: &str) -> Result<(), SequencerError> {
        let v = &self.config.vision;
        if !v.enabled || !v.seating_check || !measure {
            return Ok(());
        }
        self.vision_req_id += 1;
        let answer = self.epics.vision_request(
            VisionKind::Seating,
            self.vision_req_id,
            Duration::from_secs_f64(v.timeout),
        );
        if v.observe_only {
            match answer {
                Ok(r) => log::info(&format!(
                    "{label}: vision observed seated={}, tilt={:.2} deg, valid={} (observe_only)",
                    r.seated as i32, r.tilt, r.valid as i32
                )),
                Err(e) => log::warn(&format!("{label}: vision observation failed: {e}")),
            }
            return Ok(());
        }
        let r = answer?;
        if !r.valid {
            return Err(SequencerError(format!(
                "{label}: seating verdict UNKNOWN (quality {:.2})",
                r.quality
            )));
        }
        if !r.seated {
            return Err(SequencerError(format!(
                "{label}: puck NOT seated (tilt {:.2} deg, quality {:.2})",
                r.tilt, r.quality
            )));
        }
        log::info(&format!("{label}: seated (tilt {:.2} deg)", r.tilt));
        Ok(())
    }

    /// Slot correction plus the stored grip offset. Each factor passed
    /// its own gate when measured; the sum is re-gated because it can
    /// still exceed the limit.
    fn combine_corrections(
        &self,
        slot: Option<[f64; 3]>,
        grip: Option<[f64; 3]>,
        label: &str,
    ) -> Result<Option<[f64; 3]>, SequencerError> {
        let sum = match (slot, grip) {
            (None, None) => return Ok(None),
            (Some(a), None) => a,
            (None, Some(b)) => b,
            (Some(a), Some(b)) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
        };
        let v = &self.config.vision;
        match gate_correction(sum, v.min_correction, v.max_correction) {
            Gate::Below(_) => Ok(None),
            Gate::Apply(d) => {
                if slot.is_some() && grip.is_some() {
                    log::info(&format!(
                        "{label}: applying slot+grip correction dx={:.3} dy={:.3} dz={:.3} mm",
                        d[0], d[1], d[2]
                    ));
                }
                Ok(Some(d))
            }
            Gate::TooLarge(mag) => Err(SequencerError(format!(
                "{label}: combined vision correction {mag:.2} mm exceeds the {:.2} mm limit",
                v.max_correction
            ))),
        }
    }

    /// `base` shifted by a vision correction (mm, TCP-local), with
    /// `apply_cartesian_offset`'s fallback-to-original escape hatch
    /// closed: a correction the arm cannot realize must stop the
    /// sequence, not silently descend uncorrected.
    fn corrected(
        &self,
        base: &JointMap,
        d: Option<[f64; 3]>,
        label: &str,
    ) -> Result<JointMap, SequencerError> {
        let Some(d) = d else {
            return Ok(base.clone());
        };
        let offset = [d[0] / 1000.0, d[1] / 1000.0, d[2] / 1000.0];
        let shifted = self
            .model
            .apply_cartesian_offset(base, offset, false, label)?;
        if shifted == *base {
            return Err(SequencerError(format!(
                "{label}: IK cannot realize the vision correction"
            )));
        }
        Ok(shifted)
    }

    // ---- step executors ------------------------------------------------

    /// Shared prologue: resume-skip, then block while `Stop` is set.
    /// Returns false when the step is skipped.
    fn step_prologue(&mut self, step: i32, name: &str, start: i32) -> bool {
        if step < start {
            log::info(&format!("Skipping step {step} ({name})"));
            return false;
        }
        self.wait_for_stop_clear();
        log::info(&format!("Step {step}: {name}"));
        true
    }

    /// Shared epilogue after a successful step: publish `CurrentStep`,
    /// then honor a matching `PauseStep`.
    fn step_epilogue(&mut self, step: i32) {
        self.epics.write_current_step(step);
        self.wait_for_pause_step_change(step);
    }

    fn arm(
        &mut self,
        step: i32,
        name: &str,
        goal: &JointMap,
        start: i32,
    ) -> Result<(), SequencerError> {
        if !self.step_prologue(step, name, start) {
            return Ok(());
        }
        self.motion.move_planned(
            goal,
            self.config.sequence.velocity_scale,
            self.config.sequence.acceleration_scale,
            name,
        )?;
        log::info("  -> Completed");
        self.step_epilogue(step);
        Ok(())
    }

    fn cartesian(
        &mut self,
        step: i32,
        name: &str,
        goal: &JointMap,
        start: i32,
    ) -> Result<(), SequencerError> {
        self.cartesian_via(step, name, None, goal, start)
    }

    /// A Cartesian step with an optional pre-alignment: an active vision
    /// correction first aligns laterally at the corrected above pose so
    /// the descent stays vertical, then descends — both inside the one
    /// operator-visible step number (no renumbering, resume-compatible).
    fn cartesian_via(
        &mut self,
        step: i32,
        name: &str,
        via: Option<&JointMap>,
        goal: &JointMap,
        start: i32,
    ) -> Result<(), SequencerError> {
        if !self.step_prologue(step, name, start) {
            return Ok(());
        }
        if let Some(via) = via {
            self.motion.move_cartesian(
                via,
                self.config.sequence.velocity_scale,
                self.config.sequence.acceleration_scale,
                name,
            )?;
        }
        self.motion.move_cartesian(
            goal,
            self.config.sequence.velocity_scale,
            self.config.sequence.acceleration_scale,
            name,
        )?;
        log::info("  -> Completed (Cartesian)");
        self.step_epilogue(step);
        Ok(())
    }

    fn hand(
        &mut self,
        step: i32,
        name: &str,
        open: bool,
        start: i32,
    ) -> Result<(), SequencerError> {
        if !self.step_prologue(step, name, start) {
            return Ok(());
        }
        self.gripper.command(open);
        self.gripper.wait_reached(open, &self.epics);
        log::info("  -> Completed");
        self.step_epilogue(step);
        Ok(())
    }

    // ---- PV wait loops -------------------------------------------------

    /// Blocks until `Trigger` goes non-zero, resets it, and returns the
    /// `StartStep` value. The idle loop services gripper commands (and
    /// jogs, during calibration holds) exactly like the C++ idle
    /// callbacks. RTDE output is paused for the duration — the wait is
    /// unbounded and nobody reads the stream.
    fn wait_for_trigger(&mut self, allow_jog: bool) -> i32 {
        log::info("========================================");
        log::info("Waiting for EPICS trigger...");
        log::info("========================================");
        loop {
            let value = self.epics.read_trigger();
            if value > 0 {
                let start_step = self.epics.read_start_step();
                log::info(&format!(
                    "Trigger received! Trigger={value}, StartStep={start_step}"
                ));
                if !self.epics.write_trigger(0) {
                    log::warn("Failed to reset trigger PV to 0, continuing anyway...");
                }
                return start_step;
            }
            if value < 0 {
                log::warn("Error reading trigger PV, retrying...");
            }
            self.poll_gripper_cmd();
            if allow_jog {
                self.process_jog();
            }
            self.gripper.update_rbv(&self.epics);
            std::thread::sleep(POLL);
        }
    }

    fn wait_for_stop_clear(&mut self) {
        if self.epics.read_stop() == 0 {
            return;
        }
        log::info("STOPPED - Waiting for Stop PV to become 0...");
        loop {
            if self.epics.read_stop() == 0 {
                log::info("Stop cleared, resuming execution...");
                return;
            }
            std::thread::sleep(POLL);
        }
    }

    fn wait_for_pause_step_change(&mut self, current_step: i32) {
        let pause_step = self.epics.read_pause_step();
        if pause_step == 0 || pause_step != current_step {
            return;
        }
        log::info(&format!(
            "PAUSED at step {current_step} - Waiting for PauseStep to change..."
        ));
        loop {
            let pause_step = self.epics.read_pause_step();
            if pause_step != current_step {
                log::info(&format!(
                    "PauseStep changed to {pause_step}, resuming execution..."
                ));
                return;
            }
            std::thread::sleep(POLL);
        }
    }

    fn wait_for_measurement(&mut self) -> WaitStatus {
        log::info("========================================");
        log::info("Waiting for measurement to complete...");
        log::info("  Wait PV: 0=keep waiting, 1=continue, 2=skip remaining");
        log::info("========================================");
        loop {
            match self.epics.read_wait() {
                WaitStatus::Continue => {
                    log::info("Measurement complete, continuing...");
                    return WaitStatus::Continue;
                }
                WaitStatus::Skip => {
                    log::info("Skip requested, aborting remaining steps...");
                    return WaitStatus::Skip;
                }
                WaitStatus::Waiting => std::thread::sleep(POLL),
            }
        }
    }

    // ---- idle services -------------------------------------------------

    /// Reads the `Gripper` command PV and executes it on change — the C++
    /// pending-command mechanism, collapsed: this is only called from
    /// idle/hold loops, which is exactly where the C++ executed the
    /// pending command.
    fn poll_gripper_cmd(&mut self) {
        let cmd = self.epics.read_gripper_cmd();
        if cmd < 0 {
            return;
        }
        if self.last_gripper_cmd < 0 {
            self.last_gripper_cmd = cmd;
            log::info(&format!(
                "Gripper command PV initialized: {cmd} ({})",
                if cmd == 1 { "OPEN" } else { "CLOSE" }
            ));
            return;
        }
        if cmd != self.last_gripper_cmd {
            log::info(&format!(
                "Gripper command PV changed: {} -> {cmd} ({})",
                self.last_gripper_cmd,
                if cmd == 1 { "OPEN" } else { "CLOSE" }
            ));
            self.last_gripper_cmd = cmd;
            let open = cmd == 1;
            self.gripper.command(open);
            self.gripper.wait_reached(open, &self.epics);
        }
    }

    /// Services one jog request during a calibration hold. Jog failure is
    /// logged and swallowed (the hold continues), as in the C++.
    fn process_jog(&mut self) {
        let (jog_x, jog_y, jog_z) = self.epics.read_jog_request();
        if jog_x == 0 && jog_y == 0 && jog_z == 0 {
            return;
        }
        let step_mm = self.epics.read_jog_step_mm();
        let result = self.motion.jog(
            f64::from(jog_x) * step_mm,
            f64::from(jog_y) * step_mm,
            f64::from(jog_z) * step_mm,
            self.config.sequence.jog_velocity_scale,
        );
        match result {
            Ok(()) => log::info("TCP Jog completed successfully"),
            Err(e) => log::error(&format!("TCP Jog failed: {e}")),
        }
        // Back to the hold loop's no-reader state.
    }

    // ---- waypoint computation ------------------------------------------

    fn compute_base_waypoints(&self, w: &WaypointData) -> Result<BaseWaypoints, SequencerError> {
        let taught =
            |values: &[f64]| -> JointMap { WaypointData::arm_joints(values).into_iter().collect() };
        let holder_offsets = [
            w.holder1_on_x_offset,
            w.holder1_on_y_offset,
            w.holder1_on_z_offset,
        ];
        let sh_offsets = [
            w.sample_holder_on_x_offset,
            w.sample_holder_on_y_offset,
            w.sample_holder_on_z_offset,
        ];

        let holder_standby = self.model.apply_cartesian_offset(
            &taught(&w.holder1_standby),
            holder_offsets,
            false,
            "holder1_standby",
        )?;
        let holder_on = self.model.apply_cartesian_offset(
            &taught(&w.holder1_on_position),
            holder_offsets,
            false,
            "holder1_on_position",
        )?;
        let sample_holder_standby = taught(&w.sample_holder_standby);
        let sample_holder_on = self.model.apply_cartesian_offset(
            &taught(&w.sample_holder_on_position),
            sh_offsets,
            false,
            "sample_holder_on_position",
        )?;
        let sample_holder_above = self.model.apply_cartesian_offset(
            &sample_holder_on,
            [0.0, w.above_y_offset, 0.0],
            false,
            "sample_holder_above",
        )?;
        log::info("Waypoints calculated successfully");
        Ok(BaseWaypoints {
            holder_standby,
            holder_on,
            sample_holder_standby,
            sample_holder_on,
            sample_holder_above,
        })
    }

    fn compute_run_waypoints(
        &self,
        w: &WaypointData,
        base: &BaseWaypoints,
        holder_number: i32,
    ) -> Result<RunWaypoints, SequencerError> {
        let y_offset = f64::from(holder_number - 1) * self.config.sequence.holder_offset;
        let mut x_offset = 0.0;
        let mut z_offset = 0.0;
        if (2..=10).contains(&holder_number) {
            let idx = (holder_number - 2) as usize;
            if let Some(x) = w.holder_multi_x_offsets.get(idx) {
                x_offset = *x;
            }
            if let Some(z) = w.holder_multi_z_offsets.get(idx) {
                z_offset = *z;
            }
        }

        let wrist3 = w.wrist3_rotation_offset;
        let apply_wrist3 = |mut joints: JointMap| -> JointMap {
            if wrist3.abs() > 1e-6
                && let Some(value) = joints.get_mut("wrist_3_joint")
            {
                *value += wrist3;
            }
            joints
        };
        if wrist3.abs() > 1e-6 {
            log::info(&format!(
                "  Applying wrist_3_joint offset: {wrist3:.4} rad ({:.2} deg)",
                wrist3.to_degrees()
            ));
        }

        // Holder 10 sits at the rail end; its standby gets an extra -5 mm
        // y before the shared multi-holder offset.
        let temp_standby = if holder_number == 10 {
            self.model.apply_cartesian_offset(
                &base.holder_standby,
                [0.0, -0.005, 0.0],
                false,
                "holder10_standby",
            )?
        } else {
            base.holder_standby.clone()
        };

        // NOTE (preserved quirk): each chained offset below re-applies the
        // wrist3 offset on top of the previous result, so `above` carries
        // it twice and `retreat` three times. Production keeps
        // wrist3_rotation_offset at 0.0, where this is a no-op.
        let standby = apply_wrist3(self.model.apply_cartesian_offset(
            &temp_standby,
            [x_offset, y_offset, z_offset],
            false,
            "standby",
        )?);
        let on_pos = apply_wrist3(self.model.apply_cartesian_offset(
            &base.holder_on,
            [x_offset, y_offset, z_offset],
            false,
            "on_position",
        )?);
        let above = apply_wrist3(self.model.apply_cartesian_offset(
            &on_pos,
            [0.0, w.above_y_offset, 0.0],
            false,
            "above",
        )?);
        let retreat = apply_wrist3(self.model.apply_cartesian_offset(
            &above,
            [0.0, 0.0, w.retreat_z_offset],
            false,
            "retreat",
        )?);

        Ok(RunWaypoints {
            standby,
            on_pos,
            above,
            retreat,
            sh_standby: base.sample_holder_standby.clone(),
            sh_above: base.sample_holder_above.clone(),
            sh_on_pos: base.sample_holder_on.clone(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Boundary cases of the correction gate: `< min` is noise, `[min,
    /// max]` applies, `> max` refuses. The measure is the 3-D norm, not
    /// a per-axis check.
    #[test]
    fn gate_boundaries() {
        let (min, max) = (0.05, 3.0);
        assert!(matches!(
            gate_correction([0.04, 0.0, 0.0], min, max),
            Gate::Below(_)
        ));
        assert!(matches!(
            gate_correction([0.05, 0.0, 0.0], min, max),
            Gate::Apply(_)
        ));
        assert_eq!(
            gate_correction([0.6, -0.8, 0.0], min, max),
            Gate::Apply([0.6, -0.8, 0.0])
        );
        assert!(matches!(
            gate_correction([3.0, 0.0, 0.0], min, max),
            Gate::Apply(_)
        ));
        assert!(matches!(
            gate_correction([3.01, 0.0, 0.0], min, max),
            Gate::TooLarge(_)
        ));
        // Per-axis components under the limit whose norm exceeds it
        // must still refuse: 1.9-2.4-1.0 has norm ~3.22.
        assert!(matches!(
            gate_correction([1.9, 2.4, 1.0], min, max),
            Gate::TooLarge(_)
        ));
    }
}
