//! The trigger-driven sequence state machine, a faithful port of
//! `epics_triggered_sequence.cpp`'s main loop.
//!
//! Failure semantics are the resume design. A step failure stops the run
//! and leaves `CurrentStep` at the last completed step, so the operator
//! can resume via `StartStep` + `Trigger` after clearing the fault (the
//! IOC preserves the PVs; see CLAUDE.md "충돌/크래시 후 재개"). What it
//! must not do is end the daemon: the process leaving takes the RTDE
//! stream and the gripper's activation with it, and the next start opens
//! the fingers on whatever was being held. `CalibMode = 4` walks the arm
//! back to standby instead. Only a completed or skipped run resets
//! `CurrentStep` and `StartStep` to 0 — a recovery resets neither.

use std::time::Duration;

use cspace_core::geometry::Vector3;

use crate::config::Config;
use crate::epics::{CalibMode, Epics, VisionKind, WaitStatus};
use crate::error::SequencerError;
use crate::gripper::Gripper;
use crate::handeye;
use crate::log;
use crate::model::{JointMap, Model};
use crate::motion::{
    Bracket, Centring, MIN_EXECUTABLE_MM, Motion, ProbeLimits, Probed, TiltLimits, Tilted,
};
use crate::waypoints::WaypointData;

const POLL: Duration = Duration::from_millis(100);
/// How often the hand-eye aiming hold asks the detector where the tag is.
/// A detect costs one fresh camera frame; often enough to jog against,
/// rare enough that the jog itself stays responsive.
const HANDEYE_AIM_PROBE: Duration = Duration::from_secs(1);
/// Tool-frame step used to measure the image Jacobian before the frame
/// sweep. Big enough that the detector's ~0.1 px repeatability is
/// negligible in the fit, small enough that the arm stays in the
/// neighbourhood the aiming pose was already cleared in.
const HANDEYE_PROBE_M: f64 = 0.020;

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

/// One vision measurement: how far the target sits from where the taught
/// waypoint puts it, in mm.
///
/// The three numbers are meaningless without the pose they were measured
/// from. The camera rides the tool, so the vision node can only answer in
/// the tool frame of wherever the arm was standing — and since the hooks
/// observe at a standby pose and correct an above pose, that is no longer
/// the frame being corrected. Carrying the observation frame here is what
/// stops the two from being confused; the numbers alone cannot say which
/// they belong to.
#[derive(Debug, Clone, Copy)]
struct Correction {
    /// mm, in `frame`'s tool frame.
    d: [f64; 3],
    /// Tool orientation at the observation pose, in the model frame.
    frame: nalgebra::UnitQuaternion<f64>,
}

impl Correction {
    /// The same physical displacement, in `target`'s tool frame.
    fn in_frame(&self, target: &nalgebra::UnitQuaternion<f64>) -> [f64; 3] {
        let world = self.frame * nalgebra::Vector3::new(self.d[0], self.d[1], self.d[2]);
        let local = target.inverse_transform_vector(&world);
        [local.x, local.y, local.z]
    }
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

/// One height's worth of seat probing: where the arm stood, and what it
/// found from there.
/// The three base directions the sweep moves along, expressed in the
/// tool frame of the pose the mode was triggered at.
///
/// Taken once, at that pose, because every move here is a pure
/// translation: the tool orientation does not change, so the axes do not
/// either, and one set for the whole run is what makes `from_trigger`
/// add up.
/// The three limits the seat probe works under: measuring sideways,
/// measuring down, and moving between heights.
///
/// One value rather than three arguments because every step of the sweep
/// needs all three, and which one applies is a property of what the arm
/// is doing, not of who is calling.
#[derive(Clone, Copy)]
struct Limits {
    lateral: ProbeLimits,
    depth: ProbeLimits,
    lift: ProbeLimits,
    tilt: TiltLimits,
}

struct Axes {
    x: Vector3,
    y: Vector3,
    up: Vector3,
}

impl Axes {
    fn in_tool(motion: &mut Motion<'_>) -> Result<Self, SequencerError> {
        Ok(Self {
            x: motion.base_dir_in_tool(&Vector3::x())?,
            y: motion.base_dir_in_tool(&Vector3::y())?,
            up: motion.base_dir_in_tool(&Vector3::z())?,
        })
    }
}

struct Level {
    /// Height above the pose the mode was triggered at, mm.
    height_mm: f64,
    /// What the arm was still feeling when it arrived here, base N, and
    /// `None` at the level it was triggered at, which it did not climb
    /// to. This is what says whether the brackets below it measured the
    /// hole or the arm being leant on: in free air the same climb
    /// carries 0.14 N (doc §16.13).
    load: Option<Vector3>,
    brackets: Vec<Bracket>,
    /// `None` when the level did not get as far as the floor.
    floor: Option<Probed>,
    /// One per axis turned, empty when no tilt was asked for.
    tilts: Vec<Tilted>,
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

/// Blocks while `Stop` is set.
///
/// A free function over `&Epics` rather than a `&mut self` method so the
/// hand-eye capture, which holds a mutable borrow of `motion` across its
/// whole loop, can honour the same pause the sequence steps do instead of
/// keeping a second copy of this loop.
fn wait_for_stop_clear(epics: &Epics) {
    if epics.read_stop() == 0 {
        return;
    }
    log::info("STOPPED - Waiting for Stop PV to become 0...");
    loop {
        if epics.read_stop() == 0 {
            log::info("Stop cleared, resuming execution...");
            return;
        }
        std::thread::sleep(POLL);
    }
}

/// A list for the log, or `-` when it is empty.
fn or_dash(items: &[String]) -> String {
    if items.is_empty() {
        "-".to_string()
    } else {
        items.join(" ")
    }
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

    /// The main trigger loop. A failed run is logged and waited out, not
    /// returned; the `Result` is for faults that leave nothing to wait
    /// for.
    pub fn run(&mut self) -> Result<(), SequencerError> {
        loop {
            let start_from_step = self.wait_for_trigger(false);
            self.sequence_count += 1;

            let holder_number = self.epics.read_holder();
            let calib_mode = self.epics.read_calib_mode();
            let mode_name = match calib_mode {
                CalibMode::Holder => "Holder",
                CalibMode::SampleHolder => "SampleHolder",
                CalibMode::HandEye => "HandEye",
                CalibMode::SeatProbe => "SeatProbe",
                CalibMode::HolderMap => "HolderMap",
                CalibMode::Recover => "Recover",
                CalibMode::Normal => "Normal",
            };
            log::info("========================================");
            log::info(&format!(
                "Starting sequence #{} (from step {start_from_step}, holder {holder_number}, mode={mode_name})",
                self.sequence_count
            ));
            log::info("========================================");

            self.epics.write_wait(0);

            // A protective stop, pendant stop, or freedrive ends the
            // external-control program, and nothing moves until it is
            // sent again. Heal that here, before any step runs. The
            // unlock is gated on the Recover trigger: releasing a
            // protective stop is the operator's decision, and Recover
            // is how they say it.
            if let Err(e) = self
                .motion
                .ensure_program(matches!(calib_mode, CalibMode::Recover))
            {
                log::error(&format!(
                    "Sequence #{} not started: {e}",
                    self.sequence_count
                ));
                log::error("Nothing moved; CurrentStep and StartStep kept.");
                continue;
            }

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
            let attempt = self.compute_base_waypoints(&waypoints).and_then(|base| {
                match calib_mode {
                    // Needs waypoints for two holders (source and target),
                    // so it owns its own compute_run_waypoints calls.
                    CalibMode::HolderMap => {
                        self.run_holder_map(&waypoints, &base, holder_number, start_from_step)
                    }
                    _ => self
                        .compute_run_waypoints(&waypoints, &base, holder_number)
                        .and_then(|run| match calib_mode {
                            CalibMode::Holder => self.run_calib_holder(&run, start_from_step),
                            CalibMode::SampleHolder => {
                                self.run_calib_sample_holder(&run, start_from_step)
                            }
                            CalibMode::HandEye => self.run_handeye(),
                            CalibMode::SeatProbe => self.run_seat_probe(),
                            CalibMode::Recover => self.run_recover(&run),
                            CalibMode::Normal => self.run_normal(&run, start_from_step),
                            CalibMode::HolderMap => unreachable!("dispatched above"),
                        }),
                }
            });

            // A failed step used to end the process. The arm is stopped
            // either way, but exiting drops the RTDE stream and runs the
            // Hand-E driver's shutdown, which deactivates the gripper —
            // and a gripper that comes up deactivated takes its
            // calibration stroke on the next start, opening the fingers
            // on whatever they were holding. So the failure stops here:
            // the daemon keeps the stream, keeps the gripper exactly as
            // the failure left it, and waits. `CurrentStep` is left
            // alone, which is the resume point the invariant promises.
            let outcome = match attempt {
                Ok(outcome) => outcome,
                Err(e) => {
                    log::error(&format!("Sequence #{} failed: {e}", self.sequence_count));
                    log::error(
                        "Arm stopped, gripper untouched, CurrentStep kept as the resume point. \
                         Set StartStep and trigger to resume, or CalibMode=4 to return to standby.",
                    );
                    continue;
                }
            };

            log::info("========================================");
            match (calib_mode, outcome) {
                (CalibMode::Holder | CalibMode::SampleHolder | CalibMode::HandEye, _) => {
                    log::info(&format!(
                        "Calibration sequence #{} completed ({mode_name} mode)",
                        self.sequence_count
                    ))
                }
                (CalibMode::Normal, true) => log::info(&format!(
                    "Sequence #{}: Steps 13-23 skipped (Wait PV = 2)",
                    self.sequence_count
                )),
                (CalibMode::Normal, false) => log::info(&format!(
                    "Sequence #{} completed successfully!",
                    self.sequence_count
                )),
                (CalibMode::Recover, _) => log::info("Arm returned to holder standby"),
                (CalibMode::SeatProbe, _) => log::info("Seat probe finished; nothing written"),
                (CalibMode::HolderMap, _) => log::info(&format!(
                    "Holder map finished for holder {holder_number}; nothing written"
                )),
            }
            log::info("========================================");

            // These two moved the arm; neither finished the run that
            // stopped. Zeroing here would erase the step the operator
            // still has to act on, and it would claim idle while the
            // gripper may still be holding a sample, which is the one
            // state where `CurrentStep = 0` is a lie worth avoiding —
            // and the seat probe is entered *because* the gripper is
            // holding one.
            if matches!(calib_mode, CalibMode::Recover | CalibMode::SeatProbe) {
                log::info(
                    "CurrentStep left as it was: the interrupted run is still the resume \
                     point, and the gripper still holds whatever it held.",
                );
                continue;
            }

            self.epics.write_current_step(0);
            // StartStep is a one-shot resume override; a completed (or
            // skipped) run clears it so the next trigger starts from the
            // top. A failed run never gets here — it took the `continue`
            // above, leaving both PVs as the resume point.
            if self.epics.write_start_step(0) {
                log::info("Reset StartStep to 0 (next run starts from the beginning)");
            }
        }
    }

    /// Normal mode, steps 0-23 with the measurement wait after step 12.
    /// Returns whether steps 13-23 were skipped (`Wait` = 2).
    ///
    /// The vision hooks (all no-ops unless `vision.enabled`) observe from
    /// the standby poses, not from above. Above is where the correction
    /// is applied and it was where the measurement used to be taken, but
    /// the target is not in the picture there: the grasp point projects
    /// 55 rows below a 480-row frame and what fills the middle is the
    /// next holder up (doc/vision_correction_plan.md §12.4). From
    /// standby the same point sits at (306, 330), 11 deg off axis, and
    /// the sample-holder seat at (313, 277) — both near the centre.
    ///
    /// Applying at a different pose than it was measured from is why a
    /// correction travels as a [`Correction`] with its frame attached.
    /// It also removes the lateral via: the shift now happens on the way
    /// to above, so every descent is straight down again.
    ///
    /// Each `measure` condition requires that THIS run executed the step
    /// that parks the arm at the observation pose — after a resume that
    /// skipped it the arm is somewhere else, so no measurement is taken.
    /// Calibration modes get no hooks: they exist to measure the taught
    /// error, which a correction would mask.
    fn run_normal(&mut self, w: &RunWaypoints, start: i32) -> Result<bool, SequencerError> {
        self.hand(0, "open_hand", true, start)?;
        self.arm(1, "holder_standby", &w.standby, start)?;

        let d_pick =
            self.vision_correction(start <= 1, &w.standby, VisionKind::PickAlign, "pick@rack")?;
        let above_c = self.corrected(&w.above, d_pick, "holder_above+vision")?;
        let on_c = self.corrected(&w.on_pos, d_pick, "holder_on_position+vision")?;
        self.cartesian(2, "holder_above", &above_c, start)?;
        self.cartesian(3, "holder_on_position", &on_c, start)?;
        self.hand(4, "close_gripper", false, start)?;
        self.cartesian(5, "holder_above_return", &above_c, start)?;
        let d_grip =
            self.vision_correction(start <= 5, &above_c, VisionKind::GripOffset, "grip@rack")?;
        self.cartesian(6, "holder_retreat", &w.retreat, start)?;
        self.arm(7, "sample_holder_standby", &w.sh_standby, start)?;

        let d_slot = self.vision_correction(
            start <= 7,
            &w.sh_standby,
            VisionKind::PlaceAlign,
            "place@sample_holder",
        )?;
        let d_place =
            self.combine_corrections(d_slot, d_grip, &w.sh_on_pos, "place@sample_holder")?;
        let sh_above_c = self.corrected(&w.sh_above, d_place, "sample_holder_above+vision")?;
        let sh_on_c = self.corrected(&w.sh_on_pos, d_place, "sample_holder_on+vision")?;
        self.cartesian(8, "sample_holder_above", &sh_above_c, start)?;
        self.cartesian(9, "sample_holder_on_position", &sh_on_c, start)?;
        self.hand(10, "open_gripper", true, start)?;
        self.cartesian(11, "sample_holder_above_return", &sh_above_c, start)?;
        self.cartesian(12, "sample_holder_standby_return", &w.sh_standby, start)?;
        self.vision_seating_check(start <= 12, "seating@sample_holder")?;

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
            // The arm has stood at sh_standby since step 12, through the
            // measurement wait, so that is the pose this observation is
            // taken from — the wait does not move it.
            let d_pick2 = self.vision_correction(
                start <= 12,
                &w.sh_standby,
                VisionKind::PickAlign,
                "pick@sample_holder",
            )?;
            let sh_above2_c = self.corrected(&w.sh_above, d_pick2, "sh_above_2nd+vision")?;
            let sh_on2_c = self.corrected(&w.sh_on_pos, d_pick2, "sh_on_2nd+vision")?;
            self.cartesian(13, "sample_holder_above_2nd", &sh_above2_c, start)?;
            self.cartesian(14, "sample_holder_on_position_2nd", &sh_on2_c, start)?;
            self.hand(15, "close_gripper_2nd", false, start)?;
            self.cartesian(16, "sample_holder_above_2nd_return", &sh_above2_c, start)?;
            let d_grip2 = self.vision_correction(
                start <= 16,
                &sh_above2_c,
                VisionKind::GripOffset,
                "grip@sample_holder",
            )?;
            self.cartesian(17, "sample_holder_standby_2nd", &w.sh_standby, start)?;
            self.arm(18, "holder_standby_return", &w.standby, start)?;

            let d_slot2 = self.vision_correction(
                start <= 18,
                &w.standby,
                VisionKind::PlaceAlign,
                "place@rack",
            )?;
            let d_place2 = self.combine_corrections(d_slot2, d_grip2, &w.on_pos, "place@rack")?;
            let above_f_c = self.corrected(&w.above, d_place2, "holder_above_final+vision")?;
            let on_f_c = self.corrected(&w.on_pos, d_place2, "holder_on_final+vision")?;
            self.cartesian(19, "holder_above_final", &above_f_c, start)?;
            self.cartesian(20, "holder_on_position_final", &on_f_c, start)?;
            self.hand(21, "open_gripper_final", true, start)?;
            self.cartesian(22, "holder_above_final_return", &above_f_c, start)?;
            self.cartesian(23, "holder_standby_final", &w.standby, start)?;
            self.vision_seating_check(start <= 23, "seating@rack")?;
        }
        Ok(skip_remaining)
    }

    /// Puts the arm back at the holder standby after a run stopped part
    /// way, without ending the daemon and without touching the gripper.
    ///
    /// Two routes, tried in order, because the planner cannot start from
    /// every pose the sequence can stop in. `holder_above` is the case
    /// that forced this: RRT rejects it as an invalid start state, while
    /// the retreat from it is a move step 6 makes every cycle. So plan to
    /// standby, and when the planner refuses, retreat first — to a pose
    /// steps 7 and 18 already plan from — and try again from there.
    ///
    /// The retreat leg goes through `move_cartesian`, the same call step
    /// 6 makes, and not `move_direct`. The two disagree here and the
    /// robot settled it: interpolating the TCP pose clears this span at
    /// 100%, while the joint-space line between the same two poses is
    /// read as blocked. Taking the shorter-looking primitive left the
    /// arm stranded at `above` with recovery refusing to run.
    ///
    /// The gripper is deliberately left alone. Opening it here would
    /// drop the very sample this exists to protect, and where that
    /// sample should go is a decision for the operator, not for a
    /// recovery move.
    fn run_recover(&mut self, w: &RunWaypoints) -> Result<bool, SequencerError> {
        log::info(">>> RECOVER MODE: returning to holder standby <<<");
        let v = self.config.sequence.velocity_scale;
        let a = self.config.sequence.acceleration_scale;
        match self
            .motion
            .move_planned(&w.standby, v, a, "recover_standby")
        {
            Ok(()) => return Ok(false),
            Err(e) => log::warn(&format!(
                "recover: cannot plan to standby from here ({e}); \
                 trying by way of holder_retreat"
            )),
        }
        self.motion
            .move_cartesian(&w.retreat, v, a, "recover_retreat")?;
        self.motion
            .move_planned(&w.standby, v, a, "recover_standby")
            .map(|()| false)
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

    /// Hand-eye calibration capture: rotate the tool in place about each
    /// of its own axes, read the wrist camera at each stop, and write the
    /// (robot pose, tag pose) pairs `cv2.calibrateHandEye` needs.
    ///
    /// It runs here rather than as its own tool because the daemon owns
    /// the RTDE connection — the robot answers one client, and taking the
    /// daemon down to calibrate the camera it depends on is the wrong
    /// way round.
    ///
    /// `CurrentStep` is deliberately left alone: it means "an interrupted
    /// sequence to resume from" everywhere else, and a capture is not
    /// resumable — the arm returns to the pose it started from, so the
    /// remedy for any failure is to fix the setup and trigger again.
    fn run_handeye(&mut self) -> Result<bool, SequencerError> {
        log::info(">>> HAND-EYE CALIBRATION MODE: tool rotations in place <<<");
        let mut detector = match handeye::Detector::spawn(&self.config.handeye) {
            Ok(d) => d,
            Err(e) => {
                log::error(&e.to_string());
                return Ok(false);
            }
        };
        // Taken before anything moves, so the mode can put the arm back:
        // see `handeye_return`.
        let entry = self.motion.current_joints()?;
        self.handeye_restore_aim()?;
        self.handeye_aim(&mut detector);
        match self.handeye_capture(&mut detector)? {
            Some(path) => log::info(&format!(
                "Hand-eye capture complete. Next: {} tools/handeye/solve_joint.py {}",
                self.config.handeye.solve_python.display(),
                path.display()
            )),
            // Reported, not returned as an error: nothing is wrong with
            // the robot, the arm is back where it started, and killing a
            // production daemon over a camera that could not see the tag
            // would be a worse outcome than another trigger.
            None => log::error("Hand-eye capture produced nothing usable; not written"),
        }
        self.handeye_return(&entry)?;
        Ok(false)
    }

    /// Feels for the seat the arm is standing in: both walls along base
    /// x, both along base y, and the floor under the puck.
    ///
    /// The offsets exist so the puck goes in and comes out without
    /// rubbing — lateral contact during the insert is what shakes the
    /// sample, and shaking is what damages it. Every other way of
    /// checking them is a proxy for that: the taught pose records where
    /// an operator once believed the bore was, and the vision correction
    /// measures a rim, which is not the same object as the bore whose
    /// axis the puck has to follow. This measures the thing itself.
    ///
    /// The result is printed and nothing else. Turning "the bore centre
    /// is 0.23 mm along base y from here" into a change in the waypoint
    /// table is a decision about the rack, taken with the numbers from
    /// more than one holder in front of you, and it is not one a mode
    /// that has just been driven into contact should take on its own.
    ///
    /// Two triggers, like hand-eye: the first selects the mode and opens
    /// a jog hold, the second commits from wherever the arm was jogged
    /// to. The hold is where the operator lowers the gripped puck into
    /// the seat; the idle wait it was entered from cannot be that hold,
    /// because it services no jog (jogging there would move the taught
    /// pose the sequence starts from).
    ///
    /// `CurrentStep` is untouched, and the arm is left where the probe
    /// started rather than lifted clear: this
    /// mode is entered mid-run with a sample in the fingers, and neither
    /// deciding how to lift a puck back out of a bore nor claiming the
    /// run is over belongs here.
    fn run_seat_probe(&mut self) -> Result<bool, SequencerError> {
        log::info(">>> SEAT PROBE MODE: step into contact, measure, write nothing <<<");
        log::info("========================================");
        log::info("SEAT PROBE: jog the gripped puck down into the seat");
        log::info("  Use JogX/Y/Z + JogStep to move the TCP, Gripper to hold the puck");
        log::info("  Set Trigger=1 to probe from where the arm is");
        log::info("  The arm will push up to the configured abort force. If the puck is");
        log::info("  not in the seat, or the fingers are empty, stop here.");
        log::info("========================================");
        self.wait_for_trigger(true);
        match self.seat_probe_here()? {
            Some(e) => Err(e),
            None => Ok(false),
        }
    }

    /// The probe body shared by [`Sequencer::run_seat_probe`] (after its
    /// jog hold) and holder map mode (straight after seating the puck):
    /// step into contact, measure every configured height, print the
    /// report, write nothing.
    ///
    /// The nested result keeps "how much was measured" apart from "where
    /// is the arm": `Ok(None)` measured everything, `Ok(Some(e))` was
    /// stopped by `e` but the arm is back at the entry pose with the
    /// grip on the puck, and `Err` means even the walk back failed, so
    /// the arm is somewhere a caller must not build on.
    fn seat_probe_here(&mut self) -> Result<Option<SequencerError>, SequencerError> {
        let p = &self.config.probe;
        let depth = ProbeLimits::new(&p.depth, p.velocity_scale);
        let limits = Limits {
            lateral: ProbeLimits::new(&p.lateral, p.velocity_scale),
            depth,
            // The moves between heights are stepped like the depth probe
            // but bounded like a pick, not like a push onto a floor.
            lift: ProbeLimits {
                abort_n: p.lift_abort_n,
                ..depth
            },
            tilt: TiltLimits {
                step_rad: p.tilt.step_deg.to_radians(),
                sweep_rad: p.tilt.sweep_deg.to_radians(),
                abort_n: p.tilt.abort_n,
                abort_nm: p.tilt.abort_nm,
                velocity_scale: p.velocity_scale,
            },
        };

        // The sweep's own failure travels beside its results rather than
        // instead of them: a bracket that aborted at the bottom of a
        // ladder does not make the heights above it unmeasured, and this
        // mode's whole output is what it printed.
        let (play_mm, (levels, walked, returned)) =
            self.with_grip_loosened(|s| Ok(s.sweep_heights(limits)))?;

        log::info("========================================");
        log::info("SEAT PROBE RESULT (measured from the pose the probe started at)");
        // Half of it on each side, so the first `play/2` of every lateral
        // run is a pad leaving the puck rather than the puck crossing its
        // clearance. Printed because that is the offset between these
        // numbers and the bore.
        if play_mm > 0.0 {
            log::info(&format!("  fingers opened {play_mm:.3} mm before stepping"));
        } else {
            log::info("  the grip was held throughout — no play in front of the steps");
        }
        for level in levels {
            if self.config.probe.heights_mm.is_empty() {
                log::info("  at the pose the probe was triggered at:");
            } else {
                log::info(&format!(
                    "  at {:+.2} mm above the trigger pose:",
                    level.height_mm
                ));
            }
            if let Some(load) = level.load {
                log::info(&format!(
                    "    arrived carrying ({:+.2}, {:+.2}, {:+.2}) N in base x, y, z",
                    load.x, load.y, load.z
                ));
            }
            for bracket in level.brackets {
                log::info(&format!("    {}", bracket.summary()));
            }
            for tilt in level.tilts {
                log::info(&format!("    {}", tilt.summary()));
            }
            // The trip point carries the threshold in it; the fit does
            // not. Both are printed because they disagreeing by more than
            // a step is how a floor that was already loaded before the
            // probe began announces itself.
            let Some(floor) = &level.floor else {
                log::info("    base z: not reached — the level stopped before the floor probe");
                continue;
            };
            if let Some(why) = floor.too_hard() {
                log::info(&format!("    base z: {why}"));
                continue;
            }
            match (floor.tripped_mm(), floor.wall_mm()) {
                (Some(t), Some(f)) => log::info(&format!(
                    "    base z: floor at {f:+.3} mm from the force-versus-depth fit, \
                     tripped at {t:+.3} mm"
                )),
                (Some(t), None) => log::info(&format!(
                    "    base z: tripped at {t:+.3} mm, too few rising samples to fit a floor"
                )),
                (None, _) => log::info(&format!(
                    "    base z: no floor within {:.2} mm — the puck is not resting on anything",
                    depth.travel_mm
                )),
            }
        }
        // Where the arm is, this knows: every probe walks itself back out
        // before it returns, and a retrace that could not be flown is an
        // error rather than a logged caveat. Whether that pose is in a
        // seat is the operator's claim from the jog hold, and nothing here
        // has measured it.
        log::info(
            "  Nothing was written. The arm is back at the pose the probe \
             started from and the grip is back on the puck; lift it out \
             before the next trigger.",
        );
        log::info("========================================");
        if let Err(back) = returned {
            return Err(match walked {
                Err(e) => SequencerError(format!(
                    "{e} — and the arm did not get back to the trigger pose: {back}"
                )),
                Ok(()) => back,
            });
        }
        Ok(walked.err())
    }

    /// Holder map (`CalibMode = 6`): one trigger probes one holder's
    /// seat. The puck comes from `MapSource` — another holder, ferried
    /// through the sample holder exactly like the normal sequence does
    /// (steps 0-11, 13-20) — or, when `MapSource` is 0 or the target
    /// itself, the resident puck picked in place (steps 0-4). The probe
    /// then runs without a jog hold, since the arm just seated the puck
    /// and the taught pose is the pose under test, and the puck is left
    /// seated (steps 21-23).
    ///
    /// Step numbers and waypoints are the normal sequence's own, so
    /// PauseStep and CurrentStep behave as usual; like the other
    /// calibration modes there are no vision hooks, because the taught
    /// error is the thing being measured. A probe that measured less
    /// than asked but walked back out still releases the puck and
    /// retreats before the failure is reported — the alternative leaves
    /// the arm parked in a bore for no reason.
    ///
    /// Always runs from the top: a resume into a half-done map would
    /// grip air or probe an empty seat, so a non-zero `StartStep` is
    /// refused rather than honored.
    fn run_holder_map(
        &mut self,
        wd: &WaypointData,
        base: &BaseWaypoints,
        target: i32,
        start: i32,
    ) -> Result<bool, SequencerError> {
        if start != 0 {
            return Err(SequencerError(format!(
                "holder map always runs from the top; StartStep is {start} — set \
                 it to 0 (recover a failed map with CalibMode=4 and a fresh \
                 trigger instead)"
            )));
        }
        let source = match self.epics.read_map_source() {
            0 => target,
            s => s,
        };
        let w_t = self.compute_run_waypoints(wd, base, target)?;
        if source == target {
            log::info(&format!(
                ">>> HOLDER MAP MODE: probe holder {target} in place <<<"
            ));
            self.hand(0, "open_hand", true, 0)?;
            self.arm(1, "holder_standby", &w_t.standby, 0)?;
            self.cartesian(2, "holder_above", &w_t.above, 0)?;
            self.cartesian(3, "holder_on_position", &w_t.on_pos, 0)?;
            self.hand(4, "close_gripper", false, 0)?;
        } else {
            log::info(&format!(
                ">>> HOLDER MAP MODE: probe holder {target} with the puck from \
                 holder {source} <<<"
            ));
            let w_s = self.compute_run_waypoints(wd, base, source)?;
            self.hand(0, "open_hand", true, 0)?;
            self.arm(1, "holder_standby", &w_s.standby, 0)?;
            self.cartesian(2, "holder_above", &w_s.above, 0)?;
            self.cartesian(3, "holder_on_position", &w_s.on_pos, 0)?;
            self.hand(4, "close_gripper", false, 0)?;
            self.cartesian(5, "holder_above_return", &w_s.above, 0)?;
            self.cartesian(6, "holder_retreat", &w_s.retreat, 0)?;
            self.arm(7, "sample_holder_standby", &w_s.sh_standby, 0)?;
            self.cartesian(8, "sample_holder_above", &w_s.sh_above, 0)?;
            self.cartesian(9, "sample_holder_on_position", &w_s.sh_on_pos, 0)?;
            self.hand(10, "open_gripper", true, 0)?;
            self.cartesian(11, "sample_holder_above_return", &w_s.sh_above, 0)?;
            self.cartesian(13, "sample_holder_above_2nd", &w_t.sh_above, 0)?;
            self.cartesian(14, "sample_holder_on_position_2nd", &w_t.sh_on_pos, 0)?;
            self.hand(15, "close_gripper_2nd", false, 0)?;
            self.cartesian(16, "sample_holder_above_2nd_return", &w_t.sh_above, 0)?;
            self.cartesian(17, "sample_holder_standby_2nd", &w_t.sh_standby, 0)?;
            self.arm(18, "holder_standby_return", &w_t.standby, 0)?;
            self.cartesian(19, "holder_above_final", &w_t.above, 0)?;
            self.cartesian(20, "holder_on_position_final", &w_t.on_pos, 0)?;
        }

        let probed = self.seat_probe_here()?;
        if let Some(e) = &probed {
            log::warn(&format!(
                "probe measured less than asked ({e}); the arm walked back to \
                 the seat, so the puck is left there and the arm retreats \
                 before the failure is reported"
            ));
        }
        self.hand(21, "open_gripper_final", true, 0)?;
        self.cartesian(22, "holder_above_final_return", &w_t.above, 0)?;
        self.cartesian(23, "holder_standby_final", &w_t.standby, 0)?;
        match probed {
            Some(e) => Err(e),
            None => Ok(false),
        }
    }

    /// Probes at every configured height and always brings the arm back
    /// to the pose the mode was triggered at.
    ///
    /// The height is owned here for the same reason the grip is owned by
    /// [`Sequence::with_grip_loosened`]: a run that ends on an abort part
    /// way up a sweep would otherwise leave the arm at a height nothing
    /// records, holding a sample, and the next trigger resumes the
    /// sequence by lifting it from there. So the return is one call after
    /// the walk, on both paths, and what the walk had already measured
    /// survives the failure that stopped it.
    /// The two results travel separately so a caller can tell "measured
    /// less than asked" (first) from "the arm is not back" (second).
    #[allow(clippy::type_complexity)]
    fn sweep_heights(
        &mut self,
        limits: Limits,
    ) -> (
        Vec<Level>,
        Result<(), SequencerError>,
        Result<(), SequencerError>,
    ) {
        let axes = match Axes::in_tool(&mut self.motion) {
            Ok(axes) => axes,
            Err(e) => return (Vec::new(), Err(e), Ok(())),
        };
        // The trigger pose is level zero and is always measured: it is the
        // taught pose the sequence itself uses, and the first vertical
        // move has to start from a centre like every other one.
        let mut heights = vec![0.0];
        heights.extend_from_slice(&self.config.probe.heights_mm);
        let mut levels = Vec::new();
        // Where the arm is relative to the trigger pose, in base mm. Every
        // move this mode makes goes through it, so the way back is one
        // subtraction rather than a history to replay.
        let mut from_trigger = Vector3::zeros();
        let centring = Centring::new(&self.config.probe.centring);
        let walked = self.walk_heights(
            &heights,
            &axes,
            limits,
            centring,
            &mut from_trigger,
            &mut levels,
        );
        let returned = Self::return_to_trigger(&mut self.motion, &axes, &mut from_trigger, limits);
        (levels, walked, returned)
    }

    /// The walking half of [`Sequence::sweep_heights`], split out so that
    /// every way it can leave is a return into code that puts the arm
    /// back. `from_trigger` is where the arm stands relative to the pose
    /// the mode was triggered at, in base mm, and every component is
    /// updated only by a move that finished.
    #[allow(clippy::too_many_arguments)]
    fn walk_heights(
        &mut self,
        heights: &[f64],
        axes: &Axes,
        limits: Limits,
        centring: Centring,
        from_trigger: &mut Vector3,
        levels: &mut Vec<Level>,
    ) -> Result<(), SequencerError> {
        for &height in heights {
            let climb = height - from_trigger.z;
            // Not a plain move: a lift out of the seat drags the puck
            // sideways with 30 N/mm behind it, and a bracket measured
            // under that load is the arm's deflection rather than the
            // bore. The climb gives way to it step by step, so what it
            // spends sideways is part of where the arm now stands.
            let climbed = self.motion.climb_centred(
                axes.up,
                [axes.x, axes.y],
                climb,
                limits.lift,
                centring,
                &format!("to {height:+.2} mm"),
            )?;
            from_trigger.x += climbed.offset.x;
            from_trigger.y += climbed.offset.y;
            from_trigger.z = height;
            levels.push(Level {
                height_mm: height,
                load: (climb.abs() >= MIN_EXECUTABLE_MM).then_some(climbed.load),
                brackets: Vec::new(),
                floor: None,
                tilts: Vec::new(),
            });
            let level = levels.last_mut().expect("just pushed");
            // Pushed before it is filled, and filled in place, so that a
            // fault part way through a level still leaves the axes it had
            // already measured in the report. This mode writes nothing
            // else — what it printed is the whole of it.
            Self::measure_level(&mut self.motion, axes, level, limits)?;
            Self::centre_here(&mut self.motion, axes, level, from_trigger, limits.lateral)?;
        }
        Ok(())
    }

    /// Moves the arm to the middle of the walls this level measured.
    ///
    /// Done after every level because the next thing that happens is a
    /// vertical move, and a lift that starts off-centre spends the bore's
    /// clearance sideways on the way up: from the taught pose a straight
    /// 3 mm lift met 8.17 N of lateral force by +0.316 mm and stopped
    /// (doc §16.11). A height only means something once x and y are in
    /// the middle of what is holding the puck.
    ///
    /// An axis with no centre — nothing found, one wall only, a direction
    /// given up on at the abort force — is left alone rather than guessed
    /// at.
    fn centre_here(
        motion: &mut Motion<'a>,
        axes: &Axes,
        level: &Level,
        from_trigger: &mut Vector3,
        lateral: ProbeLimits,
    ) -> Result<(), SequencerError> {
        for (component, (bracket, dir)) in level.brackets.iter().zip([axes.x, axes.y]).enumerate() {
            let Some(centre) = bracket.centre_mm() else {
                continue;
            };
            // Below the step the walls were found with there is nothing
            // to correct: the midpoint of two trip points a step apart is
            // not known to better than that step, and the arm will not
            // execute the move anyway — a 0.008 mm command travelled
            // -0.004 mm and tripped the step-taken guard (doc §16.4).
            if centre.abs() < lateral.step_mm {
                log::info(&format!(
                    "  {} is already centred to within one {:.3} mm step ({centre:+.3} mm)",
                    bracket.label(),
                    lateral.step_mm
                ));
                continue;
            }
            let label = format!("centre {} by {centre:+.3} mm", bracket.label());
            motion.probe_reposition(dir, centre, lateral, &label)?;
            from_trigger[component] += centre;
        }
        Ok(())
    }

    /// Undoes every move the sweep made, laterally first and then down.
    ///
    /// That order and not the other: the lateral offsets were measured at
    /// height, where the clearance to move in them exists, and lowering
    /// first would ask the arm to slide sideways with the puck pressed
    /// back into its seat.
    fn return_to_trigger(
        motion: &mut Motion<'a>,
        axes: &Axes,
        from_trigger: &mut Vector3,
        limits: Limits,
    ) -> Result<(), SequencerError> {
        for (dir, component, what) in [
            (axes.x, 0, "base x"),
            (axes.y, 1, "base y"),
            (axes.up, 2, "height"),
        ] {
            let back = -from_trigger[component];
            motion.probe_reposition(dir, back, limits.lift, &format!("return the {what}"))?;
            from_trigger[component] = 0.0;
        }
        Ok(())
    }

    /// Both lateral axes and the floor, at wherever the arm is standing.
    ///
    /// Takes the motion rather than `self` so the borrow of `level` can
    /// live alongside it.
    fn measure_level(
        motion: &mut Motion<'a>,
        axes: &Axes,
        level: &mut Level,
        limits: Limits,
    ) -> Result<(), SequencerError> {
        for (name, dir) in [("base x", axes.x), ("base y", axes.y)] {
            level
                .brackets
                .push(motion.bracket_axis(dir, limits.lateral, name)?);
        }
        // Turning is last: it is the only motion here that changes the
        // tool's orientation, and every bracket above measures along an
        // axis derived from it.
        if limits.tilt.sweep_rad > 0.0 {
            for (name, axis) in [
                ("tilt about base x", axes.x),
                ("tilt about base y", axes.y),
                ("tilt about base z", axes.up),
            ] {
                level.tilts.push(motion.tilt_scan(axis, limits.tilt, name)?);
            }
        }
        // Straight down in base, not along whichever tool axis happens
        // to point that way: the seat floor is horizontal and the puck's
        // own weight is already resting on it, so the probe has to push
        // along the same line that weight acts on for the slope to mean
        // depth.
        level.floor = Some(motion.probe_until_contact(-axes.up, limits.depth, "base z-")?);
        Ok(())
    }

    /// Runs `probe` with the fingers opened by `probe.loosen_mm`, and puts
    /// the grip back where it found it before returning — on the error
    /// paths as well as the successful one.
    ///
    /// The play is what the probe needs: clamped on a seated puck the
    /// gripper, puck and bore are one closed loop, and a step measures how
    /// hard the arm deforms it rather than how far anything can move.
    /// Loose, the fingers give way first and the puck can reach a wall.
    ///
    /// Restoring is not optional and not the probe's business to remember.
    /// A run that ends on an abort or a blocked retrace leaves the operator
    /// with a puck hanging in a loosened grip over an open rack, and the
    /// next trigger resumes the sequence by lifting it. So the loosen and
    /// the restore are one call with the probe in the middle, rather than a
    /// pair for a caller to keep in step. The one exit this cannot cover is
    /// a panic, which takes the daemon down with it — and a daemon that
    /// dies holding a sample is already the documented hazard, because its
    /// successor's activation stroke opens the fingers.
    fn with_grip_loosened<T>(
        &mut self,
        probe: impl FnOnce(&mut Self) -> Result<T, SequencerError>,
    ) -> Result<(f64, T), SequencerError> {
        // One value decides both halves. Reading it once and gating the
        // restore on the same answer is what keeps "a loosened grip is
        // always restored" true without making it depend on how much play
        // the fingers actually managed to open.
        let loosening = self.config.probe.loosen_mm > 0.0;
        let play_mm = if loosening {
            self.gripper
                .loosen_by(self.config.probe.loosen_mm / 1000.0, &self.epics)
                * 1000.0
        } else {
            0.0
        };
        let out = probe(self);
        if loosening {
            self.gripper.regrip(&self.epics);
        }
        out.map(|t| (play_mm, t))
    }

    /// Puts the arm back where this mode was entered from.
    ///
    /// The other three modes end on the taught `holder_standby`, so the
    /// next trigger's step 1 — the daemon's only planned move — starts
    /// from a pose the level-tool path constraint accepts. This mode aims
    /// by jogging the camera down at the tag, and the constraint never
    /// applies while it does so: every hand-eye move is interpolated, and
    /// `level_tool` is read only by `Motion::move_planned`. Without this
    /// return the aiming pose is what the next trigger inherits, planning
    /// reports "start or goal state is itself invalid", and the daemon
    /// exits having moved nothing. Measured once at 70.8 degrees off level
    /// against the configured 3.
    ///
    /// A blocked line is logged and not returned as an error, as in
    /// [`Sequence::handeye_restore_aim`]: the robot is fine and killing
    /// the daemon would not put the arm anywhere better. It is logged at
    /// error rather than warn because the arm is then left in the state
    /// this function exists to prevent.
    fn handeye_return(&mut self, entry: &JointMap) -> Result<(), SequencerError> {
        let velocity = self.config.handeye.velocity_scale;
        let here = self.motion.current_joints()?;
        if !self.motion.direct_path_is_clear(&here, entry)? {
            log::error(
                "Cannot return to the pose hand-eye mode started from: the \
                 straight line is blocked. The arm is left off level, so the \
                 next sequence's step 1 will fail to plan — freedrive it back \
                 to a taught pose before triggering.",
            );
            return Ok(());
        }
        log::info("Returning to the pose hand-eye mode started from");
        self.motion
            .move_direct(entry, velocity, velocity, "hand-eye entry pose")
    }

    /// Returns to the pose the last usable capture started from, so the
    /// aiming hold that follows opens with the tag already in view.
    ///
    /// Nothing saved, or a straight line there that is blocked, is
    /// reported and skipped rather than fatal: the hold can reach the
    /// same place by jog, and refusing to calibrate because a convenience
    /// move was unavailable would be the wrong trade. A failure once the
    /// line has been cleared is a motion fault like any other and exits.
    ///
    /// The move is interpolated rather than planned for the same reason
    /// the capture's are: the aiming pose points the camera down at the
    /// tag and is nowhere near level, so the level-tool path constraint
    /// the sequence plans under has no solution to it.
    fn handeye_restore_aim(&mut self) -> Result<(), SequencerError> {
        let velocity = self.config.handeye.velocity_scale;
        let saved = match handeye::load_aim_pose(&self.config.handeye.out_dir) {
            Ok(Some(joints)) => joints,
            Ok(None) => {
                log::info("No aiming pose saved yet — aim from where the arm is");
                return Ok(());
            }
            Err(e) => {
                log::error(&format!("Saved aiming pose unusable: {e}"));
                return Ok(());
            }
        };
        let here = self.motion.current_joints()?;
        if !self.motion.direct_path_is_clear(&here, &saved)? {
            log::warn(
                "Saved aiming pose is not reachable in a straight line from here — jog to it",
            );
            return Ok(());
        }
        log::info("Returning to the saved aiming pose");
        self.motion
            .move_direct(&saved, velocity, velocity, "saved aiming pose")
    }

    /// Jog-enabled hold so the operator can bring the tag into view before
    /// anything moves on its own. Returns on the next trigger.
    ///
    /// The idle wait this mode was entered from runs with jog disabled —
    /// the arm sits at a taught standby pose there and jogging it would
    /// move the sequence's own start point. So the aiming happens here,
    /// after the mode is known, exactly as the other two calibration
    /// modes hold for jogging mid-sequence. Two triggers total: the first
    /// selects the mode, the second commits to the capture.
    ///
    /// The live detection readout is the point of doing this with the
    /// detector already running: "the tag is on screen" and "the detector
    /// can solve it" are different claims, and only the second one makes
    /// the capture work.
    fn handeye_aim(&mut self, detector: &mut handeye::Detector) {
        log::info("========================================");
        log::info("HAND-EYE AIMING: jog the tag into view");
        log::info("  Use JogX/Y/Z + JogStep to move the TCP");
        log::info("  Set Trigger=1 to start the capture from where the arm is");
        log::info("========================================");
        let mut next_probe = std::time::Instant::now();
        loop {
            if self.epics.read_trigger() > 0 {
                if !self.epics.write_trigger(0) {
                    log::warn("Failed to reset trigger PV to 0, continuing anyway...");
                }
                return;
            }
            self.process_jog();
            if std::time::Instant::now() >= next_probe {
                next_probe = std::time::Instant::now() + HANDEYE_AIM_PROBE;
                match detector.detect() {
                    Ok(Some(seen)) => log::info(&format!("  tag: {}", seen.summary())),
                    Ok(None) => log::info("  tag: not detected from here"),
                    Err(e) => log::warn(&format!("  detector: {e}")),
                }
            }
            std::thread::sleep(POLL);
        }
    }

    /// Steps the tool along its own x and then its own y, watching where
    /// the tag goes, and turns the two observations into the frame-sweep
    /// poses.
    ///
    /// The alternative — computing the offsets from a previous run's
    /// `T_ee_cam` — would make the capture depend on the answer it is
    /// there to produce, and would be wrong in exactly the case the
    /// operator is recalibrating for: a camera that has been remounted at
    /// a different roll. Two moves cost about twenty seconds and the
    /// mount stops mattering.
    ///
    /// Every way this can fall short — no IK for a probe, a blocked line,
    /// a tag the detector loses, probes that do not span the plane —
    /// returns an empty sweep and lets the rotation set run alone. The
    /// sweep improves the lens model; it is not what makes a capture
    /// valid, so it must not be able to stop one.
    ///
    /// Each probe that saw the tag is also pushed onto `samples`. These
    /// two poses are a 20 mm pure translation from home at an unchanged
    /// orientation, which the rotation schedule contains nothing like,
    /// and that is the geometry a look-then-move correction actually
    /// executes — so the calibration was being fitted without ever
    /// sampling it. They land in `samples` rather than in the return
    /// value because every early return above discards the sweep, and a
    /// probe that succeeded before a later one failed is still a good
    /// observation.
    fn handeye_probe_frame(
        &mut self,
        detector: &mut handeye::Detector,
        home: &JointMap,
        home_pose: &nalgebra::Isometry3<f64>,
        at_home: &handeye::Detection,
        samples: &mut Vec<handeye::Sample>,
    ) -> Result<Vec<(String, [f64; 2])>, SequencerError> {
        let velocity = self.config.handeye.velocity_scale;
        let mut probes = Vec::new();
        for (label, step) in [
            ("probe x", [HANDEYE_PROBE_M, 0.0]),
            ("probe y", [0.0, HANDEYE_PROBE_M]),
        ] {
            let target = home_pose
                * nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::new(step[0], step[1], 0.0),
                    nalgebra::UnitQuaternion::identity(),
                );
            let Some(goal) = self.model.ik_from_seed(home, &target, label)? else {
                log::warn(&format!("{label}: no IK — skipping the frame sweep"));
                return Ok(Vec::new());
            };
            if !self.motion.direct_path_is_clear(home, &goal)? {
                log::warn(&format!(
                    "{label}: no clear path — skipping the frame sweep"
                ));
                return Ok(Vec::new());
            }
            self.motion.move_direct(&goal, velocity, velocity, label)?;
            // Read before the detection and before homing, as the capture
            // loop does: it is the pose the tag was seen from that pairs
            // with the observation, not the pose the arm ends at.
            let observed = self.motion.current_joints()?;
            let seen = detector.detect();
            self.motion.move_direct(home, velocity, velocity, "home")?;
            match seen {
                Ok(Some(seen)) => {
                    let shift = [
                        seen.center_px[0] - at_home.center_px[0],
                        seen.center_px[1] - at_home.center_px[1],
                    ];
                    samples.push(handeye::Sample {
                        label: label.into(),
                        base_t_ee: self.model.fk(&observed)?,
                        joints: observed,
                        seen,
                    });
                    probes.push((step, shift));
                }
                Ok(None) => {
                    log::warn(&format!(
                        "{label}: tag not detected — skipping the frame sweep"
                    ));
                    return Ok(Vec::new());
                }
                // Left to the capture's own detect to report and act on:
                // it already turns a dead detector into "nothing usable"
                // rather than a daemon exit.
                Err(e) => {
                    log::warn(&format!(
                        "{label}: detector: {e} — skipping the frame sweep"
                    ));
                    return Ok(Vec::new());
                }
            }
        }
        let jacobian = match handeye::ImageJacobian::from_probes(&probes) {
            Ok(j) => j,
            Err(e) => {
                log::warn(&format!("{e} — skipping the frame sweep"));
                return Ok(Vec::new());
            }
        };
        let sweep = handeye::frame_sweep(&jacobian, detector.intrinsics().image_size, at_home);
        log::info(&format!(
            "Image Jacobian: {}; {} frame-sweep poses",
            jacobian.summary(),
            sweep.len()
        ));
        Ok(sweep)
    }

    /// The capture itself. `Ok(None)` is "no usable sample set" with the
    /// reason already logged; errors stay reserved for motion and
    /// structural faults, which exit the daemon like any other step
    /// failure.
    fn handeye_capture(
        &mut self,
        detector: &mut handeye::Detector,
    ) -> Result<Option<std::path::PathBuf>, SequencerError> {
        let h = &self.config.handeye;
        let (angle_deg, velocity, min_samples) = (h.angle_deg, h.velocity_scale, h.min_samples);
        let standoffs = h.standoff_mm.clone();
        let out_dir = h.out_dir.clone();

        let home = self.motion.current_joints()?;
        let home_pose = self.model.fk(&home)?;
        log::info(&format!(
            "Home: ik_frame at ({:.3}, {:.3}, {:.3}) m",
            home_pose.translation.x, home_pose.translation.y, home_pose.translation.z
        ));
        let Some(at_home) = detector.detect()? else {
            log::error("No tag visible from the current pose — aim the camera at it first");
            return Ok(None);
        };
        log::info(&format!("Tag at home: {}", at_home.summary()));

        let mut samples = vec![handeye::Sample {
            label: "home".into(),
            joints: home.clone(),
            base_t_ee: home_pose,
            seen: at_home.clone(),
        }];
        let sweep =
            self.handeye_probe_frame(detector, &home, &home_pose, &at_home, &mut samples)?;
        let schedule = handeye::schedule(
            self.model,
            &home,
            &home_pose,
            angle_deg,
            &standoffs,
            &sweep,
            |goal| self.motion.direct_path_is_clear(&home, goal),
        )?;
        let dropped: Vec<String> = schedule
            .dropped
            .iter()
            .map(|(label, why)| format!("{label}: {why}"))
            .collect();
        log::info(&format!(
            "Schedule: {} poses to visit, {} dropped ({})",
            schedule.poses.len(),
            dropped.len(),
            or_dash(&dropped)
        ));

        std::fs::create_dir_all(&out_dir)
            .map_err(|e| SequencerError(format!("cannot create {}: {e}", out_dir.display())))?;

        let mut missed = Vec::new();

        // Interpolated, not planned: every pose here is the home pose with
        // the tool turned a few degrees, and the straight line to it keeps
        // the arm in that neighbourhood. The schedule already refused the
        // ones whose line is blocked, so a refusal below is a real fault.
        let mut detector_died = None;
        for (label, goal) in &schedule.poses {
            wait_for_stop_clear(&self.epics);
            log::info(&format!("Capturing {label}"));
            self.motion.move_direct(goal, velocity, velocity, label)?;
            let observed = self.motion.current_joints()?;
            let detected = detector.detect();
            // Home between poses so a dropped detection cannot compound
            // into a drift away from the one configuration known to see
            // the tag — and, since it runs before the reply is judged, so
            // that a detector which died leaves the arm parked here
            // rather than rotated.
            self.motion.move_direct(&home, velocity, velocity, "home")?;
            match detected {
                Ok(Some(seen)) => {
                    log::info(&format!("  {}", seen.summary()));
                    samples.push(handeye::Sample {
                        label: label.clone(),
                        base_t_ee: self.model.fk(&observed)?,
                        joints: observed,
                        seen,
                    });
                }
                Ok(None) => {
                    log::warn(&format!("  {label}: tag not detected, dropping this pose"));
                    missed.push(label.clone());
                }
                // The camera side failing is not the robot failing.
                Err(e) => {
                    detector_died = Some(e);
                    break;
                }
            }
        }
        if let Some(e) = detector_died {
            log::error(&format!("detector stopped answering: {e}"));
            return Ok(None);
        }
        log::info(&format!(
            "Captured {} poses ({} missed: {})",
            samples.len(),
            missed.len(),
            or_dash(&missed)
        ));
        if samples.len() < min_samples {
            log::error(&format!(
                "only {} poses saw the tag, need {min_samples} — re-aim so the tag sits \
                 nearer the image centre, or lower handeye.angle_deg",
                samples.len()
            ));
            return Ok(None);
        }
        // Timestamped, never a fixed name: a second capture used to
        // overwrite the first without saying so, and did — the poses the
        // calibration of record was fitted to were nearly lost that way.
        // The aim pose keeps its fixed path beside it, because "the last
        // aim worth returning to" is exactly one thing and a capture
        // replacing it is the intent.
        let path = out_dir.join(format!(
            "samples_{}.yaml",
            chrono::Local::now().format("%Y%m%d_%H%M%S")
        ));
        handeye::write_samples(&path, &samples, angle_deg, detector.intrinsics())?;
        // Saved from here, not from the aiming hold: this is the pose the
        // capture actually ran from and the tag was demonstrably visible
        // at, which is the only kind worth returning to.
        handeye::save_aim_pose(&out_dir, &home)?;
        Ok(Some(path))
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
    /// Returns the correction tagged with the tool frame of `obs`, the
    /// pose the arm is standing at while the picture is taken, or `None`
    /// when vision/the hook is off, this run never reached the
    /// observation pose (`measure` false — a resume skipped the step
    /// that parks there, so the picture would be taken from an unknown
    /// pose), the correction is below the deadband, or observe_only.
    ///
    /// `obs` is the waypoint the caller has just moved to, not the
    /// arm's live joints: the two agree when the step ran, and reading
    /// the waypoint keeps the frame exact rather than picking up
    /// whatever settling error the servo left behind.
    ///
    /// Outside observe_only a timeout, an invalid verdict, and an
    /// over-limit correction are all errors: never guess, never
    /// auto-apply a large jump. In observe_only every failure is logged
    /// and swallowed — Phase C observation must not affect operation.
    fn vision_correction(
        &mut self,
        measure: bool,
        obs: &JointMap,
        kind: VisionKind,
        label: &str,
    ) -> Result<Option<Correction>, SequencerError> {
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
                Ok(Some(Correction {
                    d,
                    frame: self.model.fk(obs)?.rotation,
                }))
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
    ///
    /// The two are no longer measured from the same pose — the slot from
    /// a standby, the grip offset from the above the arm rose to after
    /// closing — so both are brought into `target`'s tool frame before
    /// they are added. Summing them as raw triples would add two vectors
    /// that live in different frames.
    fn combine_corrections(
        &self,
        slot: Option<Correction>,
        grip: Option<Correction>,
        target: &JointMap,
        label: &str,
    ) -> Result<Option<Correction>, SequencerError> {
        if slot.is_none() && grip.is_none() {
            return Ok(None);
        }
        let frame = self.model.fk(target)?.rotation;
        let mut sum = [0.0; 3];
        for c in [slot, grip].into_iter().flatten() {
            let d = c.in_frame(&frame);
            sum = [sum[0] + d[0], sum[1] + d[1], sum[2] + d[2]];
        }
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
                Ok(Some(Correction { d, frame }))
            }
            Gate::TooLarge(mag) => Err(SequencerError(format!(
                "{label}: combined vision correction {mag:.2} mm exceeds the {:.2} mm limit",
                v.max_correction
            ))),
        }
    }

    /// `base` shifted by a vision correction, with
    /// `apply_cartesian_offset`'s fallback-to-original escape hatch
    /// closed: a correction the arm cannot realize must stop the
    /// sequence, not silently descend uncorrected.
    ///
    /// The correction arrives in the observation pose's tool frame and
    /// `apply_cartesian_offset` wants it in `base`'s, so it is rotated
    /// here. Across the taught poses the two frames differ by 0.79 deg
    /// at the rack and 0.21 deg at the sample holder, which at the 3 mm
    /// limit is 41 um — small, and not a reason to assume it away.
    fn corrected(
        &self,
        base: &JointMap,
        c: Option<Correction>,
        label: &str,
    ) -> Result<JointMap, SequencerError> {
        let Some(c) = c else {
            return Ok(base.clone());
        };
        let d = c.in_frame(&self.model.fk(base)?.rotation);
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
        wait_for_stop_clear(&self.epics);
        log::info(&format!("Step {step}: {name}"));
        true
    }

    /// Shared epilogue after a successful step: publish `CurrentStep`,
    /// then honor a matching `PauseStep`. The error is the operator
    /// aborting the hold — see [`Sequencer::wait_for_pause_step_change`].
    fn step_epilogue(&mut self, step: i32) -> Result<(), SequencerError> {
        self.epics.write_current_step(step);
        self.wait_for_pause_step_change(step)
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
        self.step_epilogue(step)
    }

    /// A Cartesian step. This used to take an optional via point so a
    /// correction measured at above could be reached laterally before
    /// the descent; measuring at standby instead folds that shift into
    /// the move to above, and the via is gone with it.
    fn cartesian(
        &mut self,
        step: i32,
        name: &str,
        goal: &JointMap,
        start: i32,
    ) -> Result<(), SequencerError> {
        if !self.step_prologue(step, name, start) {
            return Ok(());
        }
        let gentle = self.config.sequence.cartesian_velocity_scale;
        self.motion.move_cartesian(
            goal,
            self.config.sequence.velocity_scale * gentle,
            self.config.sequence.acceleration_scale * gentle,
            name,
        )?;
        log::info("  -> Completed (Cartesian)");
        self.step_epilogue(step)
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
        self.step_epilogue(step)
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

    /// Holds after step `current_step` while `PauseStep` names it.
    ///
    /// Changing `PauseStep` resumes, which used to be the only way out —
    /// and resuming is forward, into the next step. An operator who
    /// paused to look at something and then wanted to stop had to kill
    /// the daemon, which is the one thing that opens the gripper on the
    /// way back up. `Wait = 2` ends the run instead: the sequence
    /// unwinds to the trigger loop with `CurrentStep` intact, and
    /// `CalibMode = 4` is there to walk the arm home.
    fn wait_for_pause_step_change(&mut self, current_step: i32) -> Result<(), SequencerError> {
        let pause_step = self.epics.read_pause_step();
        if pause_step == 0 || pause_step != current_step {
            return Ok(());
        }
        log::info(&format!(
            "PAUSED at step {current_step} - change PauseStep to resume, or Wait=2 to stop here"
        ));
        loop {
            // Only the literal 2 reads as Skip; 1, anything else, and a
            // failed read all answer Continue, so a dropped CA read
            // cannot abort a run by itself.
            if self.epics.read_wait() == WaitStatus::Skip {
                return Err(SequencerError(format!(
                    "run stopped by Wait=2 while paused at step {current_step}"
                )));
            }
            let pause_step = self.epics.read_pause_step();
            if pause_step != current_step {
                log::info(&format!(
                    "PauseStep changed to {pause_step}, resuming execution..."
                ));
                return Ok(());
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
        // Up, off the seat: tool y is base -z here. Applied after the
        // shared trim and only to this pose, so that raising the seat
        // does not move the standby pose the shared trim also carries.
        let holder_on = self.model.apply_cartesian_offset(
            &holder_on,
            [0.0, -w.holder_on_lift, 0.0],
            false,
            "holder_seat_lift",
        )?;
        // The shared angle is the rack's rigid-body pitch, so it is
        // applied to the holder-1 pose BEFORE the per-holder
        // translation on purpose: the 30(N-1) mm step then runs along
        // the tilted tool y, landing holder N where a rack pitched by
        // the same angle physically puts it (0.157 mm per holder at
        // 0.3 deg). Translating untilted instead was tried 2026-08-18
        // and pushed holder 2 into its +y wall at +0.5 mm while
        // holder 4 measures centred only with the pitched placement —
        // the sideways drift is the rack, not an artifact.
        let holder_on = self.model.apply_tool_point_rotation(
            &holder_on,
            [1.0, 0.0, 0.0],
            w.holder_on_tilt_x_deg.to_radians(),
            "holder_rack_pitch",
        )?;
        // The other lean axis, same rigid-body reasoning: a rack rolled
        // about base y (tool z here) tips the stack line sideways, so
        // the roll also goes on before the per-holder translation. At
        // these sub-degree angles the x/z application order is a
        // second-order (angle-product) effect.
        let holder_on = self.model.apply_tool_point_rotation(
            &holder_on,
            [0.0, 0.0, 1.0],
            w.holder_on_tilt_z_deg.to_radians(),
            "holder_rack_roll",
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
        let mut y_offset = f64::from(holder_number - 1) * self.config.sequence.holder_offset;
        let mut x_offset = 0.0;
        let mut z_offset = 0.0;
        if (2..=10).contains(&holder_number) {
            let idx = (holder_number - 2) as usize;
            if let Some(x) = w.holder_multi_x_offsets.get(idx) {
                x_offset = *x;
            }
            if let Some(y) = w.holder_multi_y_offsets.get(idx) {
                y_offset += *y;
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
        let on_pos = self.model.apply_cartesian_offset(
            &base.holder_on,
            [x_offset, y_offset, z_offset],
            false,
            "on_position",
        )?;
        // The per-holder trim is a local seat error, not rack
        // geometry, so unlike the shared pitch above it is turned about
        // this holder's own grasp point after the translation and moves
        // no position: a trim change tunes only the angle. The above
        // pose derives from this one and inherits the pitch, which is
        // what makes the approach come down already square instead of
        // twisting at the bottom.
        let on_pos = apply_wrist3(self.model.apply_tool_point_rotation(
            &on_pos,
            [1.0, 0.0, 0.0],
            w.holder_tilt_x_trim_deg(holder_number).to_radians(),
            "holder_seat_tilt",
        )?);
        let on_pos = apply_wrist3(self.model.apply_tool_point_rotation(
            &on_pos,
            [0.0, 0.0, 1.0],
            w.holder_tilt_z_trim_deg(holder_number).to_radians(),
            "holder_seat_tilt_z",
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

    /// A correction is a physical displacement, so re-expressing it in
    /// another tool frame must rotate it. The boundaries that matter are
    /// the frames themselves: same frame changes nothing, a quarter turn
    /// swaps the axes, and going out and back is the identity.
    #[test]
    fn a_correction_rotates_into_the_frame_it_is_applied_in() {
        use nalgebra::{UnitQuaternion, Vector3};
        let quarter_turn =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);
        let c = Correction {
            d: [1.0, 0.0, 0.0],
            frame: quarter_turn,
        };

        let same = c.in_frame(&quarter_turn);
        assert!((same[0] - 1.0).abs() < 1e-12 && same[1].abs() < 1e-12);

        // Measured along the observation frame's x, which the quarter
        // turn about z has pointed along the model frame's y.
        let world = c.in_frame(&UnitQuaternion::identity());
        assert!(world[0].abs() < 1e-12, "x {}", world[0]);
        assert!((world[1] - 1.0).abs() < 1e-12, "y {}", world[1]);

        let back = Correction {
            d: world,
            frame: UnitQuaternion::identity(),
        }
        .in_frame(&quarter_turn);
        assert!((back[0] - c.d[0]).abs() < 1e-12 && (back[1] - c.d[1]).abs() < 1e-12);
    }

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
