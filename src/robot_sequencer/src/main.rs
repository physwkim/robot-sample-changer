//! robot-sequencer: the EPICS-triggered sample-changer daemon, ROS-free.
//!
//! Replaces the `epics_triggered_sequence` C++ node plus the UR ROS
//! driver, MoveIt, and the stage-scene/ACM setup nodes: epics-ca-rs
//! talks CA to the (unchanged) robot_ioc, cspace plans, ur-driver
//! executes, robotiq-hande drives the gripper directly.
//!
//! Usage: `robot-sequencer <config/sequencer.yaml>`
//!
//! On daemon death (crash or Ctrl-C) the robot's external-control
//! program loses its reverse-interface connection and halts motion; the
//! IOC keeps `CurrentStep`, so the operator resumes per the
//! resume-after-crash procedure in CLAUDE.md.

mod config;
mod epics;
mod error;
mod gripper;
mod handeye;
mod log;
mod model;
mod motion;
mod sequence;
mod stream;
mod waypoints;

use std::path::PathBuf;
use std::process::ExitCode;

use crate::config::Config;
use crate::epics::Epics;
use crate::error::SequencerError;
use crate::gripper::Gripper;
use crate::model::Model;
use crate::motion::Motion;
use crate::sequence::Sequencer;

fn run() -> Result<(), SequencerError> {
    let config_path: PathBuf = std::env::args_os()
        .nth(1)
        .ok_or_else(|| SequencerError("usage: robot-sequencer <config.yaml>".into()))?
        .into();
    let config = Config::load(&config_path)?;

    log::info("Loading robot model...");
    let model = Model::load(&config)?;

    let epics = Epics::connect(
        &config.epics,
        config.vision.enabled.then_some(&config.vision),
    )?;
    // A trigger left at 1 by a crash must not fire a sequence the moment
    // the daemon returns; clearing it is the C++ node's startup behavior.
    epics.write_trigger(0);

    let motion = Motion::connect(&model, &config)?;
    let gripper = Gripper::connect(&config)?;

    log::info("========================================");
    log::info("EPICS Triggered Sequence Ready");
    log::info("========================================");
    log::info(&format!("  Trigger PV: {}", config.epics.trigger_pv));
    log::info(&format!("  StartStep PV: {}", config.epics.start_step_pv));
    log::info(&format!(
        "  Wait PV: {} (0=wait, 1=continue, 2=skip)",
        config.epics.wait_pv
    ));
    log::info(&format!("  Holder PV: {} (1-10)", config.epics.holder_pv));
    log::info(&format!(
        "  Stop PV: {} (1=pause, 0=resume)",
        config.epics.stop_pv
    ));
    log::info(&format!(
        "  CurrentStep PV: {} (updated after each step)",
        config.epics.current_step_pv
    ));
    log::info(&format!(
        "  Gripper PV: {} (command: 0=close, 1=open)",
        config.epics.gripper_pv
    ));
    log::info(&format!(
        "  Gripper_RBV PV: {} (status, threshold={:.3})",
        config.epics.gripper_rbv_pv, config.gripper.open_threshold
    ));
    log::info(&format!(
        "  PauseStep PV: {} (N=pause after step N until changed)",
        config.epics.pause_step_pv
    ));
    log::info(&format!("  CalibMode PV: {}", config.epics.calib_mode_pv));
    log::info("    0=Normal (full sequence)");
    log::info("    1=Holder calibration (0-5, wait, 20-23)");
    log::info("    2=SampleHolder calibration (0-8, wait, 16-23)");
    log::info("    3=Hand-eye calibration (trigger, jog to aim, trigger again; tool rotations");
    log::info(&format!(
        "      ±{:.0} deg in place -> {}/samples_<timestamp>.yaml)",
        config.handeye.angle_deg,
        config.handeye.out_dir.display()
    ));
    log::info("    4=Recover (return the arm to holder standby; gripper untouched)");
    log::info("    5=Seat probe (trigger, jog the gripped puck into the seat, trigger again;");
    log::info(&format!(
        "      steps into contact at {:.2}/{:.2} N, measures, writes nothing)",
        config.probe.bore.lateral.threshold_n, config.probe.bore.depth.threshold_n
    ));
    log::info("    6=Holder map (fetch the puck, probe the seat, leave it seated;");
    log::info(&format!(
        "      source holder from {}, 0 = the target itself)",
        config.epics.map_source_pv
    ));
    log::info("    7=Holder transfer (carry the puck straight to the target seat;");
    log::info(&format!(
        "      source holder from {}, no stage leg and no probe)",
        config.epics.map_source_pv
    ));
    if config.vision.enabled {
        log::info(&format!(
            "  Vision correction: ENABLED{} (deadband {:.2} mm, limit {:.2} mm, {})",
            if config.vision.observe_only {
                " [observe_only]"
            } else {
                ""
            },
            config.vision.min_correction,
            config.vision.max_correction,
            config.vision.req_pv
        ));
    }
    log::info("========================================");

    Sequencer::new(epics, motion, gripper, &model, &config).run()
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            log::error(&e.to_string());
            ExitCode::FAILURE
        }
    }
}
