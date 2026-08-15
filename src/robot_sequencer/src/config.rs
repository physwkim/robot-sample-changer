//! Sequencer configuration (`config/sequencer.yaml`).
//!
//! Field-for-field mirror of the replaced ROS node's parameters plus the
//! pieces that used to live in separate ROS nodes: the stage collision
//! scene (`stage_scene_utils` + `setup_stage_acm.py`) and the joint-limit
//! overrides (`ur3e_hande_moveit_config/config/joint_limits.yaml`).

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::error::SequencerError;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub robot: RobotConfig,
    pub joint_limits: JointLimitsConfig,
    pub epics: EpicsConfig,
    pub sequence: SequenceConfig,
    pub gripper: GripperConfig,
    pub scene: SceneConfig,
    #[serde(default)]
    pub vision: VisionConfig,
    #[serde(default)]
    pub handeye: HandEyeConfig,
    #[serde(default)]
    pub probe: ProbeConfig,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotConfig {
    pub ip: String,
    pub urdf: PathBuf,
    pub srdf: PathBuf,
    pub mesh_packages: HashMap<String, PathBuf>,
    pub group: String,
    pub ik_frame: String,
    pub script_file: PathBuf,
    pub output_recipe: PathBuf,
    pub input_recipe: PathBuf,
    /// RTDE output stream rate.
    ///
    /// Nothing here needs the controller's 500 Hz: trajectories go out
    /// over the trajectory interface, and RTDE is read only for a joint
    /// sample before planning and for the execution wait loop. What the
    /// rate does set is how fast unread packages fill the 131 KB socket
    /// buffer — at 500 Hz that is half a second, and URControl drops a
    /// client that lets it fill.
    ///
    /// Independent of the servoj control period, which
    /// `Motion::connect` takes from `max_frequency()` — the
    /// controller's own rate, not this request.
    #[serde(default = "default_rtde_frequency_hz")]
    pub rtde_frequency_hz: f64,
}

fn default_rtde_frequency_hz() -> f64 {
    50.0
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointLimitsConfig {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    #[serde(default)]
    pub position_overrides: HashMap<String, [f64; 2]>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EpicsConfig {
    pub trigger_pv: String,
    pub start_step_pv: String,
    pub wait_pv: String,
    pub holder_pv: String,
    pub stop_pv: String,
    pub current_step_pv: String,
    pub gripper_pv: String,
    pub gripper_rbv_pv: String,
    pub pause_step_pv: String,
    pub calib_mode_pv: String,
    pub loaded_pv: String,
    pub jog_x_pv: String,
    pub jog_y_pv: String,
    pub jog_z_pv: String,
    pub jog_step_pv: String,
}

/// The level-tool path constraint (see [`SequenceConfig::level_tool`]).
///
/// The constraint is "the tool's approach axis stays in the horizontal
/// plane", not "the tool keeps one fixed orientation": the taught poses
/// approach the holder along -Y and the stage along +X, 92 degrees
/// apart, so a fixed orientation would make the stage unreachable.
/// Rotation within the horizontal plane is therefore left free.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LevelToolConfig {
    pub enabled: bool,
    /// How far the approach axis may leave horizontal, in degrees. The
    /// taught poses are themselves 1.64 and 0.84 degrees off level, so
    /// anything below ~2 makes them unreachable.
    pub tolerance_deg: f64,
}

impl Default for LevelToolConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            tolerance_deg: 3.0,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SequenceConfig {
    pub waypoints_yaml: PathBuf,
    pub holder_offset: f64,
    pub velocity_scale: f64,
    pub acceleration_scale: f64,
    pub cartesian_translation_step: f64,
    pub cartesian_rotation_step: f64,
    pub cartesian_min_fraction: f64,
    /// Keep the gripper level through joint-space plans.
    ///
    /// Joint-space planning only constrains the goal, so RRT-Connect is
    /// free to roll the tool over on the way there — measured on the
    /// real robot, the 63-degree move between the holder and the stage
    /// swung the gripper to vertical mid-path. Cartesian steps already
    /// interpolate the pose and are unaffected.
    #[serde(default)]
    pub level_tool: LevelToolConfig,
    pub jog_velocity_scale: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GripperConfig {
    pub mode: GripperMode,
    pub open_position: f64,
    pub close_position: f64,
    pub close_settle_target: f64,
    pub reach_tolerance: f64,
    pub settle_timeout: f64,
    pub open_threshold: f64,
    /// Hand-E background communication rate. Every cycle is two Modbus
    /// transactions, and the tool-communication bridge drops roughly one
    /// transaction in a few thousand, so halving the rate halves how
    /// often that costs anything. 20 ms between position samples is well
    /// inside what the settle wait needs.
    #[serde(default = "default_gripper_poll_hz")]
    pub poll_hz: u32,
}

fn default_gripper_poll_hz() -> u32 {
    50
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum GripperMode {
    Hande,
    Simulated,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SceneConfig {
    #[serde(default)]
    pub objects: Vec<SceneObject>,
    #[serde(default)]
    pub allow_collisions_with: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SceneObject {
    pub id: String,
    pub stl: PathBuf,
    pub scale: [f64; 3],
    pub position: [f64; 3],
    pub rpy: [f64; 3],
}

/// Vision look-then-move correction (doc/vision_camera_setup.md).
/// Disabled by default; when enabled every listed PV must connect at
/// startup and a failed or invalid measurement stops the sequence (the
/// resume semantics handle it like any other step failure).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct VisionConfig {
    pub enabled: bool,
    /// Phase C observation mode: measure and log at every hook, but
    /// never move and never fail the sequence on the verdict.
    pub observe_only: bool,
    /// Seconds to wait for the vision node to answer a request.
    pub timeout: f64,
    /// Corrections below this magnitude (mm) are skipped as noise.
    pub min_correction: f64,
    /// Corrections above this magnitude (mm) stop the sequence — a
    /// mis-detection, a wrong slot, or a moved rack, never auto-applied.
    pub max_correction: f64,
    /// Per-hook enables (all only meaningful when `enabled`).
    pub pick_align: bool,
    pub grip_offset: bool,
    pub place_align: bool,
    pub seating_check: bool,
    pub req_pv: String,
    pub kind_pv: String,
    pub done_pv: String,
    pub valid_pv: String,
    pub dx_pv: String,
    pub dy_pv: String,
    pub dz_pv: String,
    pub quality_pv: String,
    pub seated_pv: String,
    pub tilt_pv: String,
}

impl Default for VisionConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            observe_only: false,
            timeout: 2.0,
            // Above the arm's own re-approach spread at the observation
            // pose, measured over 20 production cycles: sigma (0.022,
            // 0.007) mm, peak-to-peak (0.095, 0.026) mm (doc §14).
            min_correction: 0.10,
            max_correction: 3.0,
            pick_align: true,
            grip_offset: true,
            place_align: true,
            seating_check: true,
            req_pv: "Robot:Vision:Req".into(),
            kind_pv: "Robot:Vision:Kind".into(),
            done_pv: "Robot:Vision:Done".into(),
            valid_pv: "Robot:Vision:Valid".into(),
            dx_pv: "Robot:Vision:DX".into(),
            dy_pv: "Robot:Vision:DY".into(),
            dz_pv: "Robot:Vision:DZ".into(),
            quality_pv: "Robot:Vision:Quality".into(),
            seated_pv: "Robot:Vision:Seated".into(),
            tilt_pv: "Robot:Vision:Tilt".into(),
        }
    }
}

/// One probe direction's step size, allowance and force limits.
///
/// Two of these and not eight prefixed fields on [`ProbeConfig`] because
/// the numbers genuinely differ between the two directions: a bore wall is
/// half a millimetre away and answers a light touch, a seat floor is
/// several millimetres down and is what the puck's own weight already
/// rests on.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ProbeAxisConfig {
    /// One step, mm. Also the worst-case overshoot past first contact,
    /// since the probe reads force between steps and stops on the first
    /// one that trips.
    pub step_mm: f64,
    /// Total travel allowed in one direction, mm. Running out without
    /// touching anything is an answer, not a failure.
    pub travel_mm: f64,
    /// Force change from the start pose that counts as contact, N.
    pub threshold_n: f64,
    /// Force change that aborts the probe outright, N.
    pub abort_n: f64,
}

/// Force-stopped seat probing (`CalibMode` = Seat Probe). Commissioning
/// only: it measures where a bore actually is, which a taught pose only
/// records an operator's belief about.
///
/// It exists because the criterion for the offsets is that the puck goes
/// in and comes out without rubbing — lateral contact is what shakes the
/// sample — and image agreement is a proxy for that which cannot see a
/// bore whose axis differs from where its rim looks.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ProbeConfig {
    /// Velocity and acceleration scale for every probe step and every
    /// return. One value rather than one per direction: it says how
    /// gently the arm may move next to a sample, which does not change
    /// between pushing sideways and pushing down.
    pub velocity_scale: f64,
    /// Sideways, toward a bore wall.
    pub lateral: ProbeAxisConfig,
    /// Downward, toward the seat floor.
    pub depth: ProbeAxisConfig,
}

impl Default for ProbeConfig {
    fn default() -> Self {
        Self {
            velocity_scale: 0.02,
            lateral: ProbeAxisConfig {
                // Ten steps to a wall at the nominal 0.50 mm radial
                // clearance, and 0.05 mm of overshoot past it.
                step_mm: 0.05,
                // Past the clearance by enough that "no contact" means the
                // bore is not where the pose says, rather than that the
                // probe was too short.
                travel_mm: 1.5,
                // Seven times the 0.073 N the arm scatters standing still
                // (doc/vision_correction_plan.md §16.1). Reading between
                // steps is what makes a threshold this low usable at all.
                threshold_n: 0.5,
                // Well under the 8.5-22.9 N the arm was measured pushing
                // through a rubbing insert: that is the force this mode
                // exists to stop the sequence from applying, not a level
                // to probe up to.
                abort_n: 5.0,
            },
            depth: ProbeAxisConfig {
                step_mm: 0.10,
                travel_mm: 4.0,
                threshold_n: 1.0,
                abort_n: 8.0,
            },
        }
    }
}

impl Default for ProbeAxisConfig {
    /// Only reachable through a partial `lateral:`/`depth:` block, where
    /// serde fills the unnamed fields from here. The lateral numbers are
    /// the safer of the two sets to inherit.
    fn default() -> Self {
        ProbeConfig::default().lateral
    }
}

/// Eye-in-hand calibration capture (`CalibMode` = Hand-Eye). Commissioning
/// only: it produces the `T_ee_cam` that [`VisionConfig`]'s node needs
/// before it can convert pixels to a TCP-local correction at all.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct HandEyeConfig {
    /// Interpreter for `detector`; needs cv2, numpy and pyepics.
    pub python: PathBuf,
    /// Interpreter for the solver; needs cv2, numpy, scipy and yaml. A
    /// second field and not `python` again because the two roles do not
    /// fit in one environment here: the detector's needs pyepics and has
    /// no yaml, so the command the capture prints on completion used to
    /// name an interpreter that could not run it.
    pub solve_python: PathBuf,
    /// The AprilTag detector driven over stdin/stdout.
    pub detector: PathBuf,
    /// Directory each capture's `samples_<timestamp>.yaml` is written to,
    /// alongside the one `aim_pose.yaml` they share.
    pub out_dir: PathBuf,
    /// Largest tool rotation in the schedule (deg). The tag must stay in
    /// frame at the extremes, so this is bounded by how near the image
    /// centre the tag sits at the home pose, not by the arm.
    pub angle_deg: f64,
    /// Tool-z offsets (mm) the rotation set is repeated at, measured from
    /// the aiming pose; positive is toward whatever the camera is looking
    /// at. A single standoff leaves solvePnP's depth error identical in
    /// every sample, where it is indistinguishable from a camera mounted
    /// that much further out and lands in `T_ee_cam`'s translation
    /// unchallenged. Every entry costs a full rotation set of poses.
    pub standoff_mm: Vec<f64>,
    /// Velocity and acceleration scale for the capture moves — slower
    /// than the sequence, because every pose is a fresh excursion into a
    /// cramped cell rather than a taught path.
    pub velocity_scale: f64,
    /// Below this many detected poses the capture is reported as failed
    /// rather than written: `calibrateHandEye` returns an answer for
    /// under-determined input as readily as for good input.
    pub min_samples: usize,
}

impl Default for HandEyeConfig {
    fn default() -> Self {
        Self {
            python: PathBuf::from("python3"),
            solve_python: PathBuf::from("python3"),
            detector: PathBuf::from("tools/handeye/detector.py"),
            out_dir: PathBuf::from("handeye_samples"),
            angle_deg: 8.0,
            standoff_mm: vec![0.0],
            velocity_scale: 0.1,
            min_samples: 5,
        }
    }
}

impl Config {
    /// Loads the YAML and resolves every relative path against the config
    /// file's directory, so the daemon can be started from anywhere.
    pub fn load(path: &Path) -> Result<Self, SequencerError> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| SequencerError(format!("cannot read config {}: {e}", path.display())))?;
        let mut config: Config = serde_yaml::from_str(&text)
            .map_err(|e| SequencerError(format!("cannot parse config {}: {e}", path.display())))?;
        let base = path.parent().unwrap_or(Path::new("."));
        let anchor = |p: &mut PathBuf| {
            if p.is_relative() {
                *p = base.join(&*p);
            }
        };
        anchor(&mut config.robot.urdf);
        anchor(&mut config.robot.srdf);
        anchor(&mut config.robot.script_file);
        anchor(&mut config.robot.output_recipe);
        anchor(&mut config.robot.input_recipe);
        for dir in config.robot.mesh_packages.values_mut() {
            anchor(dir);
        }
        anchor(&mut config.sequence.waypoints_yaml);
        for object in &mut config.scene.objects {
            anchor(&mut object.stl);
        }
        anchor(&mut config.handeye.detector);
        anchor(&mut config.handeye.out_dir);
        // Checked always, not only when the mode is selected: there is no
        // enable flag to gate on, and an out-of-range angle should be a
        // startup error rather than a surprise after the arm has moved.
        let h = &config.handeye;
        if !(0.0..=30.0).contains(&h.angle_deg) {
            return Err(SequencerError(
                "handeye.angle_deg must be within 0..30".into(),
            ));
        }
        if !(0.0..=0.5).contains(&h.velocity_scale) {
            return Err(SequencerError(
                "handeye.velocity_scale must be within 0..0.5".into(),
            ));
        }
        if h.min_samples < 3 {
            return Err(SequencerError(
                "handeye.min_samples must be at least 3 (calibrateHandEye needs three \
                 relative motions)"
                    .into(),
            ));
        }
        // Same reasoning as the hand-eye block above, with more at stake:
        // these numbers are the only thing bounding how hard the arm is
        // allowed to push on a sample, and a probe that reads them for the
        // first time has already been triggered.
        for (name, axis) in [
            ("probe.lateral", &config.probe.lateral),
            ("probe.depth", &config.probe.depth),
        ] {
            if axis.step_mm <= 0.0 {
                return Err(SequencerError(format!("{name}.step_mm must be positive")));
            }
            if axis.travel_mm < axis.step_mm {
                return Err(SequencerError(format!(
                    "{name}.travel_mm must be at least one {name}.step_mm"
                )));
            }
            if axis.threshold_n <= 0.0 {
                return Err(SequencerError(format!(
                    "{name}.threshold_n must be positive"
                )));
            }
            // An abort at or below the contact threshold aborts every
            // probe the moment it succeeds, which reads as "the probe
            // cannot touch anything" rather than as a misconfiguration.
            if axis.abort_n <= axis.threshold_n {
                return Err(SequencerError(format!(
                    "{name}.abort_n must be above {name}.threshold_n"
                )));
            }
        }
        if !(0.0..=0.1).contains(&config.probe.velocity_scale) {
            return Err(SequencerError(
                "probe.velocity_scale must be within 0..0.1 (a probe steps into contact)".into(),
            ));
        }
        if config.vision.enabled {
            let v = &config.vision;
            // Below ~0.002 mm the IK offset helper's epsilon treats the
            // correction as zero and the strict fallback check misfires.
            if v.min_correction < 0.002 {
                return Err(SequencerError(
                    "vision.min_correction must be >= 0.002 mm".into(),
                ));
            }
            if v.max_correction < v.min_correction {
                return Err(SequencerError(
                    "vision.max_correction must be >= vision.min_correction".into(),
                ));
            }
            if v.timeout <= 0.0 {
                return Err(SequencerError("vision.timeout must be positive".into()));
            }
        }
        Ok(config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_the_production_config() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let config = Config::load(path).expect("load");
        assert_eq!(config.robot.group, "ur_manipulator");
        assert_eq!(config.robot.ik_frame, "robotiq_hande_end");
        assert_eq!(config.gripper.mode, GripperMode::Hande);
        assert_eq!(config.joint_limits.position_overrides.len(), 3);
        // The stage ships as an approximate convex decomposition (see
        // the scene comment in sequencer.yaml), so this is a part count,
        // not a single mesh — but an empty scene would mean the daemon
        // plans with no stage at all, which must not pass silently.
        assert!(!config.scene.objects.is_empty());
        // Every anchored path must exist in the checkout.
        for path in [
            &config.robot.urdf,
            &config.robot.srdf,
            &config.robot.script_file,
            &config.robot.output_recipe,
            &config.robot.input_recipe,
            &config.sequence.waypoints_yaml,
        ] {
            assert!(path.exists(), "missing: {}", path.display());
        }
        for object in &config.scene.objects {
            assert!(
                object.stl.exists(),
                "missing scene mesh: {}",
                object.stl.display()
            );
        }
        for dir in config.robot.mesh_packages.values() {
            assert!(dir.is_dir(), "missing mesh package dir: {}", dir.display());
        }
        // Production carries an explicit vision section, disabled.
        assert!(!config.vision.enabled);
        assert!(!config.vision.observe_only);
        // The hand-eye detector is spawned by path, so a typo there
        // surfaces only when an operator selects the mode on the robot.
        assert!(
            config.handeye.detector.exists(),
            "missing hand-eye detector: {}",
            config.handeye.detector.display()
        );
        assert!(
            config.handeye.python.is_absolute(),
            "handeye.python must name an interpreter with cv2/pyepics, not inherit PATH"
        );
        assert!(
            config.handeye.solve_python.is_absolute(),
            "handeye.solve_python must name an interpreter with cv2/scipy/yaml, not inherit PATH"
        );
    }

    /// The URSim config has no vision section: the serde default path
    /// must yield a disabled feature with the documented thresholds.
    #[test]
    fn ursim_config_defaults_vision_off() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer_ursim.yaml"
        ));
        let config = Config::load(path).expect("load");
        let v = &config.vision;
        assert!(!v.enabled);
        assert_eq!(v.min_correction, 0.10);
        assert_eq!(v.max_correction, 3.0);
        assert_eq!(v.req_pv, "Robot:Vision:Req");
        // Same for hand-eye: absent from the URSim config, and its
        // defaults must be the ones the daemon banner advertises.
        assert_eq!(config.handeye.angle_deg, 8.0);
        assert_eq!(config.handeye.min_samples, 5);
    }

    /// Same reasoning as the hand-eye ranges, on the numbers that bound
    /// how hard the arm may push on a sample. `abort_n` at or below
    /// `threshold_n` is the one that would not look like a
    /// misconfiguration at runtime: every probe would abort at the
    /// instant it succeeded, and read as a probe that cannot touch
    /// anything.
    #[test]
    fn rejects_probe_limits_that_would_abort_on_contact() {
        let base = std::fs::read_to_string(Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer_ursim.yaml"
        )))
        .expect("read");
        let dir = std::env::temp_dir().join("probe_config_bounds");
        std::fs::create_dir_all(&dir).expect("tmpdir");
        for (name, section, want) in [
            (
                "abort_at_threshold",
                "probe:\n  lateral:\n    threshold_n: 0.5\n    abort_n: 0.5\n",
                "probe.lateral.abort_n",
            ),
            (
                "abort_under_threshold",
                "probe:\n  depth:\n    threshold_n: 2.0\n    abort_n: 1.0\n",
                "probe.depth.abort_n",
            ),
            (
                "step_zero",
                "probe:\n  lateral:\n    step_mm: 0.0\n",
                "probe.lateral.step_mm",
            ),
            (
                "travel_under_one_step",
                "probe:\n  depth:\n    step_mm: 0.5\n    travel_mm: 0.2\n",
                "probe.depth.travel_mm",
            ),
            (
                "velocity_hi",
                "probe:\n  velocity_scale: 0.5\n",
                "probe.velocity_scale",
            ),
        ] {
            let path = dir.join(format!("{name}.yaml"));
            std::fs::write(&path, format!("{base}\n{section}")).expect("write");
            let err = Config::load(&path).expect_err(name).to_string();
            assert!(err.contains(want), "{name}: unexpected error {err}");
        }
    }

    /// The range checks run at load, not at mode selection: an operator
    /// finding out mid-capture that the schedule was nonsense has already
    /// moved the arm.
    #[test]
    fn rejects_out_of_range_handeye_limits() {
        let base = std::fs::read_to_string(Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer_ursim.yaml"
        )))
        .expect("read");
        let dir = std::env::temp_dir().join("handeye_config_bounds");
        std::fs::create_dir_all(&dir).expect("tmpdir");
        for (name, section, want) in [
            ("angle_hi", "handeye:\n  angle_deg: 45.0\n", "angle_deg"),
            ("angle_neg", "handeye:\n  angle_deg: -1.0\n", "angle_deg"),
            (
                "velocity_hi",
                "handeye:\n  velocity_scale: 0.9\n",
                "velocity_scale",
            ),
            ("samples_lo", "handeye:\n  min_samples: 2\n", "min_samples"),
        ] {
            let path = dir.join(format!("{name}.yaml"));
            std::fs::write(&path, format!("{base}\n{section}")).expect("write");
            let err = Config::load(&path).expect_err(name).to_string();
            assert!(err.contains(want), "{name}: unexpected error {err}");
        }
    }

    /// The boundary values themselves are legal — 30 deg and 0.5 are the
    /// documented maxima, not the first rejected value.
    #[test]
    fn accepts_the_handeye_limit_boundaries() {
        let base = std::fs::read_to_string(Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer_ursim.yaml"
        )))
        .expect("read");
        let dir = std::env::temp_dir().join("handeye_config_bounds");
        std::fs::create_dir_all(&dir).expect("tmpdir");
        let path = dir.join("edges.yaml");
        std::fs::write(
            &path,
            format!(
                "{base}\nhandeye:\n  angle_deg: 30.0\n  velocity_scale: 0.5\n  min_samples: 3\n"
            ),
        )
        .expect("write");
        let config = Config::load(&path).expect("boundary values are legal");
        assert_eq!(config.handeye.angle_deg, 30.0);
        assert_eq!(config.handeye.min_samples, 3);
    }
}
