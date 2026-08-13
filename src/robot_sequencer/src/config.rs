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
            min_correction: 0.05,
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
        assert_eq!(config.scene.objects.len(), 1);
        // Every anchored path must exist in the checkout.
        for path in [
            &config.robot.urdf,
            &config.robot.srdf,
            &config.robot.script_file,
            &config.robot.output_recipe,
            &config.robot.input_recipe,
            &config.sequence.waypoints_yaml,
            &config.scene.objects[0].stl,
        ] {
            assert!(path.exists(), "missing: {}", path.display());
        }
        for dir in config.robot.mesh_packages.values() {
            assert!(dir.is_dir(), "missing mesh package dir: {}", dir.display());
        }
        // Production carries an explicit vision section, disabled.
        assert!(!config.vision.enabled);
        assert!(!config.vision.observe_only);
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
        assert_eq!(v.min_correction, 0.05);
        assert_eq!(v.max_correction, 3.0);
        assert_eq!(v.req_pv, "Robot:Vision:Req");
    }
}
