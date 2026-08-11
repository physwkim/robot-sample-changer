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
    }
}
