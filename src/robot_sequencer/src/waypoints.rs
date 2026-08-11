//! Taught-waypoint file loader.
//!
//! Same file the ROS node consumed (`taught_waypoints.yaml`, ROS parameter
//! layout kept so calibrated values carry over unchanged): values live
//! under `/**: ros__parameters:`, with the same fallbacks the C++ loader
//! had (`ros__parameters` at root, then root itself). Reloaded before
//! every sequence, so operators can re-teach between runs.

use std::path::Path;

use serde_yaml::Value;

use crate::error::SequencerError;

/// Joint-value vectors are 7 wide in the file:
/// `[gripper_finger, shoulder_pan, wrist_3, wrist_2, wrist_1, elbow,
/// shoulder_lift]` (the teach tool recorded /joint_states order). Index 0
/// is the gripper finger and is not an arm joint.
pub const WAYPOINT_JOINT_ORDER: [&str; 6] = [
    "shoulder_pan_joint",
    "wrist_3_joint",
    "wrist_2_joint",
    "wrist_1_joint",
    "elbow_joint",
    "shoulder_lift_joint",
];

#[derive(Debug, Clone)]
pub struct WaypointData {
    pub holder1_standby: Vec<f64>,
    pub holder1_on_position: Vec<f64>,
    pub sample_holder_standby: Vec<f64>,
    pub sample_holder_on_position: Vec<f64>,
    pub above_y_offset: f64,
    pub retreat_z_offset: f64,
    pub holder1_on_x_offset: f64,
    pub holder1_on_y_offset: f64,
    pub holder1_on_z_offset: f64,
    pub sample_holder_on_x_offset: f64,
    pub sample_holder_on_y_offset: f64,
    pub sample_holder_on_z_offset: f64,
    pub holder_multi_x_offsets: Vec<f64>,
    pub holder_multi_z_offsets: Vec<f64>,
    pub wrist3_rotation_offset: f64,
}

fn f64_at(params: &Value, key: &str, default: f64) -> f64 {
    params.get(key).and_then(Value::as_f64).unwrap_or(default)
}

fn vec_at(params: &Value, key: &str) -> Result<Vec<f64>, SequencerError> {
    let list = params
        .get(key)
        .and_then(Value::as_sequence)
        .ok_or_else(|| SequencerError(format!("waypoints: missing or non-list '{key}'")))?;
    list.iter()
        .map(|v| {
            v.as_f64()
                .ok_or_else(|| SequencerError(format!("waypoints: non-number in '{key}'")))
        })
        .collect()
}

fn vec_at_or(params: &Value, key: &str, default_len: usize) -> Vec<f64> {
    match params.get(key).and_then(Value::as_sequence) {
        Some(list) => list.iter().filter_map(Value::as_f64).collect(),
        None => vec![0.0; default_len],
    }
}

impl WaypointData {
    pub fn load(path: &Path) -> Result<Self, SequencerError> {
        let text = std::fs::read_to_string(path).map_err(|e| {
            SequencerError(format!("cannot read waypoints {}: {e}", path.display()))
        })?;
        let root: Value = serde_yaml::from_str(&text).map_err(|e| {
            SequencerError(format!("cannot parse waypoints {}: {e}", path.display()))
        })?;
        let params = root
            .get("/**")
            .and_then(|v| v.get("ros__parameters"))
            .or_else(|| root.get("ros__parameters"))
            .unwrap_or(&root);

        let data = Self {
            holder1_standby: vec_at(params, "holder1_standby")?,
            holder1_on_position: vec_at(params, "holder1_on_position")?,
            sample_holder_standby: vec_at(params, "sample_holder_standby")?,
            sample_holder_on_position: vec_at(params, "sample_holder_on_position")?,
            above_y_offset: f64_at(params, "above_y_offset", -0.005),
            retreat_z_offset: f64_at(params, "retreat_z_offset", -0.05),
            holder1_on_x_offset: f64_at(params, "holder1_on_position_x_offset", 0.0),
            holder1_on_y_offset: f64_at(params, "holder1_on_position_y_offset", 0.0),
            holder1_on_z_offset: f64_at(params, "holder1_on_position_z_offset", 0.0),
            sample_holder_on_x_offset: f64_at(params, "sample_holder_on_position_x_offset", 0.0),
            sample_holder_on_y_offset: f64_at(params, "sample_holder_on_position_y_offset", 0.0),
            sample_holder_on_z_offset: f64_at(params, "sample_holder_on_position_z_offset", 0.0),
            holder_multi_x_offsets: vec_at_or(params, "holder_multi_x_offsets", 9),
            holder_multi_z_offsets: vec_at_or(params, "holder_multi_z_offsets", 9),
            wrist3_rotation_offset: f64_at(params, "wrist3_rotation_offset", 0.0),
        };

        for (key, list) in [
            ("holder1_standby", &data.holder1_standby),
            ("holder1_on_position", &data.holder1_on_position),
            ("sample_holder_standby", &data.sample_holder_standby),
            ("sample_holder_on_position", &data.sample_holder_on_position),
        ] {
            if list.len() != 7 {
                return Err(SequencerError(format!(
                    "waypoints: '{key}' has {} values, expected 7 (gripper + 6 arm joints)",
                    list.len()
                )));
            }
        }
        Ok(data)
    }

    /// Arm joints of a 7-wide taught vector as `(name, value)` pairs
    /// (drops the gripper finger at index 0).
    pub fn arm_joints(values: &[f64]) -> Vec<(String, f64)> {
        WAYPOINT_JOINT_ORDER
            .iter()
            .zip(&values[1..])
            .map(|(name, value)| (name.to_string(), *value))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_the_production_file() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/taught_waypoints.yaml"
        ));
        let data = WaypointData::load(path).expect("load");
        assert_eq!(data.holder1_standby.len(), 7);
        assert_eq!(data.above_y_offset, -0.005);
        assert_eq!(data.retreat_z_offset, -0.05);
        assert_eq!(data.holder1_on_y_offset, 0.0005);
        assert_eq!(data.holder_multi_x_offsets.len(), 9);
        assert_eq!(data.wrist3_rotation_offset, 0.0);
    }

    #[test]
    fn arm_joints_drops_the_gripper_slot() {
        let values = [9.9, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6];
        let joints = WaypointData::arm_joints(&values);
        assert_eq!(joints.len(), 6);
        assert_eq!(joints[0], ("shoulder_pan_joint".to_string(), 0.1));
        assert_eq!(joints[5], ("shoulder_lift_joint".to_string(), 0.6));
    }
}
