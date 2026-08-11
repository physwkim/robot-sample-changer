//! Robot model construction and the kinematics helpers the sequence uses.
//!
//! Replaces what MoveIt's `RobotModelLoader` + parameter server provided:
//! the URDF/SRDF pair comes from files named in the config, and the
//! joint-limit overrides that used to live in
//! `ur3e_hande_moveit_config/config/joint_limits.yaml` are injected here
//! (the URDF carries no acceleration limits at all, and TOTG requires
//! them).

use std::collections::BTreeMap;

use cspace_core::geometry::Isometry3;
use cspace_core::kinematics::{
    IkContext, IkTarget, NewtonRaphsonSolver, SolverParams, set_from_ik,
};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use nalgebra::Translation3;

use crate::config::Config;
use crate::error::SequencerError;
use crate::log;
use crate::waypoints::WAYPOINT_JOINT_ORDER;

/// Named joint positions, the C++ node's `std::map<std::string, double>`.
pub type JointMap = BTreeMap<String, f64>;

pub struct Model {
    pub robot: RobotModel,
    pub srdf: SrdfModel,
    pub group: String,
    pub ik_frame: String,
}

impl Model {
    pub fn load(config: &Config) -> Result<Self, SequencerError> {
        let urdf_xml = std::fs::read_to_string(&config.robot.urdf).map_err(|e| {
            SequencerError(format!(
                "cannot read URDF {}: {e}",
                config.robot.urdf.display()
            ))
        })?;
        let urdf = urdf_rs::read_from_string(&urdf_xml)
            .map_err(|e| SequencerError(format!("cannot parse URDF: {e}")))?;
        let srdf = SrdfModel::parse_file(&config.robot.srdf)
            .map_err(|e| SequencerError(format!("cannot parse SRDF: {e}")))?;
        let search_paths = MeshSearchPaths::new(
            config
                .robot
                .mesh_packages
                .iter()
                .map(|(name, dir)| (name.clone(), dir.clone())),
        );
        let mut robot = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &search_paths)
            .map_err(|e| SequencerError(format!("cannot build robot model: {e}")))?;

        for name in config.joint_limits.position_overrides.keys() {
            if !WAYPOINT_JOINT_ORDER.contains(&name.as_str()) {
                return Err(SequencerError(format!(
                    "joint_limits.position_overrides names unknown arm joint '{name}'"
                )));
            }
        }
        for name in WAYPOINT_JOINT_ORDER {
            let joint = robot
                .joint_model_mut(name)
                .map_err(|e| SequencerError(format!("arm joint '{name}' not in model: {e}")))?;
            let mut limits = joint.variable_bounds_msg();
            for limit in &mut limits {
                limit.has_velocity_limits = true;
                limit.max_velocity = config.joint_limits.max_velocity;
                limit.has_acceleration_limits = true;
                limit.max_acceleration = config.joint_limits.max_acceleration;
                if let Some([min, max]) = config.joint_limits.position_overrides.get(name) {
                    limit.has_position_limits = true;
                    limit.min_position = *min;
                    limit.max_position = *max;
                }
            }
            joint.set_variable_bounds_from_limits(&limits);
        }

        Ok(Self {
            robot,
            srdf,
            group: config.robot.group.clone(),
            ik_frame: config.robot.ik_frame.clone(),
        })
    }

    /// A state at model defaults with `joints` applied, forward kinematics
    /// up to date.
    pub fn state_with_joints(&self, joints: &JointMap) -> Result<RobotState<'_>, SequencerError> {
        let mut state = RobotState::new(&self.robot);
        state.set_to_default_values();
        for (name, value) in joints {
            state
                .set_variable_position(name, *value)
                .map_err(|e| SequencerError(format!("cannot set joint '{name}': {e}")))?;
        }
        state.update();
        Ok(state)
    }

    /// Pose of `ik_frame` at `joints`, in the model frame.
    pub fn fk(&self, joints: &JointMap) -> Result<Isometry3, SequencerError> {
        let mut state = self.state_with_joints(joints)?;
        let pose = state
            .update()
            .global_link_transform(&self.ik_frame)
            .map_err(|e| SequencerError(format!("FK to '{}' failed: {e}", self.ik_frame)))?;
        Ok(pose)
    }

    /// Fresh IK solver for the arm group. Constructed per solve so results
    /// do not depend on RNG state left behind by earlier solves.
    pub fn solver(&self) -> Result<NewtonRaphsonSolver, SequencerError> {
        NewtonRaphsonSolver::new(&self.robot, &self.group, &SolverParams::default())
            .map_err(|e| SequencerError(format!("cannot build IK solver: {e}")))
    }

    /// Port of the C++ `apply_cartesian_offset_to_joints`: offset the
    /// `ik_frame` pose in its own (TCP-local) frame — with `z_global`, the
    /// z component is applied along world z instead — and solve IK back to
    /// joints. On IK failure this warns and returns the original joints
    /// unchanged, exactly as the C++ did; errors are reserved for
    /// structural problems (unknown joint/link/group names).
    ///
    /// Deviation: the C++ gave its KDL solver a 2 s wall-clock timeout;
    /// `NewtonRaphsonSolver` bounds work by iteration/restart counts
    /// (`SolverParams::default`) instead.
    pub fn apply_cartesian_offset(
        &self,
        original: &JointMap,
        offset: [f64; 3],
        z_global: bool,
        label: &str,
    ) -> Result<JointMap, SequencerError> {
        let [x, y, z] = offset;
        if x.abs() < 1e-6 && y.abs() < 1e-6 && z.abs() < 1e-6 {
            return Ok(original.clone());
        }

        let mut state = self.state_with_joints(original)?;
        let current_pose = state
            .update()
            .global_link_transform(&self.ik_frame)
            .map_err(|e| SequencerError(format!("FK to '{}' failed: {e}", self.ik_frame)))?;

        let target_pose = if z_global {
            let mut pose = current_pose * Translation3::new(x, y, 0.0);
            pose.translation.vector.z += z;
            pose
        } else {
            current_pose * Translation3::new(x, y, z)
        };

        let mut solver = self.solver()?;
        let solved = set_from_ik(
            &mut state,
            &mut solver,
            &[IkTarget {
                pose: target_pose,
                frame: &self.ik_frame,
            }],
            &mut IkContext::default(),
        )
        .map_err(|e| SequencerError(format!("{label}: IK error: {e}")))?;

        if !solved {
            log::warn(&format!(
                "{label}: IK failed for Cartesian offset, using original joints"
            ));
            return Ok(original.clone());
        }

        let mut updated = JointMap::new();
        for name in original.keys() {
            let value = state
                .variable_position(name)
                .map_err(|e| SequencerError(format!("cannot read joint '{name}': {e}")))?;
            updated.insert(name.clone(), value);
        }
        Ok(updated)
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;
    use crate::waypoints::WaypointData;

    fn production_model() -> (Model, Config) {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let config = Config::load(path).expect("load config");
        let model = Model::load(&config).expect("load model");
        (model, config)
    }

    /// Full stack: URDF + SRDF + meshes parse, the group exists with the
    /// expected joints, and the injected limits landed.
    #[test]
    fn loads_the_production_model() {
        let (model, config) = production_model();
        let group = model
            .robot
            .joint_model_group(&model.group)
            .expect("arm group");
        assert_eq!(group.variable_names().len(), 6);
        for name in WAYPOINT_JOINT_ORDER {
            let joint = model.robot.joint_model(name).expect("arm joint");
            for limit in joint.variable_bounds_msg() {
                assert!(limit.has_velocity_limits, "{name} velocity limit");
                assert_eq!(limit.max_velocity, config.joint_limits.max_velocity);
                assert!(limit.has_acceleration_limits, "{name} acceleration limit");
                assert_eq!(limit.max_acceleration, config.joint_limits.max_acceleration);
            }
        }
    }

    /// FK to the ik_frame works on a taught waypoint, and a small local
    /// offset solves back through IK to different joints whose FK moved
    /// by the requested amount.
    #[test]
    fn cartesian_offset_round_trips_through_ik() {
        let (model, config) = production_model();
        let waypoints = WaypointData::load(&config.sequence.waypoints_yaml).expect("waypoints");
        let joints: JointMap = WaypointData::arm_joints(&waypoints.holder1_standby)
            .into_iter()
            .collect();

        let before = model.fk(&joints).expect("fk");
        let offset = [0.0, 0.003, 0.0];
        let shifted = model
            .apply_cartesian_offset(&joints, offset, false, "test")
            .expect("offset");
        assert_ne!(shifted, joints, "IK fell back to the original joints");
        let after = model.fk(&shifted).expect("fk after");

        // The offset is expressed in the ik_frame's local frame.
        let expected = before * nalgebra::Translation3::new(offset[0], offset[1], offset[2]);
        let error = (after.translation.vector - expected.translation.vector).norm();
        assert!(error < 1e-4, "position error {error} m");
        assert!(after.rotation.angle_to(&expected.rotation) < 1e-3);
    }

    /// Zero offset must return the input unchanged without invoking IK.
    #[test]
    fn zero_offset_is_identity() {
        let (model, config) = production_model();
        let waypoints = WaypointData::load(&config.sequence.waypoints_yaml).expect("waypoints");
        let joints: JointMap = WaypointData::arm_joints(&waypoints.sample_holder_standby)
            .into_iter()
            .collect();
        let out = model
            .apply_cartesian_offset(&joints, [0.0, 0.0, 0.0], false, "test")
            .expect("offset");
        assert_eq!(out, joints);
    }
}
