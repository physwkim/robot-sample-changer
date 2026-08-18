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

        match self.ik_from_seed(original, &target_pose, label)? {
            Some(joints) => Ok(joints),
            None => {
                log::warn(&format!(
                    "{label}: IK failed for Cartesian offset, using original joints"
                ));
                Ok(original.clone())
            }
        }
    }

    /// Turns `ik_frame` about a base-frame axis through its own origin
    /// and solves IK back to joints, leaving the tool point where it is.
    ///
    /// About the tool point and not the flange, because the corrections
    /// this exists for are angular teaching errors at the grasp: the
    /// grasp point is right, the approach angle is not. Same IK-failure
    /// contract as [`Model::apply_cartesian_offset`]: warn and return
    /// the original joints.
    pub fn apply_tool_point_rotation(
        &self,
        original: &JointMap,
        axis_base: [f64; 3],
        rad: f64,
        label: &str,
    ) -> Result<JointMap, SequencerError> {
        if rad.abs() < 1e-8 {
            return Ok(original.clone());
        }

        let mut state = self.state_with_joints(original)?;
        let current_pose = state
            .update()
            .global_link_transform(&self.ik_frame)
            .map_err(|e| SequencerError(format!("FK to '{}' failed: {e}", self.ik_frame)))?;

        let axis = nalgebra::Vector3::new(axis_base[0], axis_base[1], axis_base[2]);
        let turn = nalgebra::UnitQuaternion::from_scaled_axis(axis.normalize() * rad);
        // Rotation applied in base, position kept: the tool origin is the
        // pivot.
        let target_pose =
            Isometry3::from_parts(current_pose.translation, turn * current_pose.rotation);

        match self.ik_from_seed(original, &target_pose, label)? {
            Some(joints) => Ok(joints),
            None => {
                log::warn(&format!(
                    "{label}: IK failed for tool-point rotation, using original joints"
                ));
                Ok(original.clone())
            }
        }
    }

    /// Joints putting `ik_frame` at `target`, solved from `seed` so the
    /// result stays on the seed's IK branch — a UR3e pose has up to eight
    /// solutions, and the arm reaching the right point through the wrong
    /// elbow is a different path through the cell.
    ///
    /// `Ok(None)` is non-convergence, which callers treat as "this pose is
    /// not usable" rather than a failure; errors stay reserved for
    /// structural problems (unknown joint/link/group names).
    pub fn ik_from_seed(
        &self,
        seed: &JointMap,
        target: &Isometry3,
        label: &str,
    ) -> Result<Option<JointMap>, SequencerError> {
        let mut state = self.state_with_joints(seed)?;
        let mut solver = self.solver()?;
        let solved = set_from_ik(
            &mut state,
            &mut solver,
            &[IkTarget {
                pose: *target,
                frame: &self.ik_frame,
            }],
            &mut IkContext::default(),
        )
        .map_err(|e| SequencerError(format!("{label}: IK error: {e}")))?;
        if !solved {
            return Ok(None);
        }

        let mut updated = JointMap::new();
        for name in seed.keys() {
            let value = state
                .variable_position(name)
                .map_err(|e| SequencerError(format!("cannot read joint '{name}': {e}")))?;
            updated.insert(name.clone(), value);
        }
        Ok(Some(updated))
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

    /// The wrist camera as calibrated: `T_ee_cam` plus the intrinsics,
    /// enough to ask where a point in the tool frame lands in the image.
    struct Camera {
        r: nalgebra::Rotation3<f64>,
        t: nalgebra::Vector3<f64>,
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
    }

    /// The IOC's current frame size. The 1280x720 switch is still a
    /// proposal; if it lands, the margins below only grow.
    const FRAME: (f64, f64) = (640.0, 480.0);

    impl Camera {
        fn load() -> Self {
            let text = std::fs::read_to_string(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../T_ee_cam.yaml"
            ))
            .expect("T_ee_cam.yaml");
            let v: serde_yaml::Value = serde_yaml::from_str(&text).expect("parse T_ee_cam");
            let nums = |val: &serde_yaml::Value| -> Vec<f64> {
                val.as_sequence()
                    .expect("sequence")
                    .iter()
                    .map(|x| x.as_f64().expect("number"))
                    .collect()
            };
            let ee_cam = &v["T_ee_cam"];
            let t = nums(&ee_cam["translation_m"]);
            let rows: Vec<Vec<f64>> = ee_cam["rotation_matrix"]
                .as_sequence()
                .expect("rotation_matrix")
                .iter()
                .map(nums)
                .collect();
            let k = nums(&v["camera_matrix"]);
            Self {
                r: nalgebra::Rotation3::from_matrix_unchecked(nalgebra::Matrix3::new(
                    rows[0][0], rows[0][1], rows[0][2], rows[1][0], rows[1][1], rows[1][2],
                    rows[2][0], rows[2][1], rows[2][2],
                )),
                t: nalgebra::Vector3::new(t[0], t[1], t[2]),
                fx: k[0],
                cx: k[2],
                fy: k[4],
                cy: k[5],
            }
        }

        /// Pixel coordinates of a point given in the tool frame of the
        /// pose the picture is taken from, or `None` behind the camera.
        fn project(&self, p_tool: nalgebra::Vector3<f64>) -> Option<(f64, f64)> {
            let p = self.r.inverse() * (p_tool - self.t);
            (p.z > 0.0).then(|| (self.fx * p.x / p.z + self.cx, self.fy * p.y / p.z + self.cy))
        }

        fn in_frame(&self, p_tool: nalgebra::Vector3<f64>) -> bool {
            self.project(p_tool)
                .is_some_and(|(u, v)| (0.0..FRAME.0).contains(&u) && (0.0..FRAME.1).contains(&v))
        }
    }

    /// Every vision hook must observe from a pose that has the target in
    /// the picture. The hooks used to fire at the above poses, where the
    /// grasp point projects below the bottom of the frame and the disc
    /// filling the middle belongs to the next holder up — a detector
    /// would lock onto the wrong holder with confidence. Moving them to
    /// the standby poses is what this test pins: the target in frame
    /// from standby, and still out of frame from above so the old
    /// placement cannot quietly come back.
    #[test]
    fn the_hooks_observe_from_poses_that_see_the_target() {
        let (model, config) = production_model();
        let w = WaypointData::load(&config.sequence.waypoints_yaml).expect("waypoints");
        let cam = Camera::load();
        let local = |from: &JointMap, to: &JointMap| -> nalgebra::Vector3<f64> {
            let (a, b) = (model.fk(from).unwrap(), model.fk(to).unwrap());
            a.inverse_transform_point(&nalgebra::Point3::from(b.translation.vector))
                .coords
        };
        for (tag, standby, on, above) in observation_poses(&model, &config, &w) {
            assert!(
                cam.in_frame(local(&standby, &on)),
                "{tag}: the grasp point is not in the picture from standby"
            );
            assert!(
                !cam.in_frame(local(&above, &on)),
                "{tag}: the grasp point is in the picture from above — if the camera \
                 or the above offset changed, the hooks can move back"
            );
        }
    }

    /// `(tag, standby, on_position, above)` for each pose pair a vision
    /// hook fires at, built the way `compute_run_waypoints` builds them.
    fn observation_poses(
        model: &Model,
        config: &Config,
        w: &WaypointData,
    ) -> Vec<(String, JointMap, JointMap, JointMap)> {
        let taught = |v: &[f64]| -> JointMap { WaypointData::arm_joints(v).into_iter().collect() };
        let ho = [
            w.holder1_on_x_offset,
            w.holder1_on_y_offset,
            w.holder1_on_z_offset,
        ];
        let sho = [
            w.sample_holder_on_x_offset,
            w.sample_holder_on_y_offset,
            w.sample_holder_on_z_offset,
        ];
        let h_standby0 = model
            .apply_cartesian_offset(&taught(&w.holder1_standby), ho, false, "s")
            .unwrap();
        let h_on0 = model
            .apply_cartesian_offset(&taught(&w.holder1_on_position), ho, false, "o")
            .unwrap();
        let h_on0 = model
            .apply_cartesian_offset(&h_on0, [0.0, -w.holder_on_lift, 0.0], false, "olift")
            .unwrap();
        let sh_standby = taught(&w.sample_holder_standby);
        let sh_on = model
            .apply_cartesian_offset(&taught(&w.sample_holder_on_position), sho, false, "sho")
            .unwrap();
        let sh_above = model
            .apply_cartesian_offset(&sh_on, [0.0, w.above_y_offset, 0.0], false, "sha")
            .unwrap();

        // Holders 1 and 10 are the rail ends and 3 is a middle one; if
        // the framing survives the ends it survives what lies between.
        let mut out = vec![("sample_holder".to_string(), sh_standby, sh_on, sh_above)];
        for holder in [1, 3, 10] {
            let y = f64::from(holder - 1) * config.sequence.holder_offset;
            let (mut x, mut z) = (0.0, 0.0);
            if (2..=10).contains(&holder) {
                let i = (holder - 2) as usize;
                x = w.holder_multi_x_offsets.get(i).copied().unwrap_or(0.0);
                z = w.holder_multi_z_offsets.get(i).copied().unwrap_or(0.0);
            }
            let base = if holder == 10 {
                model
                    .apply_cartesian_offset(&h_standby0, [0.0, -0.005, 0.0], false, "s10")
                    .unwrap()
            } else {
                h_standby0.clone()
            };
            let standby = model
                .apply_cartesian_offset(&base, [x, y, z], false, "sb")
                .unwrap();
            let on = model
                .apply_cartesian_offset(&h_on0, [x, y, z], false, "on")
                .unwrap();
            // Mirrors compute_run_waypoints: translated untilted, then
            // pitched about this holder's own grasp point.
            let on = model
                .apply_tool_point_rotation(
                    &on,
                    [1.0, 0.0, 0.0],
                    w.holder_tilt_x_deg(holder).to_radians(),
                    "otilt",
                )
                .unwrap();
            let above = model
                .apply_cartesian_offset(&on, [0.0, w.above_y_offset, 0.0], false, "ab")
                .unwrap();
            out.push((format!("holder {holder}"), standby, on, above));
        }
        out
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
