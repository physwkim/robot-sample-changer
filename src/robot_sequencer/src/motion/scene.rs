//! The collision world the planner sees, and the path constraint it
//! plans under.

use std::sync::Arc;

use cspace_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use cspace_core::geometry::Transforms;
use cspace_core::geometry::{Isometry3, Shape, Vector3, mesh_from_bytes};
use cspace_core::state::RobotState;
use cspace_planning::constraints::{
    Constraint, KinematicConstraintSet, OrientationConstraint, OrientationTolerance,
};
use cspace_planning::scene::PlanningScene;
// Linked for its side effect: RrtConnectManager registers itself into
// PLANNER_MANAGERS via linkme; without this the linker drops the
// registration and resolve_planner("rrt_connect") fails.
use cspace_planners as _;
use nalgebra::{Translation3, UnitQuaternion};

use crate::config::Config;
use crate::error::SequencerError;
use crate::model::Model;

pub(crate) struct SceneAsset {
    id: String,
    shape: Arc<Shape>,
    pose: Isometry3,
}

/// The level-tool path constraint, resolved once at connect.
///
/// The reference is [`LEVEL_TOOL_REFERENCE`]: a -90-degree rotation
/// about world X, which is level in the `ik_frame`'s own convention —
/// that frame's Y axis points straight down, and its Z axis (the
/// approach direction) lies in the horizontal plane. So "the tool stays
/// level" is "the deviation from this reference is a rotation about Y",
/// which is what leaving the Y tolerance wide and pinching X and Z says.
///
/// Take the sign seriously. The UR's `actual_TCP_pose` reports the same
/// physical poses with the vertical axis inverted relative to the URDF
/// link, and a reference derived from RTDE instead of the model is 180
/// degrees off about X — which reads as a violation at every taught
/// pose and fails planning with "no goal state satisfying the goal
/// constraints". `taught_poses_are_level` pins the model's convention.
///
/// `RotationVector` over `XyzEuler` because the deviation here runs to
/// +-90 degrees about one axis, which is exactly where the Euler
/// decomposition's pitch singularity sits.
pub(crate) struct LevelToolConstraint {
    link_name: String,
    tolerance_rad: f64,
}

/// The level `ik_frame` orientation: see [`LevelToolConstraint`].
fn level_tool_reference() -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&nalgebra::Vector3::x_axis(), -std::f64::consts::FRAC_PI_2)
}

impl LevelToolConstraint {
    pub(super) fn new(link_name: &str, tolerance_deg: f64) -> Self {
        Self {
            link_name: link_name.to_string(),
            tolerance_rad: tolerance_deg.to_radians(),
        }
    }

    pub(super) fn build(
        &self,
        model: &Model,
        tf: &Transforms,
    ) -> Result<KinematicConstraintSet, cspace_core::error::Error> {
        let constraint = OrientationConstraint::new(
            &model.robot,
            tf,
            &self.link_name,
            model.robot.model_frame(),
            level_tool_reference(),
            OrientationTolerance::RotationVector {
                x: self.tolerance_rad,
                y: self.tolerance_rad,
                // Free: the world vertical. `decide()` maps the deviation
                // back through `desired_r_in_frame_id` before comparing
                // (orientation.rs, the RotationVector arm), so these
                // tolerances are on world axes, not on the reference
                // frame's. Tilt is therefore x and y, and the 91-degree
                // holder-to-stage turn lands entirely on z.
                z: std::f64::consts::PI,
            },
            1.0,
        )?;
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Orientation(constraint));
        Ok(set)
    }
}

pub(crate) fn load_scene_assets(config: &Config) -> Result<Vec<SceneAsset>, SequencerError> {
    let mut assets = Vec::new();
    for object in &config.scene.objects {
        let bytes = std::fs::read(&object.stl)
            .map_err(|e| SequencerError(format!("cannot read {}: {e}", object.stl.display())))?;
        let scale = Vector3::new(object.scale[0], object.scale[1], object.scale[2]);
        let mesh = mesh_from_bytes(&bytes, scale)
            .map_err(|e| SequencerError(format!("cannot parse {}: {e}", object.stl.display())))?;
        let [x, y, z] = object.position;
        let [roll, pitch, yaw] = object.rpy;
        let pose = Isometry3::from_parts(
            Translation3::new(x, y, z),
            UnitQuaternion::from_euler_angles(roll, pitch, yaw),
        );
        assets.push(SceneAsset {
            id: object.id.clone(),
            shape: Arc::new(Shape::Mesh(mesh)),
            pose,
        });
    }
    Ok(assets)
}

/// Planning scene (SRDF ACM + configured allowances) and the collision
/// backend that carries the scene objects. The WORLD lives in the env —
/// `ParryCollisionEnv` is built over a `World`, and shapes added to the
/// `PlanningScene` alone are never collision-checked (its world is not
/// the backend's). The ACM entries still key on the object ids. The
/// caller sets the robot state on the scene.
pub(crate) fn scene_with_assets<'m>(
    model: &'m Model,
    assets: &[SceneAsset],
    allow_collisions_with: &[String],
) -> (PlanningScene<'m>, ParryCollisionEnv) {
    let mut scene = PlanningScene::new(&model.robot, &model.srdf);
    let acm = scene.allowed_collision_matrix_mut();
    let mut world = World::new();
    for asset in assets {
        world.add_shape(&asset.id, Arc::clone(&asset.shape), asset.pose);
        for name in allow_collisions_with {
            acm.set_entry(&asset.id, name, true);
        }
    }
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
    (scene, env)
}

/// Index of the first state that collides (self or scene world, ACM
/// applied), or `None` when the whole path is clear. Used by the jog
/// gate; see the comment at its call site for the C++ split this
/// preserves.
pub(crate) fn first_collision_index(
    model: &Model,
    assets: &[SceneAsset],
    allow_collisions_with: &[String],
    states: &[RobotState<'_>],
) -> Result<Option<usize>, SequencerError> {
    let (mut scene, env) = scene_with_assets(model, assets, allow_collisions_with);
    let request = CollisionRequest::default();
    for (i, state) in states.iter().enumerate() {
        let q = state
            .joint_group_positions(&model.group)
            .map_err(|e| SequencerError(format!("jog: state positions: {e}")))?;
        scene
            .current_state_mut()
            .set_joint_group_positions(&model.group, &q)
            .map_err(|e| SequencerError(format!("jog: scene state: {e}")))?;
        scene.current_state_mut().update();
        if scene.check_collision(&env, &request).collision {
            return Ok(Some(i));
        }
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;
    use crate::model::JointMap;
    use crate::waypoints::WaypointData;

    fn production_model_and_state() -> (Model, JointMap) {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let config = Config::load(path).expect("load config");
        let model = Model::load(&config).expect("load model");
        let waypoints = WaypointData::load(&config.sequence.waypoints_yaml).expect("waypoints");
        let joints: JointMap = WaypointData::arm_joints(&waypoints.holder1_standby)
            .into_iter()
            .collect();
        (model, joints)
    }

    /// Every taught pose the joint-space steps plan to must satisfy the
    /// level-tool constraint, or the goal is unreachable under it and
    /// planning fails outright with "no goal state satisfying the goal
    /// constraints" — which is exactly what a reference taken from the
    /// UR's `actual_TCP_pose` produces, that frame being inverted about
    /// X relative to the `ik_frame`. This also catches teaching drift:
    /// re-teaching a pose off level silently disables the steps that
    /// plan to it.
    #[test]
    fn taught_poses_are_level() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let config = Config::load(path).expect("load config");
        let model = Model::load(&config).expect("load model");
        let waypoints = WaypointData::load(&config.sequence.waypoints_yaml).expect("waypoints");
        let tolerance = config.sequence.level_tool.tolerance_deg.to_radians();

        for (label, taught) in [
            ("holder1_standby", &waypoints.holder1_standby),
            ("sample_holder_standby", &waypoints.sample_holder_standby),
        ] {
            let joints: JointMap = WaypointData::arm_joints(taught).into_iter().collect();
            let pose = model.fk(&joints).expect("fk");
            // Mirrors OrientationConstraint::decide's RotationVector arm:
            // the deviation's rotation vector mapped back through the
            // reference, which puts the per-axis tolerances on world
            // axes. Comparing the deviation in the reference frame
            // instead — the natural-looking reading — points at the
            // wrong free axis and passes while planning fails.
            let reference = level_tool_reference();
            let deviation = (reference.inverse() * pose.rotation).scaled_axis();
            let world = reference.to_rotation_matrix() * deviation;
            assert!(
                world.x.abs() <= tolerance && world.y.abs() <= tolerance,
                "{label} is not level: tilt [{:.2}, {:.2}] deg about world x/y, tolerance \
                 {:.2} deg (z, the vertical, is free at {:.2} deg)",
                world.x.to_degrees(),
                world.y.to_degrees(),
                tolerance.to_degrees(),
                world.z.to_degrees(),
            );
        }
    }

    /// The committed stage parts must never be more permissive than the
    /// CAD mesh they approximate. `compute_exact_convex_hulls` takes the
    /// hull of each partition of the original geometry and a hull
    /// contains its set, so the union of parts contains the mesh and
    /// this holds by construction — but the parts are a committed
    /// artifact, and regenerating them with looser settings, or against
    /// a re-exported mesh, would not announce itself. Missing a
    /// collision is the failure that ends with the arm inside the stage.
    #[test]
    #[ignore = "loads the 231k-triangle CAD mesh; ~4 minutes"]
    fn stage_parts_are_never_more_permissive_than_the_cad_mesh() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let mut config = Config::load(path).expect("config");
        let model = Model::load(&config).expect("model");
        let placement = &config.scene.objects[0];
        let (scale, position, rpy) = (placement.scale, placement.position, placement.rpy);

        let parts = load_scene_assets(&config).expect("parts");
        config.scene.objects = vec![crate::config::SceneObject {
            id: "stage".into(),
            stl: Path::new(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../resources/stage.stl"
            ))
            .to_path_buf(),
            scale,
            position,
            rpy,
        }];
        let cad = load_scene_assets(&config).expect("cad mesh");

        let hit = |assets: &[SceneAsset], q: &[f64; 6]| {
            let (mut scene, env) =
                scene_with_assets(&model, assets, &config.scene.allow_collisions_with);
            scene
                .current_state_mut()
                .set_joint_group_positions(&model.group, q)
                .expect("set joints");
            scene.current_state_mut().update();
            scene
                .check_collision(&env, &CollisionRequest::default())
                .collision
        };

        // Sweep the three joints that carry the arm across the stage,
        // wrist held at the attitude every taught pose shares.
        let mut missed = Vec::new();
        let mut checked = 0;
        for a in (-8..=8).step_by(2) {
            for b in (-6..=0).step_by(2) {
                for c in (-6..=6).step_by(3) {
                    let q = [
                        f64::from(a) * 0.2,
                        f64::from(b) * 0.25,
                        f64::from(c) * 0.25,
                        -3.4,
                        -1.2,
                        0.0,
                    ];
                    checked += 1;
                    if hit(&cad, &q) && !hit(&parts, &q) {
                        missed.push(q);
                    }
                }
            }
        }
        assert!(
            missed.is_empty(),
            "stage parts missed {} of {checked} collisions the CAD mesh reports, e.g. {:?}",
            missed.len(),
            missed.first(),
        );
    }

    /// Closed axis-aligned cube (12 triangles) as binary STL bytes,
    /// centered at the origin with half-extent `half` meters. Normals are
    /// irrelevant to the collision mesh, so they are left zeroed.
    fn cube_stl(half: f32) -> Vec<u8> {
        let h = half;
        let v = [
            [-h, -h, -h],
            [h, -h, -h],
            [h, h, -h],
            [-h, h, -h],
            [-h, -h, h],
            [h, -h, h],
            [h, h, h],
            [-h, h, h],
        ];
        // Two triangles per face, vertex indices into `v`.
        let faces: [[usize; 3]; 12] = [
            [0, 1, 2],
            [0, 2, 3],
            [4, 6, 5],
            [4, 7, 6],
            [0, 4, 5],
            [0, 5, 1],
            [3, 2, 6],
            [3, 6, 7],
            [0, 3, 7],
            [0, 7, 4],
            [1, 5, 6],
            [1, 6, 2],
        ];
        let mut bytes = vec![0u8; 80];
        bytes.extend_from_slice(&(faces.len() as u32).to_le_bytes());
        for face in faces {
            bytes.extend_from_slice(&[0u8; 12]); // normal
            for idx in face {
                for coord in v[idx] {
                    bytes.extend_from_slice(&coord.to_le_bytes());
                }
            }
            bytes.extend_from_slice(&[0u8; 2]); // attribute byte count
        }
        bytes
    }

    fn cube_asset(id: &str, center: Isometry3) -> SceneAsset {
        let mesh = mesh_from_bytes(&cube_stl(0.05), Vector3::new(1.0, 1.0, 1.0)).expect("cube");
        SceneAsset {
            id: id.into(),
            shape: Arc::new(Shape::Mesh(mesh)),
            pose: center,
        }
    }

    /// The collision plumbing both `move_planned` and the jog gate share
    /// must SEE a scene mesh: a cube centered on the TCP collides, the
    /// same cube 3 m away does not (which also proves the taught state
    /// itself is collision-free, so the hit is the mesh, not the robot).
    #[test]
    fn scene_mesh_collision_is_detected_at_the_tcp() {
        let (model, joints) = production_model_and_state();
        let mut state = model.state_with_joints(&joints).expect("state");
        let tcp = state
            .update()
            .global_link_transform(&model.ik_frame)
            .expect("fk");

        let at_tcp = cube_asset("box", Isometry3::from_parts(tcp.translation, tcp.rotation));
        let states = [model.state_with_joints(&joints).expect("state")];
        assert_eq!(
            first_collision_index(&model, &[at_tcp], &[], &states).expect("check"),
            Some(0),
            "cube at the TCP must collide"
        );

        let far = cube_asset(
            "box",
            Isometry3::from_parts(Translation3::new(3.0, 3.0, 3.0), tcp.rotation),
        );
        assert_eq!(
            first_collision_index(&model, &[far], &[], &states).expect("check"),
            None,
            "cube 3 m away must not collide"
        );
    }

    /// The ACM allowances from the config must suppress a hit: the same
    /// TCP cube with every arm link allowed reports clear.
    #[test]
    fn acm_allowance_suppresses_the_scene_hit() {
        let (model, joints) = production_model_and_state();
        let mut state = model.state_with_joints(&joints).expect("state");
        let tcp = state
            .update()
            .global_link_transform(&model.ik_frame)
            .expect("fk");
        let at_tcp = cube_asset("box", Isometry3::from_parts(tcp.translation, tcp.rotation));

        let all_links: Vec<String> = model.robot.link_names().to_vec();
        let states = [model.state_with_joints(&joints).expect("state")];
        assert_eq!(
            first_collision_index(&model, &[at_tcp], &all_links, &states).expect("check"),
            None,
            "allowing every link must suppress the cube hit"
        );
    }
}
