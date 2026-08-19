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

/// Sets the scene's current state to `state`'s group positions.
fn sync_scene_state(
    scene: &mut PlanningScene<'_>,
    group: &str,
    state: &RobotState<'_>,
) -> Result<(), SequencerError> {
    let q = state
        .joint_group_positions(group)
        .map_err(|e| SequencerError(format!("collision check: state positions: {e}")))?;
    scene
        .current_state_mut()
        .set_joint_group_positions(group, &q)
        .map_err(|e| SequencerError(format!("collision check: scene state: {e}")))?;
    scene.current_state_mut().update();
    Ok(())
}

/// Index of the first state that adds a collision (self or scene world,
/// ACM applied) the path did not start with, or `None` when no state
/// does. Used by the jog gate to truncate an interpolated path; see the
/// comment at its call site for the C++ split this preserves.
///
/// Contact pairs already present at `states[0]` are exempt along the
/// whole path. The stage parts are a convex decomposition, so recesses
/// fill in, and a pose the (ungated) sequence steps legally stand in
/// can read as "inside" a part — holder 10's taught above hold reads as
/// finger-stage contact, and an absolute gate refused every jog
/// direction out of it at 0.0%. What this gate owes its callers is that
/// the motion adds no contact the start did not already have; contacts
/// the start stands in are the pose's situation, not the motion's.
pub(crate) fn first_new_collision_index(
    model: &Model,
    assets: &[SceneAsset],
    allow_collisions_with: &[String],
    states: &[RobotState<'_>],
) -> Result<Option<usize>, SequencerError> {
    let Some(start) = states.first() else {
        return Ok(None);
    };
    let (mut scene, env) = scene_with_assets(model, assets, allow_collisions_with);
    sync_scene_state(&mut scene, &model.group, start)?;
    let start_contacts = CollisionRequest {
        contacts: true,
        max_contacts: usize::MAX,
        max_contacts_per_pair: 1,
        ..CollisionRequest::default()
    };
    if let Some(data) = scene.check_collision(&env, &start_contacts).contacts {
        let acm = scene.allowed_collision_matrix_mut();
        for (a, b) in data.by_pair.keys() {
            acm.set_entry(a, b, true);
        }
    }
    let request = CollisionRequest::default();
    for (i, state) in states.iter().enumerate().skip(1) {
        sync_scene_state(&mut scene, &model.group, state)?;
        if scene.check_collision(&env, &request).collision {
            return Ok(Some(i));
        }
    }
    Ok(None)
}

/// Whether `state` collides at all (self or scene world, ACM applied) —
/// the absolute question, as opposed to the gate's start-relative one.
/// Tests document scene properties with it; production motion has no
/// caller because every gated path asks [`first_new_collision_index`].
#[cfg(test)]
pub(crate) fn state_collides(
    model: &Model,
    assets: &[SceneAsset],
    allow_collisions_with: &[String],
    state: &RobotState<'_>,
) -> Result<bool, SequencerError> {
    let (mut scene, env) = scene_with_assets(model, assets, allow_collisions_with);
    sync_scene_state(&mut scene, &model.group, state)?;
    Ok(scene
        .check_collision(&env, &CollisionRequest::default())
        .collision)
}

/// One interpolated sample of a candidate shortcut splice, held to the
/// same two judgments the planner held its own edges to: the path
/// constraint and the collision world.
fn shortcut_sample_is_valid(
    scene: &mut PlanningScene<'_>,
    env: &ParryCollisionEnv,
    request: &CollisionRequest,
    group: &str,
    constraints: Option<&KinematicConstraintSet>,
    q: &[f64],
) -> Result<bool, SequencerError> {
    let state = scene.current_state_mut();
    state
        .set_joint_group_positions(group, q)
        .map_err(|e| SequencerError(format!("shortcut: scene state: {e}")))?;
    let posed = state.update();
    if let Some(set) = constraints
        && !set.decide(&posed).satisfied
    {
        return Ok(false);
    }
    Ok(!scene.check_collision(env, request).collision)
}

/// The planner's answer, shortened: a greedy farthest-first pass over the
/// waypoints of a solved path, keeping a waypoint only when no later one
/// is reachable from it on a straight joint-space segment. RRT-Connect
/// returns its solution unsimplified — the port left OMPL's
/// `simplifySolution` behind (see cspace-planners' registry) — so two
/// configurations a few degrees apart can be joined through most of a
/// metre of TCP swing; this closes that gap where the plan is consumed.
///
/// Candidate segments are sampled every `resolution` of joint-space
/// Euclidean distance — pass the planner's own motion-validator spacing,
/// so a spliced edge is checked as finely as the edges it removes. The
/// pass is deterministic (no random shortcut sampling): a given path
/// always shortens to the same result. Original adjacent edges are the
/// planner's own and are not re-validated; waypoints themselves are
/// likewise trusted, only splice interiors are sampled.
///
/// Returns the kept indices, ascending, always including the endpoints.
pub(crate) fn shortcut_keep_indices(
    model: &Model,
    assets: &[SceneAsset],
    allow_collisions_with: &[String],
    constraints: Option<&KinematicConstraintSet>,
    qs: &[Vec<f64>],
    resolution: f64,
) -> Result<Vec<usize>, SequencerError> {
    if qs.len() < 2 {
        return Ok((0..qs.len()).collect());
    }
    let (mut scene, env) = scene_with_assets(model, assets, allow_collisions_with);
    let request = CollisionRequest::default();

    let mut keep = vec![0];
    let mut i = 0;
    while i + 1 < qs.len() {
        let mut next = i + 1;
        'candidates: for j in (i + 2..qs.len()).rev() {
            let (a, b) = (&qs[i], &qs[j]);
            let distance = a
                .iter()
                .zip(b)
                .map(|(x, y)| (x - y).powi(2))
                .sum::<f64>()
                .sqrt();
            let steps = (distance / resolution).ceil().max(1.0) as usize;
            for k in 1..steps {
                let t = k as f64 / steps as f64;
                let q: Vec<f64> = a.iter().zip(b).map(|(x, y)| x + (y - x) * t).collect();
                if !shortcut_sample_is_valid(
                    &mut scene,
                    &env,
                    &request,
                    &model.group,
                    constraints,
                    &q,
                )? {
                    continue 'candidates;
                }
            }
            next = j;
            break;
        }
        keep.push(next);
        i = next;
    }
    Ok(keep)
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

    /// The guard `Motion::move_planned` runs before planning, exercised
    /// against the same `KinematicConstraintSet` the planner would plan
    /// under rather than a second reading of "level" that could drift
    /// from it.
    ///
    /// Two boundaries, because they fail differently: the taught pose is
    /// the path every mode is supposed to end on, and the tilted one is
    /// what a hand-eye capture used to leave behind — planning reported
    /// that as "start or goal state is itself invalid", which names
    /// neither the start nor the constraint and reads like a collision.
    #[test]
    fn the_level_constraint_decides_both_sides_of_the_boundary() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let config = Config::load(path).expect("load config");
        let (model, taught) = production_model_and_state();
        let constraint = LevelToolConstraint::new(
            &config.robot.ik_frame,
            config.sequence.level_tool.tolerance_deg,
        );
        let (mut scene, _env) = scene_with_assets(&model, &[], &[]);
        scene.current_state_mut().update();
        let set = constraint
            .build(&model, scene.transforms())
            .expect("build the level-tool constraint");

        let mut level = model.state_with_joints(&taught).expect("taught state");
        assert!(
            set.decide(&level.update()).satisfied,
            "the taught holder1_standby must satisfy the constraint the steps plan under"
        );

        // Off level about the tool's own axis, which is what jogging the
        // camera down at the tag produces.
        let mut tilted = taught.clone();
        *tilted.get_mut("wrist_1_joint").expect("wrist_1_joint") += 30f64.to_radians();
        let mut tilted = model.state_with_joints(&tilted).expect("tilted state");
        assert!(
            !set.decide(&tilted.update()).satisfied,
            "30 degrees off level must be rejected at a {} degree tolerance",
            config.sequence.level_tool.tolerance_deg
        );
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
        let checked = model.state_with_joints(&joints).expect("state");
        assert!(
            state_collides(&model, &[at_tcp], &[], &checked).expect("check"),
            "cube at the TCP must collide"
        );

        let far = cube_asset(
            "box",
            Isometry3::from_parts(Translation3::new(3.0, 3.0, 3.0), tcp.rotation),
        );
        assert!(
            !state_collides(&model, &[far], &[], &checked).expect("check"),
            "cube 3 m away must not collide"
        );
    }

    /// Why the probe cannot be guarded by the scene, as an assertion.
    ///
    /// The stage is an approximate convex decomposition, so thin
    /// concavities fill in and a bore stops being a hole — the config says
    /// so about `holder1_on_position` in its own words. The taught
    /// in-bore pose therefore reads as a standing collision: everything
    /// the probe does down there happens inside one fictitious contact
    /// pair, where a geometric guard is blind and only measured force can
    /// judge. (The jog gate lives with the same fact by exempting the
    /// pairs its start already stands in — `first_new_collision_index`.)
    ///
    /// If this ever fails, the decomposition has become fine enough to
    /// represent the bore and `Guard::ContactForce` should be re-examined
    /// rather than left standing on a premise that no longer holds.
    #[test]
    fn a_taught_in_bore_pose_reads_as_a_scene_collision() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let config = Config::load(path).expect("load config");
        let model = Model::load(&config).expect("load model");
        let waypoints = WaypointData::load(&config.sequence.waypoints_yaml).expect("waypoints");
        let assets = load_scene_assets(&config).expect("scene assets");

        let joints: JointMap = WaypointData::arm_joints(&waypoints.holder1_on_position)
            .into_iter()
            .collect();
        let state = model.state_with_joints(&joints).expect("state");
        assert!(
            state_collides(&model, &assets, &config.scene.allow_collisions_with, &state)
                .expect("check"),
            "holder1_on_position must read as a scene collision — that is the \
             whole reason a force probe cannot borrow the jog's scene gate"
        );
    }

    /// The shortcut must take the farthest valid splice — a free-space
    /// dogleg collapses to its endpoints — and must refuse it for exactly
    /// the two reasons a splice can be wrong: a scene collision on the
    /// spliced segment, and a path-constraint violation on it. The blocked
    /// variants keep the via point, and each is paired with its unblocked
    /// twin so the test can only pass if the block is the cube (or the
    /// constraint), not something about the states.
    #[test]
    fn shortcut_splices_only_segments_the_planner_would_have_accepted() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let config = Config::load(path).expect("load config");
        let (model, taught) = production_model_and_state();
        let group_q = |joints: &JointMap| -> Vec<f64> {
            model
                .state_with_joints(joints)
                .expect("state")
                .joint_group_positions(&model.group)
                .expect("positions")
        };
        let q0 = group_q(&taught);

        // A dogleg: via lifts the shoulder, endpoint pans it. The straight
        // q0 -> q2 segment is a pure pan sweep in free air.
        let mut via = taught.clone();
        *via.get_mut("shoulder_lift_joint").expect("lift") += 0.2;
        let mut panned = taught.clone();
        *panned.get_mut("shoulder_pan_joint").expect("pan") += 0.3;
        let qs = [q0.clone(), group_q(&via), group_q(&panned)];

        assert_eq!(
            shortcut_keep_indices(&model, &[], &[], None, &qs, 0.05).expect("shortcut"),
            vec![0, 2],
            "a free-space dogleg must collapse to its endpoints"
        );

        // Same dogleg with a cube parked at the TCP of the splice's
        // midpoint: the pan sweep now passes through it, the planner's own
        // adjacent edges do not (the via lifts over), so the via survives.
        let mut mid = taught.clone();
        *mid.get_mut("shoulder_pan_joint").expect("pan") += 0.15;
        let mut mid_state = model.state_with_joints(&mid).expect("mid state");
        let mid_tcp = mid_state
            .update()
            .global_link_transform(&model.ik_frame)
            .expect("fk");
        let block = cube_asset(
            "block",
            Isometry3::from_parts(mid_tcp.translation, mid_tcp.rotation),
        );
        assert_eq!(
            shortcut_keep_indices(&model, &[block], &[], None, &qs, 0.05).expect("shortcut"),
            vec![0, 1, 2],
            "a cube on the spliced segment must keep the via point"
        );

        // A splice that tilts through the level-tool band: q0 and a
        // wrist_1-rotated endpoint. Without the constraint the segment
        // is a plain free-air wrist sweep and splices; with it, the
        // interior tilt (up to 30 degrees) must be rejected.
        let mut tilted = taught.clone();
        *tilted.get_mut("wrist_1_joint").expect("wrist_1") += 30f64.to_radians();
        let qs_tilting = [q0.clone(), qs[1].clone(), group_q(&tilted)];

        assert_eq!(
            shortcut_keep_indices(&model, &[], &[], None, &qs_tilting, 0.05).expect("shortcut"),
            vec![0, 2],
            "without the constraint the wrist sweep must splice"
        );

        let constraint = LevelToolConstraint::new(
            &config.robot.ik_frame,
            config.sequence.level_tool.tolerance_deg,
        );
        let (mut scene, _env) = scene_with_assets(&model, &[], &[]);
        scene.current_state_mut().update();
        let set = constraint
            .build(&model, scene.transforms())
            .expect("build the level-tool constraint");
        assert_eq!(
            shortcut_keep_indices(&model, &[], &[], Some(&set), &qs_tilting, 0.05)
                .expect("shortcut"),
            vec![0, 1, 2],
            "under the level-tool constraint the tilting splice must be refused"
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
        let checked = model.state_with_joints(&joints).expect("state");
        assert!(
            !state_collides(&model, &[at_tcp], &all_links, &checked).expect("check"),
            "allowing every link must suppress the cube hit"
        );
    }

    /// The gate is relative to its start: a pose standing in a scene
    /// contact (as every hull-filled recess pose does) must still be
    /// allowed to move within that contact, and must still be refused a
    /// contact it did not start with. Cube "hull" stands in for the
    /// convex part the start is inside; cube "metal" for what a bad jog
    /// would enter. The pan span matches the shortcut test's dogleg —
    /// far enough that "metal" at the end pose cannot reach the start.
    #[test]
    fn a_start_contact_is_exempt_but_a_new_contact_is_not() {
        let (model, taught) = production_model_and_state();
        let mut panned = taught.clone();
        *panned.get_mut("shoulder_pan_joint").expect("pan") += 0.3;

        let tcp_of = |joints: &JointMap| {
            let mut state = model.state_with_joints(joints).expect("state");
            state
                .update()
                .global_link_transform(&model.ik_frame)
                .expect("fk")
        };
        let start_tcp = tcp_of(&taught);
        let end_tcp = tcp_of(&panned);
        let hull = || {
            cube_asset(
                "hull",
                Isometry3::from_parts(start_tcp.translation, start_tcp.rotation),
            )
        };
        let metal = cube_asset(
            "metal",
            Isometry3::from_parts(end_tcp.translation, end_tcp.rotation),
        );
        let path = [
            model.state_with_joints(&taught).expect("state"),
            model.state_with_joints(&panned).expect("state"),
        ];

        assert_eq!(
            first_new_collision_index(&model, &[hull()], &[], &path).expect("check"),
            None,
            "motion within the start's own contact must pass the gate"
        );
        assert_eq!(
            first_new_collision_index(&model, &[hull(), metal], &[], &path).expect("check"),
            Some(1),
            "a contact the start did not stand in must still be refused"
        );
    }
}

#[cfg(test)]
mod run_waypoint_goals {
    use std::path::Path;

    use super::*;
    use crate::config::Config;
    use crate::model::{JointMap, Model};
    use crate::waypoints::WaypointData;

    /// Every holder standby the run computes is a planned-move goal, so it
    /// must stay on the taught configuration branch, land on its exact
    /// target pose, and clear the scene. Regression for holder 10
    /// (2026-08-18): its 275 mm standby offset failed the seed-local IK
    /// solve, and a random restart converged onto a flipped configuration
    /// (wrist_3 off by pi, forearm in the stage) — which planning then
    /// rightly reported as "no goal state satisfying the goal
    /// constraints", reading like an unreachable holder instead of what
    /// it was.
    #[test]
    fn computed_standbys_stay_on_the_taught_branch_and_clear_the_scene() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let config = Config::load(path).expect("config");
        let model = Model::load(&config).expect("model");
        let w = WaypointData::load(&config.sequence.waypoints_yaml).expect("waypoints");
        let assets = load_scene_assets(&config).expect("scene assets");
        let taught = |v: &[f64]| -> JointMap { WaypointData::arm_joints(v).into_iter().collect() };

        let ho = [w.rack_x_offset, w.rack_y_offset, w.rack_z_offset];
        let h_standby0 = model
            .apply_cartesian_offset(&taught(&w.holder1_standby), ho, false, "holder1_standby")
            .expect("h_standby0");

        let mut previous = h_standby0.clone();
        for holder in 1i32..=10 {
            let mut y = f64::from(holder - 1) * config.sequence.holder_offset;
            let (mut x, mut z) = (0.0, 0.0);
            if (1..=10).contains(&holder) {
                let i = (holder - 1) as usize;
                x = w.holder_multi_x_offsets.get(i).copied().unwrap_or(0.0);
                y += w.holder_multi_y_offsets.get(i).copied().unwrap_or(0.0);
                z = w.holder_multi_z_offsets.get(i).copied().unwrap_or(0.0);
            }
            // The compute_run_waypoints chain, holder-10 end nudge included.
            let base = if holder == 10 {
                model
                    .apply_cartesian_offset(
                        &h_standby0,
                        [0.0, -0.005, 0.0],
                        false,
                        "holder10_standby",
                    )
                    .expect("s10 nudge")
            } else {
                h_standby0.clone()
            };
            let standby = model
                .apply_cartesian_offset(&base, [x, y, z], false, "standby")
                .expect("standby");

            // On target: the silent IK-failure fallback returns the seed
            // joints, which this catches as a pose mismatch.
            let target = model.fk(&base).expect("fk base") * Translation3::new(x, y, z);
            let pose = model.fk(&standby).expect("fk standby");
            let miss = (pose.translation.vector - target.translation.vector).norm();
            assert!(
                miss < 1e-4 && pose.rotation.angle_to(&target.rotation) < 1e-3,
                "holder {holder} standby missed its target by {:.3} mm",
                miss * 1e3
            );

            // On branch: holders sit 30 mm apart, so adjacent standbys
            // must be adjacent configurations — the measured progression
            // moves no joint more than 0.21 rad per holder, while the
            // restart flip jumped wrist_3 by pi and shoulder_lift by
            // 1.1 rad between holders 9 and 10.
            for (name, value) in &standby {
                let step = (value - previous.get(name).unwrap()).abs();
                assert!(
                    step < 0.5,
                    "holder {holder} standby left the taught branch: {name} moved \
                     {step:.2} rad from holder {}",
                    holder - 1
                );
            }
            previous = standby.clone();

            let state = model.state_with_joints(&standby).expect("state");
            assert!(
                !state_collides(&model, &assets, &config.scene.allow_collisions_with, &state)
                    .expect("check"),
                "holder {holder} standby collides with the scene"
            );
        }
    }
}
