//! Robot bring-up and motion execution: ur-driver carries the trajectories
//! that cspace plans, replacing the UR ROS driver + MoveIt move_group pair.
//!
//! Bring-up is a Result-returning port of ur-rs's live-robot gate helper
//! (dashboard normalize → headless external-control program → reverse /
//! trajectory / script-command interfaces). Execution is the trajectory
//! point-forwarding idiom from the Phase 7 rehearsal: declare the
//! trajectory, stream TOTG samples as quintic spline segments, keep the
//! program alive with RTDE-paced NOOPs until the end callback, then
//! re-park the program so it survives arbitrarily long pauses.
//!
//! Split by what each part talks to: [`bringup`] to the dashboard and
//! the program interfaces, [`execute`] to the trajectory stream,
//! [`scene`] to the collision world and the planner's constraints. What
//! stays here is the type they share and the motion API the sequence
//! calls.

mod bringup;
mod execute;
mod probe;
pub use probe::{Bracket, Centring, MIN_EXECUTABLE_MM, Probed, TiltLimits, Tilted};
mod scene;

pub(crate) use probe::ProbeLimits;
pub(crate) use scene::LevelToolConstraint;
pub(crate) use scene::{
    SceneAsset, first_new_collision_index, load_scene_assets, scene_with_assets,
    shortcut_keep_indices,
};

use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI32};
use std::time::{Duration, Instant};

use cspace_core::geometry::{Isometry3, Vector3};
use cspace_core::kinematics::{CartesianInterpolator, IkContext, MaxEefStep};
use cspace_core::state::RobotState;
use cspace_core::trajectory::RobotTrajectory;
use cspace_core::trajectory::trajectory_tools::apply_totg_time_parameterization;
use cspace_planning::constraints::KinematicConstraintSet;
use cspace_planning::constraints::utils::construct_goal_joint_constraints;
use cspace_planning::planner_registry::resolve_planner;
use cspace_planning::{PlannerConfigurationMap, PlanningRequest, generate_plan};
// Linked for its side effect: RrtConnectManager registers itself into
// PLANNER_MANAGERS via linkme; without this the linker drops the
// registration and resolve_planner("rrt_connect") fails.
use cspace_planners as _;
use nalgebra::{Translation3, UnitQuaternion};

use ur_driver::control::reverse_interface::ReverseInterface;
use ur_driver::control::script_command_interface::ScriptCommandInterface;
use ur_driver::control::trajectory_point_interface::TrajectoryPointInterface;
use ur_driver::types::Vector6D;

use crate::error::SequencerError;
use crate::log;
use crate::model::{JointMap, Model};
use crate::stream::RtdeStream;

/// actual_q order on the wire (URControl base→wrist). Checked against the
/// planning group's variable order at connect time.
pub const RTDE_JOINT_ORDER: [&str; 6] = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
];

/// TOTG resample step — each sample becomes one quintic spline segment.
const RESAMPLE_DT: f64 = 0.1;
/// Sample spacing (joint-space Euclidean) when validating a shortcut
/// splice — `RrtConnectManager`'s default motion-validator resolution,
/// so a spliced segment is checked as finely as the planner edges it
/// removes.
const SHORTCUT_RESOLUTION: f64 = 0.05;
/// TOTG drops a waypoint when no joint moved further than this, which is
/// upstream's guard against exactly-repeated points in planner output, not
/// a statement about what the arm can do. A trajectory left with one point
/// is executed as a trivial success — the move is reported completed and
/// never sent.
///
/// The sequence's steps are centimetres and never approach it. A probe
/// step is not: 0.20 mm is 1.03e-3 rad at `holder1_standby` and 7.5e-4 rad
/// at `sample_holder_on_position`, so the same commanded step executes at
/// one pose and vanishes at another. Measured on the arm — jogs of 0.05
/// and 0.10 mm moved 0.000 mm, 0.20 mm and above arrived at 101%.
const MIN_ANGLE_CHANGE: f64 = 1e-3;
/// [`MIN_ANGLE_CHANGE`] for moves that are deliberately smaller than the
/// de-duplication heuristic. Still far above float noise, so exactly
/// repeated waypoints are dropped as before; four orders below the
/// smallest joint change a probe step produces at any taught pose.
const FINE_MIN_ANGLE_CHANGE: f64 = 1e-8;

/// What keeps a straight-line move from driving the arm into something.
///
/// Every interpolated move names exactly one, and the two are not
/// interchangeable settings on the same motion — they are what makes two
/// motions different.
///
/// The scene is an approximate convex decomposition of the stage CAD, and
/// `config/sequencer.yaml` already records the consequence in its own
/// words: thin concavities fill in, and `holder1_on_position` reads as a
/// collision. A convex hull has no bore. Measured on the arm: at
/// `holder_on_position` a 2 mm jog straight *up*, out of the bore, was
/// refused at 0.0% — the start state itself is inside filled-in geometry,
/// so every direction fails, including the way the arm came in.
///
/// So a probe cannot be guarded by the scene, and does not need to be. It
/// is the one primitive here that measures contact instead of predicting
/// it, and it bounds its own travel besides.
#[derive(Debug, Clone, Copy)]
enum Guard {
    /// No interpolated state may collide with the scene. Correct wherever
    /// the model and the metal agree, which is everywhere the arm is not
    /// deliberately inside a feature the model cannot represent.
    Scene,
    /// The caller reads force between steps and stops on contact, with
    /// total travel bounded before the first step. Nothing here checks
    /// geometry — see [`Motion::probe_until_contact`], which supplies both
    /// halves and is the only thing allowed to ask for this.
    ContactForce,
}
/// Below this per-joint distance a move is a no-op: TOTG rejects a
/// degenerate start==goal path ("the path requires a 180 deg. turn"), and
/// MoveIt executed such plans as trivial successes.
const ALREADY_THERE_TOLERANCE: f64 = 1e-3;
/// Interpolated fraction at or above which a straight line counts as
/// followed to the end. The interpolator reports 1.0 on success; the slack
/// is for the float, not for a partially reachable line.
const FULL_LINE: f64 = 1.0 - 1e-9;
/// Slack added to a trajectory's TOTG duration before the keepalive loop
/// declares the execution hung.
const EXECUTE_TIMEOUT_MARGIN: Duration = Duration::from_secs(10);

pub struct Motion<'m> {
    model: &'m Model,
    reverse: Arc<ReverseInterface>,
    trajectory: Arc<TrajectoryPointInterface>,
    script_command: Arc<ScriptCommandInterface>,
    // Held for its side effect: the primary stream carries the running
    // program.
    /// The 30001 stream the current program was sent over. Held open:
    /// URControl can abandon a program whose sender disconnects during
    /// the load (observed 2026-08-18: a resend that closed right after
    /// the write left the program "STOPPED <unnamed>", never running).
    primary: TcpStream,
    /// The address and rendered headless program from bring-up, kept so
    /// a dead program can be resent without restarting the daemon (a
    /// restart re-activates the Hand-E, which can open the fingers on a
    /// gripped sample).
    robot_ip: String,
    full_program: String,
    rtde: RtdeStream,
    program_running: Arc<AtomicBool>,
    trajectory_done: Arc<AtomicBool>,
    last_result: Arc<AtomicI32>,
    scene_assets: Vec<SceneAsset>,
    allow_collisions_with: Vec<String>,
    /// `None` when the constraint is disabled, so the request keeps the
    /// unconstrained `path_constraints: None` it had before.
    level_tool: Option<LevelToolConstraint>,
    translation_step: f64,
    rotation_step: f64,
    min_fraction: f64,
}

fn q_to_map(q: &Vector6D) -> JointMap {
    RTDE_JOINT_ORDER
        .iter()
        .zip(q.iter())
        .map(|(name, value)| (name.to_string(), *value))
        .collect()
}

fn map_to_q(joints: &JointMap) -> Result<Vector6D, SequencerError> {
    let mut q = [0.0; 6];
    for (slot, name) in q.iter_mut().zip(RTDE_JOINT_ORDER) {
        *slot = *joints
            .get(name)
            .ok_or_else(|| SequencerError(format!("goal is missing joint '{name}'")))?;
    }
    Ok(q)
}

impl<'m> Motion<'m> {
    /// Current joint positions. The stream is borrowed only for the
    /// read: callers plan next, and planning reads nothing.
    fn fresh_q(&mut self) -> Result<Vector6D, SequencerError> {
        self.rtde.session()?.fresh_q()
    }

    /// Current joint positions, named. For callers that build their own
    /// goals relative to wherever the arm actually is rather than from a
    /// taught waypoint.
    pub fn current_joints(&mut self) -> Result<JointMap, SequencerError> {
        Ok(q_to_map(&self.fresh_q()?))
    }

    /// Straight-line `ik_frame` interpolation from `start_state` to
    /// `target`, and the fraction of that line the arm could follow.
    fn interpolate(
        &self,
        start_state: &RobotState<'m>,
        target: &Isometry3,
        what: &str,
    ) -> Result<(Vec<RobotState<'m>>, f64), SequencerError> {
        let interpolator = CartesianInterpolator::new(
            &self.model.group,
            &self.model.ik_frame,
            MaxEefStep::new(self.translation_step, self.rotation_step),
        );
        let mut solver = self.model.solver()?;
        let (states, fraction) = interpolator
            .to_pose(start_state, &mut solver, target, &mut IkContext::default())
            .map_err(|e| SequencerError(format!("{what}: Cartesian interpolation: {e}")))?;
        Ok((states, fraction.value()))
    }

    /// Times an interpolated state sequence with TOTG and runs it.
    ///
    /// `min_angle_change` is [`MIN_ANGLE_CHANGE`] for everything the
    /// sequence does; a caller passes [`FINE_MIN_ANGLE_CHANGE`] only when
    /// its move is deliberately smaller than the de-duplication heuristic
    /// and being silently dropped would be worse than any cost of keeping
    /// it.
    fn timed_execute(
        &mut self,
        states: Vec<RobotState<'m>>,
        velocity_scale: f64,
        acceleration_scale: f64,
        min_angle_change: f64,
        label: &str,
    ) -> Result<(), SequencerError> {
        let mut trajectory = RobotTrajectory::for_group_name(&self.model.robot, &self.model.group)
            .map_err(|e| SequencerError(format!("{label}: trajectory: {e}")))?;
        for state in states {
            trajectory
                .add_suffix_way_point(state, 0.0)
                .map_err(|e| SequencerError(format!("{label}: trajectory waypoint: {e}")))?;
        }
        apply_totg_time_parameterization(
            &mut trajectory,
            velocity_scale,
            acceleration_scale,
            0.1,
            RESAMPLE_DT,
            min_angle_change,
        )
        .map_err(|e| SequencerError(format!("{label}: TOTG failed: {e}")))?;

        self.execute(&trajectory, label)
    }

    /// Whether [`Motion::move_direct`] would carry the arm from `from` to
    /// `to`: the whole straight line reachable, and no state on it
    /// adding a collision the start pose does not already stand in.
    ///
    /// This is the question a caller generating candidate goals has to ask
    /// before it commits to one, and it is deliberately the *path*
    /// question rather than the goal-state one. A goal whose endpoint is
    /// clear can still be unusable because the line to it is not, and
    /// finding that out at execution time is a motion error — which exits
    /// the daemon.
    pub fn direct_path_is_clear(
        &self,
        from: &JointMap,
        to: &JointMap,
    ) -> Result<bool, SequencerError> {
        let start_state = self.model.state_with_joints(from)?;
        let target = self.model.fk(to)?;
        let (states, fraction) = self.interpolate(&start_state, &target, "path check")?;
        if fraction < FULL_LINE || states.len() < 2 {
            return Ok(false);
        }
        let hit = first_new_collision_index(
            self.model,
            &self.scene_assets,
            &self.allow_collisions_with,
            &states,
        )?;
        Ok(hit.is_none())
    }

    /// Straight-line `ik_frame` move to the pose FK gives at `goal`, with
    /// every interpolated state checked for collisions the start pose did
    /// not already stand in, and no planned fallback.
    ///
    /// For motions that are meant to stay small. [`Motion::move_planned`]
    /// answers "some collision-free path exists", not "the line asked
    /// for": RRT-Connect samples the whole joint space, and although its
    /// answer is now shortcut before execution ([`shortcut_keep_indices`]
    /// — before that, an 8-degree tool rotation during hand-eye capture
    /// measured 0.71 m of TCP swing), a spliced segment is straight in
    /// joint space, which still arcs the TCP. Interpolating instead makes
    /// the executed motion the one that was asked for.
    ///
    /// A line that cannot be followed to the end, or that collides, is an
    /// error rather than a fallback: callers vet their goals with
    /// [`Motion::direct_path_is_clear`] first, so reaching either case
    /// here means the arm is not where the caller believed it was.
    pub fn move_direct(
        &mut self,
        goal: &JointMap,
        velocity_scale: f64,
        acceleration_scale: f64,
        label: &str,
    ) -> Result<(), SequencerError> {
        let start = self.fresh_q()?;
        let goal_q = map_to_q(goal)?;
        if already_there(&start, &goal_q) {
            log::info(&format!("{label}: already at goal, skipping move"));
            return Ok(());
        }

        let start_state = self.model.state_with_joints(&q_to_map(&start))?;
        let target = self.model.fk(goal)?;
        let (states, fraction) = self.interpolate(&start_state, &target, label)?;
        if fraction < FULL_LINE {
            return Err(SequencerError(format!(
                "{label}: only {:.1}% of the straight path is reachable",
                fraction * 100.0
            )));
        }
        if let Some(i) = first_new_collision_index(
            self.model,
            &self.scene_assets,
            &self.allow_collisions_with,
            &states,
        )? {
            return Err(SequencerError(format!(
                "{label}: the straight path enters a new collision at waypoint {i} of {}",
                states.len()
            )));
        }
        if states.len() < 2 {
            return Ok(());
        }
        log::info(&format!("  Direct path: {} waypoints", states.len()));

        self.timed_execute(
            states,
            velocity_scale,
            acceleration_scale,
            MIN_ANGLE_CHANGE,
            label,
        )
    }

    /// Planned (RRT-Connect) joint-space move, the port of the MoveGroup
    /// action fallback. Plans against the stage collision scene.
    /// Shorten a freshly planned trajectory in place — see
    /// [`shortcut_keep_indices`]. Runs between planning and TOTG: the
    /// splice only removes waypoints, and TOTG re-times whatever remains,
    /// so timing never has to be touched here.
    fn shortcut(
        &self,
        trajectory: &mut RobotTrajectory<'_>,
        constraints: Option<&KinematicConstraintSet>,
        label: &str,
    ) -> Result<(), SequencerError> {
        let n = trajectory.way_point_count();
        if n <= 2 {
            return Ok(());
        }
        let mut qs = Vec::with_capacity(n);
        for i in 0..n {
            let q = trajectory
                .way_point(i)
                .map_err(|e| SequencerError(format!("{label}: shortcut: waypoint {i}: {e}")))?
                .joint_group_positions(&self.model.group)
                .map_err(|e| SequencerError(format!("{label}: shortcut: positions {i}: {e}")))?;
            qs.push(q);
        }
        let keep = shortcut_keep_indices(
            self.model,
            &self.scene_assets,
            &self.allow_collisions_with,
            constraints,
            &qs,
            SHORTCUT_RESOLUTION,
        )?;
        if keep.len() == n {
            return Ok(());
        }
        for i in (0..n).rev() {
            if !keep.contains(&i) {
                trajectory
                    .remove_way_point(i)
                    .map_err(|e| SequencerError(format!("{label}: shortcut: remove {i}: {e}")))?;
            }
        }
        log::info(&format!("  Shortcut: {n} -> {} waypoints", keep.len()));
        Ok(())
    }

    pub fn move_planned(
        &mut self,
        goal: &JointMap,
        velocity_scale: f64,
        acceleration_scale: f64,
        label: &str,
    ) -> Result<(), SequencerError> {
        let start = self.fresh_q()?;
        let goal_q = map_to_q(goal)?;
        if already_there(&start, &goal_q) {
            log::info(&format!("{label}: already at goal, skipping move"));
            return Ok(());
        }

        let (mut scene, env) =
            scene_with_assets(self.model, &self.scene_assets, &self.allow_collisions_with);
        scene
            .current_state_mut()
            .set_joint_group_positions(&self.model.group, &start)
            .map_err(|e| SequencerError(format!("{label}: cannot set start state: {e}")))?;
        scene.current_state_mut().update();

        let mut goal_state = self.model.state_with_joints(goal)?;
        let goal_constraints = construct_goal_joint_constraints(
            &self.model.robot,
            &goal_state.update(),
            &self.model.group,
            0.0,
            0.0,
        )
        .map_err(|e| SequencerError(format!("{label}: goal constraints: {e}")))?;

        let path_constraints = self
            .level_tool
            .as_ref()
            .map(|c| c.build(self.model, scene.transforms()))
            .transpose()
            .map_err(|e| SequencerError(format!("{label}: level-tool constraint: {e}")))?;

        // A start that already violates the path constraint has no solution
        // under it, and the planner says so as "start or goal state is
        // itself invalid" — which names neither which of the two nor why,
        // and reads like a collision. Only `satisfied` is quoted here: the
        // result's `distance` sums the deviation about all three world axes
        // including the one this constraint leaves free, so it is not the
        // tilt and must not be printed as one.
        if let Some(set) = &path_constraints {
            let mut start_state = self.model.state_with_joints(&q_to_map(&start))?;
            if !set.decide(&start_state.update()).satisfied {
                return Err(SequencerError(format!(
                    "{label}: the arm's current pose does not satisfy the \
                     level-tool constraint, so no planned move can start from \
                     it. Bring the tool back to level first — hand-eye aiming \
                     leaves it tens of degrees off, and a capture that ended \
                     without returning the arm is the usual way to get here."
                )));
            }
        }

        let request = PlanningRequest {
            group_name: self.model.group.clone(),
            goal_constraints: vec![goal_constraints],
            path_constraints: path_constraints.clone(),
            max_velocity_scaling_factor: velocity_scale,
            max_acceleration_scaling_factor: acceleration_scale,
            ..PlanningRequest::default()
        };
        let planner = resolve_planner("rrt_connect", &PlannerConfigurationMap::new())
            .map_err(|e| SequencerError(format!("rrt_connect not registered: {e}")))?;
        // Timed separately from execution: the step duration in the log
        // is plan + motion, which is not enough to tell a slow planner
        // from a slow move.
        let planning_started = Instant::now();
        let response = generate_plan(&mut scene, &env, &[], &[planner], &[], request)
            .map_err(|e| SequencerError(format!("{label}: planning failed: {e}")))?;
        let planned_in = planning_started.elapsed();
        let mut trajectory = response.trajectory;
        self.shortcut(&mut trajectory, path_constraints.as_ref(), label)?;
        let shortcut_done = planning_started.elapsed();
        apply_totg_time_parameterization(
            &mut trajectory,
            velocity_scale,
            acceleration_scale,
            0.1,
            RESAMPLE_DT,
            0.001,
        )
        .map_err(|e| SequencerError(format!("{label}: TOTG failed: {e}")))?;
        log::info(&format!(
            "  Planned in {:.2} s, shortcut {:.2} s, TOTG {:.2} s",
            planned_in.as_secs_f64(),
            (shortcut_done - planned_in).as_secs_f64(),
            (planning_started.elapsed() - shortcut_done).as_secs_f64(),
        ));

        self.execute(&trajectory, label)
    }

    /// Straight-line Cartesian move of `ik_frame` to the pose FK gives at
    /// `goal`, falling back to [`Motion::move_planned`] when less than
    /// `min_fraction` of the line is reachable — the C++ node's
    /// `execute_cartesian_action`. No collision checking on the line
    /// (parity: the C++ passed avoid_collisions=false's moveit-core path).
    pub fn move_cartesian(
        &mut self,
        goal: &JointMap,
        velocity_scale: f64,
        acceleration_scale: f64,
        label: &str,
    ) -> Result<(), SequencerError> {
        let start = self.fresh_q()?;
        let goal_q = map_to_q(goal)?;
        if already_there(&start, &goal_q) {
            log::info(&format!("{label}: already at goal, skipping move"));
            return Ok(());
        }

        let start_state = self.model.state_with_joints(&q_to_map(&start))?;
        let target_pose = self.model.fk(goal)?;
        let (states, fraction) = self.interpolate(&start_state, &target_pose, label)?;

        log::info(&format!(
            "  Cartesian path: {:.1}% computed ({} waypoints)",
            fraction * 100.0,
            states.len()
        ));
        if fraction < self.min_fraction {
            log::warn(&format!(
                "  Cartesian path incomplete ({:.1}%), falling back to joint space",
                fraction * 100.0
            ));
            return self.move_planned(goal, velocity_scale, acceleration_scale, label);
        }
        if states.len() < 2 {
            log::warn("  Empty trajectory, skipping");
            return Ok(());
        }

        self.timed_execute(
            states,
            velocity_scale,
            acceleration_scale,
            MIN_ANGLE_CHANGE,
            label,
        )
    }

    /// TCP-relative jog for calibration: `d*_mm` in the `ik_frame` frame,
    /// converted to a base-frame translation, executed as a straight line
    /// at `velocity_scale`. Unlike step moves there is no planned
    /// fallback — an unreachable line is an error the operator sees.
    ///
    /// An operator's jog is at least the 0.1 mm the GUI offers, so it
    /// keeps [`MIN_ANGLE_CHANGE`], and the scene is what keeps it off the
    /// stage. [`Motion::probe_step`] is the other caller of the same
    /// mechanism, and it is guarded differently.
    pub fn jog(
        &mut self,
        dx_mm: f64,
        dy_mm: f64,
        dz_mm: f64,
        velocity_scale: f64,
    ) -> Result<(), SequencerError> {
        self.jog_with(
            dx_mm,
            dy_mm,
            dz_mm,
            velocity_scale,
            Guard::Scene,
            MIN_ANGLE_CHANGE,
        )
    }

    /// One step of a force probe: the same straight line as
    /// [`Motion::jog`], guarded by contact rather than by geometry, and
    /// small enough that TOTG's ordinary de-duplication would throw it
    /// away (see [`FINE_MIN_ANGLE_CHANGE`]).
    ///
    /// Not a jog with two flags flipped. A jog and a probe step are
    /// different motions that share a mechanism, and the difference is
    /// [`Guard`] — which is why this is its own name rather than a
    /// parameter an operator-facing call could be handed by mistake.
    /// [`Motion::probe_until_contact`] is the only caller and the only
    /// thing that supplies the guard this depends on.
    pub fn probe_step(
        &mut self,
        dx_mm: f64,
        dy_mm: f64,
        dz_mm: f64,
        velocity_scale: f64,
    ) -> Result<(), SequencerError> {
        self.jog_with(
            dx_mm,
            dy_mm,
            dz_mm,
            velocity_scale,
            Guard::ContactForce,
            FINE_MIN_ANGLE_CHANGE,
        )
    }

    /// Flies `path` as given: no interpolation, no goal, no geometry
    /// question. Used to walk a probe back out along the states it
    /// reached, which is why there is nothing to check — every one of
    /// them is a pose the arm was just standing in.
    ///
    /// This is the return that [`Motion::move_direct`] cannot be for a
    /// probe. `move_direct` re-derives a fresh line and asks the scene
    /// whether it is clear, and inside a bore the scene says no to every
    /// line including the one the arm arrived on.
    pub fn retrace(
        &mut self,
        path: &[JointMap],
        velocity_scale: f64,
        label: &str,
    ) -> Result<(), SequencerError> {
        if path.len() < 2 {
            return Ok(());
        }
        let mut states = Vec::with_capacity(path.len());
        for joints in path {
            states.push(self.model.state_with_joints(joints)?);
        }
        log::info(&format!("  {label}: retracing {} waypoints", states.len()));
        self.timed_execute(
            states,
            velocity_scale,
            velocity_scale,
            FINE_MIN_ANGLE_CHANGE,
            label,
        )
    }

    fn jog_with(
        &mut self,
        dx_mm: f64,
        dy_mm: f64,
        dz_mm: f64,
        velocity_scale: f64,
        guard: Guard,
        min_angle_change: f64,
    ) -> Result<(), SequencerError> {
        if dx_mm == 0.0 && dy_mm == 0.0 && dz_mm == 0.0 {
            return Ok(());
        }
        log::info(&format!(
            "TCP Jog: dx={dx_mm:.3}mm, dy={dy_mm:.3}mm, dz={dz_mm:.3}mm"
        ));

        let start = self.fresh_q()?;
        let mut start_state = self.model.state_with_joints(&q_to_map(&start))?;
        let tcp_tf = start_state
            .update()
            .global_link_transform(&self.model.ik_frame)
            .map_err(|e| SequencerError(format!("jog: FK failed: {e}")))?;

        // Jog axes are expressed in the calibration-offset frame
        // (ik_frame); rotate into base and translate the whole tool
        // rigidly. The C++ translated the flange pose instead — for a pure
        // translation the two are the same rigid motion.
        let offset_tcp = Vector3::new(dx_mm / 1000.0, dy_mm / 1000.0, dz_mm / 1000.0);
        let offset_base = tcp_tf.rotation * offset_tcp;
        let target = Translation3::from(offset_base) * tcp_tf;

        self.fly_to(
            &start_state,
            &target,
            velocity_scale,
            guard,
            min_angle_change,
            "TCP Jog",
        )
    }

    /// One step of a tilt probe: turn the tool `rad` about `axis`, a
    /// direction in the `ik_frame`, without moving the TCP.
    ///
    /// About the tool point and not the flange, because the question a
    /// tilt asks is whether the *sample* is square to its seat and the
    /// sample is held at the tool point. Turning about the flange would
    /// swing it through millimetres of arc and answer a different one.
    ///
    /// Guarded by contact like [`Motion::probe_step`], and for the same
    /// reason: inside a seat the scene cannot say what is clear.
    pub fn probe_twist(
        &mut self,
        axis: Vector3,
        rad: f64,
        velocity_scale: f64,
    ) -> Result<(), SequencerError> {
        if rad == 0.0 {
            return Ok(());
        }
        log::info(&format!(
            "TCP Twist: {:+.3} deg about tool ({:+.2}, {:+.2}, {:+.2})",
            rad.to_degrees(),
            axis.x,
            axis.y,
            axis.z
        ));
        let start = self.fresh_q()?;
        let mut start_state = self.model.state_with_joints(&q_to_map(&start))?;
        let tcp_tf = start_state
            .update()
            .global_link_transform(&self.model.ik_frame)
            .map_err(|e| SequencerError(format!("twist: FK failed: {e}")))?;
        let turn = UnitQuaternion::from_scaled_axis(axis.normalize() * rad);
        // Post-multiplied: the axis is the tool's and the centre is the
        // tool origin, so the TCP stays exactly where it is.
        let target = tcp_tf * Isometry3::from_parts(Translation3::identity().vector.into(), turn);
        self.fly_to(
            &start_state,
            &target,
            velocity_scale,
            Guard::ContactForce,
            FINE_MIN_ANGLE_CHANGE,
            "TCP Twist",
        )
    }

    /// The half of a jog or a twist that is the same for both: check the
    /// line, then fly it.
    fn fly_to(
        &mut self,
        start_state: &RobotState<'m>,
        target: &Isometry3,
        velocity_scale: f64,
        guard: Guard,
        min_angle_change: f64,
        label: &str,
    ) -> Result<(), SequencerError> {
        let (mut states, mut fraction) = self.interpolate(start_state, target, "jog")?;

        // The C++ jog went through move_group's Cartesian-path service,
        // whose avoid_collisions default validity-checks every
        // interpolated state and truncates at the first colliding one —
        // an operator could not jog into the stage. The sequence steps
        // used the core-layer interpolator with no validity callback, so
        // the jog and [`Motion::move_direct`] gate; the steps do not.
        //
        // The gate is relative to its start: contact pairs the starting
        // pose already stands in are exempt along the path
        // ([`first_new_collision_index`]). The ungated steps park the arm
        // "inside" the convex stage parts wherever the decomposition
        // fills a recess, and an absolute check refused every jog out of
        // holder 10's above hold at 0.0% — fingers against `stage1` in a
        // pose the sequence itself taught.
        //
        // Reachability is checked either way below: whether IK can follow
        // the line is a question about the arm, not about the scene, and
        // no guard excuses a caller from it.
        if matches!(guard, Guard::Scene)
            && let Some(i) = first_new_collision_index(
                self.model,
                &self.scene_assets,
                &self.allow_collisions_with,
                &states,
            )?
        {
            let span = (states.len() - 1).max(1) as f64;
            states.truncate(i);
            fraction *= i.saturating_sub(1) as f64 / span;
        }
        if fraction < self.min_fraction {
            return Err(SequencerError(format!(
                "{label}: Cartesian path only {:.1}% achieved",
                fraction * 100.0
            )));
        }
        if states.len() < 2 {
            return Ok(());
        }

        self.timed_execute(
            states,
            velocity_scale,
            velocity_scale,
            min_angle_change,
            label,
        )
    }
}

fn already_there(a: &Vector6D, b: &Vector6D) -> bool {
    a.iter()
        .zip(b.iter())
        .all(|(x, y)| (x - y).abs() < ALREADY_THERE_TOLERANCE)
}
