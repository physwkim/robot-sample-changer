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
mod scene;

pub(crate) use scene::LevelToolConstraint;
pub(crate) use scene::{SceneAsset, first_collision_index, load_scene_assets, scene_with_assets};

use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI32};
use std::time::{Duration, Instant};

use cspace_core::geometry::{Isometry3, Vector3};
use cspace_core::kinematics::{CartesianInterpolator, IkContext, MaxEefStep};
use cspace_core::state::RobotState;
use cspace_core::trajectory::RobotTrajectory;
use cspace_core::trajectory::trajectory_tools::apply_totg_time_parameterization;
use cspace_planning::constraints::utils::construct_goal_joint_constraints;
use cspace_planning::planner_registry::resolve_planner;
use cspace_planning::{PlannerConfigurationMap, PlanningRequest, generate_plan};
// Linked for its side effect: RrtConnectManager registers itself into
// PLANNER_MANAGERS via linkme; without this the linker drops the
// registration and resolve_planner("rrt_connect") fails.
use cspace_planners as _;
use nalgebra::Translation3;

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
    _script_command: Arc<ScriptCommandInterface>,
    // Held for its side effect: the primary stream carries the running
    // program.
    _primary: TcpStream,
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
    fn timed_execute(
        &mut self,
        states: Vec<RobotState<'m>>,
        velocity_scale: f64,
        acceleration_scale: f64,
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
            0.001,
        )
        .map_err(|e| SequencerError(format!("{label}: TOTG failed: {e}")))?;

        self.execute(&trajectory, label)
    }

    /// Whether [`Motion::move_direct`] would carry the arm from `from` to
    /// `to`: the whole straight line reachable, and no state on it
    /// colliding.
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
        let hit = first_collision_index(
            self.model,
            &self.scene_assets,
            &self.allow_collisions_with,
            &states,
        )?;
        Ok(hit.is_none())
    }

    /// Straight-line `ik_frame` move to the pose FK gives at `goal`, with
    /// every interpolated state collision-checked and no planned fallback.
    ///
    /// For motions that are meant to stay small. [`Motion::move_planned`]
    /// answers "some collision-free path exists", not "a short one":
    /// RRT-Connect samples the whole joint space and nothing downstream
    /// shortens what it returns, so two configurations a few degrees apart
    /// can be joined by a path that swings the TCP most of a metre through
    /// the cell — measured at 0.71 m on an 8-degree tool rotation during
    /// hand-eye capture. Interpolating instead makes the executed motion
    /// the one that was asked for.
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
        if let Some(i) = first_collision_index(
            self.model,
            &self.scene_assets,
            &self.allow_collisions_with,
            &states,
        )? {
            return Err(SequencerError(format!(
                "{label}: the straight path collides at waypoint {i} of {}",
                states.len()
            )));
        }
        if states.len() < 2 {
            return Ok(());
        }
        log::info(&format!("  Direct path: {} waypoints", states.len()));

        self.timed_execute(states, velocity_scale, acceleration_scale, label)
    }

    /// Planned (RRT-Connect) joint-space move, the port of the MoveGroup
    /// action fallback. Plans against the stage collision scene.
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
            path_constraints,
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
            "  Planned in {:.2} s, TOTG {:.2} s",
            planned_in.as_secs_f64(),
            (planning_started.elapsed() - planned_in).as_secs_f64(),
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

        self.timed_execute(states, velocity_scale, acceleration_scale, label)
    }

    /// TCP-relative jog for calibration: `d*_mm` in the `ik_frame` frame,
    /// converted to a base-frame translation, executed as a straight line
    /// at `velocity_scale`. Unlike step moves there is no planned
    /// fallback — an unreachable line is an error the operator sees.
    pub fn jog(
        &mut self,
        dx_mm: f64,
        dy_mm: f64,
        dz_mm: f64,
        velocity_scale: f64,
    ) -> Result<(), SequencerError> {
        if dx_mm == 0.0 && dy_mm == 0.0 && dz_mm == 0.0 {
            return Ok(());
        }
        log::info(&format!(
            "TCP Jog: dx={dx_mm:.1}mm, dy={dy_mm:.1}mm, dz={dz_mm:.1}mm"
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

        let (mut states, mut fraction) = self.interpolate(&start_state, &target, "jog")?;

        // The C++ jog went through move_group's Cartesian-path service,
        // whose avoid_collisions default validity-checks every
        // interpolated state and truncates at the first colliding one —
        // an operator could not jog into the stage. The sequence steps
        // used the core-layer interpolator with no validity callback, so
        // the jog and [`Motion::move_direct`] gate; the steps do not.
        if let Some(i) = first_collision_index(
            self.model,
            &self.scene_assets,
            &self.allow_collisions_with,
            &states,
        )? {
            let span = (states.len() - 1).max(1) as f64;
            states.truncate(i);
            fraction *= i.saturating_sub(1) as f64 / span;
        }
        if fraction < self.min_fraction {
            return Err(SequencerError(format!(
                "TCP Jog: Cartesian path only {:.1}% achieved",
                fraction * 100.0
            )));
        }
        if states.len() < 2 {
            return Ok(());
        }

        self.timed_execute(states, velocity_scale, velocity_scale, "TCP Jog")
    }
}

fn already_there(a: &Vector6D, b: &Vector6D) -> bool {
    a.iter()
        .zip(b.iter())
        .all(|(x, y)| (x - y).abs() < ALREADY_THERE_TOLERANCE)
}
