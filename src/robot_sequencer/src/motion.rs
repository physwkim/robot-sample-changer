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

use std::io::{Read as _, Write as _};
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::time::{Duration, Instant};

use cspace_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use cspace_core::geometry::{Isometry3, Shape, Vector3, mesh_from_bytes};
use cspace_core::kinematics::{CartesianInterpolator, IkContext, MaxEefStep};
use cspace_core::state::RobotState;
use cspace_core::trajectory::RobotTrajectory;
use cspace_core::trajectory::trajectory_tools::apply_totg_time_parameterization;
use cspace_planning::constraints::utils::construct_goal_joint_constraints;
use cspace_planning::planner_registry::resolve_planner;
use cspace_planning::scene::PlanningScene;
use cspace_planning::{PlannerConfigurationMap, PlanningRequest, generate_plan};
// Linked for its side effect: RrtConnectManager registers itself into
// PLANNER_MANAGERS via linkme; without this the linker drops the
// registration and resolve_planner("rrt_connect") fails.
use cspace_planners as _;
use nalgebra::{Translation3, UnitQuaternion};

use ur_driver::comm::ControlMode;
use ur_driver::control::reverse_interface::{
    ReverseInterface, ReverseInterfaceConfig, TrajectoryControlMessage,
};
use ur_driver::control::script_command_interface::ScriptCommandInterface;
use ur_driver::control::trajectory_point_interface::{TrajectoryPointInterface, TrajectoryResult};
use ur_driver::rtde::{RtdeClient, RtdeValue};
use ur_driver::types::Vector6D;
use ur_driver::ur::dashboard_client::{DashboardClient, DashboardValue};
use ur_driver::ur::external_control_script::{
    ExternalControlScriptConfig, build_program, wrap_headless,
};
use ur_driver::ur::robot_receive_timeout::RobotReceiveTimeout;

use crate::config::Config;
use crate::error::SequencerError;
use crate::log;
use crate::model::{JointMap, Model};

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
/// Slack added to a trajectory's TOTG duration before the keepalive loop
/// declares the execution hung.
const EXECUTE_TIMEOUT_MARGIN: Duration = Duration::from_secs(10);

struct SceneAsset {
    id: String,
    shape: Arc<Shape>,
    pose: Isometry3,
}

pub struct Motion<'m> {
    model: &'m Model,
    reverse: Arc<ReverseInterface>,
    trajectory: Arc<TrajectoryPointInterface>,
    _script_command: Arc<ScriptCommandInterface>,
    // Held for its side effect: the primary stream carries the running
    // program.
    _primary: TcpStream,
    rtde: RtdeClient,
    streaming: bool,
    program_running: Arc<AtomicBool>,
    trajectory_done: Arc<AtomicBool>,
    last_result: Arc<AtomicI32>,
    scene_assets: Vec<SceneAsset>,
    allow_collisions_with: Vec<String>,
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
    pub fn connect(model: &'m Model, config: &Config) -> Result<Self, SequencerError> {
        let group = model
            .robot
            .joint_model_group(&model.group)
            .map_err(|e| SequencerError(format!("group '{}' not in model: {e}", model.group)))?;
        if group.variable_names() != RTDE_JOINT_ORDER {
            return Err(SequencerError(format!(
                "group '{}' variable order {:?} does not match RTDE actual_q order",
                model.group,
                group.variable_names()
            )));
        }

        let scene_assets = load_scene_assets(config)?;

        let ip = &config.robot.ip;
        let mut dashboard = DashboardClient::new(ip);
        dashboard
            .connect(2, Duration::from_secs(1))
            .map_err(|e| SequencerError(format!("dashboard connect to {ip}: {e}")))?;
        clear_protective_stop(&mut dashboard)?;
        let response = dashboard
            .command_power_on(Duration::from_secs(300))
            .map_err(|e| SequencerError(format!("power on: {e}")))?;
        if !response.ok {
            return Err(SequencerError(format!(
                "power on failed: {}",
                response.message
            )));
        }
        let response = dashboard
            .command_brake_release()
            .map_err(|e| SequencerError(format!("brake release: {e}")))?;
        if !response.ok {
            return Err(SequencerError(format!(
                "brake release failed: {}",
                response.message
            )));
        }

        // Primary interface: source of the local IP the robot connects
        // back to, and the channel the headless program is sent over. A
        // drain thread keeps the robot-state stream from filling the
        // socket buffer.
        let mut primary = TcpStream::connect((ip.as_str(), 30001))
            .map_err(|e| SequencerError(format!("primary connect to {ip}:30001: {e}")))?;
        let local_ip = primary
            .local_addr()
            .map_err(|e| SequencerError(format!("primary local addr: {e}")))?
            .ip()
            .to_string();
        {
            let mut drain = primary
                .try_clone()
                .map_err(|e| SequencerError(format!("clone primary stream: {e}")))?;
            std::thread::spawn(move || {
                let mut sink = [0u8; 4096];
                while drain.read(&mut sink).map(|n| n > 0).unwrap_or(false) {}
            });
        }

        let mut rtde = RtdeClient::connect(
            ip,
            30004,
            read_recipe(&config.robot.output_recipe)?,
            read_recipe(&config.robot.input_recipe)?,
            500.0,
        )
        .map_err(|e| SequencerError(format!("RTDE connect: {e}")))?;
        rtde.init()
            .map_err(|e| SequencerError(format!("RTDE init: {e}")))?;
        let robot_version = rtde.urcontrol_version();
        let step_time = Duration::from_millis((1000.0 / rtde.max_frequency()) as u64);

        let program_running = Arc::new(AtomicBool::new(false));
        let program_running_cb = Arc::clone(&program_running);
        let reverse = Arc::new(
            ReverseInterface::new(ReverseInterfaceConfig {
                port: 50001,
                handle_program_state: Some(Box::new(move |state| {
                    program_running_cb.store(state, Ordering::SeqCst);
                })),
                step_time,
                robot_software_version: robot_version,
            })
            .map_err(|e| SequencerError(format!("reverse interface: {e}")))?,
        );
        let trajectory = Arc::new(
            TrajectoryPointInterface::new(50003)
                .map_err(|e| SequencerError(format!("trajectory interface: {e}")))?,
        );
        let script_command = Arc::new(
            ScriptCommandInterface::new(ReverseInterfaceConfig {
                port: 50004,
                robot_software_version: robot_version,
                ..Default::default()
            })
            .map_err(|e| SequencerError(format!("script command interface: {e}")))?,
        );

        let script_file = config.robot.script_file.to_string_lossy().into_owned();
        let program = build_program(&ExternalControlScriptConfig {
            script_file,
            local_ip,
            robot_software_version: robot_version,
            ..Default::default()
        })
        .map_err(|e| SequencerError(format!("assemble external-control program: {e}")))?;
        let full_program = format!("{}\n", wrap_headless(&program));
        primary
            .write_all(full_program.as_bytes())
            .map_err(|e| SequencerError(format!("send program: {e}")))?;

        let deadline = Instant::now() + Duration::from_secs(10);
        while !program_running.load(Ordering::SeqCst)
            || !trajectory.is_connected()
            || !script_command.client_connected()
        {
            if Instant::now() > deadline {
                return Err(SequencerError(
                    "robot did not connect to the reverse/trajectory/script-command \
                     interfaces within 10 s"
                        .into(),
                ));
            }
            std::thread::sleep(Duration::from_millis(10));
        }

        let trajectory_done = Arc::new(AtomicBool::new(false));
        let last_result = Arc::new(AtomicI32::new(TrajectoryResult::Unknown as i32));
        {
            let trajectory_done = Arc::clone(&trajectory_done);
            let last_result = Arc::clone(&last_result);
            trajectory.add_trajectory_end_callback(move |result| {
                last_result.store(result as i32, Ordering::SeqCst);
                trajectory_done.store(true, Ordering::SeqCst);
            });
        }

        let mut motion = Self {
            model,
            reverse,
            trajectory,
            _script_command: script_command,
            _primary: primary,
            rtde,
            streaming: false,
            program_running,
            trajectory_done,
            last_result,
            scene_assets,
            allow_collisions_with: config.scene.allow_collisions_with.clone(),
            translation_step: config.sequence.cartesian_translation_step,
            rotation_step: config.sequence.cartesian_rotation_step,
            min_fraction: config.sequence.cartesian_min_fraction,
        };

        // Park the program so it idles between motions, then normalize the
        // speed slider: URControl retains whatever fraction it last
        // received (a pendant touch survives program restarts), and TOTG
        // timing assumes full speed.
        motion.park()?;
        motion.ensure_streaming()?;
        let sent = motion
            .rtde
            .writer()
            .ok_or_else(|| SequencerError("RTDE writer unavailable".into()))?
            .send_speed_slider(1.0);
        if !sent {
            return Err(SequencerError("cannot send speed slider".into()));
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if let Some(RtdeValue::F64(fraction)) =
                motion.read_package()?.get("target_speed_fraction")
                && (fraction - 1.0).abs() < 1e-6
            {
                break;
            }
            if Instant::now() > deadline {
                return Err(SequencerError(
                    "speed slider did not reach 1.0 within 5 s".into(),
                ));
            }
        }

        log::info("Robot connected (program running, speed slider at 1.0)");
        Ok(motion)
    }

    /// Starts RTDE output streaming if paused. Every motion entry point
    /// calls this, so pausing at idle is always safe.
    fn ensure_streaming(&mut self) -> Result<(), SequencerError> {
        if !self.streaming {
            self.rtde
                .start()
                .map_err(|e| SequencerError(format!("RTDE start: {e}")))?;
            self.streaming = true;
        }
        Ok(())
    }

    /// Pauses RTDE output streaming. Called when the sequence enters a
    /// long PV-polling wait (idle trigger loop, measurement wait) so the
    /// unread 500 Hz stream does not accumulate in socket buffers.
    pub fn pause_streaming(&mut self) {
        if self.streaming {
            match self.rtde.pause() {
                Ok(()) => self.streaming = false,
                Err(e) => log::warn(&format!("RTDE pause failed: {e}")),
            }
        }
    }

    fn read_package(&mut self) -> Result<ur_driver::rtde::DataPackage, SequencerError> {
        self.rtde
            .get_data_package()
            .map_err(|e| SequencerError(format!("RTDE read: {e}")))
    }

    /// Current joint positions, drained so the sample reflects the present
    /// rather than the backlog accumulated while this daemon was planning
    /// or idling.
    fn fresh_q(&mut self) -> Result<Vector6D, SequencerError> {
        self.ensure_streaming()?;
        for _ in 0..100 {
            self.read_package()?;
        }
        let pkg = self.read_package()?;
        match pkg.get("actual_q") {
            Some(RtdeValue::V6D(q)) => Ok(*q),
            other => Err(SequencerError(format!(
                "actual_q missing from RTDE package: {other:?}"
            ))),
        }
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

        let request = PlanningRequest {
            group_name: self.model.group.clone(),
            goal_constraints: vec![goal_constraints],
            max_velocity_scaling_factor: velocity_scale,
            max_acceleration_scaling_factor: acceleration_scale,
            ..PlanningRequest::default()
        };
        let planner = resolve_planner("rrt_connect", &PlannerConfigurationMap::new())
            .map_err(|e| SequencerError(format!("rrt_connect not registered: {e}")))?;
        let response = generate_plan(&mut scene, &env, &[], &[planner], &[], request)
            .map_err(|e| SequencerError(format!("{label}: planning failed: {e}")))?;
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

        let interpolator = CartesianInterpolator::new(
            &self.model.group,
            &self.model.ik_frame,
            MaxEefStep::new(self.translation_step, self.rotation_step),
        );
        let mut solver = self.model.solver()?;
        let (states, fraction) = interpolator
            .to_pose(
                &start_state,
                &mut solver,
                &target_pose,
                &mut IkContext::default(),
            )
            .map_err(|e| SequencerError(format!("{label}: Cartesian interpolation: {e}")))?;

        log::info(&format!(
            "  Cartesian path: {:.1}% computed ({} waypoints)",
            fraction.value() * 100.0,
            states.len()
        ));
        if fraction.value() < self.min_fraction {
            log::warn(&format!(
                "  Cartesian path incomplete ({:.1}%), falling back to joint space",
                fraction.value() * 100.0
            ));
            return self.move_planned(goal, velocity_scale, acceleration_scale, label);
        }
        if states.len() < 2 {
            log::warn("  Empty trajectory, skipping");
            return Ok(());
        }

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

        let interpolator = CartesianInterpolator::new(
            &self.model.group,
            &self.model.ik_frame,
            MaxEefStep::new(self.translation_step, self.rotation_step),
        );
        let mut solver = self.model.solver()?;
        let (mut states, fraction) = interpolator
            .to_pose(
                &start_state,
                &mut solver,
                &target,
                &mut IkContext::default(),
            )
            .map_err(|e| SequencerError(format!("jog: Cartesian interpolation: {e}")))?;

        // The C++ jog went through move_group's Cartesian-path service,
        // whose avoid_collisions default validity-checks every
        // interpolated state and truncates at the first colliding one —
        // an operator could not jog into the stage. The sequence steps
        // used the core-layer interpolator with no validity callback, so
        // only the jog gets this gate.
        let mut fraction = fraction.value();
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

        let mut trajectory = RobotTrajectory::for_group_name(&self.model.robot, &self.model.group)
            .map_err(|e| SequencerError(format!("jog: trajectory: {e}")))?;
        for state in states {
            trajectory
                .add_suffix_way_point(state, 0.0)
                .map_err(|e| SequencerError(format!("jog: trajectory waypoint: {e}")))?;
        }
        apply_totg_time_parameterization(
            &mut trajectory,
            velocity_scale,
            velocity_scale,
            0.1,
            RESAMPLE_DT,
            0.001,
        )
        .map_err(|e| SequencerError(format!("jog: TOTG failed: {e}")))?;

        self.execute(&trajectory, "TCP Jog")
    }

    /// Streams the trajectory to the robot and waits for the end
    /// callback. The program is re-parked afterwards on every path,
    /// success or not — without the park, the next read timeout kills the
    /// external-control program.
    fn execute(
        &mut self,
        trajectory: &RobotTrajectory<'_>,
        label: &str,
    ) -> Result<(), SequencerError> {
        let outcome = self.run_trajectory(trajectory, label);
        let park = self.park();
        outcome.and(park)
    }

    fn run_trajectory(
        &mut self,
        trajectory: &RobotTrajectory<'_>,
        label: &str,
    ) -> Result<(), SequencerError> {
        self.ensure_streaming()?;
        let n = trajectory.way_point_count();
        if n < 2 {
            return Ok(());
        }
        self.trajectory_done.store(false, Ordering::SeqCst);
        self.last_result
            .store(TrajectoryResult::Unknown as i32, Ordering::SeqCst);
        self.reverse
            .write_trajectory_control_message(
                TrajectoryControlMessage::TrajectoryStart,
                (n - 1) as i32,
                RobotReceiveTimeout::millisec(200),
            )
            .map_err(|e| SequencerError(format!("{label}: trajectory start: {e}")))?;

        let group = &self.model.group;
        for i in 1..n {
            let waypoint = trajectory
                .way_point(i)
                .map_err(|e| SequencerError(format!("{label}: waypoint {i}: {e}")))?;
            let p: Vector6D = waypoint
                .joint_group_positions(group)
                .map_err(|e| SequencerError(format!("{label}: positions: {e}")))?
                .try_into()
                .map_err(|_| SequencerError(format!("{label}: waypoint {i} is not 6 joints")))?;
            let v: Vector6D = waypoint
                .joint_group_velocities(group)
                .map_err(|e| SequencerError(format!("{label}: velocities: {e}")))?
                .try_into()
                .map_err(|_| SequencerError(format!("{label}: waypoint {i} is not 6 joints")))?;
            let a: Vector6D = waypoint
                .joint_group_accelerations(group)
                .map_err(|e| SequencerError(format!("{label}: accelerations: {e}")))?
                .try_into()
                .map_err(|_| SequencerError(format!("{label}: waypoint {i} is not 6 joints")))?;
            let dt = trajectory.way_point_duration_from_previous(i) as f32;
            let accepted = self
                .trajectory
                .write_trajectory_spline_point(Some(&p), Some(&v), Some(&a), dt)
                .map_err(|e| SequencerError(format!("{label}: spline write: {e}")))?;
            if !accepted {
                return Err(SequencerError(format!(
                    "{label}: robot rejected spline point {i}"
                )));
            }
        }

        let deadline = Instant::now()
            + Duration::from_secs_f64(trajectory.duration())
            + EXECUTE_TIMEOUT_MARGIN;
        while !self.trajectory_done.load(Ordering::SeqCst) {
            if !self.program_running.load(Ordering::SeqCst) {
                return Err(SequencerError(format!(
                    "{label}: external-control program stopped during execution"
                )));
            }
            if Instant::now() > deadline {
                let _ = self.reverse.write_trajectory_control_message(
                    TrajectoryControlMessage::TrajectoryCancel,
                    0,
                    RobotReceiveTimeout::millisec(200),
                );
                return Err(SequencerError(format!(
                    "{label}: trajectory did not finish within TOTG duration + {}s",
                    EXECUTE_TIMEOUT_MARGIN.as_secs()
                )));
            }
            self.read_package()?;
            self.reverse
                .write_trajectory_control_message(
                    TrajectoryControlMessage::TrajectoryNoop,
                    0,
                    RobotReceiveTimeout::millisec(200),
                )
                .map_err(|e| SequencerError(format!("{label}: keepalive: {e}")))?;
        }

        let result = self.last_result.load(Ordering::SeqCst);
        if result == TrajectoryResult::Success as i32 {
            Ok(())
        } else {
            Err(SequencerError(format!(
                "{label}: trajectory ended with result {result} (0=success, 1=canceled, 2=failure)"
            )))
        }
    }

    /// Parks the external-control program in idle keepalive mode
    /// (InstructionExecutor's end-of-motion idiom) so it survives
    /// arbitrarily long pauses between motions.
    fn park(&mut self) -> Result<(), SequencerError> {
        self.reverse
            .write(
                Some(&[0.0; 6]),
                ControlMode::ModeIdle,
                RobotReceiveTimeout::millisec(0),
            )
            .map_err(|e| SequencerError(format!("park program: {e}")))?;
        Ok(())
    }
}

impl Drop for Motion<'_> {
    fn drop(&mut self) {
        // UrDriver::stopControl() equivalent; best effort on teardown.
        let _ = self.reverse.write(
            None,
            ControlMode::ModeStopped,
            RobotReceiveTimeout::millisec(20),
        );
    }
}

fn already_there(a: &Vector6D, b: &Vector6D) -> bool {
    a.iter()
        .zip(b.iter())
        .all(|(x, y)| (x - y).abs() < ALREADY_THERE_TOLERANCE)
}

fn read_recipe(path: &std::path::Path) -> Result<Vec<String>, SequencerError> {
    Ok(std::fs::read_to_string(path)
        .map_err(|e| SequencerError(format!("cannot read recipe {}: {e}", path.display())))?
        .lines()
        .filter(|l| !l.is_empty())
        .map(str::to_string)
        .collect())
}

/// Releases a leftover protective stop (e.g. from a crash mid-motion) the
/// way ur-rs's live-robot helper does: the controller refuses the unlock
/// within 5 s of the stop event, hence the retry loop; the stop also
/// leaves the previous program paused, so drop it.
fn clear_protective_stop(dashboard: &mut DashboardClient) -> Result<(), SequencerError> {
    let protective_stopped = |dashboard: &mut DashboardClient| -> Result<bool, SequencerError> {
        let status = dashboard
            .command_safety_status()
            .map_err(|e| SequencerError(format!("safety status: {e}")))?;
        Ok(matches!(
            status.data.get("safety_status"),
            Some(DashboardValue::Str(s)) if s == "PROTECTIVE_STOP"
        ))
    };
    if !protective_stopped(dashboard)? {
        return Ok(());
    }
    log::warn("Robot is in protective stop, trying to release it");
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        let _ = dashboard.command_close_safety_popup();
        let _ = dashboard.command_unlock_protective_stop();
        std::thread::sleep(Duration::from_secs(1));
        if !protective_stopped(dashboard)? {
            break;
        }
        if Instant::now() > deadline {
            return Err(SequencerError(
                "robot did not leave PROTECTIVE_STOP within 30 s".into(),
            ));
        }
    }
    let response = dashboard
        .command_stop()
        .map_err(|e| SequencerError(format!("stop paused program: {e}")))?;
    if !response.ok {
        return Err(SequencerError(format!(
            "stop paused program failed: {}",
            response.message
        )));
    }
    Ok(())
}

fn load_scene_assets(config: &Config) -> Result<Vec<SceneAsset>, SequencerError> {
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
fn scene_with_assets<'m>(
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
fn first_collision_index(
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
