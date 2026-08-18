//! Bring-up: what happens between "the robot is powered" and "the
//! daemon can send it a trajectory" — dashboard normalize, the headless
//! external-control program, and the four sockets the robot connects
//! back on.

use super::{LevelToolConstraint, Motion, RTDE_JOINT_ORDER, load_scene_assets};
use std::io::{Read as _, Write as _};
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::time::{Duration, Instant};

// Linked for its side effect: RrtConnectManager registers itself into
// PLANNER_MANAGERS via linkme; without this the linker drops the
// registration and resolve_planner("rrt_connect") fails.
use cspace_planners as _;

use ur_driver::control::reverse_interface::{ReverseInterface, ReverseInterfaceConfig};
use ur_driver::control::script_command_interface::ScriptCommandInterface;
use ur_driver::control::trajectory_point_interface::{TrajectoryPointInterface, TrajectoryResult};
use ur_driver::ur::dashboard_client::{DashboardClient, DashboardValue};
use ur_driver::ur::external_control_script::{
    ExternalControlScriptConfig, build_program, wrap_headless,
};

use crate::config::Config;
use crate::error::SequencerError;
use crate::log;
use crate::model::Model;
use crate::stream::RtdeStream;

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

        let rtde = RtdeStream::connect(
            ip,
            30004,
            read_recipe(&config.robot.output_recipe)?,
            read_recipe(&config.robot.input_recipe)?,
            config.robot.rtde_frequency_hz,
        )?;
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
        let robot_ip = ip.clone();

        wait_for_program(
            &program_running,
            &trajectory,
            &script_command,
            Duration::from_secs(10),
        )?;

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
            script_command,
            _primary: primary,
            robot_ip,
            full_program,
            rtde,
            program_running,
            trajectory_done,
            last_result,
            scene_assets,
            allow_collisions_with: config.scene.allow_collisions_with.clone(),
            level_tool: config.sequence.level_tool.enabled.then(|| {
                LevelToolConstraint::new(
                    &config.robot.ik_frame,
                    config.sequence.level_tool.tolerance_deg,
                )
            }),
            translation_step: config.sequence.cartesian_translation_step,
            rotation_step: config.sequence.cartesian_rotation_step,
            min_fraction: config.sequence.cartesian_min_fraction,
        };

        // Park the program so it idles between motions, then normalize the
        // speed slider: URControl retains whatever fraction it last
        // received (a pendant touch survives program restarts), and TOTG
        // timing assumes full speed.
        motion.park()?;
        motion.rtde.send_speed_slider(1.0)?;
        motion.rtde.session()?.wait_for_f64(
            "target_speed_fraction",
            1.0,
            Duration::from_secs(5),
        )?;

        log::info("Robot connected (program running, speed slider at 1.0)");
        Ok(motion)
    }
}

fn read_recipe(path: &std::path::Path) -> Result<Vec<String>, SequencerError> {
    Ok(std::fs::read_to_string(path)
        .map_err(|e| SequencerError(format!("cannot read recipe {}: {e}", path.display())))?
        .lines()
        .filter(|l| !l.is_empty())
        .map(str::to_string)
        .collect())
}

/// Asks the dashboard whether the robot is in PROTECTIVE_STOP.
fn protective_stopped(dashboard: &mut DashboardClient) -> Result<bool, SequencerError> {
    let status = dashboard
        .command_safety_status()
        .map_err(|e| SequencerError(format!("safety status: {e}")))?;
    Ok(matches!(
        status.data.get("safety_status"),
        Some(DashboardValue::Str(s)) if s == "PROTECTIVE_STOP"
    ))
}

/// Releases a leftover protective stop (e.g. from a crash mid-motion) the
/// way ur-rs's live-robot helper does: the controller refuses the unlock
/// within 5 s of the stop event, hence the retry loop; the stop also
/// leaves the previous program paused, so drop it.
fn clear_protective_stop(dashboard: &mut DashboardClient) -> Result<(), SequencerError> {
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

/// Waits for a freshly (re)sent program to call back on the reverse,
/// trajectory, and script-command interfaces.
fn wait_for_program(
    program_running: &AtomicBool,
    trajectory: &TrajectoryPointInterface,
    script_command: &ScriptCommandInterface,
    timeout: Duration,
) -> Result<(), SequencerError> {
    let deadline = Instant::now() + timeout;
    while !program_running.load(Ordering::SeqCst)
        || !trajectory.is_connected()
        || !script_command.client_connected()
    {
        if Instant::now() > deadline {
            return Err(SequencerError(format!(
                "robot did not connect to the reverse/trajectory/script-command \
                 interfaces within {} s",
                timeout.as_secs()
            )));
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    Ok(())
}

impl Motion<'_> {
    /// True while the external-control program holds all three of its
    /// callback connections. A paused program keeps them, so this alone
    /// does not mean the program can execute anything — pair it with the
    /// dashboard's `running` answer for the robot-side truth.
    fn program_alive(&self) -> bool {
        self.program_running.load(Ordering::SeqCst)
            && self.trajectory.is_connected()
            && self.script_command.client_connected()
    }

    /// Heals a dead external-control program before a sequence runs.
    ///
    /// A protective stop, a pendant stop, and freedrive all end the
    /// program, and until it is sent again every trajectory is refused.
    /// The old answer was "restart the daemon" — the restart the gripper
    /// forbids while it holds a sample. Instead this resends the same
    /// program bring-up sent and waits for the robot to call back.
    ///
    /// The unlock is gated: with `allow_unlock` false a protective stop
    /// is an error, because it means the arm hit something and releasing
    /// it is the operator's decision. The Recover trigger is how the
    /// operator says so.
    pub fn ensure_program(&mut self, allow_unlock: bool) -> Result<(), SequencerError> {
        let mut dashboard = DashboardClient::new(&self.robot_ip);
        if let Err(e) = dashboard.connect(2, Duration::from_secs(1)) {
            if self.program_alive() {
                log::warn(&format!(
                    "dashboard unreachable ({e}); the program looks alive, continuing"
                ));
                return Ok(());
            }
            return Err(SequencerError(format!(
                "dashboard connect to {}: {e}",
                self.robot_ip
            )));
        }
        if protective_stopped(&mut dashboard)? {
            if !allow_unlock {
                return Err(SequencerError(
                    "robot is in PROTECTIVE_STOP — check what the arm hit, then \
                     trigger CalibMode=4 (Recover) to unlock it and resend the \
                     program"
                        .into(),
                ));
            }
            clear_protective_stop(&mut dashboard)?;
        }
        // The robot-side answer, not the daemon-side flags: a paused
        // program keeps its sockets (and `program_running`) while it can
        // no longer execute anything.
        let running = dashboard
            .command_running()
            .map_err(|e| SequencerError(format!("dashboard running query: {e}")))?;
        if matches!(
            running.data.get("running"),
            Some(DashboardValue::Bool(true))
        ) && self.program_alive()
        {
            return Ok(());
        }
        log::warn("External-control program is not running — resending it");
        // Drop whatever half-dead program remains so the resend does not
        // race it for the callback ports.
        let _ = dashboard.command_stop();
        let deadline = Instant::now() + Duration::from_secs(5);
        while self.program_alive() {
            if Instant::now() > deadline {
                log::warn("old program connections did not drop within 5 s; resending anyway");
                break;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        self.resend_program()
    }

    /// Sends the stored program over a fresh primary connection and
    /// restores the bring-up invariants (parked program, speed slider at
    /// 1.0). The connection is dropped right after the write, like the
    /// hand-run `nc` procedure this replaces.
    fn resend_program(&mut self) -> Result<(), SequencerError> {
        {
            let mut primary = TcpStream::connect((self.robot_ip.as_str(), 30001)).map_err(|e| {
                SequencerError(format!("primary connect to {}:30001: {e}", self.robot_ip))
            })?;
            primary
                .write_all(self.full_program.as_bytes())
                .map_err(|e| SequencerError(format!("resend program: {e}")))?;
        }
        wait_for_program(
            &self.program_running,
            &self.trajectory,
            &self.script_command,
            Duration::from_secs(10),
        )?;
        self.park()?;
        self.rtde.send_speed_slider(1.0)?;
        self.rtde
            .session()?
            .wait_for_f64("target_speed_fraction", 1.0, Duration::from_secs(5))?;
        log::info("External-control program resent (running, speed slider at 1.0)");
        Ok(())
    }
}
