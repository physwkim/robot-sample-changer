//! Trajectory execution: the TOTG samples go out as quintic spline
//! segments and the program is kept alive with NOOPs until the end
//! callback, then re-parked so it survives an arbitrarily long wait.

use super::{EXECUTE_TIMEOUT_MARGIN, Motion};
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use cspace_core::trajectory::RobotTrajectory;
// Linked for its side effect: RrtConnectManager registers itself into
// PLANNER_MANAGERS via linkme; without this the linker drops the
// registration and resolve_planner("rrt_connect") fails.
use cspace_planners as _;

use ur_driver::comm::ControlMode;
use ur_driver::control::reverse_interface::TrajectoryControlMessage;
use ur_driver::control::trajectory_point_interface::TrajectoryResult;
use ur_driver::types::Vector6D;
use ur_driver::ur::robot_receive_timeout::RobotReceiveTimeout;

use crate::error::SequencerError;

impl Motion<'_> {
    /// Streams the trajectory to the robot and waits for the end
    /// callback. The program is re-parked afterwards on every path,
    /// success or not — without the park, the next read timeout kills the
    /// external-control program.
    pub(super) fn execute(
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
        // Held across the whole execution: the keepalive loop below is
        // the only thing reading RTDE while the robot moves.
        let mut session = self.rtde.session()?;
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
            session.read()?;
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
    pub(super) fn park(&mut self) -> Result<(), SequencerError> {
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
