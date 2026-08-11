//! Gripper control: the real Hand-E over the UR tool-communication
//! Modbus bridge (robotiq-hande crate), or a simulated gripper for URSim
//! (which has no tool communication). Carries the C++ node's settle-wait
//! and threshold-based `Gripper_RBV` bookkeeping, fed by the driver's
//! position readback instead of `/joint_states`.

use std::time::{Duration, Instant};

use robotiq_hande::driver::{HandeDriver, HandeDriverConfig};

use crate::config::{Config, GripperMode};
use crate::epics::Epics;
use crate::error::SequencerError;
use crate::log;

const SETTLE_POLL: Duration = Duration::from_millis(30);
/// Position change below this between polls counts as "not moving".
const STALL_EPSILON: f64 = 0.0003;
/// No stall verdict before this much total wait...
const STALL_DWELL: Duration = Duration::from_millis(250);
/// ...and this much quiet since the last observed movement.
const STALL_QUIET: Duration = Duration::from_millis(300);
const ACTIVATION_TIMEOUT: Duration = Duration::from_secs(10);

enum Backend {
    Hande(Box<HandeDriver>),
    /// Position snaps to the command; the settle wait then behaves like a
    /// real unobstructed gripper (open reaches, close settles by stall).
    Simulated {
        position: f64,
    },
}

pub struct Gripper {
    backend: Backend,
    open_position: f64,
    close_position: f64,
    close_settle_target: f64,
    reach_tolerance: f64,
    settle_timeout: Duration,
    open_threshold: f64,
    last_rbv: Option<i32>,
}

impl Gripper {
    pub fn connect(config: &Config) -> Result<Self, SequencerError> {
        let g = &config.gripper;
        let backend = match g.mode {
            GripperMode::Hande => {
                let mut driver_config = HandeDriverConfig::new(config.robot.ip.clone());
                driver_config.position_min = 0.0;
                driver_config.position_max = g.open_position;
                let mut driver = HandeDriver::connect(driver_config)
                    .map_err(|e| SequencerError(format!("Hand-E connect: {e}")))?;
                driver
                    .start(ACTIVATION_TIMEOUT)
                    .map_err(|e| SequencerError(format!("Hand-E activation: {e}")))?;
                Backend::Hande(Box::new(driver))
            }
            GripperMode::Simulated => {
                log::info("Gripper: simulated (no tool communication)");
                // The real driver's initial command opens the gripper on
                // activation; start the simulation in the same state.
                Backend::Simulated {
                    position: g.open_position,
                }
            }
        };
        Ok(Self {
            backend,
            open_position: g.open_position,
            close_position: g.close_position,
            close_settle_target: g.close_settle_target,
            reach_tolerance: g.reach_tolerance,
            settle_timeout: Duration::from_secs_f64(g.settle_timeout),
            open_threshold: g.open_threshold,
            last_rbv: None,
        })
    }

    pub fn position(&self) -> f64 {
        match &self.backend {
            Backend::Hande(driver) => driver.position(),
            Backend::Simulated { position } => *position,
        }
    }

    /// Sends the open/close position command (asynchronous; pair with
    /// [`Gripper::wait_reached`]).
    pub fn command(&mut self, open: bool) {
        let target = if open {
            self.open_position
        } else {
            self.close_position
        };
        log::info(&format!("Sending gripper command: position={target:.3}"));
        match &mut self.backend {
            Backend::Hande(driver) => driver.set_position(target, 1.0),
            Backend::Simulated { position } => *position = target,
        }
    }

    /// Port of the C++ `wait_gripper_reached`: block until the gripper
    /// reaches the wait target or stalls (reached its limit, or grabbed an
    /// object — which is why the close wait target is `close_settle_target`
    /// rather than the commanded position), up to `settle_timeout`, which
    /// only warns. Keeps `Gripper_RBV` fresh while the fingers move.
    pub fn wait_reached(&mut self, open: bool, epics: &Epics) {
        let target = if open {
            self.open_position
        } else {
            self.close_settle_target
        };
        let t0 = Instant::now();
        let mut last_move = t0;
        let mut prev: Option<f64> = None;
        loop {
            let pos = self.position();
            let now = Instant::now();
            if (pos - target).abs() < self.reach_tolerance {
                log::info(&format!("  Gripper reached target (pos={pos:.4})"));
                break;
            }
            if prev.is_none_or(|p| (pos - p).abs() >= STALL_EPSILON) {
                last_move = now;
            }
            prev = Some(pos);
            if now - t0 > STALL_DWELL && now - last_move > STALL_QUIET {
                log::info(&format!(
                    "  Gripper settled (pos={pos:.4}, target={target:.4})"
                ));
                break;
            }
            if now - t0 > self.settle_timeout {
                log::warn(&format!(
                    "  Gripper settle timeout (pos={pos:.4}, target={target:.4})"
                ));
                break;
            }
            self.update_rbv(epics);
            std::thread::sleep(SETTLE_POLL);
        }
        self.update_rbv(epics);
    }

    /// Writes `Gripper_RBV` (1 = at/above `open_threshold`) on state
    /// change, the C++ `/joint_states`-callback behavior.
    pub fn update_rbv(&mut self, epics: &Epics) {
        let state = i32::from(self.position() >= self.open_threshold);
        if self.last_rbv != Some(state) {
            self.last_rbv = Some(state);
            epics.write_gripper_rbv(state);
        }
    }
}
