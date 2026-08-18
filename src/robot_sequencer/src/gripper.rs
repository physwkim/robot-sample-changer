//! Gripper control: the real Hand-E over the UR tool-communication
//! Modbus bridge (robotiq-hande crate), or a simulated gripper for URSim
//! (which has no tool communication). Carries the C++ node's settle-wait
//! and threshold-based `Gripper_RBV` bookkeeping, fed by the driver's
//! position readback instead of `/joint_states`.

use std::time::{Duration, Instant};

use robotiq_hande::driver::{HandeDriver, HandeDriverConfig};
use robotiq_hande::gripper::Grip;

use crate::config::{Config, GripperMode};
use crate::epics::Epics;
use crate::error::SequencerError;
use crate::log;

/// Where the gripper posts `Gripper_RBV` transitions while the fingers
/// move. Production passes [`Epics`]; tests pass a no-op sink, which is
/// the seam that lets the loosen/regrip contract run under `cargo test`
/// (an [`Epics`] cannot exist without a CA server).
pub trait RbvSink {
    fn write_gripper_rbv(&self, value: i32);
}

impl RbvSink for Epics {
    fn write_gripper_rbv(&self, value: i32) {
        Epics::write_gripper_rbv(self, value);
    }
}

const SETTLE_POLL: Duration = Duration::from_millis(30);
/// Position change below this between polls counts as "not moving".
const STALL_EPSILON: f64 = 0.0003;
/// No stall verdict before this much total wait...
const STALL_DWELL: Duration = Duration::from_millis(250);
/// ...and this much quiet since the last observed movement.
const STALL_QUIET: Duration = Duration::from_millis(300);
const ACTIVATION_TIMEOUT: Duration = Duration::from_secs(10);
/// Force scale for the probe's release. A move that only has to let go of
/// something needs no force, and the fingers open into a rack: at the full
/// scale a release that met the holder would push on it with everything the
/// Hand-E has. The floor is the gripper's own minimum, not zero.
const RELEASE_FORCE: f64 = 0.0;

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
    /// How hard and how fast the fingers move on a sample. One value for
    /// every command that closes or opens on one, so that "how the
    /// sequence grips" is a single configured thing rather than a
    /// constant per call site.
    grip: Grip,
    min_grip_position: f64,
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
                driver_config.frequency_hz = g.poll_hz;
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
            grip: Grip {
                force: g.grip_force,
                speed: g.grip_speed,
            },
            min_grip_position: g.min_grip_position,
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
            Backend::Hande(driver) => driver.set_position(target, self.grip),
            Backend::Simulated { position } => *position = target,
        }
    }

    /// Opens the fingers by `by_m` from wherever they are now, waits for
    /// them to stop moving, and reports the play that opened up.
    ///
    /// The seat probe is the only caller. It needs the puck loose in the
    /// bore rather than clamped — a gripped puck closes the
    /// gripper-puck-bore loop, and a 0.05 mm step then reads 1.2 to 6.3 N
    /// with no free travel to bracket (measured 2026-08-15) — but not
    /// released, because the fingers still have to be the thing that holds
    /// it.
    ///
    /// Relative, not absolute: where the fingers stop on a puck is not
    /// known here, and the close wait does not find out (see
    /// [`Gripper::settle_at`]). Clamped to `open_position` so this can
    /// never open wider than a plain open.
    ///
    /// Asked for no play, this touches the gripper at all — not even to
    /// command where it already is. A Hand-E told to hold its current
    /// position at [`RELEASE_FORCE`] stops squeezing, so the one command
    /// that looks like a no-op is the one that would quietly give up the
    /// grip this is supposed to leave alone.
    pub fn loosen_by(&mut self, by_m: f64, epics: &dyn RbvSink) -> f64 {
        if by_m <= 0.0 {
            log::info("Keeping the grip as it is (no play asked for)");
            return 0.0;
        }
        let from = self.position();
        let target = (from + by_m).min(self.open_position);
        log::info(&format!("Loosening the grip: {from:.4} -> {target:.4} m"));
        self.settle_to(
            target,
            Grip {
                force: RELEASE_FORCE,
                speed: self.grip.speed,
            },
            epics,
        );
        let play = self.position() - from;
        log::info(&format!("  Grip loosened, {play:.4} m of play"));
        play
    }

    /// Closes onto the sample again and waits for the fingers to stop.
    ///
    /// Commands `close_position`, the same register value the sequence's
    /// close writes, rather than the position the fingers were measured at
    /// before [`Gripper::loosen_by`]: a Hand-E commanded to where the puck
    /// already is holds it with whatever force that costs, which is not the
    /// grip the arm then lifts the puck out with.
    /// Returns where the fingers settled, so the caller holding the
    /// pre-loosen width can tell whether the same object is back between
    /// the pads.
    pub fn regrip(&mut self, epics: &dyn RbvSink) -> f64 {
        log::info("Restoring the grip");
        self.settle_to(self.close_position, self.grip, epics);
        let settled = self.position();
        log::info(&format!("  Grip restored at {settled:.4} m"));
        settled
    }

    /// The band [`Gripper::settle_at`] treats as "the same place", for
    /// callers comparing two settle positions.
    pub fn reach_tolerance(&self) -> f64 {
        self.reach_tolerance
    }

    /// After a close: `Some(settled_m)` when the fingers came to rest
    /// narrower than `min_grip_position`, i.e. on each other rather than
    /// on a puck. Only meaningful on the real gripper — the simulated
    /// backend reaches the commanded position exactly, so it always
    /// answers `None` — and only when the threshold is configured.
    pub fn empty_close(&self) -> Option<f64> {
        if matches!(self.backend, Backend::Simulated { .. }) || self.min_grip_position <= 0.0 {
            return None;
        }
        let settled = self.position();
        (settled < self.min_grip_position).then_some(settled)
    }

    /// Port of the C++ `wait_gripper_reached`: block until the gripper
    /// reaches the wait target or stalls (reached its limit, or grabbed an
    /// object — which is why the close wait target is `close_settle_target`
    /// rather than the commanded position), up to `settle_timeout`, which
    /// only warns. Keeps `Gripper_RBV` fresh while the fingers move.
    pub fn wait_reached(&mut self, open: bool, epics: &dyn RbvSink) {
        let target = if open {
            self.open_position
        } else {
            self.close_settle_target
        };
        self.settle_at(target, self.reach_tolerance, epics);
    }

    /// Commands `target` and waits for the fingers to come to rest there or
    /// against something, with no tolerance band.
    ///
    /// The band is what makes [`Gripper::wait_reached`] unusable for a
    /// small move: `reach_tolerance` is 1.5 mm and the probe's release is
    /// smaller than that, so "reached" is already true at the first poll
    /// and the wait would return before the fingers had moved at all.
    /// Waiting for the
    /// stall instead costs the ~0.55 s the stall dwell takes and answers
    /// the question actually being asked — where did they end up.
    fn settle_to(&mut self, target: f64, grip: Grip, epics: &dyn RbvSink) {
        match &mut self.backend {
            Backend::Hande(driver) => driver.set_position(target, grip),
            Backend::Simulated { position } => *position = target,
        }
        self.settle_at(target, 0.0, epics);
    }

    /// Blocks until the fingers come within `tolerance` of `target` or stop
    /// moving. Shared by [`Gripper::wait_reached`] and the probe's
    /// partial-release pair, so that "stalled on something" means the same
    /// thing for all of them.
    fn settle_at(&mut self, target: f64, tolerance: f64, epics: &dyn RbvSink) {
        let t0 = Instant::now();
        let mut last_move = t0;
        let mut prev: Option<f64> = None;
        loop {
            let pos = self.position();
            let now = Instant::now();
            if (pos - target).abs() < tolerance {
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
    pub fn update_rbv(&mut self, epics: &dyn RbvSink) {
        let state = i32::from(self.position() >= self.open_threshold);
        if self.last_rbv != Some(state) {
            self.last_rbv = Some(state);
            epics.write_gripper_rbv(state);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct NoRbv;
    impl RbvSink for NoRbv {
        fn write_gripper_rbv(&self, _value: i32) {}
    }

    /// A gripper as the sequence holds it mid-run: simulated backend
    /// parked at `at_m` as if a close had settled there.
    fn gripped_at(at_m: f64) -> Gripper {
        Gripper {
            backend: Backend::Simulated { position: at_m },
            open_position: 0.025,
            close_position: 0.0,
            close_settle_target: 0.01,
            reach_tolerance: 0.0015,
            settle_timeout: Duration::from_secs(1),
            open_threshold: 0.02,
            grip: Grip {
                force: 0.05,
                speed: 0.0,
            },
            min_grip_position: 0.008,
            last_rbv: None,
        }
    }

    /// The holder-2 incident: the puck held at 11.4 mm is gone by the
    /// time the fingers close again, and the settle lands far enough
    /// from the held width that the caller's tolerance comparison must
    /// flag it. The simulated regrip closes all the way to
    /// `close_position`, which is exactly what the real fingers did
    /// over the emptied seat.
    #[test]
    fn a_regrip_that_missed_the_puck_settles_away_from_the_held_width() {
        let mut gr = gripped_at(0.0114);
        let held = gr.position();
        let play = gr.loosen_by(0.0025, &NoRbv);
        assert!((play - 0.0025).abs() < 1e-9);
        let settled = gr.regrip(&NoRbv);
        assert!((settled - held).abs() > gr.reach_tolerance());
    }

    /// The healthy pair: what the fingers held is where they settle
    /// again, inside the band the caller compares with.
    #[test]
    fn a_regrip_onto_the_same_object_settles_where_it_held() {
        let mut gr = gripped_at(0.0);
        let held = gr.position();
        gr.loosen_by(0.0025, &NoRbv);
        let settled = gr.regrip(&NoRbv);
        assert!((settled - held).abs() <= gr.reach_tolerance());
    }

    /// The loosen is clamped to `open_position`: near-open it reports
    /// only the play it actually opened.
    #[test]
    fn the_loosen_never_opens_past_a_plain_open() {
        let mut gr = gripped_at(0.024);
        let play = gr.loosen_by(0.0025, &NoRbv);
        assert!((play - 0.001).abs() < 1e-9);
    }

    /// Zero play must not touch the gripper at all — a Hand-E told to
    /// hold its current position at RELEASE_FORCE stops squeezing.
    #[test]
    fn no_play_asked_for_touches_nothing() {
        let mut gr = gripped_at(0.0114);
        assert_eq!(gr.loosen_by(0.0, &NoRbv), 0.0);
        assert!((gr.position() - 0.0114).abs() < 1e-12);
    }

    /// The empty-close check must never fire on the simulated backend —
    /// its fingers always reach the command exactly, and flagging that
    /// would fail every URSim rehearsal close.
    #[test]
    fn a_simulated_close_is_never_reported_empty() {
        let gr = gripped_at(0.0);
        assert_eq!(gr.empty_close(), None);
    }
}
