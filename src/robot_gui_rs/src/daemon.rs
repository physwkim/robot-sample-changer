//! What the daemon says it is doing, and whether it is still saying it.
//!
//! Every control on this GUI that needs the sequencer to be listening
//! asks here, and none of them infers it from a run value. The
//! difference matters because the run values outlive the daemon:
//! `Robot:CurrentStep` is the resume-after-crash marker and autosave
//! restores it, `Robot:Jog:Target` keeps the last hold's seat, so a
//! button drawn from either is offered over the wreckage of a run and
//! does nothing when pressed -- or, worse, starts a fresh one.
//!
//! `Robot:State` names the loop the daemon is in and `Robot:Alive`
//! counts its service passes. Neither alone is enough: a state that is
//! not being re-stamped is exactly the stale reading this replaces.

use std::cell::Cell;
use std::time::{Duration, Instant};

use rsdm::{Channel, Engine, EngineError, PvValue};

use crate::pvs::{robot, state_name};

/// How long `Robot:Alive` may stand still before a daemon that promised
/// beats is treated as gone. It beats every service pass -- 10 Hz in
/// every standing loop -- so this is twenty missed beats: long enough
/// to ride out a slow put, short enough that an operator does not press
/// a dead button twice.
const BEAT_STALE: Duration = Duration::from_secs(2);

/// `Robot:State` codes this GUI acts on. The daemon's `DaemonState`
/// owns the numbering.
pub const IDLE: i64 = 0;
pub const RUNNING: i64 = 1;
pub const MEASUREMENT_WAIT: i64 = 2;
pub const HOLD: i64 = 4;

/// What the two records together say about the daemon right now.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Presence {
    /// Standing in this state and reading operator commands this
    /// instant. The only presence in which a command control is honest.
    Listening(i64),
    /// Working: moving the arm or driving the gripper. The daemon says
    /// so itself, and the beat is quiet because a working daemon does
    /// not service commands -- so this is neither listening nor
    /// evidence of death, and must not be painted as either.
    Working,
    /// Promised a beat and stopped, or never said anything. Dead,
    /// disconnected, or wedged; from here they are one thing.
    Silent(Option<i64>),
}

pub struct DaemonWatch {
    state: Channel,
    alive: Channel,
    /// The last `Alive` update seen, and when this GUI saw it. A `Cell`
    /// so that reading the watch is enough to keep it current: a
    /// separate per-frame sample would be a second place that has to
    /// remember to run, which is the shape of bug this module exists
    /// to remove.
    seen: Cell<Option<(u64, Instant)>>,
}

impl DaemonWatch {
    pub fn new(engine: &Engine) -> Result<Self, EngineError> {
        Ok(Self {
            state: engine.connect(&robot("State"))?,
            alive: engine.connect(&robot("Alive"))?,
            seen: Cell::new(None),
        })
    }

    /// The whole reading, in one place. `Running` is decided by the
    /// state alone: the daemon publishes it around every blocking piece
    /// of work precisely so that the quiet which follows is accounted
    /// for, and a beat left over from the moment it started moving does
    /// not make it listening.
    pub fn read(&self) -> Presence {
        presence(self.raw_state(), self.beating())
    }

    /// Whether the daemon is in a loop that reads operator commands
    /// right now. False while it works, which is the truth of it: the
    /// gripper command, the jog and `Wait` are not read there either.
    pub fn servicing(&self) -> bool {
        matches!(self.read(), Presence::Listening(_))
    }

    /// True when the daemon is standing in `code` and servicing it.
    pub fn is(&self, code: i64) -> bool {
        self.read() == Presence::Listening(code)
    }

    /// Why a trigger cannot start a run right now, or `None` when it
    /// can.
    ///
    /// A run begins at the idle trigger wait and nowhere else. The
    /// daemon reads `Robot:Trigger` there, and drops whatever it finds
    /// in the record when any other wait opens -- so a start button
    /// offered from a measurement wait, or from the middle of a
    /// trajectory, writes a record nobody will read.
    pub fn not_ready_for_a_run(&self) -> Option<String> {
        match self.read() {
            Presence::Listening(IDLE) => None,
            Presence::Listening(s) => Some(format!(
                "a run starts at the idle wait — the daemon is in the {}",
                state_name(s)
            )),
            _ => Some(self.note()),
        }
    }

    /// Why a control is greyed, in the words the status row uses.
    pub fn note(&self) -> String {
        match self.read() {
            Presence::Listening(s) => format!("daemon is {}", state_name(s)),
            Presence::Working => "daemon is working — it reads no commands while it moves".into(),
            Presence::Silent(Some(s)) => {
                format!("daemon not responding — last said {}", state_name(s))
            }
            Presence::Silent(None) => "daemon not responding".into(),
        }
    }

    /// The status row: what to print, and how loudly.
    pub fn status(&self) -> (String, Tone) {
        match self.read() {
            Presence::Listening(s) => (state_name(s).to_string(), Tone::Good),
            Presence::Working => (
                format!("{} — no commands read", state_name(RUNNING)),
                Tone::Busy,
            ),
            Presence::Silent(Some(s)) => {
                (format!("NOT RESPONDING (was {})", state_name(s)), Tone::Bad)
            }
            Presence::Silent(None) => ("NOT RESPONDING".into(), Tone::Bad),
        }
    }

    /// Whether `Robot:Alive` has moved recently enough to count.
    fn beating(&self) -> bool {
        let stamp = self.alive.state().stamp;
        let seen = match self.seen.get() {
            Some(seen) if seen.0 == stamp => seen,
            _ => (stamp, Instant::now()),
        };
        self.seen.set(Some(seen));
        self.alive.state().connected && seen.1.elapsed() < BEAT_STALE
    }

    fn raw_state(&self) -> Option<i64> {
        self.state.state().value.as_ref().and_then(PvValue::as_i64)
    }
}

/// The whole reading, as a function of the two records. `Running` is
/// decided by the state alone: the daemon publishes it around every
/// blocking piece of work precisely so that the quiet which follows is
/// accounted for, and a beat left over from the moment it started
/// moving does not make it listening.
fn presence(state: Option<i64>, beating: bool) -> Presence {
    if state == Some(RUNNING) {
        return Presence::Working;
    }
    match (beating, state) {
        (true, Some(s)) => Presence::Listening(s),
        (true, None) => Presence::Silent(None),
        (false, s) => Presence::Silent(s),
    }
}

/// How a status line should read: the panels own the palette, this
/// module owns which of the three a reading deserves.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Tone {
    Good,
    Busy,
    Bad,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One case per boundary of the pair, because the bug this replaces
    /// was a boundary nobody had a case for: a state that reads right
    /// while nothing is re-stamping it.
    #[test]
    fn only_a_beating_standing_state_is_listening() {
        assert_eq!(presence(Some(HOLD), true), Presence::Listening(HOLD));
        assert_eq!(presence(Some(HOLD), false), Presence::Silent(Some(HOLD)));
        // The one the measurement wait's Continue turned on.
        assert_eq!(
            presence(Some(MEASUREMENT_WAIT), true),
            Presence::Listening(MEASUREMENT_WAIT)
        );
        assert_eq!(
            presence(Some(MEASUREMENT_WAIT), false),
            Presence::Silent(Some(MEASUREMENT_WAIT))
        );
        // Working either way: the beat at the moment it started moving
        // is not a promise that it is reading.
        assert_eq!(presence(Some(RUNNING), true), Presence::Working);
        assert_eq!(presence(Some(RUNNING), false), Presence::Working);
        // No state at all is not a state to service.
        assert_eq!(presence(None, true), Presence::Silent(None));
        assert_eq!(presence(None, false), Presence::Silent(None));
    }
}
