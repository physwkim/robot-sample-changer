//! The RTDE output stream, and who is allowed to leave it running.
//!
//! URControl drops a client that stops reading: the socket buffer is
//! 131 KB and at the configured rate it fills in well under a second.
//! Twice in one session this daemon lost the robot that way — once
//! because bring-up returned with the stream on and the Hand-E
//! activation read nothing for 1.4 s, once because `fresh_q` returned
//! with it on and planning read nothing for a second. Both times the
//! failure surfaced as an RTDE error at the *next* step, never at the
//! code that caused it, and both times the fix was a `pause` call at a
//! call site that had to remember to make it.
//!
//! So the stream is not something a caller turns on. It is borrowed for
//! as long as the caller reads from it, and stops when that borrow ends:
//!
//! ```ignore
//! let mut session = self.rtde.session()?;   // starts it
//! let q = session.fresh_q()?;
//! // session drops here — stream paused, whatever happens next
//! ```
//!
//! A third caller cannot reintroduce the bug by forgetting; there is
//! nothing to forget.

use std::time::Duration;

use ur_driver::rtde::{DataPackage, RtdeClient, RtdeValue};
use ur_driver::types::Vector6D;
use ur_driver::ur::version_information::VersionInformation;

use crate::error::SequencerError;
use crate::log;

/// Packages [`Session::fresh_q`] discards before taking its sample, so
/// the joint values are current rather than whatever was in flight when
/// the stream was last paused.
///
/// Three, not the hundred this started as: that number was sized for the
/// backlog a stream left running through planning and idling built up,
/// which the session cannot leave behind. At 50 Hz a hundred-deep drain
/// would cost two seconds per motion step.
const DRAIN_PACKAGES: usize = 3;

pub struct RtdeStream {
    client: RtdeClient,
    streaming: bool,
    /// Set when a pause failed, i.e. URControl had already dropped the
    /// connection. Only decorates the next `start` error — re-establishing
    /// the connection needs a daemon restart.
    lost: bool,
}

impl RtdeStream {
    pub fn connect(
        host: &str,
        port: u16,
        output_recipe: Vec<String>,
        input_recipe: Vec<String>,
        frequency_hz: f64,
    ) -> Result<Self, SequencerError> {
        let mut client = RtdeClient::connect(host, port, output_recipe, input_recipe, frequency_hz)
            .map_err(|e| SequencerError(format!("RTDE connect: {e}")))?;
        client
            .init()
            .map_err(|e| SequencerError(format!("RTDE init: {e}")))?;
        Ok(Self {
            client,
            streaming: false,
            lost: false,
        })
    }

    pub fn urcontrol_version(&self) -> VersionInformation {
        self.client.urcontrol_version()
    }

    /// The controller's own rate, which is what the servoj period is
    /// derived from — not the output rate this stream was opened at.
    pub fn max_frequency(&self) -> f64 {
        self.client.max_frequency()
    }

    pub fn send_speed_slider(&mut self, fraction: f64) -> Result<(), SequencerError> {
        let sent = self
            .client
            .writer()
            .ok_or_else(|| SequencerError("RTDE writer unavailable".into()))?
            .send_speed_slider(fraction);
        if sent {
            Ok(())
        } else {
            Err(SequencerError("cannot send speed slider".into()))
        }
    }

    /// Borrows the stream for reading, starting it if it is paused. It
    /// stops again when the returned session drops.
    pub fn session(&mut self) -> Result<Session<'_>, SequencerError> {
        if !self.streaming {
            let lost = self.lost;
            self.client.start().map_err(|e| {
                SequencerError(format!(
                    "RTDE start: {e}{}",
                    if lost {
                        " (the RTDE connection was lost earlier; \
                         restart the daemon to re-establish it)"
                    } else {
                        ""
                    }
                ))
            })?;
            self.streaming = true;
            self.lost = false;
        }
        Ok(Session { stream: self })
    }

    /// A failed pause means the connection is already gone. Clearing
    /// `streaming` anyway is what keeps that recoverable: left set, the
    /// next `session` would hand out a reader over a dead socket instead
    /// of retrying `start` and reporting why it failed.
    fn pause(&mut self) {
        if self.streaming {
            if let Err(e) = self.client.pause() {
                log::warn(&format!("RTDE pause failed: {e}"));
                self.lost = true;
            }
            self.streaming = false;
        }
    }
}

/// A borrow of the running stream. See the module doc.
pub struct Session<'a> {
    stream: &'a mut RtdeStream,
}

impl Session<'_> {
    pub fn read(&mut self) -> Result<DataPackage, SequencerError> {
        self.stream
            .client
            .get_data_package()
            .map_err(|e| SequencerError(format!("RTDE read: {e}")))
    }

    /// Current joint positions, drained so the sample reflects the
    /// present rather than what was in flight when the stream last
    /// stopped.
    pub fn fresh_q(&mut self) -> Result<Vector6D, SequencerError> {
        for _ in 0..DRAIN_PACKAGES {
            self.read()?;
        }
        match self.read()?.get("actual_q") {
            Some(RtdeValue::V6D(q)) => Ok(*q),
            other => Err(SequencerError(format!(
                "actual_q missing from RTDE package: {other:?}"
            ))),
        }
    }

    /// Reads until `field` equals `value`, or the deadline passes.
    pub fn wait_for_f64(
        &mut self,
        field: &str,
        value: f64,
        timeout: Duration,
    ) -> Result<(), SequencerError> {
        let deadline = std::time::Instant::now() + timeout;
        loop {
            if let Some(RtdeValue::F64(actual)) = self.read()?.get(field)
                && (actual - value).abs() < 1e-6
            {
                return Ok(());
            }
            if std::time::Instant::now() > deadline {
                return Err(SequencerError(format!(
                    "{field} did not reach {value} within {:.0} s",
                    timeout.as_secs_f64()
                )));
            }
        }
    }
}

impl Drop for Session<'_> {
    fn drop(&mut self) {
        self.stream.pause();
    }
}
