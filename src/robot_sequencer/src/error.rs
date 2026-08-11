//! Single string-carrying error, following robotiq-hande's `HandeError`
//! shape: every failure in this daemon either aborts the current sequence
//! (exit, preserving `CurrentStep` for resume) or is logged and degraded,
//! so a taxonomy buys nothing.

use thiserror::Error;

#[derive(Debug, Error)]
#[error("{0}")]
pub struct SequencerError(pub String);

impl From<std::io::Error> for SequencerError {
    fn from(e: std::io::Error) -> Self {
        Self(e.to_string())
    }
}

impl From<cspace_core::error::Error> for SequencerError {
    fn from(e: cspace_core::error::Error) -> Self {
        Self(e.to_string())
    }
}
