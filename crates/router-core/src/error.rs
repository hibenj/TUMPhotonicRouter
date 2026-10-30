//! The error type of the core's fallible entry points that used to return
//! a Python exception directly (phase 8 slice B of the 2026-10 do-over).
//! Each variant names the Python exception the binding raises for it
//! (`impl From<RouterError> for PyErr` in `router-py`); the message is
//! carried unchanged.

use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RouterError {
    /// An invalid configuration or argument value; Python `ValueError`.
    Config(String),
}

impl RouterError {
    /// The message without the kind.
    pub fn message(&self) -> &str {
        match self {
            RouterError::Config(message) => message,
        }
    }
}

impl fmt::Display for RouterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.message())
    }
}

impl std::error::Error for RouterError {}
