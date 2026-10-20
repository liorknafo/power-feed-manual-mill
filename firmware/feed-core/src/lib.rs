//! Hardware-independent logic for the mill power feed.
//!
//! Everything here is plain `no_std` Rust with no I/O, so it runs unchanged on the RP2040 and is
//! unit-tested on the host (`cargo test`). The firmware (`feed-app`) feeds sensor readings in and
//! applies the returned commands to the hardware.
//!
//! See `docs/design.md` §3 (state machine), §4 (pins), §7 (clutch) for the behaviour implemented here.

#![cfg_attr(not(test), no_std)]
#![cfg_attr(test, allow(clippy::field_reassign_with_default))]

pub mod clutch;
pub mod menu;
pub mod quadrature;
pub mod settings;
pub mod state;
pub mod tmc;
pub mod units;

pub use settings::Settings;
pub use state::{Controller, Fault, Inputs, Mode, Motion, Outputs, State};
