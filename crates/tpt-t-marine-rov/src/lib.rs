#![warn(missing_docs)]
//! `tpt-t-marine-rov` — tethered ROV control (spec §5.2).
//!
//! * [`thrusters`] — 6DOF thruster coordination: mix a normalized body
//!   command into per-thruster outputs (8-thruster configuration),
//!   saturating gracefully.
//! * [`arm`] — manipulator arm control: joint targets with rate limits
//!   (the pilot's second joystick / VR gloves).
//! * [`tether_guard`] — tether-aware motion: refuses or limits commands
//!   that would grow snag risk (consumes `tpt-t-marine-tether`'
//!   prediction) and drives the pilot's haptic feedback.

pub mod arm;
pub mod tether_guard;
pub mod thrusters;
