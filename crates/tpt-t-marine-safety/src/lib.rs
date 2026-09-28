#![warn(missing_docs)]
//! `tpt-t-marine-safety` — hostile-environment survival (spec §5.4).
//!
//! The safety subsystem owns the reflexes no one else may block:
//!
//! * [`flooding`] — leak alarms (from `tpt-t-marine-pressure`) drive the
//!   emergency-buoyancy action inside the spec's <100 ms reflex budget:
//!   drop the recovery weight, then blow the ballast bladder.
//! * [`thermal`] — temperature escalation: derate → vent → latch shutdown.
//! * [`lost`] — the lost-vehicle procedure: >2 hours of total comms loss
//!   → autonomous ascent → surface drift → GPS beacon via satellite until
//!   recovered.
//! * [`assist`] — integration with the universal teleop safety machine
//!   (`tpt-t-domain-bridge`): an unrecoverable fault raises
//!   `REQUESTING_TELEOP` through the `request_teleop_assistance()` API.
//!
//! Reflexes are plain `Copy` state machines advanced per safety tick
//! (1 kHz default); nothing here allocates, locks, or awaits.

pub mod assist;
pub mod flooding;
pub mod lost;
pub mod thermal;
