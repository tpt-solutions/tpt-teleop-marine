#![warn(missing_docs)]
//! `tpt-t-marine-glider` — buoyancy-driven flight control (spec §3).
//!
//! A glider has no propeller: it flies sawtooth profiles using only
//! buoyancy changes (the ballast pump) and a rudder, at a fraction of a
//! watt — the platform for multi-month, trans-oceanic missions.
//!
//! * [`flight`] — the sawtooth flight controller: pitch via the battery
//!   pack's longitudinal position, buoyancy via the pump, heading via the
//!   rudder; turn-at-depth reverses the profile.
//! * [`power`] — the ultra-low-power profile: instrument duty cycling on
//!   the same 99 %-sleep philosophy as the AUV, with the fix budget that
//!   keeps navigation honest across an ocean.

pub mod flight;
pub mod power;
