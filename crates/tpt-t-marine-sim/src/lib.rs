#![warn(missing_docs)]
//! `tpt-t-marine-sim` — the headless simulator (spec §8).
//!
//! Testing underwater vehicles requires the ocean, which is expensive and
//! weather-dependent. This crate models the pieces every workspace crate
//! needs against a deterministic, allocation-free core:
//!
//! * [`dynamics`] — 6DOF-lite vehicle dynamics with per-axis
//!   hydrodynamic drag coefficients (surge/sway/heave translation plus
//!   yaw; roll/pitch are righting-stabilized).
//! * [`channel`] — the acoustic channel: 666 µs per kilometre of latency,
//!   range-dependent loss, and three-tap multipath fading.
//! * [`currents`] — ocean current fields: uniform + sinusoidal tidal
//!   component (user-defined fields plug in the same trait), standing in
//!   for HYCOM/ROMS inputs.
//!
//! The 100×-real-time target (a 30-day mission in 7 hours of wall clock)
//! is a throughput property, gated by the `real_time_factor` test.
//! The visualizer (`crates/tpt-t-marine-visualizer`) renders trajectories,
//! link heatmaps, catenaries, point clouds, and COLREGS scenarios against
//! logged runs of this simulator.

pub mod channel;
pub mod currents;
pub mod dynamics;
