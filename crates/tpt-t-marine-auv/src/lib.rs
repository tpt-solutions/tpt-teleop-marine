#![warn(missing_docs)]
//! `tpt-t-marine-auv` — autonomous underwater vehicle mission execution
//! (spec §5.1).
//!
//! * [`waypoints`] — 3D waypoint following: line-of-sight guidance with a
//!   cross-track error and depth loop, emitting normalized body commands.
//! * [`adaptive`] — adaptive sampling: detect a thermocline or plume from
//!   the temperature/sensor trend and extend survey time inside the
//!   battery/time budget (spec: "the AUV autonomously modifies its
//!   mission to spend more time sampling that feature").
//! * [`power`] — sleep-wake power management for multi-month deployments:
//!   99 % duty sleeping, waking on a scheduled profile or an acoustic
//!   page-up from the surface buoy.
//! * [`surfacing`] — comms-triggered behaviors: on surfacing, establish
//!   the satellite link, transmit the compressed summary, and accept new
//!   waypoints or an abort before the next dive.

pub mod adaptive;
pub mod power;
pub mod surfacing;
pub mod waypoints;
