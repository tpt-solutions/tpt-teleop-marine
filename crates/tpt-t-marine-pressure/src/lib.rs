#![warn(missing_docs)]
//! `tpt-t-marine-pressure` — depth/pressure sensor management (spec §5.4).
//!
//! Every sealed compartment carries a pressure sensor; this crate turns raw
//! samples into trustworthy depth estimates and life-or-death leak alarms.
//!
//! * [`depth`] — raw pressure → depth with seawater density, temperature
//!   compensation (zero/span drift), and dynamic hysteresis (sensor-lag)
//!   correction for ascending/descending profiles.
//! * [`leak`] — pressure-differential monitoring per compartment. A
//!   `0.1 PSI` rise above the slow-drift baseline triggers a
//!   [`leak::LeakAlarm`] that the safety crate turns into an emergency
//!   buoyancy action. Detection latency is bounded: with sensors sampled at
//!   the standard 100 Hz ingress-monitoring rate the alarm fires within the
//!   spec's `<100 ms` budget (see `leak::LeakDetector::update`).

pub mod depth;
pub mod leak;
pub mod wire;
