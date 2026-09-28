#![warn(missing_docs)]
// Result<(), ()> is the deliberate signal for "busy/full — retry next
// tick" across the data plane; a custom error adds nothing.
#![allow(clippy::result_unit_err)]
//! `tpt-t-marine-comms` — the multi-modal link manager (spec §3).
//!
//! A marine vehicle crosses four communication regimes in one mission:
//! acoustic (submerged), RF (surface), cellular (near shore), and satellite
//! (open ocean). This crate:
//!
//! * [`link`] — models each modality's quality envelope and selects the
//!   active link with hysteresis, so a marginal link does not flap the
//!   vehicle between modes.
//! * [`store`] — custom store-and-forward: when every link is down,
//!   telemetry and mission updates queue in a fixed-capacity ring and drain
//!   in order when a link returns (spec: no data loss, no allocation).
//! * [`qos`] — fiber-optic tether bandwidth prioritization: control
//!   packets (<1 ms) beat H.265 video beats sonar data, with bounded
//!   per-class buffers so a sonar flood cannot delay control.

pub mod link;
pub mod qos;
pub mod store;
