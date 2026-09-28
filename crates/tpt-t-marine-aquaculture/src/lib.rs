#![warn(missing_docs)]
//! `tpt-t-marine-aquaculture` — fish-farming applications (spec §3).
//!
//! * [`net`] — fish-cage net inspection: grid coverage of the cage walls
//!   with tear/deformation detection from sonar anomaly scores.
//! * [`biomass`] — biomass estimation via hydroacoustics: echo
//!   integration over the acoustic volume → standing stock.
//! * [`mortality`] — mortality detection: bottom-target clustering from
//!   the echo grid (dead fish sink; the count estimates losses).

pub mod biomass;
pub mod mortality;
pub mod net;
