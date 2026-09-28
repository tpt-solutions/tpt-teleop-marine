#![warn(missing_docs)]
//! `tpt-t-marine-offshore` — offshore & infrastructure applications
//! (spec §3).
//!
//! * [`turbine`] — wind-turbine foundation inspection: spiral descent
//!   patterns around a monoplane/jacket, scour-zone coverage.
//! * [`cable`] — subsea cable routing: route following with burial-depth
//!   assessment stations.
//! * [`wellhead`] — oil & gas wellhead monitoring: standoff-protected
//!   approach orbit with anomaly station-keeping.

pub mod cable;
pub mod turbine;
pub mod wellhead;
