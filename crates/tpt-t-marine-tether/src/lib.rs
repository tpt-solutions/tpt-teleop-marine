#![warn(missing_docs)]
//! `tpt-t-marine-tether` — the tether management system (spec §4.4).
//!
//! An ROV tether in a 2-knot current forms a complex 3D catenary; if the
//! ROV moves unpredictably the tether snags on subsea structures. The
//! model: a lumped-mass chain of [`chain::NODES`] nodes solved by Verlet
//! integration, stepped at 100 Hz on a dedicated pinned core, predicting
//! the shape 5 seconds ahead. Snag risk alerts route to the ROV pilot via
//! `tpt-t-teleop`.
//!
//! * [`chain`] — the Verlet chain: gravity/buoyancy, quadratic water drag,
//!   distance constraints, a pinned head (the TMS at the surface) and the
//!   tail clamped to the ROV.
//! * [`predict`] — the 5-second-ahead predictor: clone the state, roll the
//!   physics forward, and score clearance against the obstacle set.
//! * [`winch`] — tension estimation and winch (TMS) control holding the
//!   tension window.

pub mod chain;
pub mod predict;
pub mod winch;
