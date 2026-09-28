#![warn(missing_docs)]
//! `tpt-t-marine-buoyancy` — variable ballast, trim, and buoyancy-engine
//! control (spec §3, unique to the marine domain).
//!
//! * [`physics`] — the compression-at-depth model: neoprene jackets and air
//!   bladders lose volume under hydrostatic pressure, so a vehicle *gains*
//!   negative buoyancy as it dives. Everything else compensates for this.
//! * [`ballast`] — variable ballast tank control: pump water in/out to hold
//!   a target net buoyancy (normally neutral) at the current depth.
//! * [`trim`] — fore/aft trim-pump control: shifts water between end tanks
//!   to hold pitch, using rate-limited pump commands.
//!
//! All controllers are allocation-free and lock-free; each is a `Copy`-sized
//! state machine advanced per control tick.

pub mod ballast;
pub mod physics;
pub mod trim;
