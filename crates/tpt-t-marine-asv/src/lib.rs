#![warn(missing_docs)]
//! `tpt-t-marine-asv` — autonomous surface vessel control (spec §5.3).
//!
//! * [`dp`] — dynamic positioning: hold position within 1 m in 2-knot
//!   currents using GPS/IMU feedback and a current feed-forward from
//!   `tpt-t-marine-current`.
//! * [`colregs`] — the COLREGS integration: tracks feed the deterministic
//!   engine; emitted maneuvers steer the vessel (Rule 14 starboard
//!   alterations, Rule 15/16 give-way, Rule 17 stand-on depth).
//! * [`waves`] — wave compensation: heave/pitch/roll compensation for
//!   personnel transfer and crane ops.

pub mod colregs;
pub mod dp;
pub mod waves;
