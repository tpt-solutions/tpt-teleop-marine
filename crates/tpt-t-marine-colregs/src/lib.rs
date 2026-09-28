#![warn(missing_docs)]
//! `tpt-t-marine-colregs` — maritime law as a deterministic state machine
//! (spec §4.5: "not ML-based; every decision traceable, auditable, and
//! legally defensible").
//!
//! The engine evaluates every tracked vessel each radar/AIS tick:
//!
//! * [`track`] — the target-track vocabulary and the geometric predicates
//!   (relative bearing, aspect, CBDR — constant bearing, decreasing
//!   range).
//! * [`encounter`] — encounter classification per Rules 13/14/15
//!   (overtaking, head-on, crossing) and the per-vessel give-way /
//!   stand-on FSM with Rule 17 depth-of-action (hold → sound doubt →
//!   act alone).
//! * [`whistle`] — Rule 34 manoeuvring sound signals as a decision
//!   output (1 short = altering to starboard, 2 = port, 3 = astern
//!   propulsion, 5 = doubt).
//! * [`log`] — the auditable decision log: a fixed ring of decisions with
//!   rule citations, timestamps, and the inputs that produced them.
//!
//! All evaluation is branchy integer/f64 arithmetic on fixed arrays — the
//! spec budget is `<50 µs` per detected vessel per tick (see the
//! `budget_per_vessel` test).

pub mod encounter;
pub mod engine;
pub mod log;
pub mod track;
pub mod whistle;
