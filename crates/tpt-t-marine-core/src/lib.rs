#![warn(missing_docs)]
//! `tpt-t-marine-core` — the shared foundation for the `tpt-teleop-marine`
//! workspace.
//!
//! Provides:
//! * [`machine`] — the mission state machine (Dive → Transit → Survey →
//!   Surface → Recover) for AUV/ASV/glider missions.
//! * [`bus`] — lock-free single-producer / single-consumer ring buffers used
//!   for zero-alloc inter-crate event routing on the hot path.
//! * [`event_loop`] — a central event-loop skeleton that drains the bus and
//!   dispatches events without allocating per tick.
//! * [`wire`] — the zero-copy rkyv wire-type prelude shared by the other
//!   marine crates (poses, waypoints, 6DOF thruster commands, health).
//!
//! Design invariants (inherited from the `tpt-teleop` sibling repos): no
//! async runtime, no `serde`, no channels-with-mutexes. The forward data
//! plane makes no per-event heap allocations and takes no locks on the
//! steady-state path.

pub mod bus;
pub mod event_loop;
pub mod machine;
pub mod nmea;
pub mod wire;
