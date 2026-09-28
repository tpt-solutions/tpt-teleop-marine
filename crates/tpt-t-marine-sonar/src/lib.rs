#![warn(missing_docs)]
// Matrix/beam algebra reads clearest with index loops; every bound is a
// const, so the loop-shape lint has nothing to offer here.
#![allow(clippy::needless_range_loop)]
#![feature(portable_simd)]
//! `tpt-t-marine-sonar` — sonar processing (spec §4.3).
//!
//! A multibeam echosounder generates 512 beams per ping at 20 Hz — about
//! 10,000 points per second. Standard point-cloud pipelines allocate on
//! every ping; this crate never does:
//!
//! * [`slab`] — a slab-allocated ring of fixed-size clouds. Each ping
//!   overwrites the oldest slab in place; consumers read the latest
//!   [`slab::SLABS`] clouds with no copies and no heap.
//! * [`raytrace`] — SIMD-across-beams processing: sound-velocity
//!   correction (Snell refraction at the transducer, constant-gradient
//!   approximation below), then ray tracing through the mounting rotation
//!   — all `f32x8` lane-parallel, deterministic per beam index.
//!   Spec budget: 2 ms per ping regardless of scene complexity (see
//!   `benches/ping.rs`).
//! * [`sidescan`] — sidescan/forward-looking envelope normalization: TVG
//!   gain compensation and per-channel sliding normalization over the
//!   fixed sample window.

pub mod raytrace;
pub mod sidescan;
pub mod slab;
