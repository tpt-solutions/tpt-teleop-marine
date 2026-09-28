#![warn(missing_docs)]
// Matrix/beam algebra reads clearest with index loops; every bound is a
// const, so the loop-shape lint has nothing to offer here.
#![allow(clippy::needless_range_loop)]
#![feature(portable_simd)]
//! `tpt-t-marine-nav` — GPS-denied 3D navigation (spec §4.2).
//!
//! An AUV without GPS relies entirely on dead reckoning from its IMU and
//! DVL; the filter runs at 200 Hz to keep drift bounded between updates.
//! This crate provides:
//!
//! * [`ekf`] — the 15-state Extended Kalman Filter: position, velocity,
//!   attitude error, accelerometer biases, gyro biases. The prediction step
//!   is SIMD-vectorized with `portable_simd` (`f32x8`/`f32x4` dot products);
//!   spec budget is `<200 µs` per cycle on a Cortex-A72 at 200 Hz (see
//!   `benches/ekf.rs`).
//! * [`ingest`] — zero-copy sensor ingestion: IMU/DVL packets are validated
//!   rkyv views mapped directly over serial DMA buffers, then fed to the
//!   filter — no rehydration, no heap.
//! * [`convert`] — custom zero-allocation WGS-84 / ECEF / NED / UTM
//!   converters (spec §7 bans `geo`/`proj`).
//! * [`biofouling`] — drag-signature monitoring: as barnacles grow, the
//!   estimated drag coefficient rises and mission speed is derated to
//!   preserve battery (spec §5.4).
//!
//! Design invariants: no allocation on the filter path, no locks, no
//! dynamic dispatch; every structure is fixed-size and `Copy`-able state.

pub mod biofouling;
pub mod convert;
pub mod ekf;
pub mod ingest;
