#![warn(missing_docs)]
//! `tpt-t-marine-current` — ocean current modeling (spec §3).
//!
//! * [`adcp`] — ingestion of Acoustic Doppler Current Profiler data:
//!   depth-binned east/north velocity profiles with QC (side-lobe
//!   contamination near the surface/bottom, outlier bins).
//! * [`drift`] — drift prediction for AUVs and ASVs: depth-weighted
//!   averaging of the profile into one transport current, projected
//!   forward to answer "where will the vehicle be in N minutes if it dies
//!   on the spot" (recovery planning) and "what current do I fight at
//!   depth d" (station keeping).

pub mod adcp;
pub mod drift;
