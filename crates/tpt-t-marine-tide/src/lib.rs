#![warn(missing_docs)]
//! `tpt-t-marine-tide` — tidal prediction via harmonic constituents
//! (spec §3).
//!
//! Water level and tidal streams are modeled as sums of sinusoidal
//! constituents with astronomically fixed frequencies and site-calibrated
//! amplitude/phase:
//!
//! ```text
//! h(t) = MSL + Σᵢ Aᵢ·cos(ωᵢ·t + φᵢ)
//! ```
//!
//! * [`harmonic`] — the constituent model: the major species (M2, S2, N2,
//!   K1, O1 …) with their standard speeds, least-squares-free direct
//!   evaluation, spring/neap behaviour falls out of the M2+S2 beat.
//! * [`stream`] — tidal streams at arbitrary points: constituent scaling
//!   factors per site cell (the ROV-near-structure use case asks "what is
//!   the current here at 14:00", not "what is it at the tide gauge").

pub mod harmonic;
pub mod stream;
