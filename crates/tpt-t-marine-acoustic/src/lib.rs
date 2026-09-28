#![warn(missing_docs)]
// Result<(), ()> is the deliberate signal for "busy/full — retry next
// tick" across the data plane; a custom error adds nothing.
#![allow(clippy::result_unit_err)]
//! `tpt-t-marine-acoustic` — the 100-bps acoustic protocol stack (spec §4.1).
//!
//! Radio does not penetrate water: acoustic modems offer 100 bps–10 kbps
//! with 500 ms–2 s latency and 30–60 % packet loss. TCP/IP is unusable; this
//! crate implements the custom stack instead:
//!
//! * [`gf`] — GF(2⁸) arithmetic (const-evaluated tables, no heap).
//! * [`rs`] — Reed-Solomon forward error correction over GF(2⁸),
//!   systematic encoding with Berlekamp-Massey / Chien / Forney decoding;
//!   the parity budget is tuned to the *measured* packet loss rate.
//! * [`arq`] — Adaptive ARQ: sliding window with selective repeat, driven
//!   by per-frame timeouts. Adapts window size to observed loss.
//! * [`critical`] — triple-redundancy send path for mission-critical
//!   commands ("abort, surface now"): three interleaved copies, 2-of-3
//!   majority vote with checksum vetting.
//! * [`delta`] — rkyv delta-compressed telemetry: only state *changes* go
//!   on the wire.
//! * [`stack`] — the whole protocol stack in one pre-allocated 4 KB buffer:
//!   no heap, no `Vec`, no `String` (spec hard requirement).
//! * [`modem`] — vendor integration profiles (WHOI Micro-Modem, EvoLogics,
//!   LinkQuest) behind one [`modem::Modem`] trait.

pub mod arq;
pub mod critical;
pub mod delta;
pub mod gf;
pub mod modem;
pub mod rs;
pub mod stack;

/// Maximum size of one acoustic frame on the wire (bytes).
pub const MAX_FRAME: usize = 255;
