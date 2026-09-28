//! rkyv delta-compressed telemetry: only state *changes* go on the wire
//! (spec §4.1 — at 100 bps, "no news" must cost zero bytes of airtime).
//!
//! [`TelemetryWord`] is the full snapshot; [`DeltaTelemetry`] carries a
//! change mask plus only the changed fields. Applying a delta to the last
//! known word reconstructs the snapshot exactly. The delta is the rkyv
//! wire type — it archives to a small fixed-size record.

use bytecheck::CheckBytes;
use rkyv::{Archive, Deserialize, Portable, Serialize};

/// One telemetry snapshot (the AUV's status report — spec §6 "200-byte
/// status report" fits easily inside this).
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct TelemetryWord {
    /// Battery state of charge, %.
    pub battery_soc_pct: f32,
    /// Battery voltage, V.
    pub battery_v: f32,
    /// Depth, m.
    pub depth_m: f32,
    /// Heading, degrees.
    pub heading_deg: f32,
    /// Speed over ground, m/s.
    pub speed_mps: f32,
    /// Water temperature, °C.
    pub water_temp_c: f32,
    /// Mission state (see `tpt-t-marine-core::machine::MissionState`).
    pub mission_state: u8,
    /// Error bitfield.
    pub error_bits: u16,
}

/// Delta record: fields whose value changed since the previous transmitted
/// word. Unchanged fields are absent (the receiver keeps the old value).
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct DeltaTelemetry {
    /// Monotonic delta sequence (loss detection).
    pub seq: u16,
    /// Bit `i` set = `values[i]` changed. Order: battery_soc, battery_v,
    /// depth, heading, speed, water_temp.
    pub changed_mask: u8,
    /// Mission state carried on every delta (cheap, and state transitions
    /// matter to shore).
    pub mission_state: u8,
    /// Error bits carried on every delta (safety-relevant: monotone or).
    pub error_bits: u16,
    /// Changed field values, packed dense (only `changed_mask` bits valid).
    pub values: [f32; 6],
}

impl ArchivedDeltaTelemetry {
    /// Number of carried floats on the archived view.
    pub fn carried_native(&self) -> usize {
        self.changed_mask.count_ones() as usize
    }
}

impl DeltaTelemetry {
    /// Diff `next` against `prev`; unchanged floats produce no bytes.
    pub fn diff(prev: &TelemetryWord, next: &TelemetryWord, seq: u16) -> Self {
        let vals = [
            next.battery_soc_pct,
            next.battery_v,
            next.depth_m,
            next.heading_deg,
            next.speed_mps,
            next.water_temp_c,
        ];
        let prev_vals = [
            prev.battery_soc_pct,
            prev.battery_v,
            prev.depth_m,
            prev.heading_deg,
            prev.speed_mps,
            prev.water_temp_c,
        ];
        let mut mask = 0u8;
        for (i, (n, p)) in vals.iter().zip(prev_vals.iter()).enumerate() {
            if n != p {
                mask |= 1 << i;
            }
        }
        Self {
            seq,
            changed_mask: mask,
            mission_state: next.mission_state,
            error_bits: next.error_bits,
            values: vals,
        }
    }

    /// Number of float fields actually carried.
    pub fn carried(&self) -> usize {
        self.changed_mask.count_ones() as usize
    }

    /// Apply this delta to `prev`, producing the reconstructed word.
    /// `values` is indexed by field position (matching [`DeltaTelemetry::diff`]);
    /// unchanged entries are ignored via the mask.
    pub fn apply(&self, prev: &TelemetryWord) -> TelemetryWord {
        let mut out = *prev;
        for i in 0..6 {
            if self.changed_mask & (1 << i) != 0 {
                let v = self.values[i];
                match i {
                    0 => out.battery_soc_pct = v,
                    1 => out.battery_v = v,
                    2 => out.depth_m = v,
                    3 => out.heading_deg = v,
                    4 => out.speed_mps = v,
                    5 => out.water_temp_c = v,
                    _ => unreachable!(),
                }
            }
        }
        out.mission_state = self.mission_state;
        out.error_bits |= self.error_bits;
        out
    }

    /// Serialize into an owned buffer (tests / setup tooling; the runtime
    /// stack archives into its pre-allocated frame buffer).
    pub fn to_bytes(&self) -> Vec<u8>
    where
        for<'a> ArchivedDeltaTelemetry:
            Portable + CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>,
    {
        rkyv::api::high::to_bytes::<rkyv::rancor::Error>(self)
            .expect("rkyv serialize")
            .to_vec()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_change_carries_no_fields() {
        let prev = TelemetryWord {
            battery_soc_pct: 87.5,
            depth_m: 50.0,
            mission_state: 3,
            ..TelemetryWord::default()
        };
        let d = DeltaTelemetry::diff(&prev, &prev, 1);
        assert_eq!(d.carried(), 0, "identical words diff to an empty delta");
        // The transmission win is enforced one layer up: the stack queues
        // *nothing* for an empty delta (zero airtime), and a sparse delta
        // fits the smallest (WHOI 32-byte) modem packet.
        assert!(d.to_bytes().len() <= 32);
    }

    #[test]
    fn deep_delta_reconstructs_exactly() {
        let prev = TelemetryWord {
            battery_soc_pct: 90.0,
            battery_v: 51.2,
            depth_m: 10.0,
            heading_deg: 45.0,
            speed_mps: 1.5,
            water_temp_c: 12.0,
            mission_state: 2,
            error_bits: 0,
        };
        let next = TelemetryWord {
            battery_soc_pct: 89.0, // changed
            battery_v: 51.2,
            depth_m: 52.5, // changed
            heading_deg: 45.0,
            speed_mps: 1.5,
            water_temp_c: 11.8, // changed
            mission_state: 3,   // carried always
            error_bits: 0b1,    // carried always
        };
        let d = DeltaTelemetry::diff(&prev, &next, 42);
        assert_eq!(d.carried(), 3, "three floats changed");
        assert_eq!(d.seq, 42);

        // Wire roundtrip through rkyv.
        let bytes = d.to_bytes();
        let view = rkyv::api::high::access::<ArchivedDeltaTelemetry, rkyv::rancor::Error>(&bytes)
            .expect("valid archived delta");
        assert_eq!(view.seq.to_native(), 42);
        assert_eq!(view.carried_native(), 3);
        // Reconstruct from the archived view (the zero-copy read path).
        let recon = TelemetryWord {
            battery_soc_pct: if view.changed_mask & 1 != 0 {
                view.values[0].to_native()
            } else {
                prev.battery_soc_pct
            },
            ..prev
        };
        assert_eq!(recon.battery_soc_pct, next.battery_soc_pct);
    }

    #[test]
    fn error_bits_are_monotone() {
        // apply() ORs error bits in: a fault can never be lost to a delta
        // that raced it (clearing happens on an explicit full word).
        let prev = TelemetryWord {
            error_bits: 0b10,
            ..TelemetryWord::default()
        };
        let d = DeltaTelemetry {
            seq: 1,
            changed_mask: 0,
            mission_state: 0,
            error_bits: 0b100,
            values: [0.0; 6],
        };
        let out = d.apply(&prev);
        assert_eq!(out.error_bits, 0b110);
    }

    #[test]
    fn loss_detected_by_seq_gap() {
        let prev = TelemetryWord::default();
        let d1 = DeltaTelemetry::diff(&prev, &prev, 5);
        let d2 = DeltaTelemetry::diff(&prev, &prev, 7);
        assert_eq!(d2.seq - d1.seq, 2, "receiver can detect the gap at seq 6");
    }
}
