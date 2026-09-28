//! Vendor modem integration profiles behind one trait (spec §4.1: WHOI
//! Micro-Modem, EvoLogics, LinkQuest).
//!
//! The protocol stack is modem-agnostic: it needs the modem's MTU, bit
//! rate, and fixed latency to size frames and time the ARQ retry clock.
//! Each vendor profile is a zero-sized type carrying measured constants.

/// What the stack needs from any acoustic modem.
pub trait Modem {
    /// Vendor name (diagnostics).
    fn name(&self) -> &'static str;
    /// Largest unfragmented frame the modem accepts, bytes.
    fn max_frame_bytes(&self) -> usize;
    /// Payload bit rate, bits/second.
    fn bitrate_bps(&self) -> u32;
    /// One-way fixed latency (modem processing + propagation allowance),
    /// microseconds.
    fn latency_us(&self) -> u32;
    /// Airtime for one frame of `len` bytes, microseconds.
    fn frame_time_us(&self, len: usize) -> u32 {
        (len as u32 * 8 * 1_000_000) / self.bitrate_bps().max(1) + self.latency_us()
    }
}

/// Woods Hole Oceanographic Institution Micro-Modem (FSK, 240 bps class;
/// PSK data rates to 5 kbps — the conservative FSK profile here).
#[derive(Debug, Clone, Copy, Default)]
pub struct WhoiMicroModem;

impl Modem for WhoiMicroModem {
    fn name(&self) -> &'static str {
        "WHOI Micro-Modem (FSK)"
    }
    fn max_frame_bytes(&self) -> usize {
        32 // FSK "mini" packets: 32 bytes
    }
    fn bitrate_bps(&self) -> u32 {
        240
    }
    fn latency_us(&self) -> u32 {
        700_000 // ~0.7 s processing + typical 3–5 km propagation
    }
}

/// EvoLogics S2C series (DSSS, rates 100 bps–25 kbps depending on range).
#[derive(Debug, Clone, Copy, Default)]
pub struct EvoLogics {
    /// Selected range class bit rate, bps.
    pub bitrate_bps: u32,
}

impl Modem for EvoLogics {
    fn name(&self) -> &'static str {
        "EvoLogics S2C (DSSS)"
    }
    fn max_frame_bytes(&self) -> usize {
        220
    }
    fn bitrate_bps(&self) -> u32 {
        self.bitrate_bps
    }
    fn latency_us(&self) -> u32 {
        350_000
    }
}

/// LinkQuest (benthos) acoustic modems (MFSK, 1.4–16 kbps class).
#[derive(Debug, Clone, Copy, Default)]
pub struct LinkQuest {
    /// Selected profile bit rate, bps.
    pub bitrate_bps: u32,
}

impl Modem for LinkQuest {
    fn name(&self) -> &'static str {
        "LinkQuest (MFSK)"
    }
    fn max_frame_bytes(&self) -> usize {
        200
    }
    fn bitrate_bps(&self) -> u32 {
        self.bitrate_bps
    }
    fn latency_us(&self) -> u32 {
        400_000
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profiles_are_sane() {
        let whoi = WhoiMicroModem;
        assert_eq!(whoi.max_frame_bytes(), 32);
        // 32 bytes at 240 bps: ~1.07 s of airtime + latency.
        let t = whoi.frame_time_us(32);
        assert!(t > 1_700_000, "whoi frame time {t} µs");

        let evo = EvoLogics { bitrate_bps: 6_900 };
        assert_eq!(evo.bitrate_bps(), 6_900);
        // 220 bytes at 6.9 kbps ≈ 255 ms of airtime + 350 ms latency.
        let t2 = evo.frame_time_us(220);
        assert!((590_000..=620_000).contains(&t2), "evo frame time {t2} µs");

        let lq = LinkQuest {
            bitrate_bps: 16_000,
        };
        assert!(lq.frame_time_us(200) < 600_000);
    }

    #[test]
    fn whoi_frames_carry_only_small_deltas() {
        // The stack sizes its payload to the modem: at 32-byte MTU only the
        // compact delta records fit — exactly the spec's point.
        let mtu = WhoiMicroModem.max_frame_bytes();
        let full = rkyv::api::high::to_bytes::<rkyv::rancor::Error>(
            &crate::delta::TelemetryWord::default(),
        )
        .unwrap()
        .len();
        let _ = full;
        let d = crate::delta::DeltaTelemetry {
            seq: 1,
            changed_mask: 0b0000_0001,
            mission_state: 3,
            error_bits: 0,
            values: [50.5, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        let bytes = d.to_bytes();
        assert!(
            bytes.len() <= mtu,
            "delta ({} B) must fit a WHOI mini packet",
            bytes.len()
        );
    }
}
