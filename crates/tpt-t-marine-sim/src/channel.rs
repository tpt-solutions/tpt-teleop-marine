//! The acoustic channel: sound-speed latency, range-dependent loss, and
//! multipath fading (spec §4.1: "500 ms–2 s latency, 30–60 % packet
//! loss").

/// Sound speed in seawater, m/s (1500 nominal; spec: 666 µs per km).
pub const SOUND_SPEED_MPS: f64 = 1500.0;

/// Channel model parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelConfig {
    /// Base loss permille at 1 km (Clearwater + spreading).
    pub base_loss_permille: u16,
    /// Additional loss permille per extra kilometre.
    pub loss_per_km_permille: u16,
    /// Modem processing latency (one way), microseconds.
    pub processing_latency_us: u64,
    /// Multipath tap delays relative to the direct path, microseconds.
    /// Energy splits across taps; the receiver's synchronizer usually
    /// locks the strongest (first) tap — delayed taps are modeled as a
    /// ghost-echo probability.
    pub multipath_delays_us: [u64; 3],
}

impl ChannelConfig {
    /// A shallow-water survey channel.
    pub const fn shallow_survey() -> Self {
        Self {
            base_loss_permille: 100,
            loss_per_km_permille: 120,
            processing_latency_us: 500_000,
            multipath_delays_us: [2_000, 5_000, 9_000],
        }
    }
}

/// One simulated transmission outcome.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TxOutcome {
    /// Whether the frame decoded on the direct path.
    pub delivered: bool,
    /// Total latency (propagation + processing), microseconds.
    pub latency_us: u64,
    /// Whether a multipath ghost also carries energy (sync ambiguity).
    pub ghost: bool,
}

/// The channel: deterministic given a loss roll (`0..1000` supplied by the
/// caller — the harness owns the RNG so the sim stays reproducible).
#[derive(Debug, Clone, Copy)]
pub struct AcousticChannel {
    /// Channel parameters.
    pub cfg: ChannelConfig,
}

impl AcousticChannel {
    /// Bind a channel.
    pub const fn new(cfg: ChannelConfig) -> Self {
        Self { cfg }
    }

    /// One-way propagation latency for a range, microseconds (666 µs/km).
    pub fn propagation_us(&self, range_m: f64) -> u64 {
        (range_m / SOUND_SPEED_MPS * 1.0e6) as u64
    }

    /// Simulate one frame transmission over `range_m`. `loss_roll` is a
    /// 0..1000 draw; the frame survives when the roll exceeds the total
    /// loss. Multipath ghosts appear when the roll lands in the first
    /// loss band's ghost window.
    pub fn transmit(&self, range_m: f64, loss_roll: u16) -> TxOutcome {
        let km = (range_m / 1000.0).max(0.0);
        let total_loss = (self.cfg.base_loss_permille as u32
            + (km * self.cfg.loss_per_km_permille as f64) as u32)
            .min(990) as u16;
        let delivered = loss_roll >= total_loss;
        let ghost = loss_roll > total_loss.saturating_sub(total_loss / 4);
        TxOutcome {
            delivered,
            latency_us: self.propagation_us(range_m) + self.cfg.processing_latency_us,
            ghost: ghost && !delivered,
        }
    }

    /// Round-trip command+ack latency budget at a range, microseconds.
    pub fn round_trip_us(&self, range_m: f64) -> u64 {
        2 * (self.propagation_us(range_m) + self.cfg.processing_latency_us)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chan() -> AcousticChannel {
        AcousticChannel::new(ChannelConfig::shallow_survey())
    }

    #[test]
    fn latency_matches_the_666us_per_km_law() {
        let c = chan();
        // 1500 m one way: 1 s of propagation + 0.5 s processing.
        let l = c.propagation_us(1500.0);
        assert!((1_000_000u64..=1_000_100).contains(&l), "prop {l}");
        let rt = c.round_trip_us(1500.0);
        assert!(rt >= 3_000_000, "round trip {rt}");
    }

    #[test]
    fn loss_grows_with_range() {
        let c = chan();
        let mut good_close = 0;
        let mut good_far = 0;
        for roll in 0..1000u16 {
            if c.transmit(100.0, roll).delivered {
                good_close += 1;
            }
            if c.transmit(8000.0, roll).delivered {
                good_far += 1;
            }
        }
        assert!(good_close > good_far, "farther must lose more");
        // At 8 km the loss is capped at 99 %: some frames still pass.
        assert!(good_far > 0);
    }

    #[test]
    fn multipath_ghosts_appear_in_the_loss_band() {
        let c = chan();
        let out = c.transmit(8000.0, 980); // deep in the loss band
        assert!(!out.delivered);
        assert!(out.ghost, "a delayed tap carries energy");
    }
}
