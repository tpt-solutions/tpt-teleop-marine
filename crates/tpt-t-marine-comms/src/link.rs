//! Link modalities, quality envelopes, and hysteresis-gated selection.

use core::fmt;

/// The four communication regimes a mission crosses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Modality {
    /// Underwater acoustic modem (100 bps–10 kbps, high loss).
    Acoustic = 0,
    /// Surface / near-surface radio (WiFi-class, short range).
    Rf = 1,
    /// Cellular (near-shore missions).
    Cellular = 2,
    /// Satellite (open ocean; global, low rate, priced per byte).
    Satellite = 3,
}

/// All modalities in selection priority order (best service first when
/// quality ties).
pub const ALL: [Modality; 4] = [
    Modality::Rf,
    Modality::Cellular,
    Modality::Satellite,
    Modality::Acoustic,
];

impl fmt::Display for Modality {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            Modality::Acoustic => "ACOUSTIC",
            Modality::Rf => "RF",
            Modality::Cellular => "CELLULAR",
            Modality::Satellite => "SATELLITE",
        };
        f.write_str(name)
    }
}

/// Live quality estimate for one modality (from beacons / ACK stats /
/// signal meters). The vehicle's own link hardware fills this in per tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkQuality {
    /// Payload bit rate achievable right now, bits/second.
    pub bitrate_bps: u32,
    /// One-way latency estimate, milliseconds.
    pub latency_ms: u32,
    /// Frame loss probability, permille (0..1000).
    pub loss_permille: u16,
    /// Whether the link can carry traffic at all (modem powered, in range).
    pub up: bool,
}

impl LinkQuality {
    /// Down/unavailable state.
    pub const DOWN: LinkQuality = LinkQuality {
        bitrate_bps: 0,
        latency_ms: u32::MAX,
        loss_permille: 1000,
        up: false,
    };

    /// A composite score (higher = better) used for ranking. Deliberately
    /// simple and deterministic: bitrate weighted against loss and latency.
    pub fn score(&self) -> u64 {
        if !self.up {
            return 0;
        }
        let loss_factor = 1000 - (self.loss_permille.min(1000) as u64);
        // bits/s × delivery fraction ÷ (latency in 100 ms units). u64 math
        // with saturating floor so extreme latency can't invert the order.
        let lat_units = (self.latency_ms as u64 / 100).max(1);
        self.bitrate_bps as u64 * loss_factor / lat_units
    }
}

/// Selection policy knobs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SelectionPolicy {
    /// A candidate must beat the active link by this score fraction
    /// (`1.0` + margin) before a transition happens — hysteresis against
    /// flapping at quality boundaries.
    pub switch_margin: u32,
    /// Minimum loss permille to consider a link usable at all.
    pub max_loss_permille: u16,
}

impl SelectionPolicy {
    /// Default margin: candidate must be 50 % better.
    pub const DEFAULT: SelectionPolicy = SelectionPolicy {
        switch_margin: 500, // per-mille over the incumbent
        max_loss_permille: 800,
    };
}

/// The link manager: tracks four modality qualities and owns the active
/// selection. `Copy`-sized, no allocation.
#[derive(Debug, Clone, Copy)]
pub struct LinkManager {
    qualities: [LinkQuality; 4],
    /// Selection policy (hysteresis margin, loss ceiling).
    pub policy: SelectionPolicy,
    active: Modality,
}

impl LinkManager {
    /// Start with `initial` active; all links down until reported.
    pub fn new(initial: Modality) -> Self {
        Self {
            qualities: [LinkQuality::DOWN; 4],
            policy: SelectionPolicy::DEFAULT,
            active: initial,
        }
    }

    /// Active modality.
    pub fn active(&self) -> Modality {
        self.active
    }

    /// Quality of one modality.
    pub fn quality(&self, m: Modality) -> LinkQuality {
        self.qualities[m as usize]
    }

    /// Feed a quality report (per tick, per modem).
    pub fn report(&mut self, m: Modality, q: LinkQuality) {
        self.qualities[m as usize] = q;
    }

    /// Re-evaluate the active link. Returns `Some(new)` when a transition
    /// occurred.
    ///
    /// Rules (in order):
    /// 1. If the active link is down or unusable, switch to the best usable
    ///    link immediately (no margin required).
    /// 2. Otherwise, switch only if some candidate beats the incumbent's
    ///    score by the hysteresis margin.
    /// 3. If nothing is usable, stay on the (dead) active modality and let
    ///    [`crate::store::StoreForward`] buffer traffic.
    pub fn tick(&mut self) -> Option<Modality> {
        let active_q = self.qualities[self.active as usize];
        let active_ok = active_q.up && active_q.loss_permille <= self.policy.max_loss_permille;

        let mut best = Modality::Acoustic;
        let mut best_score = 0u64;
        for m in ALL {
            let q = self.qualities[m as usize];
            if q.up && q.loss_permille <= self.policy.max_loss_permille {
                let s = q.score();
                if s > best_score {
                    best_score = s;
                    best = m;
                }
            }
        }

        if best_score == 0 {
            return None; // nothing usable; hold and buffer
        }

        if !active_ok {
            if best != self.active {
                self.active = best;
                return Some(self.active);
            }
            return None;
        }

        // Hysteresis: candidate must beat the incumbent by the margin.
        let incumbent = active_q.score();
        let threshold = incumbent + incumbent * self.policy.switch_margin as u64 / 1000;
        if best != self.active && best_score >= threshold {
            self.active = best;
            return Some(self.active);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn q(bitrate: u32, loss: u16) -> LinkQuality {
        LinkQuality {
            bitrate_bps: bitrate,
            latency_ms: 200,
            loss_permille: loss,
            up: true,
        }
    }

    #[test]
    fn stays_on_adequate_link_despite_better_candidates() {
        let mut lm = LinkManager::new(Modality::Satellite);
        lm.report(Modality::Satellite, q(50_000, 10));
        lm.report(Modality::Rf, q(54_000, 10)); // only 8 % better
        assert!(lm.tick().is_none(), "8 % better does not cross the margin");
        assert_eq!(lm.active(), Modality::Satellite);
    }

    #[test]
    fn switches_when_margin_crossed() {
        let mut lm = LinkManager::new(Modality::Satellite);
        lm.report(Modality::Satellite, q(10_000, 10));
        lm.report(Modality::Rf, q(100_000, 10)); // 10× better
        assert_eq!(lm.tick(), Some(Modality::Rf));
    }

    #[test]
    fn fails_over_immediately_when_active_dies() {
        let mut lm = LinkManager::new(Modality::Cellular);
        lm.report(Modality::Cellular, q(1_000_000, 5));
        lm.report(Modality::Satellite, q(10_000, 300));
        assert!(lm.tick().is_none());
        // Cellular drops.
        lm.report(Modality::Cellular, LinkQuality::DOWN);
        assert_eq!(lm.tick(), Some(Modality::Satellite));
    }

    #[test]
    fn lossy_link_is_unusable_and_reported_as_hold() {
        let mut lm = LinkManager::new(Modality::Rf);
        // Everything down: hold, no transition.
        assert!(lm.tick().is_none());
        assert_eq!(lm.active(), Modality::Rf);
        // Acoustic comes up but beyond the loss limit: still hold.
        lm.report(Modality::Acoustic, q(1_200, 950));
        assert!(lm.tick().is_none());
        // Loss drops to workable: now take it.
        lm.report(Modality::Acoustic, q(1_200, 500));
        assert_eq!(lm.tick(), Some(Modality::Acoustic));
    }

    #[test]
    fn submerged_world_is_acoustic_only() {
        let mut lm = LinkManager::new(Modality::Acoustic);
        lm.report(Modality::Acoustic, q(1_200, 400));
        assert!(lm.tick().is_none(), "acoustic alone stays active");
        // Score sanity: acoustic's low bitrate must not out-rank RF at the
        // same loss.
        assert!(q(1_200, 400).score() < q(50_000_000, 400).score());
    }

    #[test]
    fn down_quality_scores_zero() {
        assert_eq!(LinkQuality::DOWN.score(), 0);
    }
}
