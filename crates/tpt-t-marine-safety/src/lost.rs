//! The lost-vehicle procedure (spec §5.4): if all communications are lost
//! for more than two hours, the vehicle autonomously ascends, drifts on
//! the surface, and transmits a GPS beacon via satellite until recovered.
//!
//! "Communications" spans every modality: the comms crate reports *any*
//! successful uplink; a single ACK resets the timer (a vehicle that can
//! squeak one frame an hour through the acoustic channel is not lost).

/// Comms-loss threshold that triggers the procedure.
pub const LOST_THRESHOLD_S: u64 = 2 * 60 * 60; // 2 hours

/// Beacon interval once surfaced, seconds.
pub const BEACON_INTERVAL_S: u64 = 60;

/// Procedure phases.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LostPhase {
    /// Comms healthy (or under the threshold).
    Nominal,
    /// Comms lost but under the threshold; logged only.
    Watch,
    /// Threshold crossed: ascend autonomously.
    Ascending,
    /// On the surface, drifting, GPS beacon active.
    SurfacedBeacon,
}

/// Lost-vehicle monitor (`Copy`).
#[derive(Debug, Clone, Copy)]
pub struct LostVehicleMonitor {
    /// Last successful uplink, mission microseconds.
    last_comms_us: u64,
    now_us: u64,
    /// Depth, metres (positive down; ≤ 0 = surfaced).
    depth_m: f64,
    phase: LostPhase,
    /// Next beacon time once surfaced, mission microseconds.
    next_beacon_us: u64,
}

impl LostVehicleMonitor {
    /// Create with mission time zero and healthy comms.
    pub fn new() -> Self {
        Self {
            last_comms_us: 0,
            now_us: 0,
            depth_m: 0.0,
            phase: LostPhase::Nominal,
            next_beacon_us: 0,
        }
    }

    /// Current phase.
    pub fn phase(&self) -> LostPhase {
        self.phase
    }

    /// Seconds since the last uplink.
    pub fn comms_age_s(&self) -> u64 {
        (self.now_us.saturating_sub(self.last_comms_us)) / 1_000_000
    }

    /// Record a successful uplink (any modality).
    pub fn on_uplink(&mut self, now_us: u64) {
        self.last_comms_us = now_us;
        // Any uplink while in Watch returns to Nominal; once the procedure
        // has physically started (Ascending/SurfacedBeacon) it runs to
        // completion — an AUV halfway through an emergency ascent does not
        // turn around because one acoustic frame arrived.
        if self.phase == LostPhase::Watch {
            self.phase = LostPhase::Nominal;
        }
    }

    /// Advance with current time and depth.
    pub fn update(&mut self, now_us: u64, depth_m: f64) -> LostPhase {
        self.now_us = now_us;
        self.depth_m = depth_m;
        match self.phase {
            LostPhase::Nominal | LostPhase::Watch => {
                self.phase = if self.comms_age_s() >= LOST_THRESHOLD_S {
                    LostPhase::Ascending
                } else if self.comms_age_s() >= LOST_THRESHOLD_S / 4 {
                    LostPhase::Watch
                } else {
                    LostPhase::Nominal
                };
            }
            LostPhase::Ascending => {
                if depth_m <= 0.0 {
                    self.phase = LostPhase::SurfacedBeacon;
                    self.next_beacon_us = now_us;
                }
            }
            LostPhase::SurfacedBeacon => {}
        }
        self.phase
    }

    /// Whether it is time to emit the GPS beacon (surfaced phase only).
    pub fn beacon_due(&mut self, now_us: u64) -> bool {
        if self.phase != LostPhase::SurfacedBeacon || now_us < self.next_beacon_us {
            return false;
        }
        self.next_beacon_us = now_us + BEACON_INTERVAL_S * 1_000_000;
        true
    }
}

impl Default for LostVehicleMonitor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HR_US: u64 = 3_600_000_000;

    #[test]
    fn triggers_after_two_hours_of_silence() {
        let mut m = LostVehicleMonitor::new();
        // 10 minutes: nominal. One hour: Watch (quarter of the threshold).
        assert_eq!(m.update(10 * 60 * 1_000_000, 50.0), LostPhase::Nominal);
        assert_eq!(m.update(HR_US, 50.0), LostPhase::Watch);
        // Two hours and a tick: the procedure starts.
        assert_eq!(m.update(2 * HR_US + 1_000_000, 50.0), LostPhase::Ascending);
    }

    #[test]
    fn watch_phase_recovers_on_single_uplink() {
        let mut m = LostVehicleMonitor::new();
        // 40 minutes of silence: Watch.
        assert_eq!(m.update(40 * 60 * 1_000_000, 50.0), LostPhase::Watch);
        // One acoustic frame gets through.
        m.on_uplink(41 * 60 * 1_000_000);
        assert_eq!(m.update(45 * 60 * 1_000_000, 50.0), LostPhase::Nominal);
    }

    #[test]
    fn procedure_runs_to_completion_once_started() {
        let mut m = LostVehicleMonitor::new();
        m.update(2 * HR_US + 1, 50.0);
        assert_eq!(m.phase(), LostPhase::Ascending);
        // An uplink during the ascent does NOT abort it.
        m.on_uplink(2 * HR_US + 60 * 1_000_000);
        assert_eq!(
            m.update(2 * HR_US + 2 * 60 * 1_000_000, 30.0),
            LostPhase::Ascending
        );
        // Surface: beacon phase.
        assert_eq!(
            m.update(2 * HR_US + 3 * 60 * 1_000_000, 0.0),
            LostPhase::SurfacedBeacon
        );
    }

    #[test]
    fn beacon_fires_every_minute_once_surfaced() {
        let mut m = LostVehicleMonitor::new();
        m.update(2 * HR_US + 1, 50.0); // threshold crossed: Ascending
        m.update(2 * HR_US + 2, 0.0); // breaks the surface: beacon phase
        assert_eq!(m.phase(), LostPhase::SurfacedBeacon);
        let t0 = 2 * HR_US + 2; // the surfacing instant (next_beacon anchor)
        assert!(m.beacon_due(t0));
        assert!(!m.beacon_due(t0 + 30 * 1_000_000), "not due for 30 s");
        assert!(m.beacon_due(t0 + 61 * 1_000_000));
    }

    #[test]
    fn shallow_silence_stays_nominal() {
        let mut m = LostVehicleMonitor::new();
        assert_eq!(m.update(25 * 60 * 1_000_000, 50.0), LostPhase::Nominal);
    }
}
