//! Biofouling compensation (spec §5.4): as barnacles grow on the hull, drag
//! increases; this monitor detects the drag signature and derates mission
//! speed so the vehicle finishes its battery budget instead of fighting the
//! hull at full power.
//!
//! Model: quadratic drag `F_d = ½·ρ·C_d·A·v²`, thrust balance at steady
//! speed. With commanded power fraction `u` (thruster output) the clean-hull
//! steady speed is `v_clean`; the *observed* steady speed under measured
//! water speed gives the fouling factor
//!
//! ```text
//! fouling = (v_expected / v_observed)²   (speed ratio, squared because
//!                                          thrust ∝ u, drag ∝ v²)
//! ```
//!
//! tracked by an EWMA over a configurable window (fouling grows over weeks;
//! currents and waves must not gate the derate). Mission speed derating
//! preserves the *endurance* budget: power ∝ v³·fouling, so running at
//! `v_derated = v_mission · fouling^(-1/3)` restores the clean-hull power.

/// Biofouling monitor configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FoulingConfig {
    /// EWMA time constant, seconds (default: 1 hour — fouling is weekly).
    pub tau_s: f64,
    /// Fouling factor at which derating begins (1.0 = clean hull).
    pub threshold: f64,
    /// Maximum derate ceiling: never command below this fraction of the
    /// mission speed (the vehicle still has to make way).
    pub min_speed_fraction: f64,
    /// Minimum steady speed for the estimate to be valid, m/s (below this
    /// the v² signal is noise).
    pub min_speed_mps: f64,
}

impl FoulingConfig {
    /// Default survey-AUV tuning.
    pub const fn default_config() -> Self {
        Self {
            tau_s: 3600.0,
            threshold: 1.15,
            min_speed_fraction: 0.6,
            min_speed_mps: 0.5,
        }
    }
}

/// Drag-signature monitor and speed derater (`Copy`, allocation-free).
#[derive(Debug, Clone, Copy)]
pub struct BiofoulingMonitor {
    cfg: FoulingConfig,
    fouling_ewma: f64,
    last_update_us: u64,
    primed: bool,
}

impl BiofoulingMonitor {
    /// Create a monitor for a clean hull.
    pub fn new(cfg: FoulingConfig) -> Self {
        Self {
            cfg,
            fouling_ewma: 1.0,
            last_update_us: 0,
            primed: false,
        }
    }

    /// Current fouling factor (1.0 = clean).
    pub fn fouling(&self) -> f64 {
        self.fouling_ewma
    }

    /// Feed one steady-speed observation: `expected_mps` is the clean-hull
    /// speed for the current thruster command, `observed_mps` the measured
    /// speed over ground (DVL/GPS). Call at 1 Hz or slower; the EWMA ignores
    /// transients below [`FoulingConfig::min_speed_mps`].
    pub fn observe(&mut self, expected_mps: f64, observed_mps: f64, now_us: u64) {
        if observed_mps < self.cfg.min_speed_mps || expected_mps <= 0.0 {
            return;
        }
        let instant = (expected_mps / observed_mps).powi(2);
        if !self.primed {
            self.fouling_ewma = instant;
            self.primed = true;
            self.last_update_us = now_us;
            return;
        }
        let dt_s = ((now_us - self.last_update_us) as f64 / 1.0e6).max(1.0);
        let alpha = (dt_s / self.cfg.tau_s).min(1.0);
        self.fouling_ewma += alpha * (instant - self.fouling_ewma);
        self.last_update_us = now_us;
    }

    /// Derated mission speed for the current fouling state.
    ///
    /// Returns `mission_mps` unchanged below the fouling threshold; above it,
    /// `v · (fouling/threshold)^(-1/3)` — the cube-root power law that holds
    /// propeller power at the clean-hull draw — floored at
    /// `min_speed_fraction · mission_mps`.
    pub fn derate_mission_speed(&self, mission_mps: f64) -> f64 {
        if self.fouling_ewma <= self.cfg.threshold {
            return mission_mps;
        }
        let power_ratio = self.fouling_ewma / self.cfg.threshold;
        let derated = mission_mps * power_ratio.powf(-1.0 / 3.0);
        derated.max(mission_mps * self.cfg.min_speed_fraction)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clean_hull_never_derates() {
        let mut m = BiofoulingMonitor::new(FoulingConfig::default_config());
        // Steady observation: measured speed matches expectation.
        for k in 0..100 {
            m.observe(1.5, 1.5, k * 1_000_000);
        }
        assert!((m.fouling() - 1.0).abs() < 1.0e-9);
        assert_eq!(m.derate_mission_speed(1.5), 1.5);
    }

    #[test]
    fn barnacle_growth_derates_speed() {
        let mut m = BiofoulingMonitor::new(FoulingConfig::default_config());
        // Hull fouls over weeks: observed speed settles at 80 % of the
        // clean expectation → fouling ≈ (1/0.8)² = 1.5625.
        for k in 0..100_000 {
            m.observe(1.5, 1.5 * 0.8, k * 1_000_000);
        }
        assert!(
            (m.fouling() - 1.5625).abs() < 0.01,
            "EWMA converges to the drag signature: {}",
            m.fouling()
        );
        let derated = m.derate_mission_speed(1.5);
        assert!(
            (1.5 * 0.6..1.5).contains(&derated),
            "derate in range: {derated}"
        );
        // Cube-root power law: (1.5625/1.15)^(-1/3) ≈ 0.922 → 1.383.
        let expected = 1.5 * (m.fouling() / 1.15).powf(-1.0 / 3.0);
        assert!((derated - expected).abs() < 1.0e-9);
    }

    #[test]
    fn low_speed_samples_are_ignored() {
        let mut m = BiofoulingMonitor::new(FoulingConfig::default_config());
        // Station keeping / descending: 0.2 m/s < min_speed — no update.
        m.observe(0.2, 0.05, 1_000_000);
        assert!((m.fouling() - 1.0).abs() < 1.0e-12, "still clean");
    }

    #[test]
    fn derate_floor_applies() {
        let mut m = BiofoulingMonitor::new(FoulingConfig::default_config());
        // Extreme fouling: observed speed 1/3 of expectation → factor 9.
        // Raw cube-root derate would be 0.76 m/s; the floor (0.6·1.5 =
        // 0.9 m/s) must bind.
        for k in 0..100_000 {
            m.observe(1.5, 0.5, k * 1_000_000);
        }
        let derated = m.derate_mission_speed(1.5);
        assert!(
            (derated - 1.5 * 0.6).abs() < 1.0e-9,
            "must floor at min_speed_fraction: {derated}"
        );
    }

    #[test]
    fn transients_do_not_gate_the_estimate() {
        let mut m = BiofoulingMonitor::new(FoulingConfig::default_config());
        // Fouled hull (steady 0.8 ratio) with a 10-minute current burst that
        // reads fast — the 1 h EWMA should barely move off the fouled value.
        for k in 0..100_000 {
            m.observe(1.5, 1.5 * 0.8, k * 1_000_000);
        }
        let fouled = m.fouling();
        for k in 0..600 {
            m.observe(1.5, 2.0, 100_000_000_000 + k * 1_000_000); // current boost
        }
        assert!(
            m.fouling() < fouled * 0.99,
            "burst must pull slightly, got {} vs {fouled}",
            m.fouling()
        );
        assert!(
            m.fouling() > 1.2,
            "…but not wipe out the fouling signature: {}",
            m.fouling()
        );
    }
}
