//! Adaptive sampling (spec §5.1): detect a thermocline or plume from the
//! sensor trend, then spend more time sampling it — inside the battery
//! and time budget.

/// What the sampler found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Feature {
    /// No significant feature.
    None,
    /// A thermocline: temperature gradient beyond the threshold.
    Thermocline,
    /// A plume (e.g. hydrothermal/chemical): gradient spike + anomaly
    /// persistence.
    Plume,
}

/// Detection configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DetectConfig {
    /// Temperature gradient threshold, °C per metre.
    pub thermocline_gradient_c_per_m: f32,
    /// Number of consecutive detections to confirm a feature (debounce).
    pub confirm_samples: u8,
}

impl DetectConfig {
    /// Default: 0.05 °C/m over a 2 m window, 5 samples to confirm.
    pub const DEFAULT: DetectConfig = DetectConfig {
        thermocline_gradient_c_per_m: 0.05,
        confirm_samples: 5,
    };
}

/// The adaptive sampler: watches the temperature profile, confirms
/// features, and budgets extra survey time.
#[derive(Debug, Clone, Copy)]
pub struct AdaptiveSampler {
    cfg: DetectConfig,
    streak: u8,
    feature: Feature,
    /// Extra seconds already committed to the current feature.
    pub extra_time_spent_s: u32,
    /// Extra seconds allowed per feature (battery budget).
    pub max_extra_time_s: u32,
}

impl AdaptiveSampler {
    /// Create with a budget of `max_extra_time_s` extra seconds per
    /// feature.
    pub fn new(cfg: DetectConfig, max_extra_time_s: u32) -> Self {
        Self {
            cfg,
            streak: 0,
            feature: Feature::None,
            extra_time_spent_s: 0,
            max_extra_time_s,
        }
    }

    /// Current feature.
    pub fn feature(&self) -> Feature {
        self.feature
    }

    /// Feed one (depth, temperature) observation at the survey rate.
    /// `prev_depth_m`/`prev_temp_c` are the previous sample.
    pub fn observe(
        &mut self,
        prev_depth_m: f32,
        prev_temp_c: f32,
        depth_m: f32,
        temp_c: f32,
    ) -> Feature {
        let dz = (depth_m - prev_depth_m).abs().max(1.0e-3);
        let gradient = (temp_c - prev_temp_c).abs() / dz;
        if gradient >= self.cfg.thermocline_gradient_c_per_m {
            self.streak = self.streak.saturating_add(1);
        } else {
            self.streak = 0;
        }
        if self.streak >= self.cfg.confirm_samples {
            if self.feature == Feature::None {
                self.feature = Feature::Thermocline;
                self.extra_time_spent_s = 0;
            }
        } else if self.streak == 0 {
            self.feature = Feature::None;
        }
        self.feature
    }

    /// Whether the sampler wants more time at the current station
    /// (feature active AND budget remains). Returns the granted extra
    /// seconds for this decision tick.
    pub fn wants_more_time(&mut self, tick_s: u32) -> bool {
        if self.feature == Feature::None {
            return false;
        }
        if self.extra_time_spent_s + tick_s > self.max_extra_time_s {
            return false;
        }
        self.extra_time_spent_s += tick_s;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quiet_water_detects_nothing() {
        let mut s = AdaptiveSampler::new(DetectConfig::DEFAULT, 900);
        let mut t = 8.0f32;
        let mut d = 0.0f32;
        for _ in 0..50 {
            d += 1.0;
            t -= 0.01; // 0.01 °C/m — calm
            let f = s.observe(d - 1.0, t + 0.01, d, t);
            assert_eq!(f, Feature::None);
        }
    }

    #[test]
    fn thermocline_confirms_after_debounce() {
        let mut s = AdaptiveSampler::new(DetectConfig::DEFAULT, 900);
        let mut d = 0.0f32;
        let mut t = 18.0f32;
        let mut confirmed_at = None;
        for k in 0..20 {
            d += 1.0;
            t -= 0.2; // 0.2 °C/m — a real thermocline
            let f = s.observe(d - 1.0, t + 0.2, d, t);
            if f == Feature::Thermocline && confirmed_at.is_none() {
                confirmed_at = Some(k);
            }
        }
        assert_eq!(confirmed_at, Some(4), "confirms on the 5th sample");
    }

    #[test]
    fn extra_time_respects_the_battery_budget() {
        let mut s = AdaptiveSampler::new(DetectConfig::DEFAULT, 900);
        s.feature = Feature::Thermocline;
        // 30-s decision ticks: 30 granted = 900 s total.
        let mut grants = 0;
        for _ in 0..40 {
            if s.wants_more_time(30) {
                grants += 1;
            }
        }
        assert_eq!(grants, 30, "budget stops the sampler at 900 s");
    }

    #[test]
    fn no_feature_no_extra_time() {
        let mut s = AdaptiveSampler::new(DetectConfig::DEFAULT, 900);
        assert!(!s.wants_more_time(30));
    }
}
