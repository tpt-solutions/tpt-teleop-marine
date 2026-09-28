//! Leak detection via pressure-differential monitoring (spec §5.4).
//!
//! Every sealed compartment's pressure sensor is watched against a
//! *slow-drift baseline* (thermal day-cycles and depth-induced hull squeeze
//! are slow, minutes-to-hours phenomena). Water ingress is fast: the spec
//! target is a `0.1 PSI` rise triggering emergency buoyancy in `<100 ms`.
//!
//! [`LeakDetector::update`] is the whole story: feed one compartment sample
//! per tick (the standard ingress-monitoring rate is 100 Hz). Two triggers
//! fire, in escalation order:
//!
//! 1. **Rate trigger** ([`LeakCause::RapidRise`]) — instantaneous rise ≥
//!    [`LeakConfig::rate_limit_psi_per_s`] catches aggressive ingress before
//!    the full 0.1 PSI has accumulated. The default limit (2 PSI/s) sits an
//!    order of magnitude above thermal drift and below a typical seal-failure
//!    jet, so it never fires on environmental transients.
//! 2. **Absolute trigger** ([`LeakCause::PressureRise`]) — cumulative rise
//!    over the slow baseline ≥ [`LeakConfig::threshold_psi`] (spec value
//!    `0.1`). At 100 Hz this fires within 100 ms of a spec-rate ingress and
//!    arms the emergency-buoyancy reflex in `tpt-t-marine-safety`.
//!
//! Alarms latch (a flooding compartment does not get un-flooded by the
//! pressure dipping) and freeze the baseline so a slow leak cannot be
//! absorbed into it.

use crate::wire::LeakAlarm;

/// Why the detector raised an alarm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum LeakCause {
    /// Cumulative rise over baseline exceeded the threshold (spec trigger).
    PressureRise = 1,
    /// Instantaneous rise rate alone exceeded the ingress rate limit.
    RapidRise = 2,
}

/// Configuration for one compartment's leak detector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeakConfig {
    /// Rise over the slow baseline that constitutes a leak, PSI.
    /// Spec value: `0.1`.
    pub threshold_psi: f64,
    /// Instantaneous rise rate that constitutes aggressive ingress, PSI/s.
    /// Must sit far above thermal-drift rates and below real leak jets.
    pub rate_limit_psi_per_s: f64,
    /// Baseline tracking time constant, seconds. Must be much slower than
    /// leak dynamics: 30 s by default.
    pub baseline_tau_s: f64,
    /// Expected sample period, seconds (reciprocal of the sensor rate).
    pub sample_period_s: f64,
}

impl LeakConfig {
    /// Spec-default configuration for 100 Hz ingress monitoring.
    pub const fn spec() -> Self {
        Self {
            threshold_psi: 0.1,
            rate_limit_psi_per_s: 2.0,
            baseline_tau_s: 30.0,
            sample_period_s: 0.01,
        }
    }
}

/// Per-compartment leak detector state. One per monitored compartment;
/// `Copy`-sized, no allocation, no locks.
#[derive(Debug, Clone, Copy)]
pub struct LeakDetector {
    config: LeakConfig,
    compartment: u8,
    baseline_psi: f64,
    prev_psi: Option<f64>,
    alarm: Option<LeakAlarm>,
}

impl LeakDetector {
    /// Create a detector for compartment index `compartment`, primed with the
    /// current pressure as the baseline (call at surface check, before dive).
    pub fn new(compartment: u8, initial_psi: f64, config: LeakConfig) -> Self {
        Self {
            config,
            compartment,
            baseline_psi: initial_psi,
            prev_psi: Some(initial_psi),
            alarm: None,
        }
    }

    /// Whether an alarm is currently latched.
    pub fn alarm(&self) -> Option<LeakAlarm> {
        self.alarm
    }

    /// Feed one sample. Returns the latched alarm (new or ongoing) if any.
    ///
    /// While no alarm is active the slow baseline tracks the sensor with a
    /// first-order filter; once a leak is latched the baseline freezes (the
    /// compartment is flooding — "tracking" it would hide the leak) and
    /// [`LeakAlarm::delta_psi`] tracks the peak rise.
    pub fn update(&mut self, psi: f64, timestamp_us: u64) -> Option<LeakAlarm> {
        if let Some(alarm) = self.alarm {
            // Latched: keep reporting, tracking the peak pressure rise.
            self.alarm = Some(LeakAlarm {
                delta_psi: alarm.delta_psi.max(psi - self.baseline_psi),
                ..alarm
            });
            return self.alarm;
        }

        if let Some(prev) = self.prev_psi {
            let rate = (psi - prev) / self.config.sample_period_s;
            let rise = psi - self.baseline_psi;
            if rise >= self.config.threshold_psi {
                self.alarm = Some(LeakAlarm {
                    compartment: self.compartment,
                    delta_psi: rise,
                    rate_psi_per_s: rate,
                    cause: LeakCause::PressureRise as u16,
                    timestamp_us,
                });
                return self.alarm;
            }
            if rate >= self.config.rate_limit_psi_per_s && rise > 0.0 {
                self.alarm = Some(LeakAlarm {
                    compartment: self.compartment,
                    delta_psi: rise,
                    rate_psi_per_s: rate,
                    cause: LeakCause::RapidRise as u16,
                    timestamp_us,
                });
                return self.alarm;
            }
        }

        // Slow baseline tracking (first-order).
        let alpha = (self.config.sample_period_s / self.config.baseline_tau_s).min(1.0);
        self.baseline_psi += alpha * (psi - self.baseline_psi);
        self.prev_psi = Some(psi);
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::leak::LeakCause;

    #[test]
    fn slow_thermal_drift_does_not_alarm() {
        let mut d = LeakDetector::new(0, 14.5, LeakConfig::spec());
        // Depth squeeze over a 10-minute descent: 0.2 PSI total at 100 Hz
        // (3.3e-4 PSI/s — three orders below the rate limit).
        let total: usize = 10 * 60 * 100; // samples
        let step = 0.2 / total as f64;
        for k in 1..=total {
            let psi = 14.5 + step * k as f64;
            assert!(
                d.update(psi, (k as u64) * 10_000).is_none(),
                "slow drift must not alarm at sample {k}"
            );
        }
        assert!(d.alarm().is_none());
    }

    #[test]
    fn spec_ingress_alarms_within_100ms() {
        // Spec: 0.1 PSI increase → trigger in <100 ms. A seal-failure jet of
        // 1.5 PSI/s (0.015 PSI per 100 Hz sample) crosses 0.1 PSI on sample
        // 7 = 70 ms. The rate (1.5 PSI/s) stays under the 2 PSI/s rapid-rise
        // limit so the *absolute* spec trigger is exercised.
        let mut d = LeakDetector::new(1, 14.5, LeakConfig::spec());
        let mut alarm_at = None;
        for k in 1..=100u64 {
            let psi = 14.5 + 0.015 * k as f64;
            if let Some(alarm) = d.update(psi, k * 10_000) {
                alarm_at = Some((k, alarm));
                break;
            }
        }
        let (k, alarm) = alarm_at.expect("leak must alarm");
        assert_eq!(k, 7, "alarm at the 0.1 PSI crossing");
        assert!(k as f64 * 10.0 < 100.0, "detection latency <100 ms");
        assert_eq!(alarm.compartment, 1);
        assert_eq!(alarm.cause, LeakCause::PressureRise as u16);
        assert!((alarm.delta_psi - 0.105).abs() < 1.0e-3);
    }

    #[test]
    fn aggressive_ingress_alarms_early_via_rate() {
        // Fast spray: 0.03 PSI per sample = 3 PSI/s. The rate trigger fires
        // on the second sample, well before 0.1 PSI accumulates.
        let mut d = LeakDetector::new(2, 14.5, LeakConfig::spec());
        d.update(14.53, 10_000)
            .expect("first sample: rate 3 PSI/s fires");
        let alarm = d.alarm().expect("latched");
        assert_eq!(alarm.cause, LeakCause::RapidRise as u16);
        assert!(alarm.delta_psi < 0.1, "rate trigger beats the threshold");
        assert!((alarm.rate_psi_per_s - 3.0).abs() < 1e-9);
    }

    #[test]
    fn alarm_is_latched_and_reports_peak() {
        let mut d = LeakDetector::new(0, 14.5, LeakConfig::spec());
        d.update(14.53, 10_000).expect("first crossing");
        // Pressure keeps rising: alarm stays, delta tracks the peak.
        let a2 = d.update(14.80, 20_000).expect("latched");
        assert!(a2.delta_psi >= 0.3);
        // And a pressure drop does not unlatch it.
        assert!(d.update(14.60, 30_000).is_some());
    }

    #[test]
    fn baseline_freezes_on_leak_not_on_drift() {
        // A leak ramping at 1.5 PSI/s must alarm; the baseline (30 s tau)
        // cannot absorb it — rise over baseline ≈ raw rise within 100 ms.
        let mut d = LeakDetector::new(0, 14.5, LeakConfig::spec());
        let mut fired = None;
        for k in 1..=50u64 {
            let psi = 14.5 + 0.015 * k as f64;
            if d.update(psi, k * 10_000).is_some() {
                fired = Some(k);
                break;
            }
        }
        assert_eq!(fired, Some(7), "baseline must not mask a real leak");
    }
}
