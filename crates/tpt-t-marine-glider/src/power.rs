//! The ultra-low-power profile: navigation fixes on a starvation budget.
//!
//! A trans-oceanic glider surfaces for a GPS fix a few times a day and
//! runs dead reckoning between; everything else sleeps. This module
//! tracks the fix budget and dead-reckoning drift allowance.

/// Fix policy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixPolicy {
    /// Nominal hours between surfacing fixes.
    pub fix_interval_hr: f64,
    /// Maximum allowed dead-reckoning error before an *unscheduled* fix is
    /// forced, metres.
    pub max_dr_error_m: f32,
}

impl FixPolicy {
    /// Ocean-crossing default: 2 fixes/day, 5 km DR allowance.
    pub const OCEANIC: FixPolicy = FixPolicy {
        fix_interval_hr: 12.0,
        max_dr_error_m: 5000.0,
    };
}

/// The fix scheduler.
#[derive(Debug, Clone, Copy)]
pub struct FixScheduler {
    policy: FixPolicy,
    hours_since_fix: f64,
    /// Current DR error estimate, metres (fed by nav).
    pub dr_error_m: f32,
    /// Completed surface fixes.
    pub fixes_taken: u32,
}

impl FixScheduler {
    /// Create.
    pub fn new(policy: FixPolicy) -> Self {
        Self {
            policy,
            hours_since_fix: 0.0,
            dr_error_m: 0.0,
            fixes_taken: 0,
        }
    }

    /// Advance an hour fraction of underwater time.
    pub fn tick(&mut self, dt_hr: f64, dr_error_m: f32) -> bool {
        self.hours_since_fix += dt_hr;
        self.dr_error_m = dr_error_m;
        self.due()
    }

    /// Whether a surface fix is due (schedule or error budget).
    pub fn due(&self) -> bool {
        self.hours_since_fix >= self.policy.fix_interval_hr
            || self.dr_error_m >= self.policy.max_dr_error_m
    }

    /// Record a completed fix.
    pub fn on_fix(&mut self) {
        self.hours_since_fix = 0.0;
        self.dr_error_m = 0.0;
        self.fixes_taken += 1;
    }

    /// Expected fixes over a mission of `days` (planning aid).
    pub fn projected_fixes(&self, days: f64) -> u32 {
        (days * 24.0 / self.policy.fix_interval_hr).ceil() as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fix_every_twelve_hours() {
        let mut s = FixScheduler::new(FixPolicy::OCEANIC);
        assert!(!s.tick(6.0, 100.0));
        assert!(s.tick(6.1, 200.0), "past the 12 h interval");
        s.on_fix();
        assert!(!s.due(), "fresh fix");
        assert_eq!(s.fixes_taken, 1);
    }

    #[test]
    fn error_budget_forces_an_early_fix() {
        let mut s = FixScheduler::new(FixPolicy::OCEANIC);
        assert!(s.tick(1.0, 5100.0), "5.1 km of DR error forces a fix");
    }

    #[test]
    fn thirty_day_crossing_projection() {
        let s = FixScheduler::new(FixPolicy::OCEANIC);
        assert_eq!(s.projected_fixes(30.0), 60, "2 fixes/day for 30 days");
    }
}
