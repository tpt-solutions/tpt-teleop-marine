//! Sleep-wake power management (spec §5.1): multi-month deployments sleep
//! ~99 % of the time, waking for scheduled profiles or an acoustic
//! page-up from the surface buoy.

/// Power states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerState {
    /// Full sleep: only the acoustic wake detector and RTC are alive.
    Asleep,
    /// Woken: sensors and controller powered.
    Awake,
}

/// Wake configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SleepSchedule {
    /// Seconds awake per profile.
    pub awake_s: u32,
    /// Seconds asleep between profiles. The implied duty cycle
    /// (awake/(awake+sleep)) should be ≤ ~1 % for the multi-month claim.
    pub sleep_s: u32,
}

impl SleepSchedule {
    /// The spec's 99 %-sleep schedule: 10 minutes awake per ~16.6 h.
    pub const NINETY_NINE_PERCENT: SleepSchedule = SleepSchedule {
        awake_s: 600,
        sleep_s: 59_400,
    };

    /// Implied duty fraction, awake/(awake+sleep).
    pub fn duty(&self) -> f32 {
        self.awake_s as f32 / (self.awake_s + self.sleep_s) as f32
    }
}

/// The power manager.
#[derive(Debug, Clone, Copy)]
pub struct PowerManager {
    schedule: SleepSchedule,
    state: PowerState,
    /// Seconds remaining in the current phase.
    remaining_s: u32,
    /// Profiles completed.
    pub profiles: u64,
}

impl PowerManager {
    /// Create, starting awake for the first profile.
    pub fn new(schedule: SleepSchedule) -> Self {
        Self {
            schedule,
            state: PowerState::Awake,
            remaining_s: schedule.awake_s,
            profiles: 0,
        }
    }

    /// Current state.
    pub fn state(&self) -> PowerState {
        self.state
    }

    /// Seconds until the next state transition.
    pub fn remaining_s(&self) -> u32 {
        self.remaining_s
    }

    /// Advance `dt_s`; transitions Asleep→Awake when the schedule fires
    /// (a profile begins, `profiles` increments).
    pub fn tick(&mut self, dt_s: u32) -> PowerState {
        self.remaining_s = self.remaining_s.saturating_sub(dt_s);
        if self.remaining_s == 0 {
            match self.state {
                PowerState::Awake => {
                    self.state = PowerState::Asleep;
                    self.remaining_s = self.schedule.sleep_s;
                }
                PowerState::Asleep => {
                    self.state = PowerState::Awake;
                    self.remaining_s = self.schedule.awake_s;
                    self.profiles += 1;
                }
            }
        }
        self.state
    }

    /// An acoustic page-up from the buoy: wake immediately (only from
    /// sleep), restarting the awake window.
    pub fn page_up(&mut self) -> PowerState {
        if self.state == PowerState::Asleep {
            self.state = PowerState::Awake;
            self.remaining_s = self.schedule.awake_s;
        } else {
            self.remaining_s = self.schedule.awake_s;
        }
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duty_cycle_is_one_percent() {
        let s = SleepSchedule::NINETY_NINE_PERCENT;
        assert!(s.duty() <= 0.0101, "duty {}", s.duty());
    }

    #[test]
    fn scheduled_profiles_cycle() {
        let mut pm = PowerManager::new(SleepSchedule {
            awake_s: 60,
            sleep_s: 5900,
        });
        // Burn the awake window.
        for _ in 0..6 {
            pm.tick(10);
        }
        assert_eq!(pm.state(), PowerState::Asleep);
        // Nothing wakes it early by itself.
        pm.tick(1);
        assert_eq!(pm.state(), PowerState::Asleep);
        // Burn the sleep window: wakes and counts a profile.
        for _ in 0..590 {
            pm.tick(10);
        }
        assert_eq!(pm.state(), PowerState::Awake);
        assert_eq!(pm.profiles, 1);
    }

    #[test]
    fn acoustic_page_up_wakes_immediately() {
        let mut pm = PowerManager::new(SleepSchedule {
            awake_s: 60,
            sleep_s: 5900,
        });
        pm.tick(61); // into sleep
        assert_eq!(pm.state(), PowerState::Asleep);
        pm.page_up();
        assert_eq!(pm.state(), PowerState::Awake);
        assert_eq!(pm.remaining_s(), 60);
    }

    #[test]
    fn multi_month_battery_projection() {
        // 99 % duty: a 30-day mission is awake ~7.2 h in total. The spec's
        // 100×-real-time sim target (30 days in 7 h) rhymes with this.
        let s = SleepSchedule::NINETY_NINE_PERCENT;
        let total_s = 30.0 * 24.0 * 3600.0;
        let awake_s = total_s * s.duty() as f64;
        assert!((awake_s - 7.2 * 3600.0).abs() < 60.0);
    }
}
