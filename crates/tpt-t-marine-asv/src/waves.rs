//! Wave compensation (spec §5.3): active heave/pitch/roll compensation
//! for personnel transfer and crane operations.

use tpt_t_marine_core::wire::ThrusterCmd6;

/// Compensation configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaveCompConfig {
    /// Heave gain, normalized thrust per m/s of heave velocity.
    pub heave_gain: f32,
    /// Roll gain, normalized moment per rad/s of roll rate.
    pub roll_gain: f32,
    /// Pitch gain, normalized moment per rad/s of pitch rate.
    pub pitch_gain: f32,
}

impl WaveCompConfig {
    /// Workboat-class tuning.
    pub const fn workboat() -> Self {
        Self {
            heave_gain: 0.6,
            roll_gain: 0.4,
            pitch_gain: 0.4,
        }
    }
}

/// The wave compensator: rate-feedback damping (the classic ride-control
/// law) — it damps heave velocity and roll/pitch rates using the
/// thrusters, so a transfer basket or crane load stays still-ish.
#[derive(Debug, Clone, Copy)]
pub struct WaveCompensator {
    cfg: WaveCompConfig,
    /// Whether compensation is engaged.
    pub engaged: bool,
}

impl WaveCompensator {
    /// Create (disengaged).
    pub fn new(cfg: WaveCompConfig) -> Self {
        Self {
            cfg,
            engaged: false,
        }
    }

    /// Engage/disengage (crane ops toggle this).
    pub fn set_engaged(&mut self, on: bool) {
        self.engaged = on;
    }

    /// One control tick: `heave_rate` m/s (down positive), `roll_rate` and
    /// `pitch_rate` rad/s. Returns the damping thrust superimposed on the
    /// station-keeping command.
    pub fn update(
        &self,
        heave_rate_mps: f32,
        roll_rate_rad_s: f32,
        pitch_rate_rad_s: f32,
    ) -> ThrusterCmd6 {
        if !self.engaged {
            return ThrusterCmd6::default();
        }
        ThrusterCmd6 {
            // Damping opposes the motion: descending heave → upward thrust.
            heave: (-self.cfg.heave_gain * heave_rate_mps).clamp(-1.0, 1.0),
            roll: (-self.cfg.roll_gain * roll_rate_rad_s).clamp(-1.0, 1.0),
            pitch: (-self.cfg.pitch_gain * pitch_rate_rad_s).clamp(-1.0, 1.0),
            surge: 0.0,
            sway: 0.0,
            yaw: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn damping_opposes_the_motion() {
        let mut w = WaveCompensator::new(WaveCompConfig::workboat());
        w.set_engaged(true);
        // Descending at 1 m/s, rolling starboard-down at 0.2 rad/s.
        let cmd = w.update(1.0, 0.2, 0.0);
        assert!(cmd.heave < -0.3, "thrust up against descent: {}", cmd.heave);
        assert!(cmd.roll < 0.0, "counter-roll: {}", cmd.roll);
    }

    #[test]
    fn disengaged_is_inert() {
        let mut w = WaveCompensator::new(WaveCompConfig::workboat());
        let cmd = w.update(2.0, 0.5, 0.5);
        assert!(cmd.is_zero());
        w.set_engaged(true);
        assert!(!w.update(2.0, 0.5, 0.5).is_zero());
    }

    #[test]
    fn extreme_rates_saturate() {
        let mut w = WaveCompensator::new(WaveCompConfig::workboat());
        w.set_engaged(true);
        let cmd = w.update(50.0, 5.0, 5.0);
        assert!((cmd.heave - -1.0).abs() < 1e-6);
    }
}
