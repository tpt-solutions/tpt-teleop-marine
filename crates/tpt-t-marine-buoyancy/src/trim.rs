//! Fore/aft trim-pump control: hold pitch by shifting water between the end
//! tanks with rate-limited pumps.
//!
//! Moving `Δm` kg of water a lever-arm `L` apart produces a pitch moment
//! `M = Δm·g·L`. The controller holds a pitch setpoint (normally level)
//! against payload shifts — a manipulator arm reaching forward or a sample
//! tub sliding aft — using a proportional law with pitch-rate damping and
//! the pump's flow limit as the actuator constraint.
//!
//! Sign conventions (all documented in place): pitch positive = nose up;
//! a **positive transfer rate pumps water forward** (fore tank gains), which
//! makes the vehicle fore-heavy and produces a nose-up moment.

/// Trim tank configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrimTankConfig {
    /// Lever arm between the fore and aft tank centroids, m.
    pub lever_arm_m: f64,
    /// Water mass in the forward tank, kg.
    pub fore_kg: f64,
    /// Water mass in the aft tank, kg.
    pub aft_kg: f64,
    /// Total movable water across both tanks, kg.
    pub capacity_kg: f64,
    /// Max transfer flow between tanks, kg/s at full rate.
    pub flow_kg_per_s: f64,
}

/// One trim-pump command: a normalized transfer rate with diagnostics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrimCmd {
    /// Normalized transfer rate, `[-1.0, 1.0]`: **positive pumps water
    /// forward** (fore tank gains → nose-up moment), negative pumps aft.
    pub rate: f64,
    /// Fore-tank water mass at command time, kg.
    pub fore_kg: f64,
    /// Pitch setpoint being chased, rad.
    pub target_pitch_rad: f64,
}

/// Trim controller state (`Copy`, no allocation).
#[derive(Debug, Clone, Copy)]
pub struct TrimController {
    config: TrimTankConfig,
    /// Proportional gain, pump fraction per radian of pitch error.
    kp: f64,
    /// Pitch-rate damping fraction, per rad/s.
    kd: f64,
}

impl TrimController {
    /// Create a level-hold controller for the given tank set.
    pub fn new(config: TrimTankConfig) -> Self {
        Self {
            config,
            kp: 0.5,
            kd: 0.05,
        }
    }

    /// Pitch moment currently available from a full-rate transfer, N·m
    /// (positive = nose-up authority).
    pub fn max_moment_nm(&self) -> f64 {
        self.config.capacity_kg * crate::physics::G_M_S2 * self.config.lever_arm_m
    }

    /// The pitch moment produced by the current fore/aft imbalance, N·m
    /// (positive = nose-up).
    pub fn moment_nm(&self) -> f64 {
        (self.config.fore_kg - self.config.aft_kg)
            * crate::physics::G_M_S2
            * self.config.lever_arm_m
    }

    /// Advance one control tick: emit the rate-limited transfer chasing the
    /// pitch setpoint, then integrate the water movement.
    pub fn update(
        &mut self,
        pitch_rad: f64,
        pitch_rate_rad_s: f64,
        dt_s: f64,
        target_pitch_rad: f64,
    ) -> TrimCmd {
        let error = target_pitch_rad - pitch_rad;
        // Nose-down error (target above current) → positive rate → water
        // forward → nose-up moment. Damping opposes an ongoing pitch rate.
        let rate = (self.kp * error - self.kd * pitch_rate_rad_s).clamp(-1.0, 1.0);
        let cmd = TrimCmd {
            rate,
            fore_kg: self.config.fore_kg,
            target_pitch_rad,
        };
        // Transfer moves water fore↔aft; keep the pair within capacity.
        let transfer = rate * self.config.flow_kg_per_s * dt_s;
        self.config.fore_kg = (self.config.fore_kg + transfer).clamp(0.0, self.config.capacity_kg);
        self.config.aft_kg = self.config.capacity_kg - self.config.fore_kg;
        cmd
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tanks() -> TrimTankConfig {
        TrimTankConfig {
            lever_arm_m: 1.2,
            fore_kg: 5.0,
            aft_kg: 5.0,
            capacity_kg: 10.0,
            flow_kg_per_s: 0.5,
        }
    }

    #[test]
    fn nose_down_pumps_forward() {
        // Vehicle pitched −5° (nose down): trim must pump water forward to
        // lift the nose (positive rate, positive moment).
        let mut t = TrimController::new(tanks());
        let pitch = -5.0f64.to_radians();
        let cmd = t.update(pitch, 0.0, 0.1, 0.0);
        assert!(cmd.rate > 0.0, "nose-down needs water forward");
        assert!(t.moment_nm() > 0.0, "fore-heavy = nose-up moment");
    }

    #[test]
    fn converges_to_level_from_nose_down() {
        let mut t = TrimController::new(tanks());
        let mut pitch = -8.0f64.to_radians();
        let mut rate = 0.0f64;
        for _ in 0..2000 {
            t.update(pitch, rate, 0.05, 0.0);
            // Crude 1-DOF plant: moment / inertia (200 kg·m²).
            let accel = t.moment_nm() / 200.0;
            rate += accel * 0.05;
            pitch += rate * 0.05;
            if pitch.abs() < 0.1f64.to_radians() {
                break;
            }
        }
        assert!(
            pitch.abs() < 0.1f64.to_radians(),
            "trim must converge, ended at {}°",
            pitch.to_degrees()
        );
    }

    #[test]
    fn rate_saturates_and_tanks_stay_in_bounds() {
        let mut t = TrimController::new(tanks());
        // Absurd nose-up pitch: full saturation pumping aft (negative).
        let cmd = t.update(3.0, 0.0, 0.1, 0.0);
        assert_eq!(cmd.rate, -1.0);
        for _ in 0..1000 {
            t.update(3.0, 0.0, 0.1, 0.0);
        }
        assert_eq!(t.config.fore_kg, 0.0, "fore tank empties but not below");
        assert_eq!(t.config.aft_kg, t.config.capacity_kg);
        // And the other extreme fills the fore tank to capacity.
        for _ in 0..1000 {
            t.update(-3.0, 0.0, 0.1, 0.0);
        }
        assert_eq!(t.config.fore_kg, t.config.capacity_kg);
    }

    #[test]
    fn damping_opposes_pitch_rate() {
        let mut a = TrimController::new(tanks());
        let mut b = TrimController::new(tanks());
        // Same nose-down error in both; `b`'s nose is already rising fast, so
        // damping must soften its drive.
        let ra = a.update(-0.05, 0.0, 0.1, 0.0);
        let rb = b.update(-0.05, 0.5, 0.1, 0.0);
        assert!(rb.rate < ra.rate, "damping must soften the drive");
    }

    #[test]
    fn moment_authority_matches_geometry() {
        let t = TrimController::new(tanks());
        let expected = 10.0 * crate::physics::G_M_S2 * 1.2;
        assert!((t.max_moment_nm() - expected).abs() < 1.0e-9);
    }
}
