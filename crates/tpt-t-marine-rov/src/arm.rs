//! Manipulator arm control: joint targets with rate limits — the surface
//! for the pilot's second joystick or VR gloves.

/// Number of arm joints (5-function grabber class: slew, shoulder, elbow,
/// wrist rotate, gripper).
pub const JOINTS: usize = 5;

/// Arm configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmConfig {
    /// Rate limit per joint, normalized units/s.
    pub rate_limit: f32,
}

impl Default for ArmConfig {
    fn default() -> Self {
        Self { rate_limit: 0.5 }
    }
}

/// The arm controller. Targets live in normalized joint space `[-1, 1]`.
#[derive(Debug, Clone, Copy)]
pub struct ArmController {
    cfg: ArmConfig,
    /// Current joint positions.
    pub pos: [f32; JOINTS],
    /// Commanded targets.
    target: [f32; JOINTS],
}

impl ArmController {
    /// Create stowed (all zeros).
    pub fn new(cfg: ArmConfig) -> Self {
        Self {
            cfg,
            pos: [0.0; JOINTS],
            target: [0.0; JOINTS],
        }
    }

    /// Set joint targets (clamped into range).
    pub fn set_target(&mut self, target: [f32; JOINTS]) {
        for (t, &v) in self.target.iter_mut().zip(target.iter()) {
            *t = v.clamp(-1.0, 1.0);
        }
    }

    /// Current targets.
    pub fn target(&self) -> [f32; JOINTS] {
        self.target
    }

    /// Advance one tick toward the targets at the rate limit. Returns the
    /// maximum remaining error (settled when ~0).
    pub fn tick(&mut self, dt_s: f32) -> f32 {
        let step = self.cfg.rate_limit * dt_s;
        let mut max_err = 0f32;
        for k in 0..JOINTS {
            let err = (self.target[k] - self.pos[k]).clamp(-step, step);
            self.pos[k] = (self.pos[k] + err).clamp(-1.0, 1.0);
            max_err = max_err.max((self.target[k] - self.pos[k]).abs());
        }
        max_err
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joints_move_at_the_rate_limit() {
        let mut a = ArmController::new(ArmConfig::default());
        a.set_target([1.0, 0.0, 0.0, 0.0, 0.0]);
        a.tick(0.1); // 0.5/s × 0.1 s = 0.05
        assert!((a.pos[0] - 0.05).abs() < 1e-6, "pos {}", a.pos[0]);
    }

    #[test]
    fn settles_on_target_and_stops() {
        let mut a = ArmController::new(ArmConfig::default());
        a.set_target([0.8; JOINTS]);
        for _ in 0..100 {
            a.tick(0.05);
        }
        assert!((a.pos[3] - 0.8).abs() < 1e-5);
        assert_eq!(a.tick(0.05), 0.0, "settled: zero remaining error");
    }

    #[test]
    fn targets_clamp_into_range() {
        let mut a = ArmController::new(ArmConfig::default());
        a.set_target([5.0, -9.0, 0.0, 0.0, 0.3]);
        assert_eq!(a.target()[0], 1.0);
        assert_eq!(a.target()[1], -1.0);
    }
}
