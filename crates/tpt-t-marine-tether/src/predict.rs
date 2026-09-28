//! The 5-second-ahead shape predictor and snag alarm (spec §4.4).
//!
//! The predictor clones the current chain, rolls the physics forward with
//! the *persisted* ROV velocity and current, and scores the minimum
//! clearance to the obstacle set along the way. A decreasing-clearance
//! trend below the safety radius raises the snag alert that routes to the
//! pilot.

use crate::chain::{Chain, DT_S};

/// Prediction horizon, seconds (spec: 5 s).
pub const HORIZON_S: f64 = 5.0;

/// Snag alarm levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum SnagLevel {
    /// Clear: minimum predicted clearance above the caution radius.
    Clear = 0,
    /// Caution: predicted clearance under the caution radius within the
    /// horizon.
    Caution = 1,
    /// Warn: clearance under the safety radius — the ROV must not continue
    /// this motion.
    Warn = 2,
}

/// Obstacle spheres (subsea structures), NED, metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obstacle {
    /// Centre, NED metres.
    pub centre_m: [f64; 3],
    /// Radius, metres.
    pub radius_m: f64,
}

/// Snag monitor configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SnagConfig {
    /// Caution clearance, metres.
    pub caution_m: f64,
    /// Safety radius, metres.
    pub safety_m: f64,
}

impl SnagConfig {
    /// Two-metre caution, one-metre hard radius.
    pub const DEFAULT: SnagConfig = SnagConfig {
        caution_m: 2.0,
        safety_m: 1.0,
    };
}

/// The predictor output.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(missing_docs)]
pub struct Prediction {
    /// Alarm level.
    pub level: SnagLevel,
    /// Minimum clearance predicted within the horizon, metres (to the
    /// nearest obstacle surface).
    pub min_clearance_m: f64,
    /// Seconds into the horizon where the minimum occurs.
    pub at_s: f64,
}

/// The snag predictor: `persist_steps` decides how far ahead the ROV's
/// velocity is assumed constant (default: the full horizon).
#[derive(Debug, Clone, Copy)]
pub struct SnagPredictor {
    /// Alarm thresholds.
    pub cfg: SnagConfig,
}

impl SnagPredictor {
    /// Create with a config.
    pub const fn new(cfg: SnagConfig) -> Self {
        Self { cfg }
    }

    /// Roll the chain forward (cloned internally) for [`HORIZON_S`] and
    /// score clearance. `rov_vel` is the ROV's current velocity (NED, m/s)
    /// — the "if the ROV keeps doing this" extrapolation.
    pub fn predict(
        &self,
        chain: &Chain,
        rov_vel: [f64; 3],
        current: [f64; 3],
        obstacles: &[Obstacle],
    ) -> Prediction {
        let mut sim = *chain;
        let steps = (HORIZON_S / DT_S) as usize;
        let rov = chain.pos[crate::chain::NODES - 1];
        let mut best = Prediction {
            level: SnagLevel::Clear,
            min_clearance_m: f64::MAX,
            at_s: 0.0,
        };
        for k in 1..=steps {
            let t = k as f64 * DT_S;
            let rov_next = [
                rov[0] + rov_vel[0] * t,
                rov[1] + rov_vel[1] * t,
                rov[2] + rov_vel[2] * t,
            ];
            sim.step(rov_next, current, DT_S);
            // Minimum clearance across the chain and obstacles.
            let mut min_clear = f64::MAX;
            for node in &sim.pos {
                for ob in obstacles {
                    let dx = node[0] - ob.centre_m[0];
                    let dy = node[1] - ob.centre_m[1];
                    let dz = node[2] - ob.centre_m[2];
                    let d = (dx * dx + dy * dy + dz * dz).sqrt() - ob.radius_m;
                    min_clear = min_clear.min(d);
                }
            }
            if obstacles.is_empty() {
                min_clear = f64::MAX;
            }
            if min_clear < best.min_clearance_m {
                best.min_clearance_m = min_clear;
                best.at_s = t;
            }
        }
        best.level = if best.min_clearance_m < self.cfg.safety_m {
            SnagLevel::Warn
        } else if best.min_clearance_m < self.cfg.caution_m {
            SnagLevel::Caution
        } else {
            SnagLevel::Clear
        };
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chain::TetherSpec;

    fn straight_chain() -> Chain {
        Chain::new(TetherSpec::typical(), [0.0, 0.0, 0.0], [0.0, 0.0, 100.0])
    }

    #[test]
    fn clear_water_stays_clear() {
        let p = SnagPredictor::new(SnagConfig::DEFAULT);
        let pred = p.predict(&straight_chain(), [0.3, 0.0, 0.0], [0.0, 0.0, 0.0], &[]);
        assert_eq!(pred.level, SnagLevel::Clear);
    }

    #[test]
    fn motion_toward_a_structure_raises_warn() {
        let p = SnagPredictor::new(SnagConfig::DEFAULT);
        // ROV heading straight for a wellhead 2 m ahead of its tow point.
        let ob = [Obstacle {
            centre_m: [0.0, 0.0, 100.0],
            radius_m: 3.0,
        }];
        let pred = p.predict(&straight_chain(), [0.0, 0.0, 0.05], [0.0, 0.0, 0.0], &ob);
        assert_eq!(pred.level, SnagLevel::Warn, "predicted {pred:?}");
        assert!(pred.min_clearance_m < 1.0);
        // The minimum occurs partway into the horizon, not at t=0.
        assert!(pred.at_s > 0.0);
    }

    #[test]
    fn lateral_motion_slides_clear() {
        let p = SnagPredictor::new(SnagConfig::DEFAULT);
        // Wellhead 10 m ahead of the tow point.
        let ob = [Obstacle {
            centre_m: [10.0, 0.0, 100.0],
            radius_m: 3.0,
        }];
        // Driving straight at it at 2 m/s: inside the horizon the tow
        // point reaches the structure → Warn.
        let hit = p.predict(&straight_chain(), [2.0, 0.0, 0.0], [0.0, 0.0, 0.0], &ob);
        assert_eq!(hit.level, SnagLevel::Warn, "driving in: {hit:?}");
        // Sliding past (perpendicular) at the same speed: stays clear.
        let slide = p.predict(&straight_chain(), [0.0, 2.0, 0.0], [0.0, 0.0, 0.0], &ob);
        assert_eq!(slide.level, SnagLevel::Clear, "sliding past: {slide:?}");
    }
}
