//! The COLREGS integration (spec §5.3): vessel detection → encounter
//! classification → correct maneuver execution, wired to the
//! deterministic engine in `tpt-t-marine-colregs`.

use tpt_t_marine_colregs::engine::{Action, ColregsEngine, MAX_TARGETS};
use tpt_t_marine_colregs::track::{OwnState, TargetTrack};
use tpt_t_marine_colregs::whistle::Maneuver;

/// The ASV's heading response to one engine action: a commanded heading
/// offset (degrees, + = starboard) and a speed scale.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Steering {
    /// Heading offset, degrees (starboard positive).
    pub heading_offset_deg: f32,
    /// Speed scale (0..1; SlowDown → reduced).
    pub speed_scale: f32,
    /// Whistle blasts to sound (0 = silent).
    pub blasts: u8,
}

/// The COLREGS supervisor for the ASV.
#[derive(Debug, Clone, Copy)]
pub struct ColregsSupervisor {
    engine: ColregsEngine,
}

impl Default for ColregsSupervisor {
    fn default() -> Self {
        Self::new()
    }
}

impl ColregsSupervisor {
    /// Idle supervisor.
    pub fn new() -> Self {
        Self {
            engine: ColregsEngine::new(),
        }
    }

    /// Evaluate one radar/AIS tick; returns the steering decision for the
    /// riskiest encounter this tick (None = navigate freely).
    pub fn update(
        &mut self,
        own: OwnState,
        targets: &[TargetTrack],
        now_us: u64,
        dt_s: f64,
    ) -> Option<Steering> {
        self.engine.set_own(own);
        let actions: [Option<Action>; MAX_TARGETS] = self.engine.update(targets, now_us, dt_s);
        // The most constraining action wins (Warn-beating encounters first;
        // among equals, the closest target — engine order is tracker order,
        // so pick the max-offset starboard preference).
        let mut best: Option<Steering> = None;
        for a in actions.into_iter().flatten() {
            let s = steering_for(a);
            let worse = match best {
                None => true,
                Some(ref b) => s.heading_offset_deg.abs() > b.heading_offset_deg.abs(),
            };
            if worse {
                best = Some(s);
            }
        }
        best
    }
}

/// Map one engine action to ASV steering.
pub fn steering_for(a: Action) -> Steering {
    match a.maneuver {
        Maneuver::AlterStarboard(deg) => Steering {
            heading_offset_deg: deg,
            speed_scale: 0.8,
            blasts: a.blasts,
        },
        Maneuver::AlterPort(deg) => Steering {
            heading_offset_deg: -deg,
            speed_scale: 0.8,
            blasts: a.blasts,
        },
        Maneuver::SlowDown => Steering {
            heading_offset_deg: 0.0,
            speed_scale: 0.3,
            blasts: a.blasts,
        },
        Maneuver::HoldCourse => Steering {
            heading_offset_deg: 0.0,
            speed_scale: 1.0,
            blasts: 0,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crossing_vessel_on_the_starboard_side_triggers_starboard_alteration() {
        let mut sup = ColregsSupervisor::new();
        let own = OwnState {
            course_deg: 0.0,
            speed_mps: 3.0,
        };
        let targets = [TargetTrack {
            id: 1,
            bearing_deg: 55.0,
            range_m: 1500.0,
            course_deg: 270.0,
            speed_mps: 2.0,
        }];
        let s = sup.update(own, &targets, 0, 1.0).expect("give-way");
        assert!(s.heading_offset_deg > 10.0, "alter to starboard: {s:?}");
        assert_eq!(s.blasts, 1, "one short blast");
    }

    #[test]
    fn open_sea_steers_freely() {
        let mut sup = ColregsSupervisor::new();
        let own = OwnState {
            course_deg: 45.0,
            speed_mps: 3.0,
        };
        let s = sup.update(own, &[], 0, 1.0);
        assert!(s.is_none());
    }
}
