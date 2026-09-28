//! The COLREGS engine: one evaluation tick over all tracked vessels,
//! producing maneuvers, whistle signals, and log entries — under 50 µs per
//! vessel (spec §4.5).

use crate::encounter::{Encounter, EncounterFsm};
use crate::log::{Decision, DecisionLog};
use crate::track::{Cbdr, OwnState, TargetTrack};
use crate::whistle::maneuver_for;

/// Tracked vessels the engine evaluates.
pub const MAX_TARGETS: usize = 8;

/// One engine output per target.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Action {
    /// Target id.
    pub target_id: u16,
    /// Maneuver the vehicle executes.
    pub maneuver: crate::whistle::Maneuver,
    /// Whistle blasts (0 = silent).
    pub blasts: u8,
}

/// The engine: fixed FSM slots, one decision log. `Copy`-sized.
#[derive(Debug, Clone, Copy)]
pub struct ColregsEngine {
    own: OwnState,
    slots: [EncounterFsm; MAX_TARGETS],
    prev_tracks: [Option<Cbdr>; MAX_TARGETS],
    /// The auditable decision log.
    pub log: DecisionLog,
}

impl ColregsEngine {
    /// Idle engine.
    pub fn new() -> Self {
        Self {
            own: OwnState {
                course_deg: 0.0,
                speed_mps: 0.0,
            },
            slots: core::array::from_fn(|i| EncounterFsm::idle(i as u16)),
            prev_tracks: [None; MAX_TARGETS],
            log: DecisionLog::new(),
        }
    }

    /// Set own-vehicle state for this tick.
    pub fn set_own(&mut self, own: OwnState) {
        self.own = own;
    }

    /// Evaluate one tick with `targets` (≤ [`MAX_TARGETS`]). Returns the
    /// actions for risky encounters (Clear entries produce no action).
    pub fn update(
        &mut self,
        targets: &[TargetTrack],
        now_us: u64,
        dt_s: f64,
    ) -> [Option<Action>; MAX_TARGETS] {
        assert!(targets.len() <= MAX_TARGETS, "too many targets");
        let mut actions = [None; MAX_TARGETS];
        for (k, t) in targets.iter().enumerate() {
            let prev = self.prev_tracks[k];
            let class = self.slots[k].update(&self.own, t, prev, dt_s);
            self.prev_tracks[k] = Some(Cbdr {
                prev_bearing_deg: t.bearing_deg,
                prev_range_m: t.range_m,
            });
            let (maneuver, citation) = maneuver_for(class, self.slots[k].depth);
            let blasts = maneuver.whistle().map(|b| b.count()).unwrap_or(0);
            // Log every *risk-bearing* evaluation: the audit trail is for
            // the encounters that mattered, not the clear sea.
            if class != Encounter::Clear || self.slots[k].risk {
                self.log.push(Decision {
                    timestamp_us: now_us,
                    target_id: t.id,
                    class: class as u8,
                    maneuver: maneuver.code(),
                    blasts,
                    range_m: t.range_m as f32,
                    bearing_deg: t.bearing_deg as f32,
                });
                let _ = citation; // carried by maneuver_for's table; see whistle.rs
            }
            if class != Encounter::Clear {
                actions[k] = Some(Action {
                    target_id: t.id,
                    maneuver,
                    blasts,
                });
            }
        }
        actions
    }
}

impl Default for ColregsEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::track::{Quadrant, quadrant_of};
    use std::time::Instant;

    fn own() -> OwnState {
        OwnState {
            course_deg: 0.0,
            speed_mps: 3.0,
        }
    }

    fn crossing_starboard(bearing: f64, range: f64) -> TargetTrack {
        TargetTrack {
            id: 42,
            bearing_deg: bearing,
            range_m: range,
            course_deg: 270.0,
            speed_mps: 2.0,
        }
    }

    #[test]
    fn engine_emits_starboard_alteration_with_whistle() {
        let mut e = ColregsEngine::new();
        e.set_own(own());
        let targets = [crossing_starboard(60.0, 1500.0)];
        let actions = e.update(&targets, 1_000_000, 1.0);
        let a = actions[0].expect("give-way action");
        assert!(matches!(
            a.maneuver,
            crate::whistle::Maneuver::AlterStarboard(_)
        ));
        assert_eq!(a.blasts, 1);
        assert_eq!(e.log.len(), 1, "risk encounter is logged");
        let d = e.log.latest_for(42).unwrap();
        assert_eq!(d.blasts, 1);
        assert!((d.range_m - 1500.0).abs() < 0.1);
    }

    #[test]
    fn clear_sea_produces_no_actions_and_no_log() {
        let mut e = ColregsEngine::new();
        e.set_own(own());
        // Target heading away on the port quarter, slower than us: no
        // risk geometry (and nobody is overtaking anybody).
        let targets = [TargetTrack {
            id: 5,
            bearing_deg: 150.0,
            range_m: 2000.0,
            course_deg: 150.0,
            speed_mps: 1.0,
        }];
        let actions = e.update(&targets, 0, 1.0);
        assert!(actions[0].is_none());
        assert!(e.log.is_empty(), "clear sea is not logged");
    }

    #[test]
    fn budget_per_vessel() {
        // Spec: <50 µs per detected vessel per tick. Measure 8 vessels ×
        // 1_000 ticks; this is an informational absolute number (CI hosts
        // differ) with a generous assert — the arithmetic is ~100 ns.
        let mut e = ColregsEngine::new();
        let targets: Vec<TargetTrack> = (0..MAX_TARGETS)
            .map(|i| crossing_starboard(30.0 + i as f64 * 15.0, 2000.0 - i as f64 * 100.0))
            .collect();
        let start = Instant::now();
        for k in 0..1_000u64 {
            e.update(&targets, k * 1_000_000, 1.0);
        }
        let per_vessel_ns = start.elapsed().as_nanos() as f64 / (1_000.0 * MAX_TARGETS as f64);
        println!(
            "colregs: {per_vessel_ns:.0} ns/vessel ({}) budget 50_000 ns",
            per_vessel_ns
        );
        assert!(
            per_vessel_ns < 50_000.0,
            "colregs budget blown: {per_vessel_ns} ns/vessel"
        );
    }

    #[test]
    fn quadrant_roundtrip_used_by_classifier() {
        assert_eq!(quadrant_of(112.5), Quadrant::Starboard);
    }
}
