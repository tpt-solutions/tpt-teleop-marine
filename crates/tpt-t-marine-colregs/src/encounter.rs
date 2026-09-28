//! Encounter classification (Rules 13–15) and the per-vessel give-way /
//! stand-on FSM with Rule 17 depth-of-action.

use crate::track::{Cbdr, OwnState, Quadrant, TargetTrack, quadrant_of};
use crate::whistle::Blasts;
use core::fmt;

/// Encounter class.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Encounter {
    /// Rule 13: own vessel is the overtaking vessel — keep clear regardless.
    Overtaking,
    /// Rule 13: own vessel is being overtaken — stand on (with Rule 17 tail).
    BeingOvertaken,
    /// Rule 14: reciprocal course near the bow line — both alter to
    /// starboard, pass port-to-port.
    HeadOn,
    /// Rule 15: crossing; give way if the target is on own starboard side.
    CrossingGiveWay,
    /// Rule 15: crossing; target has us to starboard — stand on.
    CrossingStandOn,
    /// No risk geometry (or CBDR clear).
    Clear,
}

impl Encounter {
    /// Whether own vessel must keep out of the way.
    pub fn own_gives_way(self) -> bool {
        matches!(
            self,
            Encounter::Overtaking | Encounter::CrossingGiveWay | Encounter::HeadOn
        )
    }

    /// The Rule 34 whistle signal this encounter's primary maneuver calls
    /// for (None = hold course, no signal).
    pub fn whistle(self) -> Option<Blasts> {
        match self {
            // Both vessels alter to starboard: one short.
            Encounter::HeadOn => Some(Blasts::One),
            // Give-way altering to starboard to pass astern.
            Encounter::CrossingGiveWay | Encounter::Overtaking => Some(Blasts::One),
            _ => None,
        }
    }
}

impl fmt::Display for Encounter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            Encounter::Overtaking => "OVERTAKING(R13)",
            Encounter::BeingOvertaken => "BEING_OVERTAKEN(R13)",
            Encounter::HeadOn => "HEAD_ON(R14)",
            Encounter::CrossingGiveWay => "CROSSING_GIVE_WAY(R15)",
            Encounter::CrossingStandOn => "CROSSING_STAND_ON(R15)",
            Encounter::Clear => "CLEAR",
        };
        f.write_str(name)
    }
}

/// Classify one encounter from geometry.
pub fn classify(own: &OwnState, t: &TargetTrack) -> Encounter {
    if t.own_is_overtaking(own) {
        return Encounter::Overtaking;
    }
    if t.target_is_overtaking(own) {
        return Encounter::BeingOvertaken;
    }
    if t.is_head_on(own) {
        return Encounter::HeadOn;
    }
    match quadrant_of(t.relative_bearing()) {
        Quadrant::Starboard => Encounter::CrossingGiveWay,
        Quadrant::Port => Encounter::CrossingStandOn,
        // Fine-angle/mixed geometry falls back to the starboard-side rule.
        Quadrant::Ahead => {
            if t.relative_bearing() >= 0.0 {
                Encounter::CrossingGiveWay
            } else {
                Encounter::CrossingStandOn
            }
        }
        Quadrant::Astern => Encounter::Clear,
    }
}

/// Depth of maneuvering (Rule 17).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DepthOfAction {
    /// Rule 17(a): the give-way vessel keeps clear; stand-on holds course
    /// and speed.
    Hold,
    /// Rule 17(a)(ii): the give-way has failed to act — the stand-on may
    /// act.
    MayAct,
    /// Rule 17(b): collision can no longer be avoided by the give-way
    /// alone — the stand-on MUST act.
    MustAct,
}

/// Per-vessel encounter FSM state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EncounterFsm {
    /// Track id this slot serves.
    pub id: u16,
    /// Last classification.
    pub class: Encounter,
    /// Whether CBDR risk was established this tick.
    pub risk: bool,
    /// Rule 17 depth of action.
    pub depth: DepthOfAction,
    /// Seconds the give-way has been in risk without the geometry opening.
    pub risk_s: f64,
    /// Whether this FSM is engaged (slot in use).
    pub engaged: bool,
}

impl EncounterFsm {
    /// Idle slot for a track id.
    pub fn idle(id: u16) -> Self {
        Self {
            id,
            class: Encounter::Clear,
            risk: false,
            depth: DepthOfAction::Hold,
            risk_s: 0.0,
            engaged: false,
        }
    }

    /// Advance one evaluation tick: classify, test CBDR, deepen Rule 17
    /// action. Returns the classification.
    pub fn update(
        &mut self,
        own: &OwnState,
        t: &TargetTrack,
        prev: Option<Cbdr>,
        dt_s: f64,
    ) -> Encounter {
        self.id = t.id;
        self.engaged = true;
        let class = classify(own, t);
        self.class = class;

        let risk = prev.map(|p| p.risk(t, dt_s)).unwrap_or(false);
        if risk {
            self.risk_s += dt_s;
        } else {
            // Geometry opened: decay the pressure (and re-arm).
            self.risk_s = (self.risk_s - dt_s).max(0.0);
            if self.risk_s == 0.0 {
                self.depth = DepthOfAction::Hold;
                self.risk = false;
                return class;
            }
        }
        self.risk = true;
        // Rule 17 timing: give-way gets 60 s to act, then stand-on may
        // act; after 120 s it must.
        self.depth = if self.risk_s >= 120.0 {
            DepthOfAction::MustAct
        } else if self.risk_s >= 60.0 {
            DepthOfAction::MayAct
        } else {
            DepthOfAction::Hold
        };
        class
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn own() -> OwnState {
        OwnState {
            course_deg: 0.0,
            speed_mps: 3.0,
        }
    }

    fn t(bearing: f64, course: f64, speed: f64) -> TargetTrack {
        TargetTrack {
            id: 7,
            bearing_deg: bearing,
            range_m: 1500.0,
            course_deg: course,
            speed_mps: speed,
        }
    }

    #[test]
    fn crossing_starboard_is_give_way() {
        // Target on own starboard bow, crossing port-to-starboard.
        let x = t(60.0, 270.0, 2.0);
        assert_eq!(classify(&own(), &x), Encounter::CrossingGiveWay);
        assert!(classify(&own(), &x).whistle() == Some(Blasts::One));
    }

    #[test]
    fn crossing_port_is_stand_on() {
        let x = t(-60.0, 90.0, 2.0);
        assert_eq!(classify(&own(), &x), Encounter::CrossingStandOn);
    }

    #[test]
    fn head_on_both_alter_starboard() {
        let h = t(0.0, 180.0, 3.0);
        assert_eq!(classify(&own(), &h), Encounter::HeadOn);
        assert!(h.relative_bearing().abs() < 22.5);
    }

    #[test]
    fn overtaking_beats_crossing_classification() {
        // Geometrically on the starboard side but approaching from abaft
        // the target's beam: Rule 13 wins — own is the overtaker.
        let o = OwnState {
            course_deg: 90.0,
            speed_mps: 6.0,
        };
        let slow = TargetTrack {
            id: 9,
            bearing_deg: 90.0,
            range_m: 800.0,
            course_deg: 90.0,
            speed_mps: 1.0,
        };
        assert_eq!(classify(&o, &slow), Encounter::Overtaking);
    }

    #[test]
    fn rule_17_deepens_with_persistent_risk() {
        let o = own();
        let mut fsm = EncounterFsm::idle(7);
        let mut t = t(60.0, 270.0, 2.0);
        let mut prev = Cbdr {
            prev_bearing_deg: t.bearing_deg,
            prev_range_m: t.range_m,
        };
        // Steady bearing, 1 m/s closure, evaluated once per second.
        for _ in 0..130 {
            t.range_m -= 1.0;
            let cbdr = Cbdr {
                prev_bearing_deg: prev.prev_bearing_deg,
                prev_range_m: prev.prev_range_m,
            };
            fsm.update(&o, &t, Some(cbdr), 1.0);
            prev.prev_bearing_deg = t.bearing_deg;
            prev.prev_range_m = t.range_m;
        }
        assert!(fsm.risk);
        assert_eq!(fsm.depth, DepthOfAction::MustAct, "130 s of closure");
    }

    #[test]
    fn geometry_opening_disarms_rule_17() {
        let o = own();
        let mut fsm = EncounterFsm::idle(7);
        fsm.risk_s = 130.0;
        fsm.depth = DepthOfAction::MustAct;
        fsm.risk = true;
        let t = t(-60.0, 90.0, 2.0); // opening geometry (no closure info)
        // Sustained no-risk ticks decay the pressure to disarmed.
        for _ in 0..140 {
            fsm.update(&o, &t, None, 1.0);
        }
        assert!(!fsm.risk, "no CBDR input: pressure decays");
        assert_eq!(fsm.depth, DepthOfAction::Hold);
    }
}
