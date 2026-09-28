//! Rule 34 manoeuvring sound signals, as decision outputs.

/// Whistle blast counts the engine emits for a manoeuvre.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Blasts {
    /// One short blast: "I am altering my course to starboard."
    One = 1,
    /// Two short blasts: "I am altering my course to port."
    Two = 2,
    /// Three short blasts: "I am operating astern propulsion."
    Three = 3,
    /// Five (or more) short blasts: doubt / danger (Rule 34(d)).
    Doubt = 5,
}

impl Blasts {
    /// Blast count.
    pub fn count(self) -> u8 {
        self as u8
    }

    /// The Rule 34 citation for this signal.
    pub fn citation(self) -> &'static str {
        match self {
            Blasts::One => "COLREGS Rule 34(a)(i): altering course to starboard",
            Blasts::Two => "COLREGS Rule 34(a)(ii): altering course to port",
            Blasts::Three => "COLREGS Rule 34(a)(iii): operating astern propulsion",
            Blasts::Doubt => "COLREGS Rule 34(d): doubt or danger",
        }
    }
}

/// The maneuver the ASV executes for a give-way or head-on resolution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Maneuver {
    /// Hold course and speed.
    HoldCourse,
    /// Alter course to starboard by `deg`.
    AlterStarboard(f32),
    /// Alter course to port by `deg`.
    AlterPort(f32),
    /// Slow / stop (Rule 16 "slow down or stop" option).
    SlowDown,
}

impl Maneuver {
    /// Compact discriminant for the decision log.
    pub fn code(self) -> u8 {
        match self {
            Maneuver::HoldCourse => 0,
            Maneuver::AlterStarboard(_) => 1,
            Maneuver::AlterPort(_) => 2,
            Maneuver::SlowDown => 3,
        }
    }

    /// The whistle signal for this maneuver (None = silent hold).
    pub fn whistle(self) -> Option<Blasts> {
        match self {
            Maneuver::HoldCourse => None,
            Maneuver::AlterStarboard(_) => Some(Blasts::One),
            Maneuver::AlterPort(_) => Some(Blasts::Two),
            Maneuver::SlowDown => Some(Blasts::Three),
        }
    }
}

/// The maneuver a class calls for, with rule citation.
pub fn maneuver_for(
    class: crate::encounter::Encounter,
    depth: crate::encounter::DepthOfAction,
) -> (Maneuver, &'static str) {
    use crate::encounter::{DepthOfAction as D, Encounter as E};
    match (class, depth) {
        (E::HeadOn, _) => (
            Maneuver::AlterStarboard(30.0),
            "COLREGS Rule 14: alter course to starboard, pass port-to-port",
        ),
        (E::CrossingGiveWay, D::Hold) => (
            Maneuver::AlterStarboard(45.0),
            "COLREGS Rule 15/16: cross astern of the stand-on vessel",
        ),
        (E::CrossingGiveWay, _) => (
            Maneuver::AlterStarboard(45.0),
            "COLREGS Rule 16: early and substantial action",
        ),
        (E::Overtaking, _) => (
            Maneuver::AlterStarboard(30.0),
            "COLREGS Rule 13: the overtaking vessel keeps clear",
        ),
        (E::CrossingStandOn, D::MustAct) => (
            Maneuver::AlterStarboard(45.0),
            "COLREGS Rule 17(b): take action as the stand-on vessel alone",
        ),
        (E::CrossingStandOn, D::MayAct) => (
            Maneuver::HoldCourse,
            "COLREGS Rule 17(a)(ii): hold course; action permitted but not required",
        ),
        (E::CrossingStandOn, D::Hold) => (
            Maneuver::HoldCourse,
            "COLREGS Rule 17(a)(i): stand on, hold course and speed",
        ),
        (E::BeingOvertaken, D::Hold) => (
            Maneuver::HoldCourse,
            "COLREGS Rule 17(a)(i): the overtaking vessel keeps clear",
        ),
        (E::BeingOvertaken, _) => (
            Maneuver::HoldCourse,
            "COLREGS Rule 17: hold; overtaker must keep clear",
        ),
        (E::Clear, _) => (Maneuver::HoldCourse, "No risk geometry"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blast_counts_match_rule_34() {
        assert_eq!(Blasts::One.count(), 1);
        assert_eq!(Blasts::Two.count(), 2);
        assert_eq!(Blasts::Three.count(), 3);
        assert_eq!(Blasts::Doubt.count(), 5);
        assert!(Blasts::One.citation().contains("34(a)(i)"));
        assert!(Blasts::Doubt.citation().contains("34(d)"));
    }

    #[test]
    fn maneuvers_sound_their_whistles() {
        assert_eq!(Maneuver::AlterStarboard(30.0).whistle(), Some(Blasts::One));
        assert_eq!(Maneuver::AlterPort(30.0).whistle(), Some(Blasts::Two));
        assert_eq!(Maneuver::SlowDown.whistle(), Some(Blasts::Three));
        assert_eq!(Maneuver::HoldCourse.whistle(), None);
    }

    #[test]
    fn stand_on_holds_until_must_act() {
        use crate::encounter::{DepthOfAction as D, Encounter as E};
        let (m1, c1) = maneuver_for(E::CrossingStandOn, D::Hold);
        assert_eq!(m1, Maneuver::HoldCourse);
        assert!(c1.contains("17(a)(i)"));
        let (m2, c2) = maneuver_for(E::CrossingStandOn, D::MustAct);
        assert!(matches!(m2, Maneuver::AlterStarboard(_)));
        assert!(c2.contains("17(b)"));
    }
}
