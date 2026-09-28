//! Mission state machine for marine vehicles (spec §3, `tpt-t-marine-core`).
//!
//! Models the high-level mission lifecycle of a single vehicle — AUV, ASV,
//! glider, or ROV — as the ordered Dive → Transit → Survey → Surface →
//! Recover sequence from the design doc. Transition legality is encoded by
//! [`MissionState::next`]; illegal transitions are rejected rather than
//! panicked, so the caller can route them through the safety subsystem.
//!
//! The universal safety state machine from `tpt-t-domain-bridge`
//! (`AUTONOMOUS → REQUESTING_TELEOP → TELEOP_ACTIVE → RETURNING_TO_AUTONOMY`
//! plus `EMERGENCY_STOP`) composes *on top* of this machine: this crate
//! models the mission lifecycle, the bridge machine gates who is commanding.

use core::fmt;

/// High-level mission state of a marine vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MissionState {
    /// Vehicle is docked (moon pool, cradle, or pier) and powered down.
    Docked,
    /// Descending from the surface; GPS is lost, nav fuses INS/DVL/pressure.
    Dive,
    /// Travel between waypoints at transit speed and safe depth.
    Transit,
    /// Executing the payload mission (multibeam survey, inspection, sampling).
    Survey,
    /// Ascended / surfaced; establishing RF, cellular, or satellite link.
    Surface,
    /// Being recovered (moon pool reel-in, crane, or towed recovery).
    Recover,
    /// Fault / degraded: control handed to the safety subsystem
    /// (`tpt-t-marine-safety` reflexes, emergency buoyancy).
    Fault,
    /// Teleoperation engaged (operator driving remotely via
    /// `tpt-t-marine-teleop`).
    Teleop,
}

impl MissionState {
    /// Returns `Some(target)` if a transition from `self` to `target` is
    /// legal, otherwise `None`.
    ///
    /// The legal graph encodes the spec's Dive → Transit → Survey → Surface →
    /// Recover spine plus the shortcuts real missions take: a dive may go
    /// straight to survey when the work site is below the launch point, and
    /// survey/transit may surface directly on abort or mission end.
    pub fn next(self, target: MissionState) -> Option<MissionState> {
        if self == target {
            return Some(self);
        }
        use MissionState::*;
        let ok = matches!(
            (self, target),
            (Docked, Dive) // launch
                | (Dive, Transit)
                | (Dive, Survey)
                | (Dive, Surface) // abort ascent
                | (Transit, Survey)
                | (Transit, Dive) // descend to survey depth mid-leg
                | (Transit, Surface)
                | (Survey, Transit)
                | (Survey, Surface)
                | (Surface, Transit) // next mission leg
                | (Surface, Recover)
                | (Recover, Docked)
                | (_, Fault)     // any state may fault
                | (Fault, Surface) // emergency ascent is the fault exit
                | (Fault, Docked)  // …or straight back to the cradle
                | (_, Teleop)    // pilot assistance from any state
                | (Teleop, Dive)
                | (Teleop, Transit)
                | (Teleop, Survey)
                | (Teleop, Surface)
        );
        ok.then_some(target)
    }

    /// Whether the vehicle is submerged (GPS unavailable; nav runs on the
    /// INS/DVL/pressure/acoustic solution).
    pub fn submerged(self) -> bool {
        matches!(
            self,
            MissionState::Dive | MissionState::Transit | MissionState::Survey
        )
    }
}

impl fmt::Display for MissionState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            MissionState::Docked => "DOCKED",
            MissionState::Dive => "DIVE",
            MissionState::Transit => "TRANSIT",
            MissionState::Survey => "SURVEY",
            MissionState::Surface => "SURFACE",
            MissionState::Recover => "RECOVER",
            MissionState::Fault => "FAULT",
            MissionState::Teleop => "TELEOP",
        };
        f.write_str(name)
    }
}

/// Transition policy for a state type.
pub trait Transition {
    /// The state space this policy operates over.
    type State;
    /// Return the next state, or `None` if the transition is illegal.
    fn next(self, target: Self::State) -> Option<Self::State>;
}

impl Transition for MissionState {
    type State = MissionState;
    fn next(self, target: Self::State) -> Option<Self::State> {
        MissionState::next(self, target)
    }
}

/// A minimal generic state machine driven by an implementor of [`Transition`].
#[derive(Debug, Clone, Copy)]
pub struct StateMachine<S> {
    state: S,
}

impl<S: Copy + PartialEq + fmt::Debug> StateMachine<S> {
    /// Create a machine in `initial` state.
    pub fn new(initial: S) -> Self {
        Self { state: initial }
    }

    /// Current state.
    pub fn state(&self) -> S {
        self.state
    }

    /// Attempt a transition via [`Transition::next`].
    pub fn transition(&mut self, target: S) -> Result<(), TransitionError<S>>
    where
        S: Transition<State = S>,
    {
        match self.state.next(target) {
            Some(s) => {
                self.state = s;
                Ok(())
            }
            None => Err(TransitionError {
                from: self.state,
                to: target,
            }),
        }
    }
}

/// Error returned when a transition is rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransitionError<S> {
    /// State the machine was in.
    pub from: S,
    /// State that was requested.
    pub to: S,
}

impl<S: fmt::Debug> fmt::Display for TransitionError<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "illegal transition {:?} -> {:?}", self.from, self.to)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spec_spine_is_legal() {
        // The spec's canonical mission: Dive → Transit → Survey → Surface →
        // Recover → Docked.
        let mut m = StateMachine::new(MissionState::Docked);
        m.transition(MissionState::Dive).unwrap();
        m.transition(MissionState::Transit).unwrap();
        m.transition(MissionState::Survey).unwrap();
        m.transition(MissionState::Surface).unwrap();
        m.transition(MissionState::Recover).unwrap();
        m.transition(MissionState::Docked).unwrap();
        assert_eq!(m.state(), MissionState::Docked);
    }

    #[test]
    fn dive_direct_to_survey_and_shortcuts() {
        let mut m = StateMachine::new(MissionState::Docked);
        m.transition(MissionState::Dive).unwrap();
        // Work site directly below the launch point.
        m.transition(MissionState::Survey).unwrap();
        // Abort ascent.
        m.transition(MissionState::Surface).unwrap();
        assert_eq!(m.state(), MissionState::Surface);
    }

    #[test]
    fn illegal_jumps_rejected() {
        let mut m = StateMachine::new(MissionState::Docked);
        // Cannot survey from the cradle.
        assert!(m.transition(MissionState::Survey).is_err());
        // Cannot recover a docked vehicle.
        assert!(m.transition(MissionState::Recover).is_err());
        assert_eq!(m.state(), MissionState::Docked);
        // Surface cannot dive again without transiting/descending first.
        m.transition(MissionState::Dive).unwrap();
        m.transition(MissionState::Surface).unwrap();
        assert!(m.transition(MissionState::Recover).is_ok());
    }

    #[test]
    fn fault_from_anywhere_recovers_via_surface_or_dock() {
        let mut m = StateMachine::new(MissionState::Survey);
        assert!(m.transition(MissionState::Fault).is_ok());
        // Fault cannot go straight back to work.
        assert!(m.transition(MissionState::Survey).is_err());
        // Emergency ascent, then recovery.
        m.transition(MissionState::Surface).unwrap();
        m.transition(MissionState::Recover).unwrap();
        m.transition(MissionState::Docked).unwrap();
        assert_eq!(m.state(), MissionState::Docked);

        let mut m2 = StateMachine::new(MissionState::Transit);
        m2.transition(MissionState::Fault).unwrap();
        m2.transition(MissionState::Docked).unwrap();
        assert_eq!(m2.state(), MissionState::Docked);
    }

    #[test]
    fn teleop_from_anywhere_and_back() {
        let mut m = StateMachine::new(MissionState::Survey);
        assert!(m.transition(MissionState::Teleop).is_ok());
        assert!(m.transition(MissionState::Survey).is_ok());
        assert!(m.transition(MissionState::Teleop).is_ok());
        assert!(m.transition(MissionState::Surface).is_ok());
    }

    #[test]
    fn same_state_is_idempotent() {
        let mut m = StateMachine::new(MissionState::Transit);
        assert!(m.transition(MissionState::Transit).is_ok());
        assert_eq!(m.state(), MissionState::Transit);
    }

    #[test]
    fn submerged_flag_matches_gps_availability() {
        assert!(MissionState::Dive.submerged());
        assert!(MissionState::Transit.submerged());
        assert!(MissionState::Survey.submerged());
        assert!(!MissionState::Surface.submerged());
        assert!(!MissionState::Docked.submerged());
        assert!(!MissionState::Recover.submerged());
    }

    #[test]
    fn display_uses_spec_names() {
        assert_eq!(MissionState::Dive.to_string(), "DIVE");
        assert_eq!(MissionState::Survey.to_string(), "SURVEY");
    }
}
