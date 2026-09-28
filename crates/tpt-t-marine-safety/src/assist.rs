//! Teleop-assist integration: an unrecoverable fault raises the universal
//! `REQUESTING_TELEOP` state through the `request_teleop_assistance()`
//! API (bridge spec §5, "Fault Detection & Handover Trigger").
//!
//! [`FaultAssist`] classifies internal safety events into
//! [`AssistReason`](tpt_t_domain_bridge::assist::AssistReason)s, keeps the
//! urgency table marine-flavored (flooding and thermal shutdown are
//! immediate; drift watch is routine), and drives the universal
//! [`SafetyStateMachine`](tpt_t_domain_bridge::safety::SafetyStateMachine).

use tpt_t_domain_bridge::assist::{AssistReason, AssistRequest, AssistRequester, Urgency};
use tpt_t_domain_bridge::safety::{SafetyEvent, SafetyState, SafetyStateMachine};

/// Safety events that can request operator assistance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyFault {
    /// Flooding alarm latched (see [`crate::flooding`]).
    Flooding,
    /// Thermal shutdown latched (see [`crate::thermal`]).
    ThermalShutdown,
    /// Lost-vehicle procedure started (see [`crate::lost`]).
    LostVehicle,
    /// Navigation confidence collapsed (EKF diverging, DVL bottom-lock
    /// lost for the whole mission leg).
    NavDegraded,
    /// Mission cannot continue within its battery reserve.
    BatteryReserve,
}

impl SafetyFault {
    /// The universal reason code + urgency this fault maps to.
    pub fn classification(self) -> (AssistReason, Urgency) {
        match self {
            SafetyFault::Flooding => (AssistReason::SensorFault, Urgency::Immediate),
            SafetyFault::ThermalShutdown => (AssistReason::Other, Urgency::Immediate),
            SafetyFault::LostVehicle => (AssistReason::Other, Urgency::Elevated),
            SafetyFault::NavDegraded => (AssistReason::LowAutonomyConfidence, Urgency::Elevated),
            SafetyFault::BatteryReserve => (AssistReason::Other, Urgency::Routine),
        }
    }
}

/// The assist façade: owns the universal safety machine and the request
/// gate. `Copy`-sized.
#[derive(Debug, Clone, Copy)]
pub struct FaultAssist {
    machine: SafetyStateMachine,
    requester: AssistRequester,
}

impl FaultAssist {
    /// Create with a re-request cooldown of one second.
    pub fn new() -> Self {
        Self {
            machine: SafetyStateMachine::default(),
            requester: AssistRequester::new(1_000_000),
        }
    }

    /// Universal safety machine state.
    pub fn safety_state(&self) -> SafetyState {
        self.machine.state()
    }

    /// Raise a fault: transition the universal machine toward
    /// `REQUESTING_TELEOP` and (on success) deliver the assist request
    /// through `sink`. `detail` is a vehicle-local sub-code.
    pub fn raise<S: tpt_t_domain_bridge::assist::AssistSink>(
        &mut self,
        fault: SafetyFault,
        lat_deg: f64,
        lon_deg: f64,
        detail: u16,
        now_us: u64,
        sink: &mut S,
    ) -> Result<AssistRequest, tpt_t_domain_bridge::assist::AssistError> {
        // The machine may refuse (e.g. already requesting) — that is fine:
        // the request still goes out.
        let _ = self.machine.dispatch(SafetyEvent::RequestAssistance);
        let (reason, urgency) = fault.classification();
        let req = AssistRequest {
            timestamp_us: 0, // stamped by the requester
            seq: 0,
            reason,
            urgency,
            lat_deg,
            lon_deg,
            detail,
            operator_id: tpt_t_domain_bridge::wire::OperatorId::LEGACY_ANON,
        };
        self.requester.request(now_us, req, sink)
    }
}

impl Default for FaultAssist {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flooding_maps_to_immediate_sensor_fault() {
        let (reason, urgency) = SafetyFault::Flooding.classification();
        assert_eq!(reason, AssistReason::SensorFault);
        assert_eq!(urgency, Urgency::Immediate);
    }

    #[test]
    fn fault_drives_the_universal_machine_and_delivers_request() {
        let mut fa = FaultAssist::new();
        let mut delivered = Vec::new();
        let mut sink = |r: AssistRequest| {
            delivered.push(r);
            true
        };
        assert_eq!(fa.safety_state(), SafetyState::Autonomous);
        fa.raise(
            SafetyFault::Flooding,
            -40.5,
            175.2,
            0x0101,
            5_000_000,
            &mut sink,
        )
        .expect("delivered");
        assert_eq!(fa.safety_state(), SafetyState::RequestingTeleop);
        assert_eq!(delivered.len(), 1);
        assert_eq!(delivered[0].reason, AssistReason::SensorFault);
        assert_eq!(delivered[0].urgency, Urgency::Immediate);
    }

    #[test]
    fn cooldown_suppresses_but_machine_stays_requesting() {
        let mut fa = FaultAssist::new();
        let mut sink = |_r: AssistRequest| true;
        fa.raise(SafetyFault::NavDegraded, 0.0, 0.0, 1, 0, &mut sink)
            .unwrap();
        // 100 ms later: suppressed by cooldown…
        assert!(
            fa.raise(SafetyFault::NavDegraded, 0.0, 0.0, 1, 100_000, &mut sink)
                .is_err()
        );
        // …but the machine is still in RequestingTeleop (dispatch is
        // idempotent there — it refuses the repeat transition silently).
        assert_eq!(fa.safety_state(), SafetyState::RequestingTeleop);
    }

    #[test]
    fn full_handover_cycle() {
        let mut fa = FaultAssist::new();
        let mut sink = |_r: AssistRequest| true;
        fa.raise(SafetyFault::LostVehicle, 0.0, 0.0, 0, 0, &mut sink)
            .unwrap();
        // Operator picks up.
        fa.machine.dispatch(SafetyEvent::TeleopEngaged).unwrap();
        assert_eq!(fa.safety_state(), SafetyState::TeleopActive);
        fa.machine.dispatch(SafetyEvent::TeleopReleased).unwrap();
        fa.machine.dispatch(SafetyEvent::ReturnComplete).unwrap();
        assert_eq!(fa.safety_state(), SafetyState::Autonomous);
    }
}
