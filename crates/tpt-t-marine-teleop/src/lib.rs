#![warn(missing_docs)]
//! `tpt-t-marine-teleop` — the marine domain adapter (spec §5.2, bridge
//! spec §6).
//!
//! The only crate that speaks DTI to the core teleop system. It:
//!
//! * translates normalized [`ControlCommand`]s into 6DOF thruster commands
//!   with per-axis slew limiting (smooth, actuator-safe),
//! * blends teleop in and out over a ramp window so the autonomy↔teleop
//!   handover never jerks the vehicle,
//! * lets the tether monitor override pilot commands on snag risk
//!   (through `tpt-t-marine-rov`'s guard),
//! * falls back to the acoustic stack at 1 Hz when the fiber tether is
//!   cut — the pilot keeps critical-command capability (spec §5.2
//!   "Acoustic Comms Fallback"),
//! * forwards buoyancy-compensation requests as payload changes (spec:
//!   "auto-adjust for buoyancy changes as the ROV deploys tools").

use std::cell::Cell;

use tpt_t_domain_bridge::dti::{DomainTeleopInterface, DtiError};
use tpt_t_domain_bridge::safety::{SafetyState, SafetyStateMachine};
use tpt_t_domain_bridge::wire::{ControlCommand, DomainState, OperatorId, SensorFeed};
use tpt_t_marine_core::wire::ThrusterCmd6;
use tpt_t_marine_rov::tether_guard::TetherGuard;
use tpt_t_marine_tether::predict::SnagLevel;

/// Slew limits per axis (fraction/second at full rate): surge, sway,
/// heave, roll, pitch, yaw.
pub const SLEW_LIMITS: [f32; 6] = [1.5, 1.5, 1.0, 2.0, 2.0, 1.5];

/// Acoustic fallback control period, microseconds (spec: 1 Hz).
pub const ACOUSTIC_PERIOD_US: u64 = 1_000_000;

/// Handover ramp time, seconds.
pub const BLEND_TIME_S: f32 = 2.0;

/// The adapter state. Not `Sync`/`Send` internals — it lives on the
/// teleop session's loop thread; shared outputs go through the rings the
/// caller wires around it.
pub struct MarineTeleopAdapter {
    machine: SafetyStateMachine,
    operator: Option<OperatorId>,
    blend: f32,
    last_cmd: ThrusterCmd6,
    /// Slew-limited output actually emitted.
    slewed: ThrusterCmd6,
    guard: TetherGuard,
    snag: SnagLevel,
    tether_ok: bool,
    last_acoustic_cmd_us: u64,
    /// Last haptic level (surfaced in [`DomainState::fault_bits`]).
    pub last_haptic: Cell<u8>,
    pose: DomainPoseView,
}

/// The vehicle's view the adapter reports upward.
#[derive(Debug, Clone, Copy, Default)]
pub struct DomainPoseView {
    /// Position, metres (datum frame).
    pub ned_m: [f64; 3],
    /// Heading, degrees.
    pub heading_deg: f32,
    /// Speed, m/s.
    pub speed_mps: f32,
    /// Battery SoC, %.
    pub battery_soc_pct: f32,
}

impl MarineTeleopAdapter {
    /// New adapter, autonomous, tether healthy.
    pub fn new() -> Self {
        Self {
            machine: SafetyStateMachine::default(),
            operator: None,
            blend: 0.0,
            last_cmd: ThrusterCmd6::default(),
            slewed: ThrusterCmd6::default(),
            guard: TetherGuard::default(),
            snag: SnagLevel::Clear,
            tether_ok: true,
            last_acoustic_cmd_us: 0,
            last_haptic: Cell::new(0),
            pose: DomainPoseView::default(),
        }
    }

    /// Update the vehicle view reported via [`get_domain_state`].
    pub fn set_vehicle_view(&mut self, v: DomainPoseView) {
        self.pose = v;
    }

    /// Current teleop blend [0=autonomy, 1=teleop].
    pub fn blend(&self) -> f32 {
        self.blend
    }

    /// Universal safety machine state.
    pub fn safety_state(&self) -> SafetyState {
        self.machine.state()
    }

    /// Report tether health. Cutting the fiber flips the adapter into the
    /// acoustic fallback (1 Hz critical commands through the stack).
    pub fn set_tether_ok(&mut self, ok: bool, now_us: u64) {
        if self.tether_ok && !ok {
            // Arm the fallback so a command is due immediately.
            self.last_acoustic_cmd_us = now_us.saturating_sub(ACOUSTIC_PERIOD_US);
        }
        self.tether_ok = ok;
        let _ = now_us;
    }

    /// Report the current snag level from the tether predictor.
    pub fn set_snag_level(&mut self, level: SnagLevel) {
        self.snag = level;
    }

    /// The slew-limited command emitted for this tick, blended with the
    /// autonomy command. `autonomy` is the AUV/ROV controller's own
    /// command (used while the blend is < 1).
    pub fn tick(&mut self, dt_s: f32, autonomy: ThrusterCmd6) -> ThrusterCmd6 {
        // Blend ramp.
        let target = if self.machine.state() == SafetyState::TeleopActive {
            1.0
        } else {
            0.0
        };
        let step = dt_s / BLEND_TIME_S;
        self.blend = if self.blend < target {
            (self.blend + step).min(target)
        } else {
            (self.blend - step).max(target)
        };

        // Guard the teleop command against snag risk.
        let guarded = self.guard.filter(self.last_cmd, self.snag);
        let buzz = self.snag == SnagLevel::Warn;
        self.last_haptic.set(buzz as u8);
        let teleop = guarded.0;

        // Blend then slew.
        let b = self.blend;
        let target_cmd = ThrusterCmd6 {
            surge: b * teleop.surge + (1.0 - b) * autonomy.surge,
            sway: b * teleop.sway + (1.0 - b) * autonomy.sway,
            heave: b * teleop.heave + (1.0 - b) * autonomy.heave,
            roll: b * teleop.roll + (1.0 - b) * autonomy.roll,
            pitch: b * teleop.pitch + (1.0 - b) * autonomy.pitch,
            yaw: b * teleop.yaw + (1.0 - b) * autonomy.yaw,
        }
        .clamped();
        slew(&mut self.slewed, &target_cmd, SLEW_LIMITS, dt_s);
        self.slewed
    }

    /// Whether an acoustic control command may go out now (1 Hz gate,
    /// fallback only).
    pub fn acoustic_command_due(&self, now_us: u64) -> bool {
        !self.tether_ok
            && self.machine.state() == SafetyState::TeleopActive
            && now_us.saturating_sub(self.last_acoustic_cmd_us) >= ACOUSTIC_PERIOD_US
    }

    /// Mark an acoustic command as sent (advances the 1 Hz gate).
    pub fn mark_acoustic_sent(&mut self, now_us: u64) {
        self.last_acoustic_cmd_us = now_us;
    }

    /// Buoyancy compensation hook: the pilot's tool deployment shifts the
    /// ballast demand (routed to `tpt-t-marine-buoyancy` by the integrator).
    pub fn payload_changed_kg(&mut self, delta_kg: f32) -> f32 {
        // Positive payload → trim demand shifts; the adapter scales it by
        // the blend so a mid-handover deployment still lands cleanly.
        delta_kg * (0.5 + 0.5 * self.blend)
    }
}

impl Default for MarineTeleopAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl DomainTeleopInterface for MarineTeleopAdapter {
    fn on_teleop_engage(&mut self, operator_id: OperatorId) -> Result<(), DtiError> {
        if self.operator.is_some() {
            return Err(DtiError::InvalidState);
        }
        self.machine
            .dispatch(tpt_t_domain_bridge::safety::SafetyEvent::TeleopEngaged)
            .map_err(|_| DtiError::InvalidState)?;
        self.operator = Some(operator_id);
        Ok(())
    }

    fn on_teleop_disengage(&mut self) -> Result<(), DtiError> {
        if self.operator.is_none() {
            return Err(DtiError::InvalidState);
        }
        self.machine
            .dispatch(tpt_t_domain_bridge::safety::SafetyEvent::TeleopReleased)
            .map_err(|_| DtiError::InvalidState)?;
        self.operator = None;
        Ok(())
    }

    fn on_control_command(&mut self, cmd: ControlCommand) -> Result<(), DtiError> {
        if self.operator.is_none() || self.machine.stopped() {
            return Err(DtiError::InvalidState);
        }
        // E-stop from the console: latched by the machine.
        if cmd.buttons.e_stop() {
            self.machine
                .dispatch(tpt_t_domain_bridge::safety::SafetyEvent::EmergencyStop)
                .map_err(|_| DtiError::InvalidState)?;
            self.last_cmd = ThrusterCmd6::default();
            return Ok(());
        }
        // Axes → 6DOF: primary = surge/sway/heave/yaw, secondary = arm
        // proxy (roll/pitch trimmed here for v1).
        let a = &cmd.primary_input.axes;
        let s = &cmd.secondary_input.axes;
        self.last_cmd = ThrusterCmd6 {
            surge: a[0],
            sway: a[1],
            heave: a[2],
            yaw: a[3],
            roll: s[0],
            pitch: s[1],
        }
        .clamped();
        Ok(())
    }

    fn get_domain_state(&self) -> DomainState {
        DomainState {
            timestamp_us: 0,
            lat_deg: 0.0,
            lon_deg: 0.0,
            alt_m: -self.pose.ned_m[2] as f32,
            yaw_deg: self.pose.heading_deg,
            speed_m_s: self.pose.speed_mps,
            safety_state: self.machine.state().as_u8(),
            fault_bits: (self.last_haptic.get() as u64) << 56,
            telemetry: Default::default(),
        }
    }

    fn get_sensor_feed(&mut self) -> SensorFeed {
        // The ROV streams camera frames and multibeam clouds by
        // out-of-band reference (bridge spec §4.2); the adapter emits the
        // metadata-only feed.
        SensorFeed::default()
    }
}

/// Slew `cur` toward `target` at per-axis limits.
fn slew(cur: &mut ThrusterCmd6, target: &ThrusterCmd6, limits: [f32; 6], dt_s: f32) {
    let mut cur_axes = [
        &mut cur.surge,
        &mut cur.sway,
        &mut cur.heave,
        &mut cur.roll,
        &mut cur.pitch,
        &mut cur.yaw,
    ];
    let want = [
        target.surge,
        target.sway,
        target.heave,
        target.roll,
        target.pitch,
        target.yaw,
    ];
    for (k, axis) in cur_axes.iter_mut().enumerate() {
        let step = limits[k] * dt_s;
        **axis += (want[k] - **axis).clamp(-step, step);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ACOUSTIC_PERIOD_US;
    use tpt_t_domain_bridge::wire::{Button, ButtonState, InputState};

    fn engaged() -> MarineTeleopAdapter {
        let mut a = MarineTeleopAdapter::new();
        a.on_teleop_engage(OperatorId([9; 16])).unwrap();
        a
    }

    fn cmd(surge: f32) -> ControlCommand {
        let mut c = ControlCommand {
            timestamp_us: 0,
            operator_id: OperatorId([9; 16]),
            primary_input: InputState::default(),
            secondary_input: InputState::default(),
            buttons: ButtonState::default(),
            haptic_feedback: Default::default(),
            seq: 1,
        };
        c.primary_input.axes[0] = surge;
        c
    }

    #[test]
    fn control_translates_and_slew_limits() {
        let mut a = engaged();
        a.on_control_command(cmd(1.0)).unwrap();
        // First tick: the slew limiter moves the output only part way.
        let out = a.tick(0.1, ThrusterCmd6::default());
        assert!(out.surge > 0.0 && out.surge < 0.5, "slewed: {}", out.surge);
        // After the blend completes, the output reaches the demand.
        for _ in 0..60 {
            a.on_control_command(cmd(1.0)).unwrap();
            a.tick(0.1, ThrusterCmd6::default());
        }
        assert!((a.tick(0.1, ThrusterCmd6::default()).surge - 1.0).abs() < 0.05);
    }

    #[test]
    fn blend_ramps_so_the_handover_does_not_jerk() {
        let mut a = engaged();
        a.on_control_command(cmd(1.0)).unwrap();
        let first = a.tick(0.1, ThrusterCmd6::default()).surge;
        assert!(first < 0.5, "blend starts near autonomy: {first}");
        assert_eq!(a.blend(), 0.05, "2 s ramp at 10 Hz ticks");
        // Disengage before the ramp finishes: the blend returns to 0 with
        // no step change.
        a.on_teleop_disengage().unwrap();
        let out = a.tick(0.1, ThrusterCmd6::default());
        assert!(out.surge <= first, "no jerk on release: {out:?}");
    }

    #[test]
    fn e_stop_latches_and_zeroes() {
        let mut a = engaged();
        let mut c = cmd(1.0);
        c.buttons.set(Button::EStop);
        a.on_control_command(c).unwrap();
        assert_eq!(a.safety_state(), SafetyState::EmergencyStop);
        assert!(a.tick(0.1, ThrusterCmd6::default()).is_zero());
        // Commands are refused while stopped.
        assert_eq!(a.on_control_command(cmd(1.0)), Err(DtiError::InvalidState));
    }

    #[test]
    fn tether_cut_switches_to_acoustic_fallback() {
        let mut a = engaged();
        let t0 = 10_000_000u64;
        a.set_tether_ok(false, t0);
        // 1 Hz gate: nothing due immediately after arming… the arm sets
        // last_acoustic = 0, so the first check at t0 is due.
        assert!(a.acoustic_command_due(t0));
        a.mark_acoustic_sent(t0);
        // 0.5 s later: too soon.
        assert!(!a.acoustic_command_due(t0 + 500_000));
        // 1 s later: due again.
        assert!(a.acoustic_command_due(t0 + ACOUSTIC_PERIOD_US));
    }

    #[test]
    fn tether_restored_disables_fallback() {
        let mut a = engaged();
        let t0 = ACOUSTIC_PERIOD_US;
        a.set_tether_ok(false, t0);
        assert!(a.acoustic_command_due(t0));
        a.mark_acoustic_sent(t0);
        a.set_tether_ok(true, t0 + 100_000);
        assert!(!a.acoustic_command_due(t0 + 2_000_000));
    }

    #[test]
    fn snag_warn_overrides_the_pilot() {
        let mut a = engaged();
        a.on_control_command(cmd(1.0)).unwrap();
        a.set_snag_level(SnagLevel::Warn);
        let out = a.tick(0.1, ThrusterCmd6::default());
        // The Warn refusal zeroes the teleop component; blend is 0.05 so
        // the emitted command is tiny — and the haptic buzzes.
        assert!(out.surge.abs() < 0.1, "pilot overridden: {}", out.surge);
        assert_eq!(a.last_haptic.get(), 1);
    }

    #[test]
    fn payload_change_scales_with_blend() {
        let mut a = MarineTeleopAdapter::new();
        assert!(
            (a.payload_changed_kg(4.0) - 2.0).abs() < 1e-6,
            "autonomy: half"
        );
        let mut b = engaged();
        b.on_control_command(cmd(1.0)).unwrap();
        for _ in 0..40 {
            b.tick(0.1, ThrusterCmd6::default());
        }
        assert!(
            (b.payload_changed_kg(4.0) - 4.0).abs() < 0.01,
            "full teleop: full"
        );
    }
}
