//! The flooding reflex: leak alarm → emergency buoyancy, inside the <100 ms
//! budget (spec §5.4).
//!
//! Sequencing (per the recovery checklist): the **drop weight** releases
//! first (mechanical, fails-safe, instant), then the **ballast bladder**
//! blows over the following seconds for positive buoyancy, then the
//! vehicle is commanded to **ascend**. The reflex arms on one latched
//! [`LeakAlarm`](tpt_t_marine_pressure_wire::LeakAlarm) — safety acts on
//! the sensor crate's word alone.

/// Safety tick period, seconds (1 kHz).
pub const TICK_S: f64 = 0.001;

/// The reflex action output: what the actuator layer must do now.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EmergencyAction {
    /// Nothing (no alarm).
    None,
    /// Release the drop weight immediately (t=0 of the reflex).
    DropWeight,
    /// Weight is away; blow the ballast bladder (t≥50 ms).
    BlowBallast,
    /// Buoyant and ascending; keep the bladder open and log.
    Ascend,
}

/// A leak alarm as delivered over the wire (field-compatible with the
/// pressure crate's `wire::LeakAlarm`; duplicated here as a plain struct
/// so the safety crate needs no dependency edge on the sensor crate —
/// the wire record is the contract).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeakAlarmMsg {
    /// Compartment index.
    pub compartment: u8,
    /// Cause discriminator (1 = PressureRise, 2 = RapidRise).
    pub cause: u16,
    /// Alarm timestamp, microseconds.
    pub timestamp_us: u64,
}

/// Flooding reflex state (`Copy`, no allocation).
#[derive(Debug, Clone, Copy)]
pub struct FloodingReflex {
    armed: bool,
    alarm_us: u64,
    stage_ms: u32,
}

impl Default for FloodingReflex {
    fn default() -> Self {
        Self::new()
    }
}

impl FloodingReflex {
    /// Create an unarmed reflex.
    pub const fn new() -> Self {
        Self {
            armed: false,
            alarm_us: 0,
            stage_ms: 0,
        }
    }

    /// Whether the reflex is armed (a leak has been seen).
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Feed a latched leak alarm (idempotent: the pressure crate keeps the
    /// alarm latched, we keep the reflex armed).
    pub fn on_leak(&mut self, alarm: &LeakAlarmMsg, now_us: u64) {
        if !self.armed {
            self.armed = true;
            self.alarm_us = alarm.timestamp_us.max(now_us);
            self.stage_ms = 0;
        }
    }

    /// Advance one 1 kHz safety tick; returns the action for this tick.
    pub fn tick(&mut self, now_us: u64) -> EmergencyAction {
        if !self.armed {
            return EmergencyAction::None;
        }
        let elapsed_ms = ((now_us.saturating_sub(self.alarm_us)) / 1000) as u32;
        self.stage_ms = elapsed_ms;
        if elapsed_ms < 50 {
            EmergencyAction::DropWeight
        } else if elapsed_ms < 2_000 {
            EmergencyAction::BlowBallast
        } else {
            EmergencyAction::Ascend
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drop_weight_within_the_reflex_budget() {
        let mut r = FloodingReflex::new();
        let t0 = 1_000_000u64; // 1 s into the mission
        r.on_leak(
            &LeakAlarmMsg {
                compartment: 2,
                cause: 1,
                timestamp_us: t0,
            },
            t0,
        );
        // The very next 1 kHz tick: drop the weight. Detection (pressure
        // crate, ≤90 ms at 100 Hz) + this tick is inside the spec's
        // <100 ms *sensor-to-action* window when driven by the
        // crate's alarm (its tests prove ≤90 ms detection).
        let a = r.tick(t0 + 1_000); // +1 ms
        assert_eq!(a, EmergencyAction::DropWeight);
        assert_eq!(r.tick(t0 + 49_000), EmergencyAction::DropWeight);
    }

    #[test]
    fn ballast_blows_after_the_weight_is_away() {
        let mut r = FloodingReflex::new();
        let t0 = 0u64;
        r.on_leak(
            &LeakAlarmMsg {
                compartment: 0,
                cause: 2,
                timestamp_us: t0,
            },
            t0,
        );
        assert_eq!(r.tick(50_000), EmergencyAction::BlowBallast);
        assert_eq!(r.tick(1_999_999), EmergencyAction::BlowBallast);
        assert_eq!(r.tick(2_000_000), EmergencyAction::Ascend);
    }

    #[test]
    fn unarmed_reflex_is_inert() {
        let mut r = FloodingReflex::new();
        assert_eq!(r.tick(10_000_000), EmergencyAction::None);
        assert!(!r.armed());
    }
}
