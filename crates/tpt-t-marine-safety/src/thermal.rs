//! Thermal escalation: derate → vent → latched shutdown.

/// Thermal limits for one vehicle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalLimits {
    /// Above this, derate propulsion, °C.
    pub derate_c: f32,
    /// Above this, vent electronics bay and hard-derate, °C.
    pub vent_c: f32,
    /// Latching shutdown: only surface recovery resets, °C.
    pub shutdown_c: f32,
}

impl ThermalLimits {
    /// Survey-AUV envelope (electronics bay).
    pub const fn typical() -> Self {
        Self {
            derate_c: 65.0,
            vent_c: 75.0,
            shutdown_c: 85.0,
        }
    }
}

/// Thermal escalation state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThermalState {
    /// Normal operations.
    Nominal,
    /// Propulsion derated (power drawn down).
    Derated,
    /// Venting + hard derate.
    Venting,
    /// Latched shutdown (hysteresis: stays until reset).
    Shutdown,
}

/// Thermal monitor (`Copy`).
#[derive(Debug, Clone, Copy)]
pub struct ThermalMonitor {
    limits: ThermalLimits,
    state: ThermalState,
}

impl ThermalMonitor {
    /// Create with limits, nominal state.
    pub fn new(limits: ThermalLimits) -> Self {
        Self {
            limits,
            state: ThermalState::Nominal,
        }
    }

    /// Current state.
    pub fn state(&self) -> ThermalState {
        self.state
    }

    /// Feed one temperature sample; returns the (possibly new) state.
    /// Cooling hysteresis: recovery requires dropping 5 °C below the band.
    pub fn update(&mut self, temp_c: f32) -> ThermalState {
        use ThermalState::*;
        self.state = match (self.state, temp_c) {
            (Shutdown, _) => Shutdown, // latched
            (_, t) if t >= self.limits.shutdown_c => Shutdown,
            (Venting, t) if t >= self.limits.vent_c - 5.0 => Venting,
            (Derated, t) if t >= self.limits.derate_c - 5.0 && t < self.limits.vent_c => Derated,
            (_, t) if t >= self.limits.vent_c => Venting,
            (_, t) if t >= self.limits.derate_c => Derated,
            _ => Nominal,
        };
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escalates_and_recovers_with_hysteresis() {
        let mut m = ThermalMonitor::new(ThermalLimits::typical());
        assert_eq!(m.update(40.0), ThermalState::Nominal);
        assert_eq!(m.update(70.0), ThermalState::Derated);
        assert_eq!(m.update(78.0), ThermalState::Venting);
        // Cooling: still venting at 72 (needs < 70).
        assert_eq!(m.update(72.0), ThermalState::Venting);
        // Below the derate band − hysteresis: back to nominal.
        assert_eq!(m.update(55.0), ThermalState::Nominal);
    }

    #[test]
    fn shutdown_latches() {
        let mut m = ThermalMonitor::new(ThermalLimits::typical());
        assert_eq!(m.update(90.0), ThermalState::Shutdown);
        // Even arctic water cannot unlatch a thermal shutdown.
        assert_eq!(m.update(2.0), ThermalState::Shutdown);
    }
}
