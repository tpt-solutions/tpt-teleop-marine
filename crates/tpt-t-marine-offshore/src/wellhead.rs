//! Oil & gas wellhead monitoring: standoff-protected approach orbit with
//! anomaly station-keeping.
//!
//! Safety first: the vehicle never approaches closer than the standoff
//! radius while monitoring; anomaly re-approach requires an explicit
//! permit that shrinks (never removes) the standoff.

/// Monitoring state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MonitorPhase {
    /// Hold at the monitoring orbit.
    Orbit,
    /// Station-keeping on an anomaly for detailed inspection.
    AnomalyHold,
    /// Returning to the orbit after an anomaly hold.
    Rejoin,
}

/// Wellhead monitor configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WellheadMonitor {
    /// Wellhead position, NED metres.
    pub centre_m: [f64; 3],
    /// Safety standoff, metres — never crossed.
    pub standoff_m: f64,
    /// Monitoring orbit radius (≥ standoff + margin).
    pub orbit_m: f64,
    phase: MonitorPhase,
}

impl WellheadMonitor {
    /// Create; `orbit_m` must exceed `standoff_m`.
    pub fn new(centre_m: [f64; 3], standoff_m: f64, orbit_m: f64) -> Self {
        assert!(orbit_m > standoff_m, "orbit must exceed standoff");
        Self {
            centre_m,
            standoff_m,
            orbit_m,
            phase: MonitorPhase::Orbit,
        }
    }

    /// Current phase.
    pub fn phase(&self) -> MonitorPhase {
        self.phase
    }

    /// Clamp a commanded inspection position so the vehicle never enters
    /// the standoff sphere. Returns the (possibly pulled-back) target and
    /// whether it was modified.
    pub fn clamp_target(&self, target: [f64; 3]) -> ([f64; 3], bool) {
        let dx = target[0] - self.centre_m[0];
        let dy = target[1] - self.centre_m[1];
        let dz = target[2] - self.centre_m[2];
        let d = (dx * dx + dy * dy + dz * dz).sqrt();
        if d >= self.standoff_m {
            return (target, false);
        }
        let k = self.standoff_m / d.max(1e-9);
        (
            [
                self.centre_m[0] + dx * k,
                self.centre_m[1] + dy * k,
                self.centre_m[2] + dz * k,
            ],
            true,
        )
    }

    /// Begin an anomaly hold (moves toward the clamped anomaly position).
    pub fn begin_anomaly_hold(&mut self) {
        self.phase = MonitorPhase::AnomalyHold;
    }

    /// Finish the anomaly hold: rejoin the orbit.
    pub fn finish_anomaly_hold(&mut self) {
        if self.phase == MonitorPhase::AnomalyHold {
            self.phase = MonitorPhase::Rejoin;
        }
    }

    /// Arrived back on the orbit.
    pub fn on_orbit(&mut self) {
        if self.phase == MonitorPhase::Rejoin {
            self.phase = MonitorPhase::Orbit;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn monitor() -> WellheadMonitor {
        WellheadMonitor::new([0.0, 0.0, 80.0], 10.0, 25.0)
    }

    #[test]
    fn standoff_sphere_is_hard() {
        let m = monitor();
        // Commanded to 4 m from the centre: clamped back to 10 m.
        let (t, clamped) = m.clamp_target([3.0, 2.0, 81.0]);
        assert!(clamped);
        let d = ((t[0]).powi(2) + (t[1]).powi(2) + (t[2] - 80.0).powi(2)).sqrt();
        assert!((d - 10.0).abs() < 1e-9, "clamped distance {d}");
        // Well outside: untouched.
        let (t2, c2) = m.clamp_target([25.0, 0.0, 80.0]);
        assert!(!c2);
        assert_eq!(t2, [25.0, 0.0, 80.0]);
    }

    #[test]
    fn anomaly_hold_cycles_through_rejoin() {
        let mut m = monitor();
        assert_eq!(m.phase(), MonitorPhase::Orbit);
        m.begin_anomaly_hold();
        assert_eq!(m.phase(), MonitorPhase::AnomalyHold);
        m.finish_anomaly_hold();
        assert_eq!(m.phase(), MonitorPhase::Rejoin);
        m.on_orbit();
        assert_eq!(m.phase(), MonitorPhase::Orbit);
    }
}
