//! Comms-triggered behaviors (spec §5.1): on surfacing, the AUV
//! establishes a satellite link, transmits a compressed summary, and
//! accepts new waypoints or an abort before the next dive.

/// The surface window FSM.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfacePhase {
    /// Submerged; nothing to do.
    Diving,
    /// Just surfaced: raise the antenna, acquire the satellite.
    Acquiring,
    /// Uplink established: transmit the summary.
    Transmitting,
    /// Summary sent; listening for new waypoints / abort until the window
    /// closes.
    Listening,
    /// Window closed: dive again (or stay put if new orders arrived).
    Complete,
}

/// The surfacing behavior driver.
#[derive(Debug, Clone, Copy)]
pub struct SurfacingBehavior {
    /// Window durations, seconds.
    /// Seconds to wait for the satellite before diving empty.
    pub acquire_timeout_s: u32,
    /// Seconds to listen for orders after the summary.
    pub listen_window_s: u32,
    phase: SurfacePhase,
    in_phase_s: u32,
    /// Whether new orders (waypoints/abort) arrived this window.
    pub orders_received: bool,
    /// Whether the summary went out this window.
    pub summary_sent: bool,
}

impl SurfacingBehavior {
    /// Create with window parameters.
    pub fn new(acquire_timeout_s: u32, listen_window_s: u32) -> Self {
        Self {
            acquire_timeout_s,
            listen_window_s,
            phase: SurfacePhase::Diving,
            in_phase_s: 0,
            orders_received: false,
            summary_sent: false,
        }
    }

    /// Current phase.
    pub fn phase(&self) -> SurfacePhase {
        self.phase
    }

    /// The vehicle breaks the surface: begin the window.
    pub fn on_surface(&mut self) {
        self.phase = SurfacePhase::Acquiring;
        self.in_phase_s = 0;
        self.orders_received = false;
        self.summary_sent = false;
    }

    /// Feed one downlink frame (new waypoints or abort).
    pub fn on_downlink(&mut self, has_orders: bool) {
        if self.phase == SurfacePhase::Listening || self.phase == SurfacePhase::Transmitting {
            self.orders_received |= has_orders;
        }
    }

    /// Advance one second of surface time; returns the current phase.
    pub fn tick(&mut self, link_up: bool) -> SurfacePhase {
        self.in_phase_s += 1;
        match self.phase {
            SurfacePhase::Acquiring => {
                if link_up {
                    self.phase = SurfacePhase::Transmitting;
                    self.in_phase_s = 0;
                } else if self.in_phase_s >= self.acquire_timeout_s {
                    // No satellite overhead: dive empty-handed.
                    self.phase = SurfacePhase::Complete;
                }
            }
            SurfacePhase::Transmitting => {
                // The compressed summary is one short burst.
                self.summary_sent = true;
                self.phase = SurfacePhase::Listening;
                self.in_phase_s = 0;
            }
            SurfacePhase::Listening => {
                let done = self.orders_received || self.in_phase_s >= self.listen_window_s;
                if done {
                    self.phase = SurfacePhase::Complete;
                }
            }
            _ => {}
        }
        self.phase
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_surface_window_flows() {
        let mut b = SurfacingBehavior::new(30, 120);
        b.on_surface();
        assert_eq!(b.phase(), SurfacePhase::Acquiring);
        // Satellite found after 5 s.
        for _ in 0..5 {
            b.tick(false);
        }
        assert_eq!(b.tick(true), SurfacePhase::Transmitting);
        // Summary is a one-tick burst, then listen.
        assert_eq!(b.tick(true), SurfacePhase::Listening);
        assert!(b.summary_sent);
        // Orders arrive mid-listen: window closes immediately.
        b.on_downlink(true);
        assert_eq!(b.tick(true), SurfacePhase::Complete);
        assert!(b.orders_received);
    }

    #[test]
    fn no_satellite_dives_empty_after_timeout() {
        let mut b = SurfacingBehavior::new(10, 120);
        b.on_surface();
        for _ in 0..10 {
            b.tick(false);
        }
        assert_eq!(b.phase(), SurfacePhase::Complete);
        assert!(!b.summary_sent);
    }

    #[test]
    fn listen_window_closes_without_orders() {
        let mut b = SurfacingBehavior::new(5, 20);
        b.on_surface();
        for _ in 0..5 {
            b.tick(true);
        }
        // Transmitting → Listening → burn the 20 s window.
        b.tick(true);
        for _ in 0..20 {
            b.tick(true);
        }
        assert_eq!(b.phase(), SurfacePhase::Complete);
        assert!(!b.orders_received);
    }
}
