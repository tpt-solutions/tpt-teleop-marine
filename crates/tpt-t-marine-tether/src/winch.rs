//! Tension monitoring and winch (TMS) control.

/// Winch configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WinchConfig {
    /// Tension window the TMS holds, newtons (min, max).
    /// Minimum window tension, N.
    pub tension_min_n: f64,
    /// Maximum window tension, N.
    pub tension_max_n: f64,
    /// Winch speed, metres/second at full command (pay-in positive).
    pub speed_mps: f64,
    /// Payed-out tether limits, metres.
    /// Minimum paid-out tether, metres.
    pub min_out_m: f64,
    /// Maximum paid-out tether, metres.
    pub max_out_m: f64,
}

impl WinchConfig {
    /// Typical TMS garage winch.
    pub const fn typical() -> Self {
        Self {
            tension_min_n: 500.0,
            tension_max_n: 4000.0,
            speed_mps: 1.0,
            min_out_m: 5.0,
            max_out_m: 500.0,
        }
    }
}

/// Winch state and controller.
#[derive(Debug, Clone, Copy)]
#[allow(missing_docs)]
pub struct Winch {
    cfg: WinchConfig,
    /// Payed-out tether, metres.
    pub out_m: f64,
    /// Proportional gain, fraction per newton of window error.
    kp: f64,
}

impl Winch {
    /// Create with `out_m` paid out.
    pub fn new(cfg: WinchConfig, out_m: f64) -> Self {
        Self {
            cfg,
            out_m,
            kp: 1.0 / 500.0,
        }
    }

    /// Advance one control tick: measure tension, command the winch, then
    /// integrate the payout within the hard limits. Returns the commanded
    /// rate (positive = pay in).
    pub fn update(&mut self, tension_n: f64, dt_s: f64) -> f64 {
        let err = if tension_n < self.cfg.tension_min_n {
            // Too slack: pay in.
            self.cfg.tension_min_n - tension_n
        } else if tension_n > self.cfg.tension_max_n {
            // Too taut: pay out.
            self.cfg.tension_max_n - tension_n // negative
        } else {
            0.0
        };
        let rate = (self.kp * err).clamp(-1.0, 1.0);
        self.out_m = (self.out_m - rate * self.cfg.speed_mps * dt_s)
            .clamp(self.cfg.min_out_m, self.cfg.max_out_m);
        rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slack_tension_pays_in() {
        let mut w = Winch::new(WinchConfig::typical(), 100.0);
        let rate = w.update(300.0, 0.1);
        assert!(rate > 0.0, "pay in when slack: {rate}");
        assert!(w.out_m < 100.0);
    }

    #[test]
    fn taut_tension_pays_out() {
        let mut w = Winch::new(WinchConfig::typical(), 100.0);
        let rate = w.update(5000.0, 0.1);
        assert!(rate < 0.0, "pay out when taut");
        assert!(w.out_m > 100.0);
    }

    #[test]
    fn in_window_holds_still() {
        let mut w = Winch::new(WinchConfig::typical(), 100.0);
        let rate = w.update(2000.0, 0.1);
        assert_eq!(rate, 0.0);
        assert_eq!(w.out_m, 100.0);
    }

    #[test]
    fn payout_limits_are_hard() {
        let mut w = Winch::new(WinchConfig::typical(), 499.0);
        // Endless tautness cannot pay out past 500 m.
        for _ in 0..100 {
            w.update(9000.0, 0.1);
        }
        assert_eq!(w.out_m, 500.0);
        let mut w2 = Winch::new(WinchConfig::typical(), 6.0);
        for _ in 0..100 {
            w2.update(10.0, 0.1);
        }
        assert_eq!(w2.out_m, 5.0);
    }
}
