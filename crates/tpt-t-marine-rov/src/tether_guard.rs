//! Tether-aware motion guarding (spec §5.2): "The ROV will not move in a
//! direction that would increase tether snag risk, even if the pilot
//! commands it. The pilot receives haptic feedback through the joystick
//! when approaching tether limits."
//!
//! The guard consumes the tether crate's [`Prediction`](tpt_t_marine_tether::
//! predict::Prediction) and filters the pilot's body command: at `Caution`
//! the command scales down toward the risk-reducing directions; at `Warn`
//! it is refused outright (replaced by a stop) and the haptics ramp.

use tpt_t_marine_core::wire::ThrusterCmd6;
use tpt_t_marine_tether::predict::SnagLevel;

/// Haptic feedback level for the pilot's joystick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Haptic {
    /// Silent.
    Off = 0,
    /// Growing resistance (approaching a tether limit).
    Resistance = 1,
    /// Hard stop buzz (command refused).
    Stop = 2,
}

/// The guard.
#[derive(Debug, Clone, Copy)]
pub struct TetherGuard {
    /// Command scale retained at `Caution` (between 0 and 1).
    pub caution_scale: f32,
    last_haptic: Haptic,
}

impl Default for TetherGuard {
    fn default() -> Self {
        Self {
            caution_scale: 0.4,
            last_haptic: Haptic::Off,
        }
    }
}

impl TetherGuard {
    /// Create with a caution scale.
    pub fn new(caution_scale: f32) -> Self {
        Self {
            caution_scale,
            last_haptic: Haptic::Off,
        }
    }

    /// Haptic level the pilot currently feels.
    pub fn haptic(&self) -> Haptic {
        self.last_haptic
    }

    /// Filter one pilot command against the current snag level.
    ///
    /// * `Clear` — pass through, haptics off.
    /// * `Caution` — scale the command down (the vehicle still answers the
    ///   pilot but feels "sticky"), haptic resistance.
    /// * `Warn` — refuse: the vehicle holds station regardless of the
    ///   command, haptic stop.
    pub fn filter(&mut self, cmd: ThrusterCmd6, level: SnagLevel) -> (ThrusterCmd6, bool) {
        match level {
            SnagLevel::Clear => {
                self.last_haptic = Haptic::Off;
                (cmd, true)
            }
            SnagLevel::Caution => {
                self.last_haptic = Haptic::Resistance;
                (
                    ThrusterCmd6 {
                        surge: cmd.surge * self.caution_scale,
                        sway: cmd.sway * self.caution_scale,
                        heave: cmd.heave * self.caution_scale,
                        roll: cmd.roll,
                        pitch: cmd.pitch,
                        yaw: cmd.yaw * self.caution_scale,
                    },
                    true,
                )
            }
            SnagLevel::Warn => {
                self.last_haptic = Haptic::Stop;
                (ThrusterCmd6::default(), false)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn full() -> ThrusterCmd6 {
        ThrusterCmd6 {
            surge: 1.0,
            sway: 1.0,
            heave: 1.0,
            roll: 1.0,
            pitch: 1.0,
            yaw: 1.0,
        }
    }

    #[test]
    fn clear_passes_through_silently() {
        let mut g = TetherGuard::default();
        let (cmd, ok) = g.filter(full(), SnagLevel::Clear);
        assert!(ok);
        assert_eq!(cmd, full());
        assert_eq!(g.haptic(), Haptic::Off);
    }

    #[test]
    fn caution_scales_translation_but_keeps_attitude() {
        let mut g = TetherGuard::new(0.4);
        let (cmd, ok) = g.filter(full(), SnagLevel::Caution);
        assert!(ok);
        assert!((cmd.surge - 0.4).abs() < 1e-6);
        assert!((cmd.roll - 1.0).abs() < 1e-6, "attitude authority kept");
        assert_eq!(g.haptic(), Haptic::Resistance);
    }

    #[test]
    fn warn_refuses_and_buzzes() {
        let mut g = TetherGuard::default();
        let (cmd, ok) = g.filter(full(), SnagLevel::Warn);
        assert!(!ok, "command refused");
        assert!(cmd.is_zero(), "vehicle holds station");
        assert_eq!(g.haptic(), Haptic::Stop);
    }
}
