//! The sawtooth flight controller.
//!
//! A glider "flies" by making itself heavier than displaced water
//! (negative net buoyancy → glide down, nose-first at a target pitch),
//! then lighter (positive → glide up). The battery pack slides fore/aft
//! to set the glide pitch; the rudder steers. Turning points are depth-
//! or time-triggered.

/// Profile segment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Segment {
    /// Negative buoyancy, nose-down glide.
    Dive,
    /// Positive buoyancy, nose-up glide.
    Rise,
}

/// Glider flight configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightConfig {
    /// Target glide pitch while diving, radians (positive nose-up; diving
    /// pitch is negative).
    pub dive_pitch_rad: f32,
    /// Target glide pitch while rising, radians.
    pub rise_pitch_rad: f32,
    /// Depth turning point, metres.
    pub apogee_depth_m: f32,
    /// Surface turning point: shallower than this, rise → dive cycle
    /// tops out, metres.
    pub surface_depth_m: f32,
    /// Pump rate for buoyancy change, normalized units/s.
    pub pump_rate: f32,
    /// Battery-pack travel per radian of pitch correction.
    pub pack_gain: f32,
}

// `vy_mps` (vertical rate) is part of the control signature for future
// total-energy controllers; v1 gliders fly pitch+bubbles only.

impl FlightConfig {
    /// A 1000 m-class survey glider.
    pub const fn deep_profile() -> Self {
        Self {
            dive_pitch_rad: -0.45, // ≈ 26° nose-down
            rise_pitch_rad: 0.45,
            apogee_depth_m: 1000.0,
            surface_depth_m: 5.0,
            pump_rate: 0.2,
            pack_gain: 2.0,
        }
    }
}

/// Actuator outputs for one control tick.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GliderActuators {
    /// Pump command, `[-1, 1]` (negative = flood (heavier), positive =
    /// blow (lighter)).
    pub pump: f32,
    /// Battery pack position, `[-1, 1]` (negative = aft… nose-down).
    pub pack: f32,
    /// Rudder, `[-1, 1]`.
    pub rudder: f32,
}

/// The flight controller.
#[derive(Debug, Clone, Copy)]
pub struct GliderFlight {
    /// Flight configuration.
    pub cfg: FlightConfig,
    /// Current profile segment.
    pub segment: Segment,
    /// Heading setpoint, radians (compass convention via yaw).
    pub heading_setpoint_rad: f32,
}

impl GliderFlight {
    /// Create on the surface, starting a dive toward `heading`.
    pub fn new(cfg: FlightConfig, heading_rad: f32) -> Self {
        Self {
            cfg,
            segment: Segment::Dive,
            heading_setpoint_rad: heading_rad,
        }
    }

    /// Current segment.
    pub fn segment(&self) -> Segment {
        self.segment
    }

    /// One control tick: `depth` metres positive-down, `pitch` radians
    /// positive-nose-up, `vy_mps` vertical rate m/s (positive down;
    /// reserved for total-energy control in v2), `yaw_rad` heading.
    pub fn update(
        &mut self,
        depth_m: f32,
        pitch_rad: f32,
        vy_mps: f32,
        yaw_rad: f32,
    ) -> GliderActuators {
        let _ = vy_mps;
        // Turning points.
        match self.segment {
            Segment::Dive if depth_m >= self.cfg.apogee_depth_m => {
                self.segment = Segment::Rise;
            }
            Segment::Rise if depth_m <= self.cfg.surface_depth_m => {
                self.segment = Segment::Dive;
            }
            _ => {}
        }

        let (pump, pack_target) = match self.segment {
            // Diving: pump in water (negative = heavier), pack forward
            // (nose down).
            Segment::Dive => (
                -self.cfg.pump_rate,
                self.cfg.pack_gain * self.cfg.dive_pitch_rad,
            ),
            // Rising: blow (positive = lighter), pack aft.
            Segment::Rise => (
                self.cfg.pump_rate,
                self.cfg.pack_gain * self.cfg.rise_pitch_rad,
            ),
        };

        // Pitch hold via the pack: correct proportional to the pitch error.
        let target = match self.segment {
            Segment::Dive => self.cfg.dive_pitch_rad,
            Segment::Rise => self.cfg.rise_pitch_rad,
        };
        let pitch_err = target - pitch_rad;
        let pack = (pack_target + self.cfg.pack_gain * pitch_err).clamp(-1.0, 1.0);

        // Rudder: heading hold (compass wrap).
        let mut herr = self.heading_setpoint_rad - yaw_rad;
        while herr > core::f32::consts::PI {
            herr -= 2.0 * core::f32::consts::PI;
        }
        while herr < -core::f32::consts::PI {
            herr += 2.0 * core::f32::consts::PI;
        }
        let rudder = herr.clamp(-1.0, 1.0);

        GliderActuators { pump, pack, rudder }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> FlightConfig {
        FlightConfig::deep_profile()
    }

    #[test]
    fn dive_then_turn_at_apogee() {
        let mut f = GliderFlight::new(cfg(), 0.0);
        let a = f.update(500.0, cfg().dive_pitch_rad, 0.25, 0.0);
        assert_eq!(f.segment(), Segment::Dive);
        assert!(a.pump < 0.0, "flood to dive");
        assert!(a.pack < 0.0, "pack forward for nose-down");
        // Apogee.
        let a = f.update(1000.0, cfg().dive_pitch_rad, 0.25, 0.0);
        assert_eq!(f.segment(), Segment::Rise);
        assert!(a.pump > 0.0, "blow to rise");
        assert!(a.pack > 0.0, "pack aft for nose-up");
    }

    #[test]
    fn rise_turns_back_at_the_surface() {
        let mut f = GliderFlight::new(cfg(), 0.0);
        f.segment = Segment::Rise;
        let _ = f.update(50.0, cfg().rise_pitch_rad, -0.25, 0.0);
        assert_eq!(f.segment(), Segment::Rise, "still deep enough");
        let _ = f.update(3.0, cfg().rise_pitch_rad, -0.25, 0.0);
        assert_eq!(f.segment(), Segment::Dive, "surface turn");
    }

    #[test]
    fn pitch_error_trims_the_pack() {
        let mut f = GliderFlight::new(cfg(), 0.0);
        // Nose too high while diving: pack must move further forward.
        let a = f.update(200.0, cfg().dive_pitch_rad + 0.1, 0.2, 0.0);
        let a_nominal = f.update(200.0, cfg().dive_pitch_rad, 0.2, 0.0);
        assert!(
            a.pack < a_nominal.pack,
            "trim forward: {} vs {}",
            a.pack,
            a_nominal.pack
        );
    }

    #[test]
    fn rudder_holds_heading() {
        let mut f = GliderFlight::new(cfg(), 1.0);
        let a = f.update(100.0, cfg().dive_pitch_rad, 0.2, 0.6);
        assert!(a.rudder > 0.05, "steer right toward 1.0 rad: {}", a.rudder);
        let a = f.update(100.0, cfg().dive_pitch_rad, 0.2, 1.4);
        assert!(a.rudder < -0.05, "steer left back to 1.0 rad: {}", a.rudder);
    }
}
