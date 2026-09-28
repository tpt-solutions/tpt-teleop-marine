//! Target-track vocabulary and geometric predicates.
//!
//! Angles are compass degrees (0 = north, clockwise). "Relative bearing"
//! is the direction of the target from *own* bow; "aspect" is the
//! direction of own vessel from the *target's* bow — the two numbers that
//! classify every encounter.

use core::fmt;

/// Normalize a compass angle into `[0, 360)`.
pub fn norm360(deg: f64) -> f64 {
    let d = deg % 360.0;
    if d < 0.0 { d + 360.0 } else { d }
}

/// Smallest signed difference `a - b` in `[-180, 180)`.
pub fn ang_diff(a: f64, b: f64) -> f64 {
    let d = norm360(a - b);
    if d >= 180.0 { d - 360.0 } else { d }
}

/// One tracked vessel (radar/AIS fused track).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TargetTrack {
    /// Track id (AIS MMSI low bits or radar track number).
    pub id: u16,
    /// Bearing of the target from own vessel, degrees true.
    pub bearing_deg: f64,
    /// Range, metres.
    pub range_m: f64,
    /// Target course over ground, degrees true.
    pub course_deg: f64,
    /// Target speed over ground, m/s.
    pub speed_mps: f64,
}

/// Own-vehicle state for the tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OwnState {
    /// Own course over ground, degrees true.
    pub course_deg: f64,
    /// Own speed over ground, m/s.
    pub speed_mps: f64,
}

impl TargetTrack {
    /// Relative bearing of the target off own bow, `[−180, 180)`, positive
    /// to starboard (Rule 15's "starboard side" test).
    pub fn relative_bearing(&self) -> f64 {
        ang_diff(self.bearing_deg, 0.0)
    }

    /// Aspect: the relative bearing of *own* vessel off the *target's*
    /// bow, `[−180, 180)`, positive to the target's starboard.
    pub fn aspect(&self, own: &OwnState) -> f64 {
        ang_diff(own.course_deg + 180.0, self.course_deg)
    }

    /// Whether the encounter is overtaking from the *target's* point of
    /// view: own is overtaking the target when approaching from more than
    /// 112.5° abaft the target's beam (Rule 13).
    pub fn own_is_overtaking(&self, own: &OwnState) -> bool {
        self.aspect(own).abs() > 112.5 && own.speed_mps > self.speed_mps
    }

    /// Whether the *target* is overtaking own vessel (Rule 13, other side).
    pub fn target_is_overtaking(&self, own: &OwnState) -> bool {
        self.relative_bearing().abs() > 112.5 && self.speed_mps > own.speed_mps
    }

    /// Head-on geometry (Rule 14): reciprocal-ish courses, target near own
    /// bow line and own near target's bow line (±22.5° windows).
    pub fn is_head_on(&self, own: &OwnState) -> bool {
        let reciprocal = ang_diff(self.course_deg, own.course_deg).abs() > 157.5;
        let near_bow = self.relative_bearing().abs() < 22.5;
        let near_their_bow = self.aspect(own).abs() < 22.5;
        reciprocal && near_bow && near_their_bow
    }
}

/// Why the encounter is risky — the Rule 7 test: constant bearing,
/// decreasing range.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cbdr {
    /// Previous bearing, degrees.
    pub prev_bearing_deg: f64,
    /// Previous range, metres.
    pub prev_range_m: f64,
}

impl Cbdr {
    /// Risk exists when the bearing is (approximately) constant while the
    /// range decreases. `bearing_rate_deg_per_min` under ±0.3°/min with
    /// positive closure is the classic threshold.
    pub fn risk(&self, now: &TargetTrack, dt_s: f64) -> bool {
        if dt_s <= 0.0 {
            return false;
        }
        let closure = (self.prev_range_m - now.range_m) / dt_s; // m/s, + = closing
        if closure <= 0.0 {
            return false;
        }
        let bearing_rate = ang_diff(now.bearing_deg, self.prev_bearing_deg).abs() / (dt_s / 60.0);
        bearing_rate < 0.3
    }
}

/// Human-readable bearing quadrant (for the decision log).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Quadrant {
    /// Ahead (|rb| ≤ 22.5°).
    Ahead,
    /// Starboard bow / beam (22.5° < rb ≤ 112.5°).
    Starboard,
    /// Astern (112.5° < rb).
    Astern,
    /// Port bow / beam (−112.5° > rb ≥ −22.5° mirrored).
    Port,
}

impl fmt::Display for Quadrant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            Quadrant::Ahead => "AHEAD",
            Quadrant::Starboard => "STARBOARD",
            Quadrant::Astern => "ASTERN",
            Quadrant::Port => "PORT",
        };
        f.write_str(name)
    }
}

/// Quadrant of a relative bearing.
pub fn quadrant_of(rb: f64) -> Quadrant {
    let r = ang_diff(rb, 0.0);
    if r.abs() <= 22.5 {
        Quadrant::Ahead
    } else if r.abs() > 112.5 {
        Quadrant::Astern
    } else if r > 0.0 {
        Quadrant::Starboard
    } else {
        Quadrant::Port
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(bearing: f64, course: f64, speed: f64) -> TargetTrack {
        TargetTrack {
            id: 1,
            bearing_deg: bearing,
            range_m: 1000.0,
            course_deg: course,
            speed_mps: speed,
        }
    }

    fn own(course: f64, speed: f64) -> OwnState {
        OwnState {
            course_deg: course,
            speed_mps: speed,
        }
    }

    #[test]
    fn relative_bearing_and_aspect_round_trip() {
        // Target dead ahead: rb = 0.
        assert!(t(0.0, 90.0, 0.0).relative_bearing().abs() < 1e-9);
        // Target on the starboard beam while own heads north: rb = 90.
        assert!((t(90.0, 0.0, 0.0).relative_bearing() - 90.0).abs() < 1e-9);
        // Own heads north; target sits 1 km north of us heading south —
        // from the target's bow, we are dead ahead (aspect 0).
        let target = t(0.0, 180.0, 0.0);
        assert!(target.aspect(&own(0.0, 0.0)).abs() < 1e-9);
    }

    #[test]
    fn head_on_detection() {
        let o = own(0.0, 3.0);
        let target = t(359.5, 180.0, 3.0); // dead ahead, reciprocal
        assert!(target.is_head_on(&o));
        // Same track, same direction: not head-on (that's overtaking).
        let following = t(0.0, 0.0, 1.0);
        assert!(!following.is_head_on(&o));
    }

    #[test]
    fn overtaking_geometry() {
        // Own (5 m/s) coming up behind a slower target (2 m/s) on the same
        // course: aspect from target's bow ≈ 180 → own overtaking.
        let o = own(45.0, 5.0);
        let slower = t(45.0, 45.0, 2.0);
        assert!(slower.own_is_overtaking(&o));
        // And we are not being overtaken.
        assert!(!slower.target_is_overtaking(&o));
    }

    #[test]
    fn cbdr_risk_needs_closure_and_steady_bearing() {
        let o = own(0.0, 3.0);
        let prev = t(30.0, 180.0, 3.0);
        let mut now = prev;
        now.bearing_deg = 30.05; // ~3°/min at 1 s? No: 0.05° per second
        // 0.05°/s = 3°/min — too fast for CBDR.
        assert!(
            !Cbdr {
                prev_bearing_deg: prev.bearing_deg,
                prev_range_m: 2000.0
            }
            .risk(&now, 1.0)
        );
        // A genuinely steady bearing with closure.
        let now2 = t(30.002, 180.0, 3.0);
        assert!(
            Cbdr {
                prev_bearing_deg: 30.0,
                prev_range_m: 2000.0
            }
            .risk(&now2, 1.0)
        );
        // Opening range: no risk regardless.
        let opening = t(30.0, 180.0, 3.0);
        assert!(
            !Cbdr {
                prev_bearing_deg: 30.0,
                prev_range_m: 1000.0
            }
            .risk(&opening, 1.0)
        );
        let _ = o;
    }

    #[test]
    fn quadrants() {
        assert_eq!(quadrant_of(0.0), Quadrant::Ahead);
        assert_eq!(quadrant_of(90.0), Quadrant::Starboard);
        assert_eq!(quadrant_of(-90.0), Quadrant::Port);
        assert_eq!(quadrant_of(150.0), Quadrant::Astern);
    }
}
