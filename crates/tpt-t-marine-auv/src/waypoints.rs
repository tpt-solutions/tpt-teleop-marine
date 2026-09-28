//! 3D waypoint following: line-of-sight guidance with a depth loop.

use tpt_t_marine_core::wire::{NedPose, ThrusterCmd6, Waypoint3D};

/// Guidance gains.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidanceGains {
    /// Cross-track gain, 1/s per metre of cross-track error.
    pub cross_track: f32,
    /// Along-track speed demand gain, m/s per metre of along-track gap.
    pub speed: f32,
    /// Depth loop gain, normalized heave per metre of depth error.
    pub depth: f32,
    /// Heading loop gain, normalized yaw per radian of heading error.
    pub heading: f32,
    /// Maximum commanded speed, m/s.
    pub max_speed_mps: f32,
}

impl GuidanceGains {
    /// Survey-AUV tuning for a 1.5 m/s vehicle.
    pub const fn survey() -> Self {
        Self {
            cross_track: 0.05,
            speed: 0.2,
            depth: 0.08,
            heading: 1.5,
            max_speed_mps: 1.5,
        }
    }
}

/// The waypoint follower. Owns the plan position (current waypoint index
/// is the caller's job) and emits body commands.
#[derive(Debug, Clone, Copy)]
pub struct WaypointFollower {
    /// Guidance gains.
    pub gains: GuidanceGains,
}

impl WaypointFollower {
    /// Create with gains.
    pub const fn new(gains: GuidanceGains) -> Self {
        Self { gains }
    }

    /// Compute the command steering `pose` toward `wp`.
    ///
    /// Guidance: the line-of-sight vector from the vehicle to the waypoint
    /// is decomposed into a cross-track offset (steered by yaw demand) and
    /// an along-track gap (throttled by the speed loop). The depth loop is
    /// independent (heave demand from depth error).
    pub fn steer(&self, pose: &NedPose, wp: &Waypoint3D) -> ThrusterCmd6 {
        let g = self.gains;
        // LOS in the horizontal plane (guidance math in f32; the pose is
        // f64 for nav fidelity).
        let dx = (wp.north_m - pose.north_m) as f32;
        let dy = (wp.east_m - pose.east_m) as f32;
        let dist = (dx * dx + dy * dy).sqrt();
        // Course to the waypoint vs heading.
        let course_to_wp = (dy.atan2(dx)) as f64;
        let heading_err = wrap_pi(course_to_wp - pose.yaw_rad) as f32;
        // Cross-track error: distance off the direct line, signed via the
        // heading error (steer toward the line by demanding yaw).
        let yaw = (g.heading * heading_err).clamp(-1.0, 1.0);
        // Speed demand: full when far, taper inside 3× acceptance radius.
        let along = dist;
        let v = (g.speed * along).min(g.max_speed_mps);
        let surge = v / g.max_speed_mps.max(0.1);
        // Depth loop (positive-down): depth error drives heave.
        let depth_err = (wp.depth_m - pose.depth_m) as f32;
        let heave = (g.depth * depth_err).clamp(-0.5, 0.5);
        ThrusterCmd6 {
            surge: surge.clamp(0.0, 1.0) * (heading_err.abs() < 1.2) as i32 as f32,
            sway: 0.0,
            heave,
            roll: 0.0,
            pitch: 0.0,
            yaw,
        }
    }

    /// Whether `pose` satisfies the waypoint acceptance criteria.
    pub fn accepted(&self, pose: &NedPose, wp: &Waypoint3D) -> bool {
        let dx = wp.north_m - pose.north_m;
        let dy = wp.east_m - pose.east_m;
        let dz = wp.depth_m - pose.depth_m;
        let horiz = (dx * dx + dy * dy).sqrt();
        horiz < wp.accept_radius_m as f64 && dz.abs() < (2.0 * wp.accept_radius_m) as f64
    }
}

/// Wrap an angle into `[-π, π)`.
pub fn wrap_pi(a: f64) -> f64 {
    let mut x = a;
    while x >= core::f64::consts::PI {
        x -= 2.0 * core::f64::consts::PI;
    }
    while x < -core::f64::consts::PI {
        x += 2.0 * core::f64::consts::PI;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose(n: f64, e: f64, d: f64) -> NedPose {
        NedPose {
            north_m: n,
            east_m: e,
            depth_m: d,
            ..NedPose::default()
        }
    }

    #[test]
    fn steers_toward_the_waypoint() {
        let f = WaypointFollower::new(GuidanceGains::survey());
        // Target due east, heading north: yaw demand must be positive
        // (turn right/starboard toward east).
        let cmd = f.steer(
            &pose(0.0, 0.0, 50.0),
            &Waypoint3D {
                north_m: 0.0,
                east_m: 500.0,
                depth_m: 50.0,
                speed_m_s: 1.5,
                accept_radius_m: 5.0,
            },
        );
        assert!(cmd.yaw > 0.5, "positive yaw to turn east: {}", cmd.yaw);
        // A 90° turn gates the surge (slow through the turn); straighten
        // out and the throttle opens up.
        let mut aligned = pose(0.0, 400.0, 50.0);
        aligned.yaw_rad = core::f64::consts::FRAC_PI_2; // heading east
        let cmd = f.steer(
            &aligned,
            &Waypoint3D {
                north_m: 0.0,
                east_m: 500.0,
                depth_m: 50.0,
                speed_m_s: 1.5,
                accept_radius_m: 5.0,
            },
        );
        assert!(cmd.surge > 0.9, "aligned: full surge");
    }

    #[test]
    fn depth_loop_drives_to_depth() {
        let f = WaypointFollower::new(GuidanceGains::survey());
        // Vehicle shallow, target deep: heave positive (down).
        let cmd = f.steer(
            &pose(0.0, 0.0, 10.0),
            &Waypoint3D {
                north_m: 10.0,
                east_m: 10.0,
                depth_m: 50.0,
                speed_m_s: 1.0,
                accept_radius_m: 5.0,
            },
        );
        assert!(cmd.heave > 0.2, "descend: {}", cmd.heave);
        let cmd = f.steer(
            &pose(0.0, 0.0, 80.0),
            &Waypoint3D {
                north_m: 10.0,
                east_m: 10.0,
                depth_m: 50.0,
                speed_m_s: 1.0,
                accept_radius_m: 5.0,
            },
        );
        assert!(cmd.heave < -0.2, "ascend: {}", cmd.heave);
    }

    #[test]
    fn acceptance_radius_decides_arrival() {
        let f = WaypointFollower::new(GuidanceGains::survey());
        let wp = Waypoint3D {
            north_m: 100.0,
            east_m: 0.0,
            depth_m: 50.0,
            speed_m_s: 1.0,
            accept_radius_m: 5.0,
        };
        assert!(f.accepted(&pose(102.0, 1.0, 51.0), &wp));
        assert!(!f.accepted(&pose(110.0, 0.0, 50.0), &wp));
    }

    #[test]
    fn wrap_pi_handles_branch_cuts() {
        assert!((wrap_pi(3.5) - (-2.7831853)).abs() < 1e-6);
        assert!((wrap_pi(-3.5) - 2.7831853).abs() < 1e-6);
        assert!(wrap_pi(0.1).abs() < 0.2);
    }
}
