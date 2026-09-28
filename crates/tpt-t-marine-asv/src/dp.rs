//! Dynamic positioning: hold position within 1 m in 2-knot currents
//! (spec §5.3).
//!
//! Cascade: position error → velocity demand → body thrust, with the
//! measured current as feed-forward (the thrusters only need to cancel
//! the *error*, not fight the water blind).

use tpt_t_marine_core::wire::{NedPose, ThrusterCmd6};

/// DP gains.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DpGains {
    /// Position gain, (m/s) per metre of error.
    pub kp_pos: f32,
    /// Integral gain, (m/s) per metre·second — cancels the steady current.
    pub ki_pos: f32,
    /// Damping, (m/s) per (m/s) of velocity.
    pub kd_vel: f32,
    /// Thrust per (m/s) of velocity demand.
    pub thrust_per_mps: f32,
}

impl DpGains {
    /// A 12 m workboat class.
    pub const fn workboat() -> Self {
        Self {
            kp_pos: 0.15,
            ki_pos: 0.15,
            kd_vel: 1.0,
            thrust_per_mps: 0.6,
        }
    }
}

/// The DP controller.
#[derive(Debug, Clone, Copy)]
pub struct DynamicPositioner {
    /// PD(+I) gains.
    pub gains: DpGains,
    /// Station-keeping target, NED metres (and heading).
    target: NedPose,
    /// Integrated position error (the anti-current term).
    integral: [f32; 2],
}

impl DynamicPositioner {
    /// Capture the current position as the station.
    pub fn engage(gains: DpGains, at: NedPose) -> Self {
        Self {
            gains,
            target: at,
            integral: [0.0; 2],
        }
    }

    /// Retarget the station.
    pub fn retarget(&mut self, at: NedPose) {
        self.target = at;
    }

    /// The station.
    pub fn station(&self) -> NedPose {
        self.target
    }

    /// One control tick: `velocity_mps` is the measured NED velocity,
    /// `current` the current estimate's local vector (feed-forward).
    pub fn update(
        &mut self,
        pose: &NedPose,
        velocity_mps: [f32; 2],
        current: [f32; 2],
    ) -> ThrusterCmd6 {
        let g = self.gains;
        let err_n = (self.target.north_m - pose.north_m) as f32;
        let err_e = (self.target.east_m - pose.east_m) as f32;
        // Velocity demand from the PD loop on *ground* velocity, then the
        // current subtracted in thrust space: the thrusters pre-cancel the
        // water motion instead of the loop fighting it.
        self.integral[0] = (self.integral[0] + err_n).clamp(-10.0, 10.0);
        self.integral[1] = (self.integral[1] + err_e).clamp(-10.0, 10.0);
        let vn = g.kp_pos * err_n + g.ki_pos * self.integral[0]
            - g.kd_vel * velocity_mps[0]
            - current[0];
        let ve = g.kp_pos * err_e + g.ki_pos * self.integral[1]
            - g.kd_vel * velocity_mps[1]
            - current[1];
        // Body frame: assume heading-aligned surge/sway (DP usually holds
        // heading; here surge=north, sway=east after the yaw loop).
        ThrusterCmd6 {
            surge: (vn * g.thrust_per_mps).clamp(-1.0, 1.0),
            sway: (ve * g.thrust_per_mps).clamp(-1.0, 1.0),
            heave: 0.0,
            roll: 0.0,
            pitch: 0.0,
            // Heading hold toward the station heading.
            yaw: (0.5 * ang_diff(self.target.yaw_rad, pose.yaw_rad) as f32).clamp(-1.0, 1.0),
        }
    }

    /// Horizontal distance from the station, metres.
    pub fn offset_m(&self, pose: &NedPose) -> f64 {
        let dn = self.target.north_m - pose.north_m;
        let de = self.target.east_m - pose.east_m;
        (dn * dn + de * de).sqrt()
    }
}

/// Smallest signed angle difference `a − b` in radians, `[-π, π)`.
fn ang_diff(a: f64, b: f64) -> f64 {
    let d = (a - b) % (2.0 * core::f64::consts::PI);
    let d = if d >= 0.0 {
        d
    } else {
        d + 2.0 * core::f64::consts::PI
    };
    if d >= core::f64::consts::PI {
        d - 2.0 * core::f64::consts::PI
    } else {
        d
    }
}

/// Simulates the closed loop against a 2-knot current (the spec's DP
/// claim) and returns the final offset.
pub fn hold_sim(gains: DpGains, current: [f32; 2], ticks: usize, dt_s: f32) -> f64 {
    let start = NedPose {
        north_m: 0.0,
        east_m: 0.0,
        depth_m: 0.0,
        yaw_rad: 0.0,
        pitch_rad: 0.0,
        roll_rad: 0.0,
    };
    let mut dp = DynamicPositioner::engage(gains, start);
    let mut pose = start;
    // Vehicle starts blown 5 m south of the station.
    pose.north_m = -5.0;
    let mut vel = [0.0f32; 2];
    for _ in 0..ticks {
        let cmd = dp.update(&pose, vel, current);
        // First-order vehicle: velocity tracks commanded thrust × 2.5 m/s
        // max, then integrate. Current pushes directly.
        vel[0] += (cmd.surge * 2.5 - vel[0]) * dt_s * 0.8;
        vel[1] += (cmd.sway * 2.5 - vel[1]) * dt_s * 0.8;
        pose.north_m += (vel[0] + current[0]) as f64 * dt_s as f64;
        pose.east_m += (vel[1] + current[1]) as f64 * dt_s as f64;
    }
    dp.offset_m(&pose)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn holds_station_in_two_knot_current() {
        // Spec: hold within 1 m in 2-knot currents (≈1.03 m/s).
        let offset = hold_sim(DpGains::workboat(), [1.03, 0.0], 12_000, 0.05);
        assert!(offset < 1.0, "final offset {offset} m after 10 min DP");
    }

    #[test]
    fn zero_current_is_trivial() {
        let offset = hold_sim(DpGains::workboat(), [0.0, 0.0], 1_000, 0.05);
        assert!(offset < 0.5, "offset {offset}");
    }

    #[test]
    fn retarget_moves_the_station() {
        let start = NedPose::default();
        let mut dp = DynamicPositioner::engage(DpGains::workboat(), start);
        let mut to = start;
        to.north_m = 100.0;
        dp.retarget(to);
        assert_eq!(dp.station().north_m, 100.0);
    }
}
