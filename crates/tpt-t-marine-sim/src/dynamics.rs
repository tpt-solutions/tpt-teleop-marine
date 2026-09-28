//! 6DOF-lite vehicle dynamics: per-axis quadratic hydrodynamic drag.

use tpt_t_marine_core::wire::ThrusterCmd6;

/// Vehicle physical parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleParams {
    /// Mass, kg.
    pub mass_kg: f64,
    /// Surge drag area coefficient `½ρ·Cd·A`, kg/m.
    pub drag_surge: f64,
    /// Sway drag coefficient, kg/m.
    pub drag_sway: f64,
    /// Heave drag coefficient, kg/m.
    pub drag_heave: f64,
    /// Yaw damping, N·m·s/rad.
    pub yaw_damping: f64,
    /// Max surge thrust, N.
    pub thrust_max_n: f64,
    /// Max yaw moment, N·m.
    pub yaw_moment_max: f64,
    /// Yaw inertia, kg·m².
    pub yaw_inertia: f64,
}

impl VehicleParams {
    /// A 500 kg survey AUV.
    pub const fn survey_auv() -> Self {
        Self {
            mass_kg: 500.0,
            drag_surge: 45.0,
            drag_sway: 180.0,
            drag_heave: 260.0,
            yaw_damping: 120.0,
            thrust_max_n: 250.0,
            yaw_moment_max: 60.0,
            yaw_inertia: 90.0,
        }
    }
}

/// Vehicle state: NED position, body velocities, yaw.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct State {
    /// Position NED, metres.
    pub ned_m: [f64; 3],
    /// Body velocities (surge, sway, heave), m/s.
    pub uvw_mps: [f64; 3],
    /// Yaw, radians.
    pub yaw_rad: f64,
    /// Yaw rate, rad/s.
    pub yaw_rate: f64,
}

impl State {
    /// Advance one step under a normalized command and a NED current.
    /// Deterministic, allocation-free.
    pub fn step(
        &mut self,
        p: &VehicleParams,
        cmd: &ThrusterCmd6,
        current_ned: [f64; 3],
        dt_s: f64,
    ) {
        // Thrust to body forces (yaw moment separately).
        let fx = f64::from(cmd.surge) * p.thrust_max_n;
        let fy = f64::from(cmd.sway) * p.thrust_max_n;
        let fz = f64::from(cmd.heave) * p.thrust_max_n;
        let mz = f64::from(cmd.yaw) * p.yaw_moment_max;

        // Rotate current into the body frame.
        let (sy, cy) = self.yaw_rad.sin_cos();
        let cx = current_ned[0] * cy + current_ned[1] * sy;
        let cy_ = -current_ned[0] * sy + current_ned[1] * cy;

        // Relative velocity for drag.
        let rx = self.uvw_mps[0] - cx;
        let ry = self.uvw_mps[1] - cy_;
        let rz = self.uvw_mps[2] - current_ned[2];

        // Accelerations: (F − drag·v|v|)/m.
        let drag = |k: f64, v: f64| k * v * v * v.signum();
        let ax = (fx - drag(p.drag_surge, rx)) / p.mass_kg;
        let ay = (fy - drag(p.drag_sway, ry)) / p.mass_kg;
        let az = (fz - drag(p.drag_heave, rz)) / p.mass_kg;

        // Integrate body velocities.
        self.uvw_mps[0] += ax * dt_s;
        self.uvw_mps[1] += ay * dt_s;
        self.uvw_mps[2] += az * dt_s;

        // Yaw dynamics.
        let yaw_acc = (mz - p.yaw_damping * self.yaw_rate) / p.yaw_inertia;
        let _ = yaw_acc;
        self.yaw_rate += yaw_acc * dt_s;
        self.yaw_rad += self.yaw_rate * dt_s;

        // Body → NED translation.
        let (s2, c2) = self.yaw_rad.sin_cos();
        let vx = self.uvw_mps[0] * c2 - self.uvw_mps[1] * s2;
        let vy = self.uvw_mps[0] * s2 + self.uvw_mps[1] * c2;
        self.ned_m[0] += vx * dt_s;
        self.ned_m[1] += vy * dt_s;
        self.ned_m[2] += self.uvw_mps[2] * dt_s;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_thrust_reaches_terminal_velocity() {
        let p = VehicleParams::survey_auv();
        let mut s = State::default();
        let cmd = ThrusterCmd6 {
            surge: 1.0,
            ..ThrusterCmd6::default()
        };
        for _ in 0..20_000 {
            s.step(&p, &cmd, [0.0; 3], 0.01);
        }
        // Terminal: thrust = drag → v = sqrt(250/45) ≈ 2.36 m/s.
        assert!((s.uvw_mps[0] - (250.0f64 / 45.0).sqrt()).abs() < 0.01);
        // Still water: position moves forward only.
        assert!(s.ned_m[0] > 0.0);
        assert!(s.ned_m[1].abs() < 1e-9);
    }

    #[test]
    fn zero_command_stops_eventually() {
        let p = VehicleParams::survey_auv();
        let mut s = State::default();
        s.uvw_mps[0] = 2.0;
        let stop = ThrusterCmd6::default();
        // Quadratic drag decays as 1/t: 1000 s brings 2 m/s under 1 cm/s.
        for _ in 0..100_000 {
            s.step(&p, &stop, [0.0; 3], 0.01);
        }
        assert!(
            s.uvw_mps[0].abs() < 0.05,
            "drag stops the vehicle: {}",
            s.uvw_mps[0]
        );
    }

    #[test]
    fn yaw_command_rotates() {
        let p = VehicleParams::survey_auv();
        let mut s = State::default();
        let cmd = ThrusterCmd6 {
            yaw: 1.0,
            ..ThrusterCmd6::default()
        };
        for _ in 0..3_000 {
            s.step(&p, &cmd, [0.0; 3], 0.01);
        }
        assert!(s.yaw_rad > 0.5, "yawed to {} rad", s.yaw_rad);
    }

    #[test]
    fn current_drifts_the_vehicle() {
        let p = VehicleParams::survey_auv();
        let mut s = State::default();
        let stop = ThrusterCmd6::default();
        for _ in 0..10_000 {
            s.step(&p, &stop, [0.5, 0.0, 0.0], 0.01);
        }
        assert!(
            s.ned_m[0] > 10.0,
            "drifted with the current: {}",
            s.ned_m[0]
        );
    }
}
