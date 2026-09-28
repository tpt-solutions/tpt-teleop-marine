//! The lumped-mass Verlet chain (spec §4.4: 50 nodes, 100 Hz).
//!
//! State: node positions and their previous positions (Verlet's trick —
//! velocities are implicit). Forces per step: weight minus buoyancy,
//! quadratic drag opposing node velocity, then distance constraints
//! (several relaxation passes) enforce the segment length. Node 0 is
//! pinned to the TMS; the last node follows the ROV's tow point.

/// Nodes in the chain (spec: 50).
pub const NODES: usize = 50;

/// Physics step, seconds (100 Hz).
pub const DT_S: f64 = 0.01;

/// Tether material/geometry constants.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TetherSpec {
    /// Total tether length, metres.
    pub length_m: f64,
    /// Linear density in water (density minus displaced water), kg/m.
    pub linear_density_kg_m: f64,
    /// Drag diameter, metres.
    pub diameter_m: f64,
    /// Water density, kg/m³.
    pub water_density_kg_m3: f64,
    /// Drag coefficient (cylindrical, cross-flow).
    pub drag_cd: f64,
}

impl TetherSpec {
    /// A typical 500 m neutrally-trimmed ROV umbilical.
    pub const fn typical() -> Self {
        Self {
            length_m: 500.0,
            linear_density_kg_m: 0.15,
            diameter_m: 0.017,
            water_density_kg_m3: 1029.0,
            drag_cd: 1.2,
        }
    }

    /// Segment rest length.
    pub fn segment_m(&self) -> f64 {
        self.length_m / (NODES - 1) as f64
    }
}

/// Gravity, m/s².
const G: f64 = 9.80665;

/// The chain state (fixed arrays; no allocation anywhere).
#[derive(Debug, Clone, Copy)]
pub struct Chain {
    spec: TetherSpec,
    /// Node positions, metres, NED (down positive).
    pub pos: [[f64; 3]; NODES],
    /// Previous positions (Verlet).
    prev: [[f64; 3]; NODES],
}

impl Chain {
    /// Initialize along the line from the TMS at `origin` toward the ROV
    /// tow point `rov`, spaced at the *rest* segment length (the natural
    /// hanging geometry; any slack pools behind the tail pin).
    pub fn new(spec: TetherSpec, origin: [f64; 3], rov: [f64; 3]) -> Self {
        let mut pos = [[0.0; 3]; NODES];
        let prev = [[0.0; 3]; NODES];
        let seg = spec.segment_m();
        let dir = [rov[0] - origin[0], rov[1] - origin[1], rov[2] - origin[2]];
        let len = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2])
            .sqrt()
            .max(1e-9);
        let unit = [dir[0] / len, dir[1] / len, dir[2] / len];
        for (i, p) in pos.iter_mut().enumerate() {
            let d = (i as f64 * seg).min(len);
            *p = [
                origin[0] + unit[0] * d,
                origin[1] + unit[1] * d,
                origin[2] + unit[2] * d,
            ];
        }
        Self { spec, pos, prev }
    }

    /// Advance one 100 Hz step. `rov_pos` pins the tail; `current` is the
    /// uniform water velocity (NED, m/s).
    pub fn step(&mut self, rov_pos: [f64; 3], current: [f64; 3], dt_s: f64) {
        let seg = self.spec.segment_m();

        // 1. Verlet integration with weight/buoyancy and drag. Implicit
        // node velocity is damped (water + internal friction) and clamped
        // so constraint catches cannot inject runaway energy.
        const DAMPING: f64 = 0.04;
        const MAX_V: f64 = 15.0;
        for i in 1..NODES {
            let p = self.pos[i];
            let pp = self.prev[i];
            let mut vx = (p[0] - pp[0]) / dt_s * (1.0 - DAMPING);
            let mut vy = (p[1] - pp[1]) / dt_s * (1.0 - DAMPING);
            let mut vz = (p[2] - pp[2]) / dt_s * (1.0 - DAMPING);
            let v0 = (vx * vx + vy * vy + vz * vz).sqrt();
            if v0 > MAX_V {
                let k = MAX_V / v0;
                vx *= k;
                vy *= k;
                vz *= k;
            }
            // Relative velocity vs the water.
            let rx = vx - current[0];
            let ry = vy - current[1];
            let rz = vz - current[2];
            let speed = (rx * rx + ry * ry + rz * rz).sqrt();
            // Drag: ½·ρ·Cd·A·v² per node, opposing relative motion.
            let node_mass = self.spec.linear_density_kg_m * seg;
            let area = self.spec.diameter_m * seg;
            let drag =
                0.5 * self.spec.water_density_kg_m3 * self.spec.drag_cd * area * speed * speed
                    / node_mass.max(1.0e-6);
            let drag_ax = if speed > 1e-9 { drag * rx / speed } else { 0.0 };
            let drag_ay = if speed > 1e-9 { drag * ry / speed } else { 0.0 };
            let drag_az = if speed > 1e-9 { drag * rz / speed } else { 0.0 };
            // In-water acceleration: gravity (down) minus buoyancy fraction
            // (already in linear_density), plus drag.
            let ax = -drag_ax;
            let ay = -drag_ay;
            let az = G - drag_az;

            let next = [
                2.0 * p[0] - pp[0] + ax * dt_s * dt_s,
                2.0 * p[1] - pp[1] + ay * dt_s * dt_s,
                2.0 * p[2] - pp[2] + az * dt_s * dt_s,
            ];
            self.prev[i] = p;
            self.pos[i] = next;
        }

        // 2. Constraint relaxation: the tether is a ROPE — one-sided
        // constraints (pull nodes together when a segment exceeds its rest
        // length; slack is free). Endpoints pinned.
        for _ in 0..8 {
            self.pos[NODES - 1] = rov_pos;
            for i in 0..NODES - 1 {
                let a = self.pos[i];
                let b = self.pos[i + 1];
                let dx = b[0] - a[0];
                let dy = b[1] - a[1];
                let dz = b[2] - a[2];
                let d = (dx * dx + dy * dy + dz * dz).sqrt();
                if d <= seg {
                    continue; // slack segment: no force
                }
                let err = (d - seg) / d;
                if i == 0 {
                    self.pos[i + 1][0] -= err * dx;
                    self.pos[i + 1][1] -= err * dy;
                    self.pos[i + 1][2] -= err * dz;
                } else if i == NODES - 2 {
                    self.pos[i][0] += err * dx;
                    self.pos[i][1] += err * dy;
                    self.pos[i][2] += err * dz;
                } else {
                    self.pos[i][0] += 0.5 * err * dx;
                    self.pos[i][1] += 0.5 * err * dy;
                    self.pos[i][2] += 0.5 * err * dz;
                    self.pos[i + 1][0] -= 0.5 * err * dx;
                    self.pos[i + 1][1] -= 0.5 * err * dy;
                    self.pos[i + 1][2] -= 0.5 * err * dz;
                }
            }
        }
        self.pos[NODES - 1] = rov_pos;
    }

    /// Node accessor.
    pub fn node(&self, i: usize) -> [f64; 3] {
        self.pos[i]
    }

    /// Tension estimate at the TMS: the vertical load of the hanging
    /// portion plus drag-induced load — approximated by the last
    /// constraint impulse, i.e. the stretch force of segment 0:
    /// T ≈ k·(|p1 − p0| − seg) with a nominal stiffness. For monitoring we
    /// report the geometric estimate in newtons.
    pub fn tension_at_tms_n(&self) -> f64 {
        let seg = self.spec.segment_m();
        let dx = self.pos[1][0] - self.pos[0][0];
        let dy = self.pos[1][1] - self.pos[0][1];
        let dz = self.pos[1][2] - self.pos[0][2];
        let d = (dx * dx + dy * dy + dz * dz).sqrt();
        // Nominal umbilical stiffness, N/m (typical 5 t/m… scaled).
        const K_N_M: f64 = 5000.0;
        (K_N_M * (d - seg).abs()).max(0.0) + self.spec.linear_density_kg_m * G * seg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hanging() -> Chain {
        // TMS at the surface, ROV 100 m below.
        Chain::new(TetherSpec::typical(), [0.0, 0.0, 0.0], [0.0, 0.0, 100.0])
    }

    #[test]
    fn catenary_settles_below_the_straight_line_in_current() {
        let mut c = hanging();
        // 1 knot (0.514 m/s) current to the east.
        for _ in 0..2000 {
            c.step([0.0, 0.0, 100.0], [0.0, 0.514, 0.0], DT_S);
        }
        // Mid-water nodes drift east of the straight line.
        let mid = c.node(NODES / 2);
        assert!(mid[1] > 1.0, "current bows the tether east: {}", mid[1]);
        // Endpoints stay pinned.
        assert_eq!(c.node(0), [0.0, 0.0, 0.0]);
        assert_eq!(c.node(NODES - 1), [0.0, 0.0, 100.0]);
    }

    #[test]
    fn segments_hold_length_within_tolerance() {
        let mut c = hanging();
        for _ in 0..500 {
            c.step([2.0, 1.0, 100.0], [0.3, 0.0, 0.0], DT_S);
        }
        let seg = TetherSpec::typical().segment_m();
        for i in 0..NODES - 1 {
            let a = c.node(i);
            let b = c.node(i + 1);
            let d = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt();
            // Rope physics: segments may go slack (d < seg) but must never
            // stretch beyond rest length (+5 % relaxation tolerance).
            assert!(d <= seg * 1.05, "segment {i} stretched: {d} vs {seg}");
        }
    }

    #[test]
    fn tension_is_positive_and_finite() {
        let mut c = hanging();
        for _ in 0..100 {
            c.step([0.0, 0.0, 100.0], [0.2, 0.2, 0.0], DT_S);
        }
        let t = c.tension_at_tms_n();
        assert!(t > 0.0 && t.is_finite(), "tension {t}");
    }
}
