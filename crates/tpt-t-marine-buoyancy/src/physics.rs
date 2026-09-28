//! Compression-at-depth physics (spec: "vehicles get heavier as neoprene
//! compresses").
//!
//! Two volume losses matter for a deep-operating vehicle:
//!
//! * **Soft buoyancy material** (neoprene / foam jackets) compresses roughly
//!   linearly with pressure over the operating envelope:
//!   `V_soft(d) = V_soft0 · (1 − k_soft·d)` with `k_soft` of order
//!   0.01 %/m for deep-rated syntactic foam up to 0.1 %/m for soft neoprene
//!   (which is why neoprene-clad gliders have depth limits).
//! * **Air bladders** follow Boyle's law: `V_air(d) = V_air0 · P_atm /
//!   (P_atm + ρ·g·d)` — halved by 10 m of seawater if filled at 1 atm, so
//!   real vehicles flood them and the soft-material term dominates.
//!
//! Net buoyancy force is `ρ(d)·g·V_disp(d) − m·g`; this crate's controllers
//! exist to hold it near zero as `V_disp` shrinks with depth.

/// Standard gravity, m/s².
pub const G_M_S2: f64 = 9.80665;
/// One atmosphere in Pa.
pub const P_ATM_PA: f64 = 101_325.0;

/// Vehicle buoyancy-relevant geometry and masses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleBuoyancyModel {
    /// Dry mass, kg.
    pub mass_kg: f64,
    /// Displacement volume at the surface, m³ (hull + all buoyancy material).
    pub volume_surface_m3: f64,
    /// Fraction of that volume in compressible soft buoyancy material.
    pub soft_fraction: f64,
    /// Soft-material compressibility, per metre of depth (1.0e-4 = the
    /// jacket loses 1 % of its volume per 100 m — syntactic-foam-like).
    pub soft_compress_per_m: f64,
    /// Air volume at 1 atm inside the pressure boundary, m³ (usually ~0 —
    /// real vehicles flood their bladders; kept for surface trimming).
    pub air_volume_m3: f64,
    /// In-situ water density at the surface, kg/m³.
    pub water_density_kg_m3: f64,
}

impl VehicleBuoyancyModel {
    /// Displacement volume at depth `d` (metres), m³.
    pub fn displaced_volume(&self, depth_m: f64) -> f64 {
        let soft = self.volume_surface_m3 * self.soft_fraction;
        let hard = self.volume_surface_m3 * (1.0 - self.soft_fraction);
        // Linear soft compression (clamped at 30 % loss — beyond that the
        // material has collapsed and the linear model no longer applies).
        let soft_d = (soft * (1.0 - self.soft_compress_per_m * depth_m)).max(soft * 0.7);
        // Boyle's law for the air spaces.
        let p = P_ATM_PA + self.water_density_kg_m3 * G_M_S2 * depth_m;
        let air_d = self.air_volume_m3 * P_ATM_PA / p;
        hard + soft_d + air_d
    }

    /// Net buoyancy force at depth `d` with the given ballast water mass,
    /// newtons (positive = buoyant, negative = heavy).
    pub fn net_force_n(&self, depth_m: f64, ballast_water_kg: f64) -> f64 {
        let rho = self.water_density_kg_m3;
        rho * G_M_S2 * self.displaced_volume(depth_m) - (self.mass_kg + ballast_water_kg) * G_M_S2
    }

    /// Ballast water mass (kg) needed for neutral buoyancy at depth `d`.
    /// Negative demand means the vehicle is already heavy even with an empty
    /// tank (beyond the model's operating envelope).
    pub fn neutral_ballast_kg(&self, depth_m: f64) -> f64 {
        let rho = self.water_density_kg_m3;
        rho * self.displaced_volume(depth_m) - self.mass_kg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 500 kg survey AUV: 0.55 m³ displacement (66 kg positively buoyant
    /// at the surface — the margin it must ballast out), 40 % of the volume
    /// in soft foam compressing at 1 %/100 m.
    fn vehicle() -> VehicleBuoyancyModel {
        VehicleBuoyancyModel {
            mass_kg: 500.0,
            volume_surface_m3: 0.55,
            soft_fraction: 0.4,
            soft_compress_per_m: 1.0e-4,
            air_volume_m3: 0.0,
            water_density_kg_m3: 1029.0,
        }
    }

    #[test]
    fn positively_buoyant_at_surface() {
        let v = vehicle();
        assert!(v.net_force_n(0.0, 0.0) > 0.0, "must float before ballast");
        // Neutral trim at the surface takes ~66 kg of water aboard.
        let b = v.neutral_ballast_kg(0.0);
        assert!((b - 65.95).abs() < 0.5, "surface ballast ≈ 66 kg, got {b}");
    }

    #[test]
    fn vehicle_gets_heavier_with_depth() {
        let v = vehicle();
        let deep = 2000.0;
        let b0 = v.neutral_ballast_kg(0.0);
        let bdeep = v.neutral_ballast_kg(deep);
        assert!(
            bdeep < b0 - 30.0,
            "compression at depth must eat ballast margin: {b0} → {bdeep}"
        );
        // Magnitude: soft volume 0.22 m³ losing 20 % at 2000 m = 0.044 m³
        // ≈ 45 kg of lost buoyancy.
        assert!((b0 - bdeep - 45.3).abs() < 1.0);
        // Still trimmable: demand stays positive (within tank envelope).
        assert!(bdeep > 0.0);
    }

    #[test]
    fn soft_compression_clamps_at_collapse() {
        let v = vehicle();
        // Past the linear envelope (30 % loss at 3000 m for this model) the
        // demand must stop tracking depth: the clamp froze the volume.
        let b10000 = v.neutral_ballast_kg(10_000.0);
        let b30000 = v.neutral_ballast_kg(30_000.0);
        assert!(
            (b10000 - b30000).abs() < 1.0e-9,
            "clamp must bound the loss: {b10000} vs {b30000}"
        );
    }

    #[test]
    fn air_bladder_compresses_fast() {
        let mut v = vehicle();
        v.soft_fraction = 0.0; // pure 10 L air bladder at 1 atm
        v.air_volume_m3 = 0.010;
        v.volume_surface_m3 = 0.010;
        v.mass_kg = 1029.0 * 0.010 - 5.0 / G_M_S2; // 5 N positive at surface
        // At 10 m the air volume halves: buoyancy drops by ~50 N.
        let f0 = v.net_force_n(0.0, 0.0);
        let f10 = v.net_force_n(10.0, 0.0);
        assert!(f10 < f0 - 30.0, "air spaces collapse fast: {f0} → {f10}");
    }

    #[test]
    fn net_force_sign_convention() {
        let v = vehicle();
        // Over-ballasted => heavy (negative force); under => buoyant.
        let b = v.neutral_ballast_kg(100.0);
        assert!(v.net_force_n(100.0, b + 5.0) < 0.0);
        assert!(v.net_force_n(100.0, b - 5.0) > 0.0);
        assert!(v.net_force_n(100.0, b).abs() < 1.0e-6);
    }
}
