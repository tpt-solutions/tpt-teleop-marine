//! Wind-turbine foundation inspection: the spiral descent pattern around
//! a monopile with per-revolution depth bands (the standard GVI — general
//! visual inspection — profile).

use core::f64::consts::TAU;

/// Inspection plan parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurbinePlan {
    /// Foundation radius at the mudline, metres.
    pub foundation_radius_m: f64,
    /// Stand-off distance the ROV keeps from the structure, metres.
    pub standoff_m: f64,
    /// Vertical band between revolutions, metres.
    pub band_m: f64,
    /// Survey depth range: from `top_m` (just below LAT) to `bottom_m`
    /// (mudline + scour allowance), metres positive-down.
    pub top_m: f64,
    /// Deepest survey band centre, metres.
    pub bottom_m: f64,
}

impl TurbinePlan {
    /// Waypoint count for the plan.
    pub fn revolutions(&self) -> u32 {
        ((self.bottom_m - self.top_m) / self.band_m).ceil() as u32
    }

    /// Orbit radius (structure + stand-off).
    pub fn orbit_radius_m(&self) -> f64 {
        self.foundation_radius_m + self.standoff_m
    }

    /// Emit the spiral waypoints (north, east, depth) for one foundation
    /// at `(north, east)`. Waypoints per revolution: 16 (deterministic
    /// ring; the vehicle blends through them).
    pub fn waypoints(&self, north: f64, east: f64) -> Vec<(f64, f64, f64)> {
        let mut wps = Vec::new();
        let r = self.orbit_radius_m();
        let revs = self.revolutions();
        const PTS: usize = 16;
        for rev in 0..revs {
            let depth = self.top_m + (rev as f64 + 0.5) * self.band_m;
            for k in 0..PTS {
                let a = TAU * k as f64 / PTS as f64 + rev as f64 * 0.2; // slight twist
                wps.push((north + r * a.cos(), east + r * a.sin(), depth));
            }
        }
        wps
    }

    /// Scour-zone check points: 8 radial spokes on the seabed.
    pub fn scour_spokes(&self, north: f64, east: f64) -> Vec<(f64, f64, f64)> {
        let mut pts = Vec::new();
        for k in 0..8 {
            let a = TAU * k as f64 / 8.0;
            let r = self.orbit_radius_m() + 10.0;
            pts.push((north + r * a.cos(), east + r * a.sin(), self.bottom_m + 2.0));
        }
        pts
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan() -> TurbinePlan {
        TurbinePlan {
            foundation_radius_m: 4.0,
            standoff_m: 2.0,
            band_m: 2.0,
            top_m: 2.0,
            bottom_m: 32.0,
        }
    }

    #[test]
    fn plan_geometry_is_consistent() {
        let p = plan();
        assert_eq!(p.revolutions(), 15, "30 m of bands at 2 m");
        assert!((p.orbit_radius_m() - 6.0).abs() < 1e-9);
    }

    #[test]
    fn spiral_covers_bands_and_circles_the_structure() {
        let p = plan();
        let wps = p.waypoints(1000.0, 2000.0);
        assert_eq!(wps.len(), 15 * 16);
        // Every waypoint sits on the orbit ring.
        for (n, e, _) in &wps {
            let d = ((n - 1000.0).powi(2) + (e - 2000.0).powi(2)).sqrt();
            assert!((d - 6.0).abs() < 1e-6, "radius {d}");
        }
        // Depth bands span the range.
        assert!((wps[0].2 - 3.0).abs() < 1e-9, "first band centre");
        assert!(wps.last().unwrap().2 <= 33.0);
    }

    #[test]
    fn scour_spokes_ring_the_site() {
        let p = plan();
        let pts = p.scour_spokes(0.0, 0.0);
        assert_eq!(pts.len(), 8);
        for (n, e, d) in pts {
            let r = (n * n + e * e).sqrt();
            assert!((r - 16.0).abs() < 1e-6);
            assert!(d > 32.0);
        }
    }
}
