//! Ocean current fields: uniform + tidal components (user-defined fields
//! plug in the same [`CurrentField`] trait), standing in for HYCOM/ROMS
//! model inputs.

/// A current field provider.
pub trait CurrentField {
    /// Current (east, north) m/s at a NED position and mission time.
    fn at(&self, ned_m: [f64; 3], t_s: f64) -> [f64; 2];
}

/// Uniform flow plus a sinusoidal tidal component along an axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UniformTidal {
    /// Steady (mean) east/north current, m/s.
    pub uniform_mps: [f64; 2],
    /// Tidal amplitude, m/s.
    pub tidal_amplitude_mps: f64,
    /// Tidal period, seconds (M2 ≈ 44_714).
    pub tidal_period_s: f64,
    /// Tidal flow direction, radians (0 = east).
    pub tidal_direction_rad: f64,
}

impl UniformTidal {
    /// The M2 constituent.
    pub const fn m2(uniform_mps: [f64; 2], amplitude_mps: f64) -> Self {
        Self {
            uniform_mps,
            tidal_amplitude_mps: amplitude_mps,
            tidal_period_s: 44_714.0,
            tidal_direction_rad: 0.0,
        }
    }
}

impl CurrentField for UniformTidal {
    fn at(&self, _ned_m: [f64; 3], t_s: f64) -> [f64; 2] {
        let w = 2.0 * core::f64::consts::PI / self.tidal_period_s;
        let tide = self.tidal_amplitude_mps * (w * t_s).sin();
        [
            self.uniform_mps[0] + tide * self.tidal_direction_rad.cos(),
            self.uniform_mps[1] + tide * self.tidal_direction_rad.sin(),
        ]
    }
}

/// A no-flow field (calibration / DP tests).
#[derive(Debug, Clone, Copy, Default)]
pub struct Calm;

impl CurrentField for Calm {
    fn at(&self, _ned_m: [f64; 3], _t_s: f64) -> [f64; 2] {
        [0.0; 2]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calm_is_zero() {
        let f = Calm;
        assert_eq!(f.at([0.0; 3], 0.0), [0.0; 2]);
    }

    #[test]
    fn m2_tide_oscillates_with_the_right_period() {
        let f = UniformTidal::m2([0.1, 0.0], 0.4);
        let t0 = f.at([0.0; 3], 0.0);
        assert!((t0[0] - 0.1).abs() < 1e-12, "sin(0) = 0");
        let peak_flood = f.at([0.0; 3], 44_714.0 / 4.0);
        assert!((peak_flood[0] - (0.1 + 0.4)).abs() < 1e-9, "peak flood");
        let peak_ebb = f.at([0.0; 3], 3.0 * 44_714.0 / 4.0);
        assert!((peak_ebb[0] - (0.1 - 0.4)).abs() < 1e-9, "peak ebb");
        // Full period back to the mean flow.
        let t1 = f.at([0.0; 3], 44_714.0);
        assert!((t1[0] - 0.1).abs() < 1e-9);
    }
}
