//! Hydroacoustic biomass estimation: echo integration over the acoustic
//! volume → standing stock.
//!
//! Classic split-beam echo integration: backscatter energy summed over
//! the ensonified volume (Sv, volume backscattering strength) integrates
//! linearly to a per-ping density; calibrated fish density × mean weight
//! → biomass.

/// Acoustic calibration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AcousticCal {
    /// Scale factor from mean Sv (dB) to fish density, fish/m³ per dB of
    /// linear integration. Site-calibrated.
    pub density_per_sv: f64,
    /// Mean fish weight, kg (from the latest farm records).
    pub mean_weight_kg: f64,
}

impl AcousticCal {
    /// A typical salmon-site calibration.
    pub const fn salmon_site() -> Self {
        Self {
            density_per_sv: 0.8,
            mean_weight_kg: 4.5,
        }
    }
}

/// One ping's integration cell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EchoCell {
    /// Mean volume backscattering strength, dB re 1 m⁻¹.
    pub sv_db: f64,
    /// Ensonified volume of the cell, m³.
    pub volume_m3: f64,
}

/// The estimator.
#[derive(Debug, Clone, Copy)]
pub struct BiomassEstimator {
    cal: AcousticCal,
    /// Linear echo integral accumulated, fish·m³.
    integral: f64,
    /// Total volume integrated, m³.
    volume: f64,
}

impl BiomassEstimator {
    /// Create.
    pub fn new(cal: AcousticCal) -> Self {
        Self {
            cal,
            integral: 0.0,
            volume: 0.0,
        }
    }

    /// Integrate one ping cell. Sv → linear density: `ρ = 10^(Sv/10) × k`.
    pub fn integrate(&mut self, cell: &EchoCell) {
        let density = 10f64.powf(cell.sv_db / 10.0) * self.cal.density_per_sv;
        self.integral += density * cell.volume_m3;
        self.volume += cell.volume_m3;
    }

    /// Mean fish density over the integrated volume, fish/m³.
    pub fn mean_density(&self) -> f64 {
        if self.volume <= 0.0 {
            0.0
        } else {
            self.integral / self.volume
        }
    }

    /// Total standing stock, tonnes.
    pub fn biomass_tonnes(&self) -> f64 {
        self.integral * self.cal.mean_weight_kg / 1000.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_acoustic_is_zero_biomass() {
        let e = BiomassEstimator::new(AcousticCal::salmon_site());
        assert_eq!(e.biomass_tonnes(), 0.0);
        assert_eq!(e.mean_density(), 0.0);
    }

    #[test]
    fn echo_integration_is_linear_in_volume() {
        let mut e = BiomassEstimator::new(AcousticCal::salmon_site());
        let cell = EchoCell {
            sv_db: -40.0,
            volume_m3: 100.0,
        };
        e.integrate(&cell);
        e.integrate(&cell);
        let one = {
            let mut e1 = BiomassEstimator::new(AcousticCal::salmon_site());
            e1.integrate(&cell);
            e1
        };
        assert!((e.biomass_tonnes() - 2.0 * one.biomass_tonnes()).abs() < 1e-12);
        assert!((e.mean_density() - one.mean_density()).abs() < 1e-12);
    }

    #[test]
    fn biomass_scales_with_mean_weight() {
        // Same echo, heavier fish → more tonnes.
        let mut light = BiomassEstimator::new(AcousticCal {
            density_per_sv: 0.8,
            mean_weight_kg: 2.0,
        });
        let mut heavy = BiomassEstimator::new(AcousticCal {
            density_per_sv: 0.8,
            mean_weight_kg: 6.0,
        });
        let cell = EchoCell {
            sv_db: -35.0,
            volume_m3: 500.0,
        };
        light.integrate(&cell);
        heavy.integrate(&cell);
        assert!((heavy.biomass_tonnes() / light.biomass_tonnes() - 3.0).abs() < 1e-9);
    }
}
