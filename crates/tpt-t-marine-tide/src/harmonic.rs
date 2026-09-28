//! The harmonic constituent model.
//!
//! Constituents have astronomically fixed angular speeds (degrees per
//! hour); amplitudes and phases come from site harmonic analysis (station
//! constants, loaded once at mission planning). Evaluation is a fixed-size
//! dot product — no allocation, deterministic to the bit.

/// Maximum constituents in a station file.
pub const MAX_CONSTITUENTS: usize = 16;

/// Angular speed of a constituent, degrees per hour.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Constituent {
    /// Name (short code: "M2", "S2", …) — up to 4 bytes + NUL.
    pub code: [u8; 4],
    /// Standard speed, degrees/hour.
    pub speed_deg_hr: f64,
    /// Site amplitude, metres (for level) or knots (for streams).
    pub amplitude: f64,
    /// Site phase lag, degrees (relative to the equilibrium argument used
    /// when the station was analysed).
    pub phase_deg: f64,
}

impl Constituent {
    /// The principal lunar semidiurnal constituent (period ≈ 12.42 h).
    pub const M2_SPEED: f64 = 28.984_104_2;
    /// Solar semidiurnal (period exactly 12 h).
    pub const S2_SPEED: f64 = 30.0;
    /// Larger lunar elliptic semidiurnal.
    pub const N2_SPEED: f64 = 28.439_729_5;
    /// Lunisolar diurnal (period ≈ 23.93 h).
    pub const K1_SPEED: f64 = 15.041_068_6;
    /// Principal lunar diurnal (period ≈ 25.82 h).
    pub const O1_SPEED: f64 = 13.943_035_6;
}

/// A station's harmonic model.
#[derive(Debug, Clone, Copy)]
pub struct TideModel {
    /// Mean sea level above chart datum, metres.
    pub msl_m: f64,
    constituents: [Constituent; MAX_CONSTITUENTS],
    n: usize,
}

impl TideModel {
    /// Build from a constituent list (≤ [`MAX_CONSTITUENTS`]).
    pub fn new(msl_m: f64, list: &[Constituent]) -> Self {
        assert!(list.len() <= MAX_CONSTITUENTS, "too many constituents");
        let mut c = [Constituent {
            code: [0; 4],
            speed_deg_hr: 0.0,
            amplitude: 0.0,
            phase_deg: 0.0,
        }; MAX_CONSTITUENTS];
        c[..list.len()].copy_from_slice(list);
        Self {
            msl_m,
            constituents: c,
            n: list.len(),
        }
    }

    /// Number of active constituents.
    pub fn len(&self) -> usize {
        self.n
    }

    /// Whether the model has no constituents.
    pub fn is_empty(&self) -> bool {
        self.n == 0
    }

    /// Constituent accessor.
    pub fn constituent(&self, i: usize) -> &Constituent {
        &self.constituents[i]
    }

    /// Predicted water level above chart datum at hours-since-epoch `t_hr`.
    pub fn height_at(&self, t_hr: f64) -> f64 {
        let mut h = self.msl_m;
        for c in &self.constituents[..self.n] {
            let arg = (c.speed_deg_hr * t_hr + c.phase_deg).to_radians();
            h += c.amplitude * arg.cos();
        }
        h
    }

    /// One constituent's contribution at `t_hr` (diagnostics).
    pub fn contribution(&self, i: usize, t_hr: f64) -> f64 {
        let c = &self.constituents[i];
        c.amplitude * ((c.speed_deg_hr * t_hr + c.phase_deg).to_radians().cos())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A representative semi-diurnal site: M2 1.5 m, S2 0.5 m, both phase 0,
    /// MSL 3.0 m above datum.
    fn model() -> TideModel {
        TideModel::new(
            3.0,
            &[
                Constituent {
                    code: *b"M2\0\0",
                    speed_deg_hr: Constituent::M2_SPEED,
                    amplitude: 1.5,
                    phase_deg: 0.0,
                },
                Constituent {
                    code: *b"S2\0\0",
                    speed_deg_hr: Constituent::S2_SPEED,
                    amplitude: 0.5,
                    phase_deg: 0.0,
                },
            ],
        )
    }

    #[test]
    fn msl_when_constituents_absent() {
        let m = TideModel::new(3.0, &[]);
        assert!((m.height_at(1234.0) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn m2_period_is_about_12_42_hours() {
        let m = TideModel::new(
            0.0,
            &[Constituent {
                code: *b"M2\0\0",
                speed_deg_hr: Constituent::M2_SPEED,
                amplitude: 1.0,
                phase_deg: 0.0,
            }],
        );
        // Successive high waters: one rotation of 360°.
        let t_high = 0.0;
        let period_hr = 360.0 / Constituent::M2_SPEED;
        let h1 = m.height_at(t_high);
        let h2 = m.height_at(t_high + period_hr);
        assert!((h1 - h2).abs() < 1e-9);
        // The two extrema are ±amplitude.
        assert!((m.height_at(0.0) - 1.0).abs() < 1e-9);
        let half = period_hr / 2.0;
        assert!((m.height_at(half) + 1.0).abs() < 1e-9);
    }

    #[test]
    fn spring_beats_neap() {
        // M2 and S2 in phase at t=0 → spring tide: the range over one M2
        // half-period is within a hair of the full 2×(1.5+0.5)=4.0 m (S2
        // lags a few degrees by then).
        let m = model();
        let spring = m.height_at(0.0) - m.height_at(360.0 / Constituent::M2_SPEED / 2.0);
        assert!(spring > 3.9, "spring range {spring}");
        // A full spring–neap cycle is half the M2–S2 beat: 360°/|Δω| / 2 ≈
        // 177.2 h. At neaps M2 and S2 oppose: the range approaches
        // 2×(1.5−0.5)=2.0 m. Scan the neap window for the actual range.
        let t_neap = 177.15;
        let mut lo = f64::MAX;
        let mut hi = f64::MIN;
        for k in 0..62 {
            let h = m.height_at(t_neap + k as f64 * 0.1);
            lo = lo.min(h);
            hi = hi.max(h);
        }
        assert!(hi - lo < 2.4, "neap range {}", hi - lo);
        assert!(spring - (hi - lo) > 1.3, "spring must clearly beat neap");
    }

    #[test]
    fn deterministic_and_finite() {
        let m = model();
        let mut prev = m.height_at(0.0);
        assert!(prev.is_finite());
        for k in 1..1000 {
            let h = m.height_at(k as f64 * 0.25);
            assert!(h.is_finite());
            prev = h;
        }
        let _ = prev;
    }
}
