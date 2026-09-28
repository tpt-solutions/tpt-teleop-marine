//! Tidal streams at arbitrary points: the ROV working near a structure
//! needs "current here, now", which differs from the tide-gauge station by
//! site-dependent scaling (amplitude and phase factors per constituent,
//! interpolated from an atlas into fixed cells).
//!
//! A [`StreamCell`] holds per-constituent scale factors plus the
//! residual (non-tidal) current for its cell; the
//! [`StreamPredictor`](StreamPredictor) combines them with a reference
//! [`TideModel`](crate::harmonic::TideModel) into a local stream vector.
//! Streams are modeled as rotary: one major axis amplitude + eccentricity
//! ellipse per constituent is approximated here by independent u/v
//! amplitudes and phases — enough for planning-grade prediction.

use crate::harmonic::{Constituent, TideModel};

/// Per-cell stream factors for one constituent: u/v amplitude ratios to
/// the reference station and phase shifts (degrees).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CellFactors {
    /// Amplitude ratio, east component (relative to station amplitude).
    pub ratio_u: f64,
    /// Amplitude ratio, north component.
    pub ratio_v: f64,
    /// Phase shift, east (degrees).
    pub phase_shift_u_deg: f64,
    /// Phase shift, north (degrees).
    pub phase_shift_v_deg: f64,
}

/// One fixed atlas cell: factors per constituent (parallel arrays) + the
/// steady residual current. Capacity [`crate::harmonic::MAX_CONSTITUENTS`].
#[derive(Debug, Clone, Copy)]
pub struct StreamCell {
    /// Per-constituent amplitude/phase factors (parallel to the reference
    /// model's constituent list).
    pub factors: [CellFactors; crate::harmonic::MAX_CONSTITUENTS],
    /// Residual (wind-driven / mean) east current, m/s.
    pub residual_u_mps: f64,
    /// Residual north current, m/s.
    pub residual_v_mps: f64,
}

impl StreamCell {
    /// A cell with identical factors `r` on the u axis and zero on v — a
    /// rectilinear stream aligned east–west.
    pub fn rectilinear(r: f64, residual_u: f64, residual_v: f64) -> Self {
        let mut c = Self::uniform(r, residual_u, residual_v);
        for f in &mut c.factors {
            f.ratio_v = 0.0;
        }
        c
    }

    /// A cell with identical factors `r` on every component.
    pub fn uniform(r: f64, residual_u: f64, residual_v: f64) -> Self {
        Self {
            factors: [CellFactors {
                ratio_u: r,
                ratio_v: r,
                phase_shift_u_deg: 0.0,
                phase_shift_v_deg: 0.0,
            }; crate::harmonic::MAX_CONSTITUENTS],
            residual_u_mps: residual_u,
            residual_v_mps: residual_v,
        }
    }
}

/// Predicts the tidal stream (m/s) at a cell, given the reference model.
#[derive(Debug, Clone, Copy)]
pub struct StreamPredictor<'a> {
    reference: &'a TideModel,
}

impl<'a> StreamPredictor<'a> {
    /// Bind to a reference harmonic model (its amplitudes are interpreted
    /// as speeds, m/s, for stream stations).
    pub fn new(reference: &'a TideModel) -> Self {
        Self { reference }
    }

    /// Stream vector (u_east, v_north) at `cell` for hours-since-epoch
    /// `t_hr`.
    pub fn at(&self, cell: &StreamCell, t_hr: f64) -> (f64, f64) {
        let mut u = cell.residual_u_mps;
        let mut v = cell.residual_v_mps;
        for i in 0..self.reference.len() {
            let c: &Constituent = self.reference.constituent(i);
            let f = cell.factors[i];
            let base_arg = c.speed_deg_hr * t_hr;
            let arg_u = (base_arg + c.phase_deg + f.phase_shift_u_deg).to_radians();
            let arg_v = (base_arg + c.phase_deg + f.phase_shift_v_deg).to_radians();
            u += c.amplitude * f.ratio_u * arg_u.cos();
            v += c.amplitude * f.ratio_v * arg_v.cos();
        }
        (u, v)
    }

    /// Peak flood/ebb speed magnitude at the cell over the next `window_hr`
    /// hours (scanned at 6-minute steps — planning grade, deterministic).
    pub fn peak_speed(&self, cell: &StreamCell, t0_hr: f64, window_hr: f64) -> f64 {
        let mut peak = 0.0f64;
        let mut t = t0_hr;
        let end = t0_hr + window_hr;
        while t <= end {
            let (u, v) = self.at(cell, t);
            peak = peak.max((u * u + v * v).sqrt());
            t += 0.1;
        }
        peak
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::harmonic::Constituent;

    /// A reversing-stream station: M2 with 1.0 m/s amplitude on the u axis,
    /// 0 on v (phase 0). Reference "height" amplitude means speed here.
    fn reference() -> TideModel {
        TideModel::new(
            0.0,
            &[Constituent {
                code: *b"M2\0\0",
                speed_deg_hr: Constituent::M2_SPEED,
                amplitude: 1.0,
                phase_deg: 0.0,
            }],
        )
    }

    #[test]
    fn reversing_stream_slack_at_quarter_period() {
        let m = reference();
        let p = StreamPredictor::new(&m);
        let cell = StreamCell::rectilinear(1.0, 0.0, 0.0);
        // Flood peak at t=0 (u = 1 m/s), slack at quarter period.
        let (u0, _) = p.at(&cell, 0.0);
        assert!((u0 - 1.0).abs() < 1e-9);
        let quarter = 90.0 / Constituent::M2_SPEED;
        let (us, _) = p.at(&cell, quarter);
        assert!(us.abs() < 1e-9, "slack at quarter period: {us}");
    }

    #[test]
    fn phase_shift_delays_the_cell() {
        let m = reference();
        let p = StreamPredictor::new(&m);
        let mut cell = StreamCell::rectilinear(1.0, 0.0, 0.0);
        cell.factors[0].phase_shift_u_deg = 90.0; // 1/4 cycle lag
        let (u0, _) = p.at(&cell, 0.0);
        // Station floods at t=0; this cell lags a quarter cycle → near slack.
        assert!(u0.abs() < 0.1, "lagged cell near slack: {u0}");
    }

    #[test]
    fn residual_current_offsets_the_tide() {
        let m = reference();
        let p = StreamPredictor::new(&m);
        let cell = StreamCell::rectilinear(1.0, 0.3, -0.2);
        let (u, v) = p.at(&cell, 90.0 / Constituent::M2_SPEED); // tide slack
        assert!((u - 0.3).abs() < 1e-9);
        assert!((v + 0.2).abs() < 1e-9);
    }

    #[test]
    fn peak_speed_scans_the_window() {
        let m = reference();
        let p = StreamPredictor::new(&m);
        let cell = StreamCell::rectilinear(1.0, 0.0, 0.0);
        let peak = p.peak_speed(&cell, 0.0, 6.2);
        assert!((peak - 1.0).abs() < 0.01, "peak {peak} ≈ M2 amplitude");
    }

    #[test]
    fn attenuation_factor_scales_amplitude() {
        let m = reference();
        let p = StreamPredictor::new(&m);
        let cell = StreamCell::rectilinear(0.25, 0.0, 0.0); // sheltered cell
        let peak = p.peak_speed(&cell, 0.0, 12.5);
        assert!((peak - 0.25).abs() < 0.01, "sheltered peak {peak}");
    }
}
