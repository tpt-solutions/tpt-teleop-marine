//! Raw pressure → depth conversion with temperature compensation and
//! hysteresis (sensor-lag) correction.
//!
//! Physics: seawater depth from gauge pressure is `h = P / (ρ·g)` with the
//! in-situ density `ρ` a function of temperature and salinity (simple linear
//! seawater equation of state, accurate to ~0.1% over the 0–40 °C,
//! 0–40 PSU envelope relevant to vehicles). Two sensor effects are corrected:
//!
//! * **Temperature compensation** — piezoresistive sensors drift zero and
//!   span with temperature. Both drifts are modeled as linear coefficients
//!   calibrated at deployment.
//! * **Hysteresis / lag** — the sensing element and its oil column respond
//!   with a first-order lag, so a descending vehicle reads shallow and an
//!   ascending one reads deep. The correction subtracts `τ·(dh/dt)` from the
//!   raw estimate; `τ` is the calibrated time constant (typically
//!   10–100 ms).

/// Standard acceleration of gravity, m/s².
pub const G_M_S2: f64 = 9.80665;
/// One atmosphere in PSI, for gauge↔absolute conversions.
pub const PSI_PER_ATM: f64 = 14.6959;

/// Density of seawater, kg/m³, linearized equation of state.
///
/// `ρ(T, S) ≈ 1029.0 − 0.30·T(°C) + 0.78·(S − 35)` — good to ~0.1 kg/m³ for
/// the vehicle operating envelope (spec §5.4 environments).
pub fn seawater_density(temp_c: f64, salinity_psu: f64) -> f64 {
    1029.0 - 0.30 * temp_c + 0.78 * (salinity_psu - 35.0)
}

/// Calibration record for one depth sensor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorCalibration {
    /// Zero offset at calibration temperature, PSI.
    pub zero_psi: f64,
    /// Span error, relative (`1.0` = perfect).
    pub span: f64,
    /// Zero drift per °C from the calibration temperature, PSI/°C.
    pub zero_drift_psi_per_c: f64,
    /// Span drift per °C, relative /°C.
    pub span_drift_per_c: f64,
    /// Calibration reference temperature, °C.
    pub cal_temp_c: f64,
    /// Sensor + oil-column first-order lag time constant, seconds.
    pub lag_s: f64,
    /// Operating salinity, PSU (35.0 = standard seawater).
    pub salinity_psu: f64,
}

impl SensorCalibration {
    /// Nominal, factory-calibration-like settings.
    pub const fn nominal() -> Self {
        Self {
            zero_psi: 0.0,
            span: 1.0,
            zero_drift_psi_per_c: 0.0,
            span_drift_per_c: 0.0,
            cal_temp_c: 20.0,
            lag_s: 0.050,
            salinity_psu: 35.0,
        }
    }
}

/// One raw sample from the depth sensor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PressureSample {
    /// Raw gauge pressure, PSI.
    pub raw_psi: f64,
    /// Sensor temperature, °C.
    pub temp_c: f64,
    /// Sample timestamp, microseconds (used for the lag derivative).
    pub timestamp_us: u64,
}

/// Corrected depth estimate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepthEstimate {
    /// Compensated depth, metres positive-down.
    pub depth_m: f64,
    /// Vertical rate estimate from the previous sample, m/s (positive down).
    pub vv_m_s: f64,
    /// Temperature used for the density computation, °C.
    pub water_temp_c: f64,
}

/// Zero-allocation depth pipeline: feed raw samples in order, get compensated
/// depth out. One instance per sensor; state is `Copy`-sized.
#[derive(Debug, Clone, Copy)]
pub struct DepthPipeline {
    cal: SensorCalibration,
    prev: Option<(u64, f64)>, // (timestamp_us, depth_m)
}

impl DepthPipeline {
    /// Create a pipeline for a calibrated sensor.
    pub fn new(cal: SensorCalibration) -> Self {
        Self { cal, prev: None }
    }

    /// Temperature-compensated, density-corrected pressure (PSI) for a raw
    /// sample. Pure function of the sample and calibration.
    pub fn compensated_psi(&self, s: &PressureSample) -> f64 {
        let dt_c = s.temp_c - self.cal.cal_temp_c;
        let zero = self.cal.zero_psi + self.cal.zero_drift_psi_per_c * dt_c;
        let span = self.cal.span + self.cal.span_drift_per_c * dt_c;
        (s.raw_psi - zero) / span.max(1.0e-6)
    }

    /// Process one sample into a depth estimate.
    ///
    /// No allocation, no locks: the whole state is the previous sample.
    pub fn update(&mut self, s: PressureSample) -> DepthEstimate {
        let p_psi = self.compensated_psi(&s);
        let rho = seawater_density(s.temp_c, self.cal.salinity_psu);
        // PSI → Pa: ×6894.757; h = P/(ρ·g).
        let depth_raw = p_psi * 6894.757293168 / (rho * G_M_S2);

        // Hysteresis / lag correction: a first-order sensor reads the depth
        // τ seconds ago, so on a ramp it trails by τ·v (shallow when
        // descending, deep when ascending). Add `τ·v̂` (v positive down) to
        // align the reading with the current time; clamp so noise spikes
        // cannot invert the profile direction.
        let (vv, depth) = match self.prev {
            Some((t0, raw0)) => {
                let dt_s = ((s.timestamp_us - t0) as f64 / 1.0e6).max(1.0e-6);
                let rate = (depth_raw - raw0) / dt_s;
                let corr = (self.cal.lag_s * rate).clamp(-2.0, 2.0);
                (rate, depth_raw + corr)
            }
            None => (0.0, depth_raw),
        };
        self.prev = Some((s.timestamp_us, depth_raw));
        DepthEstimate {
            depth_m: depth,
            vv_m_s: vv,
            water_temp_c: s.temp_c,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(raw_psi: f64, temp_c: f64, t_us: u64) -> PressureSample {
        PressureSample {
            raw_psi,
            temp_c,
            timestamp_us: t_us,
        }
    }

    #[test]
    fn nominal_pressure_to_depth() {
        // ~10 m of standard seawater ≈ 14.5 PSI gauge.
        let mut p = DepthPipeline::new(SensorCalibration::nominal());
        let est = p.update(sample(14.50, 20.0, 0));
        assert!(
            (est.depth_m - 10.1).abs() < 0.2,
            "10 m ≈ 14.5 PSI, got {} m",
            est.depth_m
        );
    }

    #[test]
    fn density_varies_with_temperature_and_salinity() {
        // Cold water is denser: same pressure reads *shallower*.
        let warm = seawater_density(30.0, 35.0);
        let cold = seawater_density(2.0, 35.0);
        assert!(cold > warm);
        // Brackish water is lighter.
        assert!(seawater_density(20.0, 20.0) < seawater_density(20.0, 38.0));
    }

    #[test]
    fn temperature_drift_is_compensated() {
        // A sensor with 0.01 PSI/°C zero drift reading at 4 °C.
        let cal = SensorCalibration {
            zero_drift_psi_per_c: 0.01,
            cal_temp_c: 20.0,
            ..SensorCalibration::nominal()
        };
        let mut p = DepthPipeline::new(cal);
        let uncompensated = 0.01 * (4.0 - 20.0); // -0.16 PSI would look like shallowing
        let est = p.update(sample(14.50, 4.0, 0));
        let est_ref = {
            let mut p2 = DepthPipeline::new(SensorCalibration::nominal());
            // Same raw pressure, nominal sensor: differs by the drift term.
            p2.update(sample(14.50, 4.0, 0))
        };
        let _ = uncompensated;
        assert!(
            (est.depth_m - est_ref.depth_m).abs() > 0.05,
            "compensation must change the estimate"
        );
    }

    #[test]
    fn lag_correction_pulls_toward_true_depth() {
        // Descending at a steady rate: the lagging sensor under-reads.
        let mut p = DepthPipeline::new(SensorCalibration::nominal());
        // 50 ms lag; vehicle descends at 1 m/s. True depth at t: t·1.0.
        // Simulated sensor = true depth 50 ms ago.
        let mut last = 0.0;
        for k in 1..=10 {
            let t_us = (k * 100_000) as u64; // 10 Hz samples
            let true_depth = k as f64 * 0.1;
            let sensor_psi =
                (true_depth - 0.05) * seawater_density(20.0, 35.0) * G_M_S2 / 6894.757293168;
            let est = p.update(sample(sensor_psi, 20.0, t_us));
            last = est.depth_m;
        }
        // Without the correction the reading would trail by ~0.05 m; with it,
        // the residual is much smaller.
        assert!(
            (last - 1.0).abs() < 0.03,
            "lag-corrected depth {last} should track true 1.0 m"
        );
    }

    #[test]
    fn ascent_reads_consistently() {
        // Ascending profile: raw over-reads; correction pulls down.
        let mut p = DepthPipeline::new(SensorCalibration::nominal());
        let mut first = None;
        let mut last = 0.0;
        for k in 0..10 {
            let t_us = (k * 100_000) as u64;
            let true_depth = 1.0 - k as f64 * 0.1; // rising at 1 m/s
            let sensor_psi =
                (true_depth + 0.05) * seawater_density(20.0, 35.0) / 6894.757293168 * G_M_S2;
            let est = p.update(sample(sensor_psi, 20.0, t_us));
            if first.is_none() {
                first = Some(est.depth_m);
            }
            last = est.depth_m;
        }
        assert!(last < first.unwrap(), "depth must decrease while ascending");
    }
}
