//! Sidescan and forward-looking envelope processing: time-varied-gain
//! (TVG) compensation and sliding-window normalization over fixed sample
//! windows.
//!
//! A sidescan channel is one echo envelope per channel (port/starboard);
//! a forward-looking sonar produces one per beam. The same fixed-window
//! normalization serves both.

/// Samples per channel (fixed window, zero-alloc).
pub const SAMPLES: usize = 1024;

/// One echo envelope.
#[derive(Debug, Clone, Copy)]
pub struct Envelope {
    /// Per-sample amplitude, dB re 1 µPa (or normalized units).
    pub samples: [f32; SAMPLES],
    /// Samples-per-metre (ground speed × ping rate decides this at
    /// acquisition); 0 = unknown.
    pub samples_per_m: f32,
}

impl Default for Envelope {
    fn default() -> Self {
        Self {
            samples: [0.0; SAMPLES],
            samples_per_m: 0.0,
        }
    }
}

/// Apply spreading + absorption TVG: `s' = s + 20·log10(r) + 2·α·r/1000`
/// with r the slant range of each sample. `alpha_db_per_km` is the
/// absorption coefficient (seawater ≈ 50 dB/km at 400 kHz).
pub fn apply_tvg(env: &mut Envelope, samples_per_m: f32, alpha_db_per_km: f32) {
    env.samples_per_m = samples_per_m;
    if samples_per_m <= 0.0 {
        return;
    }
    for (i, s) in env.samples.iter_mut().enumerate() {
        let r = (i + 1) as f32 / samples_per_m;
        let gain = 20.0 * r.max(1.0).log10() + 2.0 * alpha_db_per_km * r / 1000.0;
        *s += gain;
    }
}

/// Sliding-window normalization: subtract the windowed mean, clamp at 0,
/// rescale by the running std — flattens bottom type and highlights
/// anomalies (spec's aquaculture/offshore anomaly detection consumes this).
pub fn normalize(env: &mut Envelope, window: usize) {
    assert!(window > 0 && window <= SAMPLES);
    let half = window / 2;
    let mut out = [0f32; SAMPLES];
    for i in 0..SAMPLES {
        let lo = i.saturating_sub(half);
        let hi = (i + half).min(SAMPLES - 1) + 1;
        let seg = &env.samples[lo..hi];
        let n = seg.len() as f32;
        let mean = seg.iter().sum::<f32>() / n;
        let var = seg.iter().map(|s| (s - mean) * (s - mean)).sum::<f32>() / n;
        let std = var.max(1.0e-6).sqrt();
        out[i] = ((env.samples[i] - mean) / std).clamp(0.0, 6.0);
    }
    env.samples = out;
}

/// Count samples above a normalized threshold — the anomaly score fed to
/// the edge-AI / inspector view.
pub fn anomaly_score(env: &Envelope, threshold: f32) -> usize {
    env.samples.iter().filter(|&&s| s >= threshold).count()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_envelope(level: f32) -> Envelope {
        Envelope {
            samples: [level; SAMPLES],
            samples_per_m: 0.0,
        }
    }

    #[test]
    fn tvg_raises_far_samples() {
        let mut e = flat_envelope(-60.0);
        apply_tvg(&mut e, 10.0, 50.0);
        assert!(e.samples[0] < e.samples[SAMPLES / 2]);
        assert!(e.samples[SAMPLES / 2] < e.samples[SAMPLES - 1]);
    }

    #[test]
    fn normalization_flattens_uniform_bottom() {
        let mut e = flat_envelope(-40.0);
        apply_tvg(&mut e, 10.0, 50.0);
        normalize(&mut e, 64);
        // A smooth ramp normalizes to small residuals: few anomalies.
        assert!(anomaly_score(&e, 4.0) < SAMPLES / 10);
    }

    #[test]
    fn anomaly_stands_out_after_normalization() {
        let mut e = flat_envelope(-40.0);
        apply_tvg(&mut e, 10.0, 50.0);
        // A mooring chain at sample 800.
        e.samples[800] += 25.0;
        normalize(&mut e, 64);
        assert!(anomaly_score(&e, 4.0) >= 1, "the chain must flag");
    }

    #[test]
    fn zero_spm_tvg_is_noop() {
        let mut e = flat_envelope(-30.0);
        apply_tvg(&mut e, 0.0, 50.0);
        assert_eq!(e.samples[100], -30.0);
    }
}
