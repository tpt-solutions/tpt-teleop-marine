//! ADCP (Acoustic Doppler Current Profiler) ingestion with quality
//! control.
//!
//! An ADCP reports one velocity vector per depth bin. Two standard QC
//! steps are applied at ingestion:
//!
//! * **Side-lobe rejection** — the outermost bins near the surface (for an
//!   up-looking unit) or the bottom (down-looking) are contaminated by
//!   strong side-lobe reflections; a fixed fraction (default 15 %) of the
//!   profile is rejected at each end.
//! * **Outlier rejection** — bins whose speed deviates from the profile
//!   median by more than [`MAX_BIN_DEVIATION_MPS`] are marked invalid
//!   (fish, debris, and aliasing all show up as single-bin spikes).

/// Maximum deviation from the profile median before a bin is rejected.
pub const MAX_BIN_DEVIATION_MPS: f32 = 0.5;

/// Default side-lobe rejection fraction, of the profile length, per end.
pub const SIDE_LOBE_FRACTION: f32 = 0.15;

/// Maximum depth bins per profile (fixed, zero-alloc).
pub const MAX_BINS: usize = 64;

/// One velocity bin.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Bin {
    /// Bin centre depth below the instrument, metres (positive down).
    pub depth_m: f32,
    /// East velocity, m/s.
    pub u_mps: f32,
    /// North velocity, m/s.
    pub v_mps: f32,
    /// Backscatter amplitude, dB (payload data, not used for currents).
    pub amplitude_db: f32,
    /// QC verdict for this bin.
    pub valid: bool,
}

/// One ADCP profile, ingested and QC'd. Fixed capacity.
#[derive(Debug, Clone, Copy)]
pub struct AdcpProfile {
    bins: [Bin; MAX_BINS],
    n: usize,
    /// Instrument orientation: `true` = up-looking.
    up_facing: bool,
}

impl AdcpProfile {
    /// Ingest raw bins with QC applied. Bins are ordered from the
    /// instrument outward (depth increasing).
    pub fn ingest(up_facing: bool, raw: &[Bin]) -> Self {
        assert!(raw.len() <= MAX_BINS, "profile exceeds MAX_BINS");
        let mut bins = [Bin::default(); MAX_BINS];
        bins[..raw.len()].copy_from_slice(raw);
        let n = raw.len();

        // Median speed for the outlier test (median of |(u,v)| per bin).
        let mut speeds = [0f32; MAX_BINS];
        for (i, b) in raw.iter().enumerate() {
            speeds[i] = (b.u_mps * b.u_mps + b.v_mps * b.v_mps).sqrt();
        }
        speeds[..n].sort_by(|a, b| a.total_cmp(b));
        let median = if n > 0 { speeds[n / 2] } else { 0.0 };

        // Side-lobe margins: reject the outer `fraction` at the instrument-
        // facing end (first bins if up-looking means bins go downward? An
        // up-looking unit's bins approach the surface at LOW indices).
        let side = (n as f32 * SIDE_LOBE_FRACTION) as usize;
        for (i, b) in bins[..n].iter_mut().enumerate() {
            let mut valid = true;
            if up_facing && i < side {
                valid = false; // surface side-lobe zone
            }
            if !up_facing && i >= n - side {
                valid = false; // bottom side-lobe zone
            }
            let s = (b.u_mps * b.u_mps + b.v_mps * b.v_mps).sqrt();
            if (s - median).abs() > MAX_BIN_DEVIATION_MPS {
                valid = false;
            }
            b.valid = valid;
        }

        Self { bins, n, up_facing }
    }

    /// Number of bins.
    pub fn len(&self) -> usize {
        self.n
    }

    /// Whether the profile has no bins.
    pub fn is_empty(&self) -> bool {
        self.n == 0
    }

    /// Bin accessor.
    pub fn bin(&self, i: usize) -> &Bin {
        &self.bins[i]
    }

    /// Instrument orientation.
    pub fn up_facing(&self) -> bool {
        self.up_facing
    }

    /// Count of valid bins (post-QC).
    pub fn valid_bins(&self) -> usize {
        self.bins[..self.n].iter().filter(|b| b.valid).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bin(d: f32, u: f32, v: f32) -> Bin {
        Bin {
            depth_m: d,
            u_mps: u,
            v_mps: v,
            amplitude_db: 80.0,
            valid: false,
        }
    }

    #[test]
    fn uniform_profile_survives_qc_except_side_lobes() {
        // Down-looking 40-bin profile, uniform 0.2 m/s.
        let raw: Vec<Bin> = (0..40).map(|i| bin(i as f32 * 2.0, 0.2, 0.0)).collect();
        let p = AdcpProfile::ingest(false, &raw);
        // 15 % of 40 = 6 bins rejected at the bottom end only.
        assert_eq!(p.valid_bins(), 34);
        assert!(p.bin(0).valid);
        assert!(!p.bin(39).valid);
    }

    #[test]
    fn outlier_bin_is_rejected() {
        let mut raw: Vec<Bin> = (0..40).map(|i| bin(i as f32 * 2.0, 0.2, 0.0)).collect();
        raw[20].u_mps = 2.5; // a fish at 40 m
        let p = AdcpProfile::ingest(false, &raw);
        assert!(!p.bin(20).valid, "outlier rejected");
        assert!(p.bin(19).valid, "neighbours untouched");
        assert_eq!(p.valid_bins(), 33); // 34 − 1 outlier
    }

    #[test]
    fn up_facing_rejects_the_other_end() {
        let raw: Vec<Bin> = (0..40).map(|i| bin(i as f32 * 2.0, 0.2, 0.0)).collect();
        let p = AdcpProfile::ingest(true, &raw);
        assert!(!p.bin(0).valid, "surface bins rejected for up-looking");
        assert!(p.bin(39).valid);
    }
}
