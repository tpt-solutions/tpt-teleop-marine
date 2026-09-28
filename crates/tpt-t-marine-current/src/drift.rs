//! Drift prediction for AUVs and ASVs: collapse an ADCP profile into the
//! transport current at the vehicle's depth band, then project dead-
//! reckoned drift forward (recovery planning, DP feed-forward).

use crate::adcp::AdcpProfile;

/// A water-column current estimate: the depth-averaged transport plus the
/// local current at the vehicle's depth.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CurrentEstimate {
    /// East transport current, m/s (depth-averaged over the profile).
    pub transport_u_mps: f32,
    /// North transport current, m/s.
    pub transport_v_mps: f32,
    /// Local east current at the vehicle depth, m/s.
    pub local_u_mps: f32,
    /// Local north current, m/s.
    pub local_v_mps: f32,
    /// Estimate confidence: fraction of valid bins used (0..1).
    pub confidence: f32,
}

impl CurrentEstimate {
    /// Collapse a QC'd profile into an estimate for a vehicle at
    /// `vehicle_depth_m`. Local current = nearest valid bin; transport =
    /// average over all valid bins.
    pub fn from_profile(p: &AdcpProfile, vehicle_depth_m: f32) -> Self {
        let mut out = Self::default();
        if p.is_empty() {
            return out;
        }
        let mut sum_u = 0f32;
        let mut sum_v = 0f32;
        let mut used = 0usize;
        // Local: nearest valid bin (fallback: nearest any bin).
        let mut best_valid: Option<usize> = None;
        let mut best_any: Option<usize> = None;
        let mut best_valid_dist = f32::MAX;
        let mut best_any_dist = f32::MAX;
        for i in 0..p.len() {
            let b = p.bin(i);
            if b.valid {
                sum_u += b.u_mps;
                sum_v += b.v_mps;
                used += 1;
                let d = (b.depth_m - vehicle_depth_m).abs();
                if d < best_valid_dist {
                    best_valid_dist = d;
                    best_valid = Some(i);
                }
            }
            let d = (b.depth_m - vehicle_depth_m).abs();
            if d < best_any_dist {
                best_any_dist = d;
                best_any = Some(i);
            }
        }
        if used > 0 {
            out.transport_u_mps = sum_u / used as f32;
            out.transport_v_mps = sum_v / used as f32;
            out.confidence = used as f32 / p.len() as f32;
        }
        let local = best_valid.or(best_any);
        if let Some(i) = local {
            let b = p.bin(i);
            out.local_u_mps = b.u_mps;
            out.local_v_mps = b.v_mps;
        }
        out
    }

    /// Local current vector (u, v), m/s.
    pub fn local(&self) -> (f32, f32) {
        (self.local_u_mps, self.local_v_mps)
    }
}

/// Drift projector: where does a vehicle end up under a given current?
#[derive(Debug, Clone, Copy)]
pub struct DriftPredictor {
    /// Position north, metres from the datum.
    pub north_m: f64,
    /// Position east, metres.
    pub east_m: f64,
}

impl DriftPredictor {
    /// Start from a position.
    pub fn at(north_m: f64, east_m: f64) -> Self {
        Self { north_m, east_m }
    }

    /// Project pure drift (vehicle adrift, zero own speed) for `dt_s`
    /// seconds under the local current. Returns the predicted position.
    pub fn drift(&self, est: &CurrentEstimate, dt_s: f64) -> (f64, f64) {
        (
            self.north_m + est.local_v_mps as f64 * dt_s,
            self.east_m + est.local_u_mps as f64 * dt_s,
        )
    }

    /// Project a *transiting* vehicle: dead-reckoned motion plus drift.
    /// `speed_ned = (vn, ve)` is the vehicle's water-relative speed.
    pub fn transit(
        &self,
        est: &CurrentEstimate,
        vn_mps: f32,
        ve_mps: f32,
        dt_s: f64,
    ) -> (f64, f64) {
        (
            self.north_m + (vn_mps + est.local_v_mps) as f64 * dt_s,
            self.east_m + (ve_mps + est.local_u_mps) as f64 * dt_s,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adcp::Bin;

    fn profile() -> AdcpProfile {
        // 0.5 m/s eastward throughout, plus 0.1 north near the surface bin.
        let raw: Vec<Bin> = (0..40)
            .map(|i| {
                let d = i as f32 * 2.0;
                Bin {
                    depth_m: d,
                    u_mps: 0.5,
                    v_mps: if i == 0 { 0.1 } else { 0.0 },
                    amplitude_db: 80.0,
                    valid: false,
                }
            })
            .collect();
        AdcpProfile::ingest(true, &raw) // up-looking: bin 0 invalid (side-lobe)
    }

    #[test]
    fn transport_averages_valid_bins() {
        let est = CurrentEstimate::from_profile(&profile(), 20.0);
        // Bin 0 (v=0.1) is side-lobe-rejected: transport u = 0.5, v ≈ 0.
        assert!((est.transport_u_mps - 0.5).abs() < 1e-6);
        assert!(est.transport_v_mps.abs() < 1e-6);
        assert!((est.confidence - 34.0 / 40.0).abs() < 1e-6);
    }

    #[test]
    fn local_current_picks_nearest_valid_bin() {
        let est = CurrentEstimate::from_profile(&profile(), 3.0);
        // Nearest valid bin to 3 m: bin 2 (4 m), v = 0 (bin 0 rejected).
        assert!(est.local_v_mps.abs() < 1e-6);
        assert!((est.local_u_mps - 0.5).abs() < 1e-6);
    }

    #[test]
    fn drift_projection_moves_downstream() {
        let est = CurrentEstimate::from_profile(&profile(), 20.0);
        let p = DriftPredictor::at(0.0, 0.0);
        let (n, e) = p.drift(&est, 600.0); // 10 minutes adrift
        // 0.5 m/s east for 600 s = 300 m east, no north (bin 0 rejected).
        assert!((e - 300.0).abs() < 1.0);
        assert!(n.abs() < 1.0);
    }

    #[test]
    fn transit_adds_drift_to_own_motion() {
        let est = CurrentEstimate::from_profile(&profile(), 20.0);
        let p = DriftPredictor::at(0.0, 0.0);
        // Vehicle heads north at 1 m/s; current pushes it east of track.
        let (n, e) = p.transit(&est, 1.0, 0.0, 100.0);
        assert!((n - 100.0).abs() < 0.5);
        assert!((e - 50.0).abs() < 0.5, "cross-track drift: e={e}");
    }
}
