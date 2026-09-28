//! Mortality detection: dead fish sink to the cage bottom; clustering the
//! bottom-scan echo targets estimates the loss rate.

/// One bottom echo target.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BottomTarget {
    /// Position on the cage bottom plane, metres (cage frame).
    /// Easting on the bottom plane, metres.
    pub x_m: f32,
    /// Northing on the bottom plane, metres.
    pub y_m: f32,
    /// Target strength, dB (fish-class echo).
    pub ts_db: f32,
}

/// Detection configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MortalityConfig {
    /// Clustering radius for a mortality point, metres.
    pub cluster_radius_m: f32,
    /// Minimum targets per cluster to count as a mortality event.
    pub min_targets: usize,
}

impl MortalityConfig {
    /// Default: 1.5 m clusters of ≥ 3 targets.
    pub const DEFAULT: MortalityConfig = MortalityConfig {
        cluster_radius_m: 1.5,
        min_targets: 3,
    };
}

/// A detected mortality cluster.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MortalityCluster {
    /// Cluster centroid on the bottom plane.
    pub centre_m: [f32; 2],
    /// Estimated count (targets in cluster).
    pub count: usize,
}

/// Cluster targets greedily (single pass, deterministic order) and report
/// clusters above the size threshold.
pub fn detect(targets: &[BottomTarget], cfg: &MortalityConfig) -> Vec<MortalityCluster> {
    let mut used = vec![false; targets.len()];
    let mut out = Vec::new();
    for (i, t) in targets.iter().enumerate() {
        if used[i] {
            continue;
        }
        let mut centre = [t.x_m, t.y_m];
        let mut members = 1usize;
        used[i] = true;
        for (j, u) in targets.iter().enumerate().skip(i + 1) {
            if used[j] {
                continue;
            }
            let dx = u.x_m - centre[0];
            let dy = u.y_m - centre[1];
            if dx * dx + dy * dy <= cfg.cluster_radius_m * cfg.cluster_radius_m {
                used[j] = true;
                members += 1;
                // Running centroid.
                centre[0] += (u.x_m - centre[0]) / members as f32;
                centre[1] += (u.y_m - centre[1]) / members as f32;
            }
        }
        if members >= cfg.min_targets {
            out.push(MortalityCluster {
                centre_m: centre,
                count: members,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(x: f32, y: f32) -> BottomTarget {
        BottomTarget {
            x_m: x,
            y_m: y,
            ts_db: -35.0,
        }
    }

    #[test]
    fn scattered_singles_are_not_mortality() {
        let targets = [t(0.0, 0.0), t(10.0, 0.0), t(20.0, 20.0)];
        assert!(detect(&targets, &MortalityConfig::DEFAULT).is_empty());
    }

    #[test]
    fn a_cluster_of_five_is_detected() {
        let targets = [
            t(0.0, 0.0),
            t(0.5, 0.2),
            t(-0.4, 0.3),
            t(0.3, -0.5),
            t(0.9, 0.4),
            t(50.0, 50.0), // unrelated healthy-water echo
        ];
        let clusters = detect(&targets, &MortalityConfig::DEFAULT);
        assert_eq!(clusters.len(), 1);
        assert_eq!(clusters[0].count, 5);
        assert!(clusters[0].centre_m[0].abs() < 0.3);
    }

    #[test]
    fn loss_rate_from_cluster_count() {
        // A site with 2 clusters × ~4 fish out of a 100k cage — the
        // operator sees the trend, not the absolute.
        let targets = [
            t(0.0, 0.0),
            t(0.2, 0.0),
            t(0.0, 0.2),
            t(0.2, 0.2),
            t(30.0, 30.0),
            t(30.2, 30.0),
            t(30.0, 30.2),
            t(30.2, 30.2),
        ];
        let clusters = detect(&targets, &MortalityConfig::DEFAULT);
        assert_eq!(clusters.iter().map(|c| c.count).sum::<usize>(), 8);
    }
}
