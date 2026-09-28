//! Fish-cage net inspection: a lawnmower grid over each cage wall, with
//! tear/deformation flags where the sonar anomaly score breaches limits.

/// One net-inspection cell verdict.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NetCell {
    /// Cell id (row-major over the wall grid).
    pub id: u16,
    /// Anomaly score (0..6σ from the normalized sonar envelope).
    pub anomaly_sigma: f32,
    /// Whether the cell is flagged (tear or heavy fouling).
    pub flagged: bool,
}

/// Inspection configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NetInspectionConfig {
    /// Sigma threshold for a tear flag.
    pub tear_sigma: f32,
    /// Fouling threshold (lower — biofouling dims the echo pattern).
    pub fouling_sigma: f32,
}

impl NetInspectionConfig {
    /// Default thresholds.
    pub const DEFAULT: NetInspectionConfig = NetInspectionConfig {
        tear_sigma: 4.0,
        fouling_sigma: 2.5,
    };
}

/// The net inspector: collects per-cell scores and emits a wall report.
#[derive(Debug, Clone, Copy)]
pub struct NetInspector {
    cfg: NetInspectionConfig,
    cells: [NetCell; 256],
    n: usize,
}

impl Default for NetInspector {
    fn default() -> Self {
        Self::new(NetInspectionConfig::DEFAULT)
    }
}

impl NetInspector {
    /// Empty inspector.
    pub fn new(cfg: NetInspectionConfig) -> Self {
        Self {
            cfg,
            cells: [NetCell {
                id: 0,
                anomaly_sigma: 0.0,
                flagged: false,
            }; 256],
            n: 0,
        }
    }

    /// Record one cell's score.
    pub fn observe(&mut self, id: u16, anomaly_sigma: f32) {
        assert!(self.n < 256, "grid full");
        let flagged = anomaly_sigma >= self.cfg.fouling_sigma;
        self.cells[self.n] = NetCell {
            id,
            anomaly_sigma,
            flagged,
        };
        self.n += 1;
    }

    /// Cells inspected so far.
    pub fn inspected(&self) -> usize {
        self.n
    }

    /// Count of flagged cells (fouling + tears).
    pub fn flagged(&self) -> usize {
        self.cells[..self.n].iter().filter(|c| c.flagged).count()
    }

    /// Cells flagged as probable tears (≥ tear sigma).
    pub fn tears(&self) -> impl Iterator<Item = &NetCell> {
        self.cells[..self.n]
            .iter()
            .filter(|c| c.anomaly_sigma >= self.cfg.tear_sigma)
    }

    /// The worst cell, if any.
    pub fn worst(&self) -> Option<&NetCell> {
        self.cells[..self.n]
            .iter()
            .max_by(|a, b| a.anomaly_sigma.total_cmp(&b.anomaly_sigma))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quiet_net_flags_nothing() {
        let mut insp = NetInspector::new(NetInspectionConfig::DEFAULT);
        for id in 0..64u16 {
            insp.observe(id, 0.5);
        }
        assert_eq!(insp.flagged(), 0);
        assert_eq!(insp.inspected(), 64);
    }

    #[test]
    fn fouling_and_tears_separate() {
        let mut insp = NetInspector::new(NetInspectionConfig::DEFAULT);
        insp.observe(0, 2.8); // fouling
        insp.observe(1, 4.5); // tear
        insp.observe(2, 0.4);
        assert_eq!(insp.flagged(), 2);
        assert_eq!(insp.tears().count(), 1);
        assert_eq!(insp.worst().unwrap().id, 1);
    }
}
