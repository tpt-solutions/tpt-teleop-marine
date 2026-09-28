//! Subsea cable routing: follow a route plan (station list) and assess
//! burial depth at stations from the vehicle's altitude/depth telemetry.

/// One route station.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteStation {
    /// Station north, metres (datum frame).
    pub north_m: f64,
    /// Station east, metres.
    pub east_m: f64,
    /// Nominal burial depth, metres (0 = exposed on the seabed).
    pub nominal_burial_m: f32,
}

/// Burial assessment for one station.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BurialAssessment {
    /// Measured burial depth (seabed − cable top), metres; negative =
    /// exposed/above seabed.
    pub burial_m: f32,
    /// Whether the station passes its burial requirement.
    pub ok: bool,
}

/// The router: walks a station list and tracks progress.
#[derive(Debug, Clone, Copy)]
pub struct CableRouter {
    stations: [RouteStation; 64],
    n: usize,
    /// Index of the next station to visit.
    pub next: usize,
    /// Burial requirement, metres (stations below this pass).
    pub required_burial_m: f32,
}

impl CableRouter {
    /// Build a router from a station list (≤ 64 stations).
    pub fn new(stations: &[RouteStation], required_burial_m: f32) -> Self {
        assert!(stations.len() <= 64, "route ≤ 64 stations");
        let mut s = [RouteStation {
            north_m: 0.0,
            east_m: 0.0,
            nominal_burial_m: 0.0,
        }; 64];
        s[..stations.len()].copy_from_slice(stations);
        Self {
            stations: s,
            n: stations.len(),
            next: 0,
            required_burial_m,
        }
    }

    /// Stations remaining.
    pub fn remaining(&self) -> usize {
        self.n - self.next
    }

    /// The next station to visit.
    pub fn peek(&self) -> Option<RouteStation> {
        self.stations.get(self.next).copied()
    }

    /// Record the assessment at the current station; advances.
    pub fn assess(&mut self, burial_m: f32) -> BurialAssessment {
        let a = BurialAssessment {
            burial_m,
            ok: burial_m >= self.required_burial_m,
        };
        self.next += 1;
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn route() -> Vec<RouteStation> {
        (0..5)
            .map(|k| RouteStation {
                north_m: k as f64 * 100.0,
                east_m: 0.0,
                nominal_burial_m: 1.0,
            })
            .collect()
    }

    #[test]
    fn walks_the_route_in_order() {
        let mut r = CableRouter::new(&route(), 0.8);
        assert_eq!(r.remaining(), 5);
        let first = r.peek().unwrap();
        assert!((first.north_m - 0.0).abs() < 1e-9);
        let a = r.assess(1.2);
        assert!(a.ok);
        assert_eq!(r.remaining(), 4);
        assert!((r.peek().unwrap().north_m - 100.0).abs() < 1e-9);
    }

    #[test]
    fn exposed_cable_fails_the_requirement() {
        let mut r = CableRouter::new(&route(), 0.8);
        let a = r.assess(-0.1); // cable proud of the seabed
        assert!(!a.ok, "exposed cable must fail");
    }
}
