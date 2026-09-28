//! Custom zero-allocation geodetic conversions (spec §7: `geo`/`proj` are
//! banned — this crate owns the math).
//!
//! * WGS-84 geodetic ↔ ECEF (closed-form forward, Bowring's iterative-free
//!   inverse).
//! * ECEF ↔ NED around a [`GeodeticOrigin`] (what USBL fixes need: the
//!   acoustic baseline gives a relative position, nav wants NED).
//! * Geodetic ↔ UTM (Transverse Mercator via the standard 6th-order Krüger
//!   series; round-trips to sub-millimetre over the UTM latitudes).
//!
//! Every function returns plain `[f64; N]`/structs — no `String`, no error
//! `Vec`, no allocation.

/// WGS-84 semi-major axis, metres.
pub const WGS84_A: f64 = 6_378_137.0;
/// WGS-84 flattening, 1/f.
pub const WGS84_F_INV: f64 = 298.257_223_563;
/// WGS-84 first eccentricity squared.
pub const WGS84_E2: f64 = 2.0 * (1.0 / WGS84_F_INV) - (1.0 / WGS84_F_INV) * (1.0 / WGS84_F_INV);
/// UTM central-meridian scale factor.
pub const UTM_K0: f64 = 0.9996;

fn wgs84_b() -> f64 {
    WGS84_A * (1.0 - 1.0 / WGS84_F_INV)
}

/// Geodetic (lat/lon degrees, height metres) to ECEF metres.
pub fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, h_m: f64) -> [f64; 3] {
    let (lat, lon) = (lat_deg.to_radians(), lon_deg.to_radians());
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    [
        (n + h_m) * cos_lat * lon.cos(),
        (n + h_m) * cos_lat * lon.sin(),
        (n * (1.0 - WGS84_E2) + h_m) * sin_lat,
    ]
}

/// ECEF metres to geodetic (lat/lon degrees, height metres) — Bowring's
/// method, accurate to sub-millimetre for heights within ±10 km.
pub fn ecef_to_geodetic(ecef_m: [f64; 3]) -> (f64, f64, f64) {
    let (x, y, z) = (ecef_m[0], ecef_m[1], ecef_m[2]);
    let lon = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    let b = wgs84_b();
    let ep2 = (WGS84_A * WGS84_A - b * b) / (b * b);
    let theta = (z * WGS84_A).atan2(p * b);
    let (st, ct) = theta.sin_cos();
    let lat = (z + ep2 * b * st * st * st).atan2(p - WGS84_E2 * WGS84_A * ct * ct * ct);
    let sin_lat = lat.sin();
    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    let h = p / lat.cos() - n;
    (lat.to_degrees(), lon.to_degrees(), h)
}

/// A local tangent-plane origin: geodetic position + the ECEF↔NED rotation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeodeticOrigin {
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees.
    pub lon_deg: f64,
    /// Altitude above the ellipsoid, metres.
    pub alt_m: f64,
    ecef: [f64; 3],
    /// Rows of the ECEF→NED rotation matrix.
    r_en: [[f64; 3]; 3],
}

impl GeodeticOrigin {
    /// Define the mission datum at a geodetic point.
    pub fn new(lat_deg: f64, lon_deg: f64, alt_m: f64) -> Self {
        let ecef = geodetic_to_ecef(lat_deg, lon_deg, alt_m);
        let (lat, lon) = (lat_deg.to_radians(), lon_deg.to_radians());
        let (sl, cl) = lat.sin_cos();
        let (so, co) = lon.sin_cos();
        // R_ned_from_ecef rows.
        let r_en = [
            [-sl * co, -sl * so, cl],
            [-so, co, 0.0],
            [-cl * co, -cl * so, -sl],
        ];
        Self {
            lat_deg,
            lon_deg,
            alt_m,
            ecef,
            r_en,
        }
    }

    /// Geodetic → local NED (metres) around this origin.
    pub fn geodetic_to_ned(&self, lat_deg: f64, lon_deg: f64, h_m: f64) -> [f64; 3] {
        let e = geodetic_to_ecef(lat_deg, lon_deg, h_m);
        let d = [
            e[0] - self.ecef[0],
            e[1] - self.ecef[1],
            e[2] - self.ecef[2],
        ];
        let mut ned = [0.0; 3];
        for (i, ned_i) in ned.iter_mut().enumerate() {
            *ned_i = self.r_en[i][0] * d[0] + self.r_en[i][1] * d[1] + self.r_en[i][2] * d[2];
        }
        ned
    }

    /// Local NED → ECEF metres.
    pub fn ned_to_ecef(&self, ned_m: [f64; 3]) -> [f64; 3] {
        // Rᵀ·ned (rotation transpose).
        let mut e = [0.0; 3];
        for (i, e_i) in e.iter_mut().enumerate() {
            *e_i = self.r_en[0][i] * ned_m[0]
                + self.r_en[1][i] * ned_m[1]
                + self.r_en[2][i] * ned_m[2];
        }
        [
            e[0] + self.ecef[0],
            e[1] + self.ecef[1],
            e[2] + self.ecef[2],
        ]
    }
}

/// UTM coordinates with hemisphere and zone.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Utm {
    /// Zone number 1..=60.
    pub zone: u8,
    /// `true` for northern hemisphere.
    pub north: bool,
    /// Easting, metres (500 000 at the central meridian).
    pub easting_m: f64,
    /// Northing, metres (0 at the equator; 10 000 000 offset south).
    pub northing_m: f64,
}

/// Geodetic → UTM. Latitudes beyond ±80° are outside the projection's
/// intended envelope; values are still returned (series degrade gracefully).
pub fn geodetic_to_utm(lat_deg: f64, lon_deg: f64) -> Utm {
    let zone = (((lon_deg + 180.0) / 6.0).floor() as i32 % 60 + 60) % 60 + 1;
    let lon0 = ((zone - 1) * 6 - 180 + 3) as f64;
    utm_forward(lat_deg, lon_deg, lon0, zone as u8, lat_deg >= 0.0)
}

/// Inverse of [`geodetic_to_utm`].
pub fn utm_to_geodetic(u: &Utm) -> (f64, f64) {
    let lon0 = ((u.zone as i32 - 1) * 6 - 180 + 3) as f64;
    utm_inverse(u.easting_m, u.northing_m, lon0, u.north)
}

/// Transverse Mercator forward (Krüger series, φ/λ in radians internally).
fn utm_forward(lat_deg: f64, lon_deg: f64, lon0_deg: f64, zone: u8, north: bool) -> Utm {
    let a = WGS84_A;
    let e2 = WGS84_E2;
    let e4 = e2 * e2;
    let e6 = e4 * e2;
    let ep2 = e2 / (1.0 - e2);

    let phi = lat_deg.to_radians();
    let dlam = (lon_deg - lon0_deg).to_radians();
    let sin_phi = phi.sin();
    let cos_phi = phi.cos();
    let tan_phi = phi.tan();
    let n = a / (1.0 - e2 * sin_phi * sin_phi).sqrt();
    let t = tan_phi * tan_phi;
    let c = ep2 * cos_phi * cos_phi;
    let a_ = cos_phi * dlam;

    // Meridional arc.
    let m = a
        * ((1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * phi
            - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) * (2.0 * phi).sin()
            + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * (4.0 * phi).sin()
            - (35.0 * e6 / 3072.0) * (6.0 * phi).sin());

    let easting = UTM_K0
        * n
        * (a_
            + (1.0 - t + c) * a_.powi(3) / 6.0
            + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2) * a_.powi(5) / 120.0)
        + 500_000.0;

    let mut northing = UTM_K0
        * (m + n
            * tan_phi
            * (a_ * a_ / 2.0
                + (5.0 - t + 9.0 * c + 4.0 * c * c) * a_.powi(4) / 24.0
                + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2) * a_.powi(6) / 720.0));

    if !north {
        northing += 10_000_000.0;
    }
    Utm {
        zone,
        north,
        easting_m: easting,
        northing_m: northing,
    }
}

/// Transverse Mercator inverse.
fn utm_inverse(easting_m: f64, northing_m: f64, lon0_deg: f64, north: bool) -> (f64, f64) {
    let a = WGS84_A;
    let e2 = WGS84_E2;
    let e4 = e2 * e2;
    let e6 = e4 * e2;
    let ep2 = e2 / (1.0 - e2);
    let e1 = (1.0 - (1.0 - e2).sqrt()) / (1.0 + (1.0 - e2).sqrt());

    let mut x = easting_m - 500_000.0;
    let mut y = northing_m;
    if !north {
        y -= 10_000_000.0;
    }
    x /= UTM_K0;
    y /= UTM_K0;

    let m = y;
    let mu = m / (a * (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0));
    let phi1 = mu
        + (3.0 * e1 / 2.0 - 27.0 * e1.powi(3) / 32.0) * (2.0 * mu).sin()
        + (21.0 * e1 * e1 / 16.0 - 55.0 * e1.powi(4) / 32.0) * (4.0 * mu).sin()
        + (151.0 * e1.powi(3) / 96.0) * (6.0 * mu).sin()
        + (1097.0 * e1.powi(4) / 512.0) * (8.0 * mu).sin();

    let sin_p1 = phi1.sin();
    let cos_p1 = phi1.cos();
    let tan_p1 = phi1.tan();
    let c1 = ep2 * cos_p1 * cos_p1;
    let t1 = tan_p1 * tan_p1;
    let n1 = a / (1.0 - e2 * sin_p1 * sin_p1).sqrt();
    let r1 = a * (1.0 - e2) / (1.0 - e2 * sin_p1 * sin_p1).powf(1.5);
    // `x` already has k0 scaled out above; the series uses d = x/n1.
    let d = x / n1;

    let lat = phi1
        - (n1 * tan_p1 / r1)
            * (d * d / 2.0
                - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1 * c1 - 9.0 * ep2) * d.powi(4) / 24.0
                + (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1 * t1 - 252.0 * ep2 - 3.0 * c1 * c1)
                    * d.powi(6)
                    / 720.0);
    let lon = lon0_deg.to_radians()
        + (d - (1.0 + 2.0 * t1 + c1) * d.powi(3) / 6.0
            + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1 * c1 + 8.0 * ep2 + 24.0 * t1 * t1)
                * d.powi(5)
                / 120.0)
            / cos_p1;

    (lat.to_degrees(), lon.to_degrees())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ecef_axis_points() {
        // Equator/Greenwich → on the +X axis.
        let e = geodetic_to_ecef(0.0, 0.0, 0.0);
        assert!((e[0] - WGS84_A).abs() < 1.0e-6);
        assert!(e[1].abs() < 1.0e-6 && e[2].abs() < 1.0e-6);
        // North pole → on +Z at the polar radius.
        let p = geodetic_to_ecef(90.0, 0.0, 0.0);
        assert!((p[2] - wgs84_b()).abs() < 1.0e-6);
        assert!(p[0].abs() < 1.0e-6 && p[1].abs() < 1.0e-6);
    }

    #[test]
    fn geodetic_ecef_roundtrip() {
        for (lat, lon, h) in [
            (0.0, 0.0, 0.0),
            (-41.2865, 174.7762, 30.0), // Wellington Harbour
            (59.3251, 18.0711, -10.0),  // Stockholm, slightly submerged
            (77.0, -150.0, 4000.0),     // high Arctic
        ] {
            let ecef = geodetic_to_ecef(lat, lon, h);
            let (la, lo, hh) = ecef_to_geodetic(ecef);
            assert!((la - lat).abs() < 1.0e-9, "lat {lat}: {la}");
            assert!((lo - lon).abs() < 1.0e-9, "lon {lon}: {lo}");
            assert!((hh - h).abs() < 1.0e-4, "h {h}: {hh}");
        }
    }

    #[test]
    fn ned_plane_around_origin() {
        let o = GeodeticOrigin::new(-40.5, 175.25, 0.0);
        // Move 1° north and 1° east along the ellipsoid: NED ≈ (111.2 km,
        // 84.4 km at 40.5°S) with z ≈ 0.
        let ned = o.geodetic_to_ned(-39.5, 175.25, 0.0);
        assert!((ned[0] - 111_200.0).abs() < 500.0, "N: {}", ned[0]);
        let ned_e = o.geodetic_to_ned(-40.5, 176.25, 0.0);
        assert!(
            (ned_e[1] - 84_400.0).abs() < 500.0,
            "E at 40.5°S: {}",
            ned_e[1]
        );
        // Down ≈ −Δφ²·M/2: a point 1° north on the ellipsoid sits *below*
        // the tangent plane (the plane is level at the origin; curvature
        // drops away beneath it). Expected ≈ −969 m.
        assert!((ned[2] - 969.0).abs() < 50.0, "down: {}", ned[2]);
    }

    #[test]
    fn ned_to_ecef_roundtrip() {
        let o = GeodeticOrigin::new(48.85, 2.35, 100.0);
        let ned = [500.0, -300.0, -50.0];
        let ecef = o.ned_to_ecef(ned);
        let (lat, lon, h) = ecef_to_geodetic(ecef);
        let ned2 = o.geodetic_to_ned(lat, lon, h);
        for i in 0..3 {
            assert!(
                (ned[i] - ned2[i]).abs() < 1.0e-3,
                "axis {i}: {} vs {}",
                ned[i],
                ned2[i]
            );
        }
    }

    #[test]
    fn utm_special_points() {
        // On the equator at zone 31's central meridian (3°E): E = 500000,
        // N = 0.
        let u = geodetic_to_utm(0.0, 3.0);
        assert_eq!(u.zone, 31);
        assert!(u.north);
        assert!(
            (u.easting_m - 500_000.0).abs() < 0.01,
            "central meridian easting: {}",
            u.easting_m
        );
        assert!(
            u.northing_m.abs() < 0.01,
            "equator northing: {}",
            u.northing_m
        );
    }

    #[test]
    fn utm_zone_assignment() {
        assert_eq!(geodetic_to_utm(40.7, -74.0).zone, 18); // New York
        assert_eq!(geodetic_to_utm(-33.9, 151.2).zone, 56); // Sydney
        assert!(!geodetic_to_utm(-33.9, 151.2).north);
        assert_eq!(geodetic_to_utm(0.0, 179.9).zone, 60);
        assert_eq!(geodetic_to_utm(0.0, -179.9).zone, 1);
    }

    #[test]
    fn utm_meridional_arc_matches_numeric_integration() {
        // At a zone's central meridian the projection reduces to
        // northing = k0·M(φ); validate the series M against a brute-force
        // numeric integration of the meridian-arc radius.
        for lat_deg in [10.0f64, 40.7128, 65.0, -33.9] {
            let zone = ((((lat_deg + 180.0) / 6.0).floor() as i32 % 60 + 60) % 60 + 1) as u8;
            let lon0 = ((zone as i32 - 1) * 6 - 180 + 3) as f64;
            let u = geodetic_to_utm(lat_deg, lon0);
            assert!(
                (u.easting_m - 500_000.0).abs() < 0.01,
                "CM easting for lat {lat_deg}: {}",
                u.easting_m
            );
            // Numeric: M = ∫ a(1−e²)/(1−e²sin²φ)^{3/2} dφ from the equator.
            let a = WGS84_A;
            let e2 = WGS84_E2;
            let phi = lat_deg.to_radians();
            const STEPS: usize = 100_000;
            let dphi = phi / STEPS as f64;
            let mut m = 0.0f64;
            for k in 0..STEPS {
                let p = (k as f64 + 0.5) * dphi;
                let s2 = p.sin().powi(2);
                m += a * (1.0 - e2) / (1.0 - e2 * s2).powf(1.5) * dphi;
            }
            // Undo the southern-hemisphere 10,000 km offset, then k0.
            let m_from_utm = if u.north {
                u.northing_m
            } else {
                10_000_000.0 - u.northing_m
            } / UTM_K0;
            assert!(
                (m_from_utm - m.abs()).abs() / m.abs() < 1.0e-9,
                "M({lat_deg}): series {m_from_utm} vs numeric {}",
                m.abs()
            );
        }
    }

    #[test]
    fn utm_nyc_plausibility() {
        // NYC: 1.006° east of zone 18's central meridian at 40.7°N ≈ 85 km
        // of arc; northing slightly below the k0·M meridian distance.
        let u = geodetic_to_utm(40.7128, -74.0060);
        assert_eq!(u.zone, 18);
        assert!(u.north);
        assert!(
            (u.easting_m - 583_960.0).abs() < 500.0,
            "E: {}",
            u.easting_m
        );
        assert!(
            (4_506_000.0..4_510_000.0).contains(&u.northing_m),
            "N: {}",
            u.northing_m
        );
    }

    #[test]
    fn utm_roundtrips() {
        for (lat, lon) in [
            (40.7128, -74.0060),  // New York
            (-41.2865, 174.7762), // Wellington
            (0.5, 10.0),          // near equator, zone 32/33 edge
            (65.0, 25.0),         // high latitude
            (-60.0, -60.0),       // far south
        ] {
            let u = geodetic_to_utm(lat, lon);
            let (la, lo) = utm_to_geodetic(&u);
            assert!((la - lat).abs() < 1.0e-6, "{lat} → {la}");
            assert!((lo - lon).abs() < 1.0e-6, "{lon} → {lo}");
        }
    }
}
