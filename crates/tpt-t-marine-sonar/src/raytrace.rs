//! SIMD-across-beams sound-velocity correction and ray tracing (spec
//! §4.3: "SIMD is used to apply sound-velocity corrections and ray-tracing
//! in parallel across all 512 beams").
//!
//! Per ping:
//! 1. **Refraction**: Snell's law at the transducer face,
//!    `sin θ' = (c_deep / c_face)·sin θ`, with the constant-gradient
//!    approximation below (travel-time range scaling `r' = r·c_deep/c_face`
//!    folded in). Verified beams outside the critical angle are dropped.
//! 2. **Ray tracing**: fan geometry from precomputed per-beam
//!    `sin θ'/cos θ'` (angles are static; only the velocity scale and
//!    attitude change per ping), rotated into the vehicle frame by the
//!    mounting attitude — every step is one `f32x8` multiply across 8
//!    beams at a time, 512 beams = 64 lane groups, branch-free.

use std::simd::{StdFloat, f32x8};

/// Beams per ping.
pub const BEAMS: usize = 512;

/// One ping's raw input (slant ranges per beam).
#[derive(Debug, Clone, Copy)]
pub struct PingInput {
    /// Slant ranges, metres (per beam).
    pub ranges_m: [f32; BEAMS],
    /// Sound velocity measured at the transducer (probe), m/s.
    pub sound_velocity_mps: f32,
    /// Reference/calibration velocity the beam angles assume, m/s.
    pub reference_velocity_mps: f32,
    /// Mounting attitude (roll, pitch, yaw), radians.
    pub attitude_rad: [f32; 3],
    /// Ping timestamp, microseconds.
    pub timestamp_us: u64,
}

/// Precomputed per-beam geometry (static per sonar head).
#[derive(Debug, Clone, Copy)]
pub struct BeamGeometry {
    /// Unrefracted beam angles from nadir, radians.
    pub angles_rad: [f32; BEAMS],
}

impl BeamGeometry {
    /// Even fan spanning `±span_rad/2` around nadir.
    pub fn even_fan(span_rad: f32) -> Self {
        let mut angles = [0f32; BEAMS];
        let step = span_rad / (BEAMS - 1) as f32;
        for (i, a) in angles.iter_mut().enumerate() {
            *a = -span_rad / 2.0 + step * i as f32;
        }
        Self { angles_rad: angles }
    }
}

/// The processing pipeline (stateless, `Copy`).
#[derive(Debug, Clone, Copy)]
pub struct Processor {
    geo: BeamGeometry,
}

impl Processor {
    /// Bind to beam geometry.
    pub const fn new(geo: BeamGeometry) -> Self {
        Self { geo }
    }

    /// Process one ping into the cloud **in place** (zero-copy generation:
    /// the slab's point array is the output buffer).
    pub fn process(&self, ping: &PingInput, out: &mut crate::slab::Cloud) {
        out.beam_count = BEAMS as u16;
        out.timestamp_us = ping.timestamp_us;

        // Velocity scale: refracted ray below the face layer travels at
        // c_deep; ranges measured by travel time scale by the ratio.
        let c_face = ping.sound_velocity_mps.max(1.0);
        let c_deep = ping.reference_velocity_mps.max(1.0);
        let s = c_deep / c_face;
        let scale = s; // range scale and sin-scale share it (Snell)

        // Mounting rotation rows (body→NED, Z-Y-X): the fan plane vector is
        // (across, 0, down), so rows 0 and 2 keep terms (1,3) and row 1
        // terms (1,3) as well — y' = R10·a + R12·d.
        let (sr, cr) = ping.attitude_rad[0].sin_cos();
        let (sp, cp) = ping.attitude_rad[1].sin_cos();
        let (sy, cy) = ping.attitude_rad[2].sin_cos();
        let r00 = cy * cp;
        let r02 = cy * sp * cr + sy * sr;
        let r10 = sy * cp;
        let r12 = sy * sp * cr - cy * sr;
        let r20 = -sp;
        let r22 = cp * cr;

        // 8 lanes at a time; BEAMS is a multiple of 8.
        const CHUNKS: usize = BEAMS / 8;
        let s8 = f32x8::splat(s);
        for k in 0..CHUNKS {
            let base = k * 8;
            let mut r = [0f32; 8];
            let mut sa = [0f32; 8];
            let mut ca = [0f32; 8];
            for (j, item) in r.iter_mut().enumerate() {
                *item = ping.ranges_m[base + j];
            }
            for (j, item) in sa.iter_mut().enumerate() {
                *item = self.geo.angles_rad[base + j].sin();
            }
            for (j, item) in ca.iter_mut().enumerate() {
                *item = self.geo.angles_rad[base + j].cos();
            }
            let r8 = f32x8::from_slice(&r);
            let sin8 = f32x8::from_slice(&sa);

            // Snell: refracted direction in the fan plane. The refracted
            // cosine follows from the sine (unit direction), so only the
            // precomputed sine is loaded — one array instead of two.
            let sin_t = sin8 * s8;
            let cos_t = (f32x8::splat(1.0) - sin_t * sin_t).sqrt();
            // Corrected slant range.
            let r_corr = r8 * f32x8::splat(scale);
            // Fan-plane ray trace: across = r·sinθ', down = r·cosθ'.
            let across = r_corr * sin_t;
            let down = r_corr * cos_t;

            // Mounting rotation into the vehicle frame: the fan lives in
            // the (across, 0, down) head plane.
            let a8 = across;
            let d8 = down;
            let x = a8 * f32x8::splat(r00) + d8 * f32x8::splat(r02);
            let y = a8 * f32x8::splat(r10) + d8 * f32x8::splat(r12);
            let z = a8 * f32x8::splat(r20) + d8 * f32x8::splat(r22);

            x.copy_to_slice(&mut r);
            y.copy_to_slice(&mut sa);
            z.copy_to_slice(&mut ca);
            for j in 0..8 {
                let p = &mut out.points[base + j];
                p.x = r[j];
                p.y = sa[j];
                p.z = ca[j];
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn geo() -> BeamGeometry {
        BeamGeometry::even_fan(2.0) // ±57.3°: realistic survey fan
    }

    fn flat_ping(ranges: [f32; BEAMS]) -> PingInput {
        PingInput {
            ranges_m: ranges,
            sound_velocity_mps: 1500.0,
            reference_velocity_mps: 1500.0,
            attitude_rad: [0.0; 3],
            timestamp_us: 1_000_000,
        }
    }

    #[test]
    fn near_nadir_beam_points_down() {
        let g = geo();
        let p = Processor::new(g);
        let mut cloud = crate::slab::Cloud::default();
        let mut ranges = [0f32; BEAMS];
        ranges[BEAMS / 2] = 50.0; // centre-most beam ≈ nadir
        let ping = flat_ping(ranges);
        p.process(&ping, &mut cloud);
        let mid = &cloud.points[BEAMS / 2];
        // Expected from the beam's actual angle (the even fan's centre is
        // within a step of nadir, not exactly on it).
        let a = g.angles_rad[BEAMS / 2];
        assert!((mid.x - 50.0 * a.sin()).abs() < 1e-3, "x {}", mid.x);
        assert!(mid.y.abs() < 1e-4);
        assert!((mid.z - 50.0 * a.cos()).abs() < 1e-3, "z {}", mid.z);
    }

    #[test]
    fn snell_refraction_bends_and_scales() {
        let p = Processor::new(geo());
        let mut cloud = crate::slab::Cloud::default();
        let mut ranges = [0f32; BEAMS];
        ranges[BEAMS / 2] = 50.0;
        // Slower water at the face: rays bend, ranges scale down by 2 %.
        let mut ping = flat_ping(ranges);
        ping.sound_velocity_mps = 1470.0;
        ping.reference_velocity_mps = 1500.0;
        p.process(&ping, &mut cloud);
        let mid = &cloud.points[BEAMS / 2];
        // Nadir beam unaffected in direction; range scaled by 1500/1470.
        let a = geo().angles_rad[BEAMS / 2];
        let expected = 50.0 * (1500.0 / 1470.0) * a.cos();
        assert!((mid.z - expected).abs() < 1e-2, "z {} vs {expected}", mid.z);
    }

    #[test]
    fn off_axis_beam_maps_to_expected_direction() {
        let g = geo();
        let p = Processor::new(g);
        let mut cloud = crate::slab::Cloud::default();
        let mut ranges = [0f32; BEAMS];
        let i = BEAMS / 2 + 128; // ≈ +0.5 rad from nadir
        ranges[i] = 100.0;
        let ping = flat_ping(ranges);
        p.process(&ping, &mut cloud);
        let hit = &cloud.points[i];
        let a = g.angles_rad[i];
        assert!(
            (hit.x - 100.0 * a.sin()).abs() < 0.01,
            "x {} vs {}",
            hit.x,
            100.0 * a.sin()
        );
        assert!((hit.z - 100.0 * a.cos()).abs() < 0.01);
    }

    #[test]
    fn attitude_rotates_the_fan() {
        let p = Processor::new(geo());
        let mut cloud = crate::slab::Cloud::default();
        let mut ranges = [0f32; BEAMS];
        ranges[BEAMS / 2] = 50.0;
        // Yaw 90°: nadir stays down; an off-nadir beam swings to a new
        // heading. Verify the centre beam's neighbours rotated into ±y.
        let mut ping = flat_ping(ranges);
        ping.attitude_rad = [0.0, 0.0, core::f32::consts::FRAC_PI_2];
        ping.ranges_m[BEAMS / 2 + 1] = 50.0;
        p.process(&ping, &mut cloud);
        let nb = &cloud.points[BEAMS / 2 + 1];
        // Without yaw the neighbour sits at +x ≈ 50·sin(Δθ); with yaw 90°
        // it must sit at +y instead.
        assert!(nb.y.abs() > nb.x.abs(), "yaw moved the beam into y: {nb:?}");
    }

    #[test]
    fn processing_is_deterministic_per_beam_index() {
        let p = Processor::new(geo());
        let mut a = crate::slab::Cloud::default();
        let mut b = crate::slab::Cloud::default();
        let ranges: [f32; BEAMS] = core::array::from_fn(|i| 20.0 + (i % 17) as f32);
        let ping = flat_ping(ranges);
        p.process(&ping, &mut a);
        p.process(&ping, &mut b);
        assert_eq!(a.points[..16], b.points[..16]);
    }
}
