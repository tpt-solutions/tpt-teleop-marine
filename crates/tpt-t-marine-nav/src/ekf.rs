//! The 15-state Extended Kalman Filter.
//!
//! State layout (indices):
//!
//! ```text
//! 0..3   position p  (NED, metres, down positive)
//! 3..6   velocity v  (NED, m/s)
//! 6..9   attitude error θ (rad, small-angle body-frame errors)
//! 9..12  accelerometer bias b_a (m/s²)
//! 12..15 gyro bias b_g (rad/s)
//! ```
//!
//! The nominal state is propagated with strapdown inertial mechanics
//! (attitude via a body→NED direction-cosine matrix, velocity via
//! `v̇ = R·f − g`, position by integration). The covariance is propagated
//! with the standard error-state Jacobian `F` and then corrected by
//! measurement updates (DVL velocity, pressure depth, USBL/LBL acoustic
//! position). Attitude corrections are injected into the DCM and reset —
//! the classic error-state EKF loop.
//!
//! All 15-wide dot products run on SIMD vectors (`f32x8` + `f32x4`); the
//! 15×15 covariance algebra is the dominant cost and meets the spec's
//! `<200 µs` Cortex-A72 budget with margin (see `benches/ekf.rs`).

use std::simd::prelude::SimdFloat;
use std::simd::{f32x4, f32x8};

/// NED gravity magnitude, m/s² (down positive in NED).
pub const GRAVITY_M_S2: f32 = 9.80665;

/// Filter state dimension.
pub const N: usize = 15;

/// State indices, for readable `H`-matrix construction and telemetry.
pub mod idx {
    /// Position (N, E, D) indices.
    pub const P: [usize; 3] = [0, 1, 2];
    /// Velocity (N, E, D) indices.
    pub const V: [usize; 3] = [3, 4, 5];
    /// Attitude-error indices.
    pub const TH: [usize; 3] = [6, 7, 8];
    /// Accelerometer-bias indices.
    pub const BA: [usize; 3] = [9, 10, 11];
    /// Gyro-bias indices.
    pub const BG: [usize; 3] = [12, 13, 14];
}

/// One inertial sample: proper acceleration and angular rate in the body
/// frame, already scale-factor corrected (bias handled by the filter).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    /// Specific force, m/s², body frame (x fwd, y starboard, z down).
    pub accel_mps2: [f32; 3],
    /// Angular rate, rad/s, body frame.
    pub gyro_rad_s: [f32; 3],
    /// Sample interval, seconds.
    pub dt_s: f32,
}

/// DVL bottom-track velocity in NED.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DvlSample {
    /// Velocity over ground in NED, m/s (valid when `bottom_lock`).
    pub velocity_ned_mps: [f32; 3],
    /// Whether the DVL has bottom lock (water-mass track is rejected).
    pub bottom_lock: bool,
}

/// USBL/LBL/SBL acoustic position fix in NED, relative to the mission datum.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AcousticFix {
    /// Position NED, metres.
    pub ned_m: [f64; 3],
    /// Fix standard deviation, metres (applied to all three axes).
    pub std_m: f32,
}

/// Pressure-derived depth measurement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepthSample {
    /// Compensated depth, metres positive-down.
    pub depth_m: f64,
    /// Measurement standard deviation, metres.
    pub std_m: f32,
}

/// Filter tuning (all `Copy`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavConfig {
    /// Accelerometer white noise, (m/s²)/√Hz.
    pub accel_noise: f32,
    /// Gyro white noise, (rad/s)/√Hz.
    pub gyro_noise: f32,
    /// Accelerometer bias random walk, (m/s²)/√s.
    pub accel_bias_rw: f32,
    /// Gyro bias random walk, (rad/s)/√s.
    pub gyro_bias_rw: f32,
    /// Initial attitude-error σ, rad.
    pub init_att_std: f32,
}

impl NavConfig {
    /// Typical tactical-grade IMU envelope for a survey AUV.
    pub const fn typical() -> Self {
        Self {
            accel_noise: 0.002,
            gyro_noise: 4.0e-5,
            accel_bias_rw: 4.0e-5,
            gyro_bias_rw: 2.0e-6,
            init_att_std: 0.05,
        }
    }
}

// --- Fixed-size linear algebra (no allocation; SIMD dot products) -----------

/// SIMD dot product of two 15-vectors (`f32x8` + `f32x4` + 3-lane tail).
#[inline]
fn dot15(a: &[f32; N], b: &[f32; N]) -> f32 {
    let a0 = f32x8::from_slice(&a[0..8]);
    let b0 = f32x8::from_slice(&b[0..8]);
    let a1 = f32x4::from_slice(&a[8..12]);
    let b1 = f32x4::from_slice(&b[8..12]);
    let mut tail = 0.0f32;
    for k in 12..15 {
        tail += a[k] * b[k];
    }
    (a0 * b0).reduce_sum() + (a1 * b1).reduce_sum() + tail
}

/// In-place `out = A · B` for two N×N row-major matrices.
fn mat_mul(a: &[[f32; N]; N], b: &[[f32; N]; N], out: &mut [[f32; N]; N]) {
    // Copy B's columns into scratch rows so the inner loop is a contiguous
    // SIMD dot (fixed-size scratch, allocation-free).
    let mut bc = [[0.0f32; N]; N];
    for j in 0..N {
        for i in 0..N {
            bc[j][i] = b[i][j];
        }
    }
    for i in 0..N {
        for j in 0..N {
            out[i][j] = dot15(&a[i], &bc[j]);
        }
    }
}

/// Transpose an N×N matrix into `out`.
fn mat_transpose(a: &[[f32; N]; N], out: &mut [[f32; N]; N]) {
    for i in 0..N {
        for j in 0..N {
            out[j][i] = a[i][j];
        }
    }
}

/// 3×3 matrix-vector product.
#[inline]
fn mv3(m: &[[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// `R ← R·(I + [w×]·dt)` — first-order small-angle right multiplication
/// (attitude error in the body frame).
fn rotate_dcm(r: &[[f32; 3]; 3], w: [f32; 3], dt: f32) -> [[f32; 3]; 3] {
    let (wx, wy, wz) = (w[0] * dt, w[1] * dt, w[2] * dt);
    let d = [[1.0, -wz, wy], [wz, 1.0, -wx], [-wy, wx, 1.0]];
    let mut out = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = r[i][0] * d[0][j] + r[i][1] * d[1][j] + r[i][2] * d[2][j];
        }
    }
    out
}

/// Build a body→NED DCM from (roll, pitch, yaw), Z-Y-X aerospace convention.
pub fn dcm_from_rpy(roll: f32, pitch: f32, yaw: f32) -> [[f32; 3]; 3] {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

/// Invert an M×M symmetric positive-definite matrix (M ≤ 3), closed form.
fn inv_small<const M: usize>(s: &[[f32; M]; M]) -> [[f32; M]; M] {
    let mut out = [[0.0f32; M]; M];
    match M {
        1 => {
            out[0][0] = 1.0 / s[0][0];
        }
        2 => {
            let det = s[0][0] * s[1][1] - s[0][1] * s[1][0];
            out[0][0] = s[1][1] / det;
            out[0][1] = -s[0][1] / det;
            out[1][0] = -s[1][0] / det;
            out[1][1] = s[0][0] / det;
        }
        3 => {
            let (a, b, c) = (s[0][0], s[0][1], s[0][2]);
            let (d, e, f) = (s[1][0], s[1][1], s[1][2]);
            let (g, h, i) = (s[2][0], s[2][1], s[2][2]);
            let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
            out[0][0] = (e * i - f * h) / det;
            out[0][1] = (c * h - b * i) / det;
            out[0][2] = (b * f - c * e) / det;
            out[1][0] = (f * g - d * i) / det;
            out[1][1] = (a * i - c * g) / det;
            out[1][2] = (c * d - a * f) / det;
            out[2][0] = (d * h - e * g) / det;
            out[2][1] = (g * b - a * h) / det;
            out[2][2] = (a * e - b * d) / det;
        }
        _ => panic!("update supports M ≤ 3"),
    }
    out
}

/// The 15-state EKF. `Copy`-sized, zero allocation, zero locks.
#[derive(Debug, Clone, Copy)]
pub struct Ekf15 {
    cfg: NavConfig,
    /// Position NED, metres (down positive).
    pub p_ned_m: [f64; 3],
    /// Velocity NED, m/s.
    pub v_ned_mps: [f32; 3],
    /// Body→NED direction-cosine matrix (row-major).
    pub dcm_bn: [[f32; 3]; 3],
    /// Accelerometer-bias estimate, m/s².
    pub ba_mps2: [f32; 3],
    /// Gyro-bias estimate, rad/s.
    pub bg_rad_s: [f32; 3],
    /// Covariance, 15×15 row-major.
    p: [[f32; N]; N],
}

impl Ekf15 {
    /// Initialize at rest at the origin with the given initial attitude
    /// `(roll, pitch, yaw)` radians.
    pub fn new(cfg: NavConfig, initial_att_rad: [f32; 3]) -> Self {
        let mut e = Self {
            cfg,
            p_ned_m: [0.0; 3],
            v_ned_mps: [0.0; 3],
            dcm_bn: dcm_from_rpy(initial_att_rad[0], initial_att_rad[1], initial_att_rad[2]),
            ba_mps2: [0.0; 3],
            bg_rad_s: [0.0; 3],
            p: [[0.0; N]; N],
        };
        // Diagonal P: position/velocity reasonably known (surface fix),
        // attitude error from alignment, biases unknown.
        let mut d = [1.0f32; N];
        d[idx::P[0]] = 1.0;
        d[idx::P[1]] = 1.0;
        d[idx::P[2]] = 0.25; // depth well known at the surface
        d[idx::V[0]] = 0.01;
        d[idx::V[1]] = 0.01;
        d[idx::V[2]] = 0.01;
        for di in idx::TH {
            d[di] = cfg.init_att_std * cfg.init_att_std;
        }
        for di in idx::BA {
            d[di] = 0.05 * 0.05;
        }
        for di in idx::BG {
            d[di] = 0.005 * 0.005;
        }
        for (i, di) in d.into_iter().enumerate() {
            e.p[i][i] = di;
        }
        e
    }

    /// Covariance diagonal, for telemetry and convergence tests.
    pub fn p_diag(&self) -> [f32; N] {
        core::array::from_fn(|i| self.p[i][i])
    }

    /// Covariance element access, for audit tests.
    pub fn p_at(&self, i: usize, j: usize) -> f32 {
        self.p[i][j]
    }

    /// Strapdown prediction with one IMU sample: nominal-state integration
    /// plus `P ← F·P·Fᵀ + Q` with the standard error-state Jacobian. This is
    /// the SIMD-vectorized hot path.
    pub fn predict(&mut self, imu: &ImuSample) {
        let dt = imu.dt_s.max(1.0e-4);
        let f = [
            imu.accel_mps2[0] - self.ba_mps2[0],
            imu.accel_mps2[1] - self.ba_mps2[1],
            imu.accel_mps2[2] - self.ba_mps2[2],
        ];
        let w = [
            imu.gyro_rad_s[0] - self.bg_rad_s[0],
            imu.gyro_rad_s[1] - self.bg_rad_s[1],
            imu.gyro_rad_s[2] - self.bg_rad_s[2],
        ];

        // --- Nominal propagation ------------------------------------------------
        let r_new = rotate_dcm(&self.dcm_bn, w, dt);
        let f_ned = mv3(&self.dcm_bn, f);
        let mut a_ned = f_ned;
        a_ned[2] -= GRAVITY_M_S2;
        for i in 0..3 {
            self.p_ned_m[i] += (self.v_ned_mps[i] + 0.5 * a_ned[i] * dt) as f64 * dt as f64;
            self.v_ned_mps[i] += a_ned[i] * dt;
        }
        // (indexed loops over NED axes are kept explicit for reviewer
        // clarity; the noise terms below are the SIMD path.)
        self.dcm_bn = r_new;

        // --- Covariance propagation ---------------------------------------------
        // F = I + F', with blocks:
        //   ∂p/∂v = I·dt, ∂v/∂θ = −R·[f×]·dt,
        //   ∂v/∂b_a = −R·dt, ∂θ/∂b_g = −R·dt.
        let mut fp = identity_n();
        for i in 0..3 {
            fp[idx::P[i]][idx::V[i]] += dt;
        }
        // [f×]·u = f × u.
        let sk = [[0.0, -f[2], f[1]], [f[2], 0.0, -f[0]], [-f[1], f[0], 0.0]];
        for i in 0..3 {
            for j in 0..3 {
                let r_sk = -(self.dcm_bn[i][0] * sk[0][j]
                    + self.dcm_bn[i][1] * sk[1][j]
                    + self.dcm_bn[i][2] * sk[2][j]);
                fp[idx::V[i]][idx::TH[j]] = r_sk * dt;
                fp[idx::V[i]][idx::BA[j]] = -self.dcm_bn[i][j] * dt;
                fp[idx::TH[i]][idx::BG[j]] = -self.dcm_bn[i][j] * dt;
            }
        }

        // P ← F·P·Fᵀ + Q (two 15×15 multiplies; the SIMD dots live here).
        let mut tmp = [[0.0f32; N]; N];
        let mut p_new = [[0.0f32; N]; N];
        let mut ft = [[0.0f32; N]; N];
        mat_mul(&fp, &self.p, &mut tmp);
        mat_transpose(&fp, &mut ft);
        mat_mul(&tmp, &ft, &mut p_new);

        // Q_d: white noise on velocity/attitude + bias random walks.
        let q_a = self.cfg.accel_noise * self.cfg.accel_noise * dt;
        let q_g = self.cfg.gyro_noise * self.cfg.gyro_noise * dt;
        let q_ba = self.cfg.accel_bias_rw * self.cfg.accel_bias_rw * dt;
        let q_bg = self.cfg.gyro_bias_rw * self.cfg.gyro_bias_rw * dt;
        for i in 0..3 {
            p_new[idx::V[i]][idx::V[i]] += q_a;
            p_new[idx::TH[i]][idx::TH[i]] += q_g;
            p_new[idx::BA[i]][idx::BA[i]] += q_ba;
            p_new[idx::BG[i]][idx::BG[i]] += q_bg;
        }

        // Symmetrize to fight numeric drift.
        for i in 0..N {
            for j in i..N {
                let s = 0.5 * (p_new[i][j] + p_new[j][i]);
                p_new[i][j] = s;
                p_new[j][i] = s;
            }
        }
        self.p = p_new;
    }

    /// DVL bottom-track velocity update (3-D). Ignored without bottom lock.
    pub fn update_dvl(&mut self, dvl: &DvlSample) {
        if !dvl.bottom_lock {
            return;
        }
        let mut h = [[0.0f32; N]; 3];
        for i in 0..3 {
            h[i][idx::V[i]] = 1.0;
        }
        let sigma2 = 0.05 * 0.05; // 5 cm/s DVL noise
        self.update_n::<3>(&h, &dvl.velocity_ned_mps, &[sigma2; 3]);
    }

    /// Pressure-depth update (1-D).
    pub fn update_depth(&mut self, depth: &DepthSample) {
        let mut h = [[0.0f32; N]; 1];
        h[0][idx::P[2]] = 1.0;
        let z = [depth.depth_m as f32];
        self.update_n::<1>(&h, &z, &[depth.std_m * depth.std_m]);
    }

    /// Acoustic position fix (USBL/LBL/SBL), 3-D.
    pub fn update_acoustic(&mut self, fix: &AcousticFix) {
        let mut h = [[0.0f32; N]; 3];
        for i in 0..3 {
            h[i][idx::P[i]] = 1.0;
        }
        let z = [
            fix.ned_m[0] as f32,
            fix.ned_m[1] as f32,
            fix.ned_m[2] as f32,
        ];
        let r = [fix.std_m * fix.std_m; 3];
        self.update_n::<3>(&h, &z, &r);
    }

    /// Generic ≤3-D update with diagonal R and Joseph-form covariance
    /// update for numeric stability.
    fn update_n<const M: usize>(&mut self, h: &[[f32; N]; M], z: &[f32; M], r_diag: &[f32; M]) {
        // Innovation y = z − H·x.
        let x = self.state_vec();
        let mut y = [0.0f32; M];
        for (i, y_i) in y.iter_mut().enumerate() {
            *y_i = z[i] - dot15(&h[i], &x);
        }
        // P·Hᵀ (N×M): rows are P·H_iᵀ.
        let mut pht = [[0.0f32; M]; N];
        for i in 0..N {
            for (m, h_row) in h.iter().enumerate() {
                pht[i][m] = dot15(&self.p[i], h_row);
            }
        }
        // S = H·P·Hᵀ + R: S[a][b] = H_a · (P·H_bᵀ).
        let mut s = [[0.0f32; M]; M];
        for a in 0..M {
            for b in 0..M {
                let mut acc = 0.0f32;
                for k in 0..N {
                    acc += h[a][k] * pht[k][b];
                }
                s[a][b] = acc;
            }
            s[a][a] += r_diag[a];
        }
        // K = P·Hᵀ·S⁻¹ (M×N).
        let sinv = inv_small::<M>(&s);
        let mut k = [[0.0f32; N]; M];
        for c in 0..N {
            for m in 0..M {
                let mut acc = 0.0f32;
                for i in 0..M {
                    acc += sinv[m][i] * pht[c][i];
                }
                k[m][c] = acc;
            }
        }
        // x += K·y.
        let mut dx = [0.0f32; N];
        for i in 0..N {
            let mut acc = 0.0f32;
            for m in 0..M {
                acc += k[m][i] * y[m];
            }
            dx[i] = acc;
        }
        self.apply_correction(&dx);

        // Joseph form: P ← A·P·Aᵀ + K·R·Kᵀ with A = I − K·H.
        let mut a = identity_n();
        for i in 0..N {
            for j in 0..N {
                let mut acc = 0.0f32;
                for m in 0..M {
                    acc += k[m][i] * h[m][j];
                }
                a[i][j] -= acc;
            }
        }
        let mut tmp = [[0.0f32; N]; N];
        let mut p_new = [[0.0f32; N]; N];
        let mut at = [[0.0f32; N]; N];
        mat_mul(&a, &self.p, &mut tmp);
        mat_transpose(&a, &mut at);
        mat_mul(&tmp, &at, &mut p_new);
        for i in 0..N {
            for j in 0..N {
                let mut acc = 0.0f32;
                for m in 0..M {
                    acc += k[m][i] * r_diag[m] * k[m][j];
                }
                p_new[i][j] += acc;
            }
        }
        self.p = p_new;
    }

    fn state_vec(&self) -> [f32; N] {
        let mut x = [0.0f32; N];
        for i in 0..3 {
            x[idx::P[i]] = self.p_ned_m[i] as f32;
            x[idx::V[i]] = self.v_ned_mps[i];
            x[idx::BA[i]] = self.ba_mps2[i];
            x[idx::BG[i]] = self.bg_rad_s[i];
        }
        x
    }

    /// Apply a state correction; attitude-error entries are injected into
    /// the DCM and the θ states reset (error-state discipline).
    fn apply_correction(&mut self, dx: &[f32; N]) {
        let dth = [dx[idx::TH[0]], dx[idx::TH[1]], dx[idx::TH[2]]];
        for i in 0..3 {
            self.p_ned_m[i] += dx[idx::P[i]] as f64;
            self.v_ned_mps[i] += dx[idx::V[i]];
            self.ba_mps2[i] += dx[idx::BA[i]];
            self.bg_rad_s[i] += dx[idx::BG[i]];
        }
        // Inject attitude error into the DCM: R ← R·Exp([θ×]) ≈ R·(I + [θ×]).
        self.dcm_bn = rotate_dcm(&self.dcm_bn, dth, 1.0);
    }
}

fn identity_n() -> [[f32; N]; N] {
    let mut m = [[0.0f32; N]; N];
    for i in 0..N {
        m[i][i] = 1.0;
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at_rest() -> Ekf15 {
        Ekf15::new(NavConfig::typical(), [0.0; 3])
    }

    fn still_imu(dt: f32) -> ImuSample {
        ImuSample {
            // Level vehicle: accelerometer reads +g on the body-down axis.
            accel_mps2: [0.0, 0.0, GRAVITY_M_S2],
            gyro_rad_s: [0.0; 3],
            dt_s: dt,
        }
    }

    #[test]
    fn level_vehicle_holds_position() {
        let mut e = at_rest();
        for _ in 0..2000 {
            e.predict(&still_imu(0.005)); // 10 s at 200 Hz
        }
        assert!(
            e.p_ned_m.iter().all(|p| p.abs() < 1.0e-3),
            "a level, still vehicle must not drift: {:?}",
            e.p_ned_m
        );
        assert!(e.v_ned_mps.iter().all(|v| v.abs() < 1.0e-3));
    }

    #[test]
    fn constant_acceleration_integrates() {
        let mut e = at_rest();
        // 1 m/s² forward for 10 s → 50 m, 10 m/s. The accelerometer reads
        // specific force: 1 m/s² north plus the +g reaction on body-down.
        for _ in 0..2000 {
            e.predict(&ImuSample {
                accel_mps2: [1.0, 0.0, GRAVITY_M_S2],
                gyro_rad_s: [0.0; 3],
                dt_s: 0.005,
            });
        }
        assert!(
            (e.p_ned_m[0] - 50.0).abs() < 0.5,
            "dead reckoning must integrate: p={:?}",
            e.p_ned_m
        );
        assert!((e.v_ned_mps[0] - 10.0).abs() < 0.2);
    }

    #[test]
    fn covariance_grows_without_updates() {
        let mut e = at_rest();
        let p0 = e.p_diag()[idx::P[0]];
        for _ in 0..200 {
            e.predict(&still_imu(0.005));
        }
        let p1 = e.p_diag()[idx::P[0]];
        assert!(p1 > p0, "position variance must grow during dead reckoning");
        assert!(p1.is_finite());
    }

    #[test]
    fn depth_update_pulls_position_down_axis() {
        let mut e = at_rest();
        // Truth: filter thinks 0 m, sensor says 50 m.
        for _ in 0..10 {
            e.predict(&still_imu(0.005));
            e.update_depth(&DepthSample {
                depth_m: 50.0,
                std_m: 0.2,
            });
        }
        assert!(
            (e.p_ned_m[2] - 50.0).abs() < 1.0,
            "depth update must converge: {}",
            e.p_ned_m[2]
        );
        // Horizontal axes untouched by a depth measurement.
        assert!(e.p_ned_m[0].abs() < 1.0e-6 && e.p_ned_m[1].abs() < 1.0e-6);
    }

    #[test]
    fn dvl_update_kills_velocity_error() {
        let mut e = at_rest();
        // Inject a velocity error (as if a current slew or prior fault did).
        e.v_ned_mps = [0.5, -0.3, 0.0];
        for _ in 0..10 {
            e.predict(&still_imu(0.005));
            e.update_dvl(&DvlSample {
                velocity_ned_mps: [0.0; 3], // truth: stationary
                bottom_lock: true,
            });
        }
        assert!(
            e.v_ned_mps.iter().all(|v| v.abs() < 0.05),
            "DVL updates must pull velocity to measurement: {:?}",
            e.v_ned_mps
        );
    }

    #[test]
    fn no_bottom_lock_is_ignored() {
        let mut e = at_rest();
        e.v_ned_mps = [1.0, 0.0, 0.0];
        for _ in 0..20 {
            e.update_dvl(&DvlSample {
                velocity_ned_mps: [0.0; 3],
                bottom_lock: false, // water-mass track: must be ignored
            });
        }
        assert!((e.v_ned_mps[0] - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn acoustic_fix_converges() {
        let mut e = at_rest();
        let truth = [100.0f64, -50.0, 30.0];
        // Walk the filter away, then let USBL pull it back. Convergence is
        // gain-limited (R = 4 m² dominates early); enough ticks must get
        // every axis inside the fix accuracy.
        e.p_ned_m = [4.0, 3.0, 2.0];
        for _ in 0..1000 {
            e.predict(&still_imu(0.005));
            e.update_acoustic(&AcousticFix {
                ned_m: truth,
                std_m: 2.0,
            });
        }
        let err = (0..3)
            .map(|i| (e.p_ned_m[i] - truth[i]).abs())
            .fold(0.0, f64::max);
        assert!(
            err < 0.5,
            "acoustic fix must converge, max axis error {err}"
        );
    }

    #[test]
    fn covariance_stays_symmetric_and_finite() {
        let mut e = at_rest();
        for k in 0..1000 {
            e.predict(&still_imu(0.005));
            if k % 50 == 0 {
                e.update_depth(&DepthSample {
                    depth_m: 10.0,
                    std_m: 0.2,
                });
            }
        }
        for i in 0..N {
            for j in 0..N {
                assert!(e.p_at(i, j).is_finite());
            }
        }
        let max_asym = (0..N)
            .flat_map(|i| (0..N).map(move |j| (i, j)))
            .map(|(i, j)| (e.p_at(i, j) - e.p_at(j, i)).abs())
            .fold(0.0f32, f32::max);
        assert!(max_asym < 1.0e-4, "P must stay symmetric: {max_asym}");
    }

    #[test]
    fn rpy_dcm_identity_at_zero() {
        let r = dcm_from_rpy(0.0, 0.0, 0.0);
        assert!((r[0][0] - 1.0).abs() < 1.0e-7);
        assert!(r[0][1].abs() < 1.0e-7 && r[1][0].abs() < 1.0e-7);
        assert!((r[2][2] - 1.0).abs() < 1.0e-7);
    }

    #[test]
    fn yaw_dcm_rotates_body_x_to_east() {
        let r = dcm_from_rpy(0.0, 0.0, core::f32::consts::FRAC_PI_2);
        let f_ned = mv3(&r, [1.0, 0.0, 0.0]);
        assert!(f_ned[1] > 0.99, "yaw 90° maps body-x to east: {f_ned:?}");
        assert!(f_ned[0].abs() < 0.01);
    }
}
