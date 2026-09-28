//! Manual EKF-cycle benchmark (spec §4.2: `<200 µs` per cycle at 200 Hz on a
//! Cortex-A72). No benchmark harness dependency — this is a plain binary
//! timed with `std::time::Instant`, in keeping with the zero-bloat policy.
//!
//! ```sh
//! cargo bench --manifest-path crates/tpt-t-marine-nav/Cargo.toml
//! # enforce the spec gate (exit 1 on miss):
//! TPT_NAV_STRICT=1 cargo bench --manifest-path crates/tpt-t-marine-nav/Cargo.toml
//! ```
//!
//! The gate is informational by default because absolute numbers are
//! host-dependent; CI runs it without the env var and prints the numbers.

use std::time::Instant;

use tpt_t_marine_nav::ekf::{DvlSample, Ekf15, GRAVITY_M_S2, ImuSample, N, NavConfig};

/// One 200 Hz filter tick: predict + (every 20th tick) a DVL update — the
/// survey-AUV steady-state pattern.
fn main() {
    let mut e = Ekf15::new(NavConfig::typical(), [0.0; 3]);
    let imu = ImuSample {
        accel_mps2: [0.0, 0.0, GRAVITY_M_S2],
        gyro_rad_s: [0.0; 3],
        dt_s: 0.005,
    };
    let dvl = DvlSample {
        velocity_ned_mps: [0.0; 3],
        bottom_lock: true,
    };

    const TICKS: u32 = 200_000; // 1000 simulated seconds at 200 Hz
    // Warmup (cache, branch predictors).
    for k in 0..2_000 {
        e.predict(&imu);
        if k % 20 == 0 {
            e.update_dvl(&dvl);
        }
    }

    let start = Instant::now();
    for k in 0..TICKS {
        e.predict(&imu);
        if k % 20 == 0 {
            e.update_dvl(&dvl);
        }
    }
    let elapsed = start.elapsed();
    let per_cycle_us = elapsed.as_nanos() as f64 / (TICKS as f64) / 1000.0;

    let p = e.p_diag();
    println!(
        "nav EKF: {TICKS} cycles in {:.3?} → {:.2} µs/cycle (budget 200 µs)",
        elapsed, per_cycle_us
    );
    println!("P diagonal: {:?}", &p[..N.min(6)]);
    println!(
        "final position: [{:.3} {:.3} {:.3}] m",
        e.p_ned_m[0], e.p_ned_m[1], e.p_ned_m[2]
    );

    if std::env::var("TPT_NAV_STRICT").as_deref() == Ok("1") {
        if per_cycle_us > 200.0 {
            eprintln!(
                "FAIL: EKF cycle {:.2} µs exceeds the 200 µs spec budget",
                per_cycle_us
            );
            std::process::exit(1);
        }
        println!("PASS: EKF cycle within the 200 µs spec budget");
    }
}
