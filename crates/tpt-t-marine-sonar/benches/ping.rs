//! Manual ping-processing benchmark (spec §4.3: deterministic 2 ms per
//! ping at 512 beams / 20 Hz). Same pattern as the nav bench: no harness
//! dependency, `TPT_SONAR_STRICT=1` enforces the gate.
//!
//! ```sh
//! cargo bench --manifest-path crates/tpt-t-marine-sonar/Cargo.toml
//! TPT_SONAR_STRICT=1 cargo bench --manifest-path crates/tpt-t-marine-sonar/Cargo.toml
//! ```

use std::time::Instant;

use tpt_t_marine_sonar::raytrace::{BEAMS, BeamGeometry, PingInput, Processor};
use tpt_t_marine_sonar::slab::SlabRing;

fn main() {
    let ring = SlabRing::new();
    let _ = ring;
    let mut ring = SlabRing::new();
    let proc = Processor::new(BeamGeometry::even_fan(2.0));

    // Realistic survey ping: ranges sweep 20–120 m with a texture term.
    let ranges: [f32; BEAMS] =
        core::array::from_fn(|i| 70.0 + 50.0 * ((i as f32 * 0.13).sin() * 0.5 + 0.5));

    let warm = PingInput {
        ranges_m: ranges,
        sound_velocity_mps: 1500.0,
        reference_velocity_mps: 1500.0,
        attitude_rad: [0.01, -0.02, 0.3],
        timestamp_us: 0,
    };
    // Warmup.
    for k in 0..2000u64 {
        let slab = ring.claim(k * 50_000);
        proc.process(&warm, slab);
    }

    const PINGS: u32 = 20_000; // ~16 minutes of survey at 20 Hz
    let start = Instant::now();
    for k in 0..PINGS as u64 {
        let mut ping = warm;
        ping.timestamp_us = k * 50_000;
        let slab = ring.claim(k * 50_000);
        proc.process(&ping, slab);
    }
    let elapsed = start.elapsed();
    let per_ping_us = elapsed.as_nanos() as f64 / PINGS as f64 / 1000.0;

    println!(
        "sonar: {PINGS} pings in {elapsed:.3?} → {:.2} µs/ping (budget 2000 µs @ 512 beams)",
        per_ping_us
    );
    let latest = ring.latest().expect("pings processed");
    println!(
        "latest cloud: {} beams, t={} µs, point[256]=({:.2}, {:.2}, {:.2})",
        latest.beam_count,
        latest.timestamp_us,
        latest.points[256].x,
        latest.points[256].y,
        latest.points[256].z,
    );

    if std::env::var("TPT_SONAR_STRICT").as_deref() == Ok("1") {
        if per_ping_us > 2000.0 {
            eprintln!("FAIL: {per_ping_us:.2} µs/ping exceeds the 2 ms spec budget");
            std::process::exit(1);
        }
        println!("PASS: within the 2 ms/ping spec budget");
    }
}
