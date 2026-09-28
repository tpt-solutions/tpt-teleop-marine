# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] — v1 Monolithic Release

### Added
- Phase 5: `tpt-t-marine-current` (ADCP QC, drift prediction),
  `tpt-t-marine-tide` (harmonic constituents, tidal-stream cells).
- Phase 6: `tpt-t-marine-sonar` (nightly; slab ring of 512-beam clouds,
  SIMD sound-velocity refraction + ray tracing — measured 1.7 µs/ping vs
  the 2 ms budget).
- Phase 7: `tpt-t-marine-safety` (flooding reflex <100 ms, thermal
  escalation, lost-vehicle procedure, teleop-assist bridge), and
  `tpt-t-marine-colregs` (deterministic Rules 13/14/15/17/34 FSM with
  auditable decision log, <50 µs/vessel).
- Phase 8: `tpt-t-marine-auv` (3D LOS waypoint guidance, adaptive sampling
  with battery budget, sleep-wake power, surfacing window) and
  `tpt-t-marine-glider` (sawtooth buoyancy flight, oceanic fix policy).
- Phase 9: `tpt-t-marine-tether` (50-node Verlet rope with damping,
  5-second snag predictor, winch/TMS tension window) and
  `tpt-t-marine-rov` (8-thruster 6DOF mixer, rate-limited arm, tether
  guard with pilot haptics).
- Phase 10: `tpt-t-marine-asv` (PID dynamic positioning holding 1 m in
  2-knot currents, COLREGS supervisor, wave compensation).
- Phase 11: `tpt-t-marine-offshore` (turbine spiral GVI, cable burial
  router, wellhead standoff monitor) and `tpt-t-marine-aquaculture`
  (net inspection, hydroacoustic biomass, mortality clustering).
- Phase 12: `tpt-t-marine-teleop` — the DTI adapter: 6DOF translation with
  slew limiting, blend-ramped handover, snag override with haptics,
  1 Hz acoustic fallback on tether cut, payload/buoyancy hook.
- Phase 13: `tpt-t-marine-sim` (6DOF drag dynamics, 666 µs/km acoustic
  channel with multipath, uniform+tidal current fields) and the excluded
  `tpt-t-marine-visualizer` (egui trajectory renderer).
- Phase 14: the Subsea Pipeline Survey integration test (mission upload →
  transit → dive → survey → acoustic check-ins → recovery), the 100×
  real-time throughput gate, and the zero-alloc NMEA 0183 parser with
  total-parse property tests.
- Phase 15: `cargo fmt` + `cargo clippy -D warnings` clean on the stable
  workspace and on nightly for `tpt-t-marine-nav` / `tpt-t-marine-sonar`;
  benchmarks and audit scripts green.

## [Unreleased]

### Added
- Phase 0: Cargo workspace scaffold, MIT-chain `deny.toml`, CI/lint workflows,
  `rust-toolchain.toml`, audit scripts, docs skeleton.
- Phase 1: `tpt-t-domain-bridge` — DTI trait, zero-copy rkyv wire types,
  universal safety state machine, `request_teleop_assistance()` API
  (vendored from `tpt-teleop` during co-development).
- Phase 2: `tpt-t-marine-core` — mission state machine (Dive → Transit →
  Survey → Surface → Recover), lock-free SPSC message bus, central event
  loop, rkyv wire prelude.
- Phase 3: `tpt-t-marine-nav` (nightly; 15-state SIMD EKF, INS/DVL/pressure/
  acoustic fusion, WGS84/UTM/ECEF, biofouling derating), `tpt-t-marine-pressure`
  (leak detection <100ms), `tpt-t-marine-buoyancy` (ballast/trim/compression).
