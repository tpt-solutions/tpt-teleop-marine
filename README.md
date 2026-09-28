# tpt-teleop-marine

**Tele-Presence Teleoperation — Maritime, Underwater & Offshore Operations** — a
hyper-optimized, zero-bloat Rust middleware workspace for AUVs (Autonomous
Underwater Vehicles), ROVs (Remotely Operated Vehicles), ASVs (Autonomous
Surface Vessels), and gliders.

Sister repo to [`tpt-teleop`](https://github.com/tpt-solutions/tpt-teleop)
(video/control transport), [`tpt-teleop-fleet`](https://github.com/tpt-solutions/tpt-teleop-fleet)
(indoor warehouse precision), and [`tpt-t-agri`](https://github.com/tpt-solutions/tpt-teleop-agri)
(precision agriculture). The repository retains the full `tpt-teleop-marine`
name; all internal crates use the simplified `tpt-t-marine-` prefix.

Built for the most physically hostile and communication-starved environment on
Earth — the ocean — with the same microsecond-level determinism as the rest of
the tpt ecosystem: no async runtime, no serde, no channels-with-mutexes. A
custom 100-bps acoustic protocol, SIMD-accelerated INS/DVL fusion, zero-copy
rkyv serialization, slab-allocated sonar pipelines, deterministic COLREGS, and
raw OS interfaces all the way down.

## Status

✅ **v1.0.0 — "v1 Monolithic Release"** (spec v1.0.0, August 2026). All 15
phases implemented: the vendored `tpt-t-domain-bridge` DTI, the 15-state
SIMD EKF (measured **1.6 µs/cycle** vs the 200 µs spec budget), the
zero-alloc acoustic stack (GF(256) Reed-Solomon + adaptive ARQ riding 60 %
loss, 4 KB state), slab-ring sonar at **1.7 µs/ping** vs 2 ms, deterministic
COLREGS under 50 µs/vessel, Verlet tether with 5-second snag prediction,
DP within 1 m in 2-knot currents, the headless simulator at >100× real
time, and the full Subsea Pipeline Survey data-flow integration test —
see [`todo.md`](todo.md) for the phase-by-phase record and
[`docs/quickstart.md`](docs/quickstart.md) to build it.

| CI | Lint | Deps |
|----|------|------|
| ![build](https://github.com/tpt-solutions/tpt-teleop-marine/actions/workflows/ci.yml/badge.svg) | ![lint](https://github.com/tpt-solutions/tpt-teleop-marine/actions/workflows/lint.yml/badge.svg) | ![deny](https://github.com/tpt-solutions/tpt-teleop-marine/actions/workflows/lint.yml/badge.svg) |

## Workspace Crates

| Crate | Purpose |
|-------|---------|
| `tpt-t-domain-bridge` | `DomainTeleopInterface` (DTI) trait, wire types, universal safety FSM, assist API *(vendored from `tpt-teleop`)* |
| `tpt-t-marine-core` | Mission state machine (Dive → Transit → Survey → Surface → Recover), lock-free message bus, central event loop, rkyv wire prelude |
| `tpt-t-marine-nav` | GPS-denied 3D navigation: 15-state SIMD EKF, INS/DVL/pressure/acoustic fusion, WGS84/UTM/ECEF *(nightly crate)* |
| `tpt-t-marine-pressure` | Depth/pressure sensing, temperature compensation, hysteresis, leak detection |
| `tpt-t-marine-buoyancy` | Variable ballast, trim pumps, buoyancy engines, compression-at-depth physics |
| `tpt-t-marine-acoustic` | Adaptive ARQ protocol, Reed-Solomon FEC, triple-redundancy sends, 4KB zero-alloc stack |
| `tpt-t-marine-comms` | Multi-modal link manager (acoustic/RF/cellular/satellite), store-and-forward, tether QoS |
| `tpt-t-marine-current` | ADCP ingestion, drift prediction for AUVs and ASVs |
| `tpt-t-marine-tide` | Harmonic constituent tides, tidal-stream prediction |
| `tpt-t-marine-sonar` | Multibeam/sidescan/forward-looking sonar, slab ring buffers, SIMD beamforming *(nightly crate)* |
| `tpt-t-marine-safety` | Flooding, emergency buoyancy, thermal shutdown, lost-vehicle procedure |
| `tpt-t-marine-colregs` | Deterministic COLREGS FSM: give-way/stand-on, whistle signals, auditable decisions |
| `tpt-t-marine-auv` | 3D waypoint following, adaptive sampling, comms-triggered behaviors, sleep-wake power |
| `tpt-t-marine-glider` | Buoyancy-driven flight, ultra-low-power trans-oceanic profiles |
| `tpt-t-marine-tether` | Lumped-mass Verlet tether physics, 100Hz, 5s-ahead snag prediction, winch/TMS |
| `tpt-t-marine-rov` | 6DOF thruster coordination, manipulator arm, tether-aware motion, haptics |
| `tpt-t-marine-asv` | Waypoint following, dynamic positioning, COLREGS maneuvering, wave compensation |
| `tpt-t-marine-offshore` | Wind-turbine foundation inspection, subsea cable routing, wellhead monitoring |
| `tpt-t-marine-aquaculture` | Fish-cage net inspection, hydroacoustic biomass, mortality detection |
| `tpt-t-marine-teleop` | `DomainTeleopInterface` adapter, acoustic fallback, tether-aware teleop |
| `tpt-t-marine-sim` | Headless 6DOF simulator: hydrodynamics, acoustic channel, currents, sonar returns |
| `tpt-t-marine-visualizer` | `egui` visualizer: trajectories, link heatmaps, catenary, point clouds *(excluded crate)* |

## The Zero-Allocation Data Path

```
Sensor ──Ingest──▶ Fuse/Route ──▶ Mission state (in-place)
                                          │
                                 Serialize ──▶ Acoustic/wire (4KB stack)
Total allocations (steady state): 0        Total mutex locks: 0
```

The forward data plane is built on lock-free SPSC rings (`tpt-t-marine-core`)
and proven zero-alloc on the hot path; see `tools/lock-audit.sh` (hard gate)
and `tools/alloc-audit.sh` (informational report).

## Developer Experience

Path deps during co-development: `tpt-t-marine-*` crates depend on
`tpt-t-marine-core` and, where relevant, on `tpt-t-domain-bridge` (vendored
via path dep). Nightly is required **only** for `tpt-t-marine-nav` and
`tpt-t-marine-sonar` (`#![feature(portable_simd)]`); both are excluded from
the stable workspace and build in isolation. See
[`docs/quickstart.md`](docs/quickstart.md).

## Licensing

Dual-licensed under either of:

 * Apache License, Version 2.0 — [LICENSE-APACHE](LICENSE-APACHE)
 * MIT license — [LICENSE-MIT](LICENSE-MIT)

Copyright © 2026 TPT Solutions.

Dependency policy ("the MIT chain"): dependencies are restricted to MIT,
BSD-2/3-Clause, ISC, Zlib, and MPL-2.0. Dual MIT/Apache crates are resolved
strictly under MIT; strictly Apache-2.0-only crates are banned. Enforced by
[`deny.toml`](deny.toml) in CI.
