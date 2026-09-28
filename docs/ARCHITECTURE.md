# Architecture — tpt-teleop-marine

> Status: living document. Grows as crates land; the crate map below is the
> full v1 target (spec §3).

## Workspace Map

```
crates/
  tpt-t-domain-bridge/    # DTI trait, wire types, safety FSM, assist API (Phase 1)
  tpt-t-marine-core/      # mission FSM, lock-free bus, event loop, wire prelude (Phase 2)
  tpt-t-marine-nav/       # 15-state SIMD EKF, INS/DVL/pressure/acoustic fusion — nightly (Phase 3)
  tpt-t-marine-pressure/  # depth/pressure, temp comp, hysteresis, leak detection (Phase 3)
  tpt-t-marine-buoyancy/  # ballast, trim, buoyancy engine, compression physics (Phase 3)
  tpt-t-marine-acoustic/  # adaptive ARQ, Reed-Solomon FEC, 4KB zero-alloc stack (Phase 4)
  tpt-t-marine-comms/     # link manager, store-and-forward, tether QoS (Phase 4)
  tpt-t-marine-current/   # ADCP ingestion, drift prediction (Phase 5)
  tpt-t-marine-tide/      # harmonic constituents, tidal streams (Phase 5)
  tpt-t-marine-sonar/     # multibeam/sidescan/FLS, slab rings, SIMD beams — nightly (Phase 6)
  tpt-t-marine-safety/    # flooding, emergency buoyancy, lost vehicle (Phase 7)
  tpt-t-marine-colregs/   # deterministic COLREGS FSM (Phase 7)
  tpt-t-marine-auv/       # 3D waypoints, adaptive sampling, sleep-wake (Phase 8)
  tpt-t-marine-glider/    # buoyancy flight, ultra-low power (Phase 8)
  tpt-t-marine-tether/    # Verlet catenary, snag prediction, winch/TMS (Phase 9)
  tpt-t-marine-rov/       # 6DOF thrusters, manipulator, tether-aware motion (Phase 9)
  tpt-t-marine-asv/       # DP, COLREGS maneuvering, wave compensation (Phase 10)
  tpt-t-marine-offshore/  # turbine/cable/wellhead inspection apps (Phase 11)
  tpt-t-marine-aquaculture/ # net inspection, biomass, mortality (Phase 11)
  tpt-t-marine-teleop/    # DomainTeleopInterface adapter (Phase 12)
  tpt-t-marine-sim/       # headless 6DOF + acoustic channel + currents (Phase 13)
  tpt-t-marine-visualizer/  # egui renderer — excluded from workspace (Phase 13)
```

## Dependency Direction

```
                tpt-t-domain-bridge  (universal teleop contract)
                        ▲
tpt-t-marine-core ◀── domain crates (nav, acoustic, sonar, safety, …)
        ▲                       ▲
        └──── vehicle crates (auv, rov, asv, glider, tether) ── adapter
                                     (tpt-t-marine-teleop) ◀── sim
```

- `tpt-t-marine-core` is the shared foundation: mission state machine,
  lock-free bus, event loop, and the rkyv wire-type prelude every marine
  crate re-exports for zero-copy interchange.
- `tpt-t-domain-bridge` is the universal teleop contract (vendored from
  `tpt-teleop` during co-development; see `docs/quickstart.md`).
- Vehicle crates consume domain crates; the `tpt-t-marine-teleop` adapter is
  the only crate that speaks DTI to the core teleop system.
- `tpt-t-marine-nav` and `tpt-t-marine-sonar` are nightly-only
  (`portable_simd`) and excluded from the stable workspace; other crates
  depend on their *wire types* through `tpt-t-marine-core`, never on the
  nightly crates directly, so the stable workspace builds without them.

## Crate Responsibilities

Each crate owns one spec section and one performance budget:

| Crate | Spec | Budget |
|-------|------|--------|
| `nav` | §4.2 | <200µs EKF cycle @ 200Hz on Cortex-A72 |
| `sonar` | §4.3 | deterministic 2ms/ping @ 512 beams, 20Hz |
| `acoustic` | §4.1 | full stack in a 4KB pre-allocated buffer |
| `tether` | §4.4 | 100Hz sim on a pinned core, 5s lookahead |
| `colregs` | §4.5 | <50µs per detected vessel |
| `pressure` | §5.4 | 0.1 PSI ingress → emergency buoyancy <100ms |
| `sim` | §8 | 100x real-time (30-day mission in 7h) |

## Design Invariants

1. **No async runtime** — thread-per-core, event loops over lock-free rings.
2. **No serde/bincode** — rkyv zero-copy with `#[repr(C)]` POD-like records.
3. **No locks on the hot path** — SPSC rings and atomics only; enforced by
   `tools/lock-audit.sh` as a CI hard gate.
4. **Deterministic control** — every safety/mission/COLREGS decision is a
   traceable FSM transition, never ML.
5. **Bounded memory** — all streaming structures are fixed-capacity
   (rings, slabs, 4KB protocol buffers).
