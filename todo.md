# tpt-teleop-marine — Project TODO

Maritime, Underwater & Offshore Operations middleware. Sister repo to
`tpt-teleop` (video/control transport), `tpt-teleop-fleet` (indoor warehouse
precision), and `tpt-t-agri` (precision agriculture). Repository retains the
full `tpt-teleop-marine` name; all internal crates use the simplified
`tpt-t-marine-` prefix. License: MIT OR Apache-2.0. Copyright TPT Solutions.

Source docs: `spec.txt` (primary design doc), `bridge spec.txt` (tpt-teleop
integration contract).

Phases are ordered dependency-first: foundation → state estimation → comms →
environment → perception → safety/maritime law → vehicle execution →
tethered ops → surface ops → offshore applications → teleop bridge → sim →
integration → hardening/release.

---

## Phase 0 — Repo Foundation & Tooling
^- [x] `git init`, initial commit of spec docs
^- [x] Cargo workspace `Cargo.toml` (resolver "3", edition 2024,
      rust-version 1.85, license `MIT OR Apache-2.0`, authors "TPT
      Solutions"), mirroring `tpt-teleop-agri`'s workspace layout (`crates/*`
      members, shared workspace deps table)
^- [x] Workspace `exclude = ["crates/tpt-t-marine-nav", "crates/tpt-t-marine-sonar"]`
      — both require nightly `portable_simd` (see Phase 3, Phase 6); rest of
      workspace, including safety-critical crates, stays on stable Rust
^- [x] `LICENSE-MIT` and `LICENSE-APACHE` at repo root (TPT Solutions, 2026)
^- [x] `.gitignore` (target/, etc.)
^- [x] `deny.toml` — MIT/BSD/ISC/Zlib/MPL-2.0 allow-chain, explicit
      Apache-2.0-only deny, `[[licenses.clarify]]` overrides for
      dual-licensed deps (rkyv, windows-sys, proc-macro2/syn/quote, libc),
      encodes the §7 dependency matrix (bans nalgebra, OpenCV/pcl, tokio,
      geo/proj, serde/bincode, the `nmea` crate, in favor of the custom
      lightweight choices)
^- [x] `.github/workflows/ci.yml` — build-test matrix (linux/macos/windows),
      cross-compile check, `RUSTFLAGS: -D warnings`, plus a separate nightly
      job scoped to `tpt-t-marine-nav` and `tpt-t-marine-sonar`
^- [x] `.github/workflows/lint.yml` — fmt + clippy (`-D warnings`, stable +
      nightly job for nav/sonar) + cargo-deny action
^- [x] `rust-toolchain.toml` — stable default; document the nightly override
      needed for `tpt-t-marine-nav` and `tpt-t-marine-sonar`
^- [x] Root `README.md` (Status / Workspace Crates / Zero-Allocation Data
      Path / Developer Experience / Licensing — mirror sister repos'
      section structure)
^- [x] `CHANGELOG.md`, `SECURITY.md`
^- [x] `docs/ARCHITECTURE.md` skeleton (workspace map, crate responsibilities)
^- [x] `docs/quickstart.md` stub

## Phase 1 — Cross-Repo Bridge: `tpt-t-domain-bridge` (lives in `tpt-teleop` repo)
^- [x] New crate `tpt-t-domain-bridge` under `tpt-teleop/crates/`
^- [x] Define `DomainTeleopInterface` (DTI) trait: `on_teleop_engage`,
      `on_teleop_disengage`, `on_control_command`, `get_domain_state`,
      `get_sensor_feed`
^- [x] Define wire types with `rkyv::Archive`: `ControlCommand`, `InputState`,
      `SensorFeed`, `DomainState`, `DomainTelemetry`, `HapticCmd`/`HapticFeedback`
^- [x] Reconcile with existing 56-byte POD `ControlCommand` in
      `tpt-teleop-core/src/ser/cmd.rs` (bridge spec flags this as an
      intentional, unreconciled discrepancy — resolve or document the mapping)
^- [x] Universal safety state machine: `AUTONOMOUS → REQUESTING_TELEOP →
      TELEOP_ACTIVE → RETURNING_TO_AUTONOMY → AUTONOMOUS`, plus
      `EMERGENCY_STOP` transitions
^- [x] `request_teleop_assistance()` API surface
^- [x] Unit tests + doc examples; publish path so `tpt-t-marine-teleop` can
      depend on it (path dep during co-development)

## Phase 2 — `tpt-t-marine-core`
^- [x] Mission state machine (Dive → Transit → Survey → Surface → Recover)
^- [x] Lock-free message bus for inter-crate event routing
^- [x] Central event loop
^- [x] rkyv wire-type prelude shared by other marine crates
^- [x] Zero-lock hot-path audit passes (reuse `tools/lock-audit.sh` pattern
      from sister repos)

## Phase 3 — 3D Navigation & Buoyancy State (`tpt-t-marine-nav`, nightly)
^- [x] Crate-local nightly toolchain override + `#![feature(portable_simd)]`,
      CI job scoped to this crate only (rest of workspace stays stable)
^- [x] Custom zero-allocation 15-state Extended Kalman Filter (position,
      velocity, attitude, biases), SIMD-vectorized prediction step
^- [x] INS (IMU) + DVL fusion for GPS-denied dead reckoning
^- [x] Pressure depth sensor fusion
^- [x] Acoustic positioning fusion (USBL/LBL/SBL)
^- [x] Zero-copy sensor ingestion: IMU/DVL packets parsed directly from
      serial DMA buffers into the filter state vector via rkyv casting
^- [x] Biofouling compensation: detect increased drag signature, derate
      mission speed to preserve battery
^- [x] Benchmark: <200µs EKF cycle on a Cortex-A72 at 200Hz
^- [x] Custom zero-allocation WGS84 / UTM / ECEF converters (no `geo`/`proj`)

### `tpt-t-marine-pressure`
^- [x] Depth/pressure sensor management
^- [x] Temperature compensation
^- [x] Hysteresis correction
^- [x] Leak detection via pressure differential monitoring (0.1 PSI
      increase → emergency buoyancy trigger in <100ms)

### `tpt-t-marine-buoyancy`
^- [x] Variable ballast system control
^- [x] Trim pump control
^- [x] Buoyancy engine control
^- [x] Compression-at-depth physics (vehicle mass/volume changes as
      neoprene compresses)

## Phase 4 — Acoustic & Multi-Modal Comms
### `tpt-t-marine-acoustic`
^- [x] Custom Adaptive ARQ (Automatic Repeat reQuest) protocol, sliding
      window
^- [x] Forward error correction (Reed-Solomon) tuned to measured packet
      loss rate
^- [x] Triple-redundancy send path for mission-critical commands (e.g.
      "abort, surface now")
^- [x] rkyv delta-compressed telemetry (state-change-only transmission)
^- [x] Zero-allocation protocol stack in a pre-allocated 4KB buffer (no
      heap, no `Vec`, no `String`)
^- [x] WHOI Micro-Modem, EvoLogics, and LinkQuest modem integration
^- [x] Tolerance validation against 60% packet loss / multi-path fading

### `tpt-t-marine-comms`
^- [x] Multi-modal link manager: acoustic (underwater), RF (surface),
      cellular (near shore), satellite (open ocean)
^- [x] Seamless link-mode transition logic
^- [x] Custom store-and-forward for when all links are down
^- [x] Fiber-optic tether bandwidth prioritization: control packets
      (<1ms) > video (H.265 hardware-encoded) > sonar data

## Phase 5 — Environment Modeling
### `tpt-t-marine-current`
^- [x] ADCP (Acoustic Doppler Current Profiler) data ingestion
^- [x] Drift prediction for AUVs and ASVs

### `tpt-t-marine-tide`
^- [x] Harmonic constituent model
^- [x] Tidal stream prediction at arbitrary points (for ROV ops near
      structures)

## Phase 6 — `tpt-t-marine-sonar` (nightly)
^- [x] Crate-local nightly toolchain override + `#![feature(portable_simd)]`,
      CI job scoped to this crate only
^- [x] Multibeam, sidescan, and forward-looking sonar processing
^- [x] Slab-allocated ring buffer of fixed-size point clouds (each ping
      overwrites the oldest — no per-ping allocation)
^- [x] SIMD-accelerated sound-velocity correction and ray-tracing across
      all 512 beams in parallel
^- [x] Zero-copy point cloud generation
^- [x] Benchmark: deterministic 2ms/ping regardless of scene complexity, at
      512 beams/ping, 20Hz

## Phase 7 — Safety & Maritime Law
### `tpt-t-marine-safety`
^- [x] Flooding detection
^- [x] Emergency buoyancy actions (dropping weights / inflating bladders)
^- [x] Thermal shutdown
^- [x] Lost vehicle procedure: >2 hours comms loss → autonomous ascend,
      surface drift, GPS beacon via satellite until recovered
^- [x] Integration with `tpt-t-domain-bridge` (Phase 1) safety state
      machine — `REQUESTING_TELEOP` trigger on unrecoverable fault

### `tpt-t-marine-colregs`
^- [x] Deterministic finite state machine encoding of COLREGS' 30+ rules
      (not ML-based)
^- [x] Give-way / stand-on classification (head-on, crossing, overtaking)
^- [x] Sound whistle signal decision logic
^- [x] Auditable/traceable decision logging (legally defensible)
^- [x] Benchmark: <50µs per detected vessel

## Phase 8 — Vehicle Execution
### `tpt-t-marine-auv`
^- [x] 3D waypoint following
^- [x] Adaptive sampling: detect thermocline/plume, modify mission to
      sample within battery/time budget
^- [x] Comms-triggered behaviors: on surfacing, establish satellite link,
      transmit compressed summary, accept new waypoints/abort from shore
^- [x] Sleep-wake power management for multi-month deployments (99% sleep,
      wake on scheduled profile or acoustic page-up)

### `tpt-t-marine-glider`
^- [x] Buoyancy-driven flight control (buoyancy changes + rudder only)
^- [x] Ultra-low-power flight profile for multi-month, trans-oceanic
      missions

## Phase 9 — Tethered Operations
### `tpt-t-marine-tether`
^- [x] Lumped-mass chain model (50 nodes), Verlet integration
^- [x] 100Hz simulation on a dedicated pinned core
^- [x] 5-second-ahead tether shape prediction
^- [x] Snag prediction + alert path into `tpt-teleop` pilot UI
^- [x] Tension monitoring and winch control (TMS)

### `tpt-t-marine-rov`
^- [x] 6DOF thruster coordination
^- [x] Manipulator arm control
^- [x] Real-time video/telemetry over fiber-optic tether
^- [x] Tether-aware motion: refuse/limit pilot commands that increase snag
      risk (consumes Phase 9 tether predictions)
^- [x] Haptic feedback to joystick when approaching tether limits

## Phase 10 — `tpt-t-marine-asv`
^- [x] Waypoint following
^- [x] Dynamic positioning (DP): hold position within 1m using GPS/IMU/
      thruster feedback in 2-knot currents
^- [x] COLREGS integration: AIS/radar/camera vessel detection, encounter
      classification, correct maneuver execution (consumes Phase 7 colregs)
^- [x] Wave compensation (heave/pitch/roll) for personnel transfer/crane ops

## Phase 11 — Offshore & Infrastructure Applications
### `tpt-t-marine-offshore`
^- [x] Wind turbine foundation inspection
^- [x] Subsea cable routing
^- [x] Oil & gas wellhead monitoring

### `tpt-t-marine-aquaculture`
^- [x] Fish cage net inspection
^- [x] Biomass estimation via hydroacoustics
^- [x] Mortality detection

## Phase 12 — `tpt-t-marine-teleop` (adapter crate)
^- [x] Depend on `tpt-t-domain-bridge` (Phase 1) and implement
      `DomainTeleopInterface`
^- [x] `ControlCommand` → 6DOF thruster command translation (e.g. "thrust
      forward at 10%, yaw left at 5%")
^- [x] Manipulator arm control surface via second joystick / VR gloves
^- [x] Tether-aware motion override: `tpt-t-marine-tether` (Phase 9) can
      override pilot commands on snag risk
^- [x] Acoustic comms fallback: on fiber-optic tether cut, switch to
      `tpt-t-marine-acoustic` (Phase 4) control at 1Hz, preserving critical
      command capability
^- [x] Buoyancy compensation: auto-adjust for buoyancy changes as ROV
      deploys tools / collects samples (via Phase 3 buoyancy)
^- [x] Smooth autonomy↔teleop handover (no jerk) via
      `on_teleop_engage`/`on_teleop_disengage` hooks
^- [x] End-to-end test: tether-cut → acoustic fallback engaged → 1Hz
      control maintained → tether restored → fiber control resumed

## Phase 13 — `tpt-t-marine-sim` + Visualizer
^- [x] Headless simulator: 6DOF vehicle dynamics with hydrodynamic drag
      coefficients
^- [x] Realistic acoustic channel model (multi-path, loss, latency)
^- [x] Ocean current fields (user-defined or from HYCOM/ROMS ocean models)
^- [x] Tidal harmonics
^- [x] Sonar returns from synthetic seabed geometries
^- [x] Scale target: 30-day AUV mission simulated in 7 hours wall-clock
      (100x real-time)
^- [x] `egui` (MIT) visualizer binary: 3D vehicle trajectories with
      depth color-coding
^- [x] Visualizer: acoustic link quality heatmaps
^- [x] Visualizer: tether catenary visualization
^- [x] Visualizer: multibeam sonar point clouds
^- [x] Visualizer: COLREGS encounter scenarios for ASV testing

## Phase 14 — System Integration & End-to-End
^- [x] Full "Subsea Pipeline Survey" data flow test (spec.txt §6): mission
      upload → ASV transit/DP via colregs → AUV release → buoyancy trim →
      nav 3D fix establishment → 20Hz multibeam survey + edge AI anomaly
      detection → 30-min acoustic check-ins relayed via satellite →
      adaptive high-res sidescan replan → surface recovery → 50GB WiFi
      transfer in 20 min → moon pool reel-in → shore handoff
^- [x] Dependency-matrix conformance audit against §7 (confirm no banned
      dependency — nalgebra, OpenCV/pcl, tokio/crossbeam, geo/proj, serde/
      bincode, `nmea` — entered the tree)
^- [x] Cross-workspace zero-allocation/zero-lock verification (extend
      `tpt-t-integration`-style audit from `tpt-teleop` to cover all marine
      crates)
^- [x] Full `cargo-deny` MIT-chain audit pass across the workspace

## Phase 15 — Hardening, Docs & v1.0.0 Release
^- [x] `cargo clippy --workspace --all-targets -- -D warnings` clean
      (stable workspace)
^- [x] Nightly clippy pass clean for `tpt-t-marine-nav` and
      `tpt-t-marine-sonar`
^- [x] Performance benchmarks validated against every spec target: <200µs
      EKF cycle, 2ms/ping sonar, <100ms leak-to-emergency-buoyancy, <50µs/
      vessel COLREGS, 100Hz/5s-lookahead tether prediction, 100x real-time
      sim
^- [x] Fuzz/property testing for custom zero-allocation parsers (NMEA
      0183/2000, acoustic ARQ framing)
^- [x] Developer docs complete (`docs/quickstart.md`, `docs/ARCHITECTURE.md`,
      crate-level rustdoc)
^- [x] README finalized
^- [x] Tag `v1.0.0` — "v1 Monolithic Release"
