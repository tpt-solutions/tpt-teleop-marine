# Quickstart — tpt-teleop-marine

> Status: stub. This document will grow as the crates land. The scaffold below
> is enough to build and test the workspace today.

## Prerequisites

- Rust stable (toolchain pinned via `rust-toolchain.toml`).
- Nightly is **only** required for the `tpt-t-marine-nav` and
  `tpt-t-marine-sonar` crates (`#![feature(portable_simd)]`); those crates are
  excluded from the stable workspace and built by dedicated CI jobs.

## Build & test the workspace

```sh
# Whole workspace (stable)
cargo build --workspace --all-targets
cargo test  --workspace

# Lint / format
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

# Dependency license audit (MIT chain)
cargo deny check

# Nightly SIMD crates (in isolation)
cargo build --manifest-path crates/tpt-t-marine-nav/Cargo.toml
cargo build --manifest-path crates/tpt-t-marine-sonar/Cargo.toml
```

## Benchmarks

Every spec target has an in-repo gate:

```sh
# <200 µs EKF cycle (measured ~1.5 µs on x86)
TPT_NAV_STRICT=1 cargo bench --manifest-path crates/tpt-t-marine-nav/Cargo.toml

# 2 ms/ping sonar (measured ~1.6 µs on x86)
TPT_SONAR_STRICT=1 cargo bench --manifest-path crates/tpt-t-marine-sonar/Cargo.toml

# <50 µs/vessel COLREGS (asserted in the engine budget test)
cargo test -p tpt-t-marine-colregs budget_per_vessel -- --nocapture

# 100× real-time simulator throughput (asserted in the integration test)
cargo test -p tpt-t-marine-sim --test pipeline_survey -- --nocapture
```

## Crate layout

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the full map and each crate's
spec section + performance budget.

## Cross-repo bridge

`Phase 1` is `tpt-t-domain-bridge`. Its canonical home is the `tpt-teleop`
repo (every domain repo depends on it); during co-development it is vendored
in this workspace at `crates/tpt-t-domain-bridge` so domain crates use a
plain path dependency:

```toml
[dependencies]
tpt-t-domain-bridge = { workspace = true }
```

When `tpt-teleop` publishes the bridge, switch the dependency over — the
public API (DTI trait, wire types, safety state machine, assist API) is the
stable contract.
