# Security Policy

## Supported Versions

| Version | Supported |
|---------|-----------|
| 1.0.x   | ✅        |
| < 1.0   | ❌        |

## Reporting a Vulnerability

**Do not open a public issue for security vulnerabilities.**

Report privately to <security@tpt.solutions> with:

- Affected crate(s) and version(s)
- A description of the vulnerability and its impact
- Steps to reproduce or a proof of concept
- Any suggested mitigation

You will receive an acknowledgment within 48 hours. Please allow up to 90
days for a fix and coordinated disclosure.

## Scope Notes

`tpt-teleop-marine` runs safety-critical software on unmanned underwater and
surface vehicles. Vulnerabilities that could cause:

- loss of vehicle control or position awareness,
- failure of leak-detection / emergency-buoyancy / lost-vehicle reflexes,
- violation of COLREGS collision-avoidance behavior, or
- unauthenticated command injection over acoustic, RF, cellular, or
  satellite links

are treated as **critical** regardless of CVSS base score.

## Design Posture

- No async runtime, no heap allocation, and no locks on the realtime data
  path; memory safety is enforced by the Rust type system (`#![forbid(...)]`
  where practical, `unsafe` audited per `tools/` scripts).
- Dependencies are pinned to the MIT/BSD/ISC chain via `deny.toml` and
  audited in CI (`cargo deny check`); the advisory database is checked on
  every build.
- Deterministic state machines (mission, safety, COLREGS) with complete,
  auditable transition tables — no ML on control paths.
