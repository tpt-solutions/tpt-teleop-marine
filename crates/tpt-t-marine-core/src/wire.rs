//! Zero-copy rkyv wire-type prelude shared by the other marine crates.
//!
//! All types here derive `rkyv::Archive` so they can be serialized to a
//! contiguous buffer and read back *in place* with zero allocations and zero
//! copies. They are plain-old-data-safe and contain no `Drop` glue, so
//! archived views are safe to map directly over shared memory, ring-buffer
//! slots, or the wire.
//!
//! The prelude covers the position/command/health vocabulary every marine
//! crate exchanges: NED poses, 3D waypoints, normalized 6DOF thruster
//! commands, vehicle health, and mission events. Sensor-specific types (EKF
//! states, sonar clouds, tether shapes) live in their own crates and are
//! designed to the same fixed-size, no-`Drop` rules.
//!
//! Serialization ([`serialize`]) allocates only the output buffer; the read
//! side is zero-copy via [`access`] (returns a borrowed archived view). Use
//! [`deserialize`] only when an owned `T` is genuinely required.

use bytecheck::CheckBytes;
use rkyv::{Archive, Deserialize, Portable, Serialize};

/// A rigid-body pose in the NED (north–east–down) frame: metres + unit
/// quaternion. The marine workspace's canonical local frame — GPS-denied
/// dead reckoning (`tpt-t-marine-nav`) integrates velocity in NED, and every
/// vehicle crate commands in NED.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct NedPose {
    /// North position, metres from the mission datum.
    pub north_m: f64,
    /// East position, metres from the mission datum.
    pub east_m: f64,
    /// Depth, metres positive-down from the surface.
    pub depth_m: f64,
    /// Heading (yaw), radians, compass convention (0 = north, positive
    /// clockwise).
    pub yaw_rad: f64,
    /// Pitch, radians (positive = nose up).
    pub pitch_rad: f64,
    /// Roll, radians (positive = starboard down).
    pub roll_rad: f64,
}

/// A geodetic fix in WGS-84, used at the surface (GPS available) and for
/// mission planning. `0.0` lat/lon means "no fix" (submerged or cold start).
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct GeoFix {
    /// Latitude, degrees WGS-84.
    pub lat_deg: f64,
    /// Longitude, degrees WGS-84.
    pub lon_deg: f64,
    /// Ellipsoidal altitude (negative when the datum is below sea level).
    pub alt_m: f64,
    /// Horizontal precision estimate (m); `0.0` == unknown.
    pub hprec_m: f32,
}

/// One 3D waypoint of a mission plan.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct Waypoint3D {
    /// North position, metres from the mission datum.
    pub north_m: f64,
    /// East position, metres from the mission datum.
    pub east_m: f64,
    /// Depth, metres positive-down (surface vehicles use `0.0`).
    pub depth_m: f64,
    /// Commanded speed over ground through this waypoint, m/s.
    pub speed_m_s: f32,
    /// Acceptance radius: the waypoint is reached within this distance, m.
    pub accept_radius_m: f32,
}

/// Normalized 6DOF thruster/body command (surge, sway, heave, roll, pitch,
/// yaw). All values `[-1.0, 1.0]`; this is the shared vocabulary between the
/// teleop adapter (`tpt-t-marine-teleop`), the vehicle controllers
/// (`tpt-t-marine-rov` / `-asv`), and the simulator.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct ThrusterCmd6 {
    /// Forward/backward force, normalized.
    pub surge: f32,
    /// Lateral force, normalized.
    pub sway: f32,
    /// Vertical force, normalized.
    pub heave: f32,
    /// Roll moment, normalized.
    pub roll: f32,
    /// Pitch moment, normalized.
    pub pitch: f32,
    /// Yaw moment, normalized.
    pub yaw: f32,
}

impl ThrusterCmd6 {
    /// Clamp every axis into `[-1.0, 1.0]`. Applied by the controller before
    /// thruster mixing; the teleop adapter re-applies defensively.
    pub fn clamped(mut self) -> Self {
        self.surge = self.surge.clamp(-1.0, 1.0);
        self.sway = self.sway.clamp(-1.0, 1.0);
        self.heave = self.heave.clamp(-1.0, 1.0);
        self.roll = self.roll.clamp(-1.0, 1.0);
        self.pitch = self.pitch.clamp(-1.0, 1.0);
        self.yaw = self.yaw.clamp(-1.0, 1.0);
        self
    }

    /// Whether every axis is (within epsilon) zero — the "hold station" / all
    /// stop command.
    pub fn is_zero(self) -> bool {
        self.surge.abs() < f32::EPSILON
            && self.sway.abs() < f32::EPSILON
            && self.heave.abs() < f32::EPSILON
            && self.roll.abs() < f32::EPSILON
            && self.pitch.abs() < f32::EPSILON
            && self.yaw.abs() < f32::EPSILON
    }
}

/// Vehicle health summary carried on the telemetry bus.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct VehicleHealth {
    /// Battery state of charge, percent `0.0..=100.0`.
    pub battery_soc_pct: f32,
    /// Battery pack voltage.
    pub battery_v: f32,
    /// Instantaneous battery current (negative = charging).
    pub battery_a: f32,
    /// Worst-case electronics-hull temperature, °C.
    pub temp_c: f32,
    /// Leak-sensor bitfield, one bit per monitored compartment
    /// (interpretation is the vehicle's own; any set bit is an emergency).
    pub leak_bits: u32,
    /// Seconds since boot.
    pub uptime_s: u32,
}

/// A marine-vehicle event routed on the lock-free bus.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct MarineEvent {
    /// Event kind discriminator (crate-local enum encoded as `u16`).
    pub kind: u16,
    /// Monotonic sequence number.
    pub seq: u32,
    /// Timestamp, microseconds.
    pub timestamp_us: u64,
    /// Opaque payload (interpretation depends on `kind`).
    pub payload: u64,
}

/// Serialize `value` into an owned byte buffer. Allocation occurs only for
/// the output; the read side stays zero-copy via [`access`].
pub fn serialize<T>(value: &T) -> Vec<u8>
where
    T: for<'a> Serialize<
        rkyv::api::high::HighSerializer<
            rkyv::util::AlignedVec,
            rkyv::ser::allocator::ArenaHandle<'a>,
            rkyv::rancor::Error,
        >,
    >,
{
    rkyv::api::high::to_bytes::<rkyv::rancor::Error>(value)
        .expect("rkyv serialize")
        .to_vec()
}

/// Zero-copy read: validate (`CheckBytes`) and return a borrowed archived
/// view.
///
/// Returns `None` if the bytes fail validation (wrong size, corrupt data, or
/// a type/schema mismatch). The returned reference borrows `bytes`, so no
/// copy or allocation occurs.
pub fn access<T>(bytes: &[u8]) -> Option<&<T as Archive>::Archived>
where
    T: Archive,
    <T as Archive>::Archived:
        Portable + for<'a> CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>,
{
    rkyv::api::high::access::<<T as Archive>::Archived, rkyv::rancor::Error>(bytes).ok()
}

/// Deserialize into an owned `T`. Use only when an owned value is required;
/// prefer [`access`] for zero-copy reads.
pub fn deserialize<T>(bytes: &[u8]) -> Option<T>
where
    T: Archive,
    <T as Archive>::Archived: Portable
        + for<'a> CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>
        + Deserialize<T, rkyv::api::high::HighDeserializer<rkyv::rancor::Error>>,
{
    rkyv::api::high::from_bytes::<T, rkyv::rancor::Error>(bytes).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_roundtrip_zero_copy() {
        let pose = NedPose {
            north_m: 1024.25,
            east_m: -512.75,
            depth_m: 50.5,
            yaw_rad: core::f64::consts::FRAC_PI_2,
            pitch_rad: -0.01,
            roll_rad: 0.02,
        };
        let bytes = serialize(&pose);
        let view = access::<NedPose>(&bytes).expect("valid archived pose");
        assert_eq!(view.north_m, pose.north_m);
        assert_eq!(view.east_m, pose.east_m);
        assert_eq!(view.depth_m, pose.depth_m);
        assert_eq!(view.yaw_rad, pose.yaw_rad);
        assert_eq!(view.pitch_rad, pose.pitch_rad);
        assert_eq!(view.roll_rad, pose.roll_rad);
    }

    #[test]
    fn waypoint_and_thruster_roundtrip() {
        let wp = Waypoint3D {
            north_m: 1500.0,
            east_m: -300.0,
            depth_m: 50.0,
            speed_m_s: 1.5, // ~3 knots
            accept_radius_m: 5.0,
        };
        let bytes = serialize(&wp);
        let back = deserialize::<Waypoint3D>(&bytes).unwrap();
        assert_eq!(back, wp);

        let cmd = ThrusterCmd6 {
            surge: 0.10,
            yaw: -0.05,
            ..ThrusterCmd6::default()
        };
        let bytes = serialize(&cmd);
        let view = access::<ThrusterCmd6>(&bytes).unwrap();
        assert_eq!(view.surge, 0.10);
        assert_eq!(view.yaw, -0.05);
        assert!(
            !ThrusterCmd6 {
                surge: 0.10,
                yaw: -0.05,
                ..ThrusterCmd6::default()
            }
            .is_zero()
        );
        assert!(ThrusterCmd6::default().is_zero());
    }

    #[test]
    fn thruster_clamp() {
        let cmd = ThrusterCmd6 {
            surge: 1.5,
            sway: -2.0,
            heave: 0.5,
            roll: 0.0,
            pitch: 0.0,
            yaw: -0.25,
        }
        .clamped();
        assert_eq!(cmd.surge, 1.0);
        assert_eq!(cmd.sway, -1.0);
        assert_eq!(cmd.heave, 0.5);
        assert_eq!(cmd.yaw, -0.25);
    }

    #[test]
    fn corrupt_bytes_rejected() {
        let bytes = serialize(&MarineEvent {
            kind: 1,
            seq: 2,
            timestamp_us: 3,
            payload: 4,
        });
        // A truncated buffer has no valid archived root, so validation must
        // reject it.
        let truncated = &bytes[..bytes.len().saturating_sub(1)];
        assert!(access::<MarineEvent>(truncated).is_none());
    }

    #[test]
    fn health_roundtrip() {
        let health = VehicleHealth {
            battery_soc_pct: 87.5,
            battery_v: 48.2,
            battery_a: -1.5,
            temp_c: 31.0,
            leak_bits: 0,
            uptime_s: 86_400,
        };
        let bytes = serialize(&health);
        let view = access::<VehicleHealth>(&bytes).unwrap();
        assert_eq!(view.battery_soc_pct, 87.5);
        assert_eq!(view.leak_bits, 0);
    }
}
