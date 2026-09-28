//! Zero-copy sensor ingestion: IMU/DVL packets are rkyv-validated views
//! mapped directly over serial DMA buffers (spec §4.2, "Zero-Copy Sensor
//! Fusion").
//!
//! The wire packet types are fixed-size POD-like records; [`access_imu`] /
//! [`access_dvl`] validate the bytes in place and return borrowed archived
//! views. The caller then feeds the small scalar fields straight into
//! [`Ekf15`](crate::ekf::Ekf15) — no deserialization into heap structures,
//! no rehydration copies beyond the few words the filter consumes.

use bytecheck::CheckBytes;
use rkyv::{Archive, Deserialize, Portable, Serialize};

use crate::ekf::{DvlSample, ImuSample};

/// Wire packet for one IMU sample.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct ImuPacket {
    /// Capture timestamp, microseconds.
    pub timestamp_us: u64,
    /// Specific force, m/s², body frame.
    pub accel_mps2: [f32; 3],
    /// Angular rate, rad/s, body frame.
    pub gyro_rad_s: [f32; 3],
    /// Declared sample interval, microseconds (converted to seconds when
    /// fed to the filter).
    pub dt_us: u32,
}

/// Wire packet for one DVL bottom-track report.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct DvlPacket {
    /// Capture timestamp, microseconds.
    pub timestamp_us: u64,
    /// Velocity over ground, NED, m/s.
    pub velocity_ned_mps: [f32; 3],
    /// Altitude above bottom, metres (vehicle keeping, not used by the EKF).
    pub altitude_m: f32,
    /// Bitfield of valid beams (interpretation is the DVL's own).
    pub beam_status: u8,
    /// `true` when the report is a valid bottom lock.
    pub bottom_lock: bool,
}

/// Validate and view an [`ImuPacket`] in place. `None` if the bytes fail
/// `CheckBytes` validation (truncated, corrupt, or a schema mismatch).
pub fn access_imu(bytes: &[u8]) -> Option<&ArchivedImuPacket>
where
    for<'a> ArchivedImuPacket:
        Portable + CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>,
{
    rkyv::api::high::access::<ArchivedImuPacket, rkyv::rancor::Error>(bytes).ok()
}

/// Validate and view a [`DvlPacket`] in place.
pub fn access_dvl(bytes: &[u8]) -> Option<&ArchivedDvlPacket>
where
    for<'a> ArchivedDvlPacket:
        Portable + CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>,
{
    rkyv::api::high::access::<ArchivedDvlPacket, rkyv::rancor::Error>(bytes).ok()
}

impl ArchivedImuPacket {
    /// Map the archived view straight onto the filter's input type — the
    /// "parsed directly from the DMA buffer into the filter state vector"
    /// step. Copy cost: 7 words.
    pub fn to_sample(&self) -> ImuSample {
        ImuSample {
            accel_mps2: self.accel_mps2.map(|f| f.to_native()),
            gyro_rad_s: self.gyro_rad_s.map(|f| f.to_native()),
            dt_s: self.dt_us.to_native() as f32 / 1.0e6,
        }
    }
}

impl ArchivedDvlPacket {
    /// Map the archived view onto the filter's DVL input.
    pub fn to_sample(&self) -> DvlSample {
        DvlSample {
            velocity_ned_mps: self.velocity_ned_mps.map(|f| f.to_native()),
            bottom_lock: self.bottom_lock,
        }
    }
}

/// Serialize a packet to an owned buffer (tests / setup-time tooling).
#[cfg(test)]
pub(crate) fn to_bytes<T>(value: &T) -> Vec<u8>
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_packet_maps_to_filter_input() {
        let pkt = ImuPacket {
            timestamp_us: 100_000,
            accel_mps2: [0.01, -0.02, 9.81],
            gyro_rad_s: [0.0001, 0.0, -0.0002],
            dt_us: 5_000, // 200 Hz
        };
        let bytes = to_bytes(&pkt);
        let view = access_imu(&bytes).expect("valid archived IMU packet");
        let sample = view.to_sample();
        assert_eq!(sample.accel_mps2, [0.01, -0.02, 9.81]);
        assert!((sample.dt_s - 0.005).abs() < 1.0e-9);
        assert_eq!(view.timestamp_us, 100_000);
    }

    #[test]
    fn dvl_packet_maps_and_flags_lock() {
        let pkt = DvlPacket {
            timestamp_us: 200_000,
            velocity_ned_mps: [1.2, -0.4, 0.02],
            altitude_m: 48.6,
            beam_status: 0b1111,
            bottom_lock: true,
        };
        let bytes = to_bytes(&pkt);
        let view = access_dvl(&bytes).expect("valid archived DVL packet");
        let sample = view.to_sample();
        assert!(sample.bottom_lock);
        assert_eq!(sample.velocity_ned_mps, [1.2, -0.4, 0.02]);
        assert_eq!(view.altitude_m, 48.6);
    }

    #[test]
    fn corrupt_bytes_rejected() {
        let pkt = ImuPacket {
            timestamp_us: 1,
            accel_mps2: [0.0; 3],
            gyro_rad_s: [0.0; 3],
            dt_us: 5_000,
        };
        let bytes = to_bytes(&pkt);
        let truncated = &bytes[..bytes.len() - 4];
        assert!(access_imu(truncated).is_none(), "truncated packet rejected");
        assert!(access_dvl(&bytes).is_none(), "wrong type rejected");
    }
}
