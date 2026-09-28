//! Zero-copy rkyv wire types for the pressure crate: the leak alarm that
//! arms the emergency-buoyancy reflex in `tpt-t-marine-safety`.

use bytecheck::CheckBytes;
use rkyv::{Archive, Deserialize, Portable, Serialize};

/// A latched leak alarm on the wire. Emitted by
/// `tpt-t-marine-pressure::leak` and consumed by `tpt-t-marine-safety`,
/// which converts it into the emergency-buoyancy action (dropping weights /
/// inflating bladders) inside the spec's <100 ms reflex budget.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct LeakAlarm {
    /// Monitored compartment index (vehicle's own numbering).
    pub compartment: u8,
    /// Peak pressure rise over the pre-leak baseline, PSI.
    pub delta_psi: f64,
    /// Rise rate at alarm time, PSI/s.
    pub rate_psi_per_s: f64,
    /// Cause discriminator (`tpt-t-marine-pressure::leak::LeakCause` as
    /// `u16`: 1 = PressureRise, 2 = RapidRise) so the safety crate needs no
    /// compile-time dependency on the sensor crate.
    pub cause: u16,
    /// Alarm timestamp, microseconds.
    pub timestamp_us: u64,
}

/// Serialize `value` into an owned byte buffer (setup-time use).
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

/// Zero-copy read: validate and return a borrowed archived view.
pub fn access<T>(bytes: &[u8]) -> Option<&<T as Archive>::Archived>
where
    T: Archive,
    <T as Archive>::Archived:
        Portable + for<'a> CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>,
{
    rkyv::api::high::access::<<T as Archive>::Archived, rkyv::rancor::Error>(bytes).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::leak::LeakCause;

    #[test]
    fn leak_alarm_roundtrip() {
        let alarm = LeakAlarm {
            compartment: 3,
            delta_psi: 0.105,
            rate_psi_per_s: 1.5,
            cause: LeakCause::PressureRise as u16,
            timestamp_us: 1_700_000_000_000,
        };
        let bytes = serialize(&alarm);
        let view = access::<LeakAlarm>(&bytes).expect("valid archived alarm");
        assert_eq!(view.compartment, 3);
        assert_eq!(view.delta_psi, 0.105);
        assert_eq!(view.cause, 1);
        assert_eq!(view.timestamp_us, 1_700_000_000_000);
    }
}
