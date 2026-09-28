//! The auditable decision log: every classification, maneuver, and signal
//! the engine emits lands in a fixed ring with the inputs and rule
//! citations that produced it. Legally defensible = replayable.

/// One logged decision.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Decision {
    /// Decision timestamp, microseconds.
    pub timestamp_us: u64,
    /// Target track id.
    pub target_id: u16,
    /// Encounter class (see [`crate::encounter::Encounter`]).
    pub class: u8,
    /// Maneuver emitted (see [`crate::whistle::Maneuver`] family; raw
    /// discriminant, vehicle-local).
    pub maneuver: u8,
    /// Whistle blasts (0 = none).
    pub blasts: u8,
    /// Range at decision time, metres.
    pub range_m: f32,
    /// Relative bearing at decision time, degrees.
    pub bearing_deg: f32,
}

/// Ring capacity (decisions). 4 KiB of history.
pub const LOG_CAP: usize = 128;

/// The fixed-ring decision log. Never allocates; overwrites the oldest.
#[derive(Debug, Clone, Copy)]
pub struct DecisionLog {
    buf: [Decision; LOG_CAP],
    head: usize,
    len: usize,
    /// Total decisions ever made.
    pub total: u64,
}

impl Default for DecisionLog {
    fn default() -> Self {
        Self::new()
    }
}

impl DecisionLog {
    /// Empty log.
    pub const fn new() -> Self {
        Self {
            buf: [Decision {
                timestamp_us: 0,
                target_id: 0,
                class: 0,
                maneuver: 0,
                blasts: 0,
                range_m: 0.0,
                bearing_deg: 0.0,
            }; LOG_CAP],
            head: 0,
            len: 0,
            total: 0,
        }
    }

    /// Append one decision.
    pub fn push(&mut self, d: Decision) {
        let tail = (self.head + self.len) % LOG_CAP;
        self.buf[tail] = d;
        if self.len < LOG_CAP {
            self.len += 1;
        } else {
            self.head = (self.head + 1) % LOG_CAP;
        }
        self.total = self.total.wrapping_add(1);
    }

    /// Number of decisions retained.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Iterate oldest → newest.
    pub fn iter(&self) -> impl Iterator<Item = &Decision> {
        (0..self.len).map(move |k| &self.buf[(self.head + k) % LOG_CAP])
    }

    /// The most recent decision involving `target_id`, if any.
    pub fn latest_for(&self, target_id: u16) -> Option<&Decision> {
        let mut found = None;
        for d in self.iter() {
            if d.target_id == target_id {
                found = Some(d);
            }
        }
        found
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(ts: u64, id: u16) -> Decision {
        Decision {
            timestamp_us: ts,
            target_id: id,
            class: 3,
            maneuver: 1,
            blasts: 1,
            range_m: 900.0,
            bearing_deg: 55.0,
        }
    }

    #[test]
    fn fifo_within_capacity() {
        let mut log = DecisionLog::new();
        for k in 0u64..10 {
            log.push(d(k, 1));
        }
        assert_eq!(log.len(), 10);
        let first = log.iter().next().unwrap();
        assert_eq!(first.timestamp_us, 0);
        assert_eq!(log.total, 10);
    }

    #[test]
    fn ring_overwrites_oldest() {
        let mut log = DecisionLog::new();
        for k in 0u64..(LOG_CAP as u64 + 7) {
            log.push(d(k, 1));
        }
        assert_eq!(log.len(), LOG_CAP);
        assert_eq!(log.iter().next().unwrap().timestamp_us, 7);
        assert_eq!(log.total, LOG_CAP as u64 + 7);
    }

    #[test]
    fn latest_for_finds_the_right_track() {
        let mut log = DecisionLog::new();
        log.push(d(1, 11));
        log.push(d(2, 22));
        log.push(d(3, 11));
        assert_eq!(log.latest_for(11).unwrap().timestamp_us, 3);
        assert_eq!(log.latest_for(22).unwrap().timestamp_us, 2);
        assert!(log.latest_for(99).is_none());
    }
}
