//! Store-and-forward: when every link is down, telemetry and mission
//! updates queue in a fixed-capacity ring and drain in order when a link
//! returns (spec §3, "custom store-and-forward for when all links are
//! down").
//!
//! Records are fixed-size ([`ForwardRecord`], 32 bytes): a timestamp, a
//! kind, and an opaque payload — big enough for a delta-telemetry summary
//! or a mission-event, small enough that the ring holds hours of check-ins.
//! Overflow drops the *oldest* record (a store-and-forward queue that
//! evicts newest would hide current state; shore wants the freshest
//! position above all).

/// Capacity of the queue (records).
pub const CAPACITY: usize = 256;

/// One stored record (32 bytes, POD-like, rkyv-safe by construction).
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct ForwardRecord {
    /// Record timestamp, microseconds.
    pub timestamp_us: u64,
    /// Kind discriminator (vehicle's own vocabulary).
    pub kind: u16,
    /// Payload length in bytes (≤ 20).
    pub len: u8,
    /// Reserved.
    _pad: u8,
    /// Payload bytes.
    pub payload: [u8; 20],
}

impl ForwardRecord {
    /// Build a record from a payload slice (≤ 20 bytes).
    pub fn new(timestamp_us: u64, kind: u16, payload: &[u8]) -> Self {
        assert!(payload.len() <= 20, "record payload ≤ 20 bytes");
        let mut r = Self {
            timestamp_us,
            kind,
            len: payload.len() as u8,
            _pad: 0,
            payload: [0u8; 20],
        };
        r.payload[..payload.len()].copy_from_slice(payload);
        r
    }

    /// Payload slice.
    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.len as usize]
    }
}

/// The fixed-capacity store-and-forward queue. No allocation, no locks.
#[derive(Debug, Clone, Copy)]
pub struct StoreForward {
    buf: [ForwardRecord; CAPACITY],
    head: usize, // oldest
    len: usize,
    /// Total records ever dropped by overflow (health metric).
    pub dropped: u32,
}

impl Default for StoreForward {
    fn default() -> Self {
        Self::new()
    }
}

impl StoreForward {
    /// Create an empty queue.
    pub fn new() -> Self {
        Self {
            buf: [ForwardRecord::new(0, 0, &[]); CAPACITY],
            head: 0,
            len: 0,
            dropped: 0,
        }
    }

    /// Number of queued records.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Enqueue. On overflow, the oldest record is dropped (`dropped`
    /// increments).
    pub fn push(&mut self, r: ForwardRecord) {
        let tail = (self.head + self.len) % CAPACITY;
        if self.len == CAPACITY {
            // Overwrite oldest: advance head.
            self.buf[self.head] = r;
            self.head = (self.head + 1) % CAPACITY;
            self.dropped = self.dropped.saturating_add(1);
            return;
        }
        self.buf[tail] = r;
        self.len += 1;
    }

    /// Pop the oldest record.
    pub fn pop(&mut self) -> Option<ForwardRecord> {
        if self.len == 0 {
            return None;
        }
        let r = self.buf[self.head];
        self.head = (self.head + 1) % CAPACITY;
        self.len -= 1;
        Some(r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_order() {
        let mut sf = StoreForward::new();
        for k in 0u64..10 {
            sf.push(ForwardRecord::new(k * 1000, 1, &k.to_be_bytes()));
        }
        assert_eq!(sf.len(), 10);
        for k in 0u64..10 {
            let r = sf.pop().unwrap();
            assert_eq!(r.payload(), &k.to_be_bytes());
        }
        assert!(sf.is_empty());
    }

    #[test]
    fn overflow_drops_oldest_and_counts() {
        let mut sf = StoreForward::new();
        for k in 0u64..(CAPACITY as u64 + 10) {
            sf.push(ForwardRecord::new(k, 1, &k.to_be_bytes()));
        }
        assert_eq!(sf.len(), CAPACITY);
        assert_eq!(sf.dropped, 10, "10 oldest records were evicted");
        // First surviving record is #10.
        let r = sf.pop().unwrap();
        assert_eq!(r.timestamp_us, 10);
    }

    #[test]
    fn drain_on_link_recovery_flow() {
        let mut sf = StoreForward::new();
        // All links down for a "day" of 30-minute check-ins (spec §6).
        for k in 0..48u64 {
            sf.push(ForwardRecord::new(k * 1_800_000_000, 7, b"checkin"));
        }
        assert_eq!(sf.len(), 48);
        // Link returns: drain everything in order.
        let mut drained = 0;
        while sf.pop().is_some() {
            drained += 1;
        }
        assert_eq!(drained, 48);
    }
}
