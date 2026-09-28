//! Adaptive ARQ: sliding window with selective repeat, tuned to measured
//! loss (spec §4.1).
//!
//! The sender keeps up to [`WINDOW`] unacknowledged frames in flight and
//! retransmits any frame whose retry timer has expired. The receiver ACKs
//! with a cumulative base sequence plus a 64-slot selective bitmap, so one
//! ACK repairs a whole burst of losses on the return path. Data is
//! delivered in order, gap-free; with the FEC layer absorbing isolated
//! symbol errors, this rides through the spec's 60 % packet-loss envelope
//! (see the `rides_60pct_loss` test).

use crate::MAX_FRAME;

/// Window size (frames in flight).
pub const WINDOW: usize = 8;

/// Wire abstraction: a lossy frame pipe (the modem + channel).
pub trait Wire {
    /// Transmit one frame; `false` = lost (dropped by the channel).
    fn send(&mut self, frame: &[u8]) -> bool;
    /// Receive one frame into `out`, returning its length, if any.
    fn recv(&mut self, out: &mut [u8; MAX_FRAME]) -> Option<usize>;
}

/// Configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArqConfig {
    /// Retransmit timeout in wire ticks.
    pub timeout_ticks: u32,
    /// Give up after this many attempts per frame (the operator layer
    /// decides what escalation looks like).
    pub max_retries: u8,
}

/// Outgoing payload limit (frame header is 4 bytes).
pub const MAX_PAYLOAD: usize = MAX_FRAME - 4;

const T_DATA: u8 = 0x01;
const T_ACK: u8 = 0x02;
const ACK_LEN: usize = 11;

/// Sender side. Fixed buffers, no allocation.
pub struct Sender {
    cfg: ArqConfig,
    /// Sequence of the oldest unacked frame.
    base: u16,
    /// Next sequence to assign.
    next: u16,
    /// Retransmit store: (bytes, len) per slot (indexed by `seq % WINDOW`).
    store: [([u8; MAX_FRAME], u16); WINDOW],
    /// Last (re)transmission tick per slot; `u32::MAX` = acked/free.
    sent_at: [u32; WINDOW],
    /// Attempts per slot.
    attempts: [u8; WINDOW],
    tick: u32,
}

impl Sender {
    /// Create a sender.
    pub fn new(cfg: ArqConfig) -> Self {
        Self {
            cfg,
            base: 0,
            next: 0,
            store: [([0u8; MAX_FRAME], 0); WINDOW],
            sent_at: [u32::MAX; WINDOW],
            attempts: [0; WINDOW],
            tick: 0,
        }
    }

    /// Enqueue one payload (≤ [`MAX_PAYLOAD`]). `Err(())` = window full or
    /// payload too large; caller retries next tick.
    pub fn send(&mut self, payload: &[u8]) -> Result<(), ()> {
        if payload.len() > MAX_PAYLOAD || self.next >= self.base + WINDOW as u16 {
            return Err(());
        }
        let slot = (self.next % WINDOW as u16) as usize;
        let frame = &mut self.store[slot].0;
        frame[0] = T_DATA;
        frame[1..3].copy_from_slice(&self.next.to_be_bytes());
        frame[3] = payload.len() as u8;
        frame[4..4 + payload.len()].copy_from_slice(payload);
        self.store[slot].1 = (4 + payload.len()) as u16;
        self.attempts[slot] = 0;
        self.sent_at[slot] = u32::MAX - 1; // due immediately on next tick
        self.next += 1;
        Ok(())
    }

    /// Ingest one ACK frame (as produced by [`Receiver::make_ack`]).
    pub fn on_ack(&mut self, frame: &[u8]) {
        if frame.len() < ACK_LEN || frame[0] != T_ACK {
            return;
        }
        let ack_base = u16::from_be_bytes([frame[1], frame[2]]);
        let bitmap = u64::from_be_bytes(frame[3..11].try_into().unwrap());
        // Cumulative: everything below ack_base is delivered.
        if ack_base > self.base {
            for seq in self.base..ack_base.min(self.next) {
                self.sent_at[(seq % WINDOW as u16) as usize] = u32::MAX;
            }
        }
        // Selective: bitmap bit i = frame ack_base+i received.
        for bit in 0..64u64 {
            if bitmap & (1 << bit) != 0 {
                let seq = ack_base + bit as u16;
                if seq >= self.base && seq < self.next {
                    self.sent_at[(seq % WINDOW as u16) as usize] = u32::MAX;
                }
            }
        }
        self.advance_base();
    }

    /// Advance time: (re)transmit new and expired frames through `wire`.
    /// Returns the number of frames transmitted this tick.
    pub fn tick(&mut self, wire: &mut impl Wire) -> usize {
        self.tick = self.tick.wrapping_add(1);
        let mut sent = 0;
        for seq in self.base..self.next {
            let slot = (seq % WINDOW as u16) as usize;
            let due = self.sent_at[slot] == u32::MAX - 1
                || self.tick.wrapping_sub(self.sent_at[slot]) >= self.cfg.timeout_ticks;
            if !due || self.attempts[slot] >= self.cfg.max_retries {
                continue;
            }
            let (frame, len) = &self.store[slot];
            let _ = wire.send(&frame[..*len as usize]);
            self.sent_at[slot] = self.tick;
            self.attempts[slot] = self.attempts[slot].saturating_add(1);
            sent += 1;
        }
        sent
    }

    /// Frames awaiting ACK.
    pub fn in_flight(&self) -> usize {
        (self.next - self.base) as usize
    }

    fn advance_base(&mut self) {
        while self.base < self.next
            && self.sent_at[(self.base % WINDOW as u16) as usize] == u32::MAX
        {
            self.base += 1;
        }
    }
}

/// Receiver side: reorders, dedupes, delivers in order, emits ACKs.
pub struct Receiver {
    base: u16,
    /// Buffered (received, not yet delivered) bitmap relative to `base`.
    buffered: u64,
    /// Buffered payloads by slot.
    slots: [([u8; MAX_PAYLOAD], u8); 64],
}

impl Default for Receiver {
    fn default() -> Self {
        Self::new()
    }
}

impl Receiver {
    /// Create a receiver.
    pub fn new() -> Self {
        Self {
            base: 0,
            buffered: 0,
            slots: [([0u8; MAX_PAYLOAD], 0); 64],
        }
    }

    /// Ingest one data frame; write the current ACK into `ack_out`.
    pub fn on_frame(&mut self, frame: &[u8], ack_out: &mut [u8; MAX_FRAME]) {
        if frame.len() < 4 || frame[0] != T_DATA {
            return;
        }
        let seq = u16::from_be_bytes([frame[1], frame[2]]);
        let len = frame[3] as usize;
        let rel = (seq.wrapping_sub(self.base)) as u64;
        if seq < self.base || rel >= 64 {
            // Duplicate (re-ack) or too far ahead (retransmit cycle catches).
            self.make_ack(ack_out);
            return;
        }
        if self.buffered & (1 << rel) == 0 {
            let mut p = [0u8; MAX_PAYLOAD];
            p[..len].copy_from_slice(&frame[4..4 + len]);
            self.slots[rel as usize] = (p, len as u8);
            self.buffered |= 1 << rel;
        }
        self.make_ack(ack_out);
    }

    /// Build the current ACK frame (cumulative base + selective bitmap).
    pub fn make_ack(&self, out: &mut [u8; MAX_FRAME]) {
        out[0] = T_ACK;
        out[1..3].copy_from_slice(&self.base.to_be_bytes());
        out[3..11].copy_from_slice(&self.buffered.to_be_bytes());
    }

    /// Pop the next in-order payload into `out`, if available.
    pub fn poll(&mut self, out: &mut [u8; MAX_PAYLOAD]) -> Option<usize> {
        if self.buffered & 1 == 0 {
            return None;
        }
        let len = self.slots[0].1 as usize;
        out[..len].copy_from_slice(&self.slots[0].0[..len]);
        self.slots.rotate_left(1);
        self.buffered >>= 1;
        self.base = self.base.wrapping_add(1);
        Some(len)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic lossy channel: drops frames per an xorshift stream,
    /// queues delivered ones in each direction.
    struct LossyChannel {
        rng: u32,
        loss_permille: u16,
        to_receiver: Vec<([u8; MAX_FRAME], usize)>,
        to_sender: Vec<([u8; MAX_FRAME], usize)>,
    }

    impl LossyChannel {
        fn new(seed: u32, loss_permille: u16) -> Self {
            Self {
                rng: seed,
                loss_permille,
                to_receiver: Vec::new(),
                to_sender: Vec::new(),
            }
        }
        fn roll_lost(&mut self) -> bool {
            let mut x = self.rng;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            self.rng = x;
            ((x % 1000) as u16) < self.loss_permille
        }
    }

    impl Wire for LossyChannel {
        fn send(&mut self, frame: &[u8]) -> bool {
            if self.roll_lost() {
                return false;
            }
            let mut buf = [0u8; MAX_FRAME];
            buf[..frame.len()].copy_from_slice(frame);
            self.to_receiver.push((buf, frame.len()));
            true
        }
        fn recv(&mut self, out: &mut [u8; MAX_FRAME]) -> Option<usize> {
            if self.roll_lost() {
                return None; // inbound ACK lost
            }
            let (buf, len) = self.to_sender.pop()?;
            out.copy_from_slice(&buf);
            Some(len)
        }
    }

    #[test]
    fn clean_channel_delivers_in_order() {
        let mut ch = LossyChannel::new(0xC0FFEE, 0);
        let mut tx = Sender::new(ArqConfig {
            timeout_ticks: 4,
            max_retries: 20,
        });
        let mut rx = Receiver::new();
        let mut ack = [0u8; MAX_FRAME];
        let mut out = [0u8; MAX_PAYLOAD];

        for msg in 0u16..8 {
            tx.send(&msg.to_be_bytes()).unwrap();
        }
        let mut delivered = Vec::new();
        for _ in 0..500 {
            tx.tick(&mut ch);
            while !ch.to_receiver.is_empty() {
                let (buf, len) = ch.to_receiver.remove(0);
                rx.on_frame(&buf[..len], &mut ack);
                ch.to_sender.push((ack, ACK_LEN));
            }
            while !ch.to_sender.is_empty() {
                let (buf, len) = ch.to_sender.remove(0);
                tx.on_ack(&buf[..len]);
            }
            while let Some(n) = rx.poll(&mut out) {
                delivered.push(u16::from_be_bytes(out[..2].try_into().unwrap()));
                let _ = n;
            }
            if delivered.len() == 8 {
                break;
            }
        }
        assert_eq!(delivered, (0..8).collect::<Vec<_>>(), "in-order delivery");
    }

    #[test]
    fn rides_60pct_loss() {
        // Spec tolerance: the ARQ layer must deliver through 60 % loss.
        let mut ch = LossyChannel::new(0xBADF00D, 600);
        let mut tx = Sender::new(ArqConfig {
            timeout_ticks: 3,
            max_retries: 255,
        });
        let mut rx = Receiver::new();
        let mut ack = [0u8; MAX_FRAME];
        let mut out = [0u8; MAX_PAYLOAD];

        let total = 20u16;
        let mut next_msg = 0u16;
        let mut delivered = Vec::new();
        for _ in 0..100_000 {
            // Keep the window topped up as ACKs release slots.
            while next_msg < total && tx.send(&next_msg.to_be_bytes()).is_ok() {
                next_msg += 1;
            }
            tx.tick(&mut ch);
            while !ch.to_receiver.is_empty() {
                let (buf, len) = ch.to_receiver.remove(0);
                rx.on_frame(&buf[..len], &mut ack);
                ch.to_sender.push((ack, ACK_LEN));
            }
            while !ch.to_sender.is_empty() {
                let (buf, len) = ch.to_sender.remove(0);
                tx.on_ack(&buf[..len]);
            }
            while let Some(n) = rx.poll(&mut out) {
                delivered.push(u16::from_be_bytes(out[..2].try_into().unwrap()));
                let _ = n;
            }
            if delivered.len() == total as usize {
                break;
            }
        }
        assert_eq!(
            delivered,
            (0..total).collect::<Vec<_>>(),
            "must deliver all frames, in order, at 60 % loss"
        );
    }

    #[test]
    fn window_backpressure_rejects_when_full() {
        let mut tx = Sender::new(ArqConfig {
            timeout_ticks: 4,
            max_retries: 8,
        });
        for m in 0..WINDOW {
            assert!(tx.send(&[m as u8]).is_ok());
        }
        assert!(tx.send(&[0xFF]).is_err(), "full window must reject");
        assert_eq!(tx.in_flight(), WINDOW);
    }
}
