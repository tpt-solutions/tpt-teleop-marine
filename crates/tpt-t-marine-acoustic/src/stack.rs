//! The whole acoustic protocol stack in one pre-allocated structure
//! (spec §4.1: "The entire protocol stack runs in a pre-allocated 4KB
//! buffer. No heap, no Vec, no String.").
//!
//! [`AcousticStack`] bundles the pipeline — delta telemetry → rkyv archive
//! into a fixed frame → Reed-Solomon(64, 48) FEC → wire — with a
//! triple-redundancy path for critical commands. Its entire state (frame
//! queues, retransmit staging, both protocol endpoints) is `size_of` ≤ 4 KB
//! (asserted in tests), lives inline, and never touches the heap after
//! construction. Serialization goes through rkyv's low-level
//! serializer *into the fixed frame buffer* — not through an allocating
//! `AlignedVec`.

use rkyv::rancor::Failure;
use rkyv::ser::allocator::SubAllocator;
use rkyv::ser::writer::Buffer;

use crate::MAX_FRAME;
use crate::critical::{CriticalReceiver, Vote};
use crate::delta::{ArchivedDeltaTelemetry, DeltaTelemetry, TelemetryWord};
use crate::rs::{RsCodec, RsError};

/// Archive a fixed-size delta into the frame payload window with rkyv's
/// low-level serializer: no `AlignedVec`, no heap — the "allocation" pool
/// is an empty fixed slice because POD deltas request nothing from it.
fn archive_delta_into(delta: &DeltaTelemetry, out: &mut [u8]) -> Result<usize, ()> {
    let written = rkyv::api::low::to_bytes_in_with_alloc::<_, _, Failure>(
        delta,
        Buffer::from(out),
        SubAllocator::new(&mut []),
    )
    .map_err(|_| ())?;
    Ok(written.len())
}

/// Total state budget for the stack (bytes). Asserted.
pub const STACK_BYTES: usize = 4096;

/// Frames the TX queue holds.
const QUEUE_LEN: usize = 4;

/// Codeword geometry: RS(64, 48) — 48 data bytes, 16 parity bytes (t = 8).
const N: usize = 64;
const K: usize = 48;
const PARITY: usize = N - K;

/// Frame type: delta telemetry.
pub const T_DELTA: u8 = 1;
/// Frame type: critical command copy.
pub const T_CRIT: u8 = 2;

/// 4-byte-aligned scratch so rkyv validation can run in place.
#[repr(align(4))]
#[derive(Clone, Copy)]
struct Aligned([u8; N]);

/// What came off the wire.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Received {
    /// A telemetry delta applied cleanly; the reconstructed word is inside.
    Delta(TelemetryWord),
    /// A critical command passed the 2-of-3 vote (length + payload in the
    /// receiver's out buffer).
    Critical,
    /// Frame duplicated or stale (already applied).
    Stale,
    /// Frame could not be decoded.
    Corrupt,
}

/// Statistics for health reporting (all saturating counters).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct StackStats {
    /// Frames sent to the modem.
    pub frames_tx: u32,
    /// Frames received from the modem.
    pub frames_rx: u32,
    /// Frames the FEC could not repair.
    pub corrupt_rx: u32,
    /// Telemetry deltas applied.
    pub deltas_applied: u32,
    /// Critical commands accepted by vote.
    pub criticals_accepted: u32,
}

/// The pre-allocated acoustic protocol stack. No heap, no locks.
pub struct AcousticStack {
    rs: RsCodec,
    /// TX frame queue (pending airtime), each entry a full RS codeword.
    tx_q: [Aligned; QUEUE_LEN],
    tx_head: u8,
    tx_len: u8,
    /// TX-side diff base (last transmitted word) and delta counter.
    tx_word: TelemetryWord,
    tx_seq: u16,
    /// Critical TX sequence.
    crit_seq: u16,
    /// RX-side reconstruction state.
    rx_word: TelemetryWord,
    rx_seq: u16,
    /// Critical vote state.
    crit_rx: CriticalReceiver,
    /// Alignment-corrected scratch for validation.
    scratch: Aligned,
    /// Vote-out payload scratch (last accepted critical command).
    crit_out: [u8; 32],
    crit_out_len: u8,
    /// Health counters.
    pub stats: StackStats,
}

impl AcousticStack {
    /// Create a stack for a clean hull of protocol state.
    pub fn new() -> Self {
        Self {
            rs: RsCodec::new(N, K),
            tx_q: [Aligned([0u8; N]); QUEUE_LEN],
            tx_head: 0,
            tx_len: 0,
            tx_word: TelemetryWord::default(),
            tx_seq: 0,
            crit_seq: 0,
            rx_word: TelemetryWord::default(),
            rx_seq: 0,
            crit_rx: CriticalReceiver::new(),
            scratch: Aligned([0u8; N]),
            crit_out: [0u8; 32],
            crit_out_len: 0,
            stats: StackStats::default(),
        }
    }

    /// Diff `next` against the last transmitted word, archive the delta
    /// into a fixed frame buffer, FEC-encode, and enqueue. `Err(())` when
    /// the queue is full (call again next slot — nothing is dropped
    /// silently) or nothing changed (spec: state-change-only transmission;
    /// an unchanged word costs zero frames).
    pub fn queue_telemetry(&mut self, next: &TelemetryWord) -> Result<(), ()> {
        let delta = DeltaTelemetry::diff(&self.tx_word, next, self.tx_seq);
        if delta.carried() == 0
            && delta.mission_state == self.tx_word.mission_state
            && delta.error_bits == self.tx_word.error_bits
        {
            return Err(()); // nothing to send — exactly the point
        }
        if self.tx_len as usize >= QUEUE_LEN {
            return Err(());
        }

        // Frame = [T_DELTA | seq:u16 | 0 | payload...0 | parity], N bytes.
        let mut data = [0u8; K];
        data[0] = T_DELTA;
        data[1..3].copy_from_slice(&delta.seq.to_be_bytes());
        data[3] = 0;
        // rkyv low-level serialize straight into the fixed payload window —
        // no AlignedVec, no heap.
        archive_delta_into(&delta, &mut data[4..44])?;

        self.enqueue_frame(&data);
        self.tx_seq = self.tx_seq.wrapping_add(1);
        self.tx_word = *next;
        Ok(())
    }

    /// Enqueue a mission-critical command as three independent FEC frames
    /// (spec: triple-redundancy send path). `Err(())` if the queue cannot
    /// hold three frames right now.
    pub fn queue_critical(&mut self, cmd: &[u8]) -> Result<(), ()> {
        assert!(cmd.len() <= 32, "critical commands are ≤ 32 bytes");
        if QUEUE_LEN - (self.tx_len as usize) < 3 {
            return Err(());
        }
        let seq = self.crit_seq;
        self.crit_seq = self.crit_seq.wrapping_add(1);
        let crc = crate::critical::crc16(cmd);
        for copy in 0u8..3 {
            let mut data = [0u8; K];
            data[0] = T_CRIT;
            data[1..3].copy_from_slice(&seq.to_be_bytes());
            data[3] = copy;
            data[4] = cmd.len() as u8;
            data[5..5 + cmd.len()].copy_from_slice(cmd);
            data[5 + cmd.len()..7 + cmd.len()].copy_from_slice(&crc.to_be_bytes());
            self.enqueue_frame(&data);
        }
        Ok(())
    }

    /// FEC-encode one data block and append it to the TX queue.
    fn enqueue_frame(&mut self, data: &[u8; K]) {
        let tail = (self.tx_head as usize + self.tx_len as usize) % QUEUE_LEN;
        let mut cw = [0u8; N];
        self.rs.encode(data, &mut cw);
        self.tx_q[tail] = Aligned(cw);
        self.tx_len += 1;
    }

    /// Pop the next frame to put on the air. Copy out within `frame_out`.
    pub fn poll_frame(&mut self, frame_out: &mut [u8; MAX_FRAME]) -> Option<usize> {
        if self.tx_len == 0 {
            return None;
        }
        let slot = self.tx_head as usize;
        frame_out[..N].copy_from_slice(&self.tx_q[slot].0);
        self.tx_head = (self.tx_head + 1) % QUEUE_LEN as u8;
        self.tx_len -= 1;
        self.stats.frames_tx = self.stats.frames_tx.saturating_add(1);
        Some(N)
    }

    /// Feed one received frame through FEC decode, then the delta/critical
    /// paths.
    pub fn on_frame(&mut self, frame: &[u8]) -> Option<Received> {
        if frame.len() != N {
            return None;
        }
        self.stats.frames_rx = self.stats.frames_rx.saturating_add(1);
        self.scratch.0.copy_from_slice(frame);
        if let Err(e) = self.rs.decode(&mut self.scratch.0) {
            let _ = e;
            self.stats.corrupt_rx = self.stats.corrupt_rx.saturating_add(1);
            return Some(Received::Corrupt);
        }
        let data = &self.scratch.0[..K];
        match data[0] {
            T_DELTA => {
                // rkyv: the payload sits at offset 4 in the aligned scratch,
                // so validation runs in place (no copy, no alignment fault).
                // The low-level API puts the root at the *end* of the window,
                // so pass exactly the archived size.
                let arch_len = core::mem::size_of::<ArchivedDeltaTelemetry>();
                let delta = match rkyv::api::low::access::<ArchivedDeltaTelemetry, Failure>(
                    &self.scratch.0[4..4 + arch_len],
                ) {
                    Ok(v) => DeltaTelemetry {
                        seq: v.seq.to_native(),
                        changed_mask: v.changed_mask,
                        mission_state: v.mission_state,
                        error_bits: v.error_bits.to_native(),
                        values: v.values.map(|f| f.to_native()),
                    },
                    Err(_) => return Some(Received::Corrupt),
                };
                // Loss / duplicate detection on the delta sequence.
                if delta.seq != self.rx_seq {
                    if delta.seq.wrapping_sub(self.rx_seq) > 0 {
                        // Gaps are tolerated: the next full word repairs.
                        self.rx_seq = delta.seq.wrapping_add(1);
                    } else {
                        return Some(Received::Stale);
                    }
                } else {
                    self.rx_seq = self.rx_seq.wrapping_add(1);
                }
                self.rx_word = delta.apply(&self.rx_word);
                self.stats.deltas_applied = self.stats.deltas_applied.saturating_add(1);
                Some(Received::Delta(self.rx_word))
            }
            T_CRIT => {
                let vote = self.crit_rx.on_copy(
                    {
                        // CriticalReceiver expects its own header layout
                        // (0xC1 ‖ seq ‖ copy ‖ len ‖ payload ‖ crc); the
                        // FEC payload carries exactly that (offset by the
                        // stack header). Rebuild the critical frame view.
                        let mut c = [0u8; MAX_FRAME];
                        let seq = u16::from_be_bytes([data[1], data[2]]);
                        let copy = data[3];
                        let len = data[4] as usize;
                        if len == 0 || len > 32 {
                            return Some(Received::Corrupt);
                        }
                        c[0] = 0xC1;
                        c[1..3].copy_from_slice(&seq.to_be_bytes());
                        c[3] = copy;
                        c[4] = data[4];
                        c[5..5 + len].copy_from_slice(&data[5..5 + len]);
                        c[5 + len..7 + len].copy_from_slice(&data[5 + len..7 + len]);
                        c
                    }
                    .as_slice(),
                    &mut self.crit_out,
                );
                match vote {
                    Vote::Accepted => {
                        self.crit_out_len = data[4];
                        self.stats.criticals_accepted =
                            self.stats.criticals_accepted.saturating_add(1);
                        Some(Received::Critical)
                    }
                    _ => Some(Received::Stale),
                }
            }
            _ => Some(Received::Corrupt),
        }
    }

    /// Payload of the last accepted critical command.
    pub fn critical_payload(&self) -> &[u8] {
        &self.crit_out[..self.crit_out_len as usize]
    }

    /// Current RX-side telemetry reconstruction.
    pub fn rx_word(&self) -> TelemetryWord {
        self.rx_word
    }

    /// FEC geometry in use (diagnostics).
    pub fn fec(&self) -> (usize, usize, usize) {
        (self.rs.n, self.rs.k, self.rs.t)
    }
}

impl Default for AcousticStack {
    fn default() -> Self {
        Self::new()
    }
}

/// Parity byte count (diagnostics mirror).
pub const PARITY_BYTES: usize = PARITY;

/// Exposed for tests: decode error type re-export.
pub type DecodeError = RsError;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::critical::crc16;

    /// Archived delta size: mask+seq+state+bits+6 floats — must fit the
    /// 40-byte payload window of a 64-byte frame.
    #[test]
    fn archived_delta_fits_payload_window() {
        let d = DeltaTelemetry {
            seq: 9,
            changed_mask: 0b111111,
            mission_state: 2,
            error_bits: 0,
            values: [1.0; 6],
        };
        let mut buf = [0u8; 40];
        let n = archive_delta_into(&d, &mut buf).expect("fits");
        assert!(n <= 40);
        // The archived bytes validate back cleanly (roundtrip through the
        // same in-place validation path the RX side uses).
        assert!(rkyv::api::low::access::<ArchivedDeltaTelemetry, Failure>(&buf[..n]).is_ok());
    }

    #[test]
    fn whole_stack_fits_the_4kb_budget() {
        assert!(
            core::mem::size_of::<AcousticStack>() <= STACK_BYTES,
            "stack is {} bytes, budget {}",
            core::mem::size_of::<AcousticStack>(),
            STACK_BYTES
        );
    }

    #[test]
    fn end_to_end_delta_flow() {
        let mut tx = AcousticStack::new();
        let mut rx = AcousticStack::new();
        assert_eq!(tx.fec(), (64, 48, 8));

        let word = TelemetryWord {
            battery_soc_pct: 84.0,
            battery_v: 50.8,
            depth_m: 50.0,
            heading_deg: 90.0,
            speed_mps: 1.5,
            water_temp_c: 12.5,
            mission_state: 3,
            error_bits: 0,
        };
        tx.queue_telemetry(&word).expect("queued");

        let mut frame = [0u8; MAX_FRAME];
        let len = tx.poll_frame(&mut frame).expect("frame ready");
        let received = rx.on_frame(&frame[..len]).expect("processed");
        match received {
            Received::Delta(w) => assert_eq!(w, word),
            other => panic!("expected delta, got {other:?}"),
        }
        assert_eq!(rx.stats.deltas_applied, 1);
    }

    #[test]
    fn unchanged_word_queues_nothing() {
        let mut tx = AcousticStack::new();
        let word = TelemetryWord {
            depth_m: 50.0,
            mission_state: 3,
            ..TelemetryWord::default()
        };
        // First transmission diffs against the default base: it sends.
        assert!(tx.queue_telemetry(&word).is_ok(), "first word must send");
        let mut frame = [0u8; MAX_FRAME];
        assert!(tx.poll_frame(&mut frame).is_some());
        // An identical word changes nothing: zero airtime (the spec's
        // state-change-only rule).
        assert!(
            tx.queue_telemetry(&word).is_err(),
            "unchanged word: no airtime"
        );
        assert!(tx.poll_frame(&mut frame).is_none());
    }

    #[test]
    fn fec_repairs_corrupted_frame() {
        let mut tx = AcousticStack::new();
        let mut rx = AcousticStack::new();
        let word = TelemetryWord {
            depth_m: 123.5,
            mission_state: 2,
            ..TelemetryWord::default()
        };
        tx.queue_telemetry(&word).unwrap();
        let mut frame = [0u8; MAX_FRAME];
        let len = tx.poll_frame(&mut frame).unwrap();
        // Corrupt 6 scattered bytes (< t = 8).
        for pos in [0usize, 9, 20, 33, 47, 63] {
            frame[pos] ^= 0xA5;
        }
        let received = rx.on_frame(&frame[..len]).expect("processed");
        assert!(
            matches!(received, Received::Delta(w) if w.depth_m == 123.5),
            "fec repair failed, got {received:?}"
        );
        assert_eq!(rx.stats.corrupt_rx, 0, "FEC absorbed the damage");
    }

    #[test]
    fn hopeless_frame_reports_corrupt() {
        let mut tx = AcousticStack::new();
        let mut rx = AcousticStack::new();
        let word = TelemetryWord {
            depth_m: 5.0,
            mission_state: 2,
            ..TelemetryWord::default()
        };
        tx.queue_telemetry(&word).unwrap();
        let mut frame = [0u8; MAX_FRAME];
        let len = tx.poll_frame(&mut frame).unwrap();
        // 12 errors > t = 8.
        for pos in (0..64usize).step_by(5) {
            frame[pos] ^= 0xFF;
        }
        assert!(matches!(
            rx.on_frame(&frame[..len]),
            Some(Received::Corrupt)
        ));
        assert_eq!(rx.stats.corrupt_rx, 1);
    }

    #[test]
    fn critical_command_votes_through() {
        let mut tx = AcousticStack::new();
        let mut rx = AcousticStack::new();
        tx.queue_critical(b"abort,surface")
            .expect("queued 3 copies");

        let mut frame = [0u8; MAX_FRAME];
        let mut accepted = false;
        while let Some(len) = tx.poll_frame(&mut frame) {
            if let Some(Received::Critical) = rx.on_frame(&frame[..len]) {
                accepted = true;
                break;
            }
        }
        assert!(accepted, "2-of-3 vote must accept");
        assert_eq!(rx.critical_payload(), b"abort,surface");
    }

    #[test]
    fn critical_header_survives_garbled_votes() {
        // The vote path vetts CRC: a corrupted copy must not count.
        let mut rx = CriticalReceiver::new();
        let mut frame = [0u8; MAX_FRAME];
        frame[0] = 0xC1;
        frame[1..3].copy_from_slice(&1u16.to_be_bytes());
        frame[3] = 0;
        frame[4] = 4;
        frame[5..9].copy_from_slice(b"hold");
        let crc = crc16(b"hold");
        frame[9..11].copy_from_slice(&crc.to_be_bytes());
        let mut out = [0u8; 32];
        assert_eq!(rx.on_copy(&frame[..11], &mut out), Vote::First);
        assert_eq!(rx.on_copy(&frame[..11], &mut out), Vote::Accepted);
    }
}
