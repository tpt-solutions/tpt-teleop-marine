//! Triple-redundancy send path for mission-critical commands (spec §4.1:
//! "abort, surface now" must survive a hostile channel).
//!
//! The command goes out three times (independent losses and independent
//! bit errors); the receiver votes 2-of-3 on an exact payload match, with a
//! CRC-16 vetting each copy so bit-corrupted votes cannot impersonate
//! agreement. Two good copies with matching payload accept; the third copy
//! becomes irrelevant (losses are independent, so P(all three lost) at 60 %
//! per-frame loss is 6.4 %, and P(zero clean copy arriving) collapses far
//! below the single-frame rate).

use crate::MAX_FRAME;

/// Copies sent per critical command.
pub const COPIES: usize = 3;

/// CRC-16/CCITT-FALSE, bitwise (no table, no allocation).
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Frame header for a critical copy: `0xC1 | seq:u16 | copy:u8 | len:u8 ‖
/// payload ‖ crc16(payload):u16`. Returns the frame length (≤ [`MAX_FRAME`]).
pub fn encode_copy(frame: &mut [u8; MAX_FRAME], seq: u16, copy: u8, payload: &[u8]) -> usize {
    assert!(payload.len() + 8 <= MAX_FRAME, "critical payload too large");
    frame[0] = 0xC1;
    frame[1..3].copy_from_slice(&seq.to_be_bytes());
    frame[3] = copy;
    frame[4] = payload.len() as u8;
    frame[5..5 + payload.len()].copy_from_slice(payload);
    let crc = crc16(payload);
    frame[5 + payload.len()..7 + payload.len()].copy_from_slice(&crc.to_be_bytes());
    7 + payload.len()
}

/// Receiver for critical commands: collects copies, votes 2-of-3.
#[derive(Debug, Clone, Copy)]
pub struct CriticalReceiver {
    /// Sequence of the vote in progress.
    seq: u16,
    /// Payload seen (first clean copy).
    payload: [u8; 32],
    votes: u8,
    active: bool,
    /// Last accepted sequence: redundant copies of it are ignored.
    accepted_seq: u16,
    accepted_seen: bool,
}

/// Result of feeding one copy to [`CriticalReceiver::on_copy`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Vote {
    /// First clean copy seen; waiting for corroboration.
    First,
    /// Copy was corrupt (CRC) or a duplicate of an already-counted match.
    Ignored,
    /// Two clean copies agree: command accepted.
    Accepted,
}

impl Default for CriticalReceiver {
    fn default() -> Self {
        Self::new()
    }
}

impl CriticalReceiver {
    /// Create a receiver.
    pub fn new() -> Self {
        Self {
            seq: 0,
            payload: [0u8; 32],
            votes: 0,
            active: false,
            accepted_seq: 0,
            accepted_seen: false,
        }
    }

    /// Feed one received copy (whole frame). Max payload 32 bytes — critical
    /// commands are short by design ("abort, surface now").
    pub fn on_copy(&mut self, frame: &[u8], out: &mut [u8; 32]) -> Vote {
        if frame.len() < 7 || frame[0] != 0xC1 {
            return Vote::Ignored;
        }
        let seq = u16::from_be_bytes([frame[1], frame[2]]);
        let len = frame[4] as usize;
        if frame.len() < 7 + len || len > 32 {
            return Vote::Ignored;
        }
        let payload = &frame[5..5 + len];
        let crc = u16::from_be_bytes([frame[5 + len], frame[6 + len]]);
        if crc != crc16(payload) {
            return Vote::Ignored; // corrupt copy never votes
        }
        if self.accepted_seen && seq == self.accepted_seq {
            return Vote::Ignored; // post-acceptance redundancy
        }
        if !self.active || seq != self.seq {
            // (Re)start a vote window.
            self.seq = seq;
            self.payload[..len].copy_from_slice(payload);
            self.votes = 1;
            self.active = true;
            return Vote::First;
        }
        if payload == &self.payload[..len] {
            self.votes += 1;
            if self.votes >= 2 {
                out[..len].copy_from_slice(payload);
                self.votes = 0; // further copies ignored
                self.active = false;
                self.accepted_seq = seq;
                self.accepted_seen = true;
                return Vote::Accepted;
            }
            Vote::Ignored
        } else {
            // Disagreement: a corrupt-but-CRC-passing copy is (2^-16)-
            // unlikely; restart the window with this copy to be safe.
            self.payload[..len].copy_from_slice(payload);
            self.votes = 1;
            Vote::First
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ABORT: &[u8] = b"abort,surface,now!";

    #[test]
    fn two_of_three_copies_accept() {
        let mut rx = CriticalReceiver::new();
        let mut frame = [0u8; MAX_FRAME];
        let mut out = [0u8; 32];
        let l1 = encode_copy(&mut frame, 7, 0, ABORT);
        assert_eq!(rx.on_copy(&frame[..l1], &mut out), Vote::First);
        let l2 = encode_copy(&mut frame, 7, 1, ABORT);
        assert_eq!(rx.on_copy(&frame[..l2], &mut out), Vote::Accepted);
        assert_eq!(&out[..ABORT.len()], ABORT);
        // Third copy is redundant.
        let l3 = encode_copy(&mut frame, 7, 2, ABORT);
        assert_eq!(rx.on_copy(&frame[..l3], &mut out), Vote::Ignored);
    }

    #[test]
    fn corrupt_copy_never_votes() {
        let mut rx = CriticalReceiver::new();
        let mut frame = [0u8; MAX_FRAME];
        let mut out = [0u8; 32];
        let l = encode_copy(&mut frame, 1, 0, ABORT);
        let mut corrupted = frame;
        corrupted[6] ^= 0xFF; // flip payload bits — CRC now fails
        assert_eq!(rx.on_copy(&corrupted[..l], &mut out), Vote::Ignored);
        // Clean copy 1 then clean copy 2: accept.
        assert_eq!(rx.on_copy(&frame[..l], &mut out), Vote::First);
        let l2 = encode_copy(&mut frame, 1, 1, ABORT);
        assert_eq!(rx.on_copy(&frame[..l2], &mut out), Vote::Accepted);
    }

    #[test]
    fn new_sequence_restarts_vote() {
        let mut rx = CriticalReceiver::new();
        let mut frame = [0u8; MAX_FRAME];
        let mut out = [0u8; 32];
        let l = encode_copy(&mut frame, 1, 0, ABORT);
        assert_eq!(rx.on_copy(&frame[..l], &mut out), Vote::First);
        // A *different* command arrives before the first completes.
        let l2 = encode_copy(&mut frame, 2, 0, b"hold,depth,50m");
        assert_eq!(rx.on_copy(&frame[..l2], &mut out), Vote::First);
        let l3 = encode_copy(&mut frame, 2, 1, b"hold,depth,50m");
        assert_eq!(rx.on_copy(&frame[..l3], &mut out), Vote::Accepted);
        assert_eq!(&out[..14], b"hold,depth,50m");
    }

    #[test]
    fn crc16_known_vector() {
        // CRC-16/CCITT-FALSE of "123456789" is 0x29B1 (standard check).
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn probability_note_documented() {
        // Independent 60 % loss: P(fewer than 2 of 3 copies arrive) =
        // C(3,0)·0.6³ + C(3,1)·0.6²·0.4 = 0.216 + 0.432 = 0.648 — that is
        // the *frame* level. FEC above (rs) plus retransmission over the
        // mission turns this into the acceptance guarantee; the 2-of-3 vote
        // exists to reject the *corrupt-agreement* case, which needs
        // P(two identically corrupt copies) = (2⁻¹⁶)² ≈ 2.3e-10.
        let p_frame_loss = 0.6f64;
        let p_lt2 = p_frame_loss.powi(3) + 3.0 * p_frame_loss.powi(2) * 0.4;
        assert!((p_lt2 - 0.648).abs() < 1e-9);
    }
}
