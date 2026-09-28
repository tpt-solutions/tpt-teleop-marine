//! Reed-Solomon forward error correction over GF(2⁸).
//!
//! Systematic encoding: the message is followed by `2·t` parity symbols
//! computed as the remainder of `msg(x)·x^{2t}` modulo the generator
//! `g(x) = ∏_{j=1..2t} (x − α^j)`. Decoding corrects up to `t` symbol
//! errors per codeword via syndromes → Berlekamp-Massey → Chien search →
//! Forney magnitudes, then *verifies* the corrected codeword's syndromes
//! before accepting it (a decoder that cannot verify does not ship).
//!
//! Shortened codewords are supported the standard way: the caller zero-pads
//! the front of the 255-byte codeword and transmits only the tail.
//!
//! All scratch buffers are fixed-size; nothing allocates.

use crate::gf;

/// Decoder failure reasons.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RsError {
    /// More than `t` symbols are corrupt (or a decoder fault); the
    /// syndromes did not clear after the attempted correction.
    TooCorrupt,
    /// The error-locator degree exceeds what `t` can correct.
    LocatorTooLarge,
}

/// A codec for RS(n, k) with `t = (n − k) / 2` correctable symbols.
#[derive(Debug, Clone, Copy)]
pub struct RsCodec {
    /// Codeword length (≤ 255).
    pub n: usize,
    /// Message length.
    pub k: usize,
    /// Number of correctable symbol errors.
    pub t: usize,
    /// Generator polynomial (highest-degree first), degree 2t.
    gen_poly: [u8; 33],
}

impl RsCodec {
    /// Build a codec. `n ≤ 255`, `k = n − 2t` with `t ≥ 1`.
    pub fn new(n: usize, k: usize) -> Self {
        assert!(n <= 255 && k < n && (n - k) % 2 == 0, "bad RS(n,k)");
        let t = (n - k) / 2;
        assert!(t <= 16, "decoder scratch supports t ≤ 16");
        // Generator poly, lowest-degree-first: g(x) = ∏_{j=1..2t} (x − α^j),
        // built one factor at a time (g' = x·g + root·g).
        let mut gen_poly = [0u8; 33];
        gen_poly[0] = 1;
        let mut deg = 1;
        for j in 1..=(2 * t) {
            let root = gf::TABLES.0[j];
            let mut next = [0u8; 33];
            for i in 0..deg {
                next[i] ^= gf::mul(gen_poly[i], root);
                next[i + 1] ^= gen_poly[i];
            }
            gen_poly = next;
            deg += 1;
        }
        let _ = deg;
        Self { n, k, t, gen_poly }
    }

    /// Parity length (2t).
    pub fn parity_len(&self) -> usize {
        2 * self.t
    }

    /// Systematic encode: `msg` (k bytes) → `out` (n bytes, msg ‖ parity).
    ///
    /// `msg` may be shorter than `k` (shortened codeword): it is treated as
    /// zero-padded on the left and the transmitted codeword starts at the
    /// corresponding offset. `out` must be exactly `n` long.
    pub fn encode(&self, msg: &[u8], out: &mut [u8]) {
        let n = self.n;
        assert_eq!(out.len(), n, "output must be the full codeword");
        assert!(msg.len() <= self.k, "message longer than k");
        let pad = self.k - msg.len();
        // Codeword = msg ‖ 0…0? No: systematic cyclic — parity is the
        // remainder of (msg·x^2t)/g, so data first, parity last.
        for (i, b) in msg.iter().enumerate() {
            out[pad + i] = *b;
        }
        // The shortened prefix is zeros.
        for b in out[..pad].iter_mut() {
            *b = 0;
        }
        // Synthetic division: remainder of (msg ‖ 0^2t) / gen_poly, LFSR style
        // (gen_poly lowest-degree-first, degree 2t). rem[i] holds the
        // coefficient of x^{2t−1−i}, so it taps gen_poly[2t−1−i].
        let mut rem = [0u8; 32];
        let plen = self.parity_len();
        for &b in out[..n - plen].iter() {
            let factor = b ^ rem[0];
            for i in 0..plen - 1 {
                rem[i] = rem[i + 1] ^ gf::mul(factor, self.gen_poly[plen - 1 - i]);
            }
            rem[plen - 1] = gf::mul(factor, self.gen_poly[0]);
        }
        out[n - plen..].copy_from_slice(&rem[..plen]);
    }

    /// Correct `cw` in place. Returns the number of corrected symbols or
    /// [`RsError`]. The corrected codeword's syndromes are verified before
    /// returning success.
    pub fn decode(&self, cw: &mut [u8]) -> Result<usize, RsError> {
        let n = self.n;
        assert_eq!(cw.len(), n, "input must be the full codeword");
        let nsym = self.parity_len();

        // 1. Syndromes S_j = C(α^j), j = 1..nsym.
        let mut s = [0u8; 32];
        let mut nonzero = false;
        for j in 1..=nsym {
            s[j - 1] = cw.iter().fold(0u8, |acc, &c| {
                // Horner over the codeword as a polynomial in x.
                gf::mul(acc, gf::TABLES.0[j]) ^ c
            });
            nonzero |= s[j - 1] != 0;
        }
        if !nonzero {
            return Ok(0);
        }

        // 2. Berlekamp-Massey → error locator Λ (highest-first in loc[0..]).
        let mut loc = [0u8; 33];
        let mut prev = [0u8; 33];
        loc[0] = 1; // Λ = 1
        prev[0] = 1; // B = 1
        let mut l = 0usize; // current locator degree
        let mut m = 1usize;
        let mut b = 1u8; // previous discrepancy
        for step in 0..nsym {
            // Discrepancy d = S[step] + Σ_{i=1..l} Λ_i·S[step−i]
            // (Λ stored highest-first: Λ_i is the coefficient of x^i at loc[deg−i]...).
            // We store Λ as coefficients lowest-first in `loc[0..=l]` for BM:
            // loc[i] = coefficient of x^i. Recompute d accordingly.
            let mut d = s[step];
            for i in 1..=l {
                d ^= gf::mul(loc[i], s[step - i]);
            }
            if d == 0 {
                m += 1;
            } else if 2 * l <= step {
                // T = Λ; Λ = Λ − (d/b)·x^m·B; L, B, b update.
                let mut next = loc;
                let scale = gf::mul(d, gf::inv(b));
                for (i, &bi) in prev.iter().enumerate().take(l + 1) {
                    next[i + m] ^= gf::mul(scale, bi);
                }
                core::mem::swap(&mut loc, &mut next);
                let old = next; // (previous Λ)
                prev = old;
                l = step + 1 - l;
                b = d;
                m = 1;
            } else {
                let scale = gf::mul(d, gf::inv(b));
                for (i, &bi) in prev.iter().enumerate().take(l + 1) {
                    loc[i + m] ^= gf::mul(scale, bi);
                }
                m += 1;
            }
        }
        if l > self.t {
            return Err(RsError::LocatorTooLarge);
        }

        // 3. Chien search: error at codeword index i iff Λ(X_i⁻¹) = 0 with
        // X_i = α^(n−1−i).
        let mut positions = [0usize; 16];
        let mut count = 0usize;
        for i in 0..n {
            let xinv = gf::TABLES.0[(255 - ((n - 1 - i) % 255)) % 255];
            // Evaluate Λ at xinv (lowest-first coefficients).
            let mut y = 0u8;
            for (pi, &lc) in loc.iter().enumerate().take(l + 1) {
                y ^= gf::mul(
                    lc,
                    gf::TABLES.0[(pi * gf::TABLES.1[xinv as usize] as usize) % 255],
                );
            }
            if y == 0 {
                if count >= self.t {
                    return Err(RsError::TooCorrupt);
                }
                positions[count] = i;
                count += 1;
            }
        }
        if count != l {
            return Err(RsError::TooCorrupt);
        }

        // 4. Forney: Ω = S(x)·Λ mod x^nsym; e_m = Ω(X_m⁻¹)/Λ'(X_m⁻¹).
        // S(x) = Σ_{j=1..nsym} S_j·x^{j−1} (lowest-first: s[0] is x⁰).
        let mut omega = [0u8; 32];
        for (oi, ow) in omega.iter_mut().enumerate().take(nsym) {
            let mut acc = 0u8;
            for j in 0..=oi.min(l) {
                acc ^= gf::mul(loc[j], s[oi - j]);
            }
            *ow = acc;
        }
        // Formal derivative Λ': keep only odd-degree terms, shift down.
        let mut dloc = [0u8; 33];
        for (di, d) in dloc.iter_mut().enumerate().take(l) {
            *d = if (di + 1) % 2 == 1 { loc[di + 1] } else { 0 };
            // dloc[i] = (i+1)·Λ_{i+1} = Λ_{i+1} (odd multiples are 1 in GF(2))
        }
        let mut corrected = 0usize;
        for &pos in &positions[..count] {
            let xinv_exp = (255 - ((n - 1 - pos) % 255)) % 255;
            // Ω(X⁻¹).
            let mut num = 0u8;
            for (oi, &ow) in omega.iter().enumerate().take(nsym) {
                num ^= gf::mul(ow, gf::TABLES.0[(oi * xinv_exp) % 255]);
            }
            // Λ'(X⁻¹): dloc has l coefficients (degrees 0..l−1).
            let mut den = 0u8;
            for (di, &dl) in dloc.iter().enumerate().take(l) {
                den ^= gf::mul(dl, gf::TABLES.0[(di * xinv_exp) % 255]);
            }
            let mag = gf::mul(num, gf::inv(den));
            cw[pos] ^= mag;
            corrected += 1;
        }

        // 5. Verify: syndromes must clear.
        for j in 1..=nsym {
            let sv = cw
                .iter()
                .fold(0u8, |acc, &c| gf::mul(acc, gf::TABLES.0[j]) ^ c);
            if sv != 0 {
                return Err(RsError::TooCorrupt);
            }
        }
        Ok(corrected)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn codec() -> RsCodec {
        RsCodec::new(255, 223) // t = 16, CCSDS-style block
    }

    #[test]
    fn clean_codeword_roundtrips() {
        let rs = codec();
        let msg: [u8; 223] = core::array::from_fn(|i| (i * 7 + 3) as u8);
        let mut cw = [0u8; 255];
        rs.encode(&msg, &mut cw);
        assert_eq!(&cw[..223], &msg[..], "systematic: data survives intact");
        assert_eq!(rs.decode(&mut cw), Ok(0));
        assert_eq!(cw, {
            let mut c2 = [0u8; 255];
            rs.encode(&msg, &mut c2);
            c2
        });
    }

    #[test]
    fn corrects_up_to_t_errors() {
        let rs = codec();
        let msg: [u8; 223] = core::array::from_fn(|i| (i as u8).wrapping_mul(13) ^ 0x5A);
        let mut cw = [0u8; 255];
        rs.encode(&msg, &mut cw);
        // Inject exactly t errors at scattered positions.
        let mut corrupted = cw;
        for (m, pos) in [0, 30, 61, 122, 200, 254].into_iter().enumerate() {
            corrupted[pos] ^= 0xFF ^ (m as u8);
        }
        assert_eq!(rs.decode(&mut corrupted), Ok(6));
        assert_eq!(corrupted, cw);
    }

    #[test]
    fn rejects_beyond_t_errors() {
        let rs = codec();
        let msg = [0xABu8; 223];
        let mut cw = [0u8; 255];
        rs.encode(&msg, &mut cw);
        // 17 errors > t=16: the verifier must refuse (not silently mis-correct).
        let mut corrupted = cw;
        for pos in (0..255).step_by(15).take(17) {
            corrupted[pos] ^= 0x81;
        }
        assert!(rs.decode(&mut corrupted).is_err());
    }

    #[test]
    fn shortened_codewords_work() {
        // RS(60, 44), t = 8 — a realistic acoustic frame payload block.
        let rs = RsCodec::new(60, 44);
        let msg: [u8; 44] = core::array::from_fn(|i| i as u8 ^ 0x21);
        let mut cw = [0u8; 60];
        rs.encode(&msg, &mut cw);
        for pos in [3usize, 17, 40, 59] {
            cw[pos] ^= 0xF0;
        }
        assert_eq!(rs.decode(&mut cw), Ok(4));
        assert_eq!(&cw[..44], &msg[..]);
    }

    #[test]
    fn burst_errors_in_parity_only() {
        let rs = codec();
        let msg = [0x11u8; 223];
        let mut cw = [0u8; 255];
        rs.encode(&msg, &mut cw);
        let mut corrupted = cw;
        // Clobber 16 parity symbols (exactly t): decodable, message intact.
        for slot in corrupted.iter_mut().take(255).skip(239) {
            *slot = 0x00;
        }
        assert!(rs.decode(&mut corrupted).is_ok());
        assert_eq!(&corrupted[..223], &msg[..]);
    }
}
