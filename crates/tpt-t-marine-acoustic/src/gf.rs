//! GF(2⁸) arithmetic for Reed-Solomon, with tables built at compile time
//! (const evaluation) — zero runtime setup, zero allocation.
//!
//! Polynomial: x⁸ + x⁴ + x³ + x² + 1 (0x11D), the usual RS/BCH standard.

/// Galois field element.
pub type Gf = u8;

const POLY: u16 = 0x11D;

/// Build one exp/log pair of tables in const context.
const fn build_tables() -> ([Gf; 512], [Gf; 256]) {
    let mut exp = [0u16 as Gf; 512];
    let mut log = [0u16 as Gf; 256];
    let mut x: u16 = 1;
    let mut i = 0;
    while i < 255 {
        exp[i] = x as Gf;
        log[x as usize] = i as Gf;
        x <<= 1;
        if x & 0x100 != 0 {
            x ^= POLY;
        }
        i += 1;
    }
    // Exponent table doubled so `exp[a + b]` never needs a mod-255.
    let mut i = 255;
    while i < 512 {
        exp[i] = exp[i - 255];
        i += 1;
    }
    (exp, log)
}

/// Exponentials (doubled) and discrete logs, const-evaluated.
pub const TABLES: ([Gf; 512], [Gf; 256]) = build_tables();

/// Multiply two field elements.
#[inline]
pub fn mul(a: Gf, b: Gf) -> Gf {
    if a == 0 || b == 0 {
        return 0;
    }
    TABLES.0[TABLES.1[a as usize] as usize + TABLES.1[b as usize] as usize]
}

/// Inverse of a nonzero field element.
#[inline]
pub fn inv(a: Gf) -> Gf {
    TABLES.0[255 - TABLES.1[a as usize] as usize]
}

/// Polynomial evaluation with Horner's method (coefficients highest-first).
pub fn poly_eval(poly: &[Gf], x: Gf) -> Gf {
    let mut y = poly[0];
    for &c in &poly[1..] {
        y = mul(y, x) ^ c;
    }
    y
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn field_axioms() {
        // Multiplicative identity and inverse over a sample of elements.
        for a in [1u8, 2, 7, 63, 128, 200, 255] {
            assert_eq!(mul(a, 1), a);
            assert_eq!(mul(a, inv(a)), 1);
        }
        assert_eq!(mul(0, 137), 0);
    }

    #[test]
    fn multiplication_matches_bitshift() {
        // x·x = x² = 0x04; (x⁸) wraps through the polynomial.
        assert_eq!(mul(0x02, 0x02), 0x04);
        assert_eq!(mul(0x80, 0x02), 0x1D); // x⁷·x = x⁸ ≡ poly
        // Distributivity spot check.
        assert_eq!(mul(0x53, 0xCA), mul(0x53, 0x80) ^ mul(0x53, 0x4A));
    }

    #[test]
    fn poly_eval_horner() {
        // p(x) = x² + 1 at x=2: 4+1 = 5 (coefficients highest-first [1, 0, 1]).
        assert_eq!(poly_eval(&[1, 0, 1], 2), 5);
    }
}
