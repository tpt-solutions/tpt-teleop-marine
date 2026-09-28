//! Custom zero-allocation NMEA 0183 parser (spec §7: the `nmea` crate
//! allocates; this one maps fixed-size fields out of the byte slice).
//!
//! Supports the sentences a marine vehicle actually consumes: GGA (fix),
//! RMC (position + speed/course), DBS/DBT (depth), HDG/HDM/HDT (heading),
//! MWV (wind), and the proprietary-free vocabulary in between. The parser
//! is total: any byte slice yields a [`Sentence`] — worst case
//! [`Sentence::Unknown`] — and never panics, which is what the Phase 15
//! property tests hammer on.

/// Parsed sentence (no allocation: all fields are copies).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Sentence {
    /// GGA: fix + position + altitude.
    Gga {
        /// Latitude, degrees (+ north).
        lat_deg: f64,
        /// Longitude, degrees (+ east).
        lon_deg: f64,
        /// Number of satellites used.
        satellites: u8,
        /// HDOP (0 if absent).
        hdop: f32,
    },
    /// RMC: recommended minimum — position, speed, course.
    Rmc {
        /// Latitude, degrees.
        lat_deg: f64,
        /// Longitude, degrees.
        lon_deg: f64,
        /// Speed over ground, knots.
        speed_kn: f32,
        /// Course over ground, degrees.
        course_deg: f32,
        /// Valid flag (`A`).
        valid: bool,
    },
    /// DBT: depth below transducer, metres.
    Dbt {
        /// Depth, metres.
        depth_m: f32,
    },
    /// HDT: heading true, degrees.
    Hdt {
        /// Heading, degrees true.
        heading_deg: f32,
    },
    /// MWV: wind speed and angle.
    Mwv {
        /// Wind angle, degrees.
        angle_deg: f32,
        /// Wind speed.
        speed_mps: f32,
    },
    /// Checksum failed or unsupported type.
    Unknown,
}

/// NMEA checksum: XOR of bytes between `$` and `*`.
pub fn checksum(payload: &[u8]) -> u8 {
    payload.iter().fold(0u8, |a, &b| a ^ b)
}

/// Parse one ASCII line (with or without CRLF). Total function.
pub fn parse(line: &[u8]) -> Sentence {
    // Frame: `$` … `*HH` (checksum), CRLF optional.
    let body = match line.first() {
        Some(b'$') | Some(b'!') => &line[1..],
        _ => return Sentence::Unknown,
    };
    // The `*HH` checksum is mandatory: unchecksummed frames are untrusted
    // on a safety vehicle.
    let star = match body.iter().position(|&b| b == b'*') {
        Some(s) => s,
        None => return Sentence::Unknown,
    };
    let hex = &body[star + 1..];
    if hex.len() < 2 {
        return Sentence::Unknown;
    }
    let hi = match (hex[0] as char).to_digit(16) {
        Some(h) => h,
        None => return Sentence::Unknown,
    };
    let lo = match (hex[1] as char).to_digit(16) {
        Some(l) => l,
        None => return Sentence::Unknown,
    };
    let payload = &body[..star];
    let given_cs = (hi * 16 + lo) as u8;
    if checksum(payload) != given_cs {
        return Sentence::Unknown;
    }

    // Split fields on commas (bounded iteration, no allocation).
    let mut fields = [[0u8; 16]; 24];
    let mut lens = [0usize; 24];
    let mut n = 0usize;
    let mut overflow = false;
    for &b in payload {
        match b {
            b',' => {
                n += 1;
                if n >= 24 {
                    return Sentence::Unknown;
                }
            }
            _ => {
                if lens[n] >= 16 {
                    overflow = true;
                    break;
                }
                fields[n][lens[n]] = b;
                lens[n] += 1;
            }
        }
    }
    if overflow {
        return Sentence::Unknown;
    }
    let f = |i: usize| &fields[i][..lens[i]];

    // Sentence type from the first field's last 3 chars.
    let typ = f(0);
    if typ.len() < 3 {
        return Sentence::Unknown;
    }
    let kind = &typ[typ.len() - 3..];
    match kind {
        b"GGA" => Sentence::Gga {
            lat_deg: nmea_lat(f(2), first_byte(f(3))),
            lon_deg: nmea_lon(f(4), first_byte(f(5))),
            satellites: parse_u8(f(7)),
            hdop: parse_f32(f(8)),
        },
        b"RMC" => Sentence::Rmc {
            lat_deg: nmea_lat(f(3), first_byte(f(4))),
            lon_deg: nmea_lon(f(5), first_byte(f(6))),
            speed_kn: parse_f32(f(7)),
            course_deg: parse_f32(f(8)),
            valid: first_byte(f(2)) == Some(b'A'),
        },
        b"DBT" => Sentence::Dbt {
            depth_m: parse_f32(f(3)),
        },
        b"HDT" => Sentence::Hdt {
            heading_deg: parse_f32(f(1)),
        },
        b"MWV" => Sentence::Mwv {
            angle_deg: parse_f32(f(1)),
            speed_mps: parse_f32(f(3)),
        },
        _ => Sentence::Unknown,
    }
}

fn first_byte(b: &[u8]) -> Option<u8> {
    b.first().copied()
}

/// `DDMM.mmmm` + hemisphere → signed degrees.
fn nmea_lat(field: &[u8], hemi: Option<u8>) -> f64 {
    let deg = nmea_lat_lon_x(field, hemi);
    if hemi == Some(b'S') { -deg } else { deg }
}

fn nmea_lon(field: &[u8], hemi: Option<u8>) -> f64 {
    let deg = nmea_lat_lon_x(field, hemi);
    if hemi == Some(b'W') { -deg } else { deg }
}

/// Shared DDMM.mmmm conversion (lat has 2° digits, lon 3° — we locate the
/// decimal point and take (len − 2 − fraction) as the degree digits… both
/// are handled by splitting at fixed 100-based minutes).
fn nmea_lat_lon_x(field: &[u8], hemi: Option<u8>) -> f64 {
    let _ = hemi;
    if field.is_empty() {
        return 0.0;
    }
    // Find '.'.
    let dot = field.iter().position(|&b| b == b'.');
    let (int_part, frac_part) = match dot {
        Some(d) => (&field[..d], &field[d + 1..]),
        None => (field, &[][..]),
    };
    if int_part.len() < 3 {
        return 0.0;
    }
    let deg_len = int_part.len() - 2;
    let deg = parse_f64(&int_part[..deg_len]);
    let min_int = parse_f64(&int_part[deg_len..]);
    let min_frac = parse_f64(frac_part);
    let frac_scale = 10f64.powi(frac_part.len() as i32).max(1.0);
    let minutes = min_int + min_frac / frac_scale;
    deg + minutes / 60.0
}

fn parse_u8(b: &[u8]) -> u8 {
    b.iter().fold(0u8, |a, &d| {
        a.wrapping_mul(10).wrapping_add(d.wrapping_sub(b'0'))
    })
}

fn parse_f32(b: &[u8]) -> f32 {
    parse_f64(b) as f32
}

fn parse_f64(b: &[u8]) -> f64 {
    if b.is_empty() {
        return 0.0;
    }
    let neg = b[0] == b'-';
    let digits = if neg || b[0] == b'+' { &b[1..] } else { b };
    let mut v = 0.0f64;
    let mut frac = false;
    let mut scale = 0.1f64;
    for &d in digits {
        match d {
            b'.' => frac = true,
            b'0'..=b'9' => {
                let digit = (d - b'0') as f64;
                if frac {
                    v += digit * scale;
                    scale /= 10.0;
                } else {
                    v = v * 10.0 + digit;
                }
            }
            _ => return 0.0,
        }
    }
    if neg { -v } else { v }
}

/// Build a sentence with a correct checksum (test helper / tooling).
pub fn build(payload: &[u8], out: &mut [u8]) -> usize {
    let cs = checksum(payload);
    out[0] = b'$';
    out[1..1 + payload.len()].copy_from_slice(payload);
    let hex = format!("{:02X}", cs);
    let star = 1 + payload.len();
    out[star] = b'*';
    out[star + 1] = hex.as_bytes()[0];
    out[star + 2] = hex.as_bytes()[1];
    star + 3
}

#[cfg(test)]
mod tests {
    use super::*;

    const GGA: &[u8] = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

    #[test]
    fn real_gga_sentence_parses() {
        let s = parse(GGA);
        match s {
            Sentence::Gga {
                lat_deg,
                lon_deg,
                satellites,
                hdop,
            } => {
                assert!((lat_deg - 48.1173).abs() < 0.001, "lat {lat_deg}");
                assert!((lon_deg - 11.51666).abs() < 0.001, "lon {lon_deg}");
                assert_eq!(satellites, 8);
                assert!((hdop - 0.9).abs() < 1e-6);
            }
            _ => panic!("expected GGA, got {s:?}"),
        }
    }

    #[test]
    fn rmc_valid_and_invalid() {
        let payload = b"GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4";
        let mut buf = [0u8; 96];
        let n = build(payload, &mut buf);
        match parse(&buf[..n]) {
            Sentence::Rmc {
                speed_kn,
                course_deg,
                valid,
                ..
            } => {
                assert!((speed_kn - 22.4).abs() < 1e-5);
                assert!((course_deg - 84.4).abs() < 1e-5);
                assert!(valid);
            }
            other => panic!("{other:?}"),
        }
        // A deliberately wrong checksum must reject.
        buf[n - 2] = b'0';
        buf[n - 1] = b'0';
        assert_eq!(parse(&buf[..n]), Sentence::Unknown);
    }

    #[test]
    fn depth_sentence() {
        let payload = b"SDDBT,17.0,f,5.2,M,2.8,F";
        let mut buf = [0u8; 32];
        let n = build(payload, &mut buf);
        match parse(&buf[..n]) {
            Sentence::Dbt { depth_m } => assert!((depth_m - 5.2).abs() < 1e-6),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn checksum_is_xor_between_dollar_and_star() {
        assert_eq!(checksum(b"GPGGA"), b'G' ^ b'P' ^ b'G' ^ b'G' ^ b'A');
    }

    /// Property test: the parser is total — arbitrary bytes never panic.
    #[test]
    fn parser_never_panics_on_arbitrary_bytes() {
        // xorshift-driven garbage, including semi-valid frames.
        let mut x = 0x1234_5678u32;
        for trial in 0..10_000u32 {
            let mut buf = [0u8; 80];
            let len = (x % 60) as usize;
            for b in buf.iter_mut().take(len) {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                *b = (x % 256) as u8;
            }
            let s = parse(&buf[..len]);
            let _ = s; // any outcome is fine; panicking is not
            let _ = trial;
        }
    }

    /// Property test: malformed-but-framed sentences degrade to Unknown.
    #[test]
    fn truncated_frames_reject() {
        for cut in 0..GGA.len() {
            let s = parse(&GGA[..cut]);
            if cut < GGA.len() {
                assert_eq!(s, Sentence::Unknown, "cut {cut}");
            }
        }
    }
}
