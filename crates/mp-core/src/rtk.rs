//! RTK base station support (Utilities/rtcm3.cs, ubx_m8p.cs): split a GPS byte stream into
//! RTCM3 and u-blox UBX frames, decode the 1005/1006 base position, and build the UBX
//! packets Mission Planner sends to configure an M8P / F9P as a survey-in base.

#[derive(Debug, Clone, PartialEq)]
pub enum Frame {
    /// A complete RTCM3 message (preamble to CRC) and its message number.
    Rtcm { msgtype: u16, raw: Vec<u8> },
    /// A UBX message: class, id, payload.
    Ubx { class: u8, id: u8, payload: Vec<u8> },
}

/// CRC-24Q used by RTCM3.
pub fn crc24q(data: &[u8]) -> u32 {
    let mut crc: u32 = 0;
    for &b in data {
        crc ^= (b as u32) << 16;
        for _ in 0..8 {
            crc <<= 1;
            if crc & 0x0100_0000 != 0 {
                crc ^= 0x0186_4CFB;
            }
        }
    }
    crc & 0x00FF_FFFF
}

pub fn ubx_checksum(data: &[u8]) -> [u8; 2] {
    let (mut a, mut b) = (0u8, 0u8);
    for &x in data {
        a = a.wrapping_add(x);
        b = b.wrapping_add(a);
    }
    [a, b]
}

/// ubx_m8p.generate
pub fn ubx(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
    let mut v = vec![0xb5, 0x62, class, id, (payload.len() & 0xff) as u8, (payload.len() >> 8) as u8];
    v.extend_from_slice(payload);
    let ck = ubx_checksum(&v[2..]);
    v.extend(ck);
    v
}

/// Stream splitter: bytes in, frames out. Bytes that are neither are dropped.
#[derive(Default)]
pub struct Parser {
    buf: Vec<u8>,
}

impl Parser {
    pub fn push(&mut self, data: &[u8]) -> Vec<Frame> {
        self.buf.extend_from_slice(data);
        let mut out = Vec::new();
        let mut i = 0;
        while i < self.buf.len() {
            let b = &self.buf[i..];
            if b[0] == 0xD3 {
                if b.len() < 3 {
                    break;
                }
                let len = (((b[1] & 0x03) as usize) << 8) | b[2] as usize;
                if b[1] & 0xFC != 0 {
                    i += 1;
                    continue;
                }
                if b.len() < len + 6 {
                    break;
                }
                let crc = ((b[len + 3] as u32) << 16) | ((b[len + 4] as u32) << 8) | b[len + 5] as u32;
                if crc24q(&b[..len + 3]) == crc && len >= 2 {
                    let msgtype = ((b[3] as u16) << 4) | (b[4] as u16 >> 4);
                    out.push(Frame::Rtcm { msgtype, raw: b[..len + 6].to_vec() });
                    i += len + 6;
                } else {
                    i += 1;
                }
                continue;
            }
            if b[0] == 0xB5 {
                if b.len() < 2 {
                    break;
                }
                if b[1] != 0x62 {
                    i += 1;
                    continue;
                }
                if b.len() < 6 {
                    break;
                }
                let len = b[4] as usize | ((b[5] as usize) << 8);
                if len > 4096 {
                    i += 1;
                    continue;
                }
                if b.len() < len + 8 {
                    break;
                }
                if ubx_checksum(&b[2..6 + len]) == [b[6 + len], b[7 + len]] {
                    out.push(Frame::Ubx { class: b[2], id: b[3], payload: b[6..6 + len].to_vec() });
                    i += len + 8;
                } else {
                    i += 1;
                }
                continue;
            }
            i += 1;
        }
        self.buf.drain(..i);
        if self.buf.len() > 8192 {
            self.buf.clear();
        }
        out
    }
}

fn bits(data: &[u8], pos: usize, len: usize) -> u64 {
    let mut v = 0u64;
    for k in pos..pos + len {
        let byte = data.get(k / 8).copied().unwrap_or(0);
        v = (v << 1) | ((byte >> (7 - k % 8)) & 1) as u64;
    }
    v
}

fn sbits(data: &[u8], pos: usize, len: usize) -> i64 {
    let v = bits(data, pos, len) as i64;
    if v & (1 << (len - 1)) != 0 {
        v - (1 << len)
    } else {
        v
    }
}

/// RTCM 1005 / 1006 antenna reference point, ECEF metres, from a full frame.
pub fn base_ecef(raw: &[u8]) -> Option<[f64; 3]> {
    let p = raw.get(3..raw.len().checked_sub(3)?)?;
    let t = bits(p, 0, 12);
    if t != 1005 && t != 1006 {
        return None;
    }
    // 12 type, 12 station, 6 itrf, 4 flags, 38 X, 2 flags, 38 Y, 2 reserved, 38 Z
    let x = sbits(p, 34, 38) as f64 * 0.0001;
    let y = sbits(p, 74, 38) as f64 * 0.0001;
    let z = sbits(p, 114, 38) as f64 * 0.0001;
    Some([x, y, z])
}

/// rtcm3.ecef2pos: ECEF -> (lat deg, lng deg, height m) on WGS84.
pub fn ecef_to_lla(e: [f64; 3]) -> (f64, f64, f64) {
    const RE: f64 = 6_378_137.0;
    const FE: f64 = 1.0 / 298.257223563;
    let e2 = FE * (2.0 - FE);
    let r2 = e[0] * e[0] + e[1] * e[1];
    let (mut z, mut zk, mut v) = (e[2], 0.0, RE);
    while (z - zk).abs() >= 1e-4 {
        zk = z;
        let sinp = z / (r2 + z * z).sqrt();
        v = RE / (1.0 - e2 * sinp * sinp).sqrt();
        z = e[2] + v * e2 * sinp;
    }
    let lat = if r2 > 1e-12 { (z / r2.sqrt()).atan() } else if e[2] > 0.0 { std::f64::consts::FRAC_PI_2 } else { -std::f64::consts::FRAC_PI_2 };
    let lng = if r2 > 1e-12 { e[1].atan2(e[0]) } else { 0.0 };
    let h = (r2 + z * z).sqrt() - v;
    (lat.to_degrees(), lng.to_degrees(), h)
}

/// RTCM message number -> the status light it drives (labelbase / gps / glonass / ...).
pub fn constellation(msgtype: u16) -> Option<&'static str> {
    Some(match msgtype {
        1001..=1004 | 1071..=1077 => "gps",
        1005 | 1006 | 4072 => "base",
        1009..=1012 | 1081..=1087 => "glonass",
        1091..=1097 => "galileo",
        1121..=1127 => "beidou",
        _ => return None,
    })
}

/// ubx_m8p.turnon_off: CFG-MSG rate on all ports.
pub fn cfg_msg(class: u8, id: u8, rate: u8) -> Vec<u8> {
    ubx(0x06, 0x01, &[class, id, 0, rate, 0, rate, 0, 0])
}

/// ubx_cfg_tmode3 survey-in (duration s, accuracy limit m).
pub fn tmode3_survey(dur: u32, acc_m: f64) -> Vec<u8> {
    let mut p = vec![0u8, 0];
    p.extend(1u16.to_le_bytes());
    p.extend([0u8; 12]);
    p.extend([0u8; 4]);
    p.extend(0u32.to_le_bytes());
    p.extend(dur.to_le_bytes());
    p.extend(((acc_m * 10000.0) as u32).to_le_bytes());
    p.extend([0u8; 8]);
    ubx(0x06, 0x71, &p)
}

/// ubx_cfg_tmode3 fixed position (lat/lng degrees, alt m; LLA mode).
pub fn tmode3_fixed(lat: f64, lng: f64, alt: f64) -> Vec<u8> {
    let (x, y, z) = ((lat * 1e7) as i32, (lng * 1e7) as i32, (alt * 100.0) as i32);
    let hp = |v: f64, i: i32| ((v - i as f64) * 100.0) as i8 as u8;
    let mut p = vec![0u8, 0];
    p.extend((256u16 + 2).to_le_bytes());
    p.extend(x.to_le_bytes());
    p.extend(y.to_le_bytes());
    p.extend(z.to_le_bytes());
    p.extend([hp(lat * 1e7, x), hp(lng * 1e7, y), hp(alt * 100.0, z), 0]);
    p.extend(1u32.to_le_bytes()); // fixedPosAcc 0.001 m * 1000
    p.extend(60u32.to_le_bytes());
    p.extend(2000u32.to_le_bytes());
    p.extend([0u8; 8]);
    ubx(0x06, 0x71, &p)
}

/// ubx_m8p.SetupM8P without the baud-rate hunt: 1 Hz, stationary, NMEA off, survey-in and
/// PVT feedback, 1005 every 5 s, MSM7 (or MSM4 for fw 1.30+/F9P) every second, 1230.
pub fn setup_m8p(m8p_130plus: bool) -> Vec<Vec<u8>> {
    let mut v = vec![
        ubx(0x06, 0x08, &[0xE8, 0x03, 0x01, 0x00, 0x01, 0x00]),
        ubx(0x06, 0x24, &[0xFF, 0xFF, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x0F, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x23, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
    ];
    for a in 0..=0xfu8 {
        if a != 0xb && a != 0xc && a != 0xe {
            v.push(cfg_msg(0xf0, a, 0));
        }
    }
    v.push(ubx(0x0a, 0x04, &[]));
    v.push(cfg_msg(0x01, 0x3b, 1));
    v.push(cfg_msg(0x01, 0x07, 1));
    v.push(cfg_msg(0xf5, 0x05, 5));
    let (r1, r2) = if m8p_130plus { (0, 1) } else { (1, 0) };
    for (id, r) in [(0x4a, r2), (0x4d, r1), (0x54, r2), (0x57, r1), (0x5e, r2), (0x61, r1), (0x7c, r2), (0x7f, r1)] {
        v.push(cfg_msg(0xf5, id, r));
    }
    v.push(cfg_msg(0xf5, 0xfe, 0));
    v.push(cfg_msg(0xf5, 0xe6, 5));
    v.push(cfg_msg(0x01, 0x12, 1));
    v
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Svin {
    pub dur: u32,
    pub mean_ecef: [f64; 3],
    /// metres
    pub mean_acc: f64,
    pub obs: u32,
    pub valid: bool,
    pub active: bool,
}

/// UBX NAV-SVIN (0x01 0x3B).
pub fn nav_svin(payload: &[u8]) -> Option<Svin> {
    if payload.len() < 40 {
        return None;
    }
    let u = |i: usize| u32::from_le_bytes([payload[i], payload[i + 1], payload[i + 2], payload[i + 3]]);
    let s = |i: usize| u(i) as i32 as f64;
    let hp = |i: usize| payload[i] as i8 as f64 * 0.0001;
    Some(Svin {
        dur: u(8),
        mean_ecef: [s(12) / 100.0 + hp(24), s(16) / 100.0 + hp(25), s(20) / 100.0 + hp(26)],
        mean_acc: u(28) as f64 / 10000.0,
        obs: u(32),
        valid: payload[36] != 0,
        active: payload[37] != 0,
    })
}

/// An RTCM3 frame around `payload` (for tests and the simulator).
pub fn rtcm_frame(payload: &[u8]) -> Vec<u8> {
    let mut v = vec![0xD3, (payload.len() >> 8) as u8 & 3, payload.len() as u8];
    v.extend_from_slice(payload);
    let c = crc24q(&v);
    v.extend([(c >> 16) as u8, (c >> 8) as u8, c as u8]);
    v
}

/// Encode a 1005 message for an ECEF position (station 0).
pub fn encode_1005(e: [f64; 3]) -> Vec<u8> {
    let mut bits_out: Vec<bool> = Vec::new();
    let mut put = |v: i64, n: usize| {
        for k in (0..n).rev() {
            bits_out.push((v >> k) & 1 == 1);
        }
    };
    put(1005, 12);
    put(0, 12);
    put(0, 6);
    put(0b0001, 4);
    put((e[0] / 0.0001).round() as i64 & ((1 << 38) - 1), 38);
    put(0, 2);
    put((e[1] / 0.0001).round() as i64 & ((1 << 38) - 1), 38);
    put(0, 2);
    put((e[2] / 0.0001).round() as i64 & ((1 << 38) - 1), 38);
    let mut out = vec![0u8; bits_out.len().div_ceil(8)];
    for (i, b) in bits_out.iter().enumerate() {
        if *b {
            out[i / 8] |= 0x80 >> (i % 8);
        }
    }
    rtcm_frame(&out)
}

/// $GPGGA for an NTRIP caster ("Send NTRIP GGA").
pub fn gga(lat: f64, lng: f64, alt: f64, utc: (u32, u32, f64)) -> String {
    let dm = |v: f64, w: usize| {
        let a = v.abs();
        let d = a.trunc();
        format!("{:0w$}{:07.4}", d as u32, (a - d) * 60.0, w = w)
    };
    let body = format!(
        "GPGGA,{:02}{:02}{:05.2},{},{},{},{},1,10,1.0,{:.1},M,0.0,M,,",
        utc.0,
        utc.1,
        utc.2,
        dm(lat, 2),
        if lat >= 0.0 { "N" } else { "S" },
        dm(lng, 3),
        if lng >= 0.0 { "E" } else { "W" },
        alt
    );
    let ck = body.bytes().fold(0u8, |a, b| a ^ b);
    format!("${body}*{ck:02X}\r\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc24q_check_value() {
        assert_eq!(crc24q(b"123456789"), 0xCDE703);
    }

    #[test]
    fn base_round_trip_through_parser() {
        // Canberra-ish ECEF
        let e = [-4_474_000.1234, 2_672_000.5678, -3_668_000.9012];
        let frame = encode_1005(e);
        let mut p = Parser::default();
        let mut stream = vec![0x00, 0x11];
        stream.extend(&frame);
        stream.extend(ubx(0x01, 0x3b, &[0u8; 40]));
        let frames = p.push(&stream[..10]);
        assert!(frames.is_empty());
        let frames = p.push(&stream[10..]);
        assert_eq!(frames.len(), 2);
        match &frames[0] {
            Frame::Rtcm { msgtype, raw } => {
                assert_eq!(*msgtype, 1005);
                let back = base_ecef(raw).unwrap();
                for k in 0..3 {
                    assert!((back[k] - e[k]).abs() < 0.0002, "{back:?}");
                }
            }
            f => panic!("{f:?}"),
        }
        assert!(matches!(frames[1], Frame::Ubx { class: 1, id: 0x3b, .. }));
    }

    #[test]
    fn ecef_lla() {
        let (lat, lng, h) = ecef_to_lla([6_378_137.0 + 100.0, 0.0, 0.0]);
        assert!(lat.abs() < 1e-9 && lng.abs() < 1e-9 && (h - 100.0).abs() < 1e-3);
        assert_eq!(constellation(1077), Some("gps"));
        assert_eq!(constellation(1230), None);
    }

    #[test]
    fn ubx_packets() {
        let s = tmode3_survey(60, 2.0);
        assert_eq!(s.len(), 8 + 40);
        assert_eq!(&s[..4], &[0xb5, 0x62, 0x06, 0x71]);
        assert_eq!(u32::from_le_bytes([s[6 + 28], s[6 + 29], s[6 + 30], s[6 + 31]]), 20000);
        assert_eq!(tmode3_fixed(-35.0, 149.0, 584.0).len(), 48);
        assert!(setup_m8p(true).len() > 20);
        let g = gga(-35.5, 149.25, 584.0, (1, 2, 3.0));
        assert!(g.starts_with("$GPGGA,010203.00,3530.0000,S,14915.0000,E,1,") && g.ends_with("\r\n"), "{g}");
    }
}
