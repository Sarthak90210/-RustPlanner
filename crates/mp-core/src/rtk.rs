//! RTK base station support for the app's RTK/GPS inject screen, as a thin layer over the
//! file-by-file ports of `ExtLibs/Utilities/rtcm3.cs` and `ubx_m8p.cs`
//! (`extlibs::utilities::{rtcm3, ubx_m8p}`): the byte stream goes through the C# parsers'
//! `Read` state machines, and the UBX packets come from `Ubx.generate` and `ubx_cfg_tmode3`.

use crate::extlibs::utilities::icorrections::ICorrections;
use crate::extlibs::utilities::rtcm3::{crc24, rtcm3, type1005, type1006};
use crate::extlibs::utilities::ubx_m8p::{ubx_cfg_tmode3, ubx_nav_svin, Ubx};

#[derive(Debug, Clone, PartialEq)]
pub enum Frame {
    /// A complete RTCM3 message (preamble to CRC) and its message number.
    Rtcm { msgtype: u16, raw: Vec<u8> },
    /// A UBX message: class, id, payload.
    Ubx { class: u8, id: u8, payload: Vec<u8> },
}

/// CRC-24Q used by RTCM3 (`rtcm3.crc24.crc24q`).
pub fn crc24q(data: &[u8]) -> u32 {
    crc24::crc24q(data, data.len() as u32, 0).unwrap_or(0)
}

/// `Ubx.generate(class, id, payload)`
pub fn ubx(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
    Ubx::generate(class, id, payload)
}

/// Stream splitter: every byte goes through `rtcm3.Read` and `Ubx.Read`, as Mission
/// Planner's inject loop does; a parser that completes a frame yields it.
pub struct Parser {
    rtcm: rtcm3,
    ubx: Ubx,
}

impl Default for Parser {
    fn default() -> Self {
        Parser { rtcm: rtcm3::new(), ubx: Ubx::new() }
    }
}

impl Parser {
    pub fn push(&mut self, data: &[u8]) -> Vec<Frame> {
        let mut out = Vec::new();
        for &b in data {
            match self.rtcm.Read(b) {
                Ok(msgtype) if msgtype >= 0 => {
                    let len = self.rtcm.length().max(0) as usize;
                    out.push(Frame::Rtcm { msgtype: msgtype as u16, raw: self.rtcm.packet().get(..len).map(<[u8]>::to_vec).unwrap_or_default() });
                }
                Ok(_) => {}
                // a decode exception ends that frame; the C# inject loop catches it and goes on
                Err(_) => {
                    self.rtcm.resetParser();
                }
            }
            match self.ubx.Read(b) {
                Ok(id) if id >= 0 => {
                    let len = (self.ubx.length() - 8).max(0) as usize;
                    // Read returns (class << 8) + subclass
                    out.push(Frame::Ubx {
                        class: (id >> 8) as u8,
                        id: id as u8,
                        payload: self.ubx.packet().get(6..6 + len).map(<[u8]>::to_vec).unwrap_or_default(),
                    });
                }
                Ok(_) => {}
                Err(_) => {
                    self.ubx.resetParser();
                }
            }
        }
        out
    }
}

/// RTCM 1005 / 1006 antenna reference point, ECEF metres, from a full frame
/// (`type1005.ecefposition` / `type1006.ecefposition`).
pub fn base_ecef(raw: &[u8]) -> Option<[f64; 3]> {
    let msgtype = crate::extlibs::utilities::rtcm3::getbitu(raw, 24, 12).ok()?;
    match msgtype {
        1005 => {
            let mut t = type1005::default();
            t.Read(raw).ok()?;
            Some(t.ecefposition())
        }
        1006 => {
            let mut t = type1006::default();
            t.Read(raw).ok()?;
            Some(t.ecefposition())
        }
        _ => None,
    }
}

/// `rtcm3.ecef2pos` with the result in degrees (`* rtcm3.R2D`): (lat, lng, height m).
pub fn ecef_to_lla(e: [f64; 3]) -> (f64, f64, f64) {
    let mut pos = [0.0; 3];
    rtcm3::ecef2pos(&e, &mut pos);
    (pos[0] * rtcm3::R2D, pos[1] * rtcm3::R2D, pos[2])
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

/// `Ubx.turnon_off` packet: CFG-MSG rate on ports 1 and 2.
pub fn cfg_msg(class: u8, id: u8, rate: u8) -> Vec<u8> {
    Ubx::generate(0x06, 0x01, &[class, id, 0, rate, 0, rate, 0, 0])
}

/// `new ubx_cfg_tmode3(DurationS, AccLimit)`: survey-in (duration s, accuracy limit m).
pub fn tmode3_survey(dur: u32, acc_m: f64) -> Vec<u8> {
    Ubx::generate(0x06, 0x71, &ubx_cfg_tmode3::survey_in(dur, acc_m).to_bytes())
}

/// `new ubx_cfg_tmode3(lat, lng, alt)`: a fixed position (LLA; ECEF metres when |lat| > 90).
pub fn tmode3_fixed(lat: f64, lng: f64, alt: f64) -> Vec<u8> {
    Ubx::generate(0x06, 0x71, &ubx_cfg_tmode3::fixed(lat, lng, alt, None).to_bytes())
}

/// The packets `Ubx.SetupM8P` writes after its baud-rate hunt (which needs the serial port
/// itself; with direct port access use `Ubx::SetupM8P`): 1 Hz, stationary, NMEA off,
/// MON-VER poll, survey-in and PVT feedback, 1005 every 5 s, MSM7 (or MSM4 for fw
/// 1.30+/F9P), 4072 off, 1230, NAV-VELNED, RXM raw/subframes and MON-HW.
pub fn setup_m8p(m8p_130plus: bool) -> Vec<Vec<u8>> {
    let mut v = vec![
        Ubx::generate(0x6, 0x8, &[0xE8, 0x03, 0x01, 0x00, 0x01, 0x00]),
        Ubx::generate(
            0x6,
            0x24,
            &[
                0xFF, 0xFF, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x0F, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
                0x01, 0x00, 0x00, 0x00, 0x23, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ],
        ),
    ];
    for a in 0..=0xfu8 {
        if a == 0xb || a == 0xc || a == 0xe {
            continue;
        }
        v.push(cfg_msg(0xf0, a, 0));
    }
    v.push(Ubx::generate(0xa, 0x4, &[]));
    v.push(cfg_msg(0x01, 0x3b, 1));
    v.push(cfg_msg(0x01, 0x07, 1));
    v.push(cfg_msg(0xf5, 0x05, 5));
    let (rate1, rate2) = if m8p_130plus { (0, 1) } else { (1, 0) };
    for (id, r) in [(0x4a, rate2), (0x4d, rate1), (0x54, rate2), (0x57, rate1), (0x5e, rate2), (0x61, rate1), (0x7c, rate2), (0x7f, rate1)] {
        v.push(cfg_msg(0xf5, id, r));
    }
    v.push(cfg_msg(0xf5, 0xFE, 0));
    v.push(cfg_msg(0xf5, 0xE6, 5));
    v.push(cfg_msg(0x01, 0x12, 1));
    v.push(cfg_msg(0x02, 0x15, 1));
    v.push(cfg_msg(0x02, 0x10, 1));
    v.push(cfg_msg(0x02, 0x13, 2));
    v.push(cfg_msg(0x02, 0x11, 2));
    v.push(cfg_msg(0x0a, 0x09, 2));
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

/// UBX NAV-SVIN (0x01 0x3B) through `ubx_nav_svin`.
pub fn nav_svin(payload: &[u8]) -> Option<Svin> {
    if payload.len() < ubx_nav_svin::SIZE {
        return None;
    }
    let s = ubx_nav_svin::from_bytes(payload);
    Some(Svin {
        dur: s.dur,
        mean_ecef: s.getECEF(),
        mean_acc: s.meanAcc as f64 / 10000.0,
        obs: s.obs,
        valid: s.valid != 0,
        active: s.active != 0,
    })
}

/// An RTCM3 frame around `payload` (`rtcmpreamble.Write`, payload, CRC-24Q).
pub fn rtcm_frame(payload: &[u8]) -> Vec<u8> {
    let mut v = vec![0xD3, (payload.len() >> 8) as u8 & 3, payload.len() as u8];
    v.extend_from_slice(payload);
    let c = crc24q(&v);
    v.extend([(c >> 16) as u8, (c >> 8) as u8, c as u8]);
    v
}

/// A 1005 message for an ECEF position (`type1005.Write`, station 1 as the C# default).
pub fn encode_1005(e: [f64; 3]) -> Vec<u8> {
    let mut t = type1005::default();
    let _ = t.set_ecefposition(&e);
    let mut buf = vec![0u8; 64];
    let bits = t.Write(&mut buf).unwrap_or(0) as usize;
    let end = bits.div_ceil(8);
    rtcm_frame(&buf[3..end])
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
        // |lat| > 90: ECEF fixed mode (flags 2)
        let f = tmode3_fixed(-4_474_000.0, 2_672_000.0, -3_668_000.0);
        assert_eq!(u16::from_le_bytes([f[6 + 2], f[6 + 3]]), 2);
        assert!(setup_m8p(true).len() > 20);
        let g = gga(-35.5, 149.25, 584.0, (1, 2, 3.0));
        assert!(g.starts_with("$GPGGA,010203.00,3530.0000,S,14915.0000,E,1,") && g.ends_with("\r\n"), "{g}");
    }
}
