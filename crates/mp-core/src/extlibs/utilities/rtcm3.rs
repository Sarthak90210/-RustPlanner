//! Port of `ExtLibs/Utilities/rtcm3.cs`
//!
//! RTCM v3 framing (CRC-24Q), decoding of the observation (1002/1004/1012, MSM4/MSM7 for
//! GPS/GLONASS/Galileo/QZSS/BeiDou), base position (1005/1006) and GPS ephemeris (1019)
//! messages, and the matching encoders. Bit reads past the end of a buffer throw
//! IndexOutOfRangeException out of `Read`, as in C#.
//!
//! The C# message classes are reference types and `calc_sat_pos` keeps the caller's
//! `pos`/`vel` arrays by reference; here they are values and the arrays are copied.

#![allow(non_camel_case_types, non_snake_case, clippy::excessive_precision)]

use std::sync::atomic::{AtomicI32, AtomicU64, Ordering};
use std::sync::{Arc, RwLock};

use super::icorrections::ICorrections;
use crate::dotnet::collections::{introspective_sort_named, list_capacity_for_count};
use crate::dotnet::io::{console_write, console_write_line};
use crate::dotnet::{
    cast_i32, cast_u16, cast_u32, cast_u8, current_culture, double_get_hash_code, double_to_string, format_fixed,
    index_out_of_range, DateTime, DateTimeKind, Exception,
};

const RTCM3PREAMB: u8 = 0xD3;
const PRUNIT_GPS: f64 = 299792.458; /* rtcm ver.3 unit of gps pseudorange (m) */
const PRUNIT_GLO: f64 = 599584.916; /* rtcm 3 unit of glo pseudorange (m) */
const CLIGHT: f64 = 299792458.0; /* speed of light (m/s) */
#[allow(clippy::approx_constant)] // rtcm3.cs: 3.1415926535898 (IS-GPS), not Math.PI
const SC2RAD: f64 = 3.1415926535898; /* semi-circle to radian (IS-GPS) */
const FREQ1: f64 = 1.57542E9; /* L1/E1  frequency (Hz) */
const FREQ2: f64 = 1.22760E9; /* L2     frequency (Hz) */
const RANGE_MS: f64 = CLIGHT * 0.001; /* range in 1 ms */
const P2_5: f64 = 0.03125; /* 2^-5 */
const P2_10: f64 = 0.0009765625; /* 2^-10 */
const P2_19: f64 = 1.907348632812500E-06; /* 2^-19 */
const P2_24: f64 = 5.960464477539063E-08; /* 2^-24 */
const P2_29: f64 = 1.862645149230957E-09; /* 2^-29 */
const P2_31: f64 = 4.656612873077393E-10; /* 2^-31 */
const P2_33: f64 = 1.164153218269348E-10; /* 2^-33 */
const P2_43: f64 = 1.136868377216160E-13; /* 2^-43 */
const P2_55: f64 = 2.775557561562891E-17; /* 2^-55 */
const RE_WGS84: f64 = 6378137.0; /* earth semimajor axis (WGS84) (m) */
const FE_WGS84: f64 = 1.0 / 298.257223563; /* earth flattening (WGS84) */
const PI: f64 = std::f64::consts::PI; /* pi */

/// `private static double lasttow` (bits of an f64)
static LASTTOW: AtomicU64 = AtomicU64::new(0);
/// `public static int weekGuess`
pub static WEEK_GUESS: AtomicI32 = AtomicI32::new(0);

fn lasttow() -> f64 {
    f64::from_bits(LASTTOW.load(Ordering::Relaxed))
}

fn set_lasttow(v: f64) {
    LASTTOW.store(v.to_bits(), Ordering::Relaxed);
}

fn week_guess() -> i32 {
    WEEK_GUESS.load(Ordering::Relaxed)
}

/// `buff[i/8]`, throwing past the end.
fn byte_at(buff: &[u8], idx: u32) -> Result<u8, Exception> {
    buff.get(idx as usize).copied().ok_or_else(index_out_of_range)
}

/// `getbitu(buff, pos, len)`
pub fn getbitu(buff: &[u8], pos: u32, len: u32) -> Result<u32, Exception> {
    let mut bits: u32 = 0;
    let mut i = pos;
    while i < pos.wrapping_add(len) {
        bits = (bits << 1).wrapping_add(((byte_at(buff, i / 8)? as u32) >> (7 - i % 8)) & 1);
        i += 1;
    }
    Ok(bits)
}

/// `setbitu(buff, pos, len, double data)`: `(uint)data`.
fn setbitu_d(buff: &mut [u8], pos: u32, len: u32, data: f64) -> Result<(), Exception> {
    setbitu(buff, pos, len, cast_u32(data))
}

/// `setbitu(buff, pos, len, uint data)`
pub fn setbitu(buff: &mut [u8], pos: u32, len: u32, data: u32) -> Result<(), Exception> {
    let mut mask = 1u32.wrapping_shl(len.wrapping_sub(1));

    if len == 0 || 32 < len {
        return Ok(());
    }

    let mut i = pos;
    while i < pos + len {
        let idx = (i / 8) as usize;
        let b = buff.get_mut(idx).ok_or_else(index_out_of_range)?;
        if (data & mask) > 0 {
            *b |= (1u32 << (7 - i % 8)) as u8;
        } else {
            *b &= !(1u32 << (7 - i % 8)) as u8;
        }
        i += 1;
        mask >>= 1;
    }
    Ok(())
}

/// `ROUND(x)`: `(int)Math.Floor(x + 0.5)`.
fn round(x: f64) -> f64 {
    cast_i32((x + 0.5).floor()) as f64
}

/// carrier-phase - pseudorange in cycle
fn cp_pr(cp: f64, pr_cyc: f64) -> f64 {
    let mut x = (cp - pr_cyc + 1500.0) % 3000.0;
    if x < 0.0 {
        x += 3000.0;
    }
    x -= 1500.0;
    x
}

fn getbits_38(buff: &[u8], pos: u32) -> Result<f64, Exception> {
    Ok(getbits(buff, pos, 32)? as f64 * 64.0 + getbitu(buff, pos + 32, 6)? as f64)
}

/// `getbits(buff, pos, len)`: sign-extended.
pub fn getbits(buff: &[u8], pos: u32, len: u32) -> Result<i32, Exception> {
    let bits = getbitu(buff, pos, len)?;
    if len == 0 || 32 <= len || (bits & 1u32.wrapping_shl(len.wrapping_sub(1))) == 0 {
        return Ok(bits as i32);
    }
    Ok((bits | (!0u32).wrapping_shl(len)) as i32) /* extend sign */
}

fn set38bits(buff: &mut [u8], pos: u32, value: f64) -> Result<(), Exception> {
    let word_h = cast_i32((value / 64.0).floor());
    let word_l = cast_u32(value - word_h as f64 * 64.0);
    setbits(buff, pos, 32, word_h)?;
    setbitu(buff, pos + 32, 6, word_l)
}

/// `setbits(buff, pos, len, double data)`: `(int)data`.
fn setbits_d(buff: &mut [u8], pos: u32, len: u32, data: f64) -> Result<(), Exception> {
    setbits(buff, pos, len, cast_i32(data))
}

/// `setbits(buff, pos, len, int data)`
fn setbits(buff: &mut [u8], pos: u32, len: u32, mut data: i32) -> Result<(), Exception> {
    if data < 0 {
        data |= 1i32.wrapping_shl(len.wrapping_sub(1));
    } else {
        data &= !1i32.wrapping_shl(len.wrapping_sub(1)); /* set sign bit */
    }
    setbitu(buff, pos, len, data as u32)
}

fn dot(a: &[f64], b: &[f64], mut n: i32) -> f64 {
    let mut c = 0.0;
    loop {
        n -= 1;
        if n < 0 {
            break;
        }
        c += a[n as usize] * b[n as usize];
    }
    c
}

/// `List<ob>.Sort(delegate(ob a, ob b) { return a.prn.CompareTo(b.prn); })`
fn sort_by_prn(obs: &mut Vec<ob>) {
    let cap = list_capacity_for_count(obs.len());
    let _ = introspective_sort_named(obs, cap, "System.Comparison`1[MissionPlanner.Utilities.rtcm3+ob]", |a: &ob, b: &ob| {
        Ok(a.prn.cmp(&b.prn) as i32)
    });
}

/// `public class rtcmpreamble`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct rtcmpreamble {
    pub length: u16,
    pub preamble: u8,
    pub resv1: u8,
}

impl Default for rtcmpreamble {
    fn default() -> Self {
        rtcmpreamble { length: 0, preamble: RTCM3PREAMB, resv1: 0 }
    }
}

impl rtcmpreamble {
    pub fn Read(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let mut i = 0;

        self.preamble = getbitu(buffer, i, 8)? as u8;
        i += 8;
        self.resv1 = getbitu(buffer, i, 6)? as u8;
        i += 6;
        self.length = getbitu(buffer, i, 10)? as u16;
        Ok(())
    }

    pub fn Write<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a mut [u8], Exception> {
        let mut i = 0;

        setbitu(buffer, i, 8, RTCM3PREAMB as u32)?;
        i += 8;
        setbitu(buffer, i, 6, self.resv1 as u32)?;
        i += 6;
        setbitu(buffer, i, 10, self.length as u32)?;

        Ok(buffer)
    }
}

/// `public class rtcmheader`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct rtcmheader {
    pub epoch: u32,
    pub messageno: u16,
    pub nsat: u8,
    pub refstationid: u16,
    pub smoothind: u8,
    pub smoothint: u8,
    pub sync: u8,
}

impl rtcmheader {
    pub fn Read(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let mut i = 24;

        self.messageno = getbitu(buffer, i, 12)? as u16;
        i += 12; /* message no */
        self.refstationid = getbitu(buffer, i, 12)? as u16;
        i += 12; /* ref station id */
        if self.messageno < 1009 || self.messageno > 1012 {
            self.epoch = getbitu(buffer, i, 30)?;
            i += 30; /* gps epoch time */
        } else {
            self.epoch = getbitu(buffer, i, 27)?;
            i += 27; /* glonass epoch time */
        }
        self.sync = getbitu(buffer, i, 1)? as u8;
        i += 1; /* synchronous gnss flag */
        self.nsat = getbitu(buffer, i, 5)? as u8;
        i += 5; /* no of satellites */
        self.smoothind = getbitu(buffer, i, 1)? as u8;
        i += 1; /* smoothing indicator */
        self.smoothint = getbitu(buffer, i, 3)? as u8;
        Ok(())
    }

    pub fn Write<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a mut [u8], Exception> {
        let mut i = 24;

        setbitu(buffer, i, 12, self.messageno as u32)?;
        i += 12; /* message no */
        setbitu(buffer, i, 12, self.refstationid as u32)?;
        i += 12; /* ref station id */
        setbitu(buffer, i, 30, self.epoch)?;
        i += 30; /* gps epoch time */
        setbitu(buffer, i, 1, self.sync as u32)?;
        i += 1; /* synchronous gnss flag */
        setbitu(buffer, i, 5, self.nsat as u32)?;
        i += 5; /* no of satellites */
        setbitu(buffer, i, 1, self.smoothind as u32)?;
        i += 1; /* smoothing indicator */
        setbitu(buffer, i, 3, self.smoothint as u32)?;
        Ok(buffer)
    }
}

/// `public class crc24`
pub struct crc24;

static CRC24QTAB: [u32; 256] = [
    0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17, 0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8,
    0xB42B23, 0xB8B2D5, 0x3EFE2E, 0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E, 0x64CFB0, 0xE2834B,
    0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7, 0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205,
    0x93AEFE, 0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7, 0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696,
    0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077, 0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E, 0x19A3D2,
    0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5, 0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1,
    0xA11107, 0x275DFC, 0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C, 0x7D6C62, 0xFB2099, 0xF7B96F,
    0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375, 0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C,
    0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15, 0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53,
    0xC596A8, 0xC90F5E, 0x4F43A5, 0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C, 0x3347A4, 0xB50B5F,
    0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3, 0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571,
    0x0DB98A, 0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A, 0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2,
    0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703, 0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A, 0x9E1774,
    0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863, 0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE,
    0xE3EB28, 0x65A7D3, 0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA, 0x2AE476, 0xACA88D, 0xA0317B,
    0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61, 0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
    0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8, 0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27,
    0x5B81DC, 0x57182A, 0xD154D1, 0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88, 0x87B4A6, 0x01F85D,
    0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1, 0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA,
    0x7C0401, 0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538,
];

impl crc24 {
    /// `crc24q(buf, len, crc)`: Qualcomm CRC-24Q (mask 0x1864CFB, not reversed, not XOR'd).
    pub fn crc24q(buf: &[u8], len: u32, mut crc: u32) -> Result<u32, Exception> {
        for i in 0..len {
            let idx = ((crc >> 16) ^ byte_at(buf, i)? as u32) as usize;
            crc = ((crc << 8) & 0xFFFFFF) ^ *CRC24QTAB.get(idx).ok_or_else(index_out_of_range)?;
        }
        Ok(crc)
    }
}

/// `public class ob.rawrtcm`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct rawrtcm {
    pub amb: u8,
    pub cnr1: u8,
    pub cnr2: u8,
    pub code1: u8,
    pub code2: u8,
    pub lock1: u8,
    pub lock2: u8,
    pub ppr1: i32,
    pub ppr2: i32,
    pub pr1: u32,
    pub pr21: i32,
    pub prn: u8,
    pub fcn: u8,
}

/// `public class ob`: one satellite observation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ob {
    pub cp: f64,
    pub cp2: f64,
    pub pr: f64,
    pub pr2: f64,
    pub prn: u8,
    pub raw: rawrtcm,
    pub snr: u8,
    pub tow: f64,
    pub week: i32,
    /// `char sys` (default '\0')
    pub sys: char,
}

impl Default for ob {
    fn default() -> Self {
        ob { cp: 0.0, cp2: 0.0, pr: 0.0, pr2: 0.0, prn: 0, raw: rawrtcm::default(), snr: 0, tow: 0.0, week: 0, sys: '\0' }
    }
}

/// `internal static class StaticUtils`
pub struct StaticUtils;

impl StaticUtils {
    fn datum() -> DateTime {
        DateTime::new(1980, 1, 6, 0, 0, 0, DateTimeKind::Utc).expect("valid date")
    }

    /// `GetFromGps(weeknumber, seconds)`
    pub fn GetFromGps(weeknumber: i32, seconds: f64) -> Result<DateTime, Exception> {
        let week = Self::datum().add_days(weeknumber.wrapping_mul(7) as f64)?;
        week.add_seconds(seconds)
    }

    /// `LeapSecondsGPS(year, month)`
    pub fn LeapSecondsGPS(year: i32, month: i32) -> i32 {
        Self::LeapSecondsTAI(year, month) - 19
    }

    /// `LeapSecondsTAI(year, month)` (http://maia.usno.navy.mil/ser7/tai-utc.dat)
    pub fn LeapSecondsTAI(year: i32, month: i32) -> i32 {
        let yyyymm = year.wrapping_mul(100).wrapping_add(month);
        if yyyymm >= 201701 {
            return 37;
        }
        if yyyymm >= 201507 {
            return 36;
        }
        if yyyymm >= 201207 {
            return 35;
        }
        if yyyymm >= 200901 {
            return 34;
        }
        if yyyymm >= 200601 {
            return 33;
        }
        if yyyymm >= 199901 {
            return 32;
        }
        if yyyymm >= 199707 {
            return 31;
        }
        if yyyymm >= 199601 {
            return 30;
        }
        0
    }

    /// `GetFromTime(time, ref week, ref seconds)`: (week, seconds of week).
    pub fn GetFromTime(time: DateTime) -> Result<(i32, f64), Exception> {
        let datum = Self::datum();

        let dif = time - datum;

        let weeks = cast_i32(dif.total_days() / 7.0);

        let week = weeks;

        let dif = time - datum.add_days(weeks.wrapping_mul(7) as f64)?;

        Ok((week, dif.total_seconds()))
    }
}

/// The header fields every observation message starts with, read and discarded as in C#
/// (`type`, `staid`, `tow`, `sync`, `nsat`).
fn read_legacy_header(buffer: &[u8], tow_bits: u32) -> Result<(f64, u32, u32), Exception> {
    let mut i = 24;
    let _type = getbitu(buffer, i, 12)?;
    i += 12;
    let _staid = getbitu(buffer, i, 12)?;
    i += 12;
    let tow = getbitu(buffer, i, tow_bits)? as f64 * 0.001;
    i += tow_bits;
    let _sync = getbitu(buffer, i, 1)?;
    i += 1;
    let nsat = getbitu(buffer, i, 5)?;
    i += 5;
    Ok((tow, nsat, i))
}

/// `public class type1002`: GPS L1 observations.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct type1002 {
    pub nbits: u32,
    pub obs: Vec<ob>,
}

impl type1002 {
    pub fn Read(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let (tow, nsat, _) = read_legacy_header(buffer, 30)?;

        let mut i = 24 + 64;

        let week = week_guess();

        let _gpstime = StaticUtils::GetFromGps(week, tow)?;

        set_lasttow(tow);

        for _a in 0..nsat {
            let mut ob = ob { sys: 'G', tow, week, ..Default::default() };

            ob.raw.prn = getbitu(buffer, i, 6)? as u8;
            i += 6;
            ob.raw.code1 = getbitu(buffer, i, 1)? as u8;
            i += 1;
            ob.raw.pr1 = getbitu(buffer, i, 24)?;
            i += 24;
            ob.raw.ppr1 = getbits(buffer, i, 20)?;
            i += 20;
            ob.raw.lock1 = getbitu(buffer, i, 7)? as u8;
            i += 7;
            ob.raw.amb = getbitu(buffer, i, 8)? as u8;
            i += 8;
            ob.raw.cnr1 = getbitu(buffer, i, 8)? as u8;
            i += 8;

            let pr1 = ob.raw.pr1 as f64 * 0.02 + ob.raw.amb as f64 * PRUNIT_GPS;

            let lam1 = CLIGHT / FREQ1;

            let cp1 = ob.raw.ppr1 as f64 * 0.0005 / lam1;

            if ob.raw.ppr1 as u32 != 0xFFF80000 {
                ob.prn = ob.raw.prn;
                ob.cp = pr1 / lam1 + cp1;
                ob.pr = pr1;
                ob.snr = cast_u8(ob.raw.cnr1 as f64 * 0.25); // *4.0+0.5

                self.obs.push(ob);
            }
        }

        sort_by_prn(&mut self.obs);

        self.nbits = i;
        Ok(())
    }

    pub fn Write(&mut self, buffer: &mut [u8]) -> Result<u32, Exception> {
        let mut i: u32 = 24 + 64;

        for ob in &self.obs {
            let lam1 = CLIGHT / FREQ1;

            let amb = cast_i32((ob.pr / PRUNIT_GPS).floor());
            let pr1 = round((ob.pr - amb as f64 * PRUNIT_GPS) / 0.02);
            let pr1c = pr1 * 0.02 + amb as f64 * PRUNIT_GPS;

            let ppr = cp_pr(ob.cp, pr1c / lam1);

            let ppr1 = round(ppr * lam1 / 0.0005);

            setbitu(buffer, i, 6, ob.prn as u32)?;
            i += 6;
            setbitu(buffer, i, 1, 0)?;
            i += 1;
            setbitu(buffer, i, 24, cast_u32(pr1))?;
            i += 24;
            setbits(buffer, i, 20, cast_i32(ppr1))?;
            i += 20;
            setbitu(buffer, i, 7, ob.raw.lock1 as u32)?;
            i += 7;
            setbitu(buffer, i, 8, amb as u8 as u32)?;
            i += 8;
            setbitu(buffer, i, 8, (ob.snr as i32 * 4) as u8 as u32)?;
            i += 8;
        }

        self.nbits = i;

        Ok(i)
    }
}

/// `public class type1004`: GPS L1/L2 observations.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct type1004 {
    pub nbits: u32,
    pub obs: Vec<ob>,
}

impl type1004 {
    pub fn Read(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let (tow, nsat, _) = read_legacy_header(buffer, 30)?;

        let mut i = 24 + 64;

        let week = week_guess();

        let _gpstime = StaticUtils::GetFromGps(week, tow)?;

        set_lasttow(tow);

        for _a in 0..nsat {
            let mut ob = ob { sys: 'G', tow, week, ..Default::default() };

            ob.raw.prn = getbitu(buffer, i, 6)? as u8;
            i += 6;
            ob.raw.code1 = getbitu(buffer, i, 1)? as u8;
            i += 1;
            ob.raw.pr1 = getbitu(buffer, i, 24)?;
            i += 24;
            ob.raw.ppr1 = getbits(buffer, i, 20)?;
            i += 20;
            ob.raw.lock1 = getbitu(buffer, i, 7)? as u8;
            i += 7;
            ob.raw.amb = getbitu(buffer, i, 8)? as u8;
            i += 8;
            ob.raw.cnr1 = getbitu(buffer, i, 8)? as u8;
            i += 8;
            ob.raw.code2 = getbitu(buffer, i, 2)? as u8;
            i += 2;
            ob.raw.pr21 = getbits(buffer, i, 14)?;
            i += 14;
            ob.raw.ppr2 = getbits(buffer, i, 20)?;
            i += 20;
            ob.raw.lock2 = getbitu(buffer, i, 7)? as u8;
            i += 7;
            ob.raw.cnr2 = getbitu(buffer, i, 8)? as u8;
            i += 8;

            let pr1 = ob.raw.pr1 as f64 * 0.02 + ob.raw.amb as f64 * PRUNIT_GPS;

            let lam1 = CLIGHT / FREQ1;
            let lam2 = CLIGHT / FREQ2;

            let cp1 = ob.raw.ppr1 as f64 * 0.0005 / lam1;

            if ob.raw.ppr1 as u32 != 0xFFF80000 {
                ob.prn = ob.raw.prn;
                ob.cp = pr1 / lam1 + cp1;
                ob.pr = pr1;
                ob.snr = cast_u8(ob.raw.cnr1 as f64 * 0.25); // *4.0+0.5

                ob.pr2 = pr1 + ob.raw.pr21 as f64 * 0.02;
                ob.cp2 = pr1 / lam2 + ob.raw.ppr2 as f64 * 0.0005 / lam2;

                self.obs.push(ob);
            }
        }

        sort_by_prn(&mut self.obs);

        self.nbits = i;
        Ok(())
    }

    pub fn Write(&mut self, buffer: &mut [u8]) -> Result<u32, Exception> {
        let mut i: u32 = 24 + 64;

        for ob in &self.obs {
            let lam1 = CLIGHT / FREQ1;

            let amb = cast_i32((ob.pr / PRUNIT_GPS).floor());
            let pr1 = round((ob.pr - amb as f64 * PRUNIT_GPS) / 0.02);
            let pr1c = pr1 * 0.02 + amb as f64 * PRUNIT_GPS;

            let ppr = cp_pr(ob.cp, pr1c / lam1);
            let ppr1 = round(ppr * lam1 / 0.0005);

            setbitu(buffer, i, 6, ob.prn as u32)?;
            i += 6;
            setbitu(buffer, i, 1, 0)?;
            i += 1;
            setbitu(buffer, i, 24, cast_u32(pr1))?;
            i += 24;
            setbits(buffer, i, 20, cast_i32(ppr1))?;
            i += 20;
            setbitu(buffer, i, 7, ob.raw.lock1 as u32)?;
            i += 7;
            setbitu(buffer, i, 8, amb as u8 as u32)?;
            i += 8;
            setbitu(buffer, i, 8, (ob.snr as i32 * 4) as u8 as u32)?;
            i += 8;
            // l2 - all 0's
            setbitu(buffer, i, 2, ob.raw.code2 as u32)?;
            i += 2;
            setbits(buffer, i, 14, ob.raw.pr21)?;
            i += 14;
            setbits(buffer, i, 20, ob.raw.ppr2)?;
            i += 20;
            setbitu(buffer, i, 7, ob.raw.lock2 as u32)?;
            i += 7;
            setbitu(buffer, i, 8, ob.raw.cnr2 as u32)?;
            i += 8;
        }

        self.nbits = i;

        Ok(i)
    }
}

/// `public class type1012`: GLONASS L1/L2 observations. Its `Read` prints the epoch and
/// each satellite to the console, as the C# does.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct type1012 {
    pub nbits: u32,
    pub obs: Vec<ob>,
}

impl type1012 {
    pub fn Read(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let mut i = 24;

        let _type = getbitu(buffer, i, 12)?;
        i += 12;

        let _staid = getbitu(buffer, i, 12)?;
        i += 12;
        let tow = getbitu(buffer, i, 27)? as f64 * 0.001;
        i += 27;
        let _sync = getbitu(buffer, i, 1)?;
        i += 1;
        let nsat = getbitu(buffer, i, 5)?;
        i += 5;
        let _smoothind = getbitu(buffer, i, 1)?;
        i += 1;
        let _smoothint = getbitu(buffer, i, 3)?;
        i += 3;

        let week = week_guess();

        let mut gpstime = StaticUtils::GetFromGps(week, tow - 10800.0 - (60.0 * 60.0 * 24.0))?;

        let (y, mo, _, _, _, _, _) = gpstime.parts();
        gpstime = gpstime.add_seconds(StaticUtils::LeapSecondsGPS(y, mo as i32) as f64)?;

        set_lasttow(tow);

        let (y, mo, d, h, mi, s, ms) = gpstime.parts();
        console_write_line(&format!(
            "> {} {} {} {:>2} {} {} {} {}",
            y,
            mo,
            d,
            h,
            mi,
            double_to_string(s as f64 + ms as f64 / 1000.0),
            0,
            nsat
        ));

        let c = current_culture();
        for _a in 0..nsat {
            let mut ob = ob { sys: 'R', tow, week, ..Default::default() };

            ob.raw.prn = getbitu(buffer, i, 6)? as u8;
            i += 6;
            ob.raw.code1 = getbitu(buffer, i, 1)? as u8;
            i += 1;
            ob.raw.fcn = getbitu(buffer, i, 5)? as u8;
            i += 5;
            ob.raw.pr1 = getbitu(buffer, i, 25)?;
            i += 25;
            ob.raw.ppr1 = getbits(buffer, i, 20)?;
            i += 20;
            ob.raw.lock1 = getbitu(buffer, i, 7)? as u8;
            i += 7;
            ob.raw.amb = getbitu(buffer, i, 7)? as u8;
            i += 7;
            ob.raw.cnr1 = getbitu(buffer, i, 8)? as u8;
            i += 8;
            ob.raw.code2 = getbitu(buffer, i, 2)? as u8;
            i += 2;
            ob.raw.pr21 = getbits(buffer, i, 14)?;
            i += 14;
            ob.raw.ppr2 = getbits(buffer, i, 20)?;
            i += 20;
            ob.raw.lock2 = getbitu(buffer, i, 7)? as u8;
            i += 7;
            ob.raw.cnr2 = getbitu(buffer, i, 8)? as u8;
            i += 8;

            let pr1 = ob.raw.pr1 as f64 * 0.02 + ob.raw.amb as f64 * PRUNIT_GLO;

            let lam1 = CLIGHT / FREQ1;
            let lam2 = CLIGHT / FREQ2;

            let cp1 = ob.raw.ppr1 as f64 * 0.0005 / lam1;

            if ob.raw.ppr1 as u32 != 0xFFF80000 {
                ob.prn = ob.raw.prn;
                ob.cp = pr1 / lam1 + cp1;
                ob.pr = pr1;
                ob.snr = cast_u8(ob.raw.cnr1 as f64 * 0.25); // *4.0+0.5

                ob.pr2 = pr1 + ob.raw.pr21 as f64 * 0.02;
                ob.cp2 = pr1 / lam2 + ob.raw.ppr2 as f64 * 0.0005 / lam2;

                self.obs.push(ob);

                console_write_line(&format!(
                    "R{:>2} {:>13} {:>15}0{:>15} {:>15}0{:>15}",
                    ob.prn,
                    format_fixed(ob.pr, 1, 3, c),
                    format_fixed(ob.cp, 1, 3, c),
                    format_fixed(ob.snr as f64, 1, 3, c),
                    format_fixed(ob.pr2, 1, 3, c),
                    format_fixed(ob.cp2, 1, 3, c)
                ));
            }
        }

        sort_by_prn(&mut self.obs);

        self.nbits = i;
        Ok(())
    }

    /// `Write(buffer)`: computes the fields but writes nothing (unfinished in C#).
    pub fn Write(&mut self, _buffer: &mut [u8]) -> Result<u32, Exception> {
        let i: u32 = 24 + 64;

        for ob in &self.obs {
            let lam1 = CLIGHT / FREQ1;

            let amb = cast_i32((ob.pr / PRUNIT_GPS).floor());
            let pr1 = round((ob.pr - amb as f64 * PRUNIT_GPS) / 0.02);
            let pr1c = pr1 * 0.02 + amb as f64 * PRUNIT_GPS;

            let ppr = cp_pr(ob.cp, pr1c / lam1);
            let _ppr1 = round(ppr * lam1 / 0.0005);
        }

        Ok(i)
    }
}

/// The MSM4 (`msm7 == false`) or MSM7 decoder shared by type1074/1077 and their GNSS
/// subclasses; `sys` is the system letter the subclass chain finally assigns.
fn read_msm(buffer: &[u8], msm7: bool, sys: char) -> Result<(Vec<ob>, u32), Exception> {
    let mut i = 24;

    let _type = getbitu(buffer, i, 12)?;
    i += 12;

    let _staid = getbitu(buffer, i, 12)?;
    i += 12;
    let tow = getbitu(buffer, i, 30)? as f64 * 0.001;
    i += 30;
    let _sync = getbitu(buffer, i, 1)?;
    i += 1;
    let _iod = getbitu(buffer, i, 3)?;
    i += 3;

    let _time_s = getbitu(buffer, i, 7)?;
    i += 7;
    let _clk_str = getbitu(buffer, i, 2)?;
    i += 2;
    let _clk_ext = getbitu(buffer, i, 2)?;
    i += 2;
    let _smooth = getbitu(buffer, i, 1)?;
    i += 1;
    let _tint_s = getbitu(buffer, i, 3)?;
    i += 3;

    let mut nsat = 0usize;
    let mut nsig = 0usize;
    let mut ncell = 0usize;

    // Dictionary<int, double> keyed 0..n-1, filled in order
    let mut sats: Vec<f64> = vec![];
    let mut sigs: Vec<f64> = vec![];
    let mut cellmask = [0u8; 64];

    for j in 1..=64 {
        let mask = getbitu(buffer, i, 1)?;
        i += 1;
        if mask > 0 {
            sats.push(j as f64);
            nsat += 1;
        }
    }
    for j in 1..=32 {
        let mask = getbitu(buffer, i, 1)?;
        i += 1;
        if mask > 0 {
            sigs.push(j as f64);
            nsig += 1;
        }
    }

    for j in 0..nsat * nsig {
        let v = getbitu(buffer, i, 1)? as u8;
        *cellmask.get_mut(j).ok_or_else(index_out_of_range)? = v;
        i += 1;
        if cellmask[j] > 0 {
            ncell += 1;
        }
    }

    // end of header  i=202

    let week = week_guess();

    let _gpstime = StaticUtils::GetFromGps(week, tow)?;

    set_lasttow(tow);

    let mut r = [0f64; 64];
    let mut rr = [0f64; 64];
    let mut pr = [0f64; 64];
    let mut cp = [0f64; 64];
    let mut rrf = [0f64; 64];
    let mut cnr = [0f64; 64];
    let mut ex = [0u32; 64];
    let mut half = [0u32; 64];
    let mut lock = [0u32; 64];

    for j in 0..nsat {
        r[j] = 0.0;
        rr[j] = 0.0;
        ex[j] = 15;
    }
    for j in 0..ncell {
        rrf[j] = -1E16;
        cp[j] = -1E16;
        pr[j] = -1E16;
    }

    /* decode satellite data */
    for j in 0..nsat {
        /* range */
        let rng = getbitu(buffer, i, 8)?;
        i += 8;
        if rng != 255 {
            r[j] = rng as f64 * RANGE_MS;
        }
    }
    if msm7 {
        for j in 0..nsat {
            /* extended info */
            ex[j] = getbitu(buffer, i, 4)?;
            i += 4;
        }
    }
    for j in 0..nsat {
        let rng_m = getbitu(buffer, i, 10)?;
        i += 10;
        if r[j] != 0.0 {
            r[j] += rng_m as f64 * P2_10 * RANGE_MS;
        }
    }
    if msm7 {
        for j in 0..nsat {
            /* phaserangerate */
            let rate = getbits(buffer, i, 14)?;
            i += 14;
            if rate != -8192 {
                rr[j] = rate as f64 * 1.0;
            }
        }
    }
    /* decode signal data */
    for j in 0..ncell {
        /* pseudorange */
        if msm7 {
            let prv = getbits(buffer, i, 20)?;
            i += 20;
            if prv != -524288 {
                pr[j] = prv as f64 * P2_29 * RANGE_MS;
            }
        } else {
            let prv = getbits(buffer, i, 15)?;
            i += 15;
            if prv != -16384 {
                pr[j] = prv as f64 * P2_24 * RANGE_MS;
            }
        }
    }
    for j in 0..ncell {
        /* phaserange */
        if msm7 {
            let cpv = getbits(buffer, i, 24)?;
            i += 24;
            if cpv != -8388608 {
                cp[j] = cpv as f64 * P2_31 * RANGE_MS;
            }
        } else {
            let cpv = getbits(buffer, i, 22)?;
            i += 22;
            if cpv != -2097152 {
                cp[j] = cpv as f64 * P2_29 * RANGE_MS;
            }
        }
    }
    for j in 0..ncell {
        /* lock time */
        let bits = if msm7 { 10 } else { 4 };
        lock[j] = getbitu(buffer, i, bits)?;
        i += bits;
    }
    for j in 0..ncell {
        /* half-cycle amiguity */
        half[j] = getbitu(buffer, i, 1)?;
        i += 1;
    }
    for j in 0..ncell {
        /* cnr */
        if msm7 {
            cnr[j] = getbitu(buffer, i, 10)? as f64 * 0.0625;
            i += 10;
        } else {
            cnr[j] = getbitu(buffer, i, 6)? as f64 * 1.0; // * 0.0625;
            i += 6;
        }
    }
    if msm7 {
        for j in 0..ncell {
            /* phaserangerate */
            let rrv = getbits(buffer, i, 15)?;
            i += 15;
            if rrv != -16384 {
                rrf[j] = rrv as f64 * 0.0001;
            }
        }
    }

    let mut obs: Vec<ob> = vec![];
    let mut sig = 0usize;
    let mut cellmaskused = 0usize;

    for j in 0..nsat {
        let mut ob = ob { sys: 'G', tow, week, ..Default::default() };

        ob.prn = cast_u8(sats[j]);

        for k in 0..nsig {
            if cellmask[sig] == 1 {
                let s = sigs[k];
                let l1 = if msm7 { s >= 1.0 && s <= 13.0 } else { s >= 1.0 && s <= 13.0 && s != 10.0 };
                let l2 = if msm7 { s >= 14.0 && s <= 23.0 } else { (s >= 14.0 && s <= 23.0) || s == 10.0 };
                if l1 {
                    ob.pr = r[j] + pr[cellmaskused];
                    ob.cp = r[j] + cp[cellmaskused];
                    ob.snr = cast_u8(cnr[cellmaskused]);
                } else if l2 {
                    ob.pr2 = r[j] + pr[cellmaskused];
                    ob.cp2 = r[j] + cp[cellmaskused];
                } else if s >= 24.0 && s <= 26.0 {
                    //ob.pr5 = r[j] + pr[sig];
                    //ob.cp5 = (r[j] + cp[sig]);
                }

                cellmaskused += 1;
            }

            sig += 1;
        }

        obs.push(ob);
    }

    sort_by_prn(&mut obs);

    // the subclass Read overrides then rewrite sys on every observation
    if sys != 'G' {
        for ob in obs.iter_mut() {
            ob.sys = sys;
        }
    }

    Ok((obs, i))
}

macro_rules! msm_type {
    ($name:ident, $msm7:expr, $sys:expr, $doc:literal) => {
        #[doc = $doc]
        #[derive(Debug, Clone, Default, PartialEq)]
        pub struct $name {
            pub nbits: u32,
            pub obs: Vec<ob>,
        }

        impl $name {
            pub fn Read(&mut self, buffer: &[u8]) -> Result<(), Exception> {
                let (obs, nbits) = read_msm(buffer, $msm7, $sys)?;
                self.obs.extend(obs);
                self.nbits = nbits;
                Ok(())
            }

            /// `Write(buffer)`: not implemented in C# (returns 0).
            pub fn Write(&mut self, _buffer: &mut [u8]) -> Result<u32, Exception> {
                Ok(0)
            }
        }
    };
}

msm_type!(type1074, false, 'G', "`public class type1074`: GPS MSM4.");
msm_type!(type1077, true, 'G', "`public class type1077`: GPS MSM7.");
msm_type!(type1084, false, 'R', "`public class type1084 : type1074`: GLONASS MSM4.");
msm_type!(type1087, true, 'R', "`public class type1087 : type1077`: GLONASS MSM7.");
msm_type!(type1094, false, 'E', "`public class type1094 : type1084`: Galileo MSM4.");
msm_type!(type1097, true, 'E', "`public class type1097 : type1087`: Galileo MSM7.");
msm_type!(type1114, false, 'Q', "`public class type1114 : type1084`: QZSS MSM4.");
msm_type!(type1117, true, 'Q', "`public class type1117 : type1087`: QZSS MSM7.");
msm_type!(type1124, false, 'B', "`public class type1124 : type1084`: BeiDou MSM4.");
msm_type!(type1127, true, 'B', "`public class type1127 : type1087`: BeiDou MSM7.");

/// The fields and accessors type1005 and type1006 share.
macro_rules! base_pos_common {
    () => {
        /// `ecefposition` getter
        pub fn ecefposition(&self) -> [f64; 3] {
            [self.rr0 * 0.0001, self.rr1 * 0.0001, self.rr2 * 0.0001]
        }

        /// `ecefposition` setter
        pub fn set_ecefposition(&mut self, value: &[f64]) -> Result<(), Exception> {
            if value.len() < 3 {
                return Err(index_out_of_range());
            }
            self.rr0 = value[0] / 0.0001;
            self.rr1 = value[1] / 0.0001;
            self.rr2 = value[2] / 0.0001;
            Ok(())
        }

        fn read_position(&mut self, buffer: &[u8]) -> Result<u32, Exception> {
            let mut i = 24 + 12;

            self.staid = getbitu(buffer, i, 12)? as u16;
            i += 12;
            self.itrf = getbitu(buffer, i, 6)? as u8;
            i += 6 + 4;
            self.rr0 = getbits_38(buffer, i)?;
            i += 38 + 2;
            self.rr1 = getbits_38(buffer, i)?;
            i += 38 + 2;
            self.rr2 = getbits_38(buffer, i)?;
            i += 38;
            Ok(i)
        }

        fn write_position(&self, buffer: &mut [u8]) -> Result<u32, Exception> {
            let mut i = 24;

            setbitu(buffer, i, 12, 1005)?;
            i += 12; /* message no */
            setbitu(buffer, i, 12, self.staid as u32)?;
            i += 12; /* ref station id */
            setbitu(buffer, i, 6, 0)?;
            i += 6; /* itrf realization year */
            setbitu(buffer, i, 1, 1)?;
            i += 1; /* gps indicator */
            setbitu(buffer, i, 1, 1)?;
            i += 1; /* glonass indicator */
            setbitu(buffer, i, 1, 0)?;
            i += 1; /* galileo indicator */
            setbitu(buffer, i, 1, 0)?;
            i += 1; /* ref station indicator */
            set38bits(buffer, i, self.ecefposition()[0] / 0.0001)?;
            i += 38; /* antenna ref point ecef-x */
            setbitu(buffer, i, 1, 1)?;
            i += 1; /* oscillator indicator */
            setbitu(buffer, i, 1, 0)?;
            i += 1; /* reserved */
            set38bits(buffer, i, self.ecefposition()[1] / 0.0001)?;
            i += 38; /* antenna ref point ecef-y */
            setbitu(buffer, i, 2, 0)?;
            i += 2; /* quarter cycle indicator */
            set38bits(buffer, i, self.ecefposition()[2] / 0.0001)?;
            i += 38; /* antenna ref point ecef-z */
            Ok(i)
        }
    };
}

/// `public class type1005`: base station position.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct type1005 {
    pub galileoind: u8,
    pub glonassind: u8,
    pub gpsind: u8,
    pub itrf: u8,
    pub oscind: u8,
    pub quatcycind: u8,
    pub refstatind: u8,
    pub resv: u8,
    pub rr0: f64,
    pub rr1: f64,
    pub rr2: f64,
    pub staid: u16,
}

impl Default for type1005 {
    fn default() -> Self {
        type1005 {
            galileoind: 0,
            glonassind: 0,
            gpsind: 1,
            itrf: 0,
            oscind: 1,
            quatcycind: 0,
            refstatind: 0,
            resv: 0,
            rr0: 0.0,
            rr1: 0.0,
            rr2: 0.0,
            staid: 1,
        }
    }
}

impl type1005 {
    base_pos_common!();

    pub fn Read(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        self.read_position(buffer)?;
        Ok(())
    }

    pub fn Write(&self, buffer: &mut [u8]) -> Result<u32, Exception> {
        self.write_position(buffer)
    }
}

/// `public class type1006`: base station position with antenna height. `Write` labels the
/// message 1005, as the C# does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct type1006 {
    pub anth: u16,
    pub galileoind: u8,
    pub glonassind: u8,
    pub gpsind: u8,
    pub itrf: u8,
    pub oscind: u8,
    pub quatcycind: u8,
    pub refstatind: u8,
    pub resv: u8,
    pub rr0: f64,
    pub rr1: f64,
    pub rr2: f64,
    pub staid: u16,
}

impl Default for type1006 {
    fn default() -> Self {
        type1006 {
            anth: 0,
            galileoind: 0,
            glonassind: 0,
            gpsind: 1,
            itrf: 0,
            oscind: 1,
            quatcycind: 0,
            refstatind: 0,
            resv: 0,
            rr0: 0.0,
            rr1: 0.0,
            rr2: 0.0,
            staid: 1,
        }
    }
}

impl type1006 {
    base_pos_common!();

    pub fn Read(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let i = self.read_position(buffer)?;
        self.anth = getbitu(buffer, i, 16)? as u16;
        Ok(())
    }

    pub fn Write(&self, buffer: &mut [u8]) -> Result<u32, Exception> {
        let mut i = self.write_position(buffer)?;
        setbitu(buffer, i, 16, self.anth as u32)?;
        i += 16; /* antenna height */
        Ok(i)
    }
}

/// `type1019.gps_time_t`
#[derive(Debug, Clone, Copy, Default)]
pub struct gps_time_t {
    /// Seconds since the GPS start of week.
    pub tow: f64,
    /// GPS week number.
    pub wn: u16,
}

impl gps_time_t {
    pub fn new(tow: f64, wn: u16) -> Self {
        gps_time_t { tow, wn }
    }

    /// `Equals(gps_time_t)`: `tow.Equals` (NaN equals NaN) and the week.
    pub fn equals(&self, other: &gps_time_t) -> bool {
        (self.tow == other.tow || (self.tow.is_nan() && other.tow.is_nan())) && self.wn == other.wn
    }

    /// `GetHashCode()`
    pub fn get_hash_code(&self) -> i32 {
        double_get_hash_code(self.tow).wrapping_mul(397) ^ self.wn as i32
    }
}

impl PartialEq for gps_time_t {
    /// `operator ==`: `tow == tow` (NaN never equal) and the week.
    fn eq(&self, ob2: &gps_time_t) -> bool {
        self.wn == ob2.wn && self.tow == ob2.tow
    }
}

impl std::fmt::Display for gps_time_t {
    /// `ToString()`: "wn:tow"
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}:{}", self.wn, double_to_string(self.tow))
    }
}

/// `public class type1019`: GPS ephemeris.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct type1019 {
    pub A: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub cic: f64,
    pub cis: f64,
    pub code: f64,
    pub crc: f64,
    pub crs: f64,
    pub cuc: f64,
    pub cus: f64,
    pub deln: f64,
    pub e: f64,
    pub fit: bool,
    pub flag: u32,
    pub i0: f64,
    pub idot: f64,
    pub iodc: u32,
    pub iode: u32,
    pub M0: f64,
    pub omg: f64,
    pub OMG0: f64,
    pub OMGd: f64,
    pub prn: f64,
    pub sqrtA: f64,
    pub sva: f64,
    pub svh: u32,
    pub tgd: f64,
    pub toc: f64,
    pub toes: f64,
    pub week: f64,

    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub clock_err: f64,
    pub clock_rate_err: f64,
    pub tot: gps_time_t,
}

impl type1019 {
    pub fn Read(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let mut i = 24 + 12;

        self.prn = getbitu(buffer, i, 6)? as f64;
        i += 6;
        self.week = getbitu(buffer, i, 10)? as f64;
        i += 10;
        self.sva = getbitu(buffer, i, 4)? as f64;
        i += 4;
        self.code = getbitu(buffer, i, 2)? as f64;
        i += 2;
        self.idot = getbits(buffer, i, 14)? as f64 * P2_43 * SC2RAD;
        i += 14;
        self.iode = getbitu(buffer, i, 8)?;
        i += 8;
        self.toc = getbitu(buffer, i, 16)? as f64 * 16.0;
        i += 16;
        self.af2 = getbits(buffer, i, 8)? as f64 * P2_55;
        i += 8;
        self.af1 = getbits(buffer, i, 16)? as f64 * P2_43;
        i += 16;
        self.af0 = getbits(buffer, i, 22)? as f64 * P2_31;
        i += 22;
        self.iodc = getbitu(buffer, i, 10)?;
        i += 10;
        self.crs = getbits(buffer, i, 16)? as f64 * P2_5;
        i += 16;
        self.deln = getbits(buffer, i, 16)? as f64 * P2_43 * SC2RAD;
        i += 16;
        self.M0 = getbits(buffer, i, 32)? as f64 * P2_31 * SC2RAD;
        i += 32;
        self.cuc = getbits(buffer, i, 16)? as f64 * P2_29;
        i += 16;
        self.e = getbitu(buffer, i, 32)? as f64 * P2_33;
        i += 32;
        self.cus = getbits(buffer, i, 16)? as f64 * P2_29;
        i += 16;
        self.sqrtA = getbitu(buffer, i, 32)? as f64 * P2_19;
        i += 32;
        self.toes = getbitu(buffer, i, 16)? as f64 * 16.0;
        i += 16;
        self.cic = getbits(buffer, i, 16)? as f64 * P2_29;
        i += 16;
        self.OMG0 = getbits(buffer, i, 32)? as f64 * P2_31 * SC2RAD;
        i += 32;
        self.cis = getbits(buffer, i, 16)? as f64 * P2_29;
        i += 16;
        self.i0 = getbits(buffer, i, 32)? as f64 * P2_31 * SC2RAD;
        i += 32;
        self.crc = getbits(buffer, i, 16)? as f64 * P2_5;
        i += 16;
        self.omg = getbits(buffer, i, 32)? as f64 * P2_31 * SC2RAD;
        i += 32;
        self.OMGd = getbits(buffer, i, 24)? as f64 * P2_43 * SC2RAD;
        i += 24;
        self.tgd = getbits(buffer, i, 8)? as f64 * P2_31;
        i += 8;
        self.svh = getbitu(buffer, i, 6)?;
        i += 6;
        self.flag = getbitu(buffer, i, 1)?;
        i += 1;
        self.fit = getbitu(buffer, i, 1)? > 0; /* 0:4hr,1:>4hr */

        self.A = self.sqrtA * self.sqrtA;
        Ok(())
    }

    pub fn Write(&self, buffer: &mut [u8]) -> Result<u32, Exception> {
        let mut i = 24;

        let toe = cast_u32(self.toes / 16.0);

        setbitu(buffer, i, 12, 1019)?;
        i += 12;
        setbitu_d(buffer, i, 6, self.prn)?;
        i += 6;
        setbitu_d(buffer, i, 10, self.week % 1024.0)?;
        i += 10;
        setbitu_d(buffer, i, 4, self.sva)?;
        i += 4;
        setbitu_d(buffer, i, 2, self.code)?;
        i += 2;
        setbits_d(buffer, i, 14, self.idot / P2_43 / SC2RAD)?;
        i += 14;
        setbitu(buffer, i, 8, self.iode)?;
        i += 8;
        setbitu_d(buffer, i, 16, self.toc / 16.0)?;
        i += 16;
        setbits_d(buffer, i, 8, self.af2 / P2_55)?;
        i += 8;
        setbits_d(buffer, i, 16, self.af1 / P2_43)?;
        i += 16;
        setbits_d(buffer, i, 22, self.af0 / P2_31)?;
        i += 22;
        setbitu(buffer, i, 10, self.iodc)?;
        i += 10;
        setbits_d(buffer, i, 16, self.crs / P2_5)?;
        i += 16;
        setbits_d(buffer, i, 16, self.deln / P2_43 / SC2RAD)?;
        i += 16;
        setbits_d(buffer, i, 32, self.M0 / P2_31 / SC2RAD)?;
        i += 32;
        setbits_d(buffer, i, 16, self.cuc / P2_29)?;
        i += 16;
        setbitu_d(buffer, i, 32, self.e / P2_33)?;
        i += 32;
        setbits_d(buffer, i, 16, self.cus / P2_29)?;
        i += 16;
        setbitu_d(buffer, i, 32, self.sqrtA / P2_19)?;
        i += 32;
        setbitu(buffer, i, 16, toe)?;
        i += 16;
        setbits_d(buffer, i, 16, self.cic / P2_29)?;
        i += 16;
        setbits_d(buffer, i, 32, self.OMG0 / P2_31 / SC2RAD)?;
        i += 32;
        setbits_d(buffer, i, 16, self.cis / P2_29)?;
        i += 16;
        setbits_d(buffer, i, 32, self.i0 / P2_31 / SC2RAD)?;
        i += 32;
        setbits_d(buffer, i, 16, self.crc / P2_5)?;
        i += 16;
        setbits_d(buffer, i, 32, self.omg / P2_31 / SC2RAD)?;
        i += 32;
        setbits_d(buffer, i, 24, self.OMGd / P2_43 / SC2RAD)?;
        i += 24;
        setbits_d(buffer, i, 8, self.tgd / P2_31)?;
        i += 8;
        setbitu(buffer, i, 6, self.svh)?;
        i += 6;
        setbitu(buffer, i, 1, self.flag)?;
        i += 1;
        // `fit ? 1 : 0` is an int expression, so the double overload is chosen
        setbitu_d(buffer, i, 1, if self.fit { 1.0 } else { 0.0 })?;
        i += 1;

        Ok(i)
    }

    /// `gpsdifftime(end, beginning)`: the week difference times 604800 is int arithmetic.
    fn gpsdifftime(end: gps_time_t, beginning: gps_time_t) -> f64 {
        ((end.wn as i32 - beginning.wn as i32).wrapping_mul(7 * 24 * 3600)) as f64 + end.tow - beginning.tow
    }

    /// `calc_sat_pos(pos, vel, ref clock_err, ref clock_rate_err, tot)`: satellite position
    /// and velocity (ECEF) and clock terms at `tot` (IS-GPS-200D Table 20-IV). Returns -1 when
    /// the ephemeris is more than 4 hours from `tot`; then `clock_err`/`clock_rate_err` are
    /// already updated, as in C#.
    pub fn calc_sat_pos(
        &mut self,
        pos: &mut [f64; 3],
        vel: &mut [f64; 3],
        clock_err: &mut f64,
        clock_rate_err: &mut f64,
        tot: gps_time_t,
    ) -> i32 {
        if tot == self.tot {
            *pos = self.pos;
            *vel = self.vel;
            *clock_err = self.clock_err;
            *clock_rate_err = self.clock_rate_err;

            return 0;
        }

        const NAV_OMEGAE_DOT: f64 = 7.2921151467e-005;
        const NAV_GM: f64 = 3.986005e14;

        let toc = gps_time_t::new(self.toc, cast_u16(self.week));
        let toe = gps_time_t::new(self.toes, cast_u16(self.week));

        // Satellite clock terms
        // Seconds from clock data reference time (toc)
        let mut tdiff = Self::gpsdifftime(tot, toc);

        if tdiff > (4 * 3600) as f64 {
            tdiff = Self::gpsdifftime(tot, toe);
        }

        *clock_err = self.af0 + tdiff * (self.af1 + tdiff * self.af2) - self.tgd;
        *clock_rate_err = self.af1 + 2.0 * tdiff * self.af2;

        // Seconds from the time from ephemeris reference epoch (toe)
        tdiff = Self::gpsdifftime(tot, toe);

        // If tdiff is too large our ephemeris isn't valid, maybe we want to wait until we get a
        // new one? At least let's warn the user.
        if tdiff.abs() > (4 * 3600) as f64 {
            console_write(&format!(
                " WARNING: using ephemeris older (or newer!) than 4 hours. {} {} hrs       \r",
                double_to_string(self.prn),
                double_to_string(tdiff / 3600.0)
            ));
            return -1;
        }

        // Calculate position per IS-GPS-200D p 97 Table 20-IV
        let a = self.sqrtA * self.sqrtA; // [m] Semi-major axis
        let ma_dot = (NAV_GM / (a * a * a)).sqrt() + self.deln; // [rad/sec] Corrected mean motion
        let ma = self.M0 + ma_dot * tdiff; // [rad] Corrected mean anomaly

        // Iteratively solve for the Eccentric Anomaly (from Keith Alter and David Johnston)
        let mut ea = ma; // Starting value for E
        let ecc = self.e;
        let mut count: u32 = 0;
        let mut tempd1;

        loop {
            let ea_old = ea;
            tempd1 = 1.0 - ecc * ea_old.cos();
            ea += (ma - ea_old + ecc * ea_old.sin()) / tempd1;
            count += 1;
            if count > 5 {
                break;
            }
            if (ea - ea_old).abs() <= 1.0E-14 {
                break;
            }
        }
        let ea_dot = ma_dot / tempd1;

        // Relativistic correction term
        let einstein = -4.442807633E-10 * ecc * self.sqrtA * ea.sin();

        // Begin calc for True Anomaly and Argument of Latitude
        let tempd2 = (1.0 - ecc * ecc).sqrt();
        // [rad] Argument of Latitude = True Anomaly + Argument of Perigee
        let al = (tempd2 * ea.sin()).atan2(ea.cos() - ecc) + self.omg;
        let al_dot = tempd2 * ea_dot / tempd1;

        // Calculate corrected argument of latitude based on position
        let cal = al + self.cus * (2.0 * al).sin() + self.cuc * (2.0 * al).cos();
        let cal_dot = al_dot * (1.0 + 2.0 * (self.cus * (2.0 * al).cos() - self.cuc * (2.0 * al).sin()));

        // Calculate corrected radius based on argument of latitude
        let r = a * tempd1 + self.crc * (2.0 * al).cos() + self.crs * (2.0 * al).sin();
        let r_dot = a * ecc * ea.sin() * ea_dot + 2.0 * al_dot * (self.crs * (2.0 * al).cos() - self.crc * (2.0 * al).sin());

        // Calculate inclination based on argument of latitude
        let inc = self.i0 + self.idot * tdiff + self.cic * (2.0 * al).cos() + self.cis * (2.0 * al).sin();
        let inc_dot = self.idot + 2.0 * al_dot * (self.cis * (2.0 * al).cos() - self.cic * (2.0 * al).sin());

        // Calculate position and velocity in orbital plane
        let x = r * cal.cos();
        let y = r * cal.sin();
        let x_dot = r_dot * cal.cos() - y * cal_dot;
        let y_dot = r_dot * cal.sin() + x * cal_dot;

        // Corrected longitude of ascenting node
        let om_dot = self.OMGd - NAV_OMEGAE_DOT;
        let om = self.OMG0 + tdiff * om_dot - NAV_OMEGAE_DOT * toe.tow;

        // Compute the satellite's position in Earth-Centered Earth-Fixed coordiates
        pos[0] = x * om.cos() - y * inc.cos() * om.sin();
        pos[1] = x * om.sin() + y * inc.cos() * om.cos();
        pos[2] = y * inc.sin();

        let tempd3 = y_dot * inc.cos() - y * inc.sin() * inc_dot;

        // Compute the satellite's velocity in Earth-Centered Earth-Fixed coordiates
        vel[0] = -om_dot * pos[1] + x_dot * om.cos() - tempd3 * om.sin();
        vel[1] = om_dot * pos[0] + x_dot * om.sin() + tempd3 * om.cos();
        vel[2] = y * inc.cos() * inc_dot + y_dot * inc.sin();

        *clock_err += einstein;

        self.pos = *pos;
        self.vel = *vel;
        self.clock_err = *clock_err;
        self.clock_rate_err = *clock_rate_err;
        self.tot = tot;

        0
    }
}

/// The `BasePosMessage` sender: a type1005 or type1006.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BasePos {
    Type1005(type1005),
    Type1006(type1006),
}

pub type ObsHandler = Arc<dyn Fn(&[ob]) + Send + Sync>;
pub type BasePosHandler = Arc<dyn Fn(&BasePos) + Send + Sync>;
pub type EphHandler = Arc<dyn Fn(&type1019) + Send + Sync>;

/// `public class rtcm3 : ICorrections`
pub struct rtcm3 {
    pub sync: u32,
    msglencount: i32,
    payloadlen: u32,
    pre: rtcmpreamble,
    pub step: i32,
    packet: Vec<u8>,
    /// `event EventHandler ObsMessage` (sender: the observation list)
    pub obs_message: RwLock<Vec<ObsHandler>>,
    /// `event EventHandler BasePosMessage` (sender: the type1005/type1006)
    pub base_pos_message: RwLock<Vec<BasePosHandler>>,
    /// `event EventHandler EphMessage` (sender: the type1019)
    pub eph_message: RwLock<Vec<EphHandler>>,
}

impl Default for rtcm3 {
    fn default() -> Self {
        rtcm3 {
            sync: 0,
            msglencount: 0,
            payloadlen: 0,
            pre: rtcmpreamble::default(),
            step: 0,
            packet: vec![0u8; 1024 * 4],
            obs_message: RwLock::new(vec![]),
            base_pos_message: RwLock::new(vec![]),
            eph_message: RwLock::new(vec![]),
        }
    }
}

impl rtcm3 {
    pub const D2R: f64 = PI / 180.0; /* deg to rad */
    pub const R2D: f64 = 180.0 / PI; /* rad to deg */

    pub fn new() -> Self {
        Self::default()
    }

    /// `rtcm3.weekGuess`
    pub fn week_guess() -> i32 {
        week_guess()
    }

    pub fn set_week_guess(v: i32) {
        WEEK_GUESS.store(v, Ordering::Relaxed);
    }

    fn raise_obs(&self, obs: &[ob]) {
        let hs = self.obs_message.read().unwrap_or_else(|e| e.into_inner()).clone();
        for h in hs {
            h(obs);
        }
    }

    fn raise_base(&self, b: BasePos) {
        let hs = self.base_pos_message.read().unwrap_or_else(|e| e.into_inner()).clone();
        for h in hs {
            h(&b);
        }
    }

    /// `Read(byte data)`: the message number once a frame with a valid CRC ends at this
    /// byte, otherwise -1. Decoding errors throw.
    pub fn Read(&mut self, data: u8) -> Result<i32, Exception> {
        let mut step = self.step;
        loop {
            match step {
                1 => {
                    self.packet[1] = data;
                    self.step += 1;
                }
                2 => {
                    self.packet[2] = data;
                    self.step += 1;
                    self.pre = rtcmpreamble::default();
                    self.pre.Read(&self.packet)?;
                    self.payloadlen = self.pre.length as u32;
                    self.msglencount = 0;
                    // reset on oversize packet
                    if self.payloadlen as usize > self.packet.len() {
                        self.step = 0;
                    }
                }
                3 => {
                    if (self.msglencount as u32) < self.payloadlen {
                        let idx = (self.msglencount + 3) as usize;
                        *self.packet.get_mut(idx).ok_or_else(index_out_of_range)? = data;
                        self.msglencount += 1;
                    } else {
                        self.step += 1;
                        step = 4;
                        continue; // goto case 4
                    }
                }
                4 => {
                    *self.packet.get_mut((self.payloadlen + 3) as usize).ok_or_else(index_out_of_range)? = data;
                    self.step += 1;
                }
                5 => {
                    *self.packet.get_mut((self.payloadlen + 3 + 1) as usize).ok_or_else(index_out_of_range)? = data;
                    self.step += 1;
                }
                6 => {
                    *self.packet.get_mut((self.payloadlen + 3 + 2) as usize).ok_or_else(index_out_of_range)? = data;

                    self.payloadlen += 3;
                    let crc = crc24::crc24q(&self.packet, self.payloadlen, 0)?;
                    let crcpacket = getbitu(&self.packet, self.payloadlen * 8, 24)?;

                    if crc == crcpacket {
                        let mut head = rtcmheader::default();
                        head.Read(&self.packet)?;

                        self.sync = head.sync as u32;

                        self.step = 0;

                        match head.messageno {
                            1002 => {
                                let mut tp = type1002::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1004 => {
                                let mut tp = type1004::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1012 => {
                                let mut tp = type1012::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1074 => {
                                let mut tp = type1074::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1077 => {
                                let mut tp = type1077::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1084 => {
                                let mut tp = type1084::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1087 => {
                                let mut tp = type1087::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1094 => {
                                let mut tp = type1094::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1097 => {
                                let mut tp = type1097::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1114 => {
                                let mut tp = type1114::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1117 => {
                                let mut tp = type1117::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1124 => {
                                let mut tp = type1124::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1127 => {
                                let mut tp = type1127::default();
                                tp.Read(&self.packet)?;
                                self.raise_obs(&tp.obs);
                            }
                            1005 => {
                                let mut tp = type1005::default();
                                tp.Read(&self.packet)?;
                                self.raise_base(BasePos::Type1005(tp));
                            }
                            1006 => {
                                let mut tp = type1006::default();
                                tp.Read(&self.packet)?;
                                self.raise_base(BasePos::Type1006(tp));
                            }
                            1019 => {
                                let mut tp = type1019::default();
                                tp.Read(&self.packet)?;

                                // assumes this is always a live stream
                                let (week, _tow) = StaticUtils::GetFromTime(DateTime::utc_now())?;

                                let lt = lasttow();
                                if tp.toes < (60 * 60 * 24) as f64 && lt < (60 * 60 * 24) as f64 {
                                    // both at start of a week
                                    rtcm3::set_week_guess(week);
                                } else if tp.toes > 500000.0 && lt < (60 * 60 * 24) as f64 {
                                    // eph at end of week and tow at start of week
                                    rtcm3::set_week_guess(week.wrapping_add(1));
                                } else {
                                    // everything else
                                    rtcm3::set_week_guess(week);
                                }

                                let hs = self.eph_message.read().unwrap_or_else(|e| e.into_inner()).clone();
                                for h in hs {
                                    h(&tp);
                                }
                            }
                            _ => {}
                        }

                        return Ok(head.messageno as i32);
                    }
                    self.step = 0;
                }
                // default: and case 0:
                _ => {
                    if data == RTCM3PREAMB {
                        self.step = 1;
                        self.packet[0] = data;
                    }
                }
            }
            break;
        }

        Ok(-1)
    }

    /// `ecef2pos(r, ref pos)`: ECEF to geodetic (lat, lon in radians, height).
    pub fn ecef2pos(r: &[f64], pos: &mut [f64]) {
        let e2 = FE_WGS84 * (2.0 - FE_WGS84);
        let r2 = dot(r, r, 2);
        let mut z = r[2];
        let mut zk = 0.0;
        let mut v = RE_WGS84;

        while (z - zk).abs() >= 1E-4 {
            zk = z;
            let sinp = z / (r2 + z * z).sqrt();
            v = RE_WGS84 / (1.0 - e2 * sinp * sinp).sqrt();
            z = r[2] + v * e2 * sinp;
        }
        pos[0] = if r2 > 1E-12 {
            (z / r2.sqrt()).atan()
        } else if r[2] > 0.0 {
            PI / 2.0
        } else {
            -PI / 2.0
        };
        pos[1] = if r2 > 1E-12 { r[1].atan2(r[0]) } else { 0.0 };
        pos[2] = (r2 + z * z).sqrt() - v;
    }

    /// `pos2ecef(pos, ref r)`: geodetic (lat, lon in radians, height) to ECEF.
    pub fn pos2ecef(pos: &[f64], r: &mut [f64]) {
        let sinp = pos[0].sin();
        let cosp = pos[0].cos();
        let sinl = pos[1].sin();
        let cosl = pos[1].cos();
        let e2 = FE_WGS84 * (2.0 - FE_WGS84);
        let v = RE_WGS84 / (1.0 - e2 * sinp * sinp).sqrt();

        r[0] = (v + pos[2]) * cosp * cosl;
        r[1] = (v + pos[2]) * cosp * sinl;
        r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
    }
}

impl ICorrections for rtcm3 {
    fn length(&self) -> i32 {
        (self.payloadlen + 2 + 1) as i32
    }

    fn packet(&self) -> &[u8] {
        &self.packet
    }

    fn resetParser(&mut self) -> bool {
        self.step = 0;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Wraps a payload into a frame: preamble, 10-bit length, payload, CRC-24Q.
    fn frame(payload: &[u8]) -> Vec<u8> {
        let mut f = vec![0u8; payload.len() + 6];
        let pre = rtcmpreamble { length: payload.len() as u16, ..Default::default() };
        pre.Write(&mut f).unwrap();
        f[3..3 + payload.len()].copy_from_slice(payload);
        let crc = crc24::crc24q(&f, (payload.len() + 3) as u32, 0).unwrap();
        setbitu(&mut f, ((payload.len() + 3) * 8) as u32, 24, crc).unwrap();
        f
    }

    #[test]
    fn base_position_round_trip() {
        let mut t = type1006 { anth: 1234, ..Default::default() };
        t.set_ecefposition(&[-4052051.7, 4212836.2, -2545106.0]).unwrap();
        let mut buf = vec![0u8; 64];
        let bits = t.Write(&mut buf).unwrap();
        assert_eq!(bits, 24 + 12 + 12 + 6 + 4 + 38 + 2 + 38 + 2 + 38 + 16);
        let payload = &buf[3..(bits as usize).div_ceil(8)];
        let f = frame(payload);

        let got = Arc::new(std::sync::Mutex::new(None));
        let g2 = got.clone();
        let mut p = rtcm3::new();
        p.base_pos_message.write().unwrap().push(Arc::new(move |b: &BasePos| *g2.lock().unwrap() = Some(*b)));
        let mut last = -1;
        for b in &f {
            last = p.Read(*b).unwrap();
        }
        // Write labels 1006 as 1005
        assert_eq!(last, 1005);
        match got.lock().unwrap().unwrap() {
            BasePos::Type1005(r) => assert!((r.ecefposition()[0] - -4052051.7).abs() < 1e-3),
            other => panic!("{other:?}"),
        }
        assert_eq!(p.length(), (payload.len() + 6) as i32);
    }

    #[test]
    fn bits_and_crc() {
        let mut b = [0u8; 4];
        setbitu(&mut b, 3, 10, 0x2AB).unwrap();
        assert_eq!(getbitu(&b, 3, 10).unwrap(), 0x2AB);
        setbits(&mut b, 0, 8, -3).unwrap();
        assert_eq!(getbits(&b, 0, 8).unwrap(), -3);
        assert!(getbitu(&b, 30, 8).is_err());
        assert_eq!(crc24::crc24q(b"", 0, 0).unwrap(), 0);
        let mut ecef = [0.0; 3];
        rtcm3::pos2ecef(&[0.0, 0.0, 0.0], &mut ecef);
        assert_eq!(ecef, [RE_WGS84, 0.0, 0.0]);
        let mut pos = [0.0; 3];
        rtcm3::ecef2pos(&ecef, &mut pos);
        assert!(pos[2].abs() < 1e-6);
    }
}
