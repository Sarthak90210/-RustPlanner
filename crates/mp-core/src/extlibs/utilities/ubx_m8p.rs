//! Port of `ExtLibs/Utilities/ubx_m8p.cs`
//!
//! (class `Ubx`)
//!
//! u-blox UBX framing and checksum, the message structures Mission Planner reads from an
//! M8P/F9P base, and the receiver setup used by the RTK inject screen. The structures are
//! `[StructLayout(LayoutKind.Sequential, Pack = 1)]`; `from_bytes`/`to_bytes` are the
//! little-endian byte layouts `MavlinkUtil.ByteArrayToStructure`/`StructureToByteArray`
//! produce (a short buffer leaves the remaining fields zero).

#![allow(non_camel_case_types, non_snake_case)]

use std::thread::sleep;
use std::time::Duration;

use super::icorrections::ICorrections;
use super::pointlatlngalt::PointLatLngAlt;
use crate::dotnet::{cast_i32, cast_i8, cast_u32, index_out_of_range, Exception};
use crate::extlibs::interfaces::icommsserial::ICommsSerial;

/// Little-endian reader over a zero-padded copy of the input.
struct Rd {
    b: Vec<u8>,
    p: usize,
}

impl Rd {
    fn new(src: &[u8], size: usize) -> Self {
        let mut b = vec![0u8; size];
        let n = src.len().min(size);
        b[..n].copy_from_slice(&src[..n]);
        Rd { b, p: 0 }
    }
    fn take<const N: usize>(&mut self) -> [u8; N] {
        let mut a = [0u8; N];
        a.copy_from_slice(&self.b[self.p..self.p + N]);
        self.p += N;
        a
    }
    fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }
    fn i8(&mut self) -> i8 {
        self.u8() as i8
    }
    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take())
    }
    fn i32(&mut self) -> i32 {
        i32::from_le_bytes(self.take())
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take())
    }
    fn bytes(&mut self, n: usize) -> Vec<u8> {
        let v = self.b[self.p..self.p + n].to_vec();
        self.p += n;
        v
    }
}

/// `ubx_mon_ver`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ubx_mon_ver {
    pub swVersion: Vec<u8>,
    pub hwVersion: Vec<u8>,
    pub extension: Vec<u8>,
}

impl ubx_mon_ver {
    pub const SIZE: usize = 70;
    pub fn from_bytes(b: &[u8]) -> Self {
        let mut r = Rd::new(b, Self::SIZE);
        ubx_mon_ver { swVersion: r.bytes(30), hwVersion: r.bytes(10), extension: r.bytes(30) }
    }
}

/// `ubx_mon_hw`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ubx_mon_hw {
    pub pinSel: i32,
    pub pinBank: i32,
    pub pinDir: i32,
    pub pinVal: i32,
    pub noisePerMS: u16,
    pub agcCnt: u16,
    pub aStatus: u8,
    pub aPower: u8,
    pub flags: u8,
    pub reserved1: u8,
    pub usedMask: i32,
    pub VP: Vec<u8>,
    pub jamInd: u8,
    pub reserved3: u16,
    pub pinIrq: i32,
    pub pullH: i32,
    pub pullL: i32,
}

impl ubx_mon_hw {
    pub const SIZE: usize = 60;
    pub fn from_bytes(b: &[u8]) -> Self {
        let mut r = Rd::new(b, Self::SIZE);
        ubx_mon_hw {
            pinSel: r.i32(),
            pinBank: r.i32(),
            pinDir: r.i32(),
            pinVal: r.i32(),
            noisePerMS: r.u16(),
            agcCnt: r.u16(),
            aStatus: r.u8(),
            aPower: r.u8(),
            flags: r.u8(),
            reserved1: r.u8(),
            usedMask: r.i32(),
            VP: r.bytes(17),
            jamInd: r.u8(),
            reserved3: r.u16(),
            pinIrq: r.i32(),
            pullH: r.i32(),
            pullL: r.i32(),
        }
    }
}

/// `ubx_nav_pvt`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ubx_nav_pvt {
    pub itow: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub min: u8,
    pub sec: u8,
    pub valid: u8,
    pub t_acc: u32,
    pub nano: i32,
    pub fix_type: u8,
    pub flags: u8,
    pub flags2: u8,
    pub num_sv: u8,
    pub lon: i32,
    pub lat: i32,
    pub height: i32,
    pub h_msl: i32,
    pub h_acc: u32,
    pub v_acc: u32,
    pub velN: i32,
    pub velE: i32,
    pub velD: i32,
    pub gspeed: i32,
    pub head_mot: i32,
    pub s_acc: u32,
    pub head_acc: u32,
    pub p_dop: u16,
    pub reserved1: Vec<u8>,
    pub headVeh: u32,
    pub reserved2: Vec<u8>,
}

impl ubx_nav_pvt {
    pub const SIZE: usize = 92;
    pub fn from_bytes(b: &[u8]) -> Self {
        let mut r = Rd::new(b, Self::SIZE);
        ubx_nav_pvt {
            itow: r.u32(),
            year: r.u16(),
            month: r.u8(),
            day: r.u8(),
            hour: r.u8(),
            min: r.u8(),
            sec: r.u8(),
            valid: r.u8(),
            t_acc: r.u32(),
            nano: r.i32(),
            fix_type: r.u8(),
            flags: r.u8(),
            flags2: r.u8(),
            num_sv: r.u8(),
            lon: r.i32(),
            lat: r.i32(),
            height: r.i32(),
            h_msl: r.i32(),
            h_acc: r.u32(),
            v_acc: r.u32(),
            velN: r.i32(),
            velE: r.i32(),
            velD: r.i32(),
            gspeed: r.i32(),
            head_mot: r.i32(),
            s_acc: r.u32(),
            head_acc: r.u32(),
            p_dop: r.u16(),
            reserved1: r.bytes(6),
            headVeh: r.u32(),
            reserved2: r.bytes(4),
        }
    }
}

/// `ubx_nav_svin`: survey-in status.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ubx_nav_svin {
    pub version: u8,
    pub reserved1: Vec<u8>,
    pub iTOW: u32,
    pub dur: u32,
    pub meanX: i32,
    pub meanY: i32,
    pub meanZ: i32,
    pub meanXHP: i8,
    pub meanYHP: i8,
    pub meanZHP: i8,
    pub reserved2: u8,
    pub meanAcc: u32,
    pub obs: u32,
    pub valid: u8,
    pub active: u8,
    pub reserved3: Vec<u8>,
}

impl ubx_nav_svin {
    pub const SIZE: usize = 40;
    pub fn from_bytes(b: &[u8]) -> Self {
        let mut r = Rd::new(b, Self::SIZE);
        ubx_nav_svin {
            version: r.u8(),
            reserved1: r.bytes(3),
            iTOW: r.u32(),
            dur: r.u32(),
            meanX: r.i32(),
            meanY: r.i32(),
            meanZ: r.i32(),
            meanXHP: r.i8(),
            meanYHP: r.i8(),
            meanZHP: r.i8(),
            reserved2: r.u8(),
            meanAcc: r.u32(),
            obs: r.u32(),
            valid: r.u8(),
            active: r.u8(),
            reserved3: r.bytes(2),
        }
    }

    /// `getECEF()`: the mean position in metres (cm plus 0.1 mm high-precision part).
    pub fn getECEF(&self) -> [f64; 3] {
        let x = self.meanX as f64 / 100.0 + self.meanXHP as f64 * 0.0001;
        let y = self.meanY as f64 / 100.0 + self.meanYHP as f64 * 0.0001;
        let z = self.meanZ as f64 / 100.0 + self.meanZHP as f64 * 0.0001;

        [x, y, z]
    }
}

/// `ubx_nav_velned`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ubx_nav_velned {
    pub iTOW: u32,
    pub velN: i32,
    pub velE: i32,
    pub velD: i32,
    pub speed: u32,
    pub gSpeed: u32,
    pub heading: i32,
    pub sAcc: u32,
    pub cAcc: u32,
}

impl ubx_nav_velned {
    pub const SIZE: usize = 36;
    pub fn from_bytes(b: &[u8]) -> Self {
        let mut r = Rd::new(b, Self::SIZE);
        ubx_nav_velned {
            iTOW: r.u32(),
            velN: r.i32(),
            velE: r.i32(),
            velD: r.i32(),
            speed: r.u32(),
            gSpeed: r.u32(),
            heading: r.i32(),
            sAcc: r.u32(),
            cAcc: r.u32(),
        }
    }
}

/// `ubx_cfg_nav5_getset_s`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ubx_cfg_nav5_getset_s {
    pub mask: u16,
    pub dynModel: u8,
    pub fixMode: u8,
    pub fixedAlt: i32,
    pub fixedAltVar: u32,
    pub minElev: i8,
    pub drLimit: u8,
    pub pDop: u16,
    pub tDop: u16,
    pub pAcc: u16,
    pub tAcc: u16,
    pub staticHoldThresh: u8,
    pub dgnssTimeout: u8,
    pub cnoThreshNumSVs: u8,
    pub cnoThresh: u8,
    pub reserved1: Vec<u8>,
    pub staticHoldMaxDist: u16,
    pub utcStandard: u8,
    pub reserved2: Vec<u8>,
}

impl ubx_cfg_nav5_getset_s {
    pub const SIZE: usize = 36;
    pub fn from_bytes(b: &[u8]) -> Self {
        let mut r = Rd::new(b, Self::SIZE);
        ubx_cfg_nav5_getset_s {
            mask: r.u16(),
            dynModel: r.u8(),
            fixMode: r.u8(),
            fixedAlt: r.i32(),
            fixedAltVar: r.u32(),
            minElev: r.i8(),
            drLimit: r.u8(),
            pDop: r.u16(),
            tDop: r.u16(),
            pAcc: r.u16(),
            tAcc: r.u16(),
            staticHoldThresh: r.u8(),
            dgnssTimeout: r.u8(),
            cnoThreshNumSVs: r.u8(),
            cnoThresh: r.u8(),
            reserved1: r.bytes(2),
            staticHoldMaxDist: r.u16(),
            utcStandard: r.u8(),
            reserved2: r.bytes(5),
        }
    }
}

crate::cs_enum! {
    /// `ubx_cfg_tmode3.modeflags`
    pub modeflags: i32 {
        Disabled = 0,
        SurveyIn = 1,
        FixedECEF = 2,
        LLA = 256,
        FixedLLA = 258,
    }
}

/// `ubx_cfg_tmode3` (`Size = 40`): base station time mode.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ubx_cfg_tmode3 {
    pub version: u8,
    pub reserved1: u8,
    pub flags: u16,
    /// 1e7
    pub ecefXorLat: i32,
    pub ecefYorLon: i32,
    pub ecefZorAlt: i32,
    pub ecefXOrLatHP: i8,
    pub ecefYOrLonHP: i8,
    pub ecefZOrAltHP: i8,
    pub reserved2: u8,
    pub fixedPosAcc: u32,
    pub svinMinDur: u32,
    pub svinAccLimit: u32,
    pub reserved3: Vec<u8>,
}

impl ubx_cfg_tmode3 {
    pub const SIZE: usize = 40;

    /// `new ubx_cfg_tmode3(lat, lng, alt, acc = 0.001)`: a fixed position; |lat| > 90 means
    /// the values are ECEF metres.
    pub fn fixed(lat: f64, lng: f64, alt: f64, acc: Option<f64>) -> Self {
        let acc = acc.unwrap_or(0.001);
        let mut s = ubx_cfg_tmode3::default();
        if lat.abs() > 90.0 {
            s.flags = 2; // fixed mode ecef
            s.ecefXorLat = cast_i32(lat * 100.0);
            s.ecefYorLon = cast_i32(lng * 100.0);
            s.ecefZorAlt = cast_i32(alt * 100.0);
            s.ecefXOrLatHP = cast_i8((lat * 100.0 - s.ecefXorLat as f64) * 100.0);
            s.ecefYOrLonHP = cast_i8((lng * 100.0 - s.ecefYorLon as f64) * 100.0);
            s.ecefZOrAltHP = cast_i8((alt * 100.0 - s.ecefZorAlt as f64) * 100.0);
        } else {
            s.flags = 256 + 2; // lla + fixed mode
            s.ecefXorLat = cast_i32(lat * 1e7);
            s.ecefYorLon = cast_i32(lng * 1e7);
            s.ecefZorAlt = cast_i32(alt * 100.0);
            s.ecefXOrLatHP = cast_i8((lat * 1e7 - s.ecefXorLat as f64) * 100.0);
            s.ecefYOrLonHP = cast_i8((lng * 1e7 - s.ecefYorLon as f64) * 100.0);
            s.ecefZOrAltHP = cast_i8((alt * 100.0 - s.ecefZorAlt as f64) * 100.0);
        }

        s.reserved2 = 0;
        s.fixedPosAcc = cast_u32(acc * 1000.0);
        s.svinMinDur = 60;
        s.svinAccLimit = 2000;
        s.reserved3 = vec![0u8; 8];
        s
    }

    /// `new ubx_cfg_tmode3(uint DurationS, double AccLimit)`: survey-in.
    pub fn survey_in(duration_s: u32, acc_limit: f64) -> Self {
        ubx_cfg_tmode3 {
            flags: 1, // surveyin mode
            svinMinDur: duration_s,
            svinAccLimit: cast_u32(acc_limit * 10000.0),
            reserved3: vec![0u8; 8],
            ..Default::default()
        }
    }

    /// `ubx_cfg_tmode3.Disable`
    pub fn disable() -> Self {
        ubx_cfg_tmode3 { flags: 0, reserved3: vec![0u8; 8], ..Default::default() }
    }

    /// `implicit operator byte[](ubx_cfg_tmode3)`: `StructureToByteArray`, 40 bytes.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut v = Vec::with_capacity(Self::SIZE);
        v.push(self.version);
        v.push(self.reserved1);
        v.extend_from_slice(&self.flags.to_le_bytes());
        v.extend_from_slice(&self.ecefXorLat.to_le_bytes());
        v.extend_from_slice(&self.ecefYorLon.to_le_bytes());
        v.extend_from_slice(&self.ecefZorAlt.to_le_bytes());
        v.push(self.ecefXOrLatHP as u8);
        v.push(self.ecefYOrLonHP as u8);
        v.push(self.ecefZOrAltHP as u8);
        v.push(self.reserved2);
        v.extend_from_slice(&self.fixedPosAcc.to_le_bytes());
        v.extend_from_slice(&self.svinMinDur.to_le_bytes());
        v.extend_from_slice(&self.svinAccLimit.to_le_bytes());
        let mut r3 = self.reserved3.clone();
        r3.resize(8, 0);
        v.extend_from_slice(&r3);
        v
    }

    /// `getPointLatLngAlt()`: the fixed position (ECEF metres or lat/lng/alt), or null for
    /// other modes.
    pub fn getPointLatLngAlt(&self) -> Option<PointLatLngAlt> {
        if self.flags == 2 {
            let x = self.ecefXorLat as f64 / 100.0 + self.ecefXOrLatHP as f64 * 0.0001;
            let y = self.ecefYorLon as f64 / 100.0 + self.ecefYOrLonHP as f64 * 0.0001;
            let z = self.ecefZorAlt as f64 / 100.0 + self.ecefZOrAltHP as f64 * 0.0001;

            return PointLatLngAlt::from_dblarr(&[x, y, z]).ok();
        } else if self.flags == 258 {
            let x = self.ecefXorLat as f64 / 1e7 + self.ecefXOrLatHP as f64 / 1e9;
            let y = self.ecefYorLon as f64 / 1e7 + self.ecefYOrLonHP as f64 / 1e9;
            let z = self.ecefZorAlt as f64 / 100.0 + self.ecefZOrAltHP as f64 * 0.0001;

            return PointLatLngAlt::from_dblarr(&[x, y, z]).ok();
        }

        None
    }
}

/// `public class Ubx : ICorrections`
#[derive(Debug, Clone)]
pub struct Ubx {
    step: i32,
    pub buffer: Vec<u8>,
    payloadlen: i32,
    msglencount: i32,
}

impl Default for Ubx {
    fn default() -> Self {
        Ubx { step: 0, buffer: vec![0u8; 1024 * 8], payloadlen: 0, msglencount: 0 }
    }
}

fn put(buf: &mut [u8], idx: i32, v: u8) -> Result<(), Exception> {
    if idx < 0 {
        return Err(index_out_of_range());
    }
    *buf.get_mut(idx as usize).ok_or_else(index_out_of_range)? = v;
    Ok(())
}

impl Ubx {
    pub fn new() -> Self {
        Self::default()
    }

    /// `class` (`buffer[2]`; the public buffer can be replaced by a shorter array, which
    /// throws IndexOutOfRangeException)
    pub fn class(&self) -> Result<u8, Exception> {
        self.buffer.get(2).copied().ok_or_else(index_out_of_range)
    }

    /// `subclass` (`buffer[3]`)
    pub fn subclass(&self) -> Result<u8, Exception> {
        self.buffer.get(3).copied().ok_or_else(index_out_of_range)
    }

    /// `Read(byte data)`: (class << 8) + subclass when a frame with a valid checksum ends
    /// at this byte, otherwise -1. A zero-length payload leaves the parser in its payload
    /// state until reset, and a payload within 6 bytes of the buffer size overruns it
    /// (IndexOutOfRangeException), as in C#.
    pub fn Read(&mut self, data: u8) -> Result<i32, Exception> {
        match self.step {
            1 => {
                if data == 0x62 {
                    put(&mut self.buffer, 1, data)?;
                    self.step += 1;
                } else {
                    self.step = 0;
                }
            }
            2 => {
                put(&mut self.buffer, 2, data)?;
                self.step += 1;
            }
            3 => {
                put(&mut self.buffer, 3, data)?;
                self.step += 1;
            }
            4 => {
                put(&mut self.buffer, 4, data)?;
                self.payloadlen = data as i32;
                self.step += 1;
            }
            5 => {
                put(&mut self.buffer, 5, data)?;
                self.step += 1;
                self.payloadlen += (data as i32) << 8;
                self.msglencount = 0;
                // reset on oversize packet
                if self.payloadlen > self.buffer.len() as i32 {
                    self.step = 0;
                }
            }
            6 => {
                if self.msglencount < self.payloadlen {
                    put(&mut self.buffer, self.msglencount + 6, data)?;
                    self.msglencount += 1;

                    if self.msglencount == self.payloadlen {
                        self.step += 1;
                    }
                }
            }
            7 => {
                put(&mut self.buffer, self.msglencount + 6, data)?;
                self.step += 1;
            }
            8 => {
                put(&mut self.buffer, self.msglencount + 6 + 1, data)?;

                let crc = ubx_checksum(&self.buffer, self.payloadlen + 6, 2)?;

                let first = *self.buffer.get((self.msglencount + 6) as usize).ok_or_else(index_out_of_range)?;
                let crcpacket = [first, data];

                if crc[0] == crcpacket[0] && crc[1] == crcpacket[1] {
                    self.step = 0;
                    return Ok(((self.class()? as i32) << 8) + self.subclass()? as i32);
                }

                self.step = 0;
            }
            // default: and case 0:
            _ => {
                if data == 0xb5 {
                    self.step = 1;
                    put(&mut self.buffer, 0, data)?;
                }
            }
        }

        Ok(-1)
    }

    /// `generate(cl, subclass, payload)`: a framed UBX message.
    pub fn generate(cl: u8, subclass: u8, payload: &[u8]) -> Vec<u8> {
        let mut data = vec![0u8; 2 + 2 + 2 + 2 + payload.len()];
        data[0] = 0xb5;
        data[1] = 0x62;
        data[2] = cl;
        data[3] = subclass;
        data[4] = (payload.len() & 0xff) as u8;
        data[5] = ((payload.len() >> 8) & 0xff) as u8;

        data[6..6 + payload.len()].copy_from_slice(payload);

        let n = data.len() as i32;
        let checksum = ubx_checksum(&data, n - 2, 2).expect("in range");

        data[n as usize - 2] = checksum[0];
        data[n as usize - 1] = checksum[1];

        data
    }

    fn write(port: &mut dyn ICommsSerial, packet: &[u8]) -> Result<(), Exception> {
        port.write(packet, 0, packet.len() as i32)
    }

    /// `SetupM8P(port, m8p_130plus, movingbase)`: brings the receiver to 460800 baud, 1 Hz,
    /// stationary, NMEA off, and enables the RTCM, survey-in, PVT, raw and monitor messages.
    pub fn SetupM8P(&self, port: &mut dyn ICommsSerial, m8p_130plus: bool, _movingbase: bool) -> Result<(), Exception> {
        port.base_stream()?.flush()?;

        let bauds = [port.baud_rate(), 9600, 38400, 57600, 115200, 230400, 460800];

        // change the baudrate
        for baud in bauds {
            port.set_baud_rate(baud);

            sleep(Duration::from_millis(50));
            // U = bit  01010101  - often used for autobaud
            port.write_str("UU")?;
            // port config - 460800 - uart1
            let packet = Self::generate(
                0x6,
                0x00,
                &[0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x08, 0x07, 0x00, 0x23, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00],
            );
            Self::write(port, &packet)?;
            port.base_stream()?.flush()?;
            sleep(Duration::from_millis(100));
        }

        {
            // port config - usb
            let packet = Self::generate(
                0x6,
                0x00,
                &[0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00],
            );
            Self::write(port, &packet)?;
            sleep(Duration::from_millis(300));

            port.base_stream()?.flush()?;

            port.set_baud_rate(460800);
        }

        {
            // set rate to 1hz
            let packet = Self::generate(0x6, 0x8, &[0xE8, 0x03, 0x01, 0x00, 0x01, 0x00]);
            Self::write(port, &packet)?;
            sleep(Duration::from_millis(200));

            // set navmode to stationary
            let packet = Self::generate(
                0x6,
                0x24,
                &[
                    0xFF, 0xFF, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x0F, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00,
                    0x2C, 0x01, 0x00, 0x00, 0x00, 0x23, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                ],
            );
            Self::write(port, &packet)?;
            sleep(Duration::from_millis(200));
        }

        // turn off all nmea
        for a in 0..=0xfu8 {
            if a == 0xb || a == 0xc || a == 0xe {
                continue;
            }
            self.turnon_off(port, 0xf0, a, 0)?;
        }

        // mon-ver
        self.poll_msg(port, 0xa, 0x4)?;

        // surveyin msg - for feedback
        self.turnon_off(port, 0x01, 0x3b, 1)?;

        // pvt msg - for feedback
        self.turnon_off(port, 0x01, 0x07, 1)?;

        // 1005 - 5s
        self.turnon_off(port, 0xf5, 0x05, 5)?;

        let mut rate1 = 1u8;
        let mut rate2 = 0u8;

        if m8p_130plus {
            rate1 = 0;
            rate2 = 1;
        }

        // 1074 / 1077 - 1s
        self.turnon_off(port, 0xf5, 0x4a, rate2)?;
        self.turnon_off(port, 0xf5, 0x4d, rate1)?;

        // 1084 / 1087 - 1s
        self.turnon_off(port, 0xf5, 0x54, rate2)?;
        self.turnon_off(port, 0xf5, 0x57, rate1)?;

        // 1094 / 1097 - 1s
        self.turnon_off(port, 0xf5, 0x5e, rate2)?;
        self.turnon_off(port, 0xf5, 0x61, rate1)?;

        // 1124 / 1127 - 1s
        self.turnon_off(port, 0xf5, 0x7c, rate2)?;
        self.turnon_off(port, 0xf5, 0x7f, rate1)?;

        // 4072
        self.turnon_off(port, 0xf5, 0xFE, 0)?;

        // 1230 - 5s
        self.turnon_off(port, 0xf5, 0xE6, 5)?;

        // NAV-VELNED - 1s
        self.turnon_off(port, 0x01, 0x12, 1)?;

        // rxm-raw/rawx - 1s
        self.turnon_off(port, 0x02, 0x15, 1)?;
        self.turnon_off(port, 0x02, 0x10, 1)?;

        // rxm-sfrb/sfrb - 2s
        self.turnon_off(port, 0x02, 0x13, 2)?;
        self.turnon_off(port, 0x02, 0x11, 2)?;

        // mon-hw - 2s
        self.turnon_off(port, 0x0a, 0x09, 2)?;

        sleep(Duration::from_millis(100));
        Ok(())
    }

    /// `SetupBasePos(port, basepos, surveyindur = 0, surveyinacc = 0, disable = false,
    /// movingbase = false)`: survey-in when `basepos` equals `PointLatLngAlt.Zero`, otherwise
    /// a fixed position; `disable` (or a moving base) turns time mode off, saves and reboots.
    /// `None` is a null `basepos`: unequal to Zero, so it throws NullReferenceException at
    /// `basepos.Lat` after the two sleeps.
    pub fn SetupBasePos(
        &self,
        port: &mut dyn ICommsSerial,
        basepos: Option<&PointLatLngAlt>,
        mut surveyindur: i32,
        mut surveyinacc: f64,
        mut disable: bool,
        movingbase: bool,
    ) -> Result<(), Exception> {
        if movingbase {
            disable = true;
        }

        sleep(Duration::from_millis(100));
        sleep(Duration::from_millis(100));

        if surveyindur == 0 {
            surveyindur = 60;
        }
        if surveyinacc == 0.0 {
            surveyinacc = 2.0;
        }

        if disable {
            // disable
            let packet = Self::generate(0x6, 0x71, &ubx_cfg_tmode3::disable().to_bytes());
            Self::write(port, &packet)?;
            // save - bbr
            let packet = Self::generate(0x06, 0x09, &[0, 0, 0, 0, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0x01]);
            Self::write(port, &packet)?;
            sleep(Duration::from_millis(1000));
            //reboot
            let packet = Self::generate(0x06, 0x04, &[0x14, 0xff, 2, 0]);
            Self::write(port, &packet)?;
            sleep(Duration::from_millis(3000));
            return Ok(());
        }

        // operator ==(PointLatLngAlt, PointLatLngAlt) with a null left side is false
        if basepos.is_some_and(|b| *b == PointLatLngAlt::zero()) {
            // survey in config
            let packet = Self::generate(0x6, 0x71, &ubx_cfg_tmode3::survey_in(surveyindur as u32, surveyinacc).to_bytes());
            Self::write(port, &packet)?;
        } else {
            let basepos = basepos.ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
            let data = ubx_cfg_tmode3::fixed(basepos.Lat, basepos.Lng, basepos.Alt, None).to_bytes();
            let packet = Self::generate(0x6, 0x71, &data);
            Self::write(port, &packet)?;
        }
        Ok(())
    }

    /// `turnon_off(port, clas, subclass, every_xsamples)`: CFG-MSG rate on ports 1 and 2.
    pub fn turnon_off(&self, port: &mut dyn ICommsSerial, clas: u8, subclass: u8, every_xsamples: u8) -> Result<(), Exception> {
        let datastruct1 = [clas, subclass, 0, every_xsamples, 0, every_xsamples, 0, 0];

        let packet = Self::generate(0x6, 0x1, &datastruct1);

        Self::write(port, &packet)?;

        sleep(Duration::from_millis(10));
        Ok(())
    }

    /// `poll_msg(port, clas, subclass)`: an empty-payload poll.
    pub fn poll_msg(&self, port: &mut dyn ICommsSerial, clas: u8, subclass: u8) -> Result<(), Exception> {
        let packet = Self::generate(clas, subclass, &[]);

        Self::write(port, &packet)?;

        sleep(Duration::from_millis(10));
        Ok(())
    }
}

/// `ubx_checksum(packet, size, offset = 2)`: the 8-bit Fletcher sums over [offset, size).
fn ubx_checksum(packet: &[u8], size: i32, offset: i32) -> Result<[u8; 2], Exception> {
    let mut a: u32 = 0x00;
    let mut b: u32 = 0x00;
    let mut i = offset;
    while i < size {
        if i < 0 {
            return Err(index_out_of_range());
        }
        a = a.wrapping_add(*packet.get(i as usize).ok_or_else(index_out_of_range)? as u32);
        i += 1;
        b = b.wrapping_add(a);
    }

    Ok([(a & 0xFF) as u8, (b & 0xFF) as u8])
}

impl ICorrections for Ubx {
    /// header2, class, subclass, length2, data, crc2
    fn length(&self) -> i32 {
        2 + 2 + 2 + 2 + self.payloadlen
    }

    fn packet(&self) -> &[u8] {
        &self.buffer
    }

    fn resetParser(&mut self) -> bool {
        self.step = 0;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generate_and_parse() {
        let f = Ubx::generate(0x01, 0x07, &[1, 2, 3]);
        assert_eq!(f.len(), 11);
        let mut p = Ubx::new();
        let mut last = -1;
        for b in &f {
            last = p.Read(*b).unwrap();
        }
        assert_eq!(last, 0x0107);
        assert_eq!(p.length(), 11);
        assert_eq!(ubx_cfg_tmode3::disable().to_bytes().len(), 40);
        let t = ubx_cfg_tmode3::fixed(-35.5, 149.25, 584.0, None);
        assert_eq!(t.flags, 258);
        let back = t.getPointLatLngAlt().unwrap();
        assert!((back.Lat - -35.5).abs() < 1e-9 && (back.Alt - 584.0).abs() < 1e-9);
        assert!(ubx_cfg_tmode3::survey_in(60, 2.0).getPointLatLngAlt().is_none());
    }
}
