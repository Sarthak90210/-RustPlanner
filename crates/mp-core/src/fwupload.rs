//! Firmware install: the ArduPilot/PX4 bootloader protocol (port of `ExtLibs/px4uploader`),
//! `.apj` decoding, and the firmware.ardupilot.org manifest (`APFirmware`).

use std::io::{self, Read, Write};
use std::time::{Duration, Instant};

use crate::comms::{self, ConnectionSpec, Transport};

const NOP: u8 = 0x00;
const OK: u8 = 0x10;
const INSYNC: u8 = 0x12;
const EOC: u8 = 0x20;
const GET_SYNC: u8 = 0x21;
const GET_DEVICE: u8 = 0x22;
const CHIP_ERASE: u8 = 0x23;
const PROG_MULTI: u8 = 0x27;
const GET_CRC: u8 = 0x29;
const REBOOT: u8 = 0x30;
const PROG_MULTI_MAX: usize = 252;

const INFO_BL_REV: u8 = 1;
const INFO_BOARD_ID: u8 = 2;
const INFO_BOARD_REV: u8 = 3;
const INFO_FLASH_SIZE: u8 = 4;

fn crc_table() -> [u32; 256] {
    let mut t = [0u32; 256];
    for (i, e) in t.iter_mut().enumerate() {
        let mut c = i as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
        }
        *e = c;
    }
    t
}

/// `Firmware.__crc32`: reflected CRC-32 without pre/post inversion.
pub fn crc32(bytes: &[u8], mut state: u32, tab: &[u32; 256]) -> u32 {
    for b in bytes {
        state = tab[((state ^ *b as u32) & 0xff) as usize] ^ (state >> 8);
    }
    state
}

/// `Firmware.crc(padlen)`: image CRC padded with 0xFF words up to the flash size.
pub fn image_crc(image: &[u8], padlen: usize) -> u32 {
    let tab = crc_table();
    let mut state = crc32(image, 0, &tab);
    let mut i = image.len();
    while i < padlen.saturating_sub(1) {
        state = crc32(&[0xff; 4], state, &tab);
        i += 4;
    }
    state
}

pub struct Apj {
    pub board_id: u32,
    pub image: Vec<u8>,
    pub summary: String,
}

/// Decode a `.apj` (JSON with a base64, zlib-compressed `image`).
pub fn parse_apj(text: &str) -> Result<Apj, String> {
    let v: serde_json::Value = serde_json::from_str(text).map_err(|e| e.to_string())?;
    let board_id = v["board_id"].as_u64().ok_or("missing board_id")? as u32;
    let b64 = v["image"].as_str().ok_or("missing image")?;
    let z = base64_decode(b64)?;
    let mut image = Vec::new();
    flate2::read::ZlibDecoder::new(&z[..]).read_to_end(&mut image).map_err(|e| e.to_string())?;
    while image.len() % 4 != 0 {
        image.push(0xff);
    }
    let summary = v["summary"].as_str().unwrap_or("").to_string();
    Ok(Apj { board_id, image, summary })
}

fn base64_decode(s: &str) -> Result<Vec<u8>, String> {
    let val = |c: u8| -> Option<u32> {
        Some(match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => return None,
        } as u32)
    };
    let clean: Vec<u8> = s.bytes().filter(|c| !c.is_ascii_whitespace()).collect();
    let mut out = Vec::with_capacity(clean.len() * 3 / 4);
    for chunk in clean.chunks(4) {
        let mut n = 0u32;
        let mut k: usize = 0;
        for &c in chunk {
            if c == b'=' {
                break;
            }
            n = (n << 6) | val(c).ok_or("bad base64")?;
            k += 1;
        }
        n <<= 6 * (4 - k) as u32;
        let bytes = [(n >> 16) as u8, (n >> 8) as u8, n as u8];
        out.extend_from_slice(&bytes[..k.saturating_sub(1)]);
    }
    Ok(out)
}

#[derive(Debug, Clone, Default)]
pub struct BoardInfo {
    pub port: String,
    pub bl_rev: u32,
    pub board_id: u32,
    pub board_rev: u32,
    pub fw_maxsize: u32,
}

pub struct Uploader {
    t: Box<dyn Transport>,
    pub info: BoardInfo,
}

impl Uploader {
    pub fn open(port: &str) -> io::Result<Self> {
        let t = comms::open(&ConnectionSpec::Serial { port: port.into(), baud: 115200 })?;
        Ok(Self { t, info: BoardInfo { port: port.into(), ..Default::default() } })
    }

    fn send(&mut self, b: &[u8]) -> io::Result<()> {
        self.t.write_all(b)?;
        self.t.flush()
    }

    fn recv(&mut self, n: usize, timeout: Duration) -> io::Result<Vec<u8>> {
        let end = Instant::now() + timeout;
        let mut out = Vec::with_capacity(n);
        let mut buf = [0u8; 64];
        while out.len() < n {
            if Instant::now() > end {
                return Err(io::Error::new(io::ErrorKind::TimedOut, "bootloader timeout"));
            }
            match self.t.read(&mut buf[..(n - out.len()).min(64)]) {
                Ok(k) => out.extend_from_slice(&buf[..k]),
                Err(e) if crate::comms::is_timeout(&e) => {}
                Err(e) => return Err(e),
            }
        }
        Ok(out)
    }

    fn get_sync(&mut self, timeout: Duration) -> io::Result<()> {
        let r = self.recv(2, timeout)?;
        if r[0] != INSYNC || r[1] != OK {
            return Err(io::Error::other(format!("bootloader sync failed {:02x} {:02x}", r[0], r[1])));
        }
        Ok(())
    }

    pub fn sync(&mut self) -> io::Result<()> {
        // drain
        let _ = self.recv(256, Duration::from_millis(30));
        self.send(&[GET_SYNC, EOC])?;
        self.get_sync(Duration::from_millis(500))
    }

    fn get_info(&mut self, p: u8) -> io::Result<u32> {
        self.send(&[GET_DEVICE, p, EOC])?;
        let r = self.recv(4, Duration::from_millis(500))?;
        self.get_sync(Duration::from_millis(500))?;
        Ok(u32::from_le_bytes([r[0], r[1], r[2], r[3]]))
    }

    /// `Uploader.identify`
    pub fn identify(&mut self) -> io::Result<BoardInfo> {
        self.sync()?;
        self.info.bl_rev = self.get_info(INFO_BL_REV)?;
        if !(2..=5).contains(&self.info.bl_rev) {
            return Err(io::Error::other("Bootloader protocol mismatch"));
        }
        self.info.board_id = self.get_info(INFO_BOARD_ID)?;
        self.info.board_rev = self.get_info(INFO_BOARD_REV)?;
        self.info.fw_maxsize = self.get_info(INFO_FLASH_SIZE)?;
        Ok(self.info.clone())
    }

    /// `Uploader.upload`: erase, program in 252-byte blocks, verify CRC, reboot.
    pub fn upload(&mut self, apj: &Apj, mut progress: impl FnMut(f32, &str)) -> io::Result<()> {
        if self.info.board_id != apj.board_id && !(self.info.board_id == 33 && apj.board_id == 9) {
            return Err(io::Error::other(format!("Firmware not suitable for this board fw:{} - board:{}", apj.board_id, self.info.board_id)));
        }
        if self.info.fw_maxsize != 0 && (self.info.fw_maxsize as usize) < apj.image.len() {
            return Err(io::Error::other("Firmware image is too large for this board"));
        }
        progress(0.0, "erase...");
        self.sync()?;
        self.get_info(INFO_BL_REV)?;
        self.send(&[CHIP_ERASE, EOC])?;
        self.get_sync(Duration::from_secs(20))?;
        let chunks: Vec<&[u8]> = apj.image.chunks(PROG_MULTI_MAX).collect();
        for (i, c) in chunks.iter().enumerate() {
            let mut pkt = Vec::with_capacity(c.len() + 3);
            pkt.push(PROG_MULTI);
            pkt.push(c.len() as u8);
            pkt.extend_from_slice(c);
            pkt.push(EOC);
            self.send(&pkt)?;
            self.get_sync(Duration::from_secs(1))?;
            if i % 16 == 0 {
                progress(i as f32 / chunks.len() as f32, "program...");
            }
        }
        progress(1.0, "verify...");
        let expect = image_crc(&apj.image, self.info.fw_maxsize as usize);
        self.send(&[GET_CRC, EOC])?;
        let r = self.recv(4, Duration::from_secs(10))?;
        self.get_sync(Duration::from_secs(1))?;
        let got = u32::from_le_bytes([r[0], r[1], r[2], r[3]]);
        if got != expect {
            return Err(io::Error::other(format!("Program CRC failed {got:08x} != {expect:08x}")));
        }
        progress(1.0, "reboot...");
        let _ = self.send(&[REBOOT, EOC]);
        let _ = NOP;
        Ok(())
    }
}

/// Scan every serial port for a board sitting in its bootloader (Instance_DeviceChanged).
pub fn find_bootloader(timeout: Duration) -> Option<BoardInfo> {
    let end = Instant::now() + timeout;
    while Instant::now() < end {
        for p in comms::serial_ports() {
            if let Ok(mut u) = Uploader::open(&p) {
                if let Ok(info) = u.identify() {
                    return Some(info);
                }
            }
        }
        std::thread::sleep(Duration::from_millis(250));
    }
    None
}

#[derive(Debug, Clone, serde::Deserialize)]
pub struct FirmwareInfo {
    #[serde(default)]
    pub board_id: Option<u32>,
    #[serde(rename = "mav-type", default)]
    pub mav_type: String,
    #[serde(rename = "mav-firmware-version-type", default)]
    pub version_type: String,
    #[serde(default)]
    pub format: String,
    #[serde(default)]
    pub url: String,
    #[serde(default)]
    pub vehicletype: String,
    #[serde(default)]
    pub platform: String,
    #[serde(rename = "mav-firmware-version-str", default)]
    pub version_str: Option<String>,
    #[serde(rename = "mav-firmware-version", default)]
    pub version: Option<String>,
    #[serde(rename = "USBID", default)]
    pub usbid: Vec<String>,
    #[serde(default)]
    pub latest: i64,
}

#[derive(serde::Deserialize)]
struct Manifest {
    firmware: Vec<FirmwareInfo>,
}

/// Parse `manifest.json` (already gunzipped).
pub fn parse_manifest(json: &[u8]) -> Result<Vec<FirmwareInfo>, String> {
    let m: Manifest = serde_json::from_slice(json).map_err(|e| e.to_string())?;
    Ok(m.firmware.into_iter().filter(|f| f.format == "apj").collect())
}

pub fn gunzip(bytes: &[u8]) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();
    flate2::read::GzDecoder::new(bytes).read_to_end(&mut out).map_err(|e| e.to_string())?;
    Ok(out)
}

/// `APFirmware.MAV_TYPE` names used by the Install Firmware tiles. "Copter" matches any
/// multirotor mav-type.
pub fn matches_type(f: &FirmwareInfo, mav_type: &str) -> bool {
    if mav_type == "Copter" {
        matches!(f.mav_type.as_str(), "QUADROTOR" | "HEXAROTOR" | "OCTOROTOR" | "TRICOPTER" | "COAXIAL" | "DODECAROTOR" | "DECAROTOR") || f.vehicletype == "Copter"
    } else {
        f.mav_type == mav_type
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apj_roundtrip_and_crc() {
        let raw: Vec<u8> = (0..1000u32).map(|i| (i % 256) as u8).collect();
        let mut z = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
        z.write_all(&raw).unwrap();
        let comp = z.finish().unwrap();
        const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        let mut b64 = String::new();
        for c in comp.chunks(3) {
            let b = [c[0], *c.get(1).unwrap_or(&0), *c.get(2).unwrap_or(&0)];
            let n = (b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32;
            for i in 0..4 {
                b64.push(if i <= c.len() { T[(n >> (18 - 6 * i) & 63) as usize] as char } else { '=' });
            }
        }
        let apj = parse_apj(&format!("{{\"board_id\": 9, \"image\": \"{b64}\"}}")).unwrap();
        assert_eq!(apj.board_id, 9);
        assert_eq!(&apj.image[..1000], &raw[..]);
        // standard CRC-32 of "123456789" with init/xorout 0xffffffff is 0xcbf43926
        let tab = crc_table();
        assert_eq!(!crc32(b"123456789", 0xffff_ffff, &tab), 0xcbf4_3926);
    }
}
