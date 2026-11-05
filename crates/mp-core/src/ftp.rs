//! MAVLink FTP client (`ExtLibs/ArduPilot/Mavlink/MAVFtp.cs`): FILE_TRANSFER_PROTOCOL requests
//! with retries, run on the caller's thread over a [`Link`] subscription.

use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender};
use mavlink::dialects::ardupilotmega::{MavMessage, FILE_TRANSFER_PROTOCOL_DATA};

use crate::link::{Link, LinkCommand};

pub const MSG_ID: u32 = 110;
pub const DATA_LEN: usize = 239;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Op {
    TerminateSession = 1,
    ResetSessions = 2,
    ListDirectory = 3,
    OpenFileRO = 4,
    ReadFile = 5,
    CreateFile = 6,
    WriteFile = 7,
    RemoveFile = 8,
    CreateDirectory = 9,
    RemoveDirectory = 10,
    OpenFileWO = 11,
    TruncateFile = 12,
    Rename = 13,
    CalcFileCRC32 = 14,
    BurstReadFile = 15,
    Ack = 128,
    Nak = 129,
}

/// FTP error codes carried in a NAK.
pub fn error_name(code: u8) -> &'static str {
    match code {
        1 => "Fail",
        2 => "FailErrno",
        3 => "InvalidDataSize",
        4 => "InvalidSession",
        5 => "NoSessionsAvailable",
        6 => "EOF",
        7 => "UnknownCommand",
        8 => "FileExists",
        9 => "FileProtected",
        10 => "FileNotFound",
        _ => "Unknown",
    }
}

/// One FTP packet (the 251-byte payload of FILE_TRANSFER_PROTOCOL).
#[derive(Debug, Clone)]
pub struct Packet {
    pub seq: u16,
    pub session: u8,
    pub opcode: u8,
    pub size: u8,
    pub req_opcode: u8,
    pub burst_complete: u8,
    pub offset: u32,
    pub data: Vec<u8>,
}

impl Packet {
    pub fn encode(&self) -> [u8; 251] {
        let mut p = [0u8; 251];
        p[0..2].copy_from_slice(&self.seq.to_le_bytes());
        p[2] = self.session;
        p[3] = self.opcode;
        p[4] = self.size;
        p[5] = self.req_opcode;
        p[6] = self.burst_complete;
        p[8..12].copy_from_slice(&self.offset.to_le_bytes());
        let n = self.data.len().min(DATA_LEN);
        p[12..12 + n].copy_from_slice(&self.data[..n]);
        p
    }

    pub fn decode(p: &[u8; 251]) -> Self {
        let size = p[4];
        Self {
            seq: u16::from_le_bytes([p[0], p[1]]),
            session: p[2],
            opcode: p[3],
            size,
            req_opcode: p[5],
            burst_complete: p[6],
            offset: u32::from_le_bytes([p[8], p[9], p[10], p[11]]),
            data: p[12..12 + (size as usize).min(DATA_LEN)].to_vec(),
        }
    }
}

/// A directory listing entry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Entry {
    pub name: String,
    pub is_dir: bool,
    pub size: u64,
}

/// Standard CRC-32 (ArduPilot `crc_crc32`, as used by CalcFileCRC32).
pub fn crc32(mut crc: u32, data: &[u8]) -> u32 {
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    crc
}

pub struct Ftp {
    tx: Sender<LinkCommand>,
    rx: Receiver<MavMessage>,
    sys: u8,
    comp: u8,
    seq: u16,
    pub timeout: Duration,
}

impl Ftp {
    pub fn new(link: &Link) -> Self {
        let (stx, srx) = crossbeam_channel::unbounded();
        link.send(LinkCommand::Subscribe { msgid: MSG_ID, tx: stx });
        let (sys, comp) = {
            let s = link.shared.read();
            (s.cs.sysid, s.cs.compid)
        };
        Self { tx: link.sender(), rx: srx, sys, comp, seq: 0, timeout: Duration::from_millis(1500) }
    }

    fn request(&mut self, op: Op, session: u8, offset: u32, data: &[u8], size: u8) -> Result<Packet, String> {
        for _attempt in 0..4 {
            self.seq = self.seq.wrapping_add(1);
            let pkt = Packet { seq: self.seq, session, opcode: op as u8, size, req_opcode: 0, burst_complete: 0, offset, data: data.to_vec() };
            let msg = MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA { target_network: 0, target_system: self.sys, target_component: self.comp, payload: pkt.encode() });
            self.tx.send(LinkCommand::Send(Box::new(msg))).map_err(|_| "link closed".to_string())?;
            let end = Instant::now() + self.timeout;
            while let Some(left) = end.checked_duration_since(Instant::now()) {
                match self.rx.recv_timeout(left) {
                    Ok(MavMessage::FILE_TRANSFER_PROTOCOL(f)) => {
                        let r = Packet::decode(&f.payload);
                        if r.req_opcode == op as u8 && r.seq == self.seq.wrapping_add(1) {
                            self.seq = r.seq;
                            return match r.opcode {
                                128 => Ok(r),
                                129 => Err(error_name(r.data.first().copied().unwrap_or(1)).to_string()),
                                _ => Err("bad reply".into()),
                            };
                        }
                    }
                    Ok(_) => {}
                    Err(_) => break,
                }
            }
        }
        Err("timeout".into())
    }

    fn path_bytes(path: &str) -> (Vec<u8>, u8) {
        let b = path.as_bytes().to_vec();
        let n = b.len().min(DATA_LEN) as u8;
        (b, n)
    }

    pub fn reset(&mut self) -> Result<(), String> {
        self.request(Op::ResetSessions, 0, 0, &[], 0).map(|_| ())
    }

    pub fn list(&mut self, path: &str) -> Result<Vec<Entry>, String> {
        let (p, n) = Self::path_bytes(path);
        let mut out = Vec::new();
        loop {
            let r = match self.request(Op::ListDirectory, 0, out.len() as u32, &p, n) {
                Ok(r) => r,
                Err(e) if e == "EOF" => break,
                Err(e) => return Err(e),
            };
            let mut added = 0;
            for item in r.data.split(|b| *b == 0).filter(|s| !s.is_empty()) {
                let s = String::from_utf8_lossy(item);
                added += 1;
                match s.chars().next() {
                    Some('D') => out.push(Entry { name: s[1..].to_string(), is_dir: true, size: 0 }),
                    Some('F') => {
                        let (name, size) = s[1..].split_once('\t').unwrap_or((&s[1..], "0"));
                        out.push(Entry { name: name.to_string(), is_dir: false, size: size.parse().unwrap_or(0) });
                    }
                    _ => out.push(Entry { name: String::new(), is_dir: false, size: 0 }),
                }
            }
            if added == 0 {
                break;
            }
        }
        out.retain(|e| !e.name.is_empty() && e.name != "." && e.name != "..");
        Ok(out)
    }

    pub fn get(&mut self, path: &str, mut progress: impl FnMut(u64, u64)) -> Result<Vec<u8>, String> {
        let (p, n) = Self::path_bytes(path);
        let open = self.request(Op::OpenFileRO, 0, 0, &p, n)?;
        let session = open.session;
        let size = open.data.get(0..4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]])).unwrap_or(0) as u64;
        let mut data = Vec::with_capacity(size as usize);
        let result = loop {
            match self.request(Op::ReadFile, session, data.len() as u32, &[], DATA_LEN as u8) {
                Ok(r) if r.data.is_empty() => break Ok(()),
                Ok(r) => {
                    data.extend_from_slice(&r.data);
                    progress(data.len() as u64, size);
                    if size > 0 && data.len() as u64 >= size {
                        break Ok(());
                    }
                }
                Err(e) if e == "EOF" => break Ok(()),
                Err(e) => break Err(e),
            }
        };
        let _ = self.request(Op::TerminateSession, session, 0, &[], 0);
        result.map(|_| data)
    }

    pub fn put(&mut self, path: &str, bytes: &[u8], mut progress: impl FnMut(u64, u64)) -> Result<(), String> {
        let (p, n) = Self::path_bytes(path);
        let open = self.request(Op::CreateFile, 0, 0, &p, n)?;
        let session = open.session;
        let mut off = 0usize;
        let result = loop {
            if off >= bytes.len() {
                break Ok(());
            }
            let chunk = &bytes[off..(off + DATA_LEN).min(bytes.len())];
            if let Err(e) = self.request(Op::WriteFile, session, off as u32, chunk, chunk.len() as u8) {
                break Err(e);
            }
            off += chunk.len();
            progress(off as u64, bytes.len() as u64);
        };
        let _ = self.request(Op::TerminateSession, session, 0, &[], 0);
        result
    }

    pub fn remove(&mut self, path: &str) -> Result<(), String> {
        let (p, n) = Self::path_bytes(path);
        self.request(Op::RemoveFile, 0, 0, &p, n).map(|_| ())
    }

    pub fn mkdir(&mut self, path: &str) -> Result<(), String> {
        let (p, n) = Self::path_bytes(path);
        self.request(Op::CreateDirectory, 0, 0, &p, n).map(|_| ())
    }

    pub fn rmdir(&mut self, path: &str) -> Result<(), String> {
        let (p, n) = Self::path_bytes(path);
        self.request(Op::RemoveDirectory, 0, 0, &p, n).map(|_| ())
    }

    pub fn rename(&mut self, from: &str, to: &str) -> Result<(), String> {
        let mut b = from.as_bytes().to_vec();
        b.push(0);
        b.extend_from_slice(to.as_bytes());
        let n = b.len().min(DATA_LEN) as u8;
        self.request(Op::Rename, 0, 0, &b, n).map(|_| ())
    }

    pub fn crc32(&mut self, path: &str) -> Result<u32, String> {
        let (p, n) = Self::path_bytes(path);
        let r = self.request(Op::CalcFileCRC32, 0, 0, &p, n)?;
        r.data.get(0..4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]])).ok_or_else(|| "bad reply".into())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn packet_roundtrip() {
        let p = super::Packet { seq: 7, session: 1, opcode: 5, size: 3, req_opcode: 0, burst_complete: 0, offset: 512, data: vec![1, 2, 3] };
        let d = super::Packet::decode(&p.encode());
        assert_eq!((d.seq, d.session, d.opcode, d.offset, d.data), (7, 1, 5, 512, vec![1, 2, 3]));
    }
}
