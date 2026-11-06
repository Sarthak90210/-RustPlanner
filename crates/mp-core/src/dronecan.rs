//! DroneCAN (UAVCAN v0) over MAVLink CAN_FRAME forwarding (MAV_CMD_CAN_FORWARD) or an SLCAN
//! serial adapter: frame ids, single/multi-frame transfers, NodeStatus, GetNodeInfo,
//! RestartNode and debug.LogMessage, and the node table ConfigDroneCAN shows.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use parking_lot::Mutex;

pub const NODESTATUS_ID: u16 = 341;
pub const NODESTATUS_SIG: u64 = 0x0f08_68d0_c1a7_c6f1;
pub const LOGMESSAGE_ID: u16 = 16383;
pub const GETNODEINFO_ID: u8 = 1;
pub const GETNODEINFO_SIG: u64 = 0xee46_8a81_21c4_6a9e;
pub const RESTARTNODE_ID: u8 = 5;
pub const RESTARTNODE_SIG: u64 = 0x569e_0539_4a3e_b2b6;
/// The node id Mission Planner uses on the bus.
pub const GCS_NODE: u8 = 127;

/// A 29-bit DroneCAN frame id, decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameId {
    Message { priority: u8, type_id: u16, source: u8 },
    Service { priority: u8, type_id: u8, request: bool, dest: u8, source: u8 },
}

impl FrameId {
    pub fn decode(id: u32) -> Self {
        let id = id & 0x1FFF_FFFF;
        let priority = (id >> 24) as u8;
        let source = (id & 0x7f) as u8;
        if id & 0x80 == 0 {
            FrameId::Message { priority, type_id: ((id >> 8) & 0xffff) as u16, source }
        } else {
            FrameId::Service { priority, type_id: ((id >> 16) & 0xff) as u8, request: id & 0x8000 != 0, dest: ((id >> 8) & 0x7f) as u8, source }
        }
    }

    pub fn encode(&self) -> u32 {
        match *self {
            FrameId::Message { priority, type_id, source } => (priority as u32 & 0x1f) << 24 | (type_id as u32) << 8 | (source as u32 & 0x7f),
            FrameId::Service { priority, type_id, request, dest, source } => (priority as u32 & 0x1f) << 24 | (type_id as u32) << 16 | (request as u32) << 15 | (dest as u32 & 0x7f) << 8 | 0x80 | (source as u32 & 0x7f),
        }
    }

    pub fn source(&self) -> u8 {
        match self {
            FrameId::Message { source, .. } | FrameId::Service { source, .. } => *source,
        }
    }
}

/// CRC-16-CCITT-FALSE over the data type signature (little endian) and then the payload.
pub fn transfer_crc(signature: u64, payload: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    let mut add = |b: u8| {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    };
    for b in signature.to_le_bytes() {
        add(b);
    }
    for &b in payload {
        add(b);
    }
    crc
}

/// Split a transfer into CAN frames (data bytes including the tail byte).
pub fn frames(payload: &[u8], signature: u64, transfer_id: u8) -> Vec<Vec<u8>> {
    let tid = transfer_id & 0x1f;
    if payload.len() <= 7 {
        let mut f = payload.to_vec();
        f.push(0xC0 | tid);
        return vec![f];
    }
    let mut data = transfer_crc(signature, payload).to_le_bytes().to_vec();
    data.extend_from_slice(payload);
    let chunks: Vec<&[u8]> = data.chunks(7).collect();
    let n = chunks.len();
    chunks
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let mut f = c.to_vec();
            let tail = (if i == 0 { 0x80 } else { 0 }) | (if i == n - 1 { 0x40 } else { 0 }) | (if i % 2 == 1 { 0x20 } else { 0 }) | tid;
            f.push(tail);
            f
        })
        .collect()
}

/// Reassembles transfers per (frame id, source).
#[derive(Default)]
pub struct Assembler {
    partial: BTreeMap<u32, Vec<u8>>,
}

impl Assembler {
    /// Feed one frame; returns the complete payload when the transfer ends.
    pub fn push(&mut self, id: u32, data: &[u8]) -> Option<Vec<u8>> {
        let (&tail, body) = data.split_last()?;
        let (start, end) = (tail & 0x80 != 0, tail & 0x40 != 0);
        if start && end {
            return Some(body.to_vec());
        }
        if start {
            self.partial.insert(id, body.to_vec());
            return None;
        }
        let buf = self.partial.get_mut(&id)?;
        buf.extend_from_slice(body);
        if end {
            let mut b = self.partial.remove(&id)?;
            // first two bytes are the transfer CRC
            return (b.len() >= 2).then(|| b.split_off(2));
        }
        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct NodeStatus {
    pub uptime: u32,
    pub health: u8,
    pub mode: u8,
    pub sub_mode: u8,
    pub vendor: u16,
}

impl NodeStatus {
    pub fn decode(p: &[u8]) -> Option<Self> {
        if p.len() < 7 {
            return None;
        }
        Some(Self { uptime: u32::from_le_bytes([p[0], p[1], p[2], p[3]]), health: p[4] >> 6, mode: (p[4] >> 3) & 7, sub_mode: p[4] & 7, vendor: u16::from_le_bytes([p[5], p[6]]) })
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut v = self.uptime.to_le_bytes().to_vec();
        v.push((self.health & 3) << 6 | (self.mode & 7) << 3 | (self.sub_mode & 7));
        v.extend(self.vendor.to_le_bytes());
        v
    }

    pub fn health_name(&self) -> &'static str {
        ["OK", "WARNING", "ERROR", "CRITICAL"][self.health as usize & 3]
    }

    pub fn mode_name(&self) -> &'static str {
        match self.mode {
            0 => "OPERATIONAL",
            1 => "INITIALIZATION",
            2 => "MAINTENANCE",
            3 => "SOFTWARE_UPDATE",
            7 => "OFFLINE",
            _ => "?",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct NodeInfo {
    pub status: NodeStatus,
    pub sw: (u8, u8, u32, u64),
    pub hw: (u8, u8),
    pub uid: [u8; 16],
    pub name: String,
}

impl NodeInfo {
    pub fn decode(p: &[u8]) -> Option<Self> {
        let status = NodeStatus::decode(p)?;
        let p = p.get(7..)?;
        if p.len() < 15 + 2 + 16 + 1 {
            return None;
        }
        let sw = (p[0], p[1], u32::from_le_bytes([p[3], p[4], p[5], p[6]]), u64::from_le_bytes(p[7..15].try_into().ok()?));
        let hw = (p[15], p[16]);
        let uid: [u8; 16] = p[17..33].try_into().ok()?;
        let coa = p[33] as usize;
        let name = String::from_utf8_lossy(p.get(34 + coa..)?).to_string();
        Some(Self { status, sw, hw, uid, name })
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut v = self.status.encode();
        v.extend([self.sw.0, self.sw.1, 0x03]);
        v.extend(self.sw.2.to_le_bytes());
        v.extend(self.sw.3.to_le_bytes());
        v.extend([self.hw.0, self.hw.1]);
        v.extend(self.uid);
        v.push(0);
        v.extend(self.name.as_bytes());
        v
    }
}

/// uavcan.protocol.debug.LogMessage: (level, source, text)
pub fn decode_log(p: &[u8]) -> Option<(u8, String, String)> {
    let b = *p.first()?;
    let (level, slen) = (b >> 5, (b & 0x1f) as usize);
    let src = String::from_utf8_lossy(p.get(1..1 + slen)?).to_string();
    let text = String::from_utf8_lossy(p.get(1 + slen..)?).to_string();
    Some((level, src, text))
}

/// The row ConfigDroneCAN shows per node.
#[derive(Debug, Clone, Default)]
pub struct Node {
    pub id: u8,
    pub status: NodeStatus,
    pub info: Option<NodeInfo>,
    pub last: Option<Instant>,
}

impl Node {
    pub fn name(&self) -> String {
        self.info.as_ref().map(|i| i.name.clone()).unwrap_or_else(|| "?".into())
    }
}

/// A CAN interface: frames out (id, data incl. tail) and frames in.
pub trait CanPort: Send {
    fn send(&mut self, id: u32, data: &[u8]);
    fn recv(&mut self, timeout: Duration) -> Option<(u32, Vec<u8>)>;
}

/// MAVLink CAN forwarding: CAN_FRAME both ways, MAV_CMD_CAN_FORWARD (32000) every second.
pub struct MavlinkCan {
    tx: crossbeam_channel::Sender<crate::link::LinkCommand>,
    rx: crossbeam_channel::Receiver<crate::mavlink::dialects::ardupilotmega::MavMessage>,
    bus: u8,
    sys: u8,
    comp: u8,
    last_forward: Option<Instant>,
}

impl MavlinkCan {
    pub fn new(link: &crate::link::Link, bus: u8) -> Self {
        let (stx, srx) = crossbeam_channel::unbounded();
        link.send(crate::link::LinkCommand::Subscribe { msgid: 386, tx: stx });
        let (sys, comp) = {
            let s = link.shared.read();
            (s.cs.sysid, s.cs.compid)
        };
        Self { tx: link.sender(), rx: srx, bus, sys, comp, last_forward: None }
    }
}

impl CanPort for MavlinkCan {
    fn send(&mut self, id: u32, data: &[u8]) {
        use crate::mavlink::dialects::ardupilotmega::*;
        let mut d = [0u8; 8];
        d[..data.len().min(8)].copy_from_slice(&data[..data.len().min(8)]);
        let _ = self.tx.send(crate::link::LinkCommand::Send(Box::new(MavMessage::CAN_FRAME(CAN_FRAME_DATA { id: id | 0x8000_0000, target_system: self.sys, target_component: self.comp, bus: self.bus - 1, len: data.len() as u8, data: d }))));
    }

    fn recv(&mut self, timeout: Duration) -> Option<(u32, Vec<u8>)> {
        if self.last_forward.map(|t| t.elapsed() >= Duration::from_secs(1)).unwrap_or(true) {
            self.last_forward = Some(Instant::now());
            let _ = self.tx.send(crate::link::LinkCommand::CommandLong { command: 32000, params: [self.bus as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] });
        }
        match self.rx.recv_timeout(timeout) {
            Ok(crate::mavlink::dialects::ardupilotmega::MavMessage::CAN_FRAME(f)) => Some((f.id & 0x1FFF_FFFF, f.data[..(f.len as usize).min(8)].to_vec())),
            _ => None,
        }
    }
}

/// SLCAN (Lawicel) on a serial port: "S8" + "O" to open, "Tiiiiiiiildd..\r" frames.
pub struct Slcan {
    port: Box<dyn crate::comms::Transport>,
    buf: Vec<u8>,
}

impl Slcan {
    pub fn open(mut port: Box<dyn crate::comms::Transport>) -> std::io::Result<Self> {
        port.write_all(b"\rC\rS8\rO\r")?;
        Ok(Self { port, buf: Vec::new() })
    }
}

pub fn slcan_line(id: u32, data: &[u8]) -> String {
    let mut s = format!("T{:08X}{:X}", id & 0x1FFF_FFFF, data.len());
    for b in data {
        s.push_str(&format!("{b:02X}"));
    }
    s.push('\r');
    s
}

pub fn parse_slcan(line: &str) -> Option<(u32, Vec<u8>)> {
    let l = line.trim();
    let rest = l.strip_prefix('T')?;
    let id = u32::from_str_radix(rest.get(0..8)?, 16).ok()?;
    let n = usize::from_str_radix(rest.get(8..9)?, 16).ok()?;
    let data = (0..n).map(|i| u8::from_str_radix(rest.get(9 + i * 2..11 + i * 2)?, 16).ok()).collect::<Option<Vec<u8>>>()?;
    Some((id, data))
}

impl CanPort for Slcan {
    fn send(&mut self, id: u32, data: &[u8]) {
        let _ = self.port.write_all(slcan_line(id, data).as_bytes());
    }

    fn recv(&mut self, timeout: Duration) -> Option<(u32, Vec<u8>)> {
        let end = Instant::now() + timeout;
        loop {
            if let Some(i) = self.buf.iter().position(|b| *b == b'\r') {
                let line: Vec<u8> = self.buf.drain(..=i).collect();
                if let Some(f) = parse_slcan(&String::from_utf8_lossy(&line)) {
                    return Some(f);
                }
                continue;
            }
            let mut b = [0u8; 256];
            match self.port.read(&mut b) {
                Ok(n) if n > 0 => self.buf.extend_from_slice(&b[..n]),
                _ => {
                    if Instant::now() >= end {
                        return None;
                    }
                }
            }
        }
    }
}

/// The DroneCAN session behind the page: node table, debug log, outgoing requests.
#[derive(Default)]
pub struct Session {
    pub nodes: Arc<Mutex<BTreeMap<u8, Node>>>,
    pub log: Arc<Mutex<Vec<(u8, u8, String, String)>>>,
    pub frames: Arc<std::sync::atomic::AtomicU64>,
    stop: Arc<AtomicBool>,
    requests: Arc<Mutex<Vec<(u8, u8, u64, Vec<u8>)>>>,
    running: Arc<AtomicBool>,
}

impl Session {
    pub fn running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    /// RestartNode with the magic number 0xACCE551B1E.
    pub fn restart(&self, node: u8) {
        self.requests.lock().push((node, RESTARTNODE_ID, RESTARTNODE_SIG, 0xAC_CE55_1B1E_u64.to_le_bytes()[..5].to_vec()));
    }

    pub fn start(&mut self, mut port: Box<dyn CanPort>) {
        self.stop.store(false, Ordering::Relaxed);
        self.running.store(true, Ordering::Relaxed);
        let (nodes, log, frames, stop, requests, running) = (self.nodes.clone(), self.log.clone(), self.frames.clone(), self.stop.clone(), self.requests.clone(), self.running.clone());
        std::thread::spawn(move || {
            let mut asm = Assembler::default();
            let mut tid: u8 = 0;
            let mut uptime = Instant::now();
            let mut last_status = Instant::now() - Duration::from_secs(2);
            let mut asked: BTreeMap<u8, Instant> = BTreeMap::new();
            while !stop.load(Ordering::Relaxed) {
                // our own NodeStatus once a second, as the GCS node
                if last_status.elapsed() >= Duration::from_secs(1) {
                    last_status = Instant::now();
                    let ns = NodeStatus { uptime: uptime.elapsed().as_secs() as u32, ..Default::default() };
                    let id = FrameId::Message { priority: 20, type_id: NODESTATUS_ID, source: GCS_NODE }.encode();
                    for f in frames_for(&ns.encode(), NODESTATUS_SIG, &mut tid) {
                        port.send(id, &f);
                    }
                }
                for (dest, sid, sig, payload) in std::mem::take(&mut *requests.lock()) {
                    let id = FrameId::Service { priority: 30, type_id: sid, request: true, dest, source: GCS_NODE }.encode();
                    for f in frames_for(&payload, sig, &mut tid) {
                        port.send(id, &f);
                    }
                }
                let Some((id, data)) = port.recv(Duration::from_millis(100)) else { continue };
                frames.fetch_add(1, Ordering::Relaxed);
                let fid = FrameId::decode(id);
                let Some(payload) = asm.push(id, &data) else { continue };
                match fid {
                    FrameId::Message { type_id: NODESTATUS_ID, source, .. } => {
                        let Some(ns) = NodeStatus::decode(&payload) else { continue };
                        let mut n = nodes.lock();
                        let node = n.entry(source).or_insert_with(|| Node { id: source, ..Default::default() });
                        node.status = ns;
                        node.last = Some(Instant::now());
                        // GetNodeInfo for nodes we do not know yet (retry every 3 s)
                        if node.info.is_none() && asked.get(&source).map(|t| t.elapsed().as_secs() >= 3).unwrap_or(true) {
                            asked.insert(source, Instant::now());
                            requests.lock().push((source, GETNODEINFO_ID, GETNODEINFO_SIG, Vec::new()));
                        }
                    }
                    FrameId::Message { type_id: LOGMESSAGE_ID, source, .. } => {
                        if let Some((lvl, src, text)) = decode_log(&payload) {
                            let mut l = log.lock();
                            l.insert(0, (source, lvl, src, text));
                            l.truncate(100);
                        }
                    }
                    FrameId::Service { type_id: GETNODEINFO_ID, request: false, dest: GCS_NODE, source, .. } => {
                        if let Some(info) = NodeInfo::decode(&payload) {
                            let mut n = nodes.lock();
                            let node = n.entry(source).or_insert_with(|| Node { id: source, ..Default::default() });
                            node.status = info.status;
                            node.info = Some(info);
                        }
                    }
                    _ => {}
                }
                let _ = &mut uptime;
            }
            running.store(false, Ordering::Relaxed);
        });
    }
}

fn frames_for(payload: &[u8], sig: u64, tid: &mut u8) -> Vec<Vec<u8>> {
    let f = frames(payload, sig, *tid);
    *tid = tid.wrapping_add(1) & 0x1f;
    f
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_round_trip() {
        let m = FrameId::Message { priority: 16, type_id: NODESTATUS_ID, source: 125 };
        assert_eq!(FrameId::decode(m.encode()), m);
        let s = FrameId::Service { priority: 30, type_id: GETNODEINFO_ID, request: true, dest: 125, source: GCS_NODE };
        assert_eq!(FrameId::decode(s.encode()), s);
    }

    #[test]
    fn node_info_multi_frame() {
        let info = NodeInfo { status: NodeStatus { uptime: 42, health: 0, mode: 0, sub_mode: 0, vendor: 7 }, sw: (1, 9, 0xabcdef, 0x1122334455667788), hw: (3, 1), uid: [5; 16], name: "org.ardupilot.ap_periph".into() };
        let enc = info.encode();
        let fr = frames(&enc, GETNODEINFO_SIG, 3);
        assert!(fr.len() > 1);
        assert_eq!(fr[0][..2], transfer_crc(GETNODEINFO_SIG, &enc).to_le_bytes());
        let mut a = Assembler::default();
        let mut out = None;
        for f in &fr {
            out = a.push(0x1234, f).or(out);
        }
        assert_eq!(NodeInfo::decode(&out.unwrap()).unwrap(), info);
        let ns = NodeStatus { uptime: 9, health: 2, mode: 2, sub_mode: 1, vendor: 0 };
        assert_eq!(NodeStatus::decode(&ns.encode()).unwrap(), ns);
        assert_eq!(ns.health_name(), "ERROR");
    }

    #[test]
    fn slcan_lines() {
        let l = slcan_line(0x1001557d, &[1, 2, 0xC0]);
        assert_eq!(l, "T1001557D30102C0\r");
        assert_eq!(parse_slcan(&l), Some((0x1001557d, vec![1, 2, 0xC0])));
        assert_eq!(decode_log(&[0x23, b'a', b'b', b'c', b'h', b'i']), Some((1, "abc".into(), "hi".into())));
    }
}
