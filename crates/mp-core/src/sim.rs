//! A small built-in simulated ArduCopter that speaks MAVLink over UDP.
//!
//! It stands in for SITL so the ground station can be exercised end to end (connect, params,
//! missions, modes, arming, guided, auto) on any platform, including mobile.

use std::collections::BTreeMap;
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::*;
use mavlink::{MavHeader, MavlinkVersion};
use num_traits::FromPrimitive;

use crate::geo::PointLatLngAlt;
use crate::mavcmd;
use crate::mission::Locationwp;

/// ArduPilot's default SITL home (Canberra Model Aircraft Club).
pub const SITL_HOME: PointLatLngAlt = PointLatLngAlt::new(-35.363261, 149.165230, 584.0);

const MODE_STABILIZE: u32 = 0;
const MODE_AUTO: u32 = 3;
const MODE_GUIDED: u32 = 4;
const MODE_LOITER: u32 = 5;
const MODE_RTL: u32 = 6;
const MODE_LAND: u32 = 9;

/// Vehicle type the built-in simulator reports (heartbeat type, banner and mode numbers).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SimKind {
    #[default]
    Copter,
    Plane,
    Rover,
    Tracker,
}

/// Options for one simulated vehicle.
#[derive(Debug, Clone, Copy)]
pub struct SimOptions {
    pub gcs_port: u16,
    pub sysid: u8,
    pub kind: SimKind,
    /// home offset from the SITL home, metres (north, east)
    pub offset: (f64, f64),
}

/// The simulator's MAVLink FTP filesystem (AP_Filesystem: SD card plus @SYS / @PARAM).
struct FtpFs {
    files: BTreeMap<String, Vec<u8>>,
    dirs: std::collections::BTreeSet<String>,
    sessions: std::collections::HashMap<u8, (String, bool)>,
    /// AP_Scripting REPL: bytes of /repl/in already executed
    repl_done: Option<usize>,
}

impl FtpFs {
    fn new() -> Self {
        let mut files = BTreeMap::new();
        files.insert("/APM/LOGS/00000001.BIN".to_string(), crate::dataflash::sample_log());
        files.insert("/APM/LOGS/LASTLOG.TXT".to_string(), b"1\r\n".to_vec());
        files.insert("/APM/scripts/hello_world.lua".to_string(), b"-- example script\nfunction update()\n  gcs:send_text(6, \"hello, world\")\n  return update, 10000\nend\nreturn update()\n".to_vec());
        files.insert("/APM/terrain/S36E149.DAT".to_string(), vec![0u8; 2048]);
        files.insert("@SYS/uarts.txt".to_string(), b"UARTV1\nSERIAL0 OTG1  TX=   12345 RX=     678 TXBD=  9600 RXBD=  1200\nSERIAL1 UART7 TX*       0 RX*       0 TXBD=     0 RXBD=     0\n".to_vec());
        files.insert("@SYS/threads.txt".to_string(), b"ThreadsV2\nmonitor      PRI=183 sp=0x30008000 STACK_LEFT=264\ntimer        PRI=181 sp=0x30008400 STACK_LEFT=1016\nrcin         PRI=177 sp=0x30008800 STACK_LEFT=1176\nIO           PRI= 58 sp=0x30009000 STACK_LEFT=1384\nmain         PRI=180 sp=0x2001E800 STACK_LEFT=3152\n".to_vec());
        files.insert("@SYS/tasks.txt".to_string(), b"TasksV2\nfast_loop            MIN=  12 MAX=  55 AVG=  14 OVR=  0 SLP=  0, TOT= 3.2%\n".to_vec());
        files.insert("@SYS/dma.txt".to_string(), b"DMAV1\n".to_vec());
        files.insert("@SYS/timers.txt".to_string(), b"TimersV1\n".to_vec());
        let dirs = ["/", "/APM", "/APM/LOGS", "/APM/scripts", "/APM/terrain", "@SYS", "@PARAM", "@ROMFS", "@MISSION"].iter().map(|s| s.to_string()).collect();
        Self { files, dirs, sessions: Default::default(), repl_done: None }
    }

    fn norm(p: &str) -> String {
        let p = p.trim_end_matches('\0').trim();
        // AP_Filesystem_Param query string ("?withdefaults=1")
        let p = p.split('?').next().unwrap_or(p);
        if p.is_empty() {
            return "/".into();
        }
        let p = if p.starts_with('/') || p.starts_with('@') { p.to_string() } else { format!("/{p}") };
        if p.len() > 1 { p.trim_end_matches('/').to_string() } else { p }
    }

    fn list(&self, dir: &str) -> Vec<String> {
        let dir = Self::norm(dir);
        let prefix = if dir == "/" { "/".to_string() } else { format!("{dir}/") };
        let mut out = Vec::new();
        for d in &self.dirs {
            if d != &dir && d.starts_with(&prefix) && !d[prefix.len()..].contains('/') {
                out.push(format!("D{}", &d[prefix.len()..]));
            }
        }
        if dir == "/" {
            for d in self.dirs.iter().filter(|d| d.starts_with('@')) {
                out.push(format!("D{d}"));
            }
        }
        for (f, b) in &self.files {
            if f.starts_with(&prefix) && !f[prefix.len()..].contains('/') {
                out.push(format!("F{}\t{}", &f[prefix.len()..], b.len()));
            }
        }
        out
    }

    /// MAV_CMD_SCRIPTING REPL_START / REPL_STOP: the REPL reads /repl/in and writes /repl/out.
    fn repl(&mut self, start: bool) {
        if start {
            self.dirs.insert("/repl".into());
            self.files.insert("/repl/out".into(), b"> ".to_vec());
            self.files.remove("/repl/in");
            self.repl_done = Some(0);
        } else {
            self.repl_done = None;
            self.files.remove("/repl/in");
            self.files.remove("/repl/out");
            self.dirs.remove("/repl");
        }
    }

    /// Run any complete new lines written to /repl/in (a tiny Lua subset).
    fn repl_step(&mut self) {
        let Some(done) = self.repl_done else { return };
        let input = self.files.get("/repl/in").cloned().unwrap_or_default();
        if input.len() <= done {
            return;
        }
        let Some(end) = input[done..].iter().rposition(|b| *b == b'\n').map(|i| done + i + 1) else { return };
        let mut out = String::new();
        for line in String::from_utf8_lossy(&input[done..end]).lines() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            let expr = line.strip_prefix("print(").and_then(|l| l.strip_suffix(')')).or_else(|| line.strip_prefix("return ")).unwrap_or(line);
            out.push_str(&repl_eval(expr));
            out.push_str("\n> ");
        }
        self.files.entry("/repl/out".into()).or_default().extend_from_slice(out.as_bytes());
        self.repl_done = Some(end);
    }

    fn writing(&self, path: &str) -> bool {
        self.sessions.values().any(|(p, w)| p == path && *w)
    }

    /// Handle one request payload; returns the reply payload.
    fn handle(&mut self, req: &[u8; 251], params: &BTreeMap<String, f32>, defaults: &BTreeMap<String, f32>) -> [u8; 251] {
        let seq = u16::from_le_bytes([req[0], req[1]]);
        let (session, op, size) = (req[2], req[3], req[4] as usize);
        let offset = u32::from_le_bytes([req[8], req[9], req[10], req[11]]);
        let data = &req[12..12 + size.min(239)];
        let text = String::from_utf8_lossy(data).to_string();
        let mut rep = [0u8; 251];
        rep[0..2].copy_from_slice(&seq.wrapping_add(1).to_le_bytes());
        rep[2] = session;
        rep[5] = op;
        rep[8..12].copy_from_slice(&offset.to_le_bytes());
        let ack = |rep: &mut [u8; 251], payload: &[u8]| {
            rep[3] = 128;
            rep[4] = payload.len() as u8;
            rep[12..12 + payload.len()].copy_from_slice(payload);
        };
        let nak = |rep: &mut [u8; 251], code: u8| {
            rep[3] = 129;
            rep[4] = 1;
            rep[12] = code;
        };
        match op {
            1 => {
                self.sessions.remove(&session);
                ack(&mut rep, &[]);
            }
            2 => {
                self.sessions.clear();
                ack(&mut rep, &[]);
            }
            3 => {
                let dir = Self::norm(&text);
                if !self.dirs.contains(&dir) {
                    nak(&mut rep, 10);
                } else {
                    let entries = self.list(&dir);
                    let mut buf = Vec::new();
                    for e in entries.iter().skip(offset as usize) {
                        if buf.len() + e.len() + 1 > 239 {
                            break;
                        }
                        buf.extend_from_slice(e.as_bytes());
                        buf.push(0);
                    }
                    if buf.is_empty() { nak(&mut rep, 6) } else { ack(&mut rep, &buf) }
                }
            }
            4 | 6 | 11 => {
                let path = Self::norm(&text);
                if path == "@PARAM/param.pck" && op == 4 {
                    let list: Vec<(String, f32, f32)> = params.iter().map(|(k, v)| (k.clone(), *v, defaults.get(k).copied().unwrap_or(*v))).collect();
                    self.files.insert(path.clone(), crate::parampck::pack(&list));
                }
                if op == 6 {
                    self.files.insert(path.clone(), Vec::new());
                }
                match self.files.get(&path) {
                    Some(b) => {
                        let sid = (0u8..4).find(|i| !self.sessions.contains_key(i)).unwrap_or(0);
                        self.sessions.insert(sid, (path.clone(), op != 4));
                        rep[2] = sid;
                        ack(&mut rep, &(b.len() as u32).to_le_bytes());
                    }
                    None => nak(&mut rep, 10),
                }
            }
            5 | 15 => match self.sessions.get(&session).and_then(|(p, _)| self.files.get(p)) {
                Some(b) if (offset as usize) < b.len() => {
                    let end = (offset as usize + size.clamp(1, 239)).min(b.len());
                    let chunk = b[offset as usize..end].to_vec();
                    ack(&mut rep, &chunk);
                }
                Some(_) => nak(&mut rep, 6),
                None => nak(&mut rep, 4),
            },
            7 => match self.sessions.get(&session).cloned() {
                Some((p, true)) => {
                    let f = self.files.entry(p).or_default();
                    let end = offset as usize + data.len();
                    if f.len() < end {
                        f.resize(end, 0);
                    }
                    f[offset as usize..end].copy_from_slice(data);
                    ack(&mut rep, &[]);
                }
                _ => nak(&mut rep, 4),
            },
            8 => {
                if self.files.remove(&Self::norm(&text)).is_some() { ack(&mut rep, &[]) } else { nak(&mut rep, 10) }
            }
            9 => {
                let p = Self::norm(&text);
                if self.dirs.insert(p) { ack(&mut rep, &[]) } else { nak(&mut rep, 8) }
            }
            10 => {
                let p = Self::norm(&text);
                let busy = self.files.keys().any(|f| f.starts_with(&format!("{p}/")));
                if busy { nak(&mut rep, 1) } else if self.dirs.remove(&p) { ack(&mut rep, &[]) } else { nak(&mut rep, 10) }
            }
            13 => {
                let mut parts = data.split(|b| *b == 0);
                let a = Self::norm(&String::from_utf8_lossy(parts.next().unwrap_or(&[])));
                let b = Self::norm(&String::from_utf8_lossy(parts.next().unwrap_or(&[])));
                match self.files.remove(&a) {
                    Some(v) => {
                        self.files.insert(b, v);
                        ack(&mut rep, &[]);
                    }
                    None => nak(&mut rep, 10),
                }
            }
            14 => match self.files.get(&Self::norm(&text)) {
                Some(b) => ack(&mut rep, &crate::ftp::crc32(0, b).to_le_bytes()),
                None => nak(&mut rep, 10),
            },
            _ => nak(&mut rep, 7),
        }
        rep
    }
}

/// Evaluate a REPL expression: string literals, numbers and + - * / arithmetic.
fn repl_eval(expr: &str) -> String {
    let e = expr.trim();
    if (e.starts_with('"') && e.ends_with('"') && e.len() >= 2) || (e.starts_with('\'') && e.ends_with('\'') && e.len() >= 2) {
        return e[1..e.len() - 1].to_string();
    }
    fn num(s: &str) -> Option<f64> {
        let s = s.trim();
        for op in ['+', '-'] {
            if let Some(i) = s.rfind(op).filter(|i| *i > 0) {
                let (a, b) = (num(&s[..i])?, num(&s[i + 1..])?);
                return Some(if op == '+' { a + b } else { a - b });
            }
        }
        for op in ['*', '/'] {
            if let Some(i) = s.rfind(op).filter(|i| *i > 0) {
                let (a, b) = (num(&s[..i])?, num(&s[i + 1..])?);
                return Some(if op == '*' { a * b } else { a / b });
            }
        }
        s.trim_matches(|c| c == '(' || c == ')').parse().ok()
    }
    match num(e) {
        Some(v) if v.fract() == 0.0 && v.abs() < 1e15 => format!("{}", v as i64),
        Some(v) => format!("{v}"),
        None => format!("[string \"{e}\"]:1: attempt to call a nil value"),
    }
}

impl Vehicle {
    fn list_ref(&self, t: u8) -> &Vec<Locationwp> {
        match t {
            1 => &self.fence,
            2 => &self.rally,
            _ => &self.mission,
        }
    }
}

pub struct SimHandle {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl SimHandle {
    pub fn stop(mut self) {
        self.shutdown();
    }
    fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for SimHandle {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Start a simulated vehicle that sends to `127.0.0.1:gcs_port` (like SITL's `--out udp:...:14550`).
pub fn spawn(gcs_port: u16) -> io::Result<SimHandle> {
    spawn_with(SimOptions { gcs_port, sysid: 1, kind: SimKind::Copter, offset: (0.0, 0.0) })
}

/// Start a simulated vehicle with its own system id, type and home position (swarms).
pub fn spawn_with(o: SimOptions) -> io::Result<SimHandle> {
    let gcs_port = o.gcs_port;
    let sock = UdpSocket::bind("127.0.0.1:0")?;
    sock.set_read_timeout(Some(Duration::from_millis(10)))?;
    let gcs: SocketAddr = format!("127.0.0.1:{gcs_port}")
        .parse()
        .map_err(io::Error::other)?;
    let stop = Arc::new(AtomicBool::new(false));
    let st = stop.clone();
    let thread = std::thread::Builder::new()
        .name("sim".into())
        .spawn(move || {
            let mut v = Vehicle::new(sock, gcs);
            v.configure(o);
            v.run(&st);
        })?;
    Ok(SimHandle {
        stop,
        thread: Some(thread),
    })
}

struct Vehicle {
    ftp: FtpFs,
    sysid: u8,
    kind: SimKind,
    home: PointLatLngAlt,
    sock: UdpSocket,
    gcs: SocketAddr,
    seq: u8,
    boot: Instant,
    params: BTreeMap<String, f32>,
    /// values at boot: the firmware defaults reported in param.pck
    param_defaults: BTreeMap<String, f32>,
    mission: Vec<Locationwp>,
    upload: Option<(u16, Vec<Locationwp>, MavMissionType)>,
    fence: Vec<Locationwp>,
    accel_step: u8,
    /// MAV_CMD_CAN_FORWARD: bus and when it was last requested (forwarding stops after 5 s)
    can_forward: Option<(u8, Instant)>,
    /// the simulated AP_Periph node on CAN1: boot time and transfer id
    periph: (Instant, u8),
    can_asm: crate::dronecan::Assembler,
    magcal: Option<u8>,
    param_queue: std::collections::VecDeque<usize>,
    rally: Vec<Locationwp>,
    armed: bool,
    mode: u32,
    pos: PointLatLngAlt, // alt = relative metres
    yaw: f32,
    roll: f32,
    pitch: f32,
    gs: f32,
    climb: f32,
    target: Option<PointLatLngAlt>,
    current_wp: u16,
    battery: f32,
    log: Vec<String>,
}

impl Vehicle {
    fn new(sock: UdpSocket, gcs: SocketAddr) -> Self {
        let mut params = BTreeMap::new();
        for (n, v) in [
            ("SYSID_THISMAV", 1.0),
            ("FRAME_CLASS", 1.0),
            ("FRAME_TYPE", 1.0),
            ("ANGLE_MAX", 4500.0),
            ("WPNAV_SPEED", 1000.0),
            ("WPNAV_SPEED_UP", 250.0),
            ("WPNAV_SPEED_DN", 150.0),
            ("WPNAV_RADIUS", 200.0),
            ("RTL_ALT", 1500.0),
            ("PILOT_SPEED_UP", 250.0),
            ("FLTMODE_CH", 5.0),
            ("FLTMODE1", 0.0),
            ("FLTMODE2", 2.0),
            ("FLTMODE3", 5.0),
            ("FLTMODE4", 3.0),
            ("FLTMODE5", 6.0),
            ("FLTMODE6", 9.0),
            ("ATC_RAT_RLL_P", 0.135),
            ("ATC_RAT_RLL_I", 0.135),
            ("ATC_RAT_RLL_D", 0.0036),
            ("ATC_RAT_PIT_P", 0.135),
            ("ATC_RAT_PIT_I", 0.135),
            ("ATC_RAT_PIT_D", 0.0036),
            ("ATC_RAT_YAW_P", 0.18),
            ("ATC_RAT_YAW_I", 0.018),
            ("BATT_MONITOR", 4.0),
            ("BATT_CAPACITY", 3300.0),
            ("FENCE_ENABLE", 0.0),
            ("ARMING_CHECK", 1.0),
            ("SERIAL0_BAUD", 115.0),
            ("SERIAL1_PROTOCOL", 2.0),
            ("RC1_MIN", 1100.0),
            ("RC1_MAX", 1900.0),
            ("RC1_TRIM", 1500.0),
            ("RC3_MIN", 1100.0),
            ("RC3_MAX", 1900.0),
            ("COMPASS_USE", 1.0),
            ("COMPASS_DEV_ID", 97539.0),
            ("COMPASS_PRIO1_ID", 97539.0),
            ("COMPASS_DEV_ID2", 131874.0),
            ("COMPASS_PRIO2_ID", 131874.0),
            ("COMPASS_DEV_ID3", 263178.0),
            ("COMPASS_PRIO3_ID", 263178.0),
            ("COMPASS_EXTERN2", 0.0),
            ("COMPASS_EXTERN3", 0.0),
            ("INS_ACC2_ID", 2753036.0),
            ("INS_GYR2_ID", 2752780.0),
            ("BARO2_DEVID", 65796.0),
            ("COMPASS_EXTERNAL", 1.0),
            ("COMPASS_ORIENT", 0.0),
            ("INS_ACC_ID", 2753028.0),
            ("INS_GYR_ID", 2752772.0),
            ("BARO1_DEVID", 65540.0),
            ("GPS_TYPE", 1.0),
        ] {
            params.insert(n.to_string(), v);
        }
        for i in 1..=16 {
            for (suffix, v) in [("MIN", 1100.0), ("TRIM", 1500.0), ("MAX", 1900.0), ("REVERSED", 0.0)] {
                params.entry(format!("SERVO{i}_{suffix}")).or_insert(v);
                params.entry(format!("RC{i}_{suffix}")).or_insert(v);
            }
            params.entry(format!("RC{i}_OPTION")).or_insert(0.0);
            params.entry(format!("RC{i}_DZ")).or_insert(0.0);
        }
        // real firmware defaults first, then the metadata fallback for everything else
        for line in include_str!("../assets/copter_defaults.parm").lines() {
            if let Some((k, v)) = line.split_once(',') {
                if let Ok(v) = v.trim().parse::<f32>() {
                    params.entry(k.trim().to_string()).or_insert(v);
                }
            }
        }
        // AP_OSD_Screen defaults the metadata does not carry
        for i in 1..=4 {
            params.entry(format!("OSD{i}_CHAN_MIN")).or_insert(900.0);
            params.entry(format!("OSD{i}_CHAN_MAX")).or_insert(2100.0);
        }
        // outputs the defaults file does not assign are disabled
        for i in 1..=16 {
            params.entry(format!("SERVO{i}_FUNCTION")).or_insert(0.0);
        }
        // parameters removed from Copter 4.x that the metadata still documents
        const LEGACY: &[&str] = &[
            "ACCEL_Z_", "THR_ACCEL_", "THR_ALT_", "THR_RATE_", "HLD_LAT_", "LOITER_LAT_", "RATE_RLL_", "RATE_PIT_", "RATE_YAW_", "STB_", "VEL_XY_", "POS_XY_", "POS_Z_", "VEL_Z_", "THR_MID", "CH6_", "CH7_", "CH8_", "CH9_", "CH10_", "CH11_", "CH12_", "TUNE_HIGH", "TUNE_LOW", "INS_PRODUCT_ID", "ARMING_REQUIRE", "LOIT_SPEED_MS", "WP_SPD", "WP_RADIUS_M", "PSC_NE_", "PSC_D_",
        ];
        for (k, v) in crate::param_meta::all_defaults(crate::firmware::Firmware::ArduCopter2) {
            if LEGACY.iter().any(|p| k.starts_with(p)) {
                continue;
            }
            params.entry(k).or_insert(v);
        }
        Self {
            ftp: FtpFs::new(),
            param_defaults: params.clone(),
            sysid: 1,
            kind: SimKind::Copter,
            home: SITL_HOME,
            sock,
            gcs,
            seq: 0,
            boot: Instant::now(),
            params,
            mission: vec![Locationwp {
                lat: SITL_HOME.lat,
                lng: SITL_HOME.lng,
                alt: SITL_HOME.alt as f32,
                id: 16,
                ..Default::default()
            }],
            upload: None,
            fence: Vec::new(),
            accel_step: 0,
            can_forward: None,
            periph: (Instant::now(), 0),
            can_asm: Default::default(),
            magcal: None,
            param_queue: Default::default(),
            rally: Vec::new(),
            armed: false,
            mode: MODE_STABILIZE,
            pos: PointLatLngAlt::new(SITL_HOME.lat, SITL_HOME.lng, 0.0),
            yaw: 0.0,
            roll: 0.0,
            pitch: 0.0,
            gs: 0.0,
            climb: 0.0,
            target: None,
            current_wp: 0,
            battery: 100.0,
            log: Vec::new(),
        }
    }

    fn send(&mut self, msg: MavMessage) {
        let hdr = MavHeader {
            system_id: self.sysid as u32,
            component_id: 1,
            sequence: self.seq,
        };
        self.seq = self.seq.wrapping_add(1);
        let mut buf = Vec::with_capacity(300);
        if mavlink::write_versioned_msg(&mut buf, MavlinkVersion::V2, hdr, &msg).is_ok() {
            let _ = self.sock.send_to(&buf, self.gcs);
        }
    }

    fn text(&mut self, sev: MavSeverity, s: &str) {
        self.log.push(s.to_string());
        self.send(MavMessage::STATUSTEXT(STATUSTEXT_DATA {
            severity: sev,
            text: s.into(),
            id: 0,
            chunk_seq: 0,
        }));
    }

    fn configure(&mut self, o: SimOptions) {
        self.sysid = o.sysid;
        self.kind = o.kind;
        let firmware = match o.kind {
            SimKind::Copter => crate::firmware::Firmware::ArduCopter2,
            SimKind::Plane => crate::firmware::Firmware::ArduPlane,
            SimKind::Rover => crate::firmware::Firmware::ArduRover,
            SimKind::Tracker => crate::firmware::Firmware::ArduTracker,
        };
        if o.kind != SimKind::Copter { for (name, value) in crate::param_meta::all_defaults(firmware) { self.params.entry(name).or_insert(value); } }
        self.param_defaults = self.params.clone();
        let lat = SITL_HOME.lat + (o.offset.0 / 6_378_137.0).to_degrees();
        let lng = SITL_HOME.lng + (o.offset.1 / (6_378_137.0 * SITL_HOME.lat.to_radians().cos())).to_degrees();
        self.home = PointLatLngAlt::new(lat, lng, SITL_HOME.alt);
        self.pos = PointLatLngAlt::new(lat, lng, 0.0);
        if let Some(h) = self.mission.first_mut() {
            h.lat = lat;
            h.lng = lng;
        }
        self.params.insert("SYSID_THISMAV".into(), o.sysid as f32);
    }

    /// Internal (copter) mode number as this vehicle type reports it.
    fn wire_mode(&self) -> u32 {
        match (self.kind, self.mode) {
            (SimKind::Copter, m) => m,
            (SimKind::Tracker, m) => m,
            (SimKind::Plane, MODE_AUTO) => 10,
            (SimKind::Plane, MODE_GUIDED) => 15,
            (SimKind::Plane, MODE_LOITER) => 12,
            (SimKind::Plane, MODE_RTL | MODE_LAND) => 11,
            (SimKind::Plane, _) => 2,
            (SimKind::Rover, MODE_AUTO) => 10,
            (SimKind::Rover, MODE_GUIDED) => 15,
            (SimKind::Rover, MODE_LOITER) => 5,
            (SimKind::Rover, MODE_RTL) => 11,
            (SimKind::Rover, MODE_LAND) => 4,
            (SimKind::Rover, _) => 0,
        }
    }

    fn from_wire_mode(&self, m: u32) -> u32 {
        if self.kind == SimKind::Tracker { return m; }
        match (self.kind, m) {
            (SimKind::Copter, m) => m,
            (_, 10) => MODE_AUTO,
            (_, 15) => MODE_GUIDED,
            (SimKind::Plane, 12) | (SimKind::Rover, 5) => MODE_LOITER,
            (_, 11) => MODE_RTL,
            (SimKind::Rover, 4) => MODE_LAND,
            _ => MODE_STABILIZE,
        }
    }

    /// GCS_MAVLINK::send_banner: firmware, OS, board id and frame.
    fn banner(&mut self) {
        let fw = match self.kind {
            SimKind::Copter => "ArduCopter V4.6.0 (rust-sim)",
            SimKind::Plane => "ArduPlane V4.6.0 (rust-sim)",
            SimKind::Rover => "ArduRover V4.6.0 (rust-sim)",
            SimKind::Tracker => "AntennaTracker V4.6.0 (rust-sim)",
        };
        self.text(MavSeverity::MAV_SEVERITY_INFO, fw);
        self.text(MavSeverity::MAV_SEVERITY_INFO, "ChibiOS: rust-sim");
        self.text(MavSeverity::MAV_SEVERITY_INFO, "SITL 0042001F 3331510A 34363732");
        self.text(MavSeverity::MAV_SEVERITY_INFO, "Frame: QUAD/X");
    }

    fn run(&mut self, stop: &AtomicBool) {
        let mut buf = vec![0u8; 2048];
        let mut last_fast = Instant::now();
        let mut last_slow = Instant::now();
        self.banner();
        while !stop.load(Ordering::Relaxed) {
            if let Ok((n, _)) = self.sock.recv_from(&mut buf) {
                let mut r = mavlink::MavlinkReader::new(&buf[..n]);
                while let Ok((_, msg)) = r.read_any_message::<MavMessage>() {
                    self.handle(msg);
                }
            }
            if !self.param_queue.is_empty() {
                let count = self.params.len() as u16;
                let names: Vec<(String, f32)> = self.params.iter().map(|(k, v)| (k.clone(), *v)).collect();
                for _ in 0..25 {
                    let Some(i) = self.param_queue.pop_front() else { break };
                    if let Some((k, v)) = names.get(i).cloned() {
                        self.send_param(&k, v, i as u16, count);
                    }
                }
            }
            let now = Instant::now();
            let dt = now.duration_since(last_fast).as_secs_f32();
            if dt >= 0.1 {
                last_fast = now;
                self.physics(dt);
                self.fast_telemetry();
            }
            if now.duration_since(last_slow) >= Duration::from_secs(1) {
                last_slow = now;
                self.slow_telemetry();
            }
        }
    }

    fn handle(&mut self, msg: MavMessage) {
        // ignore traffic addressed to another vehicle of a swarm
        if let Some(t) = <MavMessage as mavlink::Message>::target_system_id(&msg) {
            if t != 0 && t != self.sysid {
                return;
            }
        }
        match msg {
            MavMessage::PARAM_REQUEST_LIST(_) => {
                // ArduPilot sends its banner on a param list request
                self.banner();
                // stream the list over time like a real autopilot
                self.param_queue = (0..self.params.len()).collect();
            }
            MavMessage::PARAM_REQUEST_READ(r) => {
                let count = self.params.len() as u16;
                let entry = if r.param_index >= 0 {
                    self.params
                        .iter()
                        .nth(r.param_index as usize)
                        .map(|(k, v)| (k.clone(), *v, r.param_index as u16))
                } else {
                    let name = name16(&r.param_id);
                    self.params
                        .iter()
                        .position(|(k, _)| *k == name)
                        .map(|i| (name.clone(), self.params[&name], i as u16))
                };
                if let Some((k, v, i)) = entry {
                    self.send_param(&k, v, i, count);
                }
            }
            MavMessage::PARAM_SET(p) => {
                let name = name16(&p.param_id);
                if let Some(idx) = self.params.keys().position(|k| *k == name) {
                    self.params.insert(name.clone(), p.param_value);
                    let count = self.params.len() as u16;
                    self.send_param(&name, p.param_value, idx as u16, count);
                }
            }
            MavMessage::MISSION_REQUEST_LIST(r) => {
                let count = self.list(r.mission_type).len() as u16;
                self.send(MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
                    count,
                    target_system: 255,
                    target_component: 190,
                    mission_type: r.mission_type,
                    opaque_id: 0,
                }));
            }
            MavMessage::MISSION_REQUEST_INT(r) => {
                if let Some(wp) = self.list(r.mission_type).get(r.seq as usize).copied() {
                    if let Some(item) = wp.to_mission_item_int(r.seq, 255, 190, r.mission_type) {
                        self.send(MavMessage::MISSION_ITEM_INT(item));
                    }
                }
            }
            MavMessage::MISSION_COUNT(c) => {
                if c.mission_type == MavMissionType::MAV_MISSION_TYPE_ALL {
                    self.mission_ack(c.mission_type, MavMissionResult::MAV_MISSION_UNSUPPORTED);
                } else if c.count == 0 {
                    match c.mission_type {
                        MavMissionType::MAV_MISSION_TYPE_FENCE => self.fence.clear(),
                        MavMissionType::MAV_MISSION_TYPE_RALLY => self.rally.clear(),
                        _ => self.mission.truncate(1),
                    }
                    self.mission_ack(c.mission_type, MavMissionResult::MAV_MISSION_ACCEPTED);
                } else {
                    self.upload = Some((c.count, Vec::new(), c.mission_type));
                    self.request_upload_item(0, c.mission_type);
                }
            }
            MavMessage::MISSION_ITEM_INT(m) => {
                if m.current == 2 {
                    // guided "fly to here"
                    let wp = Locationwp::from_mission_item_int(&m);
                    self.target = Some(PointLatLngAlt::new(wp.lat, wp.lng, wp.alt as f64));
                    self.mission_ack(m.mission_type, MavMissionResult::MAV_MISSION_ACCEPTED);
                    return;
                }
                let mut done = None;
                let mut next = None;
                if let Some((count, items, _)) = self.upload.as_mut() {
                    if m.seq as usize == items.len() {
                        items.push(Locationwp::from_mission_item_int(&m));
                        if items.len() == *count as usize {
                            done = Some(std::mem::take(items));
                        } else {
                            next = Some(items.len() as u16);
                        }
                    }
                }
                if let Some(n) = next {
                    self.request_upload_item(n, m.mission_type);
                }
                if let Some(items) = done {
                    self.upload = None;
                    match m.mission_type {
                        MavMissionType::MAV_MISSION_TYPE_FENCE => self.fence = items,
                        MavMissionType::MAV_MISSION_TYPE_RALLY => self.rally = items,
                        _ => self.mission = items,
                    }
                    self.mission_ack(m.mission_type, MavMissionResult::MAV_MISSION_ACCEPTED);
                    let n = self.list(m.mission_type).len();
                    self.text(
                        MavSeverity::MAV_SEVERITY_INFO,
                        &format!("Mission: {n} items received"),
                    );
                }
            }
            MavMessage::MISSION_SET_CURRENT(s) => {
                self.current_wp = s.seq;
                self.target = None;
            }
            MavMessage::SET_MODE(s) => {
                let m = self.from_wire_mode(s.custom_mode);
                self.set_mode(m)
            }
            MavMessage::COMMAND_LONG(c) => self.command(c),
            MavMessage::FILE_TRANSFER_PROTOCOL(f) => {
                // AP_Filesystem_Mission: the lists as packed files, read back once a write closes
                for t in 0u8..3 {
                    let p = crate::missionpck::path(t);
                    if !self.ftp.writing(p) {
                        let packed = crate::missionpck::pack(self.list_ref(t), t, 0);
                        self.ftp.files.insert(p.to_string(), packed);
                    }
                }
                let rep = self.ftp.handle(&f.payload, &self.params, &self.param_defaults);
                self.ftp.repl_step();
                for t in 0u8..3 {
                    let p = crate::missionpck::path(t);
                    if self.ftp.writing(p) {
                        continue;
                    }
                    if let Some(b) = self.ftp.files.get(p) {
                        if *b != crate::missionpck::pack(self.list_ref(t), t, 0) {
                            if let Ok((items, _, _)) = crate::missionpck::unpack(b) {
                                match t {
                                    1 => self.fence = items,
                                    2 => self.rally = items,
                                    _ => self.mission = items,
                                }
                            }
                        }
                    }
                }
                self.send(MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA { target_network: 0, target_system: 255, target_component: 190, payload: rep }));
            }
            MavMessage::CAN_FRAME(f) => self.periph_frame(f.id & 0x1FFF_FFFF, &f.data[..(f.len as usize).min(8)]),
            MavMessage::LOG_REQUEST_LIST(_) => {
                // two onboard DataFlash logs
                let size = crate::dataflash::sample_log().len() as u32;
                for (id, size) in [(1u16, size), (2, size)] {
                    self.send(MavMessage::LOG_ENTRY(LOG_ENTRY_DATA { time_utc: 1_700_000_000 + id as u32 * 600, size, id, num_logs: 2, last_log_num: 2 }));
                }
            }
            MavMessage::LOG_REQUEST_DATA(r) => {
                let log = crate::dataflash::sample_log();
                let size = log.len() as u32;
                let end = r.ofs.saturating_add(r.count).min(size);
                let mut ofs = r.ofs;
                while ofs < end {
                    let n = (end - ofs).min(90);
                    let mut data = [0u8; 90];
                    data[..n as usize].copy_from_slice(&log[ofs as usize..(ofs + n) as usize]);
                    self.send(MavMessage::LOG_DATA(LOG_DATA_DATA { ofs, id: r.id, count: n as u8, data }));
                    ofs += n;
                }
            }
            _ => {}
        }
    }

    fn list(&self, mt: MavMissionType) -> &Vec<Locationwp> {
        match mt {
            MavMissionType::MAV_MISSION_TYPE_FENCE => &self.fence,
            MavMissionType::MAV_MISSION_TYPE_RALLY => &self.rally,
            _ => &self.mission,
        }
    }

    fn request_upload_item(&mut self, seq: u16, mt: MavMissionType) {
        self.send(MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
            seq,
            target_system: 255,
            target_component: 190,
            mission_type: mt,
        }));
    }

    fn mission_ack(&mut self, mt: MavMissionType, r: MavMissionResult) {
        self.send(MavMessage::MISSION_ACK(MISSION_ACK_DATA {
            target_system: 255,
            target_component: 190,
            mavtype: r,
            mission_type: mt,
            opaque_id: 0,
        }));
    }

    fn send_param(&mut self, name: &str, value: f32, index: u16, count: u16) {
        self.send(MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_value: value,
            param_count: count,
            param_index: index,
            param_id: name.into(),
            param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
        }));
    }

    fn set_mode(&mut self, mode: u32) {
        self.mode = mode;
        self.target = None;
        if mode == MODE_AUTO {
            self.current_wp = 1;
        }
        if mode == MODE_RTL {
            self.target = Some(PointLatLngAlt::new(
                self.home.lat,
                self.home.lng,
                self.params["RTL_ALT"] as f64 / 100.0,
            ));
        }
    }

    fn ack(&mut self, command: MavCmd, result: MavResult) {
        self.send(MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command,
            result,
            progress: 0,
            result_param2: 0,
            target_system: 255,
            target_component: 190,
        }));
    }

    fn command(&mut self, c: COMMAND_LONG_DATA) {
        use MavCmd::*;
        let res = match c.command {
            MAV_CMD_COMPONENT_ARM_DISARM => {
                let arm = c.param1 >= 0.5;
                if !arm && self.pos.alt > 0.5 && c.param2 != 21196.0 {
                    self.text(
                        MavSeverity::MAV_SEVERITY_CRITICAL,
                        "Disarm: vehicle is flying",
                    );
                    MavResult::MAV_RESULT_DENIED
                } else {
                    self.armed = arm;
                    self.text(
                        MavSeverity::MAV_SEVERITY_INFO,
                        if arm {
                            "Arming motors"
                        } else {
                            "Disarming motors"
                        },
                    );
                    MavResult::MAV_RESULT_ACCEPTED
                }
            }
            MAV_CMD_DO_SET_MODE => {
                let m = self.from_wire_mode(c.param2 as u32);
                self.set_mode(m);
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_NAV_TAKEOFF => {
                if !self.armed || self.mode != MODE_GUIDED {
                    MavResult::MAV_RESULT_FAILED
                } else {
                    self.target = Some(PointLatLngAlt::new(
                        self.pos.lat,
                        self.pos.lng,
                        c.param7 as f64,
                    ));
                    MavResult::MAV_RESULT_ACCEPTED
                }
            }
            MAV_CMD_NAV_RETURN_TO_LAUNCH => {
                self.set_mode(MODE_RTL);
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_NAV_LAND => {
                self.set_mode(MODE_LAND);
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_MISSION_START => {
                self.set_mode(MODE_AUTO);
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_REQUEST_MESSAGE => {
                match c.param1 as u32 {
                    242 => self.home_position(),
                    148 => self.send(MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
                        flight_sw_version: (4 << 24) | (6 << 16),
                        // ArduPilot: MISSION_FLOAT | PARAM_FLOAT | MISSION_INT | COMMAND_INT | FTP | SET_ATTITUDE_TARGET | ... | MAVLINK2 | MISSION_FENCE | MISSION_RALLY
                        capabilities: MavProtocolCapability::from_bits_truncate(0xFFEF),
                        ..Default::default()
                    })),
                    _ => {}
                }
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_PREFLIGHT_CALIBRATION if c.param5 == 1.0 => {
                // full 6-position accel cal: ask for each position in turn
                self.accel_step = 1;
                self.ack(c.command, MavResult::MAV_RESULT_ACCEPTED);
                self.ask_accel_pos();
                return;
            }
            MAV_CMD_FLASH_BOOTLOADER if c.param5 == 290876.0 => {
                self.text(MavSeverity::MAV_SEVERITY_INFO, "Flash bootloader: done");
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_CAN_FORWARD => {
                let first = self.can_forward.is_none();
                self.can_forward = (c.param1 >= 1.0).then(|| (c.param1 as u8, Instant::now()));
                if first && self.can_forward.is_some() {
                    self.periph_log("AP_Periph started");
                }
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_SCRIPTING => {
                // MAV_SCRIPTING_CMD_REPL_START = 0, REPL_STOP = 1
                self.ftp.repl(c.param1 < 0.5);
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_DO_START_MAG_CAL => {
                self.magcal = Some(0);
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_DO_CANCEL_MAG_CAL => {
                self.magcal = None;
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAV_CMD_DO_ACCEPT_MAG_CAL => MavResult::MAV_RESULT_ACCEPTED,
            MAV_CMD_ACCELCAL_VEHICLE_POS => {
                if self.accel_step >= 6 {
                    self.accel_step = 0;
                    self.text(MavSeverity::MAV_SEVERITY_INFO, "Calibration successful");
                    self.send_accel_pos(16777215.0);
                } else if self.accel_step > 0 {
                    self.accel_step += 1;
                    self.ask_accel_pos();
                }
                return;
            }
            MAV_CMD_PREFLIGHT_CALIBRATION
            | MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
            | MAV_CMD_DO_SET_SERVO
            | MAV_CMD_DO_REPEAT_SERVO
            | MAV_CMD_DO_SET_RELAY => MavResult::MAV_RESULT_ACCEPTED,
            _ => MavResult::MAV_RESULT_UNSUPPORTED,
        };
        self.ack(c.command, res);
    }

    fn ask_accel_pos(&mut self) {
        let names = ["", "level", "on its LEFT side", "on its RIGHT side", "nose DOWN", "nose UP", "on its BACK"];
        let n = names[self.accel_step as usize];
        self.text(MavSeverity::MAV_SEVERITY_CRITICAL, &format!("Place vehicle {n} and press any key."));
        self.send_accel_pos(self.accel_step as f32);
    }

    fn send_accel_pos(&mut self, pos: f32) {
        self.send(MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: pos,
            command: MavCmd::MAV_CMD_ACCELCAL_VEHICLE_POS,
            target_system: 255,
            target_component: 190,
            ..Default::default()
        }));
    }

    fn home_position(&mut self) {
        self.send(MavMessage::HOME_POSITION(HOME_POSITION_DATA {
            latitude: (self.home.lat * 1e7) as i32,
            longitude: (self.home.lng * 1e7) as i32,
            altitude: (self.home.alt * 1000.0) as i32,
            q: [1.0, 0.0, 0.0, 0.0],
            ..Default::default()
        }));
    }

    /// Simple kinematic model: fly toward the target at WPNAV_SPEED, climb at PILOT_SPEED_UP.
    fn physics(&mut self, dt: f32) {
        if self.armed {
            self.battery = (self.battery - dt * 0.02).max(0.0);
        }
        if self.armed && self.mode == MODE_AUTO && self.target.is_none() {
            self.next_auto_target();
        }
        if self.mode == MODE_LAND && self.armed {
            self.target = Some(PointLatLngAlt::new(self.pos.lat, self.pos.lng, 0.0));
        }

        let speed = self.params["WPNAV_SPEED"] / 100.0;
        let vz_max = self.params["WPNAV_SPEED_UP"] / 100.0;
        let (mut gs, mut climb, mut roll, mut pitch) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        if let (true, Some(t)) = (self.armed, self.target) {
            let here = PointLatLngAlt::new(self.pos.lat, self.pos.lng, 0.0);
            let d = here.distance(&t) as f32;
            if d > 0.5 {
                let brg = here.bearing(&t) as f32;
                let mut dy = brg - self.yaw;
                if dy > 180.0 {
                    dy -= 360.0;
                }
                if dy < -180.0 {
                    dy += 360.0;
                }
                self.yaw = (self.yaw + dy.clamp(-90.0 * dt, 90.0 * dt) + 360.0) % 360.0;
                roll = (dy * 0.3).clamp(-20.0, 20.0);
                gs = speed.min(d / 2.0 + 0.5);
                pitch = -(gs / speed) * 12.0;
                let np = here.newpos(brg as f64, (gs * dt) as f64);
                self.pos.lat = np.lat;
                self.pos.lng = np.lng;
            }
            let dz = t.alt as f32 - self.pos.alt as f32;
            climb = dz.clamp(-vz_max * dt, vz_max * dt) / dt.max(1e-3);
            self.pos.alt = (self.pos.alt + (climb * dt) as f64).max(0.0);

            let wp_radius = self.params["WPNAV_RADIUS"] / 100.0;
            if d <= wp_radius.max(1.0) && dz.abs() < 0.5 {
                if self.mode == MODE_AUTO {
                    self.text(
                        MavSeverity::MAV_SEVERITY_INFO,
                        &format!("Reached command #{}", self.current_wp),
                    );
                    self.current_wp += 1;
                    self.target = None;
                } else if self.mode == MODE_RTL {
                    self.set_mode(MODE_LAND);
                }
            }
        }
        if self.armed && self.pos.alt < 0.05 && matches!(self.mode, MODE_LAND) {
            self.armed = false;
            self.text(MavSeverity::MAV_SEVERITY_INFO, "Land complete, disarming");
            self.set_mode(MODE_LOITER);
        }
        self.gs = gs;
        self.climb = climb;
        self.roll += (roll - self.roll) * 0.3;
        self.pitch += (pitch - self.pitch) * 0.3;
    }

    fn next_auto_target(&mut self) {
        loop {
            let Some(wp) = self.mission.get(self.current_wp as usize).copied() else {
                self.text(
                    MavSeverity::MAV_SEVERITY_INFO,
                    "Mission complete, changing mode to RTL",
                );
                self.set_mode(MODE_RTL);
                return;
            };
            match wp.id {
                mavcmd::TAKEOFF => {
                    self.target = Some(PointLatLngAlt::new(
                        self.pos.lat,
                        self.pos.lng,
                        wp.alt as f64,
                    ));
                    return;
                }
                mavcmd::WAYPOINT
                | mavcmd::SPLINE_WAYPOINT
                | mavcmd::LOITER_TIME
                | mavcmd::LOITER_TURNS
                | mavcmd::LOITER_UNLIM => {
                    let (lat, lng) = if wp.lat == 0.0 && wp.lng == 0.0 {
                        (self.pos.lat, self.pos.lng)
                    } else {
                        (wp.lat, wp.lng)
                    };
                    self.target = Some(PointLatLngAlt::new(lat, lng, wp.alt as f64));
                    return;
                }
                mavcmd::LAND => {
                    self.set_mode(MODE_LAND);
                    return;
                }
                mavcmd::RETURN_TO_LAUNCH => {
                    self.set_mode(MODE_RTL);
                    return;
                }
                mavcmd::DO_JUMP => {
                    self.current_wp = wp.p1 as u16;
                }
                _ => self.current_wp += 1,
            }
        }
    }

    fn fast_telemetry(&mut self) {
        let t = self.boot.elapsed().as_millis() as u32;
        self.send(MavMessage::ATTITUDE(ATTITUDE_DATA {
            time_boot_ms: t,
            roll: self.roll.to_radians(),
            pitch: self.pitch.to_radians(),
            yaw: if self.yaw > 180.0 {
                self.yaw - 360.0
            } else {
                self.yaw
            }
            .to_radians(),
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
        }));
        let (vn, ve) = (
            self.gs * self.yaw.to_radians().cos(),
            self.gs * self.yaw.to_radians().sin(),
        );
        self.send(MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
            time_boot_ms: t,
            lat: (self.pos.lat * 1e7) as i32,
            lon: (self.pos.lng * 1e7) as i32,
            alt: ((self.home.alt + self.pos.alt) * 1000.0) as i32,
            relative_alt: (self.pos.alt * 1000.0) as i32,
            vx: (vn * 100.0) as i16,
            vy: (ve * 100.0) as i16,
            vz: (-self.climb * 100.0) as i16,
            hdg: (self.yaw * 100.0) as u16,
        }));
        self.send(MavMessage::VFR_HUD(VFR_HUD_DATA {
            airspeed: self.gs,
            groundspeed: self.gs,
            alt: (self.home.alt + self.pos.alt) as f32,
            climb: self.climb,
            heading: self.yaw as i16,
            throttle: if self.armed {
                45 + (self.climb * 5.0) as u16
            } else {
                0
            },
        }));
        if let Some(tgt) = self.target {
            let here = PointLatLngAlt::new(self.pos.lat, self.pos.lng, 0.0);
            self.send(MavMessage::NAV_CONTROLLER_OUTPUT(
                NAV_CONTROLLER_OUTPUT_DATA {
                    nav_roll: self.roll,
                    nav_pitch: self.pitch,
                    alt_error: tgt.alt as f32 - self.pos.alt as f32,
                    aspd_error: 0.0,
                    xtrack_error: 0.0,
                    nav_bearing: here.bearing(&tgt) as i16,
                    target_bearing: here.bearing(&tgt) as i16,
                    wp_dist: here.distance(&tgt) as u16,
                },
            ));
        }
    }

    fn magcal_tick(&mut self) {
        let Some(pct) = self.magcal else { return };
        let pct = (pct + 10).min(100);
        self.magcal = if pct >= 100 { None } else { Some(pct) };
        self.send(MavMessage::MAG_CAL_PROGRESS(MAG_CAL_PROGRESS_DATA { compass_id: 0, completion_pct: pct, cal_status: MagCalStatus::MAG_CAL_RUNNING_STEP_TWO, ..Default::default() }));
        if pct >= 100 {
            self.send(MavMessage::MAG_CAL_REPORT(MAG_CAL_REPORT_DATA {
                fitness: 4.3,
                ofs_x: 12.0,
                ofs_y: -31.0,
                ofs_z: 44.0,
                diag_x: 1.0,
                diag_y: 1.0,
                diag_z: 1.0,
                compass_id: 0,
                cal_status: MagCalStatus::MAG_CAL_SUCCESS,
                autosaved: 1,
                ..Default::default()
            }));
            self.text(MavSeverity::MAV_SEVERITY_INFO, "Mag(0) good orientation: 0 1.0");
        }
    }

    /// A light aircraft orbiting 3 km from home, reported as ADSB traffic.
    fn adsb_traffic(&mut self) {
        let t = self.boot.elapsed().as_secs_f64();
        let ang = t * 0.02;
        let (lat, lng) = (self.home.lat + (3000.0 * ang.cos()) / 111_320.0, self.home.lng + (3000.0 * ang.sin()) / (111_320.0 * self.home.lat.to_radians().cos()));
        let heading = ((ang.to_degrees() + 90.0).rem_euclid(360.0) * 100.0) as u16;
        self.send(MavMessage::ADSB_VEHICLE(ADSB_VEHICLE_DATA {
            ICAO_address: 0x7C6B2D,
            lat: (lat * 1e7) as i32,
            lon: (lng * 1e7) as i32,
            altitude: 1_200_000,
            heading,
            hor_velocity: 6000,
            squawk: 1200,
            callsign: "VH-RSM".into(),
            tslc: 1,
            flags: AdsbFlags::ADSB_FLAGS_VALID_COORDS | AdsbFlags::ADSB_FLAGS_VALID_ALTITUDE | AdsbFlags::ADSB_FLAGS_VALID_HEADING | AdsbFlags::ADSB_FLAGS_VALID_CALLSIGN,
            ..Default::default()
        }));
    }

    /// Send a DroneCAN transfer from the AP_Periph node (id 125) as CAN_FRAMEs.
    fn periph_send(&mut self, id: crate::dronecan::FrameId, payload: &[u8], sig: u64) {
        let Some((bus, _)) = self.can_forward else { return };
        let tid = self.periph.1;
        self.periph.1 = (tid + 1) & 0x1f;
        for f in crate::dronecan::frames(payload, sig, tid) {
            let mut data = [0u8; 8];
            data[..f.len()].copy_from_slice(&f);
            self.send(MavMessage::CAN_FRAME(CAN_FRAME_DATA { id: id.encode() | 0x8000_0000, target_system: 255, target_component: 190, bus: bus - 1, len: f.len() as u8, data }));
        }
    }

    fn periph_log(&mut self, text: &str) {
        let src = b"periph";
        // level INFO (1) in the top 3 bits, then the 5-bit source length
        let mut p = vec![(1u8 << 5) | (src.len() as u8 & 0x1f)];
        p.extend_from_slice(src);
        p.extend_from_slice(text.as_bytes());
        self.periph_send(crate::dronecan::FrameId::Message { priority: 24, type_id: crate::dronecan::LOGMESSAGE_ID, source: 125 }, &p, 0xd654_a48e_0c04_9d75);
    }

    fn periph_status(&self) -> crate::dronecan::NodeStatus {
        crate::dronecan::NodeStatus { uptime: self.periph.0.elapsed().as_secs() as u32, health: 0, mode: 0, sub_mode: 0, vendor: 0 }
    }

    /// Requests on the bus addressed to the AP_Periph node.
    fn periph_frame(&mut self, id: u32, data: &[u8]) {
        use crate::dronecan::*;
        let Some(payload) = self.can_asm.push(id, data) else { return };
        if let FrameId::Service { type_id, request: true, dest: 125, source, .. } = FrameId::decode(id) {
            let reply = FrameId::Service { priority: 30, type_id, request: false, dest: source, source: 125 };
            match type_id {
                GETNODEINFO_ID => {
                    let info = NodeInfo { status: self.periph_status(), sw: (1, 7, 0x5a3c1e2b, 0x0123_4567_89ab_cdef), hw: (1, 0), uid: [0x32, 0x00, 0x2c, 0x00, 0x0d, 0x51, 0x30, 0x34, 0x39, 0x36, 0x31, 0x38, 0, 0, 0, 0], name: "org.ardupilot.ap_periph".into() };
                    self.periph_send(reply, &info.encode(), GETNODEINFO_SIG);
                }
                RESTARTNODE_ID if payload.len() >= 5 && payload[..5] == 0xAC_CE55_1B1E_u64.to_le_bytes()[..5] => {
                    self.periph_send(reply, &[0x80], RESTARTNODE_SIG);
                    self.periph.0 = Instant::now();
                    self.periph_log("Rebooting");
                }
                _ => {}
            }
        }
    }

    fn slow_telemetry(&mut self) {
        self.magcal_tick();
        // AP_Periph NodeStatus at 1 Hz while the GCS keeps CAN forwarding alive
        if let Some((_, t)) = self.can_forward {
            if t.elapsed() > Duration::from_secs(5) {
                self.can_forward = None;
            } else {
                let ns = self.periph_status().encode();
                self.periph_send(crate::dronecan::FrameId::Message { priority: 16, type_id: crate::dronecan::NODESTATUS_ID, source: 125 }, &ns, crate::dronecan::NODESTATUS_SIG);
            }
        }
        self.adsb_traffic();
        let mut base = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            | MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED;
        if self.armed {
            base |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        }
        self.send(MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: self.wire_mode(),
            mavtype: match self.kind {
                SimKind::Copter => MavType::MAV_TYPE_QUADROTOR,
                SimKind::Plane => MavType::MAV_TYPE_FIXED_WING,
                SimKind::Rover => MavType::MAV_TYPE_GROUND_ROVER,
                SimKind::Tracker => MavType::MAV_TYPE_ANTENNA_TRACKER,
            },
            autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: base,
            system_status: if self.armed {
                MavState::MAV_STATE_ACTIVE
            } else {
                MavState::MAV_STATE_STANDBY
            },
            mavlink_version: 3,
        }));
        let sensors = MavSysStatusSensor::from_bits_truncate(0x0020_FC2F);
        self.send(MavMessage::SYS_STATUS(SYS_STATUS_DATA {
            onboard_control_sensors_present: sensors,
            onboard_control_sensors_enabled: sensors,
            onboard_control_sensors_health: sensors,
            load: 250,
            voltage_battery: (11_100.0 + self.battery * 15.0) as u16,
            current_battery: if self.armed { 1250 } else { 50 },
            battery_remaining: self.battery as i8,
            ..Default::default()
        }));
        self.send(MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            time_usec: self.boot.elapsed().as_micros() as u64,
            lat: (self.pos.lat * 1e7) as i32,
            lon: (self.pos.lng * 1e7) as i32,
            alt: ((self.home.alt + self.pos.alt) * 1000.0) as i32,
            eph: 121,
            epv: 200,
            vel: (self.gs * 100.0) as u16,
            cog: (self.yaw * 100.0) as u16,
            fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
            satellites_visible: 10,
            ..Default::default()
        }));
        self.send(MavMessage::MISSION_CURRENT(MISSION_CURRENT_DATA {
            seq: self.current_wp,
            ..Default::default()
        }));
        let thr = if self.armed { 1500 } else { 1000 };
        self.send(MavMessage::RC_CHANNELS(RC_CHANNELS_DATA {
            time_boot_ms: self.boot.elapsed().as_millis() as u32,
            chancount: 8,
            chan1_raw: 1500,
            chan2_raw: 1500,
            chan3_raw: 1000,
            chan4_raw: 1500,
            chan5_raw: 1100 + (self.mode.min(6) as u16) * 100,
            chan6_raw: 1500,
            chan7_raw: 1000,
            chan8_raw: 1000,
            rssi: 255,
            ..Default::default()
        }));
        self.send(MavMessage::SERVO_OUTPUT_RAW(SERVO_OUTPUT_RAW_DATA {
            servo1_raw: thr,
            servo2_raw: thr,
            servo3_raw: thr,
            servo4_raw: thr,
            ..Default::default()
        }));
        let t = self.boot.elapsed().as_secs_f32();
        self.send(MavMessage::RAW_IMU(RAW_IMU_DATA {
            time_usec: self.boot.elapsed().as_micros() as u64,
            xacc: (self.pitch * 17.0 + (t * 7.0).sin() * 3.0) as i16,
            yacc: (-self.roll * 17.0 + (t * 5.0).cos() * 3.0) as i16,
            zacc: -1000 + ((t * 9.0).sin() * 5.0) as i16,
            xgyro: ((t * 3.0).sin() * 4.0) as i16,
            ygyro: ((t * 2.0).cos() * 4.0) as i16,
            zgyro: ((t * 1.3).sin() * 2.0) as i16,
            xmag: (200.0 * self.yaw.to_radians().cos()) as i16,
            ymag: (-200.0 * self.yaw.to_radians().sin()) as i16,
            zmag: -420,
            ..Default::default()
        }));
        self.send(MavMessage::VIBRATION(VIBRATION_DATA {
            vibration_x: 3.1,
            vibration_y: 2.8,
            vibration_z: 4.5,
            ..Default::default()
        }));
        self.home_position();
        let _ = MavCmd::from_u16(0);
    }
}

fn name16(id: &mavlink::types::CharArray<16>) -> String {
    let b: &[u8] = &id[..];
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).to_string()
}
