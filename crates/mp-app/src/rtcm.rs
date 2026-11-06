//! RTK/GPS Inject (ConfigSerialInjectGPS): read RTCM from a serial port, TCP, UDP or an NTRIP
//! caster, split it into RTCM3 messages and forward each as GPS_RTCM_DATA (fragmented at 180
//! bytes, with the MAVLink fragment/sequence flags). A u-blox base on a serial port can be
//! configured for survey-in, and its NAV-SVIN reports are shown.

use std::collections::{BTreeMap, HashMap};
use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use mp_core::comms::{self, ConnectionSpec};
use mp_core::link::LinkCommand;
use mp_core::mavlink::dialects::ardupilotmega::{MavMessage, GPS_RTCM_DATA_DATA};
use mp_core::rtk;
use parking_lot::Mutex;

#[derive(Default, Clone)]
pub struct InjectStatus {
    /// bytes per second in (input data rate), as bits per second
    pub bps: u32,
    /// bits per second forwarded to the vehicle
    pub bps_sent: u32,
    pub total: u64,
    pub msgs: u64,
    pub error: Option<String>,
    pub running: bool,
    /// msgseen: "rtcm1005" -> count
    pub seen: BTreeMap<String, u64>,
    /// status light -> last time a message for it arrived
    pub lights: HashMap<&'static str, Instant>,
    /// RTCM 1005/1006 base position (lat, lng, alt)
    pub base: Option<(f64, f64, f64)>,
    pub svin: Option<rtk::Svin>,
    /// CFG-TMODE3 flags reported by the receiver
    pub tmode: Option<u16>,
}

impl InjectStatus {
    /// ExpireType: base lights stay green 20 s, the constellations 5 s.
    pub fn light(&self, name: &str) -> bool {
        let keep = if name == "base" { 20 } else { 5 };
        self.lights.get(name).map(|t| t.elapsed().as_secs() < keep).unwrap_or(false)
    }
}

/// What to do with the receiver when connecting (chk_autoconfig) and NTRIP GGA.
#[derive(Clone, Default)]
pub struct InjectOptions {
    /// configuration written to the receiver before reading (UBX / Septentrio / Unicore)
    pub init: Vec<Vec<u8>>,
    /// chk_sendgga: vehicle position for $GPGGA every 5 s (NTRIP)
    pub gga: Option<Arc<Mutex<Option<(f64, f64, f64)>>>>,
}

#[derive(Default)]
pub struct Injector {
    pub status: Arc<Mutex<InjectStatus>>,
    stop: Arc<AtomicBool>,
    /// bytes to write to the receiver (survey restart, fixed base)
    write: Arc<Mutex<Vec<Vec<u8>>>>,
}

pub enum Source {
    Spec(ConnectionSpec),
    /// ntrip://user:pass@host:port/mount
    Ntrip(String),
}

trait Stream: Read + Write + Send {}
impl<T: Read + Write + Send> Stream for T {}

fn ntrip_connect(url: &str) -> std::io::Result<TcpStream> {
    let rest = url.trim_start_matches("ntrip://");
    let (auth, hostpart) = match rest.rsplit_once('@') {
        Some((a, h)) => (Some(a), h),
        None => (None, rest),
    };
    let (hostport, mount) = hostpart.split_once('/').unwrap_or((hostpart, ""));
    let addr = if hostport.contains(':') { hostport.to_string() } else { format!("{hostport}:2101") };
    let mut s = TcpStream::connect(addr)?;
    let mut req = format!("GET /{mount} HTTP/1.0\r\nUser-Agent: NTRIP MissionPlannerRS\r\nNtrip-Version: Ntrip/2.0\r\n");
    if let Some(a) = auth {
        req.push_str(&format!("Authorization: Basic {}\r\n", base64(a.as_bytes())));
    }
    req.push_str("\r\n");
    s.write_all(req.as_bytes())?;
    s.set_read_timeout(Some(Duration::from_millis(200)))?;
    Ok(s)
}

fn base64(input: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    for c in input.chunks(3) {
        let b = [c[0], *c.get(1).unwrap_or(&0), *c.get(2).unwrap_or(&0)];
        let n = (b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32;
        for i in 0..4 {
            if i <= c.len() {
                out.push(T[(n >> (18 - 6 * i) & 63) as usize] as char);
            } else {
                out.push('=');
            }
        }
    }
    out
}

/// Split one RTCM buffer into GPS_RTCM_DATA messages (MAVLink fragmentation rules).
pub fn fragments(data: &[u8], seq: &mut u8) -> Vec<MavMessage> {
    let mut out = Vec::new();
    let chunks: Vec<&[u8]> = data.chunks(180).collect();
    let fragmented = chunks.len() > 1;
    for (i, c) in chunks.iter().enumerate().take(4) {
        let mut d = [0u8; 180];
        d[..c.len()].copy_from_slice(c);
        let flags = (fragmented as u8) | ((i as u8 & 3) << 1) | ((*seq & 0x1f) << 3);
        out.push(MavMessage::GPS_RTCM_DATA(GPS_RTCM_DATA_DATA { flags, len: c.len() as u8, data: d }));
    }
    *seq = seq.wrapping_add(1);
    out
}

fn utc_now() -> (u32, u32, f64) {
    let s = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
    let day = s.as_secs_f64() % 86400.0;
    ((day / 3600.0) as u32, ((day % 3600.0) / 60.0) as u32, day % 60.0)
}

impl Injector {
    pub fn running(&self) -> bool {
        self.status.lock().running
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    /// Queue raw bytes for the receiver (only a serial / TCP source can take them).
    pub fn write(&self, bytes: Vec<u8>) {
        self.write.lock().push(bytes);
    }

    pub fn start(&self, src: Source, send: crossbeam_channel::Sender<LinkCommand>, opts: InjectOptions) {
        self.stop.store(false, Ordering::Relaxed);
        let stop = self.stop.clone();
        let status = self.status.clone();
        let pending = self.write.clone();
        *status.lock() = InjectStatus { running: true, ..Default::default() };
        let ntrip = matches!(src, Source::Ntrip(_));
        std::thread::spawn(move || {
            let opened: std::io::Result<Box<dyn Stream>> = match &src {
                Source::Spec(spec) => comms::open(spec).map(|t| Box::new(t) as Box<dyn Stream>),
                Source::Ntrip(url) => ntrip_connect(url).map(|t| Box::new(t) as Box<dyn Stream>),
            };
            let mut port = match opened {
                Ok(p) => p,
                Err(e) => {
                    let mut s = status.lock();
                    s.error = Some(e.to_string());
                    s.running = false;
                    return;
                }
            };
            for p in &opts.init {
                let _ = port.write_all(p);
                std::thread::sleep(Duration::from_millis(20));
            }
            let mut buf = [0u8; 2048];
            let mut seq = 0u8;
            let mut parser = rtk::Parser::default();
            let mut window = Instant::now();
            let (mut win_in, mut win_out) = (0u64, 0u64);
            let mut framed = false;
            let mut unframed = 0usize;
            let mut last_gga = Instant::now() - Duration::from_secs(10);
            while !stop.load(Ordering::Relaxed) {
                for w in std::mem::take(&mut *pending.lock()) {
                    let _ = port.write_all(&w);
                }
                if ntrip {
                    if let Some(pos) = opts.gga.as_ref().and_then(|g| *g.lock()) {
                        if last_gga.elapsed() >= Duration::from_secs(5) {
                            last_gga = Instant::now();
                            let _ = port.write_all(rtk::gga(pos.0, pos.1, pos.2, utc_now()).as_bytes());
                        }
                    }
                }
                match port.read(&mut buf) {
                    Ok(0) => {}
                    Ok(n) => {
                        win_in += n as u64;
                        let frames = parser.push(&buf[..n]);
                        let mut s = status.lock();
                        s.total += n as u64;
                        if frames.is_empty() && !framed {
                            unframed += n;
                        }
                        for f in frames {
                            match f {
                                rtk::Frame::Rtcm { msgtype, raw } => {
                                    framed = true;
                                    for m in fragments(&raw, &mut seq) {
                                        let _ = send.send(LinkCommand::Send(Box::new(m)));
                                    }
                                    win_out += raw.len() as u64;
                                    s.msgs += 1;
                                    *s.seen.entry(format!("rtcm{msgtype}")).or_default() += 1;
                                    if let Some(l) = rtk::constellation(msgtype) {
                                        s.lights.insert(l, Instant::now());
                                    }
                                    if let Some(e) = rtk::base_ecef(&raw) {
                                        s.base = Some(rtk::ecef_to_lla(e));
                                    }
                                }
                                rtk::Frame::Ubx { class: 0x01, id: 0x3b, payload } => s.svin = rtk::nav_svin(&payload),
                                rtk::Frame::Ubx { class: 0x06, id: 0x71, payload } if payload.len() >= 4 => s.tmode = Some(u16::from_le_bytes([payload[2], payload[3]])),
                                rtk::Frame::Ubx { class, id, .. } => {
                                    *s.seen.entry(format!("ubx{class:02x}{id:02x}")).or_default() += 1;
                                }
                            }
                        }
                        // not RTCM3 at all (SBP, raw corrections): forward the bytes as they are
                        if !framed && unframed > 4096 {
                            for m in fragments(&buf[..n], &mut seq) {
                                let _ = send.send(LinkCommand::Send(Box::new(m)));
                            }
                            win_out += n as u64;
                            s.msgs += 1;
                        }
                    }
                    Err(e) if matches!(e.kind(), std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock) => {}
                    Err(e) => {
                        status.lock().error = Some(e.to_string());
                        break;
                    }
                }
                if window.elapsed() >= Duration::from_secs(1) {
                    let mut s = status.lock();
                    s.bps = (win_in * 8) as u32;
                    s.bps_sent = (win_out * 8) as u32;
                    win_in = 0;
                    win_out = 0;
                    window = Instant::now();
                }
            }
            status.lock().running = false;
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fragmenting() {
        let mut seq = 0;
        let m = fragments(&[1u8; 400], &mut seq);
        assert_eq!(m.len(), 3);
        if let MavMessage::GPS_RTCM_DATA(d) = &m[2] {
            assert_eq!(d.len, 40);
            assert_eq!(d.flags & 1, 1);
            assert_eq!((d.flags >> 1) & 3, 2);
        } else {
            panic!();
        }
        assert_eq!(base64(b"user:pass"), "dXNlcjpwYXNz");
    }
}
