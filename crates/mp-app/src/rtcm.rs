//! RTK/GPS Inject (ConfigSerialInjectGPS): read RTCM from a serial port, TCP, UDP or an NTRIP
//! caster and forward it to the vehicle as GPS_RTCM_DATA (fragmented at 180 bytes, with the
//! MAVLink fragment/sequence flags).

use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use mp_core::comms::{self, ConnectionSpec};
use mp_core::link::LinkCommand;
use mp_core::mavlink::dialects::ardupilotmega::{MavMessage, GPS_RTCM_DATA_DATA};
use parking_lot::Mutex;

#[derive(Default, Clone)]
pub struct InjectStatus {
    pub bps: u32,
    pub total: u64,
    pub msgs: u64,
    pub error: Option<String>,
    pub running: bool,
}

#[derive(Default)]
pub struct Injector {
    pub status: Arc<Mutex<InjectStatus>>,
    stop: Arc<AtomicBool>,
}

pub enum Source {
    Spec(ConnectionSpec),
    /// ntrip://user:pass@host:port/mount
    Ntrip(String),
}

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

impl Injector {
    pub fn running(&self) -> bool {
        self.status.lock().running
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    pub fn start(&self, src: Source, send: crossbeam_channel::Sender<LinkCommand>) {
        self.stop.store(false, Ordering::Relaxed);
        let stop = self.stop.clone();
        let status = self.status.clone();
        *status.lock() = InjectStatus { running: true, ..Default::default() };
        std::thread::spawn(move || {
            let mut reader: Box<dyn Read + Send> = match &src {
                Source::Spec(spec) => match comms::open(spec) {
                    Ok(t) => Box::new(t),
                    Err(e) => {
                        let mut s = status.lock();
                        s.error = Some(e.to_string());
                        s.running = false;
                        return;
                    }
                },
                Source::Ntrip(url) => match ntrip_connect(url) {
                    Ok(t) => Box::new(t),
                    Err(e) => {
                        let mut s = status.lock();
                        s.error = Some(e.to_string());
                        s.running = false;
                        return;
                    }
                },
            };
            let mut buf = [0u8; 1024];
            let mut seq = 0u8;
            let mut window = Instant::now();
            let mut window_bytes = 0u64;
            while !stop.load(Ordering::Relaxed) {
                match reader.read(&mut buf) {
                    Ok(0) => {}
                    Ok(n) => {
                        for m in fragments(&buf[..n], &mut seq) {
                            let _ = send.send(LinkCommand::Send(Box::new(m)));
                        }
                        window_bytes += n as u64;
                        let mut s = status.lock();
                        s.total += n as u64;
                        s.msgs += 1;
                    }
                    Err(e) if matches!(e.kind(), std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock) => {}
                    Err(e) => {
                        status.lock().error = Some(e.to_string());
                        break;
                    }
                }
                if window.elapsed() >= Duration::from_secs(1) {
                    status.lock().bps = (window_bytes * 8) as u32;
                    window_bytes = 0;
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
