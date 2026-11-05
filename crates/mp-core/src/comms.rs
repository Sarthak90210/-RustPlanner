//! Transport layer, the equivalent of `ExtLibs/Comms` (`ICommsSerial`): serial, TCP client,
//! UDP listen ("UDP") and UDP client ("UDPCl").
//!
//! Every transport reads with a short timeout so the link thread can interleave reads and
//! writes on a single thread.

use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs, UdpSocket};
use std::time::Duration;

pub const READ_TIMEOUT: Duration = Duration::from_millis(20);

/// Baud rates offered by Mission Planner's baud combo.
pub const BAUD_RATES: &[u32] = &[
    4800, 9600, 14400, 19200, 28800, 38400, 57600, 111100, 115200, 230400, 256000, 460800, 500000,
    921600, 1500000, 2000000,
];

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectionSpec {
    /// Mission Planner "AUTO": scan serial ports for a MAVLink heartbeat (CommsSerialScan).
    Auto,
    Serial {
        port: String,
        baud: u32,
    },
    Tcp {
        host: String,
        port: u16,
    },
    /// Listen on a local UDP port and reply to whoever talks to us (Mission Planner "UDP").
    UdpListen {
        port: u16,
    },
    /// Send to a remote UDP host/port (Mission Planner "UDPCl").
    UdpClient {
        host: String,
        port: u16,
    },
}

impl std::fmt::Display for ConnectionSpec {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConnectionSpec::Auto => write!(f, "AUTO"),
            ConnectionSpec::Serial { port, baud } => write!(f, "{port} @ {baud}"),
            ConnectionSpec::Tcp { host, port } => write!(f, "tcp:{host}:{port}"),
            ConnectionSpec::UdpListen { port } => write!(f, "udp:0.0.0.0:{port}"),
            ConnectionSpec::UdpClient { host, port } => write!(f, "udpcl:{host}:{port}"),
        }
    }
}

pub trait Transport: Read + Write + Send {}
impl<T: Read + Write + Send> Transport for T {}

/// Lists serial ports (desktop only; mobile platforms have no direct serial access).
pub fn serial_ports() -> Vec<String> {
    #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
    {
        serialport::available_ports()
            .map(|v| v.into_iter().map(|p| p.port_name).collect())
            .unwrap_or_default()
    }
    #[cfg(any(target_os = "android", target_os = "ios", target_arch = "wasm32"))]
    {
        Vec::new()
    }
}

pub fn open(spec: &ConnectionSpec) -> io::Result<Box<dyn Transport>> {
    match spec {
        ConnectionSpec::Auto => {
            let (port, baud) = autoscan(std::time::Duration::from_secs(50))
                .ok_or_else(|| io::Error::new(io::ErrorKind::TimedOut, "Timeout"))?;
            open_serial(&port, baud)
        }
        ConnectionSpec::Serial { port, baud } => open_serial(port, *baud),
        ConnectionSpec::Tcp { host, port } => {
            let addr = resolve(host, *port)?;
            let s = TcpStream::connect_timeout(&addr, Duration::from_secs(5))?;
            s.set_nodelay(true)?;
            s.set_read_timeout(Some(READ_TIMEOUT))?;
            Ok(Box::new(TcpTransport(s)))
        }
        ConnectionSpec::UdpListen { port } => {
            let sock = UdpSocket::bind(("0.0.0.0", *port))?;
            sock.set_read_timeout(Some(READ_TIMEOUT))?;
            Ok(Box::new(UdpTransport::new(sock, None)))
        }
        ConnectionSpec::UdpClient { host, port } => {
            let addr = resolve(host, *port)?;
            let sock = UdpSocket::bind(("0.0.0.0", 0))?;
            sock.set_read_timeout(Some(READ_TIMEOUT))?;
            Ok(Box::new(UdpTransport::new(sock, Some(addr))))
        }
    }
}

fn resolve(host: &str, port: u16) -> io::Result<SocketAddr> {
    (host, port)
        .to_socket_addrs()?
        .next()
        .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "host not found"))
}

#[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
fn open_serial(port: &str, baud: u32) -> io::Result<Box<dyn Transport>> {
    let p = serialport::new(port, baud)
        .timeout(READ_TIMEOUT)
        .open()
        .map_err(|e| io::Error::other(e.to_string()))?;
    // Mission Planner toggles DTR/RTS on open; most flight controllers expect DTR high.
    let mut p = p;
    let _ = p.write_data_terminal_ready(true);
    let _ = p.write_request_to_send(true);
    Ok(Box::new(SerialTransport(p)))
}

#[cfg(any(target_os = "android", target_os = "ios", target_arch = "wasm32"))]
fn open_serial(_port: &str, _baud: u32) -> io::Result<Box<dyn Transport>> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "serial ports are not available on this platform",
    ))
}

#[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
struct SerialTransport(Box<dyn serialport::SerialPort>);

#[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
impl Read for SerialTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.0.read(buf) {
            Ok(0) => Err(io::ErrorKind::TimedOut.into()),
            r => r,
        }
    }
}

#[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
impl Write for SerialTransport {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}

struct TcpTransport(TcpStream);

impl Read for TcpTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.0.read(buf) {
            Ok(0) => Err(io::Error::new(
                io::ErrorKind::ConnectionAborted,
                "TCP connection closed",
            )),
            r => r,
        }
    }
}

impl Write for TcpTransport {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}

/// UDP is datagram based; buffer one datagram and hand it out to the stream-style reader.
pub struct UdpTransport {
    sock: UdpSocket,
    remote: Option<SocketAddr>,
    buf: Vec<u8>,
    pos: usize,
    len: usize,
    out: Vec<u8>,
    /// listen mode: every address heard from (a swarm on one port)
    peers: Vec<SocketAddr>,
    listen: bool,
}

impl UdpTransport {
    pub fn new(sock: UdpSocket, remote: Option<SocketAddr>) -> Self {
        Self {
            sock,
            remote,
            buf: vec![0; 65536],
            pos: 0,
            len: 0,
            out: Vec::with_capacity(512),
            peers: Vec::new(),
            listen: remote.is_none(),
        }
    }
}

impl Read for UdpTransport {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        if self.pos >= self.len {
            let (n, from) = self.sock.recv_from(&mut self.buf)?;
            if n == 0 {
                return Err(io::ErrorKind::TimedOut.into());
            }
            // "UDP" mode replies to the most recent sender, like Mission Planner's CommsUdp.
            self.remote = Some(from);
            if self.listen && !self.peers.contains(&from) && self.peers.len() < 32 {
                self.peers.push(from);
            }
            self.pos = 0;
            self.len = n;
        }
        let n = out.len().min(self.len - self.pos);
        out[..n].copy_from_slice(&self.buf[self.pos..self.pos + n]);
        self.pos += n;
        Ok(n)
    }
}

impl Write for UdpTransport {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.out.extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        if !self.out.is_empty() {
            if self.listen && self.peers.len() > 1 {
                for p in &self.peers {
                    self.sock.send_to(&self.out, p)?;
                }
            } else if let Some(r) = self.remote {
                self.sock.send_to(&self.out, r)?;
            }
        }
        self.out.clear();
        Ok(())
    }
}

/// Port of `CommsSerialScan.Scan`: try every serial port at the common baud rates and return the
/// first one that delivers a valid MAVLink HEARTBEAT.
pub fn autoscan(deadline: std::time::Duration) -> Option<(String, u32)> {
    let end = std::time::Instant::now() + deadline;
    while std::time::Instant::now() < end {
        for port in serial_ports() {
            for baud in [115200u32, 57600, 921600, 38400, 500000] {
                if let Ok(t) = open_serial(&port, baud) {
                    if probe_heartbeat(t, std::time::Duration::from_millis(1500)) {
                        return Some((port, baud));
                    }
                }
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    None
}

fn probe_heartbeat(t: Box<dyn Transport>, wait: std::time::Duration) -> bool {
    use mavlink::dialects::ardupilotmega::MavMessage;
    let end = std::time::Instant::now() + wait;
    let mut r = mavlink::MavlinkReader::new(t);
    while std::time::Instant::now() < end {
        match r.read_any_message::<MavMessage>() {
            Ok((_, MavMessage::HEARTBEAT(_))) => return true,
            Ok(_) => {}
            Err(mavlink::error::MessageReadError::Io(e)) if matches!(e.kind(), io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock) => {}
            Err(mavlink::error::MessageReadError::Io(_)) => return false,
            Err(_) => {}
        }
    }
    false
}
