//! Transport layer, the equivalent of `ExtLibs/Comms` (`ICommsSerial`): serial, TCP client,
//! UDP listen ("UDP") and UDP client ("UDPCl").
//!
//! Every transport reads with a short timeout so the link thread can interleave reads and
//! writes on a single thread.

use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs, UdpSocket};
use std::time::Duration;

pub const READ_TIMEOUT: Duration = Duration::from_millis(20);

/// A read that ended without data: a timeout, a would-block, or on Windows a socket read
/// whose timeout elapsed while the overlapped receive was still pending (ERROR_IO_PENDING).
pub fn is_timeout(e: &io::Error) -> bool {
    matches!(e.kind(), io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock) || (cfg!(windows) && e.raw_os_error() == Some(997))
}

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
    /// WebSocket connection (Mission Planner "WS" / CommsWebSocket).
    WebSocket {
        url: String,
    },
    /// Bluetooth Low Energy transport (Mission Planner CommsBLE).
    Ble {
        address: String,
    },
    /// Native WinUSB transport (Mission Planner CommsWinUSB).
    WinUsb {
        path: String,
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
            ConnectionSpec::WebSocket { url } => write!(f, "ws:{url}"),
            ConnectionSpec::Ble { address } => write!(f, "ble:{address}"),
            ConnectionSpec::WinUsb { path } => write!(f, "winusb:{path}"),
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
        ConnectionSpec::WebSocket { url } => {
            let ws = WebSocketTransport::connect(url)?;
            Ok(Box::new(ws))
        }
        ConnectionSpec::Ble { address } => {
            if address.trim().is_empty() {
                return Err(io::Error::new(io::ErrorKind::InvalidInput, "BLE address required"));
            }
            Ok(Box::new(DriverTransport::new(format!("BLE:{address}"))))
        }
        ConnectionSpec::WinUsb { path } => {
            if path.trim().is_empty() {
                return Err(io::Error::new(io::ErrorKind::InvalidInput, "WinUSB device path required"));
            }
            Ok(Box::new(DriverTransport::new(format!("WinUSB:{path}"))))
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

/// Open a serial port as a raw transport (SiK radio AT commands, NMEA, terminals).
pub fn open_serial_port(port: &str, baud: u32) -> io::Result<Box<dyn Transport>> {
    open_serial(port, baud)
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

pub struct WebSocketTransport {
    stream: TcpStream,
    read_buf: Vec<u8>,
    read_pos: usize,
}

impl WebSocketTransport {
    pub fn connect(url: &str) -> io::Result<Self> {
        let trimmed = url.trim();
        let stripped = trimmed
            .strip_prefix("ws://")
            .or_else(|| trimmed.strip_prefix("wss://"))
            .unwrap_or(trimmed);
        let (host_port, path) = match stripped.split_once('/') {
            Some((hp, p)) => (hp, format!("/{}", p)),
            None => (stripped, "/".to_string()),
        };
        let (host, port) = match host_port.split_once(':') {
            Some((h, p)) => (h, p.parse::<u16>().map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?),
            None => (host_port, if trimmed.starts_with("wss://") { 443 } else { 80 }),
        };
        let addr = resolve(host, port)?;
        let mut stream = TcpStream::connect_timeout(&addr, Duration::from_secs(5))?;
        stream.set_nodelay(true)?;
        stream.set_read_timeout(Some(Duration::from_secs(3)))?;

        let handshake = format!(
            "GET {path} HTTP/1.1\r\n\
             Host: {host}:{port}\r\n\
             Upgrade: websocket\r\n\
             Connection: Upgrade\r\n\
             Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\
             Sec-WebSocket-Version: 13\r\n\r\n"
        );
        stream.write_all(handshake.as_bytes())?;
        stream.flush()?;

        let mut resp = Vec::new();
        let mut b = [0u8; 1];
        while !resp.windows(4).any(|w| w == b"\r\n\r\n") {
            if stream.read(&mut b)? == 0 {
                return Err(io::Error::new(io::ErrorKind::ConnectionAborted, "WebSocket handshake closed prematurely"));
            }
            resp.push(b[0]);
            if resp.len() > 8192 {
                return Err(io::Error::new(io::ErrorKind::InvalidData, "WebSocket handshake response too large"));
            }
        }
        let resp_str = String::from_utf8_lossy(&resp);
        if !resp_str.contains("101") {
            return Err(io::Error::new(io::ErrorKind::ConnectionRefused, format!("WebSocket rejected: {resp_str}")));
        }

        stream.set_read_timeout(Some(READ_TIMEOUT))?;
        Ok(Self {
            stream,
            read_buf: Vec::new(),
            read_pos: 0,
        })
    }
}

impl Read for WebSocketTransport {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        if self.read_pos < self.read_buf.len() {
            let n = out.len().min(self.read_buf.len() - self.read_pos);
            out[..n].copy_from_slice(&self.read_buf[self.read_pos..self.read_pos + n]);
            self.read_pos += n;
            return Ok(n);
        }
        self.read_buf.clear();
        self.read_pos = 0;

        let mut hdr = [0u8; 2];
        self.stream.read_exact(&mut hdr)?;
        let opcode = hdr[0] & 0x0F;
        let masked = (hdr[1] & 0x80) != 0;
        let mut payload_len = (hdr[1] & 0x7F) as usize;

        if payload_len == 126 {
            let mut ext = [0u8; 2];
            self.stream.read_exact(&mut ext)?;
            payload_len = u16::from_be_bytes(ext) as usize;
        } else if payload_len == 127 {
            let mut ext = [0u8; 8];
            self.stream.read_exact(&mut ext)?;
            payload_len = u64::from_be_bytes(ext) as usize;
        }

        let mask = if masked {
            let mut m = [0u8; 4];
            self.stream.read_exact(&mut m)?;
            Some(m)
        } else {
            None
        };

        let mut data = vec![0u8; payload_len];
        self.stream.read_exact(&mut data)?;
        if let Some(m) = mask {
            for (i, b) in data.iter_mut().enumerate() {
                *b ^= m[i % 4];
            }
        }

        match opcode {
            0x1 | 0x2 => {
                let n = out.len().min(data.len());
                out[..n].copy_from_slice(&data[..n]);
                if n < data.len() {
                    self.read_buf = data;
                    self.read_pos = n;
                }
                Ok(n)
            }
            0x8 => {
                Err(io::Error::new(io::ErrorKind::ConnectionAborted, "WebSocket closed by remote"))
            }
            0x9 => {
                let mut pong = vec![0x8A, 0x80 | (data.len().min(125) as u8), 0, 0, 0, 0];
                pong.extend_from_slice(&data[..data.len().min(125)]);
                let _ = self.stream.write_all(&pong);
                let _ = self.stream.flush();
                Err(io::ErrorKind::TimedOut.into())
            }
            _ => Err(io::ErrorKind::TimedOut.into()),
        }
    }
}

impl Write for WebSocketTransport {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mask_key = [0x41, 0x52, 0x44, 0x55];
        let mut frame = Vec::with_capacity(buf.len() + 14);
        frame.push(0x82);
        let len = buf.len();
        if len <= 125 {
            frame.push(0x80 | (len as u8));
        } else if len <= 65535 {
            frame.push(0x80 | 126);
            frame.extend_from_slice(&(len as u16).to_be_bytes());
        } else {
            frame.push(0x80 | 127);
            frame.extend_from_slice(&(len as u64).to_be_bytes());
        }
        frame.extend_from_slice(&mask_key);
        for (i, &b) in buf.iter().enumerate() {
            frame.push(b ^ mask_key[i % 4]);
        }
        self.stream.write_all(&frame)?;
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        self.stream.flush()
    }
}

/// In-memory queue transport for simulated or driver-backed serial streams (BLE, WinUSB).
pub struct DriverTransport {
    pub label: String,
    pub in_buf: std::collections::VecDeque<u8>,
    pub out_buf: Vec<u8>,
}

impl DriverTransport {
    pub fn new(label: impl Into<String>) -> Self {
        Self {
            label: label.into(),
            in_buf: std::collections::VecDeque::new(),
            out_buf: Vec::new(),
        }
    }
}

impl Read for DriverTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.in_buf.is_empty() {
            return Err(io::ErrorKind::TimedOut.into());
        }
        let n = buf.len().min(self.in_buf.len());
        for i in 0..n {
            buf[i] = self.in_buf.pop_front().unwrap();
        }
        Ok(n)
    }
}

impl Write for DriverTransport {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.out_buf.extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
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
            Err(mavlink::error::MessageReadError::Io(e)) if crate::comms::is_timeout(&e) => {}
            Err(mavlink::error::MessageReadError::Io(_)) => return false,
            Err(_) => {}
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn connection_spec_display_and_transports() {
        assert_eq!(ConnectionSpec::WebSocket { url: "ws://127.0.0.1:8080".into() }.to_string(), "ws:ws://127.0.0.1:8080");
        assert_eq!(ConnectionSpec::Ble { address: "AA:BB:CC:DD:EE:FF".into() }.to_string(), "ble:AA:BB:CC:DD:EE:FF");
        assert_eq!(ConnectionSpec::WinUsb { path: "USB#VID_1234&PID_5678".into() }.to_string(), "winusb:USB#VID_1234&PID_5678");

        // Ble transport driver test
        let mut ble = open(&ConnectionSpec::Ble { address: "11:22:33:44:55:66".into() }).unwrap();
        assert_eq!(ble.write(b"mavlink-packet").unwrap(), 14);

        // WinUsb transport driver test
        let mut usb = open(&ConnectionSpec::WinUsb { path: "DEVICE_PATH".into() }).unwrap();
        assert_eq!(usb.write(b"winusb-data").unwrap(), 11);

        // Blank addresses rejected with InvalidInput
        assert!(open(&ConnectionSpec::Ble { address: "".into() }).is_err());
        assert!(open(&ConnectionSpec::WinUsb { path: " ".into() }).is_err());
    }
}

