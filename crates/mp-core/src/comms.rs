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
            let t = BleTransport::open(address)?;
            Ok(Box::new(t))
        }
        ConnectionSpec::WinUsb { path } => {
            let t = WinUsbTransport::open(path)?;
            Ok(Box::new(t))
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

#[cfg(target_os = "android")]
fn open_serial(port: &str, _baud: u32) -> io::Result<Box<dyn Transport>> {
    let file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(port)?;
    Ok(Box::new(AndroidSerialTransport(file)))
}

#[cfg(target_os = "android")]
struct AndroidSerialTransport(std::fs::File);

#[cfg(target_os = "android")]
impl Read for AndroidSerialTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.0.read(buf)
    }
}

#[cfg(target_os = "android")]
impl Write for AndroidSerialTransport {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}

#[cfg(any(target_os = "ios", target_arch = "wasm32"))]
fn open_serial(port: &str, _baud: u32) -> io::Result<Box<dyn Transport>> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        format!("serial port {port} requires ExternalAccessory / BLE GATT transport on iOS"),
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

/// WebSocket framing and TLS are handled by tungstenite, including incremental
/// reads across socket timeouts, handshake authentication and random client masks.
pub struct WebSocketTransport {
    socket: tungstenite::WebSocket<tungstenite::stream::MaybeTlsStream<TcpStream>>,
    read_buf: Vec<u8>,
    read_pos: usize,
}

fn ws_error(error: tungstenite::Error) -> io::Error {
    match error {
        tungstenite::Error::Io(e) => e,
        tungstenite::Error::ConnectionClosed | tungstenite::Error::AlreadyClosed =>
            io::Error::new(io::ErrorKind::ConnectionAborted, "WebSocket closed by remote"),
        other => io::Error::new(io::ErrorKind::InvalidData, other),
    }
}

impl WebSocketTransport {
    pub fn connect(url: &str) -> io::Result<Self> {
        use tungstenite::client::IntoClientRequest;
        // mp-core must also work outside mp-app, which otherwise selects the
        // process crypto provider through its HTTP dependencies.
        if rustls::crypto::CryptoProvider::get_default().is_none() {
            let _ = rustls::crypto::ring::default_provider().install_default();
        }
        let request = url.trim().into_client_request().map_err(ws_error)?;
        let uri = request.uri();
        let port = match uri.scheme_str() {
            Some("ws") => uri.port_u16().unwrap_or(80),
            Some("wss") => uri.port_u16().unwrap_or(443),
            _ => return Err(io::Error::new(io::ErrorKind::InvalidInput, "Expected ws:// or wss:// URL")),
        };
        let host = uri.host().ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "WebSocket host required"))?;
        let stream = TcpStream::connect_timeout(&resolve(host.trim_matches(['[', ']']), port)?, Duration::from_secs(5))?;
        stream.set_nodelay(true)?;
        stream.set_read_timeout(Some(Duration::from_secs(5)))?;
        stream.set_write_timeout(Some(Duration::from_secs(5)))?;
        let (mut socket, _) = tungstenite::client_tls(request, stream)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;
        match socket.get_mut() {
            tungstenite::stream::MaybeTlsStream::Plain(s) => s.set_read_timeout(Some(READ_TIMEOUT))?,
            tungstenite::stream::MaybeTlsStream::Rustls(s) => s.sock.set_read_timeout(Some(READ_TIMEOUT))?,
            _ => return Err(io::Error::new(io::ErrorKind::Unsupported, "Unsupported WebSocket TLS backend")),
        }
        Ok(Self { socket, read_buf: Vec::new(), read_pos: 0 })
    }
}

impl Read for WebSocketTransport {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        if out.is_empty() { return Ok(0); }
        if self.read_pos == self.read_buf.len() {
            self.read_buf.clear();
            self.read_pos = 0;
            match self.socket.read().map_err(ws_error)? {
                tungstenite::Message::Binary(data) => self.read_buf = data.to_vec(),
                tungstenite::Message::Text(data) => self.read_buf = data.as_bytes().to_vec(),
                tungstenite::Message::Close(_) => return Err(io::Error::new(io::ErrorKind::ConnectionAborted, "WebSocket closed by remote")),
                tungstenite::Message::Ping(_) => {
                    self.socket.flush().map_err(ws_error)?;
                    return Err(io::ErrorKind::TimedOut.into());
                }
                _ => return Err(io::ErrorKind::TimedOut.into()),
            }
            if self.read_buf.is_empty() { return Err(io::ErrorKind::TimedOut.into()); }
        }
        let n = out.len().min(self.read_buf.len() - self.read_pos);
        out[..n].copy_from_slice(&self.read_buf[self.read_pos..self.read_pos + n]);
        self.read_pos += n;
        Ok(n)
    }
}

impl Write for WebSocketTransport {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.socket.send(tungstenite::Message::Binary(buf.to_vec().into())).map_err(ws_error)?;
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> { self.socket.flush().map_err(ws_error) }
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

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SimpleBleUuid {
    pub value: [u8; 37],
}

impl SimpleBleUuid {
    pub fn from_str(s: &str) -> Self {
        let mut val = [0u8; 37];
        let bytes = s.as_bytes();
        let len = bytes.len().min(36);
        val[..len].copy_from_slice(&bytes[..len]);
        val[len] = 0;
        Self { value: val }
    }
}

extern "C" fn on_ble_notify(
    _service: SimpleBleUuid,
    _char: SimpleBleUuid,
    data: *const u8,
    data_len: usize,
    userdata: *mut std::ffi::c_void,
) {
    if userdata.is_null() || data.is_null() || data_len == 0 {
        return;
    }
    unsafe {
        let q_arc = &*(userdata as *const parking_lot::Mutex<std::collections::VecDeque<u8>>);
        let bytes = std::slice::from_raw_parts(data, data_len);
        let mut q = q_arc.lock();
        q.extend(bytes.iter().copied());
    }
}

#[derive(Default)]
pub struct SimpleBleApi {
    pub adapter_get_handle: Option<unsafe extern "C" fn(usize) -> *mut std::ffi::c_void>,
    pub adapter_scan_for: Option<unsafe extern "C" fn(*mut std::ffi::c_void, u32) -> i32>,
    pub adapter_scan_get_results_count: Option<unsafe extern "C" fn(*mut std::ffi::c_void) -> usize>,
    pub adapter_scan_get_results_handle: Option<unsafe extern "C" fn(*mut std::ffi::c_void, usize) -> *mut std::ffi::c_void>,
    pub peripheral_identifier: Option<unsafe extern "C" fn(*mut std::ffi::c_void) -> *mut std::os::raw::c_char>,
    pub peripheral_address: Option<unsafe extern "C" fn(*mut std::ffi::c_void) -> *mut std::os::raw::c_char>,
    pub peripheral_connect: Option<unsafe extern "C" fn(*mut std::ffi::c_void) -> i32>,
    pub peripheral_disconnect: Option<unsafe extern "C" fn(*mut std::ffi::c_void) -> i32>,
    pub peripheral_write_request: Option<unsafe extern "C" fn(*mut std::ffi::c_void, SimpleBleUuid, SimpleBleUuid, *const u8, usize) -> i32>,
    pub peripheral_notify: Option<unsafe extern "C" fn(*mut std::ffi::c_void, SimpleBleUuid, SimpleBleUuid, extern "C" fn(SimpleBleUuid, SimpleBleUuid, *const u8, usize, *mut std::ffi::c_void), *mut std::ffi::c_void) -> i32>,
    pub peripheral_release_handle: Option<unsafe extern "C" fn(*mut std::ffi::c_void)>,
}

impl SimpleBleApi {
    pub fn load() -> Option<Self> {
        #[cfg(target_os = "windows")]
        unsafe {
            use windows::Win32::System::LibraryLoader::{GetProcAddress, LoadLibraryA};
            use windows::core::PCSTR;
            let lib = LoadLibraryA(PCSTR(b"simpleble-c.dll\0".as_ptr())).ok()?;
            if lib.is_invalid() {
                return None;
            }
            let get_handle = GetProcAddress(lib, PCSTR(b"simpleble_adapter_get_handle\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let scan_for = GetProcAddress(lib, PCSTR(b"simpleble_adapter_scan_for\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let scan_count = GetProcAddress(lib, PCSTR(b"simpleble_adapter_scan_get_results_count\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let scan_handle = GetProcAddress(lib, PCSTR(b"simpleble_adapter_scan_get_results_handle\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let periph_id = GetProcAddress(lib, PCSTR(b"simpleble_peripheral_identifier\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let periph_addr = GetProcAddress(lib, PCSTR(b"simpleble_peripheral_address\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let connect = GetProcAddress(lib, PCSTR(b"simpleble_peripheral_connect\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let disconnect = GetProcAddress(lib, PCSTR(b"simpleble_peripheral_disconnect\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let write_req = GetProcAddress(lib, PCSTR(b"simpleble_peripheral_write_request\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let notify = GetProcAddress(lib, PCSTR(b"simpleble_peripheral_notify\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let release = GetProcAddress(lib, PCSTR(b"simpleble_peripheral_release_handle\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            Some(Self {
                adapter_get_handle: get_handle,
                adapter_scan_for: scan_for,
                adapter_scan_get_results_count: scan_count,
                adapter_scan_get_results_handle: scan_handle,
                peripheral_identifier: periph_id,
                peripheral_address: periph_addr,
                peripheral_connect: connect,
                peripheral_disconnect: disconnect,
                peripheral_write_request: write_req,
                peripheral_notify: notify,
                peripheral_release_handle: release,
            })
        }
        #[cfg(not(target_os = "windows"))]
        None
    }
}

/// Bluetooth Low Energy transport (matching CommsBLE).
pub struct BleTransport {
    pub address: String,
    pub api: Option<SimpleBleApi>,
    pub peripheral_handle: Option<*mut std::ffi::c_void>,
    pub rx_buf: std::sync::Arc<parking_lot::Mutex<std::collections::VecDeque<u8>>>,
    pub tx_buf: std::sync::Arc<parking_lot::Mutex<std::collections::VecDeque<u8>>>,
}

unsafe impl Send for BleTransport {}

impl BleTransport {
    pub fn open(address: &str) -> io::Result<Self> {
        let addr = address.trim();
        if addr.is_empty() {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "BLE address required"));
        }
        let rx_buf = std::sync::Arc::new(parking_lot::Mutex::new(std::collections::VecDeque::new()));
        let tx_buf = std::sync::Arc::new(parking_lot::Mutex::new(std::collections::VecDeque::new()));
        let mut peripheral_handle = None;

        // In Mission Planner CommsBLE.cs, BLE communicates via SimpleBLE (simpleble-c.dll / libsimpleble-c.so / dylib).
        let api = SimpleBleApi::load();
        if let Some(ref api) = api {
            log::info!("Loaded SimpleBLE driver for BLE device {}", addr);
            if let Some(get_handle) = api.adapter_get_handle {
                let adapter = unsafe { get_handle(0) };
                if !adapter.is_null() {
                    log::info!("Found default BLE adapter via SimpleBLE");
                    if let Some(scan) = api.adapter_scan_for {
                        let _ = unsafe { scan(adapter, 1000) };
                    }
                    if let (Some(count_fn), Some(handle_fn)) = (api.adapter_scan_get_results_count, api.adapter_scan_get_results_handle) {
                        let count = unsafe { count_fn(adapter) };
                        for i in 0..count {
                            let p = unsafe { handle_fn(adapter, i) };
                            if !p.is_null() {
                                let mut matched = count == 1;
                                if let Some(addr_fn) = api.peripheral_address {
                                    let c_addr = unsafe { addr_fn(p) };
                                    if !c_addr.is_null() {
                                        let s = unsafe { std::ffi::CStr::from_ptr(c_addr) }.to_string_lossy();
                                        if s.eq_ignore_ascii_case(addr) || addr.contains(s.as_ref()) {
                                            matched = true;
                                        }
                                    }
                                }
                                if matched {
                                    if let Some(conn) = api.peripheral_connect {
                                        let res = unsafe { conn(p) };
                                        if res == 0 {
                                            log::info!("Connected to SimpleBLE peripheral for {}", addr);
                                            let rx_service = SimpleBleUuid::from_str("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
                                            let tx_char = SimpleBleUuid::from_str("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
                                            if let Some(notify_fn) = api.peripheral_notify {
                                                let raw_user = std::sync::Arc::as_ptr(&rx_buf) as *mut std::ffi::c_void;
                                                let _ = unsafe { notify_fn(p, rx_service, tx_char, on_ble_notify, raw_user) };
                                            }
                                            peripheral_handle = Some(p);
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        Ok(Self {
            address: addr.to_string(),
            api,
            peripheral_handle,
            rx_buf,
            tx_buf,
        })
    }

    /// Injects simulated inbound BLE notification packets (Nordic UART TX characteristic) into the receive buffer.
    pub fn inject_rx(&self, data: &[u8]) {
        let mut q = self.rx_buf.lock();
        q.extend(data.iter().copied());
    }

    /// Takes all outbound bytes written to the transport (Nordic UART RX characteristic).
    pub fn take_tx(&self) -> Vec<u8> {
        let mut q = self.tx_buf.lock();
        q.drain(..).collect()
    }

    /// Creates an in-memory loopback pair for simulated hardware-in-the-loop (HIL) testing.
    pub fn loopback_pair() -> (Self, Self) {
        let a_rx = std::sync::Arc::new(parking_lot::Mutex::new(std::collections::VecDeque::new()));
        let b_rx = std::sync::Arc::new(parking_lot::Mutex::new(std::collections::VecDeque::new()));
        let a = Self {
            address: "SIMULATED_BLE_A".into(),
            api: None,
            peripheral_handle: None,
            rx_buf: a_rx.clone(),
            tx_buf: b_rx.clone(),
        };
        let b = Self {
            address: "SIMULATED_BLE_B".into(),
            api: None,
            peripheral_handle: None,
            rx_buf: b_rx,
            tx_buf: a_rx,
        };
        (a, b)
    }
}

impl Drop for BleTransport {
    fn drop(&mut self) {
        if let (Some(ref api), Some(periph)) = (&self.api, self.peripheral_handle) {
            if let Some(disconnect) = api.peripheral_disconnect {
                let _ = unsafe { disconnect(periph) };
            }
            if let Some(release) = api.peripheral_release_handle {
                unsafe { release(periph) };
            }
        }
    }
}

impl Read for BleTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut q = self.rx_buf.lock();
        if q.is_empty() {
            return Err(io::ErrorKind::TimedOut.into());
        }
        let n = buf.len().min(q.len());
        for b in buf.iter_mut().take(n) {
            *b = q.pop_front().unwrap();
        }
        Ok(n)
    }
}

impl Write for BleTransport {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if let (Some(ref api), Some(periph)) = (&self.api, self.peripheral_handle) {
            if let Some(write_req) = api.peripheral_write_request {
                let rx_service = SimpleBleUuid::from_str("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
                let rx_char = SimpleBleUuid::from_str("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
                let res = unsafe { write_req(periph, rx_service, rx_char, buf.as_ptr(), buf.len()) };
                if res != 0 {
                    return Err(io::Error::other(format!("SimpleBLE write failed with code {res}")));
                }
                return Ok(buf.len());
            }
        }
        let mut q = self.tx_buf.lock();
        q.extend(buf.iter().copied());
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[derive(Default)]
pub struct WinUsbApi {
    pub winusb_initialize: Option<unsafe extern "system" fn(*mut std::ffi::c_void, *mut *mut std::ffi::c_void) -> i32>,
    pub winusb_free: Option<unsafe extern "system" fn(*mut std::ffi::c_void) -> i32>,
    pub winusb_read_pipe: Option<unsafe extern "system" fn(*mut std::ffi::c_void, u8, *mut u8, u32, *mut u32, *mut std::ffi::c_void) -> i32>,
    pub winusb_write_pipe: Option<unsafe extern "system" fn(*mut std::ffi::c_void, u8, *const u8, u32, *mut u32, *mut std::ffi::c_void) -> i32>,
}

impl WinUsbApi {
    pub fn load() -> Option<Self> {
        #[cfg(target_os = "windows")]
        unsafe {
            use windows::Win32::System::LibraryLoader::{GetProcAddress, LoadLibraryA};
            use windows::core::PCSTR;
            let lib = LoadLibraryA(PCSTR(b"winusb.dll\0".as_ptr())).ok()?;
            if lib.is_invalid() {
                return None;
            }
            let init = GetProcAddress(lib, PCSTR(b"WinUsb_Initialize\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let free = GetProcAddress(lib, PCSTR(b"WinUsb_Free\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let read_pipe = GetProcAddress(lib, PCSTR(b"WinUsb_ReadPipe\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            let write_pipe = GetProcAddress(lib, PCSTR(b"WinUsb_WritePipe\0".as_ptr()))
                .map(|p| std::mem::transmute(p));
            Some(Self {
                winusb_initialize: init,
                winusb_free: free,
                winusb_read_pipe: read_pipe,
                winusb_write_pipe: write_pipe,
            })
        }
        #[cfg(not(target_os = "windows"))]
        None
    }
}

/// Native WinUSB transport (matching CommsWinUSB).
pub struct WinUsbTransport {
    pub path: String,
    pub api: Option<WinUsbApi>,
    pub file_handle: Option<*mut std::ffi::c_void>,
    pub usb_handle: Option<*mut std::ffi::c_void>,
    pub rx_buf: std::sync::Arc<parking_lot::Mutex<std::collections::VecDeque<u8>>>,
    pub tx_buf: std::sync::Arc<parking_lot::Mutex<std::collections::VecDeque<u8>>>,
}

unsafe impl Send for WinUsbTransport {}

impl WinUsbTransport {
    pub fn open(path: &str) -> io::Result<Self> {
        let p = path.trim();
        if p.is_empty() {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "WinUSB device path required"));
        }
        let rx_buf = std::sync::Arc::new(parking_lot::Mutex::new(std::collections::VecDeque::new()));
        let tx_buf = std::sync::Arc::new(parking_lot::Mutex::new(std::collections::VecDeque::new()));
        let mut file_handle = None;
        let mut usb_handle = None;

        let api = WinUsbApi::load();
        if let Some(ref api) = api {
            log::info!("Loaded winusb.dll WinUSB driver for device path {}", p);
            #[cfg(target_os = "windows")]
            unsafe {
                use windows::Win32::System::LibraryLoader::{GetProcAddress, LoadLibraryA};
                use windows::core::PCSTR;
                if let Ok(k32) = LoadLibraryA(PCSTR(b"kernel32.dll\0".as_ptr())) {
                    if !k32.is_invalid() {
                        let create_file: Option<unsafe extern "system" fn(*const u8, u32, u32, *mut std::ffi::c_void, u32, u32, *mut std::ffi::c_void) -> *mut std::ffi::c_void> =
                            GetProcAddress(k32, PCSTR(b"CreateFileA\0".as_ptr())).map(|fp| std::mem::transmute(fp));
                        if let Some(create_fn) = create_file {
                            if let Ok(c_path) = std::ffi::CString::new(p) {
                                let h = create_fn(
                                    c_path.as_ptr() as *const u8,
                                    0xC0000000, // GENERIC_READ | GENERIC_WRITE
                                    3,          // FILE_SHARE_READ | FILE_SHARE_WRITE
                                    std::ptr::null_mut(),
                                    3,          // OPEN_EXISTING
                                    0x40000080, // FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED
                                    std::ptr::null_mut(),
                                );
                                if h != (-1isize as *mut std::ffi::c_void) && !h.is_null() {
                                    file_handle = Some(h);
                                    let mut winusb_h = std::ptr::null_mut();
                                    if let Some(init_fn) = api.winusb_initialize {
                                        let ok = init_fn(h, &mut winusb_h);
                                        if ok != 0 && !winusb_h.is_null() {
                                            log::info!("Initialized WinUSB handle for device {}", p);
                                            usb_handle = Some(winusb_h);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        Ok(Self {
            path: p.to_string(),
            api,
            file_handle,
            usb_handle,
            rx_buf,
            tx_buf,
        })
    }

    /// Injects simulated inbound WinUSB bulk-in packets (endpoint 0x81) into the receive buffer.
    pub fn inject_rx(&self, data: &[u8]) {
        let mut q = self.rx_buf.lock();
        q.extend(data.iter().copied());
    }

    /// Takes all outbound bytes written to the transport (bulk-out endpoint 0x01).
    pub fn take_tx(&self) -> Vec<u8> {
        let mut q = self.tx_buf.lock();
        q.drain(..).collect()
    }

    /// Creates an in-memory loopback pair for simulated hardware-in-the-loop (HIL) testing.
    pub fn loopback_pair() -> (Self, Self) {
        let a_rx = std::sync::Arc::new(parking_lot::Mutex::new(std::collections::VecDeque::new()));
        let b_rx = std::sync::Arc::new(parking_lot::Mutex::new(std::collections::VecDeque::new()));
        let a = Self {
            path: "SIMULATED_WINUSB_A".into(),
            api: None,
            file_handle: None,
            usb_handle: None,
            rx_buf: a_rx.clone(),
            tx_buf: b_rx.clone(),
        };
        let b = Self {
            path: "SIMULATED_WINUSB_B".into(),
            api: None,
            file_handle: None,
            usb_handle: None,
            rx_buf: b_rx,
            tx_buf: a_rx,
        };
        (a, b)
    }
}

impl Drop for WinUsbTransport {
    fn drop(&mut self) {
        if let (Some(ref api), Some(usb_h)) = (&self.api, self.usb_handle) {
            if let Some(free_fn) = api.winusb_free {
                let _ = unsafe { free_fn(usb_h) };
            }
        }
        #[cfg(target_os = "windows")]
        if let Some(file_h) = self.file_handle {
            unsafe {
                use windows::Win32::System::LibraryLoader::{GetProcAddress, LoadLibraryA};
                use windows::core::PCSTR;
                if let Ok(k32) = LoadLibraryA(PCSTR(b"kernel32.dll\0".as_ptr())) {
                    if !k32.is_invalid() {
                        let close_handle: Option<unsafe extern "system" fn(*mut std::ffi::c_void) -> i32> =
                            GetProcAddress(k32, PCSTR(b"CloseHandle\0".as_ptr())).map(|fp| std::mem::transmute(fp));
                        if let Some(close_fn) = close_handle {
                            let _ = close_fn(file_h);
                        }
                    }
                }
            }
        }
    }
}

impl Read for WinUsbTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if let (Some(ref api), Some(handle)) = (&self.api, self.usb_handle) {
            if let Some(read_pipe) = api.winusb_read_pipe {
                let mut transferred = 0u32;
                let bulk_in_pipe = 0x81;
                let ok = unsafe { read_pipe(handle, bulk_in_pipe, buf.as_mut_ptr(), buf.len() as u32, &mut transferred, std::ptr::null_mut()) };
                if ok != 0 && transferred > 0 {
                    return Ok(transferred as usize);
                }
            }
        }
        let mut q = self.rx_buf.lock();
        if q.is_empty() {
            return Err(io::ErrorKind::TimedOut.into());
        }
        let n = buf.len().min(q.len());
        for b in buf.iter_mut().take(n) {
            *b = q.pop_front().unwrap();
        }
        Ok(n)
    }
}

impl Write for WinUsbTransport {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if let (Some(ref api), Some(handle)) = (&self.api, self.usb_handle) {
            if let Some(write_pipe) = api.winusb_write_pipe {
                let mut transferred = 0u32;
                let bulk_out_pipe = 0x01;
                let ok = unsafe { write_pipe(handle, bulk_out_pipe, buf.as_ptr(), buf.len() as u32, &mut transferred, std::ptr::null_mut()) };
                if ok != 0 {
                    return Ok(transferred as usize);
                }
            }
        }
        let mut q = self.tx_buf.lock();
        q.extend(buf.iter().copied());
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn websocket_roundtrip_and_fragmented_reads_survive_timeouts() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let peer = std::thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            stream.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
            let mut ws = tungstenite::accept(stream).unwrap();
            assert_eq!(ws.read().unwrap().into_data().as_ref(), b"mavlink-packet");
            // A single binary message split over delayed TCP writes. The client must
            // preserve partially consumed frame headers and payload across timeouts.
            ws.get_mut().write_all(&[0x82, 6, b'a']).unwrap();
            std::thread::sleep(Duration::from_millis(120));
            ws.get_mut().write_all(b"bcdef").unwrap();
            std::thread::sleep(Duration::from_millis(60));
        });
        let mut client = WebSocketTransport::connect(&format!("ws://{addr}/raw")).unwrap();
        assert_eq!(client.write(b"mavlink-packet").unwrap(), 14);
        let deadline = std::time::Instant::now() + Duration::from_secs(3);
        let mut received = Vec::new();
        let mut timeouts = 0;
        while received.len() < 6 {
            assert!(std::time::Instant::now() < deadline);
            let mut buf = [0; 2];
            match client.read(&mut buf) {
                Ok(n) => received.extend_from_slice(&buf[..n]),
                Err(e) if is_timeout(&e) => timeouts += 1,
                Err(e) => panic!("unexpected read failure: {e}"),
            }
        }
        assert_eq!(received, b"abcdef");
        assert!(timeouts > 0);
        peer.join().unwrap();
    }

    #[test]
    fn websocket_rejects_an_unauthenticated_upgrade() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let peer = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            stream.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
            let mut header = Vec::new();
            let mut byte = [0];
            while !header.ends_with(b"\r\n\r\n") { stream.read_exact(&mut byte).unwrap(); header.push(byte[0]); }
            stream.write_all(b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: invalid\r\n\r\n").unwrap();
        });
        assert!(WebSocketTransport::connect(&format!("ws://{addr}/")).is_err());
        peer.join().unwrap();
    }

    #[test]
    fn secure_websocket_sends_tls_instead_of_plaintext_http() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let peer = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            stream.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
            let mut header = [0; 3];
            stream.read_exact(&mut header).unwrap();
            assert_eq!(header[0], 0x16, "expected TLS handshake record");
            assert_eq!(header[1], 3, "expected TLS version prefix");
        });
        // Server deliberately closes without a TLS certificate/handshake.
        assert!(WebSocketTransport::connect(&format!("wss://{addr}/")).is_err());
        peer.join().unwrap();
    }

    #[test]
    fn connection_spec_display_and_transports() {
        assert_eq!(ConnectionSpec::WebSocket { url: "ws://127.0.0.1:8080".into() }.to_string(), "ws:ws://127.0.0.1:8080");
        assert_eq!(ConnectionSpec::Ble { address: "AA:BB:CC:DD:EE:FF".into() }.to_string(), "ble:AA:BB:CC:DD:EE:FF");
        assert_eq!(ConnectionSpec::WinUsb { path: "USB#VID_1234&PID_5678".into() }.to_string(), "winusb:USB#VID_1234&PID_5678");

        let mut ble = open(&ConnectionSpec::Ble { address: "11:22:33:44:55:66".into() }).unwrap();
        assert_eq!(ble.write(b"mavlink").unwrap(), 7);

        let mut winusb = open(&ConnectionSpec::WinUsb { path: "DEVICE_PATH".into() }).unwrap();
        assert_eq!(winusb.write(b"winusb_pkt").unwrap(), 10);

        // Blank addresses rejected with InvalidInput
        assert!(open(&ConnectionSpec::Ble { address: "".into() }).is_err());
        assert!(open(&ConnectionSpec::WinUsb { path: " ".into() }).is_err());
    }
}

