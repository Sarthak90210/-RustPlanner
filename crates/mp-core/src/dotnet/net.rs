//! `System.Net.Sockets.TcpClient` / `UdpClient` / `IPEndPoint`, as far as the ported comms
//! classes use them.
//!
//! The emulation keeps the .NET observable behaviour those classes rely on:
//! - `Available` is the socket's FIONREAD count (`ioctlsocket` / `ioctl`), so UDP reports what
//!   the OS reports to .NET on the same platform.
//! - `Socket.Connected` reflects the most recent operation: true after a connect, false after a
//!   failed send/receive or a close; a receive of 0 bytes leaves it unchanged.
//! - `new UdpClient()` owns a socket that is not yet bound; `Send` binds it to an ephemeral
//!   port, `Receive` on an unbound socket throws, and `Close()` drops the socket (`Client`
//!   becomes null).

use std::io::{Read, Write};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, TcpStream, ToSocketAddrs, UdpSocket};
use std::sync::{Arc, Mutex};

use super::{Exception, Stream};

fn socket_error(e: std::io::Error) -> Exception {
    Exception::Socket(e.to_string())
}

#[cfg(windows)]
fn fionread_raw(s: std::os::windows::io::RawSocket) -> std::io::Result<u32> {
    use windows::Win32::Networking::WinSock::{ioctlsocket, FIONREAD, SOCKET};
    let mut n: u32 = 0;
    let r = unsafe { ioctlsocket(SOCKET(s as usize), FIONREAD, &mut n) };
    if r != 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(n)
}

#[cfg(unix)]
fn fionread_raw(fd: std::os::unix::io::RawFd) -> std::io::Result<u32> {
    let mut n: libc::c_int = 0;
    let r = unsafe { libc::ioctl(fd, libc::FIONREAD as _, &mut n) };
    if r != 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(n as u32)
}

#[cfg(windows)]
fn available_tcp(s: &TcpStream) -> std::io::Result<u32> {
    use std::os::windows::io::AsRawSocket;
    fionread_raw(s.as_raw_socket())
}
#[cfg(unix)]
fn available_tcp(s: &TcpStream) -> std::io::Result<u32> {
    use std::os::unix::io::AsRawFd;
    fionread_raw(s.as_raw_fd())
}
#[cfg(windows)]
fn available_udp(s: &UdpSocket) -> std::io::Result<u32> {
    use std::os::windows::io::AsRawSocket;
    fionread_raw(s.as_raw_socket())
}
#[cfg(unix)]
fn available_udp(s: &UdpSocket) -> std::io::Result<u32> {
    use std::os::unix::io::AsRawFd;
    fionread_raw(s.as_raw_fd())
}
/// Targets without sockets (wasm32): .NET has no runtime there; every socket call fails.
#[cfg(not(any(windows, unix)))]
fn available_tcp(_s: &TcpStream) -> std::io::Result<u32> {
    Err(std::io::Error::from(std::io::ErrorKind::Unsupported))
}
#[cfg(not(any(windows, unix)))]
fn available_udp(_s: &UdpSocket) -> std::io::Result<u32> {
    Err(std::io::Error::from(std::io::ErrorKind::Unsupported))
}

/// `System.Net.Sockets.AddressFamily`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressFamily {
    InterNetwork,
    InterNetworkV6,
}

/// `IPAddress.AddressFamily`
pub fn address_family(a: &IpAddr) -> AddressFamily {
    match a {
        IpAddr::V4(_) => AddressFamily::InterNetwork,
        IpAddr::V6(_) => AddressFamily::InterNetworkV6,
    }
}

/// `IPAddress.TryParse(s, out addr)`: IPv6 text, or IPv4 in any `inet_aton` form the .NET
/// parser accepts (`a`, `a.b`, `a.b.c`, `a.b.c.d`; decimal, `0` octal or `0x` hex parts).
pub fn ip_address_try_parse(s: &str) -> Option<IpAddr> {
    if s.contains(':') {
        let t = s.trim_start_matches('[');
        let t = t.split(']').next().unwrap_or(t);
        let t = t.split('%').next().unwrap_or(t);
        return t.parse::<std::net::Ipv6Addr>().ok().map(IpAddr::V6);
    }
    let parts: Vec<&str> = s.split('.').collect();
    if parts.is_empty() || parts.len() > 4 {
        return None;
    }
    let mut vals = vec![];
    for p in &parts {
        if p.is_empty() {
            return None;
        }
        let (digits, radix) = if let Some(h) = p.strip_prefix("0x").or_else(|| p.strip_prefix("0X")) {
            (h, 16)
        } else if p.len() > 1 && p.starts_with('0') {
            (&p[1..], 8)
        } else {
            (*p, 10)
        };
        let v = if digits.is_empty() { 0 } else { u64::from_str_radix(digits, radix).ok()? };
        vals.push(v);
    }
    let n = vals.len();
    for v in &vals[..n - 1] {
        if *v > 0xFF {
            return None;
        }
    }
    let last_max: u64 = match n {
        1 => 0xFFFF_FFFF,
        2 => 0xFF_FFFF,
        3 => 0xFFFF,
        _ => 0xFF,
    };
    if vals[n - 1] > last_max {
        return None;
    }
    let mut addr: u64 = vals[n - 1];
    for (i, v) in vals[..n - 1].iter().enumerate() {
        addr |= v << (24 - 8 * i);
    }
    Some(IpAddr::V4(Ipv4Addr::from(addr as u32)))
}

/// `IPAddress.Parse(s)`
pub fn ip_address_parse(s: &str) -> Result<IpAddr, Exception> {
    ip_address_try_parse(s).ok_or_else(|| Exception::Format("An invalid IP address was specified.".into()))
}

/// `IPAddress.GetAddressBytes()`
pub fn get_address_bytes(a: &IpAddr) -> Vec<u8> {
    match a {
        IpAddr::V4(v) => v.octets().to_vec(),
        IpAddr::V6(v) => v.octets().to_vec(),
    }
}

/// `Dns.GetHostEntry(host).AddressList`
pub fn dns_get_host_entry(host: &str) -> Result<Vec<IpAddr>, Exception> {
    let addrs: Vec<IpAddr> = (host, 0).to_socket_addrs().map_err(socket_error)?.map(|a| a.ip()).collect();
    if addrs.is_empty() {
        return Err(Exception::Socket("No such host is known".into()));
    }
    Ok(addrs)
}

/// `System.Net.IPEndPoint`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IPEndPoint {
    pub address: IpAddr,
    pub port: u16,
}

impl IPEndPoint {
    /// `new IPEndPoint(IPAddress.Any, 0)`
    pub fn any() -> Self {
        IPEndPoint { address: IpAddr::V4(Ipv4Addr::UNSPECIFIED), port: 0 }
    }

    pub fn new(address: IpAddr, port: u16) -> Self {
        IPEndPoint { address, port }
    }

    fn socket_addr(&self) -> SocketAddr {
        SocketAddr::new(self.address, self.port)
    }
}

impl From<SocketAddr> for IPEndPoint {
    fn from(a: SocketAddr) -> Self {
        IPEndPoint { address: a.ip(), port: a.port() }
    }
}

impl std::fmt::Display for IPEndPoint {
    /// `IPEndPoint.ToString()`: `address:port`
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}:{}", self.address, self.port)
    }
}

/// `Dns.GetHostAddresses` + connect to the first address that accepts, as `TcpClient.Connect`
/// does (IPv4 addresses first for an IPv4 client).
fn connect_host(host: &str, port: u16) -> Result<TcpStream, Exception> {
    let addrs: Vec<SocketAddr> = (host, port).to_socket_addrs().map_err(socket_error)?.collect();
    let mut last = None;
    for a in addrs.iter().filter(|a| a.is_ipv4()).chain(addrs.iter().filter(|a| a.is_ipv6())) {
        match TcpStream::connect(a) {
            Ok(s) => return Ok(s),
            Err(e) => last = Some(e),
        }
    }
    Err(match last {
        Some(e) => socket_error(e),
        None => Exception::Socket("No such host is known".into()),
    })
}

type PendingConnect = Arc<Mutex<Option<Result<TcpStream, Exception>>>>;

/// `System.Net.Sockets.TcpClient`
pub struct TcpClient {
    stream: Option<TcpStream>,
    connected: bool,
    disposed: bool,
    no_delay: bool,
    pending: Option<PendingConnect>,
}

impl Default for TcpClient {
    fn default() -> Self {
        Self::new()
    }
}

impl TcpClient {
    /// `new TcpClient()`: a socket that is not connected.
    pub fn new() -> Self {
        TcpClient { stream: None, connected: false, disposed: false, no_delay: false, pending: None }
    }

    /// `new TcpClient(host, port)`: resolves and connects, throwing SocketException on failure.
    pub fn connect_new(host: &str, port: i32) -> Result<Self, Exception> {
        let mut c = TcpClient::new();
        c.connect(host, port)?;
        Ok(c)
    }

    /// `Connect(host, port)`
    pub fn connect(&mut self, host: &str, port: i32) -> Result<(), Exception> {
        if self.disposed {
            return Err(Exception::ObjectDisposed("Cannot access a disposed object.\r\nObject name: 'System.Net.Sockets.TcpClient'.".into()));
        }
        let port = u16::try_from(port).map_err(|_| Exception::ArgumentOutOfRange(format!(
            "Specified argument was out of the range of valid values.\r\nParameter name: port"
        )))?;
        let s = connect_host(host, port)?;
        s.set_nodelay(self.no_delay).ok();
        self.stream = Some(s);
        self.connected = true;
        Ok(())
    }

    /// `ConnectAsync(host, port)`: the connect completes on another thread; `Connected` turns
    /// true when it succeeds.
    pub fn connect_async(&mut self, host: &str, port: i32) {
        let slot: PendingConnect = Arc::new(Mutex::new(None));
        let s2 = slot.clone();
        let host = host.to_string();
        std::thread::spawn(move || {
            let r = u16::try_from(port)
                .map_err(|_| Exception::ArgumentOutOfRange("port".into()))
                .and_then(|p| connect_host(&host, p));
            *s2.lock().unwrap_or_else(|e| e.into_inner()) = Some(r);
        });
        self.pending = Some(slot);
    }

    fn poll_pending(&mut self) {
        let done = match &self.pending {
            Some(p) => p.lock().unwrap_or_else(|e| e.into_inner()).take(),
            None => return,
        };
        if let Some(r) = done {
            self.pending = None;
            if let Ok(s) = r {
                s.set_nodelay(self.no_delay).ok();
                self.stream = Some(s);
                self.connected = true;
            }
        }
    }

    /// `client.Client != null`: false once disposed.
    pub fn has_client(&self) -> bool {
        !self.disposed
    }

    /// `Client.Connected`
    pub fn connected(&mut self) -> bool {
        self.poll_pending();
        self.connected && self.stream.is_some()
    }

    /// `NoDelay` / `Client.NoDelay`
    pub fn set_no_delay(&mut self, v: bool) {
        self.no_delay = v;
        if let Some(s) = &self.stream {
            s.set_nodelay(v).ok();
        }
    }

    /// `Available`
    pub fn available(&mut self) -> Result<i32, Exception> {
        self.poll_pending();
        match &self.stream {
            Some(s) => available_tcp(s).map(|n| n as i32).map_err(socket_error),
            None => Ok(0),
        }
    }

    /// `Client.RemoteEndPoint`
    pub fn remote_end_point(&self) -> Option<IPEndPoint> {
        self.stream.as_ref().and_then(|s| s.peer_addr().ok()).map(IPEndPoint::from)
    }

    /// `Client.Receive(buffer, offset, size, SocketFlags.None)`: blocks until data arrives.
    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        let Some(s) = self.stream.as_mut() else {
            return Err(Exception::Socket("A request to send or receive data was disallowed because the socket is not connected".into()));
        };
        match s.read(buffer) {
            Ok(n) => Ok(n),
            Err(e) => {
                self.connected = false;
                Err(socket_error(e))
            }
        }
    }

    /// `Client.Send(buffer, size, SocketFlags.None)`
    pub fn send(&mut self, buffer: &[u8]) -> Result<usize, Exception> {
        let Some(s) = self.stream.as_mut() else {
            return Err(Exception::Socket("A request to send or receive data was disallowed because the socket is not connected".into()));
        };
        match s.write_all(buffer) {
            Ok(()) => Ok(buffer.len()),
            Err(e) => {
                self.connected = false;
                Err(socket_error(e))
            }
        }
    }

    /// `GetStream()`: throws when the socket is not connected.
    pub fn get_stream(&mut self) -> Result<NetworkStream<'_>, Exception> {
        if self.disposed {
            return Err(Exception::ObjectDisposed("Cannot access a disposed object.\r\nObject name: 'System.Net.Sockets.TcpClient'.".into()));
        }
        if !self.connected() {
            return Err(Exception::InvalidOperation("The operation is not allowed on non-connected sockets.".into()));
        }
        Ok(NetworkStream { client: self })
    }

    /// A second handle on the connected socket, for a stream that must own its transport
    /// (`new SslStream(client.GetStream())`, `StreamReader`/`StreamWriter`).
    pub fn try_clone_stream(&mut self) -> Result<TcpStream, Exception> {
        if self.disposed {
            return Err(Exception::ObjectDisposed("Cannot access a disposed object.\r\nObject name: 'System.Net.Sockets.TcpClient'.".into()));
        }
        if !self.connected() {
            return Err(Exception::InvalidOperation("The operation is not allowed on non-connected sockets.".into()));
        }
        self.stream.as_ref().unwrap().try_clone().map_err(socket_error)
    }

    /// `Client.IOControl(IOControlCode.KeepAliveValues, TcpKeepAlive(on, time, interval), null)`:
    /// the Windows `SIO_KEEPALIVE_VALS` ioctl. Other platforms have no such control code and
    /// throw, as the C# does under Mono.
    pub fn set_keep_alive_values(&mut self, on: bool, keepalive_time_ms: u32, keepalive_interval_ms: u32) -> Result<(), Exception> {
        let Some(s) = self.stream.as_ref() else {
            return Err(Exception::Socket("A request to send or receive data was disallowed because the socket is not connected".into()));
        };
        keep_alive_values(s, on, keepalive_time_ms, keepalive_interval_ms)
    }

    /// `Dispose()` / `Client.Dispose()`
    pub fn dispose(&mut self) {
        if let Some(s) = self.stream.take() {
            let _ = s.shutdown(std::net::Shutdown::Both);
        }
        self.connected = false;
        self.disposed = true;
        self.pending = None;
    }
}

#[cfg(windows)]
fn keep_alive_values(s: &TcpStream, on: bool, keepalive_time_ms: u32, keepalive_interval_ms: u32) -> Result<(), Exception> {
            use std::os::windows::io::AsRawSocket;
            #[link(name = "ws2_32")]
            #[allow(non_snake_case)]
            extern "system" {
                fn WSAIoctl(
                    s: usize,
                    dwIoControlCode: u32,
                    lpvInBuffer: *const std::ffi::c_void,
                    cbInBuffer: u32,
                    lpvOutBuffer: *mut std::ffi::c_void,
                    cbOutBuffer: u32,
                    lpcbBytesReturned: *mut u32,
                    lpOverlapped: *mut std::ffi::c_void,
                    lpCompletionRoutine: *mut std::ffi::c_void,
                ) -> i32;
            }
            const SIO_KEEPALIVE_VALS: u32 = 0x9800_0004;
            let mut in_value = [0u8; 12];
            in_value[0..4].copy_from_slice(&(on as u32).to_le_bytes());
            in_value[4..8].copy_from_slice(&keepalive_time_ms.to_le_bytes());
            in_value[8..12].copy_from_slice(&keepalive_interval_ms.to_le_bytes());
            let mut ret = 0u32;
            let r = unsafe {
                WSAIoctl(
                    s.as_raw_socket() as usize,
                    SIO_KEEPALIVE_VALS,
                    in_value.as_ptr() as *const _,
                    12,
                    std::ptr::null_mut(),
                    0,
                    &mut ret,
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                )
            };
            if r != 0 {
                return Err(socket_error(std::io::Error::last_os_error()));
            }
            Ok(())
}

#[cfg(not(windows))]
fn keep_alive_values(_s: &TcpStream, _on: bool, _time: u32, _interval: u32) -> Result<(), Exception> {
    Err(Exception::NotSupported("The attempted operation is not supported for the type of object referenced".into()))
}

/// `System.Net.Sockets.NetworkStream` over a connected `TcpClient`.
pub struct NetworkStream<'a> {
    client: &'a mut TcpClient,
}

impl Stream for NetworkStream<'_> {
    fn can_timeout(&self) -> bool {
        true
    }
    /// The socket's `ReceiveTimeout` of 0 (infinite) reads back as `Timeout.Infinite`.
    fn read_timeout(&self) -> i32 {
        -1
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        self.client.receive(buffer).map_err(|e| Exception::Io(format!("Unable to read data from the transport connection: {}.", e.message())))
    }
    fn write(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        self.client
            .send(buffer)
            .map(|_| ())
            .map_err(|e| Exception::Io(format!("Unable to write data to the transport connection: {}.", e.message())))
    }
}

/// `System.Net.Sockets.UdpClient`
pub struct UdpClient {
    /// `Client`: false once closed.
    created: bool,
    family: AddressFamily,
    socket: Option<UdpSocket>,
    connected: bool,
}

impl Default for UdpClient {
    fn default() -> Self {
        Self::new()
    }
}

/// The family's any address (`IPAddress.Any` / `IPv6Any`), or loopback under
/// `MP_LOOPBACK_ONLY` (see `crate::comms::listen_ip`).
fn any_addr(family: AddressFamily) -> IpAddr {
    let loopback = crate::comms::listen_ip().is_loopback();
    match family {
        AddressFamily::InterNetwork => IpAddr::V4(crate::comms::listen_ip()),
        AddressFamily::InterNetworkV6 if loopback => IpAddr::V6(std::net::Ipv6Addr::LOCALHOST),
        AddressFamily::InterNetworkV6 => IpAddr::V6(std::net::Ipv6Addr::UNSPECIFIED),
    }
}

impl UdpClient {
    /// `new UdpClient()`: an IPv4 socket, bound on first use.
    pub fn new() -> Self {
        Self::with_family(AddressFamily::InterNetwork)
    }

    /// `new UdpClient(AddressFamily family)`
    pub fn with_family(family: AddressFamily) -> Self {
        UdpClient { created: true, family, socket: None, connected: false }
    }

    /// `new UdpClient(port)`: bound to `0.0.0.0:port`.
    pub fn bind(port: i32) -> Result<Self, Exception> {
        Self::bind_family(port, AddressFamily::InterNetwork)
    }

    /// `new UdpClient(port, AddressFamily family)`: bound to the family's any address.
    pub fn bind_family(port: i32, family: AddressFamily) -> Result<Self, Exception> {
        let port = u16::try_from(port)
            .map_err(|_| Exception::ArgumentOutOfRange("Specified argument was out of the range of valid values.\r\nParameter name: port".into()))?;
        let s = UdpSocket::bind((any_addr(family), port)).map_err(socket_error)?;
        Ok(UdpClient { created: true, family, socket: Some(s), connected: false })
    }

    /// `client.Client != null`
    pub fn has_client(&self) -> bool {
        self.created
    }

    /// `Client.Connected`
    pub fn connected(&self) -> bool {
        self.created && self.connected
    }

    fn null_ref() -> Exception {
        Exception::NullReference("Object reference not set to an instance of an object.".into())
    }

    fn disposed() -> Exception {
        Exception::ObjectDisposed("Cannot access a disposed object.\r\nObject name: 'System.Net.Sockets.UdpClient'.".into())
    }

    /// `Available`; 0 for a socket that is not bound yet.
    pub fn available(&self) -> Result<i32, Exception> {
        if !self.created {
            return Err(Self::null_ref());
        }
        match &self.socket {
            Some(s) => available_udp(s).map(|n| n as i32).map_err(socket_error),
            None => Ok(0),
        }
    }

    /// `Receive(ref IPEndPoint remoteEP)`: blocks for one datagram.
    pub fn receive(&mut self, remote_ep: &mut IPEndPoint) -> Result<Vec<u8>, Exception> {
        if !self.created {
            return Err(Self::null_ref());
        }
        let Some(s) = &self.socket else {
            return Err(Exception::InvalidOperation("You must call the Bind method before performing this operation.".into()));
        };
        let mut buf = vec![0u8; 65536];
        let (n, from) = s.recv_from(&mut buf).map_err(socket_error)?;
        buf.truncate(n);
        *remote_ep = IPEndPoint::from(from);
        Ok(buf)
    }

    fn ensure_bound(&mut self) -> Result<&UdpSocket, Exception> {
        if self.socket.is_none() {
            self.socket = Some(UdpSocket::bind((any_addr(self.family), 0)).map_err(socket_error)?);
        }
        Ok(self.socket.as_ref().unwrap())
    }

    /// `Send(byte[] dgram, int bytes, IPEndPoint endPoint)` and `Client.SendTo(buffer, size,
    /// SocketFlags.None, remoteEP)`: bind an ephemeral port first if needed; both send from index 0.
    pub fn send(&mut self, dgram: &[u8], bytes: usize, end_point: &IPEndPoint) -> Result<usize, Exception> {
        if !self.created {
            return Err(Self::disposed());
        }
        let s = self.ensure_bound()?;
        s.send_to(&dgram[..bytes], end_point.socket_addr()).map_err(socket_error)
    }

    /// `JoinMulticastGroup(IPAddress multicastAddr)`
    pub fn join_multicast_group(&mut self, multicast_addr: IpAddr) -> Result<(), Exception> {
        if !self.created {
            return Err(Self::disposed());
        }
        let s = self.ensure_bound()?;
        match multicast_addr {
            IpAddr::V4(a) => s.join_multicast_v4(&a, &Ipv4Addr::UNSPECIFIED),
            IpAddr::V6(a) => s.join_multicast_v6(&a, 0),
        }
        .map_err(socket_error)
    }

    /// `DropMulticastGroup(IPAddress multicastAddr)`
    pub fn drop_multicast_group(&mut self, multicast_addr: IpAddr) -> Result<(), Exception> {
        if !self.created {
            return Err(Self::disposed());
        }
        let Some(s) = &self.socket else { return Err(Exception::Socket("An invalid argument was supplied".into())) };
        match multicast_addr {
            IpAddr::V4(a) => s.leave_multicast_v4(&a, &Ipv4Addr::UNSPECIFIED),
            IpAddr::V6(a) => s.leave_multicast_v6(&a, 0),
        }
        .map_err(socket_error)
    }

    /// A second handle on the same OS socket, for work done on another thread.
    pub fn try_clone_socket(&self) -> Option<UdpSocket> {
        self.socket.as_ref().and_then(|s| s.try_clone().ok())
    }

    /// `Close()` / `Dispose()`: disposes the socket; `Client` becomes null.
    pub fn close(&mut self) {
        self.socket = None;
        self.created = false;
        self.connected = false;
    }

    /// The bound local end point, if any (`Client.LocalEndPoint`).
    pub fn local_end_point(&self) -> Option<IPEndPoint> {
        self.socket.as_ref().and_then(|s| s.local_addr().ok()).map(IPEndPoint::from)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ip_parsing_matches_dotnet() {
        assert_eq!(ip_address_try_parse("127.0.0.1"), Some(IpAddr::V4(Ipv4Addr::LOCALHOST)));
        assert_eq!(ip_address_try_parse("127.1"), Some(IpAddr::V4(Ipv4Addr::LOCALHOST)));
        assert_eq!(ip_address_try_parse("1"), Some(IpAddr::V4(Ipv4Addr::new(0, 0, 0, 1))));
        assert_eq!(ip_address_try_parse("0x7f.1"), Some(IpAddr::V4(Ipv4Addr::LOCALHOST)));
        assert_eq!(ip_address_try_parse("010.0.0.1"), Some(IpAddr::V4(Ipv4Addr::new(8, 0, 0, 1))));
        assert!(ip_address_try_parse("256.1.1.1").is_none());
        assert!(ip_address_try_parse("localhost").is_none());
        assert!(ip_address_try_parse("::1").is_some());
    }

    #[test]
    fn udp_unbound_then_send_binds() {
        let mut rx = UdpClient::bind(0).unwrap();
        let port = rx.local_end_point().unwrap().port;
        let mut tx = UdpClient::new();
        assert!(matches!(tx.receive(&mut IPEndPoint::any()), Err(Exception::InvalidOperation(_))));
        tx.send(b"hello", 3, &IPEndPoint::new(IpAddr::V4(Ipv4Addr::LOCALHOST), port)).unwrap();
        let mut from = IPEndPoint::any();
        let d = rx.receive(&mut from).unwrap();
        assert_eq!(d, b"hel");
        assert_eq!(from.port, tx.local_end_point().unwrap().port);
        tx.close();
        assert!(!tx.has_client());
    }

    #[test]
    fn tcp_available_and_connected() {
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = l.local_addr().unwrap().port();
        let mut c = TcpClient::connect_new("127.0.0.1", port as i32).unwrap();
        let (mut srv, _) = l.accept().unwrap();
        assert!(c.connected());
        srv.write_all(b"abcd").unwrap();
        let t0 = std::time::Instant::now();
        while c.available().unwrap() < 4 && t0.elapsed().as_secs() < 2 {
            std::thread::yield_now();
        }
        assert_eq!(c.available().unwrap(), 4);
        let mut b = [0u8; 4];
        assert_eq!(c.receive(&mut b).unwrap(), 4);
        c.dispose();
        assert!(!c.connected());
        assert!(!c.has_client());
    }
}
