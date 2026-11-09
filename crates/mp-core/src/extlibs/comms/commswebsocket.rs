//! Port of `ExtLibs/Comms/CommsWebSocket.cs`
//!
//! `System.Net.WebSockets.ClientWebSocket` is emulated over `tungstenite`. The reader thread
//! (the C# `Task.Run(RunReader)`) and writers share the socket behind a lock; the socket's read
//! timeout is short so a waiting reader gives the lock up to writers.
//!
//! The C# `Open(string url)` is `async void`, so a failing connect never reaches the caller of
//! `Open()`: the exception is raised on the synchronisation context instead. Here it is logged,
//! `Open()` returns normally and `IsOpen` stays false, which is what the caller observes.

use std::any::Any;
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::commsbase::{inputboxreturn, CommsBase};
use super::commsstream::CommsStream;
use crate::dotnet::{ascii_get_bytes, ascii_get_string, Exception, MemoryStream, SeekOrigin, Stream};
use crate::extlibs::interfaces::ICommsSerial;

const LOG: &str = "MissionPlanner.Comms.WebSocket";

type Ws = tungstenite::WebSocket<tungstenite::stream::MaybeTlsStream<TcpStream>>;

/// `System.Net.WebSockets.WebSocketState`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WebSocketState {
    None,
    Connecting,
    Open,
    CloseSent,
    CloseReceived,
    Closed,
    Aborted,
}

/// `System.Net.WebSockets.ClientWebSocket`
pub struct ClientWebSocket {
    ws: Option<Ws>,
    state: WebSocketState,
}

impl ClientWebSocket {
    pub fn new() -> Self {
        ClientWebSocket { ws: None, state: WebSocketState::None }
    }

    pub fn state(&self) -> WebSocketState {
        self.state
    }

    /// `ConnectAsync(uri, ct).GetAwaiter().GetResult()`
    pub fn connect(&mut self, url: &str) -> Result<(), Exception> {
        use tungstenite::client::IntoClientRequest;
        if rustls::crypto::CryptoProvider::get_default().is_none() {
            let _ = rustls::crypto::ring::default_provider().install_default();
        }
        // new Uri(url)
        let request = url
            .into_client_request()
            .map_err(|_| Exception::UriFormat("Invalid URI: The format of the URI could not be determined.".into()))?;
        let uri = request.uri().clone();
        let port = match uri.scheme_str() {
            Some("ws") => uri.port_u16().unwrap_or(80),
            Some("wss") => uri.port_u16().unwrap_or(443),
            _ => {
                return Err(Exception::Argument(
                    "Only Uris starting with 'ws://' or 'wss://' are supported.\r\nParameter name: uri".into(),
                ))
            }
        };
        let host = uri.host().unwrap_or("").trim_matches(['[', ']']).to_string();
        self.state = WebSocketState::Connecting;
        let r = (|| -> Result<Ws, Exception> {
            let addr = (host.as_str(), port)
                .to_socket_addrs()
                .map_err(|e| Exception::WebSocket(format!("Unable to connect to the remote server: {e}")))?
                .next()
                .ok_or_else(|| Exception::WebSocket("Unable to connect to the remote server".into()))?;
            let stream =
                TcpStream::connect(addr).map_err(|e| Exception::WebSocket(format!("Unable to connect to the remote server: {e}")))?;
            stream.set_nodelay(true).ok();
            let (mut socket, _) = tungstenite::client_tls(request, stream)
                .map_err(|e| Exception::WebSocket(format!("Unable to connect to the remote server: {e}")))?;
            match socket.get_mut() {
                tungstenite::stream::MaybeTlsStream::Plain(s) => s.set_read_timeout(Some(Duration::from_millis(50))).ok(),
                tungstenite::stream::MaybeTlsStream::Rustls(s) => s.sock.set_read_timeout(Some(Duration::from_millis(50))).ok(),
                _ => None,
            };
            Ok(socket)
        })();
        match r {
            Ok(ws) => {
                self.ws = Some(ws);
                self.state = WebSocketState::Open;
                Ok(())
            }
            Err(e) => {
                self.state = WebSocketState::Closed;
                Err(e)
            }
        }
    }

    /// `SendAsync(data, type, endOfMessage: true, ct)`
    pub fn send(&mut self, data: &[u8], text: bool) -> Result<(), Exception> {
        if self.state != WebSocketState::Open {
            return Err(Exception::WebSocket(format!(
                "The WebSocket is in an invalid state ('{:?}') for this operation. Valid states are: 'Open, CloseReceived'",
                self.state
            )));
        }
        let ws = self.ws.as_mut().unwrap();
        let msg = if text {
            tungstenite::Message::Text(String::from_utf8_lossy(data).into_owned().into())
        } else {
            tungstenite::Message::Binary(data.to_vec().into())
        };
        ws.send(msg).map_err(|e| {
            self.state = WebSocketState::Aborted;
            Exception::WebSocket(e.to_string())
        })
    }

    /// One `ReceiveAsync`; `Ok(None)` when nothing arrived within the socket's read timeout.
    fn receive(&mut self) -> Result<Option<tungstenite::Message>, Exception> {
        let Some(ws) = self.ws.as_mut() else {
            return Err(Exception::WebSocket("The WebSocket is not connected.".into()));
        };
        match ws.read() {
            Ok(m) => Ok(Some(m)),
            Err(tungstenite::Error::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock || e.kind() == std::io::ErrorKind::TimedOut => {
                Ok(None)
            }
            Err(e) => {
                self.state = WebSocketState::Aborted;
                Err(Exception::WebSocket(e.to_string()))
            }
        }
    }

    /// `Dispose()`
    pub fn dispose(&mut self) {
        if let Some(mut ws) = self.ws.take() {
            let _ = ws.close(None);
        }
        if self.state != WebSocketState::None {
            self.state = WebSocketState::Closed;
        }
    }
}

impl Default for ClientWebSocket {
    fn default() -> Self {
        Self::new()
    }
}

struct Shared {
    /// `client`; `None` after `Dispose()`.
    client: Mutex<Option<ClientWebSocket>>,
    /// `readMemoryStream` and `readCursor`, both under `lock (readMemoryStream)`.
    read: Mutex<(MemoryStream, i32)>,
    auto_reconnect: AtomicBool,
    socketio: AtomicBool,
    url: Mutex<String>,
}

impl Shared {
    fn is_open(&self) -> bool {
        self.client.lock().unwrap_or_else(|e| e.into_inner()).as_ref().is_some_and(|c| c.state() == WebSocketState::Open)
    }

    /// `private async void Open(string url)`
    fn open_url(&self, url: &str) {
        *self.url.lock().unwrap_or_else(|e| e.into_inner()) = url.to_string();
        let mut c = ClientWebSocket::new();
        let r = c.connect(url);
        let ok = r.is_ok();
        *self.client.lock().unwrap_or_else(|e| e.into_inner()) = Some(c);
        match r {
            Err(e) => log::error!(target: LOG, "{e}"),
            Ok(()) => {
                log::info!(target: LOG, "WS opened System.Net.WebSockets.ClientWebSocket");
                // https://github.com/socketio/engine.io-protocol  socket.io
                if ok {
                    let mut g = self.client.lock().unwrap_or_else(|e| e.into_inner());
                    if let Some(c) = g.as_mut() {
                        let _ = c.send(&ascii_get_bytes("2probe"), true);
                    }
                }
            }
        }
    }

    /// `RunReader()`
    fn run_reader(&self) {
        while self.is_open() {
            let r = {
                let mut g = self.client.lock().unwrap_or_else(|e| e.into_inner());
                match g.as_mut() {
                    Some(c) => c.receive(),
                    None => Err(Exception::NullReference("Object reference not set to an instance of an object.".into())),
                }
            };
            let result = r.and_then(|m| {
                let Some(m) = m else { return Ok(()) };
                match m {
                    tungstenite::Message::Binary(data) => {
                        let mut rd = self.read.lock().unwrap_or_else(|e| e.into_inner());
                        rd.0.seek(0, SeekOrigin::End)?;
                        // ReceiveAsync fills an 8 KiB buffer per call; the chunks land back to back.
                        rd.0.write_from(&data, 0, data.len());
                        Ok(())
                    }
                    tungstenite::Message::Text(t) => {
                        let stringdata = ascii_get_string(t.as_bytes());
                        log::info!(target: LOG, "{stringdata}");
                        match stringdata.chars().next() {
                            None => Err(Exception::IndexOutOfRange("Index was outside the bounds of the array.".into())),
                            Some('0') => {
                                let re = regex::Regex::new(r#""sid":"([^"]+)""#).unwrap();
                                if let Some(mat) = re.captures(&stringdata) {
                                    self.url.lock().unwrap_or_else(|e| e.into_inner()).push_str(&format!("&sid={}", &mat[1]));
                                    self.socketio.store(true, Ordering::SeqCst);
                                }
                                Ok(())
                            }
                            Some('3') => {
                                let mut g = self.client.lock().unwrap_or_else(|e| e.into_inner());
                                if let Some(c) = g.as_mut() {
                                    let _ = c.send(&ascii_get_bytes("40/MAVControl,"), true);
                                    let _ = c.send(&ascii_get_bytes("5"), true);
                                }
                                Ok(())
                            }
                            _ => Ok(()),
                        }
                    }
                    tungstenite::Message::Close(_) => {
                        let mut g = self.client.lock().unwrap_or_else(|e| e.into_inner());
                        if let Some(c) = g.as_mut() {
                            c.state = WebSocketState::CloseReceived;
                        }
                        log::info!(target: LOG, "unknown websocket data");
                        Ok(())
                    }
                    // control frames are answered inside ClientWebSocket and never surface
                    _ => Ok(()),
                }
            });
            if let Err(ex) = result {
                log::error!(target: LOG, "{ex}");
                if self.auto_reconnect.load(Ordering::SeqCst) {
                    std::thread::sleep(Duration::from_millis(100));
                    let url = self.url.lock().unwrap_or_else(|e| e.into_inner()).clone();
                    self.open_url(&url);
                }
            }
        }
    }

    fn bytes_to_read(&self) -> i32 {
        let rd = self.read.lock().unwrap_or_else(|e| e.into_inner());
        (rd.0.len() - rd.1 as i64) as i32
    }
}

/// `MissionPlanner.Comms.WebSocket : CommsBase, ICommsSerial, IDisposable`
pub struct WebSocket {
    base: CommsBase,
    shared: Arc<Shared>,
    #[allow(dead_code)]
    reconnectnoprompt: bool,
    pub retrys: i32,
    pub port: String,
    write_buffer_size: i32,
    write_timeout: i32,
    rts_enable: bool,
    read_timeout: i32,
    read_buffer_size: i32,
    baud_rate: i32,
    data_bits: i32,
    dtr_enable: bool,
}

impl Default for WebSocket {
    fn default() -> Self {
        Self::new()
    }
}

impl WebSocket {
    pub fn new() -> Self {
        WebSocket {
            base: CommsBase::new(),
            shared: Arc::new(Shared {
                client: Mutex::new(Some(ClientWebSocket::new())),
                read: Mutex::new((MemoryStream::default(), 0)),
                auto_reconnect: AtomicBool::new(true),
                socketio: AtomicBool::new(false),
                url: Mutex::new(String::new()),
            }),
            reconnectnoprompt: false,
            retrys: 3,
            port: String::new(),
            write_buffer_size: 0,
            write_timeout: 0,
            rts_enable: false,
            read_timeout: 500,
            read_buffer_size: 0,
            baud_rate: 0,
            data_bits: 0,
            dtr_enable: false,
        }
    }

    /// `public bool autoReconnect`
    pub fn auto_reconnect(&self) -> bool {
        self.shared.auto_reconnect.load(Ordering::SeqCst)
    }

    pub fn set_auto_reconnect(&self, v: bool) {
        self.shared.auto_reconnect.store(v, Ordering::SeqCst);
    }
}

impl ICommsSerial for WebSocket {
    fn base_stream(&mut self) -> Result<Box<dyn Stream + '_>, Exception> {
        Ok(Box::new(CommsStream::new(self, 0)))
    }

    fn baud_rate(&self) -> i32 {
        self.baud_rate
    }
    fn set_baud_rate(&mut self, value: i32) {
        self.baud_rate = value;
    }

    fn bytes_to_read(&mut self) -> Result<i32, Exception> {
        Ok(self.shared.bytes_to_read())
    }

    fn bytes_to_write(&mut self) -> i32 {
        0
    }

    fn data_bits(&self) -> i32 {
        self.data_bits
    }
    fn set_data_bits(&mut self, value: i32) {
        self.data_bits = value;
    }

    fn dtr_enable(&self) -> bool {
        self.dtr_enable
    }
    fn set_dtr_enable(&mut self, value: bool) {
        self.dtr_enable = value;
    }

    fn is_open(&mut self) -> bool {
        self.shared.is_open()
    }

    fn port_name(&self) -> String {
        "WS".into()
    }
    fn set_port_name(&mut self, _value: &str) {}

    fn read_buffer_size(&self) -> i32 {
        self.read_buffer_size
    }
    fn set_read_buffer_size(&mut self, value: i32) {
        self.read_buffer_size = value;
    }

    fn read_timeout(&self) -> i32 {
        self.read_timeout
    }
    fn set_read_timeout(&mut self, value: i32) {
        self.read_timeout = value;
    }

    fn rts_enable(&self) -> bool {
        self.rts_enable
    }
    fn set_rts_enable(&mut self, value: bool) {
        self.rts_enable = value;
    }

    fn write_buffer_size(&self) -> i32 {
        self.write_buffer_size
    }
    fn set_write_buffer_size(&mut self, value: i32) {
        self.write_buffer_size = value;
    }

    fn write_timeout(&self) -> i32 {
        self.write_timeout
    }
    fn set_write_timeout(&mut self, value: i32) {
        self.write_timeout = value;
    }

    fn close(&mut self) -> Result<(), Exception> {
        self.shared.auto_reconnect.store(false, Ordering::SeqCst);
        let mut g = self.shared.client.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(c) = g.as_mut() {
            c.dispose();
        }
        *g = Some(ClientWebSocket::new());
        Ok(())
    }

    fn discard_in_buffer(&mut self) -> Result<(), Exception> {
        let size = self.shared.bytes_to_read();
        let mut crap = vec![0u8; size.max(0) as usize];
        log::info!(target: LOG, "WSSerial DiscardInBuffer {}", size);
        self.read(&mut crap, 0, size)?;
        Ok(())
    }

    fn open(&mut self) -> Result<(), Exception> {
        if self.is_open() {
            log::warn!(target: LOG, "websocket socket already open");
            return Ok(());
        }

        log::info!(target: LOG, "websocket Open");

        let mut url = self.base.on_settings("WS_url", "", false);

        if self.base.on_input_box_show("remote host", "Enter url (eg http://user:pass@host:port/wspath)", &mut url)
            == inputboxreturn::Cancel
        {
            return Err(Exception::Other("Canceled by request".into()));
        }

        self.base.on_settings("WS_url", &url, true);

        self.shared.open_url(&url);

        let shared = self.shared.clone();
        std::thread::spawn(move || shared.run_reader());
        Ok(())
    }

    fn read(&mut self, readto: &mut [u8], offset: i32, length: i32) -> Result<i32, Exception> {
        if length < 1 {
            return Ok(0);
        }
        let mut rd = self.shared.read.lock().unwrap_or_else(|e| e.into_inner());
        let cursor = rd.1 as i64;
        if rd.0.seek(cursor, SeekOrigin::Begin).is_err() || readto.len() < (offset + length) as usize {
            return Err(Exception::Other("Socket Closed".into()));
        }
        let read = rd.0.read_into(readto, offset as usize, length as usize) as i32;
        rd.1 += read;
        if rd.0.len() - rd.1 as i64 == 0 {
            rd.0.set_length(0);
            rd.1 = 0;
        }
        Ok(read)
    }

    fn read_byte(&mut self) -> Result<i32, Exception> {
        let mut count = 0;
        while self.shared.bytes_to_read() == 0 {
            std::thread::sleep(Duration::from_millis(1));
            if count > self.read_timeout {
                return Err(Exception::Other("NetSerial Timeout on read".into()));
            }
            count += 1;
        }
        let mut buffer = [0u8; 1];
        self.read(&mut buffer, 0, 1)?;
        Ok(buffer[0] as i32)
    }

    fn read_char(&mut self) -> Result<i32, Exception> {
        self.read_byte()
    }

    fn read_existing(&mut self) -> Result<String, Exception> {
        let n = self.shared.bytes_to_read();
        let mut data = vec![0u8; n.max(0) as usize];
        if !data.is_empty() {
            self.read(&mut data, 0, n)?;
        }
        Ok(ascii_get_string(&data))
    }

    fn read_line(&mut self) -> Result<String, Exception> {
        let mut temp = vec![0u8; 4000];
        let mut count = 0usize;
        let mut timeout = 0;

        while timeout <= 100 {
            if !self.is_open() {
                break;
            }
            if self.shared.bytes_to_read() > 0 {
                let letter = self.read_byte()? as u8;
                temp[count] = letter;
                if letter == b'\n' {
                    break;
                }
                count += 1;
                if count == temp.len() {
                    break;
                }
                timeout = 0;
            } else {
                timeout += 1;
                std::thread::sleep(Duration::from_millis(5));
            }
        }

        temp.resize(count + 1, 0);
        Ok(ascii_get_string(&temp))
    }

    fn write_str(&mut self, line: &str) -> Result<(), Exception> {
        let data = ascii_get_bytes(line);
        let n = data.len() as i32;
        self.write(&data, 0, n)
    }

    /// Binary frames; socket.io sessions prefix the engine.io message type 4. Errors are logged.
    fn write(&mut self, write: &[u8], offset: i32, length: i32) -> Result<(), Exception> {
        let (o, l) = (offset.max(0) as usize, length.max(0) as usize);
        let payload: Vec<u8> = if self.shared.socketio.load(Ordering::SeqCst) {
            let mut temp = vec![0u8; l + 1];
            temp[0] = 4;
            temp[1..].copy_from_slice(&write[o..o + l]);
            temp
        } else {
            write[o..o + l].to_vec()
        };
        let mut g = self.shared.client.lock().unwrap_or_else(|e| e.into_inner());
        let r = match g.as_mut() {
            Some(c) => c.send(&payload, false),
            None => Err(Exception::NullReference("Object reference not set to an instance of an object.".into())),
        };
        if let Err(ex) = r {
            log::error!(target: LOG, "{ex}");
        }
        Ok(())
    }

    fn write_line(&mut self, line: &str) -> Result<(), Exception> {
        let line = format!("{line}\n");
        self.write_str(&line)
    }

    fn toggleDTR(&mut self) -> Result<(), Exception> {
        Ok(())
    }

    /// `Dispose(true)`: Close, then `client = null`.
    fn dispose(&mut self) {
        let _ = self.close();
        *self.shared.client.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_http_scheme_like_clientwebsocket() {
        let mut c = ClientWebSocket::new();
        assert!(matches!(c.connect("http://user:pass@127.0.0.1:1/x"), Err(Exception::Argument(_))));
    }

    #[test]
    fn read_cursor_resets_when_drained() {
        let mut w = WebSocket::new();
        {
            let mut rd = w.shared.read.lock().unwrap();
            rd.0.write_from(b"abc", 0, 3);
        }
        assert_eq!(w.bytes_to_read().unwrap(), 3);
        let mut b = [0u8; 2];
        assert_eq!(w.read(&mut b, 0, 2).unwrap(), 2);
        assert_eq!(w.bytes_to_read().unwrap(), 1);
        assert_eq!(w.read(&mut b, 0, 2).unwrap(), 1);
        assert_eq!(w.bytes_to_read().unwrap(), 0);
        assert_eq!(w.shared.read.lock().unwrap().0.len(), 0);
        assert!(!w.is_open());
        assert_eq!(w.port_name(), "WS");
    }

    #[test]
    fn local_echo_server_round_trip() {
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = l.local_addr().unwrap().port();
        let srv = std::thread::spawn(move || {
            let (s, _) = l.accept().unwrap();
            let mut ws = tungstenite::accept(s).unwrap();
            let probe = ws.read().unwrap();
            assert_eq!(probe.into_text().unwrap().as_str(), "2probe");
            ws.send(tungstenite::Message::Binary(b"hello".to_vec().into())).unwrap();
            let m = ws.read().unwrap();
            assert_eq!(m.into_data().as_ref(), b"xy");
        });
        let mut w = WebSocket::new();
        let shared = w.shared.clone();
        shared.open_url(&format!("ws://127.0.0.1:{port}/"));
        assert!(w.is_open());
        let s2 = shared.clone();
        std::thread::spawn(move || s2.run_reader());
        let t0 = std::time::Instant::now();
        while w.bytes_to_read().unwrap() < 5 && t0.elapsed().as_secs() < 3 {
            std::thread::sleep(Duration::from_millis(5));
        }
        let mut b = [0u8; 5];
        assert_eq!(w.read(&mut b, 0, 5).unwrap(), 5);
        assert_eq!(&b, b"hello");
        w.write(b"axyz", 1, 2).unwrap();
        srv.join().unwrap();
        w.close().unwrap();
    }
}
