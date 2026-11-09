//! Port of `ExtLibs/Comms/CommsNTRIP.cs`
//!
//! An NTRIP v1/v2 client: HTTP(S) request to the caster, then the RTCM byte stream, with a
//! GGA sentence sent every 30 s once a position is known. `SslStream` is rustls with the
//! webpki roots; `StreamReader.ReadLine` reads ahead in 1 KiB blocks and its buffered bytes
//! are dropped with the reader, as in C#.

use std::any::Any;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::Arc;

use base64::Engine;

use super::commsbase::{inputboxreturn, CommsBase};
use crate::dotnet::net::TcpClient;
use crate::dotnet::uri::{unescape_data_string, Uri};
use crate::dotnet::{
    ascii_get_bytes, ascii_get_string, format_fixed, format_int_zero, int_parse, math_round, DateTime, Exception, Stream,
    INVARIANT_CULTURE,
};
use crate::extlibs::interfaces::ICommsSerial;

const LOG: &str = "MissionPlanner.Comms.CommsNTRIP";

fn null_ref() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

/// `CommsNTRIP.GgaInfo`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GgaInfo {
    pub lat: f64,
    pub lng: f64,
    pub alt: f64,
    /// NMEA GGA fix quality (0 invalid, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float)
    pub quality: i32,
    pub sats: i32,
    pub hdop: f64,
}

impl Default for GgaInfo {
    fn default() -> Self {
        GgaInfo { lat: 0.0, lng: 0.0, alt: 0.0, quality: 1, sats: 10, hdop: 1.0 }
    }
}

/// `Func<GgaInfo>`: `Ok(None)` is a null result.
pub type GgaSourceFn = Arc<dyn Fn() -> Result<Option<GgaInfo>, Exception> + Send + Sync>;

/// `st`: the caster stream, plain `NetworkStream` or `SslStream`.
pub enum NtripStream {
    Plain(TcpStream),
    Tls(Box<rustls::StreamOwned<rustls::ClientConnection, TcpStream>>),
}

impl Read for NtripStream {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        match self {
            NtripStream::Plain(s) => s.read(buf),
            NtripStream::Tls(s) => s.read(buf),
        }
    }
}

impl Write for NtripStream {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        match self {
            NtripStream::Plain(s) => s.write(buf),
            NtripStream::Tls(s) => s.write(buf),
        }
    }
    fn flush(&mut self) -> std::io::Result<()> {
        match self {
            NtripStream::Plain(s) => s.flush(),
            NtripStream::Tls(s) => s.flush(),
        }
    }
}

impl Stream for NtripStream {
    fn can_timeout(&self) -> bool {
        true
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        Read::read(self, buffer).map_err(|e| Exception::Io(e.to_string()))
    }
    fn write(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        self.write_all(buffer).map_err(|e| Exception::Io(e.to_string()))
    }
}

/// `new StreamReader(st)`: UTF-8, 1024-byte reads.
struct StreamReader<'a> {
    st: &'a mut NtripStream,
    buf: Vec<u8>,
    pos: usize,
    eof: bool,
}

impl<'a> StreamReader<'a> {
    fn new(st: &'a mut NtripStream) -> Self {
        StreamReader { st, buf: vec![], pos: 0, eof: false }
    }

    fn fill(&mut self) -> Result<bool, Exception> {
        if self.eof {
            return Ok(false);
        }
        let mut b = [0u8; 1024];
        let n = Read::read(&mut *self.st, &mut b).map_err(|e| Exception::Io(e.to_string()))?;
        if n == 0 {
            self.eof = true;
            return Ok(false);
        }
        self.buf.drain(..self.pos);
        self.pos = 0;
        self.buf.extend_from_slice(&b[..n]);
        Ok(true)
    }

    /// `ReadLine()`: up to `\r`, `\n` or `\r\n`; null at the end of the stream.
    fn read_line(&mut self) -> Result<Option<String>, Exception> {
        let mut line: Vec<u8> = vec![];
        loop {
            if self.pos >= self.buf.len() && !self.fill()? {
                return Ok(if line.is_empty() { None } else { Some(String::from_utf8_lossy(&line).into_owned()) });
            }
            let c = self.buf[self.pos];
            self.pos += 1;
            if c == b'\n' {
                break;
            }
            if c == b'\r' {
                if self.pos >= self.buf.len() {
                    let _ = self.fill()?;
                }
                if self.pos < self.buf.len() && self.buf[self.pos] == b'\n' {
                    self.pos += 1;
                }
                break;
            }
            line.push(c);
        }
        let mut s = String::from_utf8_lossy(&line).into_owned();
        if s.starts_with('\u{feff}') {
            s.remove(0);
        }
        Ok(Some(s))
    }

    /// `ReadToEnd()`
    fn read_to_end(&mut self) -> Result<String, Exception> {
        while self.fill()? {}
        let s = String::from_utf8_lossy(&self.buf[self.pos..]).into_owned();
        self.pos = self.buf.len();
        Ok(s)
    }
}

/// `CommsNTRIP : CommsBase, ICommsSerial, IDisposable`
pub struct CommsNTRIP {
    base: CommsBase,
    _lastnmea: DateTime,
    pub alt: f64,
    /// `public TcpClient client`; `None` after `Dispose()`.
    pub client: Option<TcpClient>,
    st: Option<NtripStream>,
    host: String,
    pub lat: f64,
    pub lng: f64,
    pub ntrip_v1: bool,
    /// Optional live position source for the GGA sent to the caster. When set and it returns
    /// a value, it is used instead of the static lat/lng/alt.
    pub gga_source: Option<GgaSourceFn>,
    remote_uri: Option<Uri>,
    retrys: i32,
    pub port: String,
    write_buffer_size: i32,
    write_timeout: i32,
    rts_enable: bool,
    read_timeout: i32,
    read_buffer_size: i32,
    baud_rate: i32,
    data_bits: i32,
    port_name: String,
    dtr_enable: bool,
}

impl Default for CommsNTRIP {
    fn default() -> Self {
        Self::new()
    }
}

impl CommsNTRIP {
    pub fn new() -> Self {
        CommsNTRIP {
            base: CommsBase::new(),
            _lastnmea: DateTime::MIN_VALUE,
            alt: 0.0,
            client: Some(TcpClient::new()),
            st: None,
            host: String::new(),
            lat: 0.0,
            lng: 0.0,
            ntrip_v1: false,
            gga_source: None,
            remote_uri: None,
            retrys: 3,
            port: String::new(),
            write_buffer_size: 0,
            write_timeout: 0,
            rts_enable: false,
            read_timeout: 500,
            read_buffer_size: 0,
            baud_rate: 0,
            data_bits: 0,
            port_name: String::new(),
            dtr_enable: false,
        }
    }

    fn client(&mut self) -> Result<&mut TcpClient, Exception> {
        self.client.as_mut().ok_or_else(null_ref)
    }

    /// `PercentEncode(string value)`: everything but `0-9 A-Z a-z - . _ ~ @ / :` becomes `%XX`
    /// of the UTF-16 unit truncated to a byte.
    pub fn percent_encode(value: &str) -> String {
        let mut retval = String::new();
        for c in value.encode_utf16() {
            if (48..=57).contains(&c)
                || (65..=90).contains(&c)
                || (97..=122).contains(&c)
                || c == 45
                || c == 46
                || c == 95
                || c == 126
                || c == 64
                || c == 47
                || c == 58
            {
                retval.push(c as u8 as char);
            } else {
                retval.push_str(&format!("%{:02X}", c as u8));
            }
        }
        retval
    }

    /// `Open(string url)`
    pub fn open_url(&mut self, url: &str) -> Result<(), Exception> {
        // Need to ensure URI is % encoded, except the first "@", colons and backslashes
        let count = url.split('@').count() - 1;

        let mut url = Self::percent_encode(url);

        if count > 1 {
            url = url.replacen('@', "%40", 1);
        }

        url = url.replace("ntrip://", "http://");

        self.remote_uri = Some(Uri::parse(&url)?);

        self.do_connect()
    }

    fn do_connect(&mut self) -> Result<(), Exception> {
        let remote_uri = self.remote_uri.clone().ok_or_else(null_ref)?;
        let username_password = remote_uri.user_info.clone();
        let userpass2 = unescape_data_string(&username_password);
        let mut auth = format!(
            "Authorization: Basic {}\r\n",
            base64::engine::general_purpose::STANDARD.encode(ascii_get_bytes(&userpass2))
        );

        if username_password.is_empty() {
            auth = String::new();
        }

        self.host = remote_uri.host.clone();
        self.port = remote_uri.port.to_string();

        let port = int_parse(&self.port)?;
        self.client = Some(TcpClient::connect_new(&self.host, port)?);
        // fails under mono
        let _ = self.client()?.set_keep_alive_values(true, 36000000, 3000);

        let tcp = self.client()?.try_clone_stream()?;
        let st = if self.port == "443" || remote_uri.scheme == "https" {
            NtripStream::Tls(Box::new(authenticate_as_client(&self.host, tcp)?))
        } else {
            NtripStream::Plain(tcp)
        };
        self.st = Some(st);

        let linev1 = format!(
            "GET {} HTTP/1.0\r\nUser-Agent: NTRIP MissionPlanner/1.0\r\n{}Connection: close\r\n\r\n",
            remote_uri.path_and_query, auth
        );

        let linev2 = format!(
            "GET {} HTTP/1.1\r\nHost: {}:{}\r\nNtrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIP MissionPlanner/1.0\r\n{}Connection: close\r\n\r\n",
            remote_uri.path_and_query, remote_uri.host, remote_uri.port, auth
        );

        let st = self.st.as_mut().unwrap();
        // new StreamWriter(st): UTF-8 without a BOM
        if self.ntrip_v1 {
            st.write_all(linev1.as_bytes()).map_err(|e| Exception::Io(e.to_string()))?;
            log::info!(target: LOG, "{}", linev1);
        } else {
            st.write_all(linev2.as_bytes()).map_err(|e| Exception::Io(e.to_string()))?;
            log::info!(target: LOG, "{}", linev2);
        }
        std::io::Write::flush(st).map_err(|e| Exception::Io(e.to_string()))?;

        let mut sr = StreamReader::new(st);
        let line = sr.read_line()?;

        log::info!(target: LOG, "{}", line.as_deref().unwrap_or(""));

        let Some(line) = line else { return Err(null_ref()) };

        if !line.contains("200") {
            drop(sr);
            if let Some(c) = self.client.as_mut() {
                c.dispose();
            }
            self.client = Some(TcpClient::new());
            return Err(Exception::Other(format!("Bad ntrip Response\n\n{line}")));
        }

        if line.contains("SOURCETABLE") {
            let rest = sr.read_to_end()?;
            log::info!(target: LOG, "{}", rest);
            if let Some(c) = self.client.as_mut() {
                c.dispose();
            }
            self.client = Some(TcpClient::new());
            return Err(Exception::Other(format!("Got SOURCETABLE - Bad ntrip mount point\n\n{line}")));
        }
        drop(sr);

        // vrs may take up to 60+ seconds to respond
        self.send_nmea()?;

        self.verify_connected()
    }

    /// `ToNmeaDegMin(degrees)`: decimal degrees to NMEA ddmm.mm (unsigned; the hemisphere is
    /// sent separately).
    pub fn to_nmea_deg_min(degrees: f64) -> f64 {
        let abs = degrees.abs();
        let mut whole = abs.floor();
        let mut minutes = math_round((abs - whole) * 60.0, 2);
        if minutes >= 60.0 {
            whole += 1.0;
            minutes = 0.0;
        }
        whole * 100.0 + minutes
    }

    fn send_nmea(&mut self) -> Result<(), Exception> {
        if self._lastnmea.add_milliseconds(30_000.0)? >= DateTime::now() {
            return Ok(());
        }

        let mut info: Option<GgaInfo> = None;
        if let Some(src) = self.gga_source.clone() {
            match src() {
                Ok(i) => info = i,
                Err(ex) => log::warn!(target: LOG, "ntrip GgaSource failed {ex}"),
            }
        }

        let info = info.unwrap_or(GgaInfo { lat: self.lat, lng: self.lng, alt: self.alt, ..Default::default() });

        if info.lat == 0.0 && info.lng == 0.0 {
            return Ok(());
        }

        let inv = &INVARIANT_CULTURE;
        let line = format!(
            "$GP{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            "GGA",
            DateTime::utc_now().format("HHmmss.ff", inv),
            format_fixed(Self::to_nmea_deg_min(info.lat), 4, 2, inv),
            if info.lat < 0.0 { "S" } else { "N" },
            format_fixed(Self::to_nmea_deg_min(info.lng), 5, 2, inv),
            if info.lng < 0.0 { "W" } else { "E" },
            info.quality,
            format_int_zero(info.sats as i64, 2),
            format_fixed(info.hdop, 1, 1, inv),
            format_fixed(info.alt, 1, 2, inv),
            "M",
            0,
            "M",
            "0.0",
            "0"
        );

        let checksum = Self::get_checksum(&line)?;
        self.write_line(&format!("{line}*{checksum}"))?;

        log::info!(target: LOG, "{}*{}", line, checksum);

        self._lastnmea = DateTime::now();
        Ok(())
    }

    /// `GetChecksum(sentence)`: XOR of the characters, skipping `$` and `*`, as two hex digits.
    pub fn get_checksum(sentence: &str) -> Result<String, Exception> {
        let mut checksum: i32 = 0;
        for character in sentence.encode_utf16() {
            match character {
                0x24 => {}
                0x2A => continue,
                _ => {
                    let b = u8::try_from(character)
                        .map_err(|_| Exception::Overflow("Value was either too large or too small for an unsigned byte.".into()))?;
                    if checksum == 0 {
                        checksum = b as i32;
                    } else {
                        checksum ^= b as i32;
                    }
                }
            }
        }
        Ok(format!("{:02X}", checksum))
    }

    fn verify_connected(&mut self) -> Result<(), Exception> {
        if !self.is_open() {
            if let Some(c) = self.client.as_mut() {
                c.dispose();
                self.client = Some(TcpClient::new());
            }

            // this should only happen if we have established a connection in the first place
            if self.client.is_some() && self.retrys > 0 {
                log::info!(target: LOG, "ntrip reconnect");
                self.do_connect()?;
                self.retrys -= 1;
            }

            return Err(Exception::Other("The ntrip is closed".into()));
        }
        Ok(())
    }
}

/// `new SslStream(client.GetStream(), false).AuthenticateAsClient(host)`
fn authenticate_as_client(host: &str, tcp: TcpStream) -> Result<rustls::StreamOwned<rustls::ClientConnection, TcpStream>, Exception> {
    let auth_err = |e: String| Exception::Authentication(format!("The remote certificate is invalid according to the validation procedure. {e}"));
    if rustls::crypto::CryptoProvider::get_default().is_none() {
        let _ = rustls::crypto::ring::default_provider().install_default();
    }
    let mut roots = rustls::RootCertStore::empty();
    roots.extend(webpki_roots::TLS_SERVER_ROOTS.iter().cloned());
    let config = rustls::ClientConfig::builder().with_root_certificates(roots).with_no_client_auth();
    let name = rustls::pki_types::ServerName::try_from(host.to_string()).map_err(|e| auth_err(e.to_string()))?;
    let conn = rustls::ClientConnection::new(Arc::new(config), name).map_err(|e| auth_err(e.to_string()))?;
    let mut tls = rustls::StreamOwned::new(conn, tcp);
    while tls.conn.is_handshaking() {
        tls.conn.complete_io(&mut tls.sock).map_err(|e| auth_err(e.to_string()))?;
    }
    Ok(tls)
}

impl ICommsSerial for CommsNTRIP {
    fn base_stream(&mut self) -> Result<Box<dyn Stream + '_>, Exception> {
        match self.st.as_mut() {
            Some(s) => Ok(Box::new(s)),
            None => Err(null_ref()),
        }
    }

    fn baud_rate(&self) -> i32 {
        self.baud_rate
    }
    fn set_baud_rate(&mut self, value: i32) {
        self.baud_rate = value;
    }

    /// Sends the GGA if it is due, then reports the socket's unread bytes.
    fn bytes_to_read(&mut self) -> Result<i32, Exception> {
        self.send_nmea()?;
        self.client()?.available()
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
        match self.client.as_mut() {
            Some(c) if c.has_client() => c.connected(),
            _ => false,
        }
    }

    fn port_name(&self) -> String {
        self.port_name.clone()
    }
    fn set_port_name(&mut self, value: &str) {
        self.port_name = value.to_string();
    }

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
        if let Some(c) = self.client.as_mut() {
            c.dispose();
        }
        self.client = Some(TcpClient::new());
        Ok(())
    }

    fn discard_in_buffer(&mut self) -> Result<(), Exception> {
        self.verify_connected()?;
        let size = self.client()?.available()?;
        let mut crap = vec![0u8; size.max(0) as usize];
        log::info!(target: LOG, "ntrip DiscardInBuffer {}", size);
        self.read(&mut crap, 0, size)?;
        Ok(())
    }

    fn open(&mut self) -> Result<(), Exception> {
        if self.client()?.connected() {
            log::warn!(target: LOG, "ntrip socket already open");
            return Ok(());
        }

        log::info!(target: LOG, "ntrip Open");

        let mut url = self.base.on_settings("NTRIP_url", "", false);

        if self.base.on_input_box_show("remote host", "Enter url (eg http://user:pass@host:port/mount)", &mut url)
            == inputboxreturn::Cancel
        {
            return Err(Exception::Other("Canceled by request".into()));
        }

        self.base.on_settings("NTRIP_url", &url, true);

        self.open_url(&url)
    }

    fn read(&mut self, readto: &mut [u8], offset: i32, length: i32) -> Result<i32, Exception> {
        self.verify_connected()?;

        self.send_nmea()?;

        if length < 1 {
            return Ok(0);
        }
        let (o, l) = (offset as usize, length as usize);
        match self.st.as_mut() {
            Some(st) if readto.len() >= o + l => match Read::read(st, &mut readto[o..o + l]) {
                Ok(n) => Ok(n as i32),
                Err(_) => Err(Exception::Other("ntrip Socket Closed".into())),
            },
            _ => Err(Exception::Other("ntrip Socket Closed".into())),
        }
    }

    fn read_byte(&mut self) -> Result<i32, Exception> {
        self.verify_connected()?;
        let mut count = 0;
        while self.bytes_to_read()? == 0 {
            std::thread::sleep(std::time::Duration::from_millis(1));
            if count > self.read_timeout {
                return Err(Exception::Other("ntrip Timeout on read".into()));
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
        self.verify_connected()?;
        let n = self.client()?.available()?;
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
            if self.bytes_to_read()? > 0 {
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
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
        }

        temp.resize(count + 1, 0);
        Ok(ascii_get_string(&temp))
    }

    fn write_str(&mut self, line: &str) -> Result<(), Exception> {
        self.verify_connected()?;
        let data = ascii_get_bytes(line);
        let n = data.len() as i32;
        self.write(&data, 0, n)
    }

    /// `st.Write(write, offset, length)`; errors are swallowed.
    fn write(&mut self, write: &[u8], offset: i32, length: i32) -> Result<(), Exception> {
        self.verify_connected()?;
        let (o, l) = (offset.max(0) as usize, length.max(0) as usize);
        if let Some(st) = self.st.as_mut() {
            if write.len() >= o + l {
                let _ = st.write_all(&write[o..o + l]);
            }
        }
        Ok(())
    }

    /// `WriteLine`: CR LF.
    fn write_line(&mut self, line: &str) -> Result<(), Exception> {
        self.verify_connected()?;
        let line = format!("{line}\r\n");
        self.write_str(&line)
    }

    fn toggleDTR(&mut self) -> Result<(), Exception> {
        Ok(())
    }

    fn dispose(&mut self) {
        let _ = self.close();
        self.client = None;
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
    fn percent_encoding_and_checksum() {
        assert_eq!(CommsNTRIP::percent_encode("http://u:p w@h:2101/M?x"), "http://u:p%20w@h:2101/M%3Fx");
        assert_eq!(CommsNTRIP::get_checksum("$GPGGA,1").unwrap(), {
            let mut c = 0u8;
            for b in "GPGGA,1".bytes() {
                c ^= b;
            }
            format!("{:02X}", c)
        });
    }

    #[test]
    fn nmea_deg_min() {
        assert!((CommsNTRIP::to_nmea_deg_min(-35.5) - 3530.0).abs() < 1e-9);
        assert!((CommsNTRIP::to_nmea_deg_min(10.99999999) - 1100.0).abs() < 1e-9);
        assert_eq!(format_fixed(CommsNTRIP::to_nmea_deg_min(149.165), 5, 2, &INVARIANT_CULTURE), "14909.90");
    }

    #[test]
    fn caster_handshake_and_rtcm() {
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = l.local_addr().unwrap().port();
        let srv = std::thread::spawn(move || {
            let (mut s, _) = l.accept().unwrap();
            let mut req = vec![0u8; 1024];
            let n = s.read(&mut req).unwrap();
            let req = String::from_utf8_lossy(&req[..n]).to_string();
            s.write_all(b"ICY 200 OK\r\n").unwrap();
            std::thread::sleep(std::time::Duration::from_millis(100));
            s.write_all(&[0xD3, 0, 1]).unwrap();
            std::thread::sleep(std::time::Duration::from_millis(300));
            req
        });
        let mut n = CommsNTRIP::new();
        n.open_url(&format!("http://user:pa ss@127.0.0.1:{port}/MOUNT")).unwrap();
        let mut b = [0u8; 3];
        let t0 = std::time::Instant::now();
        while n.bytes_to_read().unwrap() < 3 && t0.elapsed().as_secs() < 3 {
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert_eq!(n.read(&mut b, 0, 3).unwrap(), 3);
        assert_eq!(b, [0xD3, 0, 1]);
        let req = srv.join().unwrap();
        assert!(req.starts_with("GET /MOUNT HTTP/1.1\r\n"), "{req}");
        assert!(req.contains(&format!("Host: 127.0.0.1:{port}\r\n")));
        assert!(req.contains("Ntrip-Version: Ntrip/2.0\r\n"));
        let b64 = base64::engine::general_purpose::STANDARD.encode("user:pa ss");
        assert!(req.contains(&format!("Authorization: Basic {b64}\r\n")));
    }
}
