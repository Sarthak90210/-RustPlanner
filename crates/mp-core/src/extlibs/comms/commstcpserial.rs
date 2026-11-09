//! Port of `ExtLibs/Comms/CommsTCPSerial.cs`

use std::any::Any;

use super::commsbase::{inputboxreturn, CommsBase};
use crate::dotnet::net::{IPEndPoint, TcpClient};
use crate::dotnet::{ascii_get_bytes, ascii_get_string, int_parse, DateTime, Exception, Stream};
use crate::extlibs::interfaces::ICommsSerial;

const LOG: &str = "MissionPlanner.Comms.TcpSerial";

fn null_ref() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

/// `TcpSerial : CommsBase, ICommsSerial, IDisposable`
pub struct TcpSerial {
    base: CommsBase,
    pub auto_reconnect: bool,
    /// `public TcpClient client`; `None` after `Dispose()`.
    pub client: Option<TcpClient>,
    in_open: bool,
    closed: bool,
    last_reconnect_time: DateTime,
    reconnectnoprompt: bool,
    #[allow(dead_code)]
    remote_ip_end_point: IPEndPoint,
    pub retrys: i32,
    pub config_ref: String,
    pub port: String,
    write_buffer_size: i32,
    write_timeout: i32,
    rts_enable: bool,
    read_timeout: i32,
    read_buffer_size: i32,
    baud_rate: i32,
    data_bits: i32,
    dtr_enable: bool,
    pub host: String,
}

impl Default for TcpSerial {
    fn default() -> Self {
        Self::new()
    }
}

impl TcpSerial {
    pub fn new() -> Self {
        TcpSerial {
            base: CommsBase::new(),
            auto_reconnect: false,
            client: Some(TcpClient::new()),
            in_open: false,
            closed: false,
            last_reconnect_time: DateTime::MIN_VALUE,
            reconnectnoprompt: false,
            remote_ip_end_point: IPEndPoint::any(),
            retrys: 3,
            config_ref: String::new(),
            port: "5760".into(),
            write_buffer_size: 0,
            write_timeout: 0,
            rts_enable: false,
            read_timeout: 500,
            read_buffer_size: 0,
            baud_rate: 0,
            data_bits: 0,
            dtr_enable: false,
            host: String::new(),
        }
    }

    fn client(&mut self) -> Result<&mut TcpClient, Exception> {
        self.client.as_mut().ok_or_else(null_ref)
    }

    fn open_inner(&mut self) -> Result<(), Exception> {
        self.in_open = true;
        self.closed = false;

        if self.client()?.connected() {
            log::warn!(target: LOG, "tcpserial socket already open");
            return Ok(());
        }

        let mut dest = self.port.clone();
        let mut host = "127.0.0.1".to_string();

        if self.host.is_empty() {
            dest = self.base.on_settings(&format!("TCP_port{}", self.config_ref), &dest, false);
            host = self.base.on_settings(&format!("TCP_host{}", self.config_ref), &host, false);

            if !self.reconnectnoprompt {
                if self.base.on_input_box_show("remote host", "Enter host name/ip (ensure remote end is already started)", &mut host)
                    == inputboxreturn::Cancel
                {
                    return Err(Exception::Other("Canceled by request".into()));
                }
                if self.base.on_input_box_show("remote Port", "Enter remote port", &mut dest) == inputboxreturn::Cancel {
                    return Err(Exception::Other("Canceled by request".into()));
                }
            }
            self.host = host;
        }

        self.port = dest;

        log::info!(target: LOG, "TCP Open {} {}", self.host, self.port);

        let (p, h) = (self.port.clone(), self.host.clone());
        self.base.on_settings(&format!("TCP_port{}", self.config_ref), &p, true);
        self.base.on_settings(&format!("TCP_host{}", self.config_ref), &h, true);

        self.init_tcp_client(&h, &p)
    }

    fn init_tcp_client(&mut self, host: &str, port: &str) -> Result<(), Exception> {
        let port = int_parse(port)?;
        let mut client = TcpClient::connect_new(host, port)?;
        client.set_no_delay(true);
        self.client = Some(client);

        self.verify_connected()?;

        self.reconnectnoprompt = true;
        Ok(())
    }

    fn do_auto_reconnect(&mut self) {
        if !self.auto_reconnect {
            return;
        }
        if DateTime::now() > self.last_reconnect_time {
            if let Some(c) = self.client.as_mut() {
                c.dispose();
            }
            self.client = Some(TcpClient::new());

            let host = self.base.on_settings(&format!("TCP_host{}", self.config_ref), "", false);
            let Ok(port) = int_parse(&self.base.on_settings(&format!("TCP_port{}", self.config_ref), "", false)) else {
                return;
            };

            log::info!(target: LOG, "doAutoReconnect {} {}", host, port);

            self.client.as_mut().unwrap().connect_async(&host, port);

            if let Ok(t) = DateTime::now().add_milliseconds(5000.0) {
                self.last_reconnect_time = t;
            }
        }
    }

    fn verify_connected(&mut self) -> Result<(), Exception> {
        if !self.is_open() {
            if let Some(c) = self.client.as_mut() {
                c.dispose();
            }

            // this should only happen if we have established a connection in the first place
            if self.client.is_some() && self.retrys > 0 {
                log::info!(target: LOG, "tcp reconnect");
                self.client = Some(TcpClient::new());
                let host = self.base.on_settings(&format!("TCP_host{}", self.config_ref), "", false);
                let port = int_parse(&self.base.on_settings(&format!("TCP_port{}", self.config_ref), "", false))?;
                self.client.as_mut().unwrap().connect(&host, port)?;
                self.retrys -= 1;
            }

            return Err(Exception::Other("The socket/serialproxy is closed".into()));
        }
        Ok(())
    }
}

impl ICommsSerial for TcpSerial {
    fn base_stream(&mut self) -> Result<Box<dyn Stream + '_>, Exception> {
        Ok(Box::new(self.client()?.get_stream()?))
    }

    fn baud_rate(&self) -> i32 {
        self.baud_rate
    }
    fn set_baud_rate(&mut self, value: i32) {
        self.baud_rate = value;
    }

    fn bytes_to_read(&mut self) -> Result<i32, Exception> {
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
        let Some(client) = self.client.as_mut() else { return false };
        if !client.has_client() {
            return false;
        }
        if self.auto_reconnect && !client.connected() && !self.in_open && !self.closed {
            self.do_auto_reconnect();
        }
        self.client.as_mut().is_some_and(|c| c.connected())
    }

    fn port_name(&self) -> String {
        if let Some(ep) = self.client.as_ref().filter(|c| c.has_client()).and_then(|c| c.remote_end_point()) {
            return format!("TCP{}", ep.port);
        }
        format!("TCP{}", self.port)
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
        log::info!(target: LOG, "TCP Close {} {}", self.host, self.port);
        if let Some(c) = self.client.as_mut() {
            c.dispose();
        }
        self.closed = true;
        self.client = Some(TcpClient::new());
        self.host = String::new();
        Ok(())
    }

    fn discard_in_buffer(&mut self) -> Result<(), Exception> {
        self.verify_connected()?;
        let size = self.client()?.available()?;
        let mut crap = vec![0u8; size.max(0) as usize];
        log::info!(target: LOG, "TcpSerial DiscardInBuffer {}", size);
        self.read(&mut crap, 0, size)?;
        Ok(())
    }

    fn open(&mut self) -> Result<(), Exception> {
        let r = self.open_inner();
        if r.is_err() {
            // disable if the first connect fails
            self.auto_reconnect = false;
        }
        self.in_open = false;
        r
    }

    fn read(&mut self, readto: &mut [u8], offset: i32, length: i32) -> Result<i32, Exception> {
        self.verify_connected()?;
        if length < 1 {
            return Ok(0);
        }
        let (o, l) = (offset as usize, length as usize);
        match self.client.as_mut().map(|c| c.receive(&mut readto[o..o + l])) {
            Some(Ok(n)) => Ok(n as i32),
            _ => Err(Exception::Other("Socket Closed".into())),
        }
    }

    fn read_byte(&mut self) -> Result<i32, Exception> {
        self.verify_connected()?;
        let mut count = 0;
        while self.bytes_to_read()? == 0 {
            std::thread::sleep(std::time::Duration::from_millis(1));
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
                    // normal line
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

        // Array.Resize(ref temp, count + 1): a full buffer grows by one zero byte.
        temp.resize(count + 1, 0);
        Ok(ascii_get_string(&temp))
    }

    fn write_str(&mut self, line: &str) -> Result<(), Exception> {
        self.verify_connected()?;
        let data = ascii_get_bytes(line);
        let n = data.len() as i32;
        self.write(&data, 0, n)
    }

    /// `Write(byte[] write, int offset, int length)`: `Socket.Send(write, length, None)` sends
    /// from index 0 whatever the offset; send errors are swallowed.
    fn write(&mut self, write: &[u8], _offset: i32, length: i32) -> Result<(), Exception> {
        self.verify_connected()?;
        if let Some(c) = self.client.as_mut() {
            let _ = c.send(&write[..length.max(0) as usize]);
        }
        Ok(())
    }

    fn write_line(&mut self, line: &str) -> Result<(), Exception> {
        self.verify_connected()?;
        let line = format!("{line}\n");
        self.write_str(&line)
    }

    fn toggleDTR(&mut self) -> Result<(), Exception> {
        Ok(())
    }

    /// `Dispose()` -> `Dispose(true)`: Close, then `client = null`.
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
    use std::io::{Read, Write};

    #[test]
    fn connects_reads_lines_and_closes() {
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = l.local_addr().unwrap().port();
        let mut t = TcpSerial::new();
        t.host = "127.0.0.1".into();
        t.port = port.to_string();
        t.open().unwrap();
        let (mut srv, _) = l.accept().unwrap();
        assert!(t.is_open());
        assert_eq!(t.port_name(), format!("TCP{port}"));
        srv.write_all(b"hello\nrest").unwrap();
        assert_eq!(t.read_line().unwrap(), "hello\n");
        t.write(b"abcdef", 2, 3).unwrap();
        let mut got = [0u8; 3];
        srv.read_exact(&mut got).unwrap();
        assert_eq!(&got, b"abc", "Socket.Send ignores the offset");
        t.close().unwrap();
        assert!(!t.is_open());
        assert_eq!(t.host, "");
        assert!(matches!(t.read(&mut [0u8; 1], 0, 1), Err(Exception::Other(m)) if m == "The socket/serialproxy is closed"));
        t.dispose();
        assert!(!t.is_open());
        assert!(matches!(t.bytes_to_read(), Err(Exception::NullReference(_))));
    }
}
