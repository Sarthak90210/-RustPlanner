//! Port of `ExtLibs/Comms/CommsUdpSerial.cs`

use std::any::Any;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use super::commsbase::{inputboxreturn, CommsBase};
use super::udpstream::UDPStream;
use crate::dotnet::net::{IPEndPoint, UdpClient};
use crate::dotnet::{ascii_get_bytes, ascii_get_string, int_parse, DateTime, Exception, MemoryStream, SeekOrigin, Stream};
use crate::extlibs::interfaces::ICommsSerial;

const LOG: &str = "MissionPlanner.Comms.UdpSerial";

fn null_ref() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

/// `UdpSerial : CommsBase, ICommsSerial, IDisposable`
pub struct UdpSerial {
    base: CommsBase,
    /// `public readonly List<IPEndPoint> EndPointList`
    pub end_point_list: Vec<IPEndPoint>,
    _isopen: bool,
    /// `public bool CancelConnect`: shared so another thread can stop a waiting `Open()`.
    pub cancel_connect: Arc<AtomicBool>,
    /// `public UdpClient client`; `None` after `Dispose()`.
    pub client: Option<UdpClient>,
    rbuffer: MemoryStream,
    /// The remote end point we send messages to.
    pub remote_ip_end_point: IPEndPoint,
    pub config_ref: String,
    /// `Port`; null (empty) when built around an existing client.
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

impl Default for UdpSerial {
    fn default() -> Self {
        Self::new()
    }
}

impl UdpSerial {
    pub fn new() -> Self {
        UdpSerial {
            base: CommsBase::new(),
            end_point_list: vec![],
            _isopen: false,
            cancel_connect: Arc::new(AtomicBool::new(false)),
            client: Some(UdpClient::new()),
            rbuffer: MemoryStream::default(),
            remote_ip_end_point: IPEndPoint::any(),
            config_ref: String::new(),
            port: "14550".into(),
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

    /// `UdpSerial(UdpClient client)`: wraps a client that is already receiving.
    pub fn with_client(client: UdpClient) -> Self {
        let mut s = Self::new();
        s.client = Some(client);
        s._isopen = true;
        s.port = String::new();
        s
    }

    /// The `IsOpen` setter.
    pub fn set_is_open(&mut self, value: bool) {
        self._isopen = value;
    }

    fn client(&mut self) -> Result<&mut UdpClient, Exception> {
        self.client.as_mut().ok_or_else(null_ref)
    }

    fn verify_connected(&mut self) -> Result<(), Exception> {
        if self.client.is_none() || !self.is_open() {
            self.close()?;
            return Err(Exception::Other("The socket/serialproxy is closed".into()));
        }
        Ok(())
    }
}

impl ICommsSerial for UdpSerial {
    fn base_stream(&mut self) -> Result<Box<dyn Stream + '_>, Exception> {
        Ok(Box::new(UDPStream::new(self)))
    }

    fn baud_rate(&self) -> i32 {
        self.baud_rate
    }
    fn set_baud_rate(&mut self, value: i32) {
        self.baud_rate = value;
    }

    /// `client.Available + rbuffer.Length - rbuffer.Position`
    fn bytes_to_read(&mut self) -> Result<i32, Exception> {
        let a = self.client()?.available()? as i64;
        Ok((a + self.rbuffer.len() - self.rbuffer.pos()) as i32)
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
        if !self.client.as_ref().is_some_and(|c| c.has_client()) {
            return false;
        }
        self._isopen
    }

    fn port_name(&self) -> String {
        format!("UDP{}", self.port)
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
        self._isopen = false;
        if let Some(c) = self.client.as_mut() {
            c.close();
        }
        self.client = Some(UdpClient::new());
        Ok(())
    }

    fn discard_in_buffer(&mut self) -> Result<(), Exception> {
        self.verify_connected()?;
        let size = self.client()?.available()?;
        let mut crap = vec![0u8; size.max(0) as usize];
        log::info!(target: LOG, "UdpSerial DiscardInBuffer {}", size);
        self.read(&mut crap, 0, size)?;
        Ok(())
    }

    fn open(&mut self) -> Result<(), Exception> {
        let connected = self.client()?.connected();
        if connected || self.is_open() {
            log::info!(target: LOG, "UDPSerial socket already open");
            return Ok(());
        }

        self.client()?.close();

        let mut dest = self.port.clone();
        dest = self.base.on_settings(&format!("UDP_port{}", self.config_ref), &dest, false);

        if self.base.on_input_box_show("Listern Port", "Enter Local port (ensure remote end is already sending)", &mut dest)
            == inputboxreturn::Cancel
        {
            return Ok(());
        }
        self.port = dest;

        let p = self.port.clone();
        self.base.on_settings(&format!("UDP_port{}", self.config_ref), &p, true);

        //######################################

        if let Some(c) = self.client.as_mut() {
            c.close();
        }

        self.client = Some(UdpClient::bind(int_parse(&self.port)?)?);

        loop {
            std::thread::sleep(std::time::Duration::from_millis(500));

            if self.cancel_connect.load(Ordering::SeqCst) {
                if let Some(c) = self.client.as_mut() {
                    c.close();
                }
                return Ok(());
            }

            if self.bytes_to_read()? > 0 {
                break;
            }
        }

        if self.bytes_to_read()? == 0 {
            return Ok(());
        }

        // reset any previous connection
        self.remote_ip_end_point = IPEndPoint::any();
        let mut ep = self.remote_ip_end_point;
        match self.client()?.receive(&mut ep) {
            Ok(_) => {
                self.remote_ip_end_point = ep;
                log::info!(target: LOG, "UDPSerial connecting to {} : {}", ep.address, ep.port);
                self.end_point_list.push(ep);
                self._isopen = true;
                Ok(())
            }
            Err(ex) => {
                if let Some(c) = self.client.as_mut() {
                    if c.has_client() && c.connected() {
                        c.close();
                    }
                }
                log::info!(target: LOG, "{}", ex);
                Err(Exception::Other(format!("The socket/UDPSerial is closed {ex}")))
            }
        }
    }

    fn read(&mut self, readto: &mut [u8], offset: i32, length: i32) -> Result<i32, Exception> {
        self.verify_connected()?;
        if length < 1 {
            return Ok(0);
        }
        let mut length = length as i64;

        let deadline = DateTime::now().add_milliseconds(self.read_timeout as f64)?;

        if self.rbuffer.pos() == self.rbuffer.len() {
            self.rbuffer.set_length(0);
        }

        let position = self.rbuffer.pos();

        while (self.rbuffer.len() - self.rbuffer.pos()) < length && DateTime::now() < deadline {
            // read more
            while self.client()?.available()? > 0 && (self.rbuffer.len() - self.rbuffer.pos()) < length {
                let mut current_remote_ip_end_point = IPEndPoint::any();
                // assumes the udp packets are mavlink aligned, if we are receiving from more than one source
                let b = self.client()?.receive(&mut current_remote_ip_end_point)?;
                self.rbuffer.seek(0, SeekOrigin::End)?;
                self.rbuffer.write_from(&b, 0, b.len());
                self.rbuffer.seek(position, SeekOrigin::Begin)?;

                if !self.end_point_list.contains(&current_remote_ip_end_point) {
                    self.end_point_list.push(current_remote_ip_end_point);
                }
            }

            std::thread::yield_now();
        }

        // prevent read past end of array
        if self.rbuffer.len() - self.rbuffer.pos() < length {
            length = self.rbuffer.len() - self.rbuffer.pos();
        }

        Ok(self.rbuffer.read_into(readto, offset as usize, length as usize) as i32)
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

    /// Sized from `client.Available` only, not counting bytes already buffered.
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

    /// Sends to every end point heard from; `UdpClient.Send(write, length, ep)` sends from
    /// index 0 whatever the offset; send errors are swallowed.
    fn write(&mut self, write: &[u8], _offset: i32, length: i32) -> Result<(), Exception> {
        self.verify_connected()?;
        let eps = self.end_point_list.clone();
        if let Some(c) = self.client.as_mut() {
            for ip_end_point in &eps {
                let _ = c.send(write, length.max(0) as usize, ip_end_point);
            }
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
    use std::net::{Ipv4Addr, UdpSocket};

    #[test]
    fn wrapped_client_reads_and_replies_to_senders() {
        let rx = UdpClient::bind(0).unwrap();
        let port = rx.local_end_point().unwrap().port;
        let mut u = UdpSerial::with_client(rx);
        assert_eq!(u.port_name(), "UDP");
        assert!(u.is_open());
        let peer = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        peer.send_to(b"abc", (Ipv4Addr::LOCALHOST, port)).unwrap();
        peer.send_to(b"def", (Ipv4Addr::LOCALHOST, port)).unwrap();
        let mut buf = [0u8; 6];
        let mut got = 0;
        let t0 = std::time::Instant::now();
        while got < 6 && t0.elapsed().as_secs() < 3 {
            got += u.read(&mut buf, got as i32, 6 - got as i32).unwrap() as usize;
        }
        assert_eq!(&buf, b"abcdef");
        assert_eq!(u.end_point_list.len(), 1);
        u.write(b"xyz", 1, 2).unwrap();
        let mut r = [0u8; 8];
        let (n, _) = peer.recv_from(&mut r).unwrap();
        assert_eq!(&r[..n], b"xy");
        u.close().unwrap();
        assert!(!u.is_open());
        assert!(u.read(&mut buf, 0, 1).is_err());
    }
}
