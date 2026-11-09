//! Port of `ExtLibs/Comms/CommsUDPSerialConnect.cs`

use std::any::Any;
use std::net::IpAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use super::commsbase::{inputboxreturn, CommsBase};
use crate::dotnet::net::{
    address_family, dns_get_host_entry, get_address_bytes, ip_address_parse, ip_address_try_parse, IPEndPoint, UdpClient,
};
use crate::dotnet::{ascii_get_string, int_parse, DateTime, Exception, MemoryStream, NullStream, SeekOrigin, Stream};
use crate::extlibs::interfaces::ICommsSerial;

const LOG: &str = "MissionPlanner.Comms.UdpSerialConnect";

fn null_ref() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

/// `UdpSerialConnect : CommsBase, ICommsSerial, IDisposable`: UDP client mode ("UDPCl"),
/// sending to one host end point.
pub struct UdpSerialConnect {
    base: CommsBase,
    /// `public UdpClient client`: set `host_end_point` as well, if injecting.
    pub client: Option<UdpClient>,
    rbuffer: MemoryStream,
    pub remote_ip_end_point: IPEndPoint,
    #[allow(dead_code)]
    retrys: i32,
    pub host_end_point: Option<IPEndPoint>,
    pub config_ref: String,
    pub port: String,
    write_buffer_size: i32,
    write_timeout: i32,
    rts_enable: bool,
    read_timeout: i32,
    read_buffer_size: i32,
    baud_rate: i32,
    data_bits: i32,
    /// `IsOpen { get; set; }`, shared with the multicast re-join task.
    is_open: Arc<AtomicBool>,
    dtr_enable: bool,
}

impl Default for UdpSerialConnect {
    fn default() -> Self {
        Self::new()
    }
}

impl UdpSerialConnect {
    pub fn new() -> Self {
        UdpSerialConnect {
            base: CommsBase::new(),
            client: Some(UdpClient::new()),
            rbuffer: MemoryStream::default(),
            remote_ip_end_point: IPEndPoint::any(),
            retrys: 3,
            host_end_point: None,
            config_ref: String::new(),
            port: "14550".into(),
            write_buffer_size: 0,
            write_timeout: 0,
            rts_enable: false,
            read_timeout: 500,
            read_buffer_size: 0,
            baud_rate: 0,
            data_bits: 0,
            is_open: Arc::new(AtomicBool::new(false)),
            dtr_enable: false,
        }
    }

    /// The `IsOpen` setter.
    pub fn set_is_open(&self, value: bool) {
        self.is_open.store(value, Ordering::SeqCst);
    }

    fn client(&mut self) -> Result<&mut UdpClient, Exception> {
        self.client.as_mut().ok_or_else(null_ref)
    }

    /// `Open(string host, string port)`
    pub fn open_host(&mut self, host: &str, port: &str) -> Result<(), Exception> {
        self.port = port.to_string();

        let p = self.port.clone();
        self.base.on_settings(&format!("UDP_port{}", self.config_ref), &p, true);
        self.base.on_settings(&format!("UDP_host{}", self.config_ref), host, true);

        let addr = match ip_address_try_parse(host) {
            Some(addr) => addr,
            None => *dns_get_host_entry(host)?.first().unwrap(),
        };
        let port_num = int_parse(&self.port)?;
        // new IPEndPoint(addr, port) validates the port range.
        let port_num = u16::try_from(port_num).map_err(|_| {
            Exception::ArgumentOutOfRange("Specified argument was out of the range of valid values.\r\nParameter name: port".into())
        })?;
        let host_end_point = IPEndPoint::new(addr, port_num);
        self.host_end_point = Some(host_end_point);

        if Self::is_in_range("224.0.0.0", "239.255.255.255", &host_end_point.address.to_string())? {
            log::info!(target: LOG, "UdpSerialConnect bind to port {}", self.port);
            self.client = Some(UdpClient::bind_family(int_parse(&self.port)?, address_family(&host_end_point.address))?);

            self.set_is_open(true);

            // Task.Run: re-join the group every 30 s while open.
            let is_open = self.is_open.clone();
            let sock = self.client.as_ref().and_then(|c| c.try_clone_socket());
            let host = host.to_string();
            std::thread::spawn(move || {
                let Some(sock) = sock else { return };
                while is_open.load(Ordering::SeqCst) {
                    log::info!(target: LOG, "UdpSerialConnect join multicast group {}", host);
                    let group = match ip_address_parse(&host) {
                        Ok(g) => g,
                        Err(_) => return,
                    };
                    let r = match group {
                        IpAddr::V4(a) => sock.join_multicast_v4(&a, &std::net::Ipv4Addr::UNSPECIFIED),
                        IpAddr::V6(a) => sock.join_multicast_v6(&a, 0),
                    };
                    if r.is_err() {
                        return;
                    }
                    std::thread::sleep(std::time::Duration::from_secs(30));
                }
            });
            log::info!(target: LOG, "UdpSerialConnect default endpoint {}", host_end_point);
        } else {
            self.client = Some(UdpClient::with_family(address_family(&host_end_point.address)));
        }

        self.set_is_open(true);

        self.verify_connected();
        Ok(())
    }

    /// `IsInRange(startIpAddr, endIpAddr, address)`: compares the addresses as the signed
    /// 32-bit integers `BitConverter.ToInt32` reads from their reversed bytes.
    pub fn is_in_range(start_ip_addr: &str, end_ip_addr: &str, address: &str) -> Result<bool, Exception> {
        fn as_int(s: &str) -> Result<i64, Exception> {
            let mut b = get_address_bytes(&ip_address_parse(s)?);
            b.reverse();
            Ok(i32::from_le_bytes([b[0], b[1], b[2], b[3]]) as i64)
        }
        let ip_start = as_int(start_ip_addr)?;
        let ip_end = as_int(end_ip_addr)?;
        let ip = as_int(address)?;
        Ok(ip >= ip_start && ip <= ip_end)
    }

    fn verify_connected(&mut self) {}
}

impl ICommsSerial for UdpSerialConnect {
    fn base_stream(&mut self) -> Result<Box<dyn Stream + '_>, Exception> {
        Ok(Box::new(NullStream))
    }

    fn baud_rate(&self) -> i32 {
        self.baud_rate
    }
    fn set_baud_rate(&mut self, value: i32) {
        self.baud_rate = value;
    }

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
        self.is_open.load(Ordering::SeqCst)
    }

    fn port_name(&self) -> String {
        format!("UDPCl{}", self.port)
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
        self.set_is_open(false);

        if let (Some(ep), Some(c)) = (self.host_end_point, self.client.as_mut()) {
            let _ = c.drop_multicast_group(ep.address);
        }

        if let Some(c) = self.client.as_mut() {
            c.close();
        }

        self.client = Some(UdpClient::new());
        Ok(())
    }

    fn discard_in_buffer(&mut self) -> Result<(), Exception> {
        self.verify_connected();
        let size = self.client()?.available()?;
        let mut crap = vec![0u8; size.max(0) as usize];
        log::info!(target: LOG, "UdpSerialConnect DiscardInBuffer {}", size);
        self.read(&mut crap, 0, size)?;
        Ok(())
    }

    fn open(&mut self) -> Result<(), Exception> {
        if self.is_open() {
            log::warn!(target: LOG, "UdpSerialConnect socket already open");
            return Ok(());
        }

        log::info!(target: LOG, "UDP Open");

        let mut dest = self.port.clone();
        let mut host = "127.0.0.1".to_string();

        dest = self.base.on_settings(&format!("UDP_port{}", self.config_ref), &dest, false);
        host = self.base.on_settings(&format!("UDP_host{}", self.config_ref), &host, false);

        if self.base.on_input_box_show("remote host", "Enter host name/ip (ensure remote end is already started)", &mut host)
            == inputboxreturn::Cancel
        {
            return Err(Exception::Other("Canceled by request".into()));
        }
        if self.base.on_input_box_show("remote Port", "Enter remote port", &mut dest) == inputboxreturn::Cancel {
            return Err(Exception::Other("Canceled by request".into()));
        }

        self.open_host(&host, &dest)
    }

    fn read(&mut self, readto: &mut [u8], offset: i32, length: i32) -> Result<i32, Exception> {
        self.verify_connected();
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

                self.remote_ip_end_point = current_remote_ip_end_point;
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
        self.verify_connected();
        let mut count = 0;
        while self.bytes_to_read()? == 0 {
            std::thread::sleep(std::time::Duration::from_millis(1));
            if count > self.read_timeout {
                return Err(Exception::Other("UdpSerialConnect Timeout on read".into()));
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
        self.verify_connected();
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

    /// `Write(string)`: `ASCIIEncoding.UTF8` is `Encoding.UTF8`, so text goes out as UTF-8.
    fn write_str(&mut self, line: &str) -> Result<(), Exception> {
        self.verify_connected();
        let data = line.as_bytes().to_vec();
        let n = data.len() as i32;
        self.write(&data, 0, n)
    }

    /// `Client.SendTo(write, length, SocketFlags.None, hostEndPoint)`: from index 0, errors swallowed.
    fn write(&mut self, write: &[u8], _offset: i32, length: i32) -> Result<(), Exception> {
        self.verify_connected();
        let ep = self.host_end_point;
        if let (Some(c), Some(ep)) = (self.client.as_mut(), ep) {
            let _ = c.send(write, length.max(0) as usize, &ep);
        }
        Ok(())
    }

    fn write_line(&mut self, line: &str) -> Result<(), Exception> {
        self.verify_connected();
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
    fn multicast_range_uses_signed_compare() {
        assert!(UdpSerialConnect::is_in_range("224.0.0.0", "239.255.255.255", "239.255.145.50").unwrap());
        assert!(!UdpSerialConnect::is_in_range("224.0.0.0", "239.255.255.255", "127.0.0.1").unwrap());
        assert!(!UdpSerialConnect::is_in_range("224.0.0.0", "239.255.255.255", "240.0.0.1").unwrap());
    }

    #[test]
    fn sends_to_host_and_reads_replies() {
        let peer = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let port = peer.local_addr().unwrap().port();
        let mut u = UdpSerialConnect::new();
        u.open_host("127.0.0.1", &port.to_string()).unwrap();
        assert!(u.is_open());
        assert_eq!(u.port_name(), format!("UDPCl{port}"));
        u.write_str("hé").unwrap();
        let mut r = [0u8; 8];
        let (n, from) = peer.recv_from(&mut r).unwrap();
        assert_eq!(&r[..n], "hé".as_bytes(), "UTF-8 text");
        peer.send_to(b"ok", from).unwrap();
        let mut b = [0u8; 2];
        let mut got = 0;
        let t0 = std::time::Instant::now();
        while got < 2 && t0.elapsed().as_secs() < 3 {
            got += u.read(&mut b, got as i32, 2 - got as i32).unwrap() as usize;
        }
        assert_eq!(&b, b"ok");
        u.close().unwrap();
        assert!(!u.is_open());
    }
}
