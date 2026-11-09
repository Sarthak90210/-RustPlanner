//! `System.IO.Ports.SerialPort` (the BCL class `WinSerialPort` derives from), emulated over the
//! `serialport` crate.
//!
//! Mission Planner ships Mono's copy of this class in `ExtLibs/Comms/System.IO.Ports`; the
//! manifest maps those files here. The emulation keeps the observable .NET behaviour the comms
//! layer relies on: properties set while closed are applied on `Open`, reads block until at
//! least one byte or `ReadTimeout` (`TimeoutException`), `ReadLine` keeps unread data on a
//! timeout, text is encoded with `Encoding.ASCII`, and members used on a closed port throw
//! `InvalidOperationException` with the BCL messages.

use super::{ascii_get_bytes, ascii_get_string, Exception, Stream};

/// `System.IO.Ports.Handshake`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Handshake {
    #[default]
    None,
    XOnXOff,
    RequestToSend,
    RequestToSendXOnXOff,
}

/// `SerialPort.InfiniteTimeout`
pub const INFINITE_TIMEOUT: i32 = -1;

fn port_closed() -> Exception {
    Exception::InvalidOperation("The port is closed.".into())
}

fn timed_out() -> Exception {
    Exception::Timeout("The operation has timed out.".into())
}

#[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
type Port = Box<dyn serialport::SerialPort>;
#[cfg(any(target_os = "android", target_os = "ios", target_arch = "wasm32"))]
type Port = ();

/// `System.IO.Ports.SerialPort`
pub struct SerialPort {
    port: Option<Port>,
    /// Bytes read from the device but not yet returned (ReadLine look-ahead).
    pending: Vec<u8>,
    baud_rate: i32,
    data_bits: i32,
    dtr_enable: bool,
    rts_enable: bool,
    handshake: Handshake,
    port_name: String,
    read_timeout: i32,
    write_timeout: i32,
    read_buffer_size: i32,
    write_buffer_size: i32,
    pub new_line: String,
}

impl Default for SerialPort {
    fn default() -> Self {
        Self::new()
    }
}

impl SerialPort {
    /// `new SerialPort()`: COM1, 9600 8N1, no handshake, infinite timeouts.
    pub fn new() -> Self {
        SerialPort {
            port: None,
            pending: vec![],
            baud_rate: 9600,
            data_bits: 8,
            dtr_enable: false,
            rts_enable: false,
            handshake: Handshake::None,
            port_name: "COM1".into(),
            read_timeout: INFINITE_TIMEOUT,
            write_timeout: INFINITE_TIMEOUT,
            read_buffer_size: 4096,
            write_buffer_size: 2048,
            new_line: "\n".into(),
        }
    }

    pub fn is_open(&self) -> bool {
        self.port.is_some()
    }

    pub fn port_name(&self) -> &str {
        &self.port_name
    }

    pub fn set_port_name(&mut self, value: &str) -> Result<(), Exception> {
        if value.is_empty() {
            return Err(Exception::Argument("The PortName cannot be empty.\r\nParameter name: PortName".into()));
        }
        if self.is_open() {
            return Err(Exception::InvalidOperation("'PortName' cannot be set while the port is open.".into()));
        }
        self.port_name = value.to_string();
        Ok(())
    }

    pub fn baud_rate(&self) -> i32 {
        self.baud_rate
    }

    pub fn set_baud_rate(&mut self, value: i32) -> Result<(), Exception> {
        if value <= 0 {
            return Err(Exception::ArgumentOutOfRange("Positive number required.\r\nParameter name: BaudRate".into()));
        }
        #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
        if let Some(p) = self.port.as_mut() {
            p.set_baud_rate(value as u32).map_err(map_err)?;
        }
        self.baud_rate = value;
        Ok(())
    }

    pub fn data_bits(&self) -> i32 {
        self.data_bits
    }

    pub fn set_data_bits(&mut self, value: i32) -> Result<(), Exception> {
        if !(5..=8).contains(&value) {
            return Err(Exception::ArgumentOutOfRange("Argument must be between 5 and 8.\r\nParameter name: DataBits".into()));
        }
        #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
        if let Some(p) = self.port.as_mut() {
            p.set_data_bits(data_bits(value)).map_err(map_err)?;
        }
        self.data_bits = value;
        Ok(())
    }

    pub fn dtr_enable(&self) -> bool {
        self.dtr_enable
    }

    pub fn set_dtr_enable(&mut self, value: bool) -> Result<(), Exception> {
        #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
        if let Some(p) = self.port.as_mut() {
            p.write_data_terminal_ready(value).map_err(map_err)?;
        }
        self.dtr_enable = value;
        Ok(())
    }

    pub fn rts_enable(&self) -> bool {
        self.rts_enable
    }

    pub fn set_rts_enable(&mut self, value: bool) -> Result<(), Exception> {
        if matches!(self.handshake, Handshake::RequestToSend | Handshake::RequestToSendXOnXOff) {
            return Err(Exception::InvalidOperation(
                "RtsEnable cannot be accessed if Handshake is set to RequestToSend or RequestToSendXOnXOff.".into(),
            ));
        }
        #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
        if let Some(p) = self.port.as_mut() {
            p.write_request_to_send(value).map_err(map_err)?;
        }
        self.rts_enable = value;
        Ok(())
    }

    pub fn handshake(&self) -> Handshake {
        self.handshake
    }

    pub fn set_handshake(&mut self, value: Handshake) -> Result<(), Exception> {
        #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
        if let Some(p) = self.port.as_mut() {
            p.set_flow_control(flow_control(value)).map_err(map_err)?;
        }
        self.handshake = value;
        Ok(())
    }

    pub fn read_timeout(&self) -> i32 {
        self.read_timeout
    }

    pub fn set_read_timeout(&mut self, value: i32) -> Result<(), Exception> {
        if value < 0 && value != INFINITE_TIMEOUT {
            return Err(Exception::ArgumentOutOfRange(
                "Timeout must be greater than zero or equal to InfiniteTimeout.\r\nParameter name: ReadTimeout".into(),
            ));
        }
        self.read_timeout = value;
        Ok(())
    }

    pub fn write_timeout(&self) -> i32 {
        self.write_timeout
    }

    pub fn set_write_timeout(&mut self, value: i32) -> Result<(), Exception> {
        if value <= 0 && value != INFINITE_TIMEOUT {
            return Err(Exception::ArgumentOutOfRange(
                "Timeout must be greater than zero or equal to InfiniteTimeout.\r\nParameter name: WriteTimeout".into(),
            ));
        }
        self.write_timeout = value;
        Ok(())
    }

    pub fn read_buffer_size(&self) -> i32 {
        self.read_buffer_size
    }

    pub fn set_read_buffer_size(&mut self, value: i32) -> Result<(), Exception> {
        if value <= 0 {
            return Err(Exception::ArgumentOutOfRange("Positive number required.\r\nParameter name: ReadBufferSize".into()));
        }
        if self.is_open() {
            return Err(Exception::InvalidOperation("ReadBufferSize cannot be set while the port is open.".into()));
        }
        self.read_buffer_size = value;
        Ok(())
    }

    pub fn write_buffer_size(&self) -> i32 {
        self.write_buffer_size
    }

    pub fn set_write_buffer_size(&mut self, value: i32) -> Result<(), Exception> {
        if value <= 0 {
            return Err(Exception::ArgumentOutOfRange("Positive number required.\r\nParameter name: WriteBufferSize".into()));
        }
        if self.is_open() {
            return Err(Exception::InvalidOperation("WriteBufferSize cannot be set while the port is open.".into()));
        }
        self.write_buffer_size = value;
        Ok(())
    }

    /// `Open()`
    #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
    pub fn open(&mut self) -> Result<(), Exception> {
        if self.is_open() {
            return Err(Exception::InvalidOperation("The port is already open.".into()));
        }
        let mut p = serialport::new(self.port_name.clone(), self.baud_rate as u32)
            .data_bits(data_bits(self.data_bits))
            .flow_control(flow_control(self.handshake))
            .timeout(std::time::Duration::from_millis(1))
            .open()
            .map_err(|e| match e.kind {
                serialport::ErrorKind::NoDevice => Exception::Io(format!("The port '{}' does not exist.", self.port_name)),
                serialport::ErrorKind::Io(std::io::ErrorKind::PermissionDenied) => {
                    Exception::UnauthorizedAccess(format!("Access to the port '{}' is denied.", self.port_name))
                }
                serialport::ErrorKind::Io(std::io::ErrorKind::NotFound) => {
                    Exception::Io(format!("The port '{}' does not exist.", self.port_name))
                }
                _ => Exception::Io(e.description),
            })?;
        p.write_data_terminal_ready(self.dtr_enable).map_err(map_err)?;
        if !matches!(self.handshake, Handshake::RequestToSend | Handshake::RequestToSendXOnXOff) {
            p.write_request_to_send(self.rts_enable).map_err(map_err)?;
        }
        self.pending.clear();
        self.port = Some(p);
        Ok(())
    }

    #[cfg(any(target_os = "android", target_os = "ios", target_arch = "wasm32"))]
    pub fn open(&mut self) -> Result<(), Exception> {
        Err(Exception::NotSupported("Serial ports are not available on this platform.".into()))
    }

    /// `Close()`
    pub fn close(&mut self) {
        self.port = None;
        self.pending.clear();
    }

    #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
    fn os_bytes_to_read(&self) -> Result<usize, Exception> {
        let p = self.port.as_ref().ok_or_else(port_closed)?;
        Ok(p.bytes_to_read().map_err(map_err)? as usize)
    }

    #[cfg(any(target_os = "android", target_os = "ios", target_arch = "wasm32"))]
    fn os_bytes_to_read(&self) -> Result<usize, Exception> {
        Err(port_closed())
    }

    /// `BytesToRead`
    pub fn bytes_to_read(&self) -> Result<i32, Exception> {
        Ok((self.os_bytes_to_read()? + self.pending.len()) as i32)
    }

    /// `BytesToWrite`
    pub fn bytes_to_write(&self) -> Result<i32, Exception> {
        let p = self.port.as_ref().ok_or_else(port_closed)?;
        os::bytes_to_write(p)
    }

    /// Fill `pending` with whatever the device has, waiting up to `ReadTimeout` for the first
    /// byte. Returns false on a timeout.
    #[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
    fn fill(&mut self) -> Result<bool, Exception> {
        use std::io::Read;
        let start = std::time::Instant::now();
        let timeout = self.read_timeout;
        let p = self.port.as_mut().ok_or_else(port_closed)?;
        let mut buf = [0u8; 4096];
        loop {
            match p.read(&mut buf) {
                Ok(0) => {}
                Ok(n) => {
                    self.pending.extend_from_slice(&buf[..n]);
                    return Ok(true);
                }
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut || e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(e) => return Err(Exception::Io(e.to_string())),
            }
            if timeout != INFINITE_TIMEOUT && start.elapsed().as_millis() as i64 >= timeout as i64 {
                return Ok(false);
            }
        }
    }

    #[cfg(any(target_os = "android", target_os = "ios", target_arch = "wasm32"))]
    fn fill(&mut self) -> Result<bool, Exception> {
        Err(port_closed())
    }

    /// `Read(byte[] buffer, int offset, int count)`
    pub fn read(&mut self, buffer: &mut [u8], offset: i32, count: i32) -> Result<i32, Exception> {
        if !self.is_open() {
            return Err(port_closed());
        }
        if offset < 0 || count < 0 {
            return Err(Exception::ArgumentOutOfRange("Non-negative number required.".into()));
        }
        if buffer.len() < (offset + count) as usize {
            return Err(Exception::Argument(
                "Offset and length were out of bounds for the array or count is greater than the number of elements from index to the end of the source collection.".into(),
            ));
        }
        if count == 0 {
            return Ok(0);
        }
        if self.pending.is_empty() && !self.fill()? {
            return Err(timed_out());
        }
        let n = (count as usize).min(self.pending.len());
        let o = offset as usize;
        buffer[o..o + n].copy_from_slice(&self.pending[..n]);
        self.pending.drain(..n);
        Ok(n as i32)
    }

    /// `ReadByte()`
    pub fn read_byte(&mut self) -> Result<i32, Exception> {
        let mut b = [0u8; 1];
        self.read(&mut b, 0, 1)?;
        Ok(b[0] as i32)
    }

    /// `ReadChar()`: one character decoded with `Encoding.ASCII`.
    pub fn read_char(&mut self) -> Result<i32, Exception> {
        let b = self.read_byte()?;
        Ok(if b < 0x80 { b } else { '?' as i32 })
    }

    /// `ReadExisting()`
    pub fn read_existing(&mut self) -> Result<String, Exception> {
        if !self.is_open() {
            return Err(port_closed());
        }
        let n = self.os_bytes_to_read()?;
        if n > 0 {
            self.fill()?;
        }
        let s = ascii_get_string(&self.pending);
        self.pending.clear();
        Ok(s)
    }

    /// `ReadLine()`: up to `NewLine`, which is consumed but not returned. On a timeout the
    /// partial line stays buffered.
    pub fn read_line(&mut self) -> Result<String, Exception> {
        if !self.is_open() {
            return Err(port_closed());
        }
        let nl = ascii_get_bytes(&self.new_line);
        let start = std::time::Instant::now();
        loop {
            if let Some(i) = self.pending.windows(nl.len()).position(|w| w == nl.as_slice()) {
                let line = ascii_get_string(&self.pending[..i]);
                self.pending.drain(..i + nl.len());
                return Ok(line);
            }
            let save = self.read_timeout;
            if save != INFINITE_TIMEOUT {
                let left = save as i64 - start.elapsed().as_millis() as i64;
                if left <= 0 {
                    return Err(timed_out());
                }
                self.read_timeout = left as i32;
            }
            let got = self.fill();
            self.read_timeout = save;
            if !got? {
                return Err(timed_out());
            }
        }
    }

    /// `Write(byte[] buffer, int offset, int count)`
    pub fn write(&mut self, buffer: &[u8], offset: i32, count: i32) -> Result<(), Exception> {
        let timeout = self.write_timeout;
        let p = self.port.as_mut().ok_or_else(port_closed)?;
        if offset < 0 || count < 0 {
            return Err(Exception::ArgumentOutOfRange("Non-negative number required.".into()));
        }
        if buffer.len() < (offset + count) as usize {
            return Err(Exception::Argument(
                "Offset and length were out of bounds for the array or count is greater than the number of elements from index to the end of the source collection.".into(),
            ));
        }
        let data = &buffer[offset as usize..(offset + count) as usize];
        os::write_all(p, data, timeout)
    }

    /// `Write(string text)`: `Encoding.ASCII`.
    pub fn write_str(&mut self, text: &str) -> Result<(), Exception> {
        let b = ascii_get_bytes(text);
        let n = b.len() as i32;
        self.write(&b, 0, n)
    }

    /// `WriteLine(string text)`: text + `NewLine`.
    pub fn write_line(&mut self, text: &str) -> Result<(), Exception> {
        let s = format!("{}{}", text, self.new_line);
        self.write_str(&s)
    }

    /// `DiscardInBuffer()`
    pub fn discard_in_buffer(&mut self) -> Result<(), Exception> {
        let p = self.port.as_ref().ok_or_else(port_closed)?;
        os::clear_input(p)?;
        self.pending.clear();
        Ok(())
    }

    /// `BaseStream`: only while open.
    pub fn base_stream(&mut self) -> Result<SerialStream<'_>, Exception> {
        if !self.is_open() {
            return Err(Exception::InvalidOperation("The BaseStream is only available when the port is open.".into()));
        }
        Ok(SerialStream { port: self })
    }

    /// `SerialPort.GetPortNames()`
    pub fn get_port_names() -> Result<Vec<String>, Exception> {
        os::port_names()
    }
}

/// The calls that need the OS serial driver.
#[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
mod os {
    use super::{map_err, timed_out, Exception, Port, INFINITE_TIMEOUT};

    pub fn bytes_to_write(p: &Port) -> Result<i32, Exception> {
        Ok(p.bytes_to_write().map_err(map_err)? as i32)
    }

    pub fn write_all(p: &mut Port, data: &[u8], timeout: i32) -> Result<(), Exception> {
        use std::io::Write;
        let to = if timeout == INFINITE_TIMEOUT { u64::MAX / 4 } else { timeout as u64 };
        p.set_timeout(std::time::Duration::from_millis(to)).map_err(map_err)?;
        let r = p.write_all(data);
        let r = r.and_then(|_| p.flush());
        p.set_timeout(std::time::Duration::from_millis(1)).map_err(map_err)?;
        r.map_err(|e| if e.kind() == std::io::ErrorKind::TimedOut { timed_out() } else { Exception::Io(e.to_string()) })
    }

    pub fn clear_input(p: &Port) -> Result<(), Exception> {
        p.clear(serialport::ClearBuffer::Input).map_err(map_err)
    }

    pub fn port_names() -> Result<Vec<String>, Exception> {
        Ok(serialport::available_ports().map_err(map_err)?.into_iter().map(|p| p.port_name).collect())
    }
}

#[cfg(any(target_os = "android", target_os = "ios", target_arch = "wasm32"))]
mod os {
    use super::{port_closed, Exception, Port};

    pub fn bytes_to_write(_p: &Port) -> Result<i32, Exception> {
        Err(port_closed())
    }

    pub fn write_all(_p: &mut Port, _data: &[u8], _timeout: i32) -> Result<(), Exception> {
        Err(port_closed())
    }

    pub fn clear_input(_p: &Port) -> Result<(), Exception> {
        Err(port_closed())
    }

    pub fn port_names() -> Result<Vec<String>, Exception> {
        Ok(vec![])
    }
}

/// The `SerialStream` behind `SerialPort.BaseStream`.
pub struct SerialStream<'a> {
    port: &'a mut SerialPort,
}

impl Stream for SerialStream<'_> {
    fn can_timeout(&self) -> bool {
        true
    }
    fn read_timeout(&self) -> i32 {
        self.port.read_timeout
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        let n = buffer.len() as i32;
        self.port.read(buffer, 0, n).map(|r| r as usize)
    }
    fn write(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let n = buffer.len() as i32;
        self.port.write(buffer, 0, n)
    }
}

#[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
fn map_err(e: serialport::Error) -> Exception {
    Exception::Io(e.description)
}

#[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
fn data_bits(v: i32) -> serialport::DataBits {
    match v {
        5 => serialport::DataBits::Five,
        6 => serialport::DataBits::Six,
        7 => serialport::DataBits::Seven,
        _ => serialport::DataBits::Eight,
    }
}

#[cfg(not(any(target_os = "android", target_os = "ios", target_arch = "wasm32")))]
fn flow_control(h: Handshake) -> serialport::FlowControl {
    match h {
        Handshake::None => serialport::FlowControl::None,
        Handshake::XOnXOff => serialport::FlowControl::Software,
        Handshake::RequestToSend | Handshake::RequestToSendXOnXOff => serialport::FlowControl::Hardware,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closed_port_behaviour() {
        let mut p = SerialPort::new();
        assert_eq!(p.baud_rate(), 9600);
        assert_eq!(p.read_timeout(), INFINITE_TIMEOUT);
        assert!(matches!(p.bytes_to_read(), Err(Exception::InvalidOperation(_))));
        assert!(matches!(p.read(&mut [0u8; 1], 0, 1), Err(Exception::InvalidOperation(_))));
        assert!(matches!(p.base_stream(), Err(Exception::InvalidOperation(_))));
        assert!(p.set_baud_rate(0).is_err());
        assert!(p.set_data_bits(9).is_err());
        p.set_handshake(Handshake::RequestToSend).unwrap();
        assert!(p.set_rts_enable(true).is_err());
        assert!(p.set_port_name("").is_err());
        p.set_port_name("COM7").unwrap();
        assert_eq!(p.port_name(), "COM7");
    }
}
