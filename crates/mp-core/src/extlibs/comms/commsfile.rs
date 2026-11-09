//! Port of `ExtLibs/Comms/CommsFile.cs`

use std::any::Any;

use super::commsbase::CommsBase;
use crate::dotnet::{DateTime, Exception, FileStream, Stream};
use crate::extlibs::interfaces::ICommsSerial;

fn null_ref() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

/// `CommsFile : CommsBase, ICommsSerial, IDisposable`: replays a file, rate limited to `bps`.
pub struct CommsFile {
    #[allow(dead_code)]
    base: CommsBase,
    _stream: Option<FileStream>,
    bytecredit: i32,
    currentbps: i32,
    lastread: DateTime,
    lastsecond: i32,
    length: i64,
    step: i32,
    pub bps: i32,
    baud_rate: i32,
    bytes_to_write: i32,
    data_bits: i32,
    dtr_enable: bool,
    port_name: String,
    read_buffer_size: i32,
    read_timeout: i32,
    rts_enable: bool,
    write_buffer_size: i32,
    write_timeout: i32,
}

impl Default for CommsFile {
    fn default() -> Self {
        Self::new()
    }
}

impl CommsFile {
    /// `CommsFile()`
    pub fn new() -> Self {
        CommsFile {
            base: CommsBase::new(),
            _stream: None,
            bytecredit: 0,
            currentbps: 0,
            lastread: DateTime::MIN_VALUE,
            lastsecond: 0,
            length: 0,
            step: 0,
            bps: 0,
            baud_rate: 0,
            bytes_to_write: 0,
            data_bits: 0,
            dtr_enable: false,
            port_name: String::new(),
            read_buffer_size: 0,
            read_timeout: 0,
            rts_enable: false,
            write_buffer_size: 0,
            write_timeout: 0,
        }
    }

    /// `CommsFile(string filename)`
    pub fn with_file(filename: &str) -> Result<Self, Exception> {
        let mut c = Self::new();
        c.open_file(filename)?;
        Ok(c)
    }

    /// The `BaseStream` setter: also records the stream's length.
    pub fn set_base_stream(&mut self, value: FileStream) -> Result<(), Exception> {
        self.length = value.length()?;
        self._stream = Some(value);
        Ok(())
    }

    /// `public int BytesToWrite { set; }`
    pub fn set_bytes_to_write(&mut self, value: i32) {
        self.bytes_to_write = value;
    }

    /// `Open(string filename)`
    pub fn open_file(&mut self, filename: &str) -> Result<(), Exception> {
        self.bps = i32::MAX;
        self.port_name = filename.to_string();
        let s = FileStream::open_read(&self.port_name)?;
        self.set_base_stream(s)?;
        self.length = self._stream.as_ref().unwrap().length()?;
        Ok(())
    }

    /// `SetLength(long length)`: changes only the end point `IsOpen` compares against.
    pub fn set_length(&mut self, length: i64) {
        self.length = length;
    }

    fn stream(&mut self) -> Result<&mut FileStream, Exception> {
        self._stream.as_mut().ok_or_else(null_ref)
    }

    /// `DateTime.Now.Second`
    fn now_second() -> i32 {
        DateTime::now().parts().5 as i32
    }
}

impl ICommsSerial for CommsFile {
    fn base_stream(&mut self) -> Result<Box<dyn Stream + '_>, Exception> {
        Ok(Box::new(self.stream()?))
    }

    fn baud_rate(&self) -> i32 {
        self.baud_rate
    }
    fn set_baud_rate(&mut self, value: i32) {
        self.baud_rate = value;
    }

    fn bytes_to_read(&mut self) -> Result<i32, Exception> {
        let s = self.stream()?;
        if !s.can_read() {
            return Ok(0);
        }
        Ok((s.length()? - s.position()?) as i32)
    }

    fn bytes_to_write(&mut self) -> i32 {
        self.bytes_to_write
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
        match &self._stream {
            Some(s) if s.can_read() => s.position().map(|p| p < self.length).unwrap_or(false),
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

    /// `Close()`: `BaseStream.Dispose()`.
    fn close(&mut self) -> Result<(), Exception> {
        self.stream()?.dispose();
        Ok(())
    }

    fn discard_in_buffer(&mut self) -> Result<(), Exception> {
        Ok(())
    }

    /// `Open()`: opens `PortName`.
    fn open(&mut self) -> Result<(), Exception> {
        self.bps = i32::MAX;
        let s = FileStream::open_read(&self.port_name)?;
        self.set_base_stream(s)?;
        self.length = self._stream.as_ref().unwrap().length()?;
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8], offset: i32, count: i32) -> Result<i32, Exception> {
        if !self.is_open() {
            return Err(Exception::EndOfStream("File not open".into()));
        }

        loop {
            // check if we have credit and continue
            if count < self.bytecredit {
                self.bytecredit -= count;
                break;
            }

            // get the time taken since last read in seconds
            let lapsed_since_last_read = (DateTime::utc_now() - self.lastread).total_milliseconds() / 1000.0;

            // escape if we are out of range
            if !(0.0..=2.0).contains(&lapsed_since_last_read) {
                break;
            }

            // get our target bps for this time slice.
            let targetbps = crate::dotnet::cast_i32(self.bps as f64 * lapsed_since_last_read).wrapping_add(self.bytecredit);

            // check if out target+count is less than our required bps
            if count < targetbps {
                self.bytecredit = targetbps - count;
                break;
            }

            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        self.lastread = DateTime::utc_now();

        let sec = Self::now_second();
        if self.lastsecond != sec {
            self.currentbps = 0;
            self.lastsecond = sec;
        }

        self.currentbps = self.currentbps.wrapping_add(count);
        let ret = self.stream()?.read_at(buffer, offset as usize, count as usize)? as i32;

        if buffer[0] == 254 && offset == 1 {
            self.step = buffer[1] as i32 + 5 + 2; // + header + checksum
        }

        self.step -= ret;

        Ok(ret)
    }

    fn read_byte(&mut self) -> Result<i32, Exception> {
        self.stream()?.read_byte()
    }

    fn read_char(&mut self) -> Result<i32, Exception> {
        self.stream()?.read_byte()
    }

    fn read_existing(&mut self) -> Result<String, Exception> {
        Ok(String::new())
    }

    fn read_line(&mut self) -> Result<String, Exception> {
        Ok(String::new())
    }

    fn write_str(&mut self, _text: &str) -> Result<(), Exception> {
        Ok(())
    }

    fn write(&mut self, _buffer: &[u8], _offset: i32, _count: i32) -> Result<(), Exception> {
        Ok(())
    }

    fn write_line(&mut self, _text: &str) -> Result<(), Exception> {
        Ok(())
    }

    fn toggleDTR(&mut self) -> Result<(), Exception> {
        Ok(())
    }

    fn dispose(&mut self) {
        if self._stream.is_some() {
            let _ = self.close();
        }
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
    fn replays_a_file() {
        let dir = std::env::temp_dir().join(format!("commsfile_{}", std::process::id()));
        std::fs::write(&dir, [254u8, 3, 1, 2, 3, 4, 5, 6, 7, 8, 9]).unwrap();
        let mut c = CommsFile::with_file(dir.to_str().unwrap()).unwrap();
        assert!(c.is_open());
        assert_eq!(c.bytes_to_read().unwrap(), 11);
        let mut b = [0u8; 11];
        assert_eq!(c.read(&mut b, 0, 1).unwrap(), 1);
        assert_eq!(c.read(&mut b, 1, 10).unwrap(), 10);
        assert!(!c.is_open(), "IsOpen is Position < length");
        assert!(matches!(c.read(&mut b, 0, 1), Err(Exception::EndOfStream(_))));
        c.close().unwrap();
        assert_eq!(c.bytes_to_read().unwrap(), 0, "a disposed stream cannot read");
        let mut empty = CommsFile::new();
        assert!(matches!(empty.bytes_to_read(), Err(Exception::NullReference(_))));
        let _ = std::fs::remove_file(dir);
    }
}
