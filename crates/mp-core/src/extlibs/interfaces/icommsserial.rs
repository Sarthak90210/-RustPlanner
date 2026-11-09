//! Port of `ExtLibs/Interfaces/ICommsSerial.cs`

use std::any::Any;

use crate::dotnet::{Exception, Stream};

/// `MissionPlanner.Comms.ICommsSerial : IDisposable`
///
/// Properties become getter/setter pairs. Members whose C# implementations throw (reads,
/// writes, `Open`, `BytesToRead`) return `Result`. `as_any` stands in for the `is`/`as` type
/// tests callers make on the concrete transport.
pub trait ICommsSerial: Send {
    // Properties
    /// `Stream BaseStream { get; }`
    fn base_stream(&mut self) -> Result<Box<dyn Stream + '_>, Exception>;

    fn baud_rate(&self) -> i32;
    fn set_baud_rate(&mut self, value: i32);

    fn bytes_to_read(&mut self) -> Result<i32, Exception>;

    fn bytes_to_write(&mut self) -> i32;

    fn data_bits(&self) -> i32;
    fn set_data_bits(&mut self, value: i32);

    fn dtr_enable(&self) -> bool;
    fn set_dtr_enable(&mut self, value: bool);

    fn is_open(&mut self) -> bool;

    fn port_name(&self) -> String;
    fn set_port_name(&mut self, value: &str);

    fn read_buffer_size(&self) -> i32;
    fn set_read_buffer_size(&mut self, value: i32);

    fn read_timeout(&self) -> i32;
    fn set_read_timeout(&mut self, value: i32);

    fn rts_enable(&self) -> bool;
    fn set_rts_enable(&mut self, value: bool);

    fn write_buffer_size(&self) -> i32;
    fn set_write_buffer_size(&mut self, value: i32);

    fn write_timeout(&self) -> i32;
    fn set_write_timeout(&mut self, value: i32);

    // Methods (from the SerialPort class)
    fn close(&mut self) -> Result<(), Exception>;

    fn discard_in_buffer(&mut self) -> Result<(), Exception>;

    fn open(&mut self) -> Result<(), Exception>;

    /// `int Read(byte[] buffer, int offset, int count)`
    fn read(&mut self, buffer: &mut [u8], offset: i32, count: i32) -> Result<i32, Exception>;

    fn read_byte(&mut self) -> Result<i32, Exception>;

    fn read_char(&mut self) -> Result<i32, Exception>;

    fn read_existing(&mut self) -> Result<String, Exception>;

    fn read_line(&mut self) -> Result<String, Exception>;

    /// `void Write(string text)`
    fn write_str(&mut self, text: &str) -> Result<(), Exception>;

    /// `void Write(byte[] buffer, int offset, int count)`
    fn write(&mut self, buffer: &[u8], offset: i32, count: i32) -> Result<(), Exception>;

    fn write_line(&mut self, text: &str) -> Result<(), Exception>;

    #[allow(non_snake_case)]
    fn toggleDTR(&mut self) -> Result<(), Exception>;

    /// `IDisposable.Dispose()`
    fn dispose(&mut self);

    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
}
