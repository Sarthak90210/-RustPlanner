//! Port of `ExtLibs/Comms/CommsStream.cs`

use std::sync::{Arc, Mutex};

use crate::dotnet::{Exception, Stream};
use crate::extlibs::interfaces::ICommsSerial;

/// The `_len` and `Position` of one C# `CommsStream` object. A transport that keeps its
/// `BaseStream` (CommsInjection) holds this so every view of the stream sees the same values.
#[derive(Debug, Default)]
pub struct CommsStreamState {
    pub len: i64,
    pub position: i64,
}

/// `CommsStream : Stream` over an `ICommsSerial`, with a caller-supplied length.
pub struct CommsStream<'a> {
    state: Arc<Mutex<CommsStreamState>>,
    pub comms_serial: &'a mut dyn ICommsSerial,
}

impl<'a> CommsStream<'a> {
    /// `new CommsStream(comm, len)`
    pub fn new(comm: &'a mut dyn ICommsSerial, len: i64) -> Self {
        let s = CommsStream { state: Arc::new(Mutex::new(CommsStreamState::default())), comms_serial: comm };
        s.set_length(len);
        s
    }

    /// A view of an existing stream object's state.
    pub fn with_state(comm: &'a mut dyn ICommsSerial, state: Arc<Mutex<CommsStreamState>>) -> Self {
        CommsStream { state, comms_serial: comm }
    }

    fn st(&self) -> std::sync::MutexGuard<'_, CommsStreamState> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn can_read(&self) -> bool {
        true
    }

    pub fn can_write(&self) -> bool {
        true
    }

    /// `Position { set; }`
    pub fn set_position(&self, value: i64) {
        self.st().position = value;
    }

    /// `Seek` ignores its arguments and returns the position.
    pub fn seek(&mut self, _offset: i64) -> i64 {
        self.st().position
    }

    pub fn set_length(&self, value: i64) {
        self.st().len = value;
    }

    /// `Read(buffer, offset, count)`
    pub fn read_at(&mut self, buffer: &mut [u8], offset: i32, count: i32) -> Result<i32, Exception> {
        let read = self.comms_serial.read(buffer, offset, count)?;
        self.st().position += read as i64;
        Ok(read)
    }

    /// `Write(buffer, offset, count)`
    pub fn write_at(&mut self, buffer: &[u8], offset: i32, count: i32) -> Result<(), Exception> {
        self.comms_serial.write(buffer, offset, count)
    }
}

impl Stream for CommsStream<'_> {
    fn can_seek(&self) -> bool {
        false
    }
    fn position(&self) -> Result<i64, Exception> {
        Ok(self.st().position)
    }
    fn length(&self) -> Result<i64, Exception> {
        Ok(self.st().len)
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        let n = buffer.len() as i32;
        self.read_at(buffer, 0, n).map(|r| r as usize)
    }
    fn write(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let n = buffer.len() as i32;
        self.write_at(buffer, 0, n)
    }
}
