//! Port of `ExtLibs/Comms/UDPStream.cs`

use super::commsudpserial::UdpSerial;
use crate::dotnet::{Exception, Stream};
use crate::extlibs::interfaces::ICommsSerial;

/// `UDPStream : Stream` over a `UdpSerial`. Length, Position, Seek and SetLength throw
/// NotImplementedException.
pub struct UDPStream<'a> {
    udp_serial: &'a mut UdpSerial,
}

impl<'a> UDPStream<'a> {
    pub fn new(udp_serial: &'a mut UdpSerial) -> Self {
        UDPStream { udp_serial }
    }

    pub fn can_read(&self) -> bool {
        true
    }

    pub fn can_write(&self) -> bool {
        true
    }

    pub fn seek(&mut self, _offset: i64) -> Result<i64, Exception> {
        Err(Exception::NotImplemented("The method or operation is not implemented.".into()))
    }

    pub fn set_length(&mut self, _value: i64) -> Result<(), Exception> {
        Err(Exception::NotImplemented("The method or operation is not implemented.".into()))
    }
}

impl Stream for UDPStream<'_> {
    fn can_seek(&self) -> bool {
        false
    }
    fn position(&self) -> Result<i64, Exception> {
        Err(Exception::NotImplemented("The method or operation is not implemented.".into()))
    }
    fn length(&self) -> Result<i64, Exception> {
        Err(Exception::NotImplemented("The method or operation is not implemented.".into()))
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        let n = buffer.len() as i32;
        self.udp_serial.read(buffer, 0, n).map(|r| r as usize)
    }
    fn write(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let n = buffer.len() as i32;
        self.udp_serial.write(buffer, 0, n)
    }
}
