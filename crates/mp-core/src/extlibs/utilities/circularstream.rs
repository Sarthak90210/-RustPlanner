//! Port of `ExtLibs/Utilities/CircularStream.cs`
//!
//! (namespace `CircularBuffer`)
//!
//! A `Stream` over a `CircularBuffer<byte>`: writes put bytes, reads take them. Position,
//! Seek and SetLength throw NotImplementedException, although CanSeek is true.

use super::circularbuffer::CircularBuffer;
use crate::dotnet::{Exception, SeekOrigin, Stream};

fn not_implemented() -> Exception {
    Exception::NotImplemented("The method or operation is not implemented.".into())
}

/// `public class CircularStream : Stream`
#[derive(Debug, Clone)]
pub struct CircularStream {
    buffer: CircularBuffer<u8>,
}

impl CircularStream {
    /// `new CircularStream(int bufferCapacity)`
    pub fn new(buffer_capacity: i32) -> Result<Self, Exception> {
        Ok(CircularStream { buffer: CircularBuffer::new(buffer_capacity)? })
    }

    /// `Position` getter
    pub fn get_position(&self) -> Result<i64, Exception> {
        Err(not_implemented())
    }

    /// `Position` setter
    pub fn set_position(&mut self, _value: i64) -> Result<(), Exception> {
        Err(not_implemented())
    }

    /// `Capacity`
    pub fn capacity(&self) -> i32 {
        self.buffer.capacity()
    }

    pub fn set_capacity(&mut self, value: i32) -> Result<(), Exception> {
        self.buffer.set_capacity(value)
    }

    /// `Length`: the buffered byte count.
    pub fn len(&self) -> i64 {
        self.buffer.size() as i64
    }

    pub fn is_empty(&self) -> bool {
        self.buffer.size() == 0
    }

    /// `CanRead`
    pub fn can_read(&self) -> bool {
        true
    }

    /// `CanWrite`
    pub fn can_write(&self) -> bool {
        true
    }

    /// `GetBuffer()`: the whole backing array.
    pub fn get_buffer(&self) -> Vec<u8> {
        self.buffer.get_buffer().to_vec()
    }

    /// `ToArray()`: the buffered bytes in order.
    pub fn to_array(&self) -> Vec<u8> {
        self.buffer.to_array()
    }

    /// `Write(byte[] buffer, int offset, int count)`
    pub fn write_range(&mut self, buffer: &[u8], offset: i32, count: i32) -> Result<(), Exception> {
        self.buffer.put_range(buffer, offset, count).map(|_| ())
    }

    /// `WriteByte(byte value)`
    pub fn write_byte(&mut self, value: u8) -> Result<(), Exception> {
        self.buffer.put(value)
    }

    /// `Read(byte[] buffer, int offset, int count)`
    pub fn read_range(&mut self, buffer: &mut [u8], offset: i32, count: i32) -> i32 {
        self.buffer.get_range(buffer, offset, count)
    }

    /// `ReadByte()`: `buffer.Get()` widened to int; an empty buffer throws.
    pub fn read_byte(&mut self) -> Result<i32, Exception> {
        self.buffer.get().map(|b| b as i32)
    }

    /// `Seek(offset, origin)`
    pub fn seek(&mut self, _offset: i64, _origin: SeekOrigin) -> Result<i64, Exception> {
        Err(not_implemented())
    }

    /// `SetLength(value)`
    pub fn set_length(&mut self, _value: i64) -> Result<(), Exception> {
        Err(not_implemented())
    }
}

impl Stream for CircularStream {
    /// `CanSeek`: true, although Seek throws.
    fn can_seek(&self) -> bool {
        true
    }

    fn position(&self) -> Result<i64, Exception> {
        Err(not_implemented())
    }

    fn length(&self) -> Result<i64, Exception> {
        Ok(self.len())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        let n = buffer.len() as i32;
        Ok(self.read_range(buffer, 0, n).max(0) as usize)
    }

    fn write(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        self.write_range(buffer, 0, buffer.len() as i32)
    }

    /// `Flush()`: nothing to do.
    fn flush(&mut self) -> Result<(), Exception> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bytes_come_out_in_order() {
        let mut s = CircularStream::new(4).unwrap();
        s.write_range(&[1, 2, 3], 0, 3).unwrap();
        assert_eq!(s.len(), 3);
        assert_eq!(s.read_byte().unwrap(), 1);
        let mut b = [0u8; 4];
        assert_eq!(Stream::read(&mut s, &mut b).unwrap(), 2);
        assert_eq!(&b[..2], &[2, 3]);
        assert!(s.read_byte().is_err());
        assert!(s.seek(0, SeekOrigin::Begin).is_err() && s.get_position().is_err());
        assert!(Stream::can_seek(&s));
    }
}
