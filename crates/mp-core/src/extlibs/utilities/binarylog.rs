//! Port of `ExtLibs/Utilities/BinaryLog.cs`
//!
//! Convert a binary (DataFlash `.bin`) log to an ASCII log, and read single messages as
//! objects for `DFLogBuffer`. The boxed `object`s the C# returns are `LogObject`s; a
//! `System.IO.Stream` with `CanSeek` is any `Read + Seek`.

#![allow(non_snake_case, non_camel_case_types)]

use std::io::{Read, Seek, SeekFrom};
use std::sync::RwLock;

use crate::dotnet::io::console_write_line as console_write_line_args;
use crate::dotnet::{bit_converter, double_to_string_culture, float_to_string_culture, Exception, INVARIANT_CULTURE};
use crate::extlibs::utilities::HalfHelper;

/// A seekable `System.IO.Stream`.
pub trait LogStream: Read + Seek + Send {}
impl<T: Read + Seek + Send> LogStream for T {}

/// `Stream.Position`
pub fn stream_position(s: &mut dyn LogStream) -> i64 {
    s.stream_position().map(|p| p as i64).unwrap_or(0)
}

/// `Stream.Length`
pub fn stream_length(s: &mut dyn LogStream) -> i64 {
    let pos = s.stream_position().unwrap_or(0);
    let len = s.seek(SeekFrom::End(0)).unwrap_or(0);
    let _ = s.seek(SeekFrom::Start(pos));
    len as i64
}

/// `Stream.ReadByte()`: -1 at the end of the stream.
pub fn stream_read_byte(s: &mut dyn LogStream) -> i32 {
    let mut b = [0u8; 1];
    match s.read(&mut b) {
        Ok(1) => b[0] as i32,
        _ => -1,
    }
}

/// `Stream.Read(buffer, 0, buffer.Length)` on a file or memory stream: fills the buffer
/// unless the end is reached first; returns the count read.
pub fn stream_read(s: &mut dyn LogStream, buffer: &mut [u8]) -> usize {
    let mut n = 0;
    while n < buffer.len() {
        match s.read(&mut buffer[n..]) {
            Ok(0) | Err(_) => break,
            Ok(k) => n += k,
        }
    }
    n
}

/// `BitConverter.ToInt64`, which `bit_converter` does not carry.
fn to_i64(value: &[u8], start_index: i32) -> Result<i64, Exception> {
    bit_converter::to_u64(value, start_index).map(|v| v as i64)
}

/// `Encoding.ASCII.GetString(bytes)`: bytes above 0x7F decode as '?'.
pub fn ascii_get_string(bytes: &[u8]) -> String {
    bytes.iter().map(|&b| if b < 0x80 { b as char } else { '?' }).collect()
}

/// `Encoding.ASCII.GetString(bytes, index, count)` with its argument checks.
fn ascii_get_string_range(bytes: &[u8], index: usize, count: usize) -> Result<String, Exception> {
    if index + count > bytes.len() {
        return Err(Exception::ArgumentOutOfRange("Index and count must refer to a location within the buffer.\r\nParameter name: bytes".into()));
    }
    Ok(ascii_get_string(&bytes[index..index + count]))
}

/// `new ReadOnlySpan<byte>(message, offset, 64).ToArray()`
fn span_to_array(message: &[u8], offset: usize, len: usize) -> Result<Vec<u8>, Exception> {
    if offset + len > message.len() {
        return Err(Exception::ArgumentOutOfRange("Specified argument was out of the range of valid values.".into()));
    }
    Ok(message[offset..offset + len].to_vec())
}

/// `[StructLayout(LayoutKind.Sequential, Pack = 1)] struct log_Format` (86 bytes)
#[derive(Debug, Clone)]
pub struct log_Format {
    pub type_: u8,
    pub length: u8,
    pub name: [u8; 4],
    pub format: [u8; 16],
    pub labels: [u8; 64],
}

impl log_Format {
    /// `Marshal.SizeOf(log_Format)`
    pub const SIZE: usize = 86;

    /// `Marshal.PtrToStructure` over the bytes read.
    fn from_bytes(b: &[u8; Self::SIZE]) -> Self {
        let mut name = [0u8; 4];
        let mut format = [0u8; 16];
        let mut labels = [0u8; 64];
        name.copy_from_slice(&b[2..6]);
        format.copy_from_slice(&b[6..22]);
        labels.copy_from_slice(&b[22..86]);
        log_Format { type_: b[0], length: b[1], name, format, labels }
    }
}

/// `struct log_format_cache`
#[derive(Debug, Clone, Default)]
pub struct log_format_cache {
    pub type_: u8,
    pub length: u8,
    pub name: String,
    pub format: String,
}

/// `struct UnionArray`: 64 bytes read as 32 `short`s.
#[derive(Debug, Clone, PartialEq)]
pub struct UnionArray {
    bytes: Vec<u8>,
}

impl UnionArray {
    pub fn new(bytes: Vec<u8>) -> Self {
        UnionArray { bytes }
    }

    /// `Bytes`
    pub fn Bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// `Shorts`
    pub fn Shorts(&self) -> Vec<i16> {
        self.bytes.chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]])).collect()
    }
}

impl std::fmt::Display for UnionArray {
    /// `"[" + String.Join(" ", shorts) + "]"`
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s: Vec<String> = self.Shorts().iter().map(|v| v.to_string()).collect();
        write!(f, "[{}]", s.join(" "))
    }
}

/// The boxed values a message decodes to.
#[derive(Debug, Clone, PartialEq)]
pub enum LogObject {
    SByte(i8),
    Byte(u8),
    Int16(i16),
    UInt16(u16),
    Int32(i32),
    UInt32(u32),
    Int64(i64),
    UInt64(u64),
    Single(f32),
    Double(f64),
    String(String),
    Bytes(Vec<u8>),
    UnionArray(UnionArray),
    Null,
}

impl LogObject {
    /// `Extensions.IsNumber(object)`: the integral, floating and decimal type codes.
    pub fn IsNumber(&self) -> bool {
        matches!(
            self,
            LogObject::SByte(_)
                | LogObject::Byte(_)
                | LogObject::Int16(_)
                | LogObject::UInt16(_)
                | LogObject::Int32(_)
                | LogObject::UInt32(_)
                | LogObject::Int64(_)
                | LogObject::UInt64(_)
                | LogObject::Single(_)
                | LogObject::Double(_)
        )
    }

    /// `((IConvertible)a).ToString(CultureInfo.InvariantCulture)` for a number.
    pub fn to_string_invariant(&self) -> String {
        match self {
            LogObject::Single(v) => float_to_string_culture(*v, &INVARIANT_CULTURE),
            LogObject::Double(v) => double_to_string_culture(*v, &INVARIANT_CULTURE),
            other => other.ToString().unwrap_or_default(),
        }
    }

    /// `a?.ToString()` (current culture for the floating types).
    pub fn ToString(&self) -> Option<String> {
        Some(match self {
            LogObject::SByte(v) => v.to_string(),
            LogObject::Byte(v) => v.to_string(),
            LogObject::Int16(v) => v.to_string(),
            LogObject::UInt16(v) => v.to_string(),
            LogObject::Int32(v) => v.to_string(),
            LogObject::UInt32(v) => v.to_string(),
            LogObject::Int64(v) => v.to_string(),
            LogObject::UInt64(v) => v.to_string(),
            LogObject::Single(v) => crate::dotnet::float_to_string(*v),
            LogObject::Double(v) => crate::dotnet::double_to_string(*v),
            LogObject::String(s) => s.clone(),
            LogObject::Bytes(_) => "System.Byte[]".into(),
            LogObject::UnionArray(u) => u.to_string(),
            LogObject::Null => return None,
        })
    }

    /// The `IConvertible` types (every number and `string`); `byte[]`, `UnionArray` and
    /// `null` are not.
    pub fn is_convertible(&self) -> bool {
        self.IsNumber() || matches!(self, LogObject::String(_))
    }
}

/// `public delegate string getFlightMode(string firmware, int modeno)`
pub type getFlightMode = fn(&str, i32) -> Option<String>;

/// `public static event getFlightMode onFlightMode`: a multicast delegate; invoking it
/// runs every handler and yields the last one's result
static ON_FLIGHT_MODE: RwLock<Vec<getFlightMode>> = RwLock::new(Vec::new());

/// `public class BinaryLog`
pub struct BinaryLog {
    logformat: crate::dotnet::collections::Dictionary<String, log_Format>,
    _firmware: String,
    packettypecache: Vec<log_format_cache>,
}

impl Default for BinaryLog {
    fn default() -> Self {
        Self::new()
    }
}

impl BinaryLog {
    pub const HEAD_BYTE1: u8 = 0xA3; // Decimal 163
    pub const HEAD_BYTE2: u8 = 0x95; // Decimal 149

    pub fn new() -> Self {
        BinaryLog { logformat: Default::default(), _firmware: String::new(), packettypecache: vec![log_format_cache::default(); 256] }
    }

    /// `onFlightMode += handler`
    pub fn add_onFlightMode(handler: getFlightMode) {
        ON_FLIGHT_MODE.write().unwrap().push(handler);
    }

    /// `ConvertBin(inputfn, outputfn, progress)`
    pub fn ConvertBin(inputfn: &str, outputfn: &str, progress: Option<&mut dyn FnMut(i32)>) -> Result<(), Exception> {
        BinaryLog::new().ConvertBini(inputfn, outputfn, progress)
    }

    fn ConvertBini(&mut self, inputfn: &str, outputfn: &str, mut progress: Option<&mut dyn FnMut(i32)>) -> Result<(), Exception> {
        use std::io::Write;
        let mut stream = std::fs::File::create(outputfn).map_err(|e| crate::dotnet::io::file_error(outputfn, &e))?;
        let file = std::fs::File::open(inputfn).map_err(|e| crate::dotnet::io::file_error(inputfn, &e))?;
        let mut br = std::io::BufReader::with_capacity(1024 * 1024, file);
        let mut displaytimer = crate::dotnet::DateTime::MIN_VALUE;
        let length = stream_length(&mut br);
        while stream_position(&mut br) < length {
            let now = crate::dotnet::DateTime::now();
            if displaytimer.parts().5 != now.parts().5 {
                let pct = stream_position(&mut br) as f32 / length as f32;
                if let Some(p) = progress.as_mut() {
                    p((pct * 100.0) as i32);
                }
                console_write_line_args(&format!("ConvertBin {}", crate::dotnet::float_to_string(pct * 100.0)));
                displaytimer = now;
            }
            let data = self.ReadMessage(&mut br, length).into_bytes();
            stream.write_all(&data).map_err(|e| Exception::Io(e.to_string()))?;
        }
        Ok(())
    }

    /// `ReadMessage(br, length)`: the next message as a ", "-joined line ending in "\r\n", or
    /// "" at the end. A message that fails to decode is reported and skipped.
    pub fn ReadMessage(&mut self, br: &mut dyn LogStream, length: i64) -> String {
        let mut log_step = 0;
        while stream_position(br) < length {
            let data = stream_read_byte(br) as u8;
            match log_step {
                0 => {
                    if data == Self::HEAD_BYTE1 {
                        log_step += 1;
                    }
                }
                1 => {
                    if data == Self::HEAD_BYTE2 {
                        log_step += 1;
                    } else {
                        log_step = 0;
                    }
                }
                2 => {
                    log_step = 0;
                    match self.message_line(data, br) {
                        Ok(line) => {
                            // we need to know the mav type to use the correct mode list.
                            if line.contains("PARM, RATE_RLL_P") || line.contains("ArduCopter") || line.contains("Copter") {
                                self._firmware = "ArduCopter2".into();
                            } else if line.contains("PARM, H_SWASH_PLATE") || line.contains("ArduCopter") {
                                self._firmware = "ArduCopter2".into();
                            } else if line.contains("PARM, PTCH2SRV_P") || line.contains("ArduPlane") || line.contains("Plane") {
                                self._firmware = "ArduPlane".into();
                            } else if line.contains("PARM, SKID_STEER_OUT") || line.contains("ArduRover") || line.contains("Rover") {
                                self._firmware = "ArduRover".into();
                            } else if line.contains("AntennaTracker") || line.contains("Tracker") {
                                self._firmware = "ArduTracker".into();
                            }
                            return line;
                        }
                        Err(_) => console_write_line_args(&format!("Bad Binary log line {data}")),
                    }
                }
                _ => {}
            }
        }
        String::new()
    }

    /// The try block of `ReadMessage`'s case 2: `String.Join(", ", objects) + "\r\n"`.
    fn message_line(&mut self, data: u8, br: &mut dyn LogStream) -> Result<String, Exception> {
        // String.Join over a null array throws ArgumentNullException
        let objects = self.logEntryObjects(data, br)?.ok_or_else(|| Exception::Argument("Value cannot be null.\r\nParameter name: values".into()))?;
        let mut parts = Vec::with_capacity(objects.len());
        for a in &objects {
            let s = if a.IsNumber() {
                a.to_string_invariant()
            } else if let LogObject::Bytes(b) = a {
                let s = ascii_get_string(b);
                let s = s.trim_matches('\0');
                // Escape \ as \\
                let s = s.replace('\\', "\\\\");
                // Escape certain whitespace characters
                let s = s.replace('\n', "\\n").replace('\r', "\\r").replace('\t', "\\t");
                // Escape all other non-printable characters; Aggregate on an empty sequence throws
                if s.is_empty() {
                    return Err(Exception::InvalidOperation("Sequence contains no elements".into()));
                }
                s.chars().map(|c| if (c as u32) < 32 || (c as u32) > 127 { format!("\\x{:02X}", c as u32) } else { c.to_string() }).collect()
            } else {
                a.ToString().unwrap_or_default()
            };
            parts.push(s);
        }
        Ok(parts.join(", ") + "\r\n")
    }

    /// `ReadMessageTypeOffset(br, length)`: the next message's type and its start (the first
    /// header byte), skipping over its body; (0, 0) at the end.
    pub fn ReadMessageTypeOffset(&mut self, br: &mut dyn LogStream, length: i64) -> (u8, i64) {
        let mut log_step = 0;
        while stream_position(br) < length {
            let data = stream_read_byte(br) as u8;
            match log_step {
                0 => {
                    if data == Self::HEAD_BYTE1 {
                        log_step += 1;
                    }
                }
                1 => {
                    if data == Self::HEAD_BYTE2 {
                        log_step += 1;
                    } else {
                        log_step = 0;
                    }
                }
                2 => {
                    log_step = 0;
                    let pos = stream_position(br) - 3;
                    // read fmt or seek length of packet
                    match self.logEntryFMT(data, br) {
                        Ok(()) => return (data, pos),
                        Err(_) => console_write_line_args(&format!("Bad Binary log line {data}")),
                    }
                }
                _ => {}
            }
        }
        (0, 0)
    }

    /// `logEntryFMT(packettype, br)`
    fn logEntryFMT(&mut self, packettype: u8, br: &mut dyn LogStream) -> Result<(), Exception> {
        match packettype {
            0x80 => {
                // FMT
                let mut bytearray = [0u8; log_Format::SIZE];
                stream_read(br, &mut bytearray);
                let logfmt = log_Format::from_bytes(&bytearray);
                let lgname = ascii_get_string(&logfmt.name).trim_matches('\0').to_string();
                self.logformat.insert(lgname, logfmt.clone());
                self.packettypecache[logfmt.type_ as usize] = log_format_cache {
                    length: logfmt.length,
                    type_: logfmt.type_,
                    name: ascii_get_string(&logfmt.name).trim_matches('\0').to_string(),
                    format: ascii_get_string(&logfmt.format).trim_matches('\0').to_string(),
                };
            }
            _ => {
                let mut size = 0usize;
                if self.packettypecache[packettype as usize].length != 0 {
                    size = self.packettypecache[packettype as usize].length as usize;
                }
                // didnt find a match, return unknown packet type
                if size == 0 {
                    return Ok(());
                }
                // new byte[size - 3] with a negative size throws OverflowException
                if size < 3 {
                    return Err(Exception::Overflow("Arithmetic operation resulted in an overflow.".into()));
                }
                let mut buf = vec![0u8; size - 3];
                stream_read(br, &mut buf);
            }
        }
        Ok(())
    }

    /// `ReadMessageObjects(br, length)`: the next message's objects, `None` at the end.
    pub fn ReadMessageObjects(&mut self, br: &mut dyn LogStream, length: i64) -> Option<Vec<LogObject>> {
        let mut log_step = 0;
        while stream_position(br) < length {
            let data = stream_read_byte(br) as u8;
            match log_step {
                0 => {
                    if data == Self::HEAD_BYTE1 {
                        log_step += 1;
                    }
                }
                1 => {
                    if data == Self::HEAD_BYTE2 {
                        log_step += 1;
                    } else {
                        log_step = 0;
                    }
                }
                2 => {
                    log_step = 0;
                    match self.logEntryObjects(data, br) {
                        Ok(line) => return line,
                        Err(_) => console_write_line_args(&format!("Bad Binary log line {data}")),
                    }
                }
                _ => {}
            }
        }
        None
    }

    /// `logEntryObjects(packettype, br)`: `Ok(None)` for an unknown type.
    fn logEntryObjects(&mut self, packettype: u8, br: &mut dyn LogStream) -> Result<Option<Vec<LogObject>>, Exception> {
        match packettype {
            0x80 => {
                // FMT
                let mut bytearray = [0u8; log_Format::SIZE];
                stream_read(br, &mut bytearray);
                let logfmt = log_Format::from_bytes(&bytearray);
                let lgname = ascii_get_string(&logfmt.name).trim_matches('\0').to_string();
                let lgformat = ascii_get_string(&logfmt.format).trim_matches('\0').to_string();
                let lglabels = ascii_get_string(&logfmt.labels).trim_matches('\0').to_string();
                self.logformat.insert(lgname.clone(), logfmt.clone());
                Ok(Some(vec![
                    LogObject::String("FMT".into()),
                    LogObject::Byte(logfmt.type_),
                    LogObject::Byte(logfmt.length),
                    LogObject::String(lgname),
                    LogObject::String(lgformat),
                    LogObject::String(lglabels),
                ]))
            }
            _ => {
                let mut format = String::new();
                let mut name = String::new();
                let mut size = 0usize;
                if self.packettypecache[packettype as usize].length != 0 {
                    let fmt = &self.packettypecache[packettype as usize];
                    name = fmt.name.clone();
                    format = fmt.format.clone();
                    size = fmt.length as usize;
                } else {
                    let fmts: Vec<log_Format> = self.logformat.values().cloned().collect();
                    for fmt in fmts {
                        self.packettypecache[fmt.type_ as usize] = log_format_cache {
                            length: fmt.length,
                            type_: fmt.type_,
                            name: ascii_get_string(&fmt.name).trim_matches('\0').to_string(),
                            format: ascii_get_string(&fmt.format).trim_matches('\0').to_string(),
                        };
                        if fmt.type_ == packettype {
                            name = self.packettypecache[fmt.type_ as usize].name.clone();
                            format = self.packettypecache[fmt.type_ as usize].format.clone();
                            size = fmt.length as usize;
                        }
                    }
                }
                // didnt find a match, return unknown packet type
                if size == 0 {
                    return Ok(None);
                }
                if size < 3 {
                    return Err(Exception::Overflow("Arithmetic operation resulted in an overflow.".into()));
                }
                // size - 3 = message - messagetype - (header *2)
                let mut data = vec![0u8; size - 3];
                stream_read(br, &mut data);
                self.ProcessMessageObjects(&data, &name, &format).map(Some)
            }
        }
    }

    fn ProcessMessageObjects(&self, message: &[u8], name: &str, format: &str) -> Result<Vec<LogObject>, Exception> {
        let mut offset = 0usize;
        let mut answer = Vec::with_capacity(format.len() + 1);
        answer.push(LogObject::String(name.to_string()));
        for ch in format.chars() {
            let (item, size) = self.GetObjectFromMessage(ch, message, offset)?;
            answer.push(item);
            offset += size;
        }
        Ok(answer)
    }

    /// `GetObjectFromMessage(type, message, offset)`: the decoded value and its size.
    pub fn GetObjectFromMessage(&self, type_: char, message: &[u8], offset: usize) -> Result<(LogObject, usize), Exception> {
        let o = offset as i32;
        let byte_at = |i: usize| message.get(i).copied().ok_or_else(crate::dotnet::index_out_of_range);
        Ok(match type_ {
            'b' => (LogObject::SByte(byte_at(offset)? as i8), 1),
            'B' => (LogObject::Byte(byte_at(offset)?), 1),
            'h' => (LogObject::Int16(bit_converter::to_i16(message, o)?), 2),
            'H' => (LogObject::UInt16(bit_converter::to_u16(message, o)?), 2),
            'i' => (LogObject::Int32(bit_converter::to_i32(message, o)?), 4),
            'I' => (LogObject::UInt32(bit_converter::to_u32(message, o)?), 4),
            'q' => (LogObject::Int64(to_i64(message, o)?), 8),
            'Q' => (LogObject::UInt64(bit_converter::to_u64(message, o)?), 8),
            'g' => (LogObject::Single(HalfHelper::halfushort_to_single(bit_converter::to_u16(message, o)?)), 2),
            'f' => (LogObject::Single(bit_converter::to_single(message, o)?), 4),
            'd' => (LogObject::Double(bit_converter::to_double(message, o)?), 8),
            'c' => (LogObject::Double(bit_converter::to_i16(message, o)? as f64 / 100.0), 2),
            'C' => (LogObject::Double(bit_converter::to_u16(message, o)? as f64 / 100.0), 2),
            'e' => (LogObject::Double(bit_converter::to_i32(message, o)? as f64 / 100.0), 4),
            'E' => (LogObject::Double(bit_converter::to_u32(message, o)? as f64 / 100.0), 4),
            'L' => (LogObject::Double(bit_converter::to_i32(message, o)? as f64 / 10000000.0), 4),
            'n' => (LogObject::String(ascii_get_string_range(message, offset, 4)?.trim_matches('\0').to_string()), 4),
            'N' => (LogObject::String(ascii_get_string_range(message, offset, 16)?.trim_matches('\0').to_string()), 16),
            'M' => {
                let modeno = byte_at(offset)? as i32;
                // onFlightMode?.Invoke(...): null with no handlers, else the last handler's value
                let handlers = ON_FLIGHT_MODE.read().unwrap().clone();
                let mode = handlers.iter().map(|h| h(&self._firmware, modeno)).last().flatten().unwrap_or_else(|| modeno.to_string());
                (LogObject::String(mode), 1)
            }
            'Z' => (LogObject::Bytes(span_to_array(message, offset, 64)?), 64),
            'a' => (LogObject::UnionArray(UnionArray::new(span_to_array(message, offset, 64)?)), 2 * 32),
            _ => (LogObject::Null, 0),
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// One FMT for FMT itself, one FMT for "TST" (QBZ) and a TST message.
    pub(crate) fn sample() -> Vec<u8> {
        let mut out = Vec::new();
        let fmt = |out: &mut Vec<u8>, t: u8, len: u8, name: &str, format: &str, labels: &str| {
            out.extend_from_slice(&[0xA3, 0x95, 0x80, t, len]);
            let mut n = [0u8; 4];
            n[..name.len()].copy_from_slice(name.as_bytes());
            out.extend_from_slice(&n);
            let mut f = [0u8; 16];
            f[..format.len()].copy_from_slice(format.as_bytes());
            out.extend_from_slice(&f);
            let mut l = [0u8; 64];
            l[..labels.len()].copy_from_slice(labels.as_bytes());
            out.extend_from_slice(&l);
        };
        fmt(&mut out, 0x80, 89, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns");
        fmt(&mut out, 0x81, 3 + 8 + 1 + 64, "TST", "QBZ", "TimeUS,V,Msg");
        out.extend_from_slice(&[0xA3, 0x95, 0x81]);
        out.extend_from_slice(&1_000_000u64.to_le_bytes());
        out.push(7);
        let mut z = [0u8; 64];
        z[..2].copy_from_slice(b"hi");
        out.extend_from_slice(&z);
        out
    }

    #[test]
    fn reads_lines_and_objects() {
        let data = sample();
        let mut s = std::io::Cursor::new(data.clone());
        let mut bl = BinaryLog::new();
        let len = data.len() as i64;
        assert_eq!(bl.ReadMessage(&mut s, len), "FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\r\n");
        assert_eq!(bl.ReadMessage(&mut s, len), "FMT, 129, 76, TST, QBZ, TimeUS,V,Msg\r\n");
        assert_eq!(bl.ReadMessage(&mut s, len), "TST, 1000000, 7, hi\r\n");
        assert_eq!(bl.ReadMessage(&mut s, len), "");

        let mut s = std::io::Cursor::new(data.clone());
        let mut bl = BinaryLog::new();
        assert_eq!(bl.ReadMessageTypeOffset(&mut s, len), (0x80, 0));
        assert_eq!(bl.ReadMessageTypeOffset(&mut s, len), (0x80, 89));
        assert_eq!(bl.ReadMessageTypeOffset(&mut s, len), (0x81, 178));
        assert_eq!(bl.ReadMessageTypeOffset(&mut s, len), (0, 0));
        s.set_position(178);
        let o = bl.ReadMessageObjects(&mut s, len).unwrap();
        assert_eq!(o[0], LogObject::String("TST".into()));
        assert_eq!(o[1], LogObject::UInt64(1_000_000));
    }

    #[test]
    fn empty_z_string_drops_the_line() {
        // Aggregate over an empty string throws, so the whole message is skipped
        let mut data = sample();
        let n = data.len();
        for b in &mut data[n - 64..] {
            *b = 0;
        }
        let mut s = std::io::Cursor::new(data.clone());
        let mut bl = BinaryLog::new();
        let len = data.len() as i64;
        bl.ReadMessage(&mut s, len);
        bl.ReadMessage(&mut s, len);
        assert_eq!(bl.ReadMessage(&mut s, len), "");
    }
}
