//! `System.IO` text reading and writing as the ported code uses it: `StreamReader` (encoding
//! detection from the byte order mark, UTF-8 by default, `ReadLine` splitting on CR, LF and
//! CR LF), `StreamWriter` (UTF-8 without a BOM, `Environment.NewLine`), and the exceptions
//! `File.Open` throws.

use super::Exception;

/// `Environment.NewLine`: CR LF on Windows, LF elsewhere (Mono).
pub const NEW_LINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

/// `Console.WriteLine(s)`: the text and `Environment.NewLine`; a closed or broken stdout is
/// ignored, as .NET's console writer does.
pub fn console_write_line(s: &str) {
    use std::io::Write;
    let mut out = std::io::stdout().lock();
    let _ = out.write_all(s.as_bytes());
    let _ = out.write_all(NEW_LINE.as_bytes());
    let _ = out.flush();
}

/// `Console.Write(s)`
pub fn console_write(s: &str) {
    use std::io::Write;
    let mut out = std::io::stdout().lock();
    let _ = out.write_all(s.as_bytes());
    let _ = out.flush();
}

/// The exception `File.Open`/`new StreamReader(path)` throws for an I/O error on `path`.
pub fn file_error(path: &str, e: &std::io::Error) -> Exception {
    // .NET names the full path in these messages (Path.GetFullPath)
    let full = std::path::absolute(path).map(|p| p.to_string_lossy().into_owned()).unwrap_or_else(|_| path.to_string());
    let path = full.as_str();
    match e.kind() {
        std::io::ErrorKind::NotFound => {
            let p = std::path::Path::new(path);
            match p.parent() {
                Some(d) if !d.as_os_str().is_empty() && !d.exists() => {
                    Exception::DirectoryNotFound(format!("Could not find a part of the path '{path}'."))
                }
                _ => Exception::FileNotFound(format!("Could not find file '{path}'.")),
            }
        }
        std::io::ErrorKind::PermissionDenied => Exception::UnauthorizedAccess(format!("Access to the path '{path}' is denied.")),
        _ => Exception::Io(e.to_string()),
    }
}

/// `StreamReader.ReadToEnd()` on a whole byte buffer: a UTF-8, UTF-16 LE/BE or UTF-32 LE BOM
/// selects the encoding and is dropped; otherwise UTF-8. Invalid sequences become U+FFFD.
pub fn decode_detect(bytes: &[u8]) -> String {
    if bytes.starts_with(&[0xEF, 0xBB, 0xBF]) {
        return String::from_utf8_lossy(&bytes[3..]).into_owned();
    }
    if bytes.starts_with(&[0x00, 0x00, 0xFE, 0xFF]) {
        return bytes[4..]
            .chunks(4)
            .map(|c| {
                let mut a = [0u8; 4];
                a[..c.len()].copy_from_slice(c);
                char::from_u32(u32::from_be_bytes(a)).unwrap_or('\u{FFFD}')
            })
            .collect();
    }
    if bytes.starts_with(&[0xFF, 0xFE, 0x00, 0x00]) {
        return bytes[4..]
            .chunks(4)
            .map(|c| {
                let mut a = [0u8; 4];
                a[..c.len()].copy_from_slice(c);
                char::from_u32(u32::from_le_bytes(a)).unwrap_or('\u{FFFD}')
            })
            .collect();
    }
    if bytes.starts_with(&[0xFF, 0xFE]) {
        let u: Vec<u16> = bytes[2..].chunks(2).map(|c| u16::from_le_bytes([c[0], *c.get(1).unwrap_or(&0)])).collect();
        return String::from_utf16_lossy(&u);
    }
    if bytes.starts_with(&[0xFE, 0xFF]) {
        let u: Vec<u16> = bytes[2..].chunks(2).map(|c| u16::from_be_bytes([c[0], *c.get(1).unwrap_or(&0)])).collect();
        return String::from_utf16_lossy(&u);
    }
    String::from_utf8_lossy(bytes).into_owned()
}

/// `new StreamReader(stream, Encoding.ASCII)` over a .NET stream: reads in 1 KB chunks
/// (bytes above 0x7F decode to '?'), and `ReadLine` splits on CR, LF or CR LF, returning
/// null at the end of the stream. Stream exceptions (timeouts) propagate.
pub struct StreamReaderAscii<'a> {
    stream: &'a mut dyn super::Stream,
    buf: Vec<u8>,
    pos: usize,
    len: usize,
}

impl<'a> StreamReaderAscii<'a> {
    pub fn new(stream: &'a mut dyn super::Stream) -> Self {
        StreamReaderAscii { stream, buf: vec![0u8; 1024], pos: 0, len: 0 }
    }

    /// false at the end of the stream
    fn fill(&mut self) -> Result<bool, Exception> {
        if self.pos < self.len {
            return Ok(true);
        }
        self.pos = 0;
        self.len = self.stream.read(&mut self.buf)?;
        Ok(self.len > 0)
    }

    /// `ReadLine()`
    pub fn read_line(&mut self) -> Result<Option<String>, Exception> {
        if !self.fill()? {
            return Ok(None);
        }
        let mut line = String::new();
        loop {
            if !self.fill()? {
                return Ok(Some(line));
            }
            let b = self.buf[self.pos];
            self.pos += 1;
            match b {
                b'\n' => return Ok(Some(line)),
                b'\r' => {
                    if self.fill()? && self.buf[self.pos] == b'\n' {
                        self.pos += 1;
                    }
                    return Ok(Some(line));
                }
                b if b < 0x80 => line.push(b as char),
                _ => line.push('?'),
            }
        }
    }
}

/// `new StreamReader(path).ReadToEnd()`
pub fn read_all_text(path: &str) -> Result<String, Exception> {
    let bytes = std::fs::read(path).map_err(|e| file_error(path, &e))?;
    Ok(decode_detect(&bytes))
}

/// The lines `while (!sr.EndOfStream) sr.ReadLine()` yields: split on CR LF, CR or LF; a
/// terminator at the very end does not start another line.
pub fn split_lines(text: &str) -> Vec<String> {
    let mut lines = vec![];
    let mut cur = String::new();
    let mut chars = text.chars().peekable();
    let mut pending = false;
    while let Some(c) = chars.next() {
        pending = true;
        match c {
            '\r' => {
                if chars.peek() == Some(&'\n') {
                    chars.next();
                }
                lines.push(std::mem::take(&mut cur));
                pending = false;
            }
            '\n' => {
                lines.push(std::mem::take(&mut cur));
                pending = false;
            }
            _ => cur.push(c),
        }
    }
    if pending {
        lines.push(cur);
    }
    lines
}

/// `new StreamReader(path)` read line by line to the end.
pub fn read_lines(path: &str) -> Result<Vec<String>, Exception> {
    Ok(split_lines(&read_all_text(path)?))
}

/// `new StreamWriter(File.Open(path, FileMode.Create))`: UTF-8 without a BOM, buffered.
/// Dropping it flushes, as the `using` block's `Dispose` does, so text written before an
/// exception still reaches the file.
#[derive(Debug)]
pub struct StreamWriter {
    path: String,
    out: Option<std::io::BufWriter<std::fs::File>>,
}

impl StreamWriter {
    /// Opens (creates or truncates) the file, failing as `File.Open` does.
    pub fn create(path: &str) -> Result<Self, Exception> {
        let f = std::fs::File::create(path).map_err(|e| file_error(path, &e))?;
        Ok(StreamWriter { path: path.to_string(), out: Some(std::io::BufWriter::with_capacity(1024, f)) })
    }

    fn put(&mut self, s: &str) -> Result<(), Exception> {
        use std::io::Write;
        let path = self.path.clone();
        match self.out.as_mut() {
            Some(w) => w.write_all(s.as_bytes()).map_err(|e| file_error(&path, &e)),
            None => Err(Exception::ObjectDisposed("Cannot write to a closed TextWriter.".into())),
        }
    }

    /// `WriteLine(s)`
    pub fn write_line(&mut self, s: &str) -> Result<(), Exception> {
        self.put(s)?;
        self.put(NEW_LINE)
    }

    /// `Write(s)`
    pub fn write(&mut self, s: &str) -> Result<(), Exception> {
        self.put(s)
    }

    /// `Dispose()`: flushes and closes the file.
    pub fn close(mut self) -> Result<(), Exception> {
        use std::io::Write;
        match self.out.take() {
            Some(mut w) => w.flush().map_err(|e| file_error(&self.path, &e)),
            None => Ok(()),
        }
    }
}

impl Drop for StreamWriter {
    fn drop(&mut self) {
        use std::io::Write;
        if let Some(w) = self.out.as_mut() {
            let _ = w.flush();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn readline_semantics() {
        assert_eq!(split_lines("a\r\nb\rc\nd"), vec!["a", "b", "c", "d"]);
        assert_eq!(split_lines("a\n"), vec!["a"]);
        assert_eq!(split_lines("a\n\n"), vec!["a", ""]);
        assert!(split_lines("").is_empty());
        assert_eq!(decode_detect(&[0xEF, 0xBB, 0xBF, b'x']), "x");
    }
}
