//! `System.Guid`: parsing (`new Guid(string)`), `ToByteArray`, `ToString()` ("D" format),
//! `Equals` and `GetHashCode`, with the .NET Framework field layout.

use super::Exception;

/// `System.Guid`: `_a` (int), `_b`, `_c` (short) and the bytes `_d`..`_k`.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Default, PartialOrd, Ord)]
pub struct Guid {
    a: i32,
    b: i16,
    c: i16,
    d: [u8; 8],
}

fn bad_format() -> Exception {
    Exception::Format("Guid should contain 32 digits with 4 dashes (xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx).".into())
}

impl Guid {
    /// `Guid.Empty`
    pub const EMPTY: Guid = Guid { a: 0, b: 0, c: 0, d: [0; 8] };

    /// `new Guid(int a, short b, short c, byte[] d)`
    pub const fn from_parts(a: i32, b: i16, c: i16, d: [u8; 8]) -> Guid {
        Guid { a, b, c, d }
    }

    /// `new Guid(string g)`: the "D", "N", "B" and "P" forms (32 hex digits, optionally with
    /// the four dashes, optionally inside braces or parentheses), surrounding white space
    /// allowed. Anything else throws `FormatException`.
    pub fn parse(g: &str) -> Result<Guid, Exception> {
        let s = g.trim();
        let inner = if (s.starts_with('{') && s.ends_with('}')) || (s.starts_with('(') && s.ends_with(')')) {
            if s.len() < 2 {
                return Err(bad_format());
            }
            let i = &s[1..s.len() - 1];
            // the braced forms must carry the dashes
            if !i.contains('-') {
                return Err(bad_format());
            }
            i
        } else {
            s
        };
        let digits: String = if inner.contains('-') {
            let parts: Vec<&str> = inner.split('-').collect();
            let lens = [8usize, 4, 4, 4, 12];
            if parts.len() != 5 || parts.iter().zip(lens.iter()).any(|(p, l)| p.len() != *l) {
                return Err(bad_format());
            }
            parts.concat()
        } else {
            if inner.len() != 32 {
                return Err(Exception::Format("Guid should contain 32 digits with 4 dashes (xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx).".into()));
            }
            inner.to_string()
        };
        if !digits.bytes().all(|c| c.is_ascii_hexdigit()) {
            return Err(Exception::Format("Guid string should only contain hexadecimal characters.".into()));
        }
        let hex = |i: usize, n: usize| u64::from_str_radix(&digits[i..i + n], 16).unwrap_or(0);
        let mut d = [0u8; 8];
        for (k, b) in d.iter_mut().enumerate() {
            *b = hex(16 + 2 * k, 2) as u8;
        }
        Ok(Guid { a: hex(0, 8) as u32 as i32, b: hex(8, 4) as u16 as i16, c: hex(12, 4) as u16 as i16, d })
    }

    /// `ToByteArray()`: `_a`, `_b`, `_c` little endian, then `_d`..`_k`.
    pub fn to_byte_array(&self) -> [u8; 16] {
        let mut r = [0u8; 16];
        r[0..4].copy_from_slice(&self.a.to_le_bytes());
        r[4..6].copy_from_slice(&self.b.to_le_bytes());
        r[6..8].copy_from_slice(&self.c.to_le_bytes());
        r[8..16].copy_from_slice(&self.d);
        r
    }

    /// `GetHashCode()`: `_a ^ ((_b << 16) | (ushort)_c) ^ ((_f << 24) | _k)`
    pub fn get_hash_code(&self) -> i32 {
        let bc = ((self.b as i32) << 16) | (self.c as u16 as i32);
        let fk = ((self.d[2] as i32) << 24) | self.d[7] as i32;
        self.a ^ bc ^ fk
    }
}

impl std::fmt::Display for Guid {
    /// `ToString()`: the "D" format, lower case.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:08x}-{:04x}-{:04x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
            self.a as u32,
            self.b as u16,
            self.c as u16,
            self.d[0],
            self.d[1],
            self.d[2],
            self.d[3],
            self.d[4],
            self.d[5],
            self.d[6],
            self.d[7]
        )
    }
}

impl std::fmt::Debug for Guid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Display::fmt(self, f)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_bytes_and_text() {
        let g = Guid::parse("0521335C-92EC-47A8-98A5-6FD333DDA9C0").unwrap();
        assert_eq!(g.to_string(), "0521335c-92ec-47a8-98a5-6fd333dda9c0");
        assert_eq!(&g.to_byte_array()[..8], &[0x5c, 0x33, 0x21, 0x05, 0xec, 0x92, 0xa8, 0x47]);
        assert_eq!(&g.to_byte_array()[8..], &[0x98, 0xa5, 0x6f, 0xd3, 0x33, 0xdd, 0xa9, 0xc0]);
        assert_eq!(Guid::parse("{0521335c-92ec-47a8-98a5-6fd333dda9c0}").unwrap(), g);
        assert_eq!(Guid::parse("0521335c92ec47a898a56fd333dda9c0").unwrap(), g);
        assert!(matches!(Guid::parse("0521335c-92ec-47a8-98a5"), Err(Exception::Format(_))));
        assert!(matches!(Guid::parse("0521335x-92ec-47a8-98a5-6fd333dda9c0"), Err(Exception::Format(_))));
        assert_eq!(Guid::EMPTY.to_string(), "00000000-0000-0000-0000-000000000000");
        assert_eq!(Guid::EMPTY.get_hash_code(), 0);
    }
}
