//! Port of `ExtLibs/Utilities/nmea.cs`

use super::icorrections::ICorrections;
use crate::dotnet::ascii_get_string;

/// `public class nmea : ICorrections`: frames NMEA sentences starting "$G" and checks their
/// checksum.
#[allow(non_camel_case_types)]
#[derive(Debug, Clone)]
pub struct nmea {
    step: i32,
    pub buffer: Vec<u8>,
    payloadlen: i32,
    msglencount: i32,
}

impl Default for nmea {
    fn default() -> Self {
        nmea { step: 0, buffer: vec![0u8; 1024], payloadlen: 0, msglencount: 0 }
    }
}

impl nmea {
    pub fn new() -> Self {
        Self::default()
    }

    /// `Read(byte data)`: 1 when a full sentence with a valid checksum ends at this byte,
    /// otherwise -1.
    #[allow(non_snake_case)]
    pub fn Read(&mut self, data: u8) -> i32 {
        match self.step {
            1 => {
                if data == b'G' {
                    self.buffer[1] = data;
                    self.step += 1;
                } else {
                    self.step = 0;
                }
            }
            2 => {
                if self.msglencount > 1000 {
                    self.step = 0;
                }

                self.buffer[(self.msglencount + 2) as usize] = data;
                self.msglencount += 1;
                if data == b'\n' {
                    let line = ascii_get_string(&self.buffer[..(self.msglencount + 2) as usize]);
                    let items: Vec<&str> = line.trim().split([',', '*']).collect();
                    if items[items.len() - 1] == Self::get_checksum(&line) {
                        return 1;
                    }
                    self.step = 0;
                }
            }
            // default: and case 0:
            _ => {
                if data == b'$' {
                    self.step = 1;
                    self.msglencount = 0;
                    self.buffer[0] = data;
                }
            }
        }

        -1
    }

    /// `GetChecksum(string sentence)`: XOR of the characters between '$' and '*', as two
    /// upper-case hex digits.
    fn get_checksum(sentence: &str) -> String {
        // Loop through all chars to get a checksum
        let mut checksum: i32 = 0;
        for character in sentence.chars() {
            match character {
                // Ignore the dollar sign
                '$' => {}
                // Stop processing before the asterisk
                '*' => return format!("{checksum:02X}"),
                _ => {
                    // the decoded text is ASCII, so Convert.ToByte(char) cannot overflow
                    if checksum == 0 {
                        checksum = character as i32;
                    } else {
                        checksum ^= character as i32;
                    }
                }
            }
        }
        // Return the checksum formatted as a two-character hexadecimal
        format!("{checksum:02X}")
    }
}

impl ICorrections for nmea {
    /// header2, class, subclass, length2, data, crc2: always 8 (`payloadlen` is never set).
    fn length(&self) -> i32 {
        2 + 2 + 2 + 2 + self.payloadlen
    }

    fn packet(&self) -> &[u8] {
        &self.buffer
    }

    fn resetParser(&mut self) -> bool {
        self.step = 0;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_valid_sentence() {
        let s = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        let mut p = nmea::new();
        let mut got = -1;
        for b in s {
            got = p.Read(*b);
        }
        assert_eq!(got, 1);
        let mut p = nmea::new();
        let bad = b"$GPGGA,1*00\r\n";
        assert!(bad.iter().all(|b| p.Read(*b) == -1));
        assert_eq!(p.length(), 8);
    }
}
