//! Port of `ExtLibs/Utilities/sbp.cs`
//!
//! Swift Binary Protocol (Piksi) framing with its CRC-16-CCITT.

use std::sync::OnceLock;

use super::icorrections::ICorrections;

crate::cs_enum! {
    /// `public enum InitialCrcValue`
    pub InitialCrcValue: i32 {
        Zeros = 0,
        NonZero1 = 0xffff,
        NonZero2 = 0x1D0F,
    }
}

/// `public class piksimsg`
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, PartialEq)]
pub struct piksimsg {
    /// 0x55
    pub preamble: u8,
    pub msg_type: u16,
    pub sender: u16,
    /// payload length
    pub length: u8,
    /// `null` until the length byte arrives
    pub payload: Option<Vec<u8>>,
    /// - preamble
    pub crc: u16,
    pub buffer: Vec<u8>,
}

impl Default for piksimsg {
    fn default() -> Self {
        piksimsg { preamble: 0, msg_type: 0, sender: 0, length: 0, payload: None, crc: 0, buffer: vec![0u8; 4096] }
    }
}

/// `public class Crc16Ccitt`
#[derive(Debug, Clone, Copy)]
pub struct Crc16Ccitt {
    initial_value: u16,
}

const POLY: u16 = 4129;

/// `static ushort[] table`, built by the first constructor call.
fn table() -> &'static [u16; 256] {
    static T: OnceLock<[u16; 256]> = OnceLock::new();
    T.get_or_init(|| {
        let mut table = [0u16; 256];
        for (i, slot) in table.iter_mut().enumerate() {
            let mut temp: u16 = 0;
            let mut a: u16 = (i << 8) as u16;
            for _ in 0..8 {
                if ((temp ^ a) & 0x8000) != 0 {
                    temp = (temp << 1) ^ POLY;
                } else {
                    temp <<= 1;
                }
                a <<= 1;
            }
            *slot = temp;
        }
        table
    })
}

impl Crc16Ccitt {
    /// `new Crc16Ccitt(InitialCrcValue initialValue)`
    pub fn new(initial_value: InitialCrcValue) -> Self {
        let _ = table();
        Crc16Ccitt { initial_value: initial_value.0 as u16 }
    }

    /// `Accumulate(byte data, ushort crc)`
    #[allow(non_snake_case)]
    pub fn Accumulate(&self, data: u8, crc: u16) -> u16 {
        (crc << 8) ^ table()[((crc >> 8) ^ (0xff & data as u16)) as usize]
    }

    /// `ComputeChecksum(byte[] bytes)`
    #[allow(non_snake_case)]
    pub fn ComputeChecksum(&self, bytes: &[u8]) -> u16 {
        let mut crc = self.initial_value;
        for b in bytes {
            crc = (crc << 8) ^ table()[((crc >> 8) ^ (0xff & *b as u16)) as usize];
        }
        crc
    }

    /// `ComputeChecksumBytes(byte[] bytes)`: big endian.
    #[allow(non_snake_case)]
    pub fn ComputeChecksumBytes(&self, bytes: &[u8]) -> [u8; 2] {
        let crc = self.ComputeChecksum(bytes);
        [(crc >> 8) as u8, (crc & 0x00ff) as u8]
    }
}

/// `public class sbp : ICorrections`
#[allow(non_camel_case_types)]
#[derive(Debug, Clone)]
pub struct sbp {
    state: i32,
    msg: piksimsg,
    lengthcount: i32,
    crc: Option<Crc16Ccitt>,
    crcpacket: u16,
}

impl Default for sbp {
    fn default() -> Self {
        sbp { state: 0, msg: piksimsg::default(), lengthcount: 0, crc: None, crcpacket: 0 }
    }
}

impl sbp {
    pub fn new() -> Self {
        Self::default()
    }

    fn accumulate(&self, data: u8) -> u16 {
        // `crc` is set on the preamble, the only way into these states
        match &self.crc {
            Some(c) => c.Accumulate(data, self.crcpacket),
            None => self.crcpacket,
        }
    }

    /// `read(byte data)`: the message type when a frame with a valid CRC ends at this byte,
    /// otherwise -1.
    pub fn read(&mut self, data: u8) -> i32 {
        let mut state = self.state;
        loop {
            match state {
                1 => {
                    self.msg.msg_type = data as u16;
                    self.msg.buffer[1] = data;
                    self.crcpacket = self.accumulate(data);
                    self.state += 1;
                }
                2 => {
                    self.msg.msg_type = self.msg.msg_type.wrapping_add((data as u16) << 8);
                    self.msg.buffer[2] = data;
                    self.crcpacket = self.accumulate(data);
                    self.state += 1;
                }
                3 => {
                    self.msg.sender = data as u16;
                    self.msg.buffer[3] = data;
                    self.crcpacket = self.accumulate(data);
                    self.state += 1;
                }
                4 => {
                    self.msg.sender = self.msg.sender.wrapping_add((data as u16) << 8);
                    self.msg.buffer[4] = data;
                    self.crcpacket = self.accumulate(data);
                    self.state += 1;
                }
                5 => {
                    self.msg.length = data;
                    self.msg.buffer[5] = data;
                    self.crcpacket = self.accumulate(data);
                    self.msg.payload = Some(vec![0u8; self.msg.length as usize]);
                    self.msg.buffer.resize(8 + data as usize, 0);
                    self.lengthcount = 0;
                    self.state += 1;
                }
                6 => {
                    if self.lengthcount == self.msg.length as i32 {
                        self.state += 1;
                        state = 7;
                        continue; // goto case 7
                    } else {
                        let lc = self.lengthcount as usize;
                        if let Some(p) = self.msg.payload.as_mut() {
                            p[lc] = data;
                        }
                        self.msg.buffer[6 + lc] = data;
                        self.crcpacket = self.accumulate(data);
                        self.lengthcount += 1;
                    }
                }
                7 => {
                    self.msg.crc = data as u16;
                    self.msg.buffer[6 + self.lengthcount as usize] = data;
                    self.state += 1;
                }
                8 => {
                    self.msg.crc = self.msg.crc.wrapping_add((data as u16) << 8);
                    self.msg.buffer[7 + self.lengthcount as usize] = data;
                    self.state = 0;

                    if self.msg.crc == self.crcpacket {
                        return self.msg.msg_type as i32;
                    }
                }
                // default: and case 0:
                _ => {
                    if data == 0x55 {
                        self.state += 1;
                        self.msg = piksimsg::default();
                        self.msg.preamble = data;
                        self.msg.buffer[0] = data;
                        self.crc = Some(Crc16Ccitt::new(InitialCrcValue::Zeros));
                        self.crcpacket = InitialCrcValue::Zeros.0 as u16;
                    }
                }
            }
            break;
        }

        -1
    }
}

impl ICorrections for sbp {
    fn length(&self) -> i32 {
        self.msg.length as i32 + 8
    }

    fn packet(&self) -> &[u8] {
        &self.msg.buffer
    }

    fn resetParser(&mut self) -> bool {
        self.state = 0;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_with_valid_crc() {
        let body = [0x02u8, 0x01, 0x34, 0x12, 0x02, 0xAA, 0xBB];
        let crc = Crc16Ccitt::new(InitialCrcValue::Zeros).ComputeChecksum(&body);
        let mut f = vec![0x55];
        f.extend_from_slice(&body);
        f.push(crc as u8);
        f.push((crc >> 8) as u8);
        let mut p = sbp::new();
        let mut last = -1;
        for b in &f {
            last = p.read(*b);
        }
        assert_eq!(last, 0x0102);
        assert_eq!(p.msg.sender, 0x1234);
        assert_eq!(p.length(), 10);
        assert_eq!(p.packet().len(), 10);
        // the CCITT check value for "123456789" with a zero start is 0x31C3
        assert_eq!(Crc16Ccitt::new(InitialCrcValue::Zeros).ComputeChecksum(b"123456789"), 0x31C3);
    }
}
