//! Port of `ExtLibs/Mavlink/MavlinkCRC.cs`

/// `MAVLink.MavlinkCRC`
pub struct MavlinkCRC;

impl MavlinkCRC {
    #[allow(dead_code)]
    const X25_INIT_CRC: u16 = 0xffff;
    #[allow(dead_code)]
    const X25_VALIDATE_CRC: u16 = 0xf0b8;

    pub fn crc_accumulate(b: u8, crc: u16) -> u16 {
        let mut ch = b ^ (crc & 0x00ff) as u8;
        ch ^= ch << 4;
        let ch = ch as u16;
        (crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4)
    }

    /// CRC over `p_buffer[1..length]`: the STX byte is skipped. `length < 1` returns 0xffff.
    pub fn crc_calculate(p_buffer: &[u8], length: i32) -> u16 {
        if length < 1 {
            return 0xffff;
        }
        let mut crc_tmp = Self::X25_INIT_CRC;
        for i in 1..length as usize {
            crc_tmp = Self::crc_accumulate(p_buffer[i], crc_tmp);
        }
        crc_tmp
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn x25_check_value() {
        // CRC-16/MCRF4XX of "123456789" is 0x6F91; crc_calculate skips byte 0.
        let mut buf = vec![0u8];
        buf.extend_from_slice(b"123456789");
        assert_eq!(MavlinkCRC::crc_calculate(&buf, buf.len() as i32), 0x6F91);
        assert_eq!(MavlinkCRC::crc_calculate(&buf, 0), 0xffff);
    }
}
