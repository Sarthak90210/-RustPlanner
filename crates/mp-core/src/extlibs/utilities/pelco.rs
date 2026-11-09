//! Port of `ExtLibs/Utilities/Pelco.cs`
//!
//! Pelco-D / Pelco-P camera control packets. `GenerateCommand`'s checksum adds the 0xFF sync
//! byte too (the Pelco-D spec leaves it out), truncated to a byte, as the C# does.

#![allow(non_snake_case, non_camel_case_types)]

use crate::dotnet::Exception;

crate::cs_enum! {
    /// `[Flags] public enum cmd1 : byte`
    pub cmd1: u8 {
        None = 0,
        FocusNear = 1,
        IrisOpen = 2,
        IrisClose = 4,
        CameraOnOrOff = 8,
        AutoOrManualScan = 16,
        Reserved1 = 32,
        Reserved2 = 64,
        Sense = 128,
    }
}

crate::cs_enum! {
    /// `[Flags] public enum cmd2 : byte`
    pub cmd2: u8 {
        None = 0,
        FixedTo0 = 1,
        PanRight = 2,
        PanLeft = 4,
        TiltUp = 8,
        TiltDown = 16,
        ZoomTele = 32,
        ZoomWide = 64,
        FocusFar = 128,
    }
}

/// `public struct D`: a Pelco-D packet (sync 0xFF, address, command 1/2, data 1/2, checksum).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct D {
    pub Sync: u8,
    pub CameraAddress: u8,
    pub Command1: cmd1,
    pub Command2: cmd2,
    pub Data1: u8,
    pub Data2: u8,
    pub CheckSum: u8,
}

/// `public struct P`: a Pelco-P packet (STX 0xA0, address 0x00-0x1F, data 1-4, ETX 0xAF,
/// checksum).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct P {
    pub STX: u8,
    pub CameraAddress: u8,
    pub Data1: u8,
    pub Data2: u8,
    pub Data3: u8,
    pub Data4: u8,
    pub ETX: u8,
    pub CheckSum: u8,
}

/// `public class Pelco`
#[derive(Debug, Clone, Copy, Default)]
pub struct Pelco;

impl Pelco {
    pub fn new() -> Self {
        Pelco
    }

    /// `GenerateCommand(cameraaddr, cmd1, cmd2, data1, data2)`: a Pelco-D packet; the
    /// checksum is `(byte)(0xff + cameraaddr + cmd1 + cmd2 + data1 + data2)`.
    pub fn GenerateCommand(&self, cameraaddr: u8, command1: cmd1, command2: cmd2, data1: u8, data2: u8) -> Vec<u8> {
        let sum = 0xff_i32 + cameraaddr as i32 + command1.0 as i32 + command2.0 as i32 + data1 as i32 + data2 as i32;
        vec![0xff, cameraaddr, command1.0, command2.0, data1, data2, sum as u8]
    }

    /// `CheckSum(byte[] input)` (private in C#): the sum of everything between the sync byte
    /// and the checksum, mod 256, against the last byte. An empty array throws (`Last()`).
    #[allow(dead_code)]
    fn CheckSum(&self, input: &[u8]) -> Result<bool, Exception> {
        let cscount = input.len() as i64 - 2; // Sync and CS
        let currentcs = *input.last().ok_or_else(|| Exception::InvalidOperation("Sequence contains no elements".into()))?;
        let take = cscount.max(0) as usize;
        let sum: i32 = input.iter().skip(1).take(take).map(|a| *a as i32).sum();
        let inputcs = sum % 256;
        Ok(inputcs == currentcs as i32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checksum_includes_the_sync_byte() {
        let p = Pelco::new();
        // "Pan Left at high speed: FF 01 00 04 3F 00 44" by the spec; the C# adds 0xFF too
        let pkt = p.GenerateCommand(1, cmd1::None, cmd2::PanLeft, 0x3f, 0);
        assert_eq!(pkt, vec![0xff, 0x01, 0x00, 0x04, 0x3f, 0x00, 0x43]);
        // CheckSum leaves the sync byte out, so it accepts the spec packet
        assert!(p.CheckSum(&[0xff, 0x01, 0x00, 0x04, 0x3f, 0x00, 0x44]).unwrap());
        assert!(!p.CheckSum(&pkt).unwrap());
        assert!(p.CheckSum(&[]).is_err());
    }
}
