//! Port of `ExtLibs/Utilities/Half/HalfHelper.cs`
//!
//! Table-driven half/float conversion (Jeroen van der Zijp, "Fast Half Float Conversions").
//! Single to half truncates the mantissa (no rounding), as the C# does.

use std::sync::OnceLock;

use super::half::Half;

struct Tables {
    mantissa: Vec<u32>,
    exponent: Vec<u32>,
    offset: Vec<u16>,
    base: Vec<u16>,
    shift: Vec<i8>,
}

fn tables() -> &'static Tables {
    static T: OnceLock<Tables> = OnceLock::new();
    T.get_or_init(|| Tables {
        mantissa: generate_mantissa_table(),
        exponent: generate_exponent_table(),
        offset: generate_offset_table(),
        base: generate_base_table(),
        shift: generate_shift_table(),
    })
}

/// Transforms the subnormal representation to a normalized one.
fn convert_mantissa(i: i32) -> u32 {
    let mut m = (i << 13) as u32; // Zero pad mantissa bits
    let mut e: u32 = 0; // Zero exponent

    // While not normalized
    while (m & 0x0080_0000) == 0 {
        e = e.wrapping_sub(0x0080_0000); // Decrement exponent (1<<23)
        m <<= 1; // Shift mantissa
    }
    m &= !0x0080_0000u32; // Clear leading 1 bit
    e = e.wrapping_add(0x3880_0000); // Adjust bias ((127-14)<<23)
    m | e // Return combined number
}

fn generate_mantissa_table() -> Vec<u32> {
    let mut t = vec![0u32; 2048];
    t[0] = 0;
    for i in 1..1024 {
        t[i as usize] = convert_mantissa(i);
    }
    for i in 1024..2048 {
        t[i as usize] = (0x3800_0000 + ((i - 1024) << 13)) as u32;
    }
    t
}

fn generate_exponent_table() -> Vec<u32> {
    let mut t = vec![0u32; 64];
    t[0] = 0;
    for i in 1..31 {
        t[i as usize] = (i << 23) as u32;
    }
    t[31] = 0x4780_0000;
    t[32] = 0x8000_0000;
    for i in 33..63u32 {
        t[i as usize] = 0x8000_0000u32.wrapping_add((i - 32) << 23);
    }
    t[63] = 0xc780_0000;
    t
}

fn generate_offset_table() -> Vec<u16> {
    let mut t = vec![0u16; 64];
    t[0] = 0;
    for v in t.iter_mut().take(32).skip(1) {
        *v = 1024;
    }
    t[32] = 0;
    for v in t.iter_mut().skip(33) {
        *v = 1024;
    }
    t
}

fn generate_base_table() -> Vec<u16> {
    let mut t = vec![0u16; 512];
    for i in 0..256usize {
        let e = (127 - i as i32) as i8;
        let e32 = e as i32;
        if e > 24 {
            // Very small numbers map to zero
            t[i] = 0x0000;
            t[i | 0x100] = 0x8000;
        } else if e > 14 {
            // Small numbers map to denorms. The C# shifts by 18 + e (33..42); an int shift
            // uses only the low five bits of the count, which gives e - 14.
            t[i] = 0x0400i32.wrapping_shr((18 + e32) as u32) as u16;
            t[i | 0x100] = (0x0400i32.wrapping_shr((18 + e32) as u32) | 0x8000) as u16;
        } else if e >= -15 {
            // Normal numbers just lose precision
            t[i] = ((15 - e32) << 10) as u16;
            t[i | 0x100] = (((15 - e32) << 10) | 0x8000) as u16;
        } else if e > -128 {
            // Large numbers map to Infinity
            t[i] = 0x7c00;
            t[i | 0x100] = 0xfc00;
        } else {
            // Infinity and NaN's stay Infinity and NaN's
            t[i] = 0x7c00;
            t[i | 0x100] = 0xfc00;
        }
    }
    t
}

fn generate_shift_table() -> Vec<i8> {
    let mut t = vec![0i8; 512];
    for i in 0..256usize {
        let e = (127 - i as i32) as i8;
        if e > 24 {
            // Very small numbers map to zero
            t[i] = 24;
            t[i | 0x100] = 24;
        } else if e > 14 {
            // Small numbers map to denorms
            t[i] = e - 1;
            t[i | 0x100] = e - 1;
        } else if e >= -15 {
            // Normal numbers just lose precision
            t[i] = 13;
            t[i | 0x100] = 13;
        } else if e > -128 {
            // Large numbers map to Infinity
            t[i] = 24;
            t[i | 0x100] = 24;
        } else {
            // Infinity and NaN's stay Infinity and NaN's
            t[i] = 13;
            t[i | 0x100] = 13;
        }
    }
    t
}

/// `internal static class HalfHelper`
pub struct HalfHelper;

impl HalfHelper {
    /// `HalfushortToSingle(ushort value)`
    pub fn halfushort_to_single(value: u16) -> f32 {
        let t = tables();
        let result = t.mantissa[t.offset[(value >> 10) as usize] as usize + (value & 0x3ff) as usize]
            .wrapping_add(t.exponent[(value >> 10) as usize]);
        f32::from_bits(result)
    }

    /// `HalfToSingle(Half half)`
    pub fn half_to_single(half: Half) -> f32 {
        Self::halfushort_to_single(half.value)
    }

    /// `SingleToHalf(float single)`
    pub fn single_to_half(single: f32) -> Half {
        let t = tables();
        let value = single.to_bits();

        let shift = t.shift[(value >> 23) as usize] as i32;
        let result = (t.base[((value >> 23) & 0x1ff) as usize] as u32).wrapping_add((value & 0x007f_ffff).wrapping_shr(shift as u32)) as u16;
        Half::to_half(result)
    }

    /// `Negate(Half half)`
    pub fn negate(half: Half) -> Half {
        Half::to_half(half.value ^ 0x8000)
    }

    /// `Abs(Half half)`
    pub fn abs(half: Half) -> Half {
        Half::to_half(half.value & 0x7fff)
    }

    /// `IsNaN(Half half)`
    pub fn is_nan(half: Half) -> bool {
        (half.value & 0x7fff) > 0x7c00
    }

    /// `IsInfinity(Half half)`
    pub fn is_infinity(half: Half) -> bool {
        (half.value & 0x7fff) == 0x7c00
    }

    /// `IsPositiveInfinity(Half half)`
    pub fn is_positive_infinity(half: Half) -> bool {
        half.value == 0x7c00
    }

    /// `IsNegativeInfinity(Half half)`
    pub fn is_negative_infinity(half: Half) -> bool {
        half.value == 0xfc00
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversions() {
        assert_eq!(HalfHelper::single_to_half(1.0).value, 0x3c00);
        assert_eq!(HalfHelper::single_to_half(-2.0).value, 0xc000);
        assert_eq!(HalfHelper::single_to_half(65504.0).value, 0x7bff);
        assert_eq!(HalfHelper::single_to_half(1e6).value, 0x7c00);
        // smallest subnormal half: 2^-24
        assert_eq!(HalfHelper::single_to_half(5.960_464_5e-8).value, 0x0001);
        assert_eq!(HalfHelper::halfushort_to_single(0x3c00), 1.0);
        assert_eq!(HalfHelper::halfushort_to_single(0x0001), 5.960_464_5e-8);
        // truncation, not rounding: 1 + 2^-11 (exactly halfway) stays 1
        assert_eq!(HalfHelper::single_to_half(1.000_488_3).value, 0x3c00);
    }
}
