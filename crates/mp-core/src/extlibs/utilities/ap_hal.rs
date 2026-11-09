//! Port of `ExtLibs/Utilities/AP_HAL.cs`

#![allow(non_camel_case_types)]

/// `internal class AP_HAL`
pub struct AP_HAL;

impl AP_HAL {
    /// `millis()`: `(uint)(DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds`;
    /// the x64 conversion keeps the low 32 bits of the truncated value.
    pub fn millis() -> u32 {
        let ms = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64() * 1000.0).unwrap_or(0.0);
        (ms as i64) as u32
    }
}
