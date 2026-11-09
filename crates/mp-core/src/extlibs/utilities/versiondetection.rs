//! Port of `ExtLibs/Utilities/VersionDetection.cs`
//!
//! Firmware version text ("ArduCopter V4.3.0-rc2", "4.5dev") to a `System.Version`. The
//! repeated group keeps its last capture, as .NET's `Groups[n].Value` does. The letter branch
//! (group 7) reads group 6's first character, which throws when group 6 did not match.

#![allow(non_snake_case)]

use std::sync::OnceLock;

use regex::Regex;

use crate::dotnet::version::Version;
use crate::dotnet::Exception;

/// `public class VersionDetection`
pub struct VersionDetection;

fn versionregex() -> &'static Regex {
    static R: OnceLock<Regex> = OnceLock::new();
    R.get_or_init(|| Regex::new(r"([0-9]+)\.([0-9]+)(\.([0-9]+)|-rc([0-9]+)|([a-z]{2,20})|([a-z]))*").expect("valid pattern"))
}

impl VersionDetection {
    /// `GetVersion(string input)`
    pub fn GetVersion(input: &str) -> Result<Version, Exception> {
        let versionregex = versionregex();

        let caps = versionregex.captures(input);

        log::info!("{input}");

        if let Some(m) = caps {
            let g = |i: usize| m.get(i).map(|c| c.as_str()).unwrap_or("");
            // start with major.monor
            let mut verstring = format!("{}.{}", g(1), g(2));

            if !g(4).is_empty() {
                verstring += &format!(".{}", g(4));
            }
            if !g(5).is_empty() {
                // -rc
                verstring += &format!(".{}", g(5));
            }
            if !g(6).is_empty() {
                // dev
                verstring += ".255";
            }
            if !g(7).is_empty() {
                // convert a to 1, b to 2, etc, it will break at j
                let first = g(6)
                    .chars()
                    .next()
                    .ok_or_else(|| Exception::IndexOutOfRange("Index was outside the bounds of the array.".into()))?;
                let lower = first.to_lowercase().next().unwrap_or(first);
                let c = char::from_u32((lower as u32).wrapping_sub(0x30) & 0xffff).unwrap_or('\u{fffd}');
                verstring += &format!(".{c}");
            }

            let version = Version::parse(&verstring)?;

            log::info!("{version}");

            return Ok(version);
        }

        Err(Exception::Other("Bad Version".into()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn firmware_strings() {
        let v = |s: &str| VersionDetection::GetVersion(s).map(|v| v.to_string());
        assert_eq!(v("ArduCopter V4.3.0").unwrap(), "4.3.0");
        assert_eq!(v("ArduPlane V4.5.0-rc2").unwrap(), "4.5.0.2");
        assert_eq!(v("APM:Copter V4.6dev").unwrap(), "4.6.255");
        assert!(v("no version here").is_err());
        // a single letter without a preceding word: Groups[6] is empty and indexing it throws
        assert!(v("4.0b").is_err());
    }
}
