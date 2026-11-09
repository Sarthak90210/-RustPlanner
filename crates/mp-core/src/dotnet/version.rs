//! `System.Version` (.NET Framework 4.7.2): major.minor[.build[.revision]], undefined parts
//! held as -1, compared component by component with -1 below 0.

use super::Exception;

/// `public sealed class Version`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Version {
    major: i32,
    minor: i32,
    build: i32,
    revision: i32,
}

impl Version {
    /// `new Version(int major, int minor)` (and the build/revision overloads): every part
    /// must be non-negative.
    pub fn new(major: i32, minor: i32, build: i32, revision: i32) -> Result<Version, Exception> {
        for (v, n) in [(major, "major"), (minor, "minor")] {
            if v < 0 {
                return Err(Exception::ArgumentOutOfRange(format!("Version's parameters must be greater than or equal to zero.\r\nParameter name: {n}")));
            }
        }
        Ok(Version { major, minor, build, revision })
    }

    /// `new Version(string version)` / `Version.Parse`: two to four '.'-separated parts, each
    /// an `int.Parse(part, NumberStyles.Integer, InvariantCulture)` that must be >= 0.
    pub fn parse(input: &str) -> Result<Version, Exception> {
        let parts: Vec<&str> = input.split('.').collect();
        if parts.len() < 2 || parts.len() > 4 {
            return Err(Exception::Argument("Version string portion was too short or too long.".into()));
        }
        let mut v = [-1i32; 4];
        for (i, p) in parts.iter().enumerate() {
            let n = parse_component(p)?;
            if n < 0 {
                return Err(Exception::ArgumentOutOfRange("Version's parameters must be greater than or equal to zero.\r\nParameter name: version".into()));
            }
            v[i] = n;
        }
        Ok(Version { major: v[0], minor: v[1], build: v[2], revision: v[3] })
    }

    pub fn major(&self) -> i32 {
        self.major
    }

    pub fn minor(&self) -> i32 {
        self.minor
    }

    /// `Build`: -1 when undefined.
    pub fn build(&self) -> i32 {
        self.build
    }

    /// `Revision`: -1 when undefined.
    pub fn revision(&self) -> i32 {
        self.revision
    }
}

/// `Int32.Parse(s, NumberStyles.Integer, CultureInfo.InvariantCulture)`: surrounding white
/// space and one leading sign.
fn parse_component(s: &str) -> Result<i32, Exception> {
    let t = s.trim_matches(|c: char| matches!(c, ' ' | '\t' | '\n' | '\u{b}' | '\u{c}' | '\r'));
    let (neg, digits) = match t.strip_prefix('-') {
        Some(d) => (true, d),
        None => (false, t.strip_prefix('+').unwrap_or(t)),
    };
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return Err(Exception::Format("Input string was not in a correct format.".into()));
    }
    let mut v: i64 = 0;
    for b in digits.bytes() {
        v = v * 10 + (b - b'0') as i64;
        if v > i32::MAX as i64 + 1 {
            return Err(Exception::Overflow("Value was either too large or too small for an Int32.".into()));
        }
    }
    let v = if neg { -v } else { v };
    i32::try_from(v).map_err(|_| Exception::Overflow("Value was either too large or too small for an Int32.".into()))
}

impl std::fmt::Display for Version {
    /// `ToString()`: the defined parts.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}", self.major, self.minor)?;
        if self.build >= 0 {
            write!(f, ".{}", self.build)?;
            if self.revision >= 0 {
                write!(f, ".{}", self.revision)?;
            }
        }
        Ok(())
    }
}

impl PartialOrd for Version {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Version {
    /// `CompareTo`: major, minor, build, revision in turn.
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        (self.major, self.minor, self.build, self.revision).cmp(&(other.major, other.minor, other.build, other.revision))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_and_print() {
        assert_eq!(Version::parse("4.0.255").unwrap().to_string(), "4.0.255");
        assert_eq!(Version::parse(" 1 . 2").unwrap().to_string(), "1.2");
        assert!(Version::parse("1").is_err());
        assert!(Version::parse("1.2.3.4.5").is_err());
        assert!(Version::parse("1.x").is_err());
        assert!(Version::parse("1.-2").is_err());
        assert!(Version::parse("4.1").unwrap() > Version::parse("4.0.9").unwrap());
        assert!(Version::parse("4.0").unwrap() < Version::parse("4.0.0").unwrap());
    }
}
