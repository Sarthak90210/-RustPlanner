//! Port of `ExtLibs/Core/Utils/MiscUtils.cs`
//!
//! `ParseEnum<T>` takes the enum's `Enum.Parse` (the `cs_enum!` `parse` function), since Rust
//! has no `typeof(T)` to look the members up from.

use crate::dotnet::Exception;

/// `public class MiscUtils`
pub struct MiscUtils;

impl MiscUtils {
    /// `ParseEnum<T>(string name)`: `(T)Enum.Parse(typeof(T), name)` with its exceptions.
    pub fn parse_enum<T>(name: Option<&str>, parse: fn(&str) -> Option<T>) -> Result<T, Exception> {
        let Some(name) = name else {
            return Err(Exception::ArgumentNull("Value cannot be null.\r\nParameter name: value".into()));
        };
        if name.trim().is_empty() {
            return Err(Exception::Argument("Must specify valid information for parsing in the string.".into()));
        }
        parse(name).ok_or_else(|| Exception::Argument(format!("Requested value '{name}' was not found.")))
    }

    /// `Swap<T>(ref T a, ref T b)`
    pub fn swap<T>(a: &mut T, b: &mut T) {
        std::mem::swap(a, b);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    crate::cs_enum! { pub Fruit: i32 { Apple = 0, Pear = 1 } }

    #[test]
    fn parse_and_swap() {
        assert_eq!(MiscUtils::parse_enum(Some("Pear"), Fruit::parse).unwrap(), Fruit::Pear);
        assert_eq!(MiscUtils::parse_enum(Some("1"), Fruit::parse).unwrap(), Fruit::Pear);
        assert_eq!(
            MiscUtils::parse_enum(Some("Plum"), Fruit::parse).unwrap_err().message(),
            "Requested value 'Plum' was not found."
        );
        assert!(matches!(MiscUtils::parse_enum(None, Fruit::parse), Err(Exception::ArgumentNull(_))));
        assert!(matches!(MiscUtils::parse_enum(Some(" "), Fruit::parse), Err(Exception::Argument(_))));
        let (mut a, mut b) = (1, 2);
        MiscUtils::swap(&mut a, &mut b);
        assert_eq!((a, b), (2, 1));
    }
}
