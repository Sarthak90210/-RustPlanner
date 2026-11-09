//! Port of `ExtLibs/Utilities/EnumTranslator.cs`
//!
//! The C# reflects over `typeof(T).GetFields(Static | Public)` and reads each field's
//! `DisplayTextAttribute` and `PrivateAttribute`. Rust has no reflection, so an enum that is
//! translated implements `EnumReflect`: its public static fields in metadata (declaration)
//! order, with their attributes, and `Enum.ToString()` for a value.

#![allow(non_snake_case)]

use super::attributes::displaytextattribute::DisplayTextAttribute;
use super::attributes::privateattribute::PrivateAttribute;
use crate::dotnet::collections::Dictionary;
use crate::dotnet::{culture_compare, Exception};

/// One `FieldInfo` of an enum type.
#[derive(Debug, Clone)]
pub struct EnumFieldInfo {
    /// `FieldInfo.Name`
    pub name: String,
    /// the member's value, as `Convert.ToInt64` would read it
    pub value: i64,
    /// `[DisplayText]`, when the member has one
    pub display_text: Option<DisplayTextAttribute>,
    /// `[Private]`, when the member has one
    pub private: Option<PrivateAttribute>,
}

/// What `EnumTranslator` reads through reflection.
pub trait EnumReflect {
    /// `typeof(T).GetFields(BindingFlags.Static | BindingFlags.GetField | BindingFlags.Public)`
    fn fields() -> Vec<EnumFieldInfo>;
    /// `((T)value).ToString()`: the member name .NET picks for the value.
    fn to_cs_string(value: i64) -> String;
}

fn duplicate_key() -> Exception {
    Exception::Argument("An item with the same key has already been added.".into())
}

/// `Dictionary.Add`: a duplicate key throws.
fn add(d: &mut Dictionary<i32, String>, k: i32, v: String) -> Result<(), Exception> {
    if d.contains_key(&k) {
        return Err(duplicate_key());
    }
    d.insert(k, v);
    Ok(())
}

/// `Convert.ToInt32(enum)`: values outside int throw.
fn to_int32(v: i64) -> Result<i32, Exception> {
    i32::try_from(v).map_err(|_| Exception::Overflow("Value was either too large or too small for an Int32.".into()))
}

/// `public static class EnumTranslator`
pub struct EnumTranslator;

impl EnumTranslator {
    /// `Translate<T>()`: private members hidden, sorted by text.
    pub fn Translate<T: EnumReflect>() -> Result<Dictionary<i32, String>, Exception> {
        Self::Translate_private::<T>(true)
    }

    /// `EnumToList<T>()`: `Translate<T>("", true, false)`, so `[Private(true)]` members are
    /// hidden; declaration order.
    pub fn EnumToList<T: EnumReflect>() -> Result<Vec<(i32, String)>, Exception> {
        let ans = Self::Translate_full::<T>("", true, false)?;
        Ok(ans.iter().map(|(k, v)| (*k, v.clone())).collect())
    }

    /// `Translate<T>(bool checkPrivate)`
    pub fn Translate_private<T: EnumReflect>(check_private: bool) -> Result<Dictionary<i32, String>, Exception> {
        Self::Translate_default::<T>("", check_private)
    }

    /// `Translate<T>(string defaultText, bool checkPrivate)`
    pub fn Translate_default<T: EnumReflect>(default_text: &str, check_private: bool) -> Result<Dictionary<i32, String>, Exception> {
        Self::Translate_full::<T>(default_text, check_private, true)
    }

    /// `Translate<T>(string defaultText, bool checkPrivate, bool sorting)`: -1 for the
    /// default text, then each member's display text (its name without `[DisplayText]`),
    /// skipping `[Private(true)]` members when `checkPrivate`, sorted by text (stable,
    /// current culture) when `sorting`.
    pub fn Translate_full<T: EnumReflect>(default_text: &str, check_private: bool, sorting: bool) -> Result<Dictionary<i32, String>, Exception> {
        let mut types = Dictionary::new();
        let mut temp_types = Dictionary::new();
        if !default_text.is_empty() {
            add(&mut types, -1, default_text.to_string())?;
        }
        for field_info in T::fields() {
            let mut add_it = true;
            let display_text = match &field_info.display_text {
                Some(a) => a.text(),
                None => T::to_cs_string(field_info.value),
            };
            if check_private {
                if let Some(p) = &field_info.private {
                    add_it = !p.is_private();
                }
            }
            if add_it {
                add(&mut temp_types, to_int32(field_info.value)?, display_text)?;
            }
        }
        let mut list: Vec<(i32, String)> = temp_types.iter().map(|(k, v)| (*k, v.clone())).collect();
        if sorting {
            // OrderBy(x => x.Value): a stable sort with the current culture's comparer
            list.sort_by(|a, b| culture_compare(&a.1, &b.1));
        }
        for (k, v) in list {
            add(&mut types, k, v)?;
        }
        Ok(types)
    }

    /// `GetDisplayText<T>(T value)`: the member's display text, or "" when the value is not
    /// a named member (`GetField(value.ToString())` is null).
    pub fn GetDisplayText<T: EnumReflect>(value: i64) -> String {
        let name = T::to_cs_string(value);
        // GetField(name): the field whose own name is that text (for aliases sharing a value,
        // the one ToString() picked)
        let fields = T::fields();
        let found = fields.iter().find(|f| f.name == name);
        match found {
            Some(f) => match &f.display_text {
                Some(a) => a.text(),
                None => T::to_cs_string(f.value),
            },
            None => String::new(),
        }
    }

    /// `GetValue<T>(string item)`: the key of the first entry of `Translate<T>()` whose text
    /// is `item`, else -1.
    pub fn GetValue<T: EnumReflect>(item: &str) -> Result<i32, Exception> {
        let list = Self::Translate::<T>()?;

        for (k, v) in list.iter() {
            if v == item {
                return Ok(*k);
            }
        }

        Ok(-1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Sample;

    impl EnumReflect for Sample {
        fn fields() -> Vec<EnumFieldInfo> {
            vec![
                EnumFieldInfo { name: "Relative".into(), value: 3, display_text: None, private: None },
                EnumFieldInfo { name: "Abs".into(), value: 0, display_text: Some(DisplayTextAttribute::new("Absolute").unwrap()), private: None },
                EnumFieldInfo { name: "Terrain".into(), value: 10, display_text: None, private: Some(PrivateAttribute::new(true)) },
            ]
        }
        fn to_cs_string(value: i64) -> String {
            match value {
                3 => "Relative".into(),
                0 => "Abs".into(),
                10 => "Terrain".into(),
                v => v.to_string(),
            }
        }
    }

    #[test]
    fn translate_orders_and_hides() {
        let list = EnumTranslator::EnumToList::<Sample>().unwrap();
        assert_eq!(list, vec![(3, "Relative".to_string()), (0, "Absolute".into())]);
        let t = EnumTranslator::Translate::<Sample>().unwrap();
        assert_eq!(t.iter().map(|(k, _)| *k).collect::<Vec<_>>(), vec![0, 3]);
        let d = EnumTranslator::Translate_default::<Sample>("None", false).unwrap();
        assert_eq!(d.iter().map(|(k, _)| *k).collect::<Vec<_>>(), vec![-1, 0, 3, 10]);
        assert_eq!(EnumTranslator::GetValue::<Sample>("Relative").unwrap(), 3);
        assert_eq!(EnumTranslator::GetValue::<Sample>("Terrain").unwrap(), -1);
        assert_eq!(EnumTranslator::GetDisplayText::<Sample>(0), "Absolute");
        assert_eq!(EnumTranslator::GetDisplayText::<Sample>(7), "");
    }
}
