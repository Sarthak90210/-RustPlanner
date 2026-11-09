//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/Extensions.cs`
//!
//! The `SerializationInfo` helpers over the `dotnet::serialization` shim. A C# null is `None`.

#![allow(non_snake_case)]

use std::any::Any;

use crate::dotnet::serialization::SerializationInfo;

/// `GetValue<T>(info, key) where T : class`: the value, or null if it is missing or of
/// another type.
pub fn GetValue<T: Any + Clone>(info: &SerializationInfo, key: &str) -> Option<T> {
    match info.get_value::<T>(key) {
        Ok(v) => v,
        Err(ex) => {
            log::debug!("Extensions.GetValue: {}", ex.message());
            None
        }
    }
}

/// `GetValue<T>(info, key, defaultValue) where T : class`: the value, or `defaultValue` if
/// it was null.
pub fn GetValueOrDefault<T: Any + Clone>(info: &SerializationInfo, key: &str, defaultValue: Option<T>) -> Option<T> {
    let deserialized_value = GetValue::<T>(info, key);
    if deserialized_value.is_some() {
        return deserialized_value;
    }
    defaultValue
}

/// `GetStruct<T>(info, key, T defaultValue) where T : struct`: a missing member, one of
/// another type, or a stored null (unboxing null throws) all give `defaultValue`.
pub fn GetStruct<T: Any + Clone>(info: &SerializationInfo, key: &str, defaultValue: T) -> T {
    match info.get_value::<T>(key) {
        Ok(Some(v)) => v,
        Ok(None) => {
            log::debug!("Extensions.GetStruct: Object reference not set to an instance of an object.");
            defaultValue
        }
        Err(ex) => {
            log::debug!("Extensions.GetStruct: {}", ex.message());
            defaultValue
        }
    }
}

/// `GetStruct<T>(info, key, Nullable<T> defaultValue) where T : struct`: a stored null is
/// returned as null; only a failed lookup gives `defaultValue`.
pub fn GetStructNullable<T: Any + Clone>(info: &SerializationInfo, key: &str, defaultValue: Option<T>) -> Option<T> {
    match info.get_value::<T>(key) {
        Ok(v) => v,
        Err(ex) => {
            log::debug!("Extensions.GetStruct: {}", ex.message());
            defaultValue
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_and_nulls() {
        let mut i = SerializationInfo::new();
        i.add_value("name", Some(String::from("route"))).unwrap();
        i.add_value::<i32>("nil", None).unwrap();
        i.add_value("zoom", Some(7i32)).unwrap();
        assert_eq!(GetValue::<String>(&i, "name").as_deref(), Some("route"));
        assert_eq!(GetValue::<String>(&i, "missing"), None);
        assert_eq!(GetValueOrDefault(&i, "missing", Some(String::from("d"))).as_deref(), Some("d"));
        assert_eq!(GetStruct(&i, "zoom", 0i32), 7);
        assert_eq!(GetStruct(&i, "nil", 3i32), 3);
        assert_eq!(GetStruct(&i, "name", 3i32), 3);
        assert_eq!(GetStructNullable(&i, "nil", Some(3i32)), None);
        assert_eq!(GetStructNullable(&i, "missing", Some(3i32)), Some(3));
    }
}
