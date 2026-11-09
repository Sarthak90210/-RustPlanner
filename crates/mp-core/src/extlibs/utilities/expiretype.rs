//! Port of `ExtLibs/Utilities/ExpireType.cs`
//!
//! The static `ExpireType` keys its dictionaries by `object` (the C# callers pass label
//! controls, compared by reference); Rust callers pass a stable string id for the object.

use std::collections::HashMap;
use std::sync::Mutex;

use crate::dotnet::{DateTime, Exception};

/// `ExpireType<T>`: a value with the time it was set and when it expires.
#[derive(Debug, Clone)]
pub struct ExpireTypeValue<T> {
    #[allow(dead_code)]
    _set_at: DateTime,
    #[allow(dead_code)]
    _expire_at: DateTime,
    pub value: T,
}

impl<T> ExpireTypeValue<T> {
    /// `ExpireType(T input, double seconds, T expiredValue)`: `expiredValue` is not stored.
    pub fn new(input: T, seconds: f64, _expired_value: T) -> Result<Self, Exception> {
        let set_at = DateTime::now();
        Ok(ExpireTypeValue { _expire_at: set_at.add_milliseconds(seconds * 1000.0)?, _set_at: set_at, value: input })
    }
}

struct State {
    set_at: HashMap<String, DateTime>,
    expire_at: HashMap<String, DateTime>,
}

static STATE: Mutex<Option<State>> = Mutex::new(None);

/// `public static class ExpireType`
pub struct ExpireType;

impl ExpireType {
    fn with<R>(f: impl FnOnce(&mut State) -> R) -> R {
        let mut g = STATE.lock().unwrap_or_else(|e| e.into_inner());
        let s = g.get_or_insert_with(|| State { set_at: HashMap::new(), expire_at: HashMap::new() });
        f(s)
    }

    /// `Set(object input, double seconds)`
    pub fn set(input: &str, seconds: f64) -> Result<(), Exception> {
        let exp = DateTime::now().add_milliseconds(seconds * 1000.0)?;
        Self::with(|s| {
            s.set_at.insert(input.to_string(), DateTime::now());
            s.expire_at.insert(input.to_string(), exp);
        });
        Self::cleanup();
        Ok(())
    }

    /// `HasExpired(object check)`: true when never set or past its expiry.
    pub fn has_expired(check: &str) -> bool {
        Self::with(|s| match s.expire_at.get(check) {
            Some(t) => *t < DateTime::now(),
            None => true,
        })
    }

    /// `cleanup()`: the C# removes each expired `KeyValuePair` itself as the key, which is never
    /// in the dictionaries, so nothing is removed.
    fn cleanup() {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expiry() {
        assert!(ExpireType::has_expired("lbl_never"));
        ExpireType::set("lbl_a", 60.0).unwrap();
        assert!(!ExpireType::has_expired("lbl_a"));
        ExpireType::set("lbl_b", -1.0).unwrap();
        assert!(ExpireType::has_expired("lbl_b"));
        assert_eq!(ExpireTypeValue::new(5, 1.0, 0).unwrap().value, 5);
    }
}
