//! `System.Runtime.Serialization.SerializationInfo`, reduced to a name -> boxed value bag
//! with `AddValue` and `GetValue(name, type)`. A C# null value is `None`.

use std::any::Any;
use std::sync::Arc;

use super::Exception;

type Value = Option<Arc<dyn Any + Send + Sync>>;

/// `SerializationInfo`
#[derive(Default, Clone)]
pub struct SerializationInfo {
    members: Vec<(String, Value)>,
}

impl SerializationInfo {
    pub fn new() -> Self {
        SerializationInfo::default()
    }

    /// `AddValue(name, value)`: a duplicate name throws `SerializationException`.
    pub fn add_value<T: Any + Send + Sync>(&mut self, name: &str, value: Option<T>) -> Result<(), Exception> {
        if self.members.iter().any(|m| m.0 == name) {
            return Err(Exception::Serialization("Cannot add the same member twice to a SerializationInfo object.".into()));
        }
        self.members.push((name.to_string(), value.map(|v| Arc::new(v) as Arc<dyn Any + Send + Sync>)));
        Ok(())
    }

    /// `GetValue(name, typeof(T))`: a missing member throws `SerializationException`, a
    /// null member is `None`, and a member of another type fails the `IConvertible`
    /// conversion with `InvalidCastException`.
    pub fn get_value<T: Any + Clone>(&self, name: &str) -> Result<Option<T>, Exception> {
        let m = self.members.iter().find(|m| m.0 == name).ok_or_else(|| Exception::Serialization(format!("Member '{name}' was not found.")))?;
        match &m.1 {
            None => Ok(None),
            Some(v) => match v.downcast_ref::<T>() {
                Some(t) => Ok(Some(t.clone())),
                None => Err(Exception::InvalidCast("Object must implement IConvertible.".into())),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_and_get() {
        let mut i = SerializationInfo::new();
        i.add_value("a", Some(5i32)).unwrap();
        i.add_value::<String>("b", None).unwrap();
        assert!(i.add_value("a", Some(1i32)).is_err());
        assert_eq!(i.get_value::<i32>("a").unwrap(), Some(5));
        assert_eq!(i.get_value::<String>("b").unwrap(), None);
        assert!(matches!(i.get_value::<i32>("zz"), Err(Exception::Serialization(m)) if m == "Member 'zz' was not found."));
        assert!(matches!(i.get_value::<String>("a"), Err(Exception::InvalidCast(_))));
    }
}
