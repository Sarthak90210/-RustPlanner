//! Port of `ExtLibs/Interfaces/ServiceLocator.cs`

use std::any::{Any, TypeId};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use crate::dotnet::Exception;

static LIST: Mutex<Option<HashMap<TypeId, Arc<dyn Any + Send + Sync>>>> = Mutex::new(None);

/// `Interfaces.ServiceLocator`: one instance per registered type.
pub struct ServiceLocator;

impl ServiceLocator {
    /// `Register<T>(Func<T> start)`: throws "Type already exists" on a second registration.
    pub fn register<T: Any + Send + Sync>(start: impl FnOnce() -> T) -> Result<(), Exception> {
        let mut g = LIST.lock().unwrap_or_else(|e| e.into_inner());
        let list = g.get_or_insert_with(HashMap::new);
        if list.contains_key(&TypeId::of::<T>()) {
            return Err(Exception::Other("Type already exists".into()));
        }
        list.insert(TypeId::of::<T>(), Arc::new(start()));
        Ok(())
    }

    /// `Get<T>()`: KeyNotFoundException when the type was never registered.
    pub fn get<T: Any + Send + Sync>() -> Result<Arc<T>, Exception> {
        let g = LIST.lock().unwrap_or_else(|e| e.into_inner());
        let ans = g
            .as_ref()
            .and_then(|l| l.get(&TypeId::of::<T>()).cloned())
            .ok_or_else(|| Exception::KeyNotFound("The given key was not present in the dictionary.".into()))?;
        Ok(ans.downcast::<T>().expect("registered under its own TypeId"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Gps(u8);

    #[test]
    fn register_once_then_get() {
        assert!(ServiceLocator::get::<Gps>().is_err());
        ServiceLocator::register(|| Gps(3)).unwrap();
        assert!(ServiceLocator::register(|| Gps(4)).is_err());
        assert_eq!(ServiceLocator::get::<Gps>().unwrap().0, 3);
    }
}
