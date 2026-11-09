//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/Singleton.cs`
//!
//! `Singleton<T>` keeps one lazily made `T` per type, like the nested `SingletonCreator`
//! class whose static constructor runs `new T()` once. A Rust generic cannot own a static,
//! so the per-type cells live in a map keyed by `TypeId`. If the constructor throws, the
//! exception is stored and every `Instance` access rethrows it. While the constructor runs,
//! `Instance` seen from inside it is still null (`check_ctor` passes), as in C#.

#![allow(non_snake_case)]

use std::any::{Any, TypeId};
use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

use crate::dotnet::Exception;

/// `where T : new()`: the parameterless constructor `SingletonCreator` calls.
pub trait SingletonNew: Sized + Send + Sync + 'static {
    fn new_instance() -> Result<Self, Exception>;
}

type Cell = Arc<OnceLock<Result<&'static (dyn Any + Send + Sync), Exception>>>;

fn cell_for(id: TypeId) -> Cell {
    static CELLS: OnceLock<Mutex<HashMap<TypeId, Cell>>> = OnceLock::new();
    let map = CELLS.get_or_init(|| Mutex::new(HashMap::new()));
    map.lock().unwrap_or_else(|e| e.into_inner()).entry(id).or_default().clone()
}

/// `public class Singleton<T> where T : new()`
pub struct Singleton<T>(std::marker::PhantomData<T>);

impl<T: SingletonNew> Singleton<T> {
    /// `protected Singleton()`: the guard a derived constructor runs first.
    pub fn check_ctor() -> Result<(), Exception> {
        if let Some(Ok(_)) = cell_for(TypeId::of::<T>()).get() {
            return Err(Exception::Other("You have tried to create a new singleton class where you should have instanced it. Replace your \"new class()\" with \"class.Instance\"".into()));
        }
        Ok(())
    }

    /// `public static T Instance`
    pub fn Instance() -> Result<&'static T, Exception> {
        let cell = cell_for(TypeId::of::<T>());
        let r = cell.get_or_init(|| match T::new_instance() {
            Ok(v) => {
                let leaked: &'static T = Box::leak(Box::new(v));
                Ok(leaked as &'static (dyn Any + Send + Sync))
            }
            Err(exception) => {
                log::trace!("Singleton: {exception}");
                Err(exception)
            }
        });
        match r {
            Ok(v) => Ok(v.downcast_ref::<T>().expect("singleton cell holds its own type")),
            Err(e) => Err(e.clone()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicI32, Ordering};

    static MADE: AtomicI32 = AtomicI32::new(0);

    struct Thing {
        n: i32,
    }

    impl SingletonNew for Thing {
        fn new_instance() -> Result<Self, Exception> {
            Singleton::<Thing>::check_ctor()?;
            Ok(Thing { n: MADE.fetch_add(1, Ordering::SeqCst) + 1 })
        }
    }

    struct Broken;

    impl SingletonNew for Broken {
        fn new_instance() -> Result<Self, Exception> {
            Err(Exception::InvalidOperation("boom".into()))
        }
    }

    #[test]
    fn one_instance_and_stored_exception() {
        let a = Singleton::<Thing>::Instance().unwrap();
        let b = Singleton::<Thing>::Instance().unwrap();
        assert!(std::ptr::eq(a, b));
        assert_eq!(a.n, 1);
        // a second `new Thing()` after the instance exists throws
        assert!(matches!(Thing::new_instance(), Err(Exception::Other(_))));
        assert_eq!(Singleton::<Broken>::Instance().err(), Some(Exception::InvalidOperation("boom".into())));
        assert!(Singleton::<Broken>::Instance().is_err());
    }
}
