//! Port of `ExtLibs/Core/Events.cs`
//!
//! The generic delegates take the sender as `Option<&dyn Any>` (C# `object sender`, which the
//! callers in Core pass as `null`).

use std::any::Any;

/// `public class GenericEventArgs<T> : EventArgs`
#[allow(non_snake_case)]
#[derive(Debug, Clone, PartialEq)]
pub struct GenericEventArgs<T> {
    pub Obj: T,
}

impl<T> GenericEventArgs<T> {
    /// `new GenericEventArgs<T>(T obj)`
    pub fn new(obj: T) -> Self {
        GenericEventArgs { Obj: obj }
    }
}

/// `public delegate void GenericEventHandler<T>(object sender, GenericEventArgs<T> e)`
pub type GenericEventHandler<T> = dyn Fn(Option<&dyn Any>, &GenericEventArgs<T>) + Send + Sync;

/// `public class GenericEventArgs<A, B> : EventArgs`
#[allow(non_snake_case)]
#[derive(Debug, Clone, PartialEq)]
pub struct GenericEventArgs2<A, B> {
    pub ObjA: A,
    pub ObjB: B,
}

impl<A, B> GenericEventArgs2<A, B> {
    /// `new GenericEventArgs<A, B>(A objA, B objB)`
    pub fn new(obj_a: A, obj_b: B) -> Self {
        GenericEventArgs2 { ObjA: obj_a, ObjB: obj_b }
    }
}

/// `public delegate void GenericEventHandler<A, B>(object sender, GenericEventArgs<A, B> e)`
pub type GenericEventHandler2<A, B> = dyn Fn(Option<&dyn Any>, &GenericEventArgs2<A, B>) + Send + Sync;

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[test]
    fn args_hold_values() {
        let e = GenericEventArgs::new(5);
        assert_eq!(e.Obj, 5);
        let e2 = GenericEventArgs2::new("a", 2.5);
        assert_eq!(e2.ObjA, "a");
        assert_eq!(e2.ObjB, 2.5);
        let h: Arc<GenericEventHandler<i32>> = Arc::new(|s, e| assert!(s.is_none() && e.Obj == 5));
        h(None, &e);
    }
}
