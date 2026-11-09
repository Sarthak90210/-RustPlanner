//! Port of `ExtLibs/Core/ExtendedObjects/EventList.cs`
//!
//! `EventList<T> : List<T>` hides `Add`/`Insert`/`AddRange`/`Remove`/`Clear` with versions that
//! raise events. The Rust type derefs to the underlying `Vec<T>` for the inherited `List<T>`
//! members, as calling through a `List<T>` reference in C# reaches the base methods. The
//! handlers receive the item (or, for `Clearing`, the list itself) by reference, as the C#
//! reference-type arguments alias the caller's objects.

use std::any::Any;

use crate::extlibs::comms::commsbase::StaticEvent;
use crate::extlibs::core::events::GenericEventArgs;

/// `GenericEventHandler<T>` as `EventList<T>` raises it for one item.
pub type ItemEventHandler<T> = dyn for<'a> Fn(Option<&dyn Any>, &GenericEventArgs<&'a T>) + Send + Sync;
/// `GenericEventHandler<List<T>>` as `EventList<T>.Clearing` raises it.
pub type ListEventHandler<T> = dyn for<'a> Fn(Option<&dyn Any>, &GenericEventArgs<&'a Vec<T>>) + Send + Sync;

/// `public class EventList<T> : List<T>`
#[allow(non_snake_case)]
pub struct EventList<T: 'static> {
    items: Vec<T>,
    /// `public event GenericEventHandler<T> ObjectRemoved`
    pub ObjectRemoved: StaticEvent<ItemEventHandler<T>>,
    /// `public event GenericEventHandler<T> ObjectAdded`
    pub ObjectAdded: StaticEvent<ItemEventHandler<T>>,
    /// `public event GenericEventHandler<List<T>> Clearing`
    pub Clearing: StaticEvent<ListEventHandler<T>>,
}

impl<T: 'static> Default for EventList<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: 'static> std::ops::Deref for EventList<T> {
    type Target = Vec<T>;
    fn deref(&self) -> &Vec<T> {
        &self.items
    }
}

impl<T: 'static> std::ops::DerefMut for EventList<T> {
    fn deref_mut(&mut self) -> &mut Vec<T> {
        &mut self.items
    }
}

impl<T: 'static> EventList<T> {
    /// `new EventList()`
    pub fn new() -> Self {
        EventList { items: Vec::new(), ObjectRemoved: StaticEvent::new(), ObjectAdded: StaticEvent::new(), Clearing: StaticEvent::new() }
    }

    /// `new EventList(IEnumerable<T> collection)`: copies without raising events.
    pub fn from_collection<I: IntoIterator<Item = T>>(collection: I) -> Self {
        let mut l = Self::new();
        l.items.extend(collection);
        l
    }

    /// `new EventList(int capacity)`: a negative capacity throws as `List<T>(int)` does.
    pub fn with_capacity(capacity: i32) -> Result<Self, crate::dotnet::Exception> {
        if capacity < 0 {
            return Err(crate::dotnet::Exception::ArgumentOutOfRange(
                "Non-negative number required.\r\nParameter name: capacity".into(),
            ));
        }
        let mut l = Self::new();
        l.items.reserve(capacity as usize);
        Ok(l)
    }

    /// `public new void Add(T item)`
    pub fn add(&mut self, item: T) {
        self.items.push(item);
        let last = self.items.len() - 1;
        self.call_object_added_at(last);
    }

    /// `public new void Insert(int index, T item)`: an index outside `0..=Count` throws before
    /// any event, as `List<T>.Insert` does.
    pub fn insert(&mut self, index: i32, item: T) -> Result<(), crate::dotnet::Exception> {
        if index < 0 || index as usize > self.items.len() {
            return Err(crate::dotnet::Exception::ArgumentOutOfRange(
                "Index must be within the bounds of the List.\r\nParameter name: index".into(),
            ));
        }
        self.items.insert(index as usize, item);
        self.call_object_added_at(index as usize);
        Ok(())
    }

    /// `public new void AddRange(IEnumerable<T> collection)`: one `Add` (and event) per item.
    pub fn add_range<I: IntoIterator<Item = T>>(&mut self, collection: I) {
        for item in collection {
            self.add(item);
        }
    }

    /// `public new void Remove(T item)`: removes the first equal item (if any) and raises
    /// `ObjectRemoved` whether or not one was found.
    pub fn remove(&mut self, item: &T)
    where
        T: PartialEq,
    {
        if let Some(i) = self.items.iter().position(|x| x == item) {
            self.items.remove(i);
        }
        self.call_object_removed(item);
    }

    /// `public void RemoveRange(IEnumerable<T> collection)`
    pub fn remove_range<'a, I: IntoIterator<Item = &'a T>>(&mut self, collection: I)
    where
        T: PartialEq,
    {
        for item in collection {
            self.remove(item);
        }
    }

    /// `public new void Clear()`: `Clearing` sees the list before it is emptied.
    pub fn clear(&mut self) {
        self.call_clearing();
        self.items.clear();
    }

    fn call_object_added_at(&self, index: usize) {
        self.call_object_added(&self.items[index]);
    }

    /// `protected void Call_ObjectAdded(T item)`
    pub fn call_object_added(&self, item: &T) {
        let args = GenericEventArgs::new(item);
        for h in self.ObjectAdded.list() {
            h(None, &args);
        }
    }

    /// `protected void Call_ObjectRemoved(T item)`
    pub fn call_object_removed(&self, item: &T) {
        let args = GenericEventArgs::new(item);
        for h in self.ObjectRemoved.list() {
            h(None, &args);
        }
    }

    /// `protected void Call_Clearing(List<T> items)` with `items == this`.
    pub fn call_clearing(&self) {
        let args = GenericEventArgs::new(&self.items);
        for h in self.Clearing.list() {
            h(None, &args);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    #[test]
    fn events_fire_in_order() {
        let log = Arc::new(Mutex::new(Vec::<String>::new()));
        let mut l: EventList<i32> = EventList::new();
        let lg = log.clone();
        l.ObjectAdded.add(Arc::new(move |_, e: &GenericEventArgs<&i32>| lg.lock().unwrap().push(format!("add {}", e.Obj))));
        let lg = log.clone();
        l.ObjectRemoved.add(Arc::new(move |_, e: &GenericEventArgs<&i32>| lg.lock().unwrap().push(format!("rem {}", e.Obj))));
        let lg = log.clone();
        l.Clearing.add(Arc::new(move |_, e: &GenericEventArgs<&Vec<i32>>| lg.lock().unwrap().push(format!("clear {}", e.Obj.len()))));
        l.add(1);
        l.add_range([2, 3]);
        l.insert(0, 9).unwrap();
        assert!(l.insert(9, 9).is_err());
        l.remove(&7); // not present: still raises
        l.remove(&2);
        assert_eq!(*l, vec![9, 1, 3]);
        l.clear();
        assert!(l.is_empty());
        assert_eq!(
            *log.lock().unwrap(),
            vec!["add 1", "add 2", "add 3", "add 9", "rem 7", "rem 2", "clear 3"]
        );
        // Vec's own push (the hidden List<T>.Add) raises nothing
        l.push(4);
        assert_eq!(log.lock().unwrap().len(), 7);
    }
}
