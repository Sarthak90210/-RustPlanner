//! Port of `ExtLibs/Mavlink/MAVLinkParamList.cs`
//!
//! The C# class guards its name indexer with a `ReaderWriterLock` while the inherited
//! `List<T>` members stay unsynchronised. In Rust the owner (`MAVState`) holds the whole list
//! behind its own lock, so the methods here take `&self` / `&mut self` without locking again.

use indexmap::IndexMap;

use super::mavlinkparam::MAVLinkParam;
use crate::dotnet::Exception;

/// `PropertyChangedEventHandler(object sender, PropertyChangedEventArgs e)`
type PropertyChangedHandler = Box<dyn Fn(&MAVLinkParamList, &str) + Send + Sync>;

/// `MAVLink.MAVLinkParamList : List<MAVLinkParam>, INotifyPropertyChanged`
#[derive(Default)]
pub struct MAVLinkParamList {
    items: Vec<MAVLinkParam>,
    pub total_reported: i32,
    property_changed: Vec<(usize, PropertyChangedHandler)>,
    next_handler: usize,
}

impl std::fmt::Debug for MAVLinkParamList {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MAVLinkParamList").field("items", &self.items).field("total_reported", &self.total_reported).finish()
    }
}

impl Clone for MAVLinkParamList {
    /// Copies the items; event subscribers stay with the original, as a new C# list has none.
    fn clone(&self) -> Self {
        MAVLinkParamList { items: self.items.clone(), total_reported: self.total_reported, property_changed: vec![], next_handler: 0 }
    }
}

impl std::ops::Deref for MAVLinkParamList {
    type Target = Vec<MAVLinkParam>;
    fn deref(&self) -> &Vec<MAVLinkParam> {
        &self.items
    }
}

/// The inherited `List<MAVLinkParam>` members (`Insert`, `Remove`, `Sort`, `this[int]`, ...).
impl std::ops::DerefMut for MAVLinkParamList {
    fn deref_mut(&mut self) -> &mut Vec<MAVLinkParam> {
        &mut self.items
    }
}

impl MAVLinkParamList {
    pub fn new() -> Self {
        Self::default()
    }

    /// `TotalReceived`: the number of items held.
    pub fn total_received(&self) -> i32 {
        self.items.len() as i32
    }

    /// `this[string name]` getter: the first item with that name, or null.
    pub fn get(&self, name: &str) -> Option<&MAVLinkParam> {
        self.items.iter().find(|i| i.name == name)
    }

    /// Mutable access to the item C# would return by reference from `this[name]`.
    pub fn get_mut(&mut self, name: &str) -> Option<&mut MAVLinkParam> {
        self.items.iter_mut().find(|i| i.name == name)
    }

    /// `this[string name]` setter: replaces the first item with that name (raising
    /// PropertyChanged "Item"), or appends without raising it.
    pub fn set(&mut self, name: &str, value: MAVLinkParam) {
        if let Some(index) = self.items.iter().position(|i| i.name == name) {
            self.items[index] = value;
            self.on_property_changed("Item");
            return;
        }
        self.items.push(value);
    }

    /// `this[string[] names]` getter: the one item found among `names`, or null when none or
    /// more than one exist. Used where a parameter was renamed and both names never coexist.
    pub fn get_any(&self, names: &[&str]) -> Option<&MAVLinkParam> {
        let mut item = None;
        for s in names {
            if let Some(new_item) = self.get(s) {
                if item.is_some() {
                    return None;
                }
                item = Some(new_item);
            }
        }
        item
    }

    /// `this[string[] names]` setter. The C# body only reassigns a local, so it changes nothing.
    pub fn set_any(&mut self, _names: &[&str], _value: MAVLinkParam) {}

    /// `Keys`
    pub fn keys(&self) -> Vec<String> {
        self.items.iter().map(|i| i.name.clone()).collect()
    }

    /// `ContainsKey(v)`
    pub fn contains_key(&self, v: &str) -> bool {
        self.items.iter().any(|i| i.name == v)
    }

    /// `new Clear()`: also resets `TotalReported`.
    pub fn clear(&mut self) {
        self.total_reported = 0;
        self.items.clear();
    }

    /// `new Add(item)`: `this[item.Name] = item`.
    pub fn add(&mut self, item: MAVLinkParam) {
        let name = item.name.clone();
        self.set(&name, item);
    }

    /// `new AddRange(collection)`: appends without de-duplicating, then raises "AddRange".
    pub fn add_range(&mut self, collection: impl IntoIterator<Item = MAVLinkParam>) {
        self.items.extend(collection);
        self.on_property_changed("AddRange");
    }

    /// `implicit operator Dictionary<string, double>`: later duplicates overwrite earlier ones.
    pub fn to_dictionary(&self) -> Result<IndexMap<String, f64>, Exception> {
        let mut copy = IndexMap::new();
        for item in &self.items {
            copy.insert(item.name.clone(), item.value()?);
        }
        Ok(copy)
    }

    /// `PropertyChanged += handler`; the returned id removes it again.
    pub fn add_property_changed(&mut self, handler: impl Fn(&MAVLinkParamList, &str) + Send + Sync + 'static) -> usize {
        let id = self.next_handler;
        self.next_handler += 1;
        self.property_changed.push((id, Box::new(handler)));
        id
    }

    /// `PropertyChanged -= handler`
    pub fn remove_property_changed(&mut self, id: usize) {
        self.property_changed.retain(|(i, _)| *i != id);
    }

    fn on_property_changed(&self, property_name: &str) {
        for (_, h) in &self.property_changed {
            h(self, property_name);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extlibs::mavlink::mavlink::MAV_PARAM_TYPE;
    use std::sync::{Arc, Mutex};

    fn p(n: &str, v: f64) -> MAVLinkParam {
        MAVLinkParam::new(n, v, MAV_PARAM_TYPE::REAL32, None)
    }

    #[test]
    fn indexer_and_events() {
        let seen = Arc::new(Mutex::new(vec![]));
        let mut l = MAVLinkParamList::new();
        let s = seen.clone();
        let id = l.add_property_changed(move |_, n| s.lock().unwrap().push(n.to_string()));
        l.add(p("A", 1.0));
        l.add(p("A", 2.0));
        l.add_range([p("A", 3.0), p("B", 4.0)]);
        assert_eq!(l.len(), 3, "AddRange does not de-duplicate");
        assert_eq!(l.get("A").unwrap().value().unwrap(), 2.0);
        assert_eq!(*seen.lock().unwrap(), vec!["Item", "AddRange"]);
        assert_eq!(l.to_dictionary().unwrap()["A"], 3.0);
        assert!(l.get_any(&["A", "B"]).is_none());
        assert_eq!(l.get_any(&["B", "C"]).unwrap().name, "B");
        l.remove_property_changed(id);
        l.add(p("B", 9.0));
        assert_eq!(seen.lock().unwrap().len(), 2, "unsubscribed handler is not called");
        l.insert(0, p("Z", 0.0));
        assert_eq!(l[0].name, "Z");
        l.total_reported = 5;
        l.clear();
        assert_eq!((l.len(), l.total_reported), (0, 0));
    }
}
