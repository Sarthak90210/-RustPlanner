//! Port of `ExtLibs/Comms/CommsBase.cs`

use std::any::Any;
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// `delegate string SettingsOption(string name, string value, bool set = false)`
pub type SettingsOption = Arc<dyn Fn(&str, &str, bool) -> String + Send + Sync>;

/// `delegate void ApplyThemeTo(object control)`
pub type ApplyThemeTo = Arc<dyn Fn(&dyn Any) + Send + Sync>;

/// `delegate inputboxreturn InputBoxShow(string title, string prompttext, ref string text)`
pub type InputBoxShow = Arc<dyn Fn(&str, &str, &mut String) -> inputboxreturn + Send + Sync>;

#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum inputboxreturn {
    OK,
    Cancel,
    NotSet,
}

/// A static C# event: handlers run in subscription order; a delegate with a return value
/// yields the last handler's result.
pub struct StaticEvent<T: ?Sized> {
    handlers: RwLock<Vec<(usize, Arc<T>)>>,
    next: RwLock<usize>,
}

impl<T: ?Sized> StaticEvent<T> {
    pub const fn new() -> Self {
        StaticEvent { handlers: RwLock::new(Vec::new()), next: RwLock::new(0) }
    }

    /// `event += handler`; the id removes it again.
    pub fn add(&self, handler: Arc<T>) -> usize {
        let mut n = self.next.write().unwrap_or_else(|e| e.into_inner());
        let id = *n;
        *n += 1;
        self.handlers.write().unwrap_or_else(|e| e.into_inner()).push((id, handler));
        id
    }

    /// `event -= handler`
    pub fn remove(&self, id: usize) {
        self.handlers.write().unwrap_or_else(|e| e.into_inner()).retain(|(i, _)| *i != id);
    }

    /// The invocation list; empty is the C# `null` event.
    pub fn list(&self) -> Vec<Arc<T>> {
        self.handlers.read().unwrap_or_else(|e| e.into_inner()).iter().map(|(_, h)| h.clone()).collect()
    }
}

impl<T: ?Sized> Default for StaticEvent<T> {
    fn default() -> Self {
        Self::new()
    }
}

/// `public static event InputBoxShow InputBoxShow`
pub static INPUT_BOX_SHOW: StaticEvent<dyn Fn(&str, &str, &mut String) -> inputboxreturn + Send + Sync> = StaticEvent::new();
/// `public static event SettingsOption Settings`
pub static SETTINGS: StaticEvent<dyn Fn(&str, &str, bool) -> String + Send + Sync> = StaticEvent::new();
/// `public static event ApplyThemeTo ApplyTheme`
pub static APPLY_THEME: StaticEvent<dyn Fn(&dyn Any) + Send + Sync> = StaticEvent::new();

/// `public abstract class CommsBase`: embedded in each transport.
#[derive(Debug, Default, Clone)]
pub struct CommsBase {
    cache: HashMap<String, String>,
}

impl CommsBase {
    pub fn new() -> Self {
        Self::default()
    }

    /// `protected virtual void ApplyThemeTo(object control)`
    pub fn apply_theme_to(&self, control: &dyn Any) {
        for h in APPLY_THEME.list() {
            h(control);
        }
    }

    /// `protected virtual inputboxreturn OnInputBoxShow(string title, string prompttext, ref string text)`
    pub fn on_input_box_show(&self, title: &str, prompttext: &str, text: &mut String) -> inputboxreturn {
        let list = INPUT_BOX_SHOW.list();
        if list.is_empty() {
            return inputboxreturn::NotSet;
        }
        let mut ret = inputboxreturn::NotSet;
        for h in list {
            ret = h(title, prompttext, text);
        }
        ret
    }

    /// `protected virtual string OnSettings(string name, string value, bool set = false)`
    pub fn on_settings(&mut self, name: &str, value: &str, set: bool) -> String {
        // answer using external function
        let list = SETTINGS.list();
        if !list.is_empty() {
            // get the external saved value
            let mut answer = String::new();
            for h in list {
                answer = h(name, value, set);
            }
            // return value if its a bad answer
            if answer.is_empty() {
                return value.to_string();
            }
            // return external value
            return answer;
        }

        // save it if we dont have a config
        if set {
            self.cache.insert(name.to_string(), value.to_string());
        }

        // return it if we have seen it
        if let Some(v) = self.cache.get(name) {
            return v.clone();
        }

        // return what was passed in if no answer
        value.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn settings_cache_without_handler() {
        // No SETTINGS handler is registered by any test in this crate.
        let mut b = CommsBase::new();
        assert_eq!(b.on_settings("TCP_port", "5760", false), "5760");
        assert_eq!(b.on_settings("TCP_port", "1234", true), "1234");
        assert_eq!(b.on_settings("TCP_port", "9999", false), "1234");
        let mut t = "x".to_string();
        assert_eq!(b.on_input_box_show("a", "b", &mut t), inputboxreturn::NotSet);
    }
}
