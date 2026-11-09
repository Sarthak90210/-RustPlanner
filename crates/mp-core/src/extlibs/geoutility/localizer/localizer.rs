//! Port of `ExtLibs/GeoUtility/Localizer/Localizer.cs`

use std::sync::atomic::{AtomicBool, Ordering};

use crate::dotnet::current_ui_culture_name;
use crate::extlibs::geoutility::properties::resources_designer;

/// `private static string _culture`: "" until a `Message` is created under a German UI
/// culture, then "DE_" for the rest of the process (nothing ever resets it).
static CULTURE_DE: AtomicBool = AtomicBool::new(false);

/// `internal class Message`: a localized string picked by its ID.
#[derive(Debug, Clone, Default)]
pub struct Message {
    id: String,
}

impl Message {
    fn check_culture() {
        // CultureInfo.CurrentUICulture.TwoLetterISOLanguageName == "de"
        let name = current_ui_culture_name();
        if name.split('-').next().is_some_and(|l| l.eq_ignore_ascii_case("de")) {
            CULTURE_DE.store(true, Ordering::Relaxed);
        }
    }

    /// `new Message()`
    pub fn empty() -> Self {
        Self::check_culture();
        Message { id: String::new() }
    }

    /// `new Message(id)`
    pub fn new(id: &str) -> Self {
        Self::check_culture();
        Message { id: id.to_string() }
    }

    /// `ID`
    pub fn id(&self) -> &str {
        &self.id
    }

    pub fn set_id(&mut self, value: &str) {
        self.id = value.to_string();
    }

    /// `Text`: the resource `_culture + ID`, `""` when the ID is empty and `null` when the
    /// resource does not exist.
    pub fn text(&self) -> Option<String> {
        if self.id.is_empty() {
            return Some(String::new());
        }
        let culture = if CULTURE_DE.load(Ordering::Relaxed) { "DE_" } else { "" };
        resources_designer::get_string(&format!("{culture}{}", self.id)).map(str::to_string)
    }
}
