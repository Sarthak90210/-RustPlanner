//! Port of `ExtLibs/Utilities/Attributes/DisplayTextAttribute.cs`

use crate::dotnet::Exception;
use crate::extlibs::utilities::l10nu::L10NU;

/// `public sealed class GroupText : DisplayNameAttribute`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GroupText {
    /// `DisplayNameAttribute.DisplayName`
    pub display_name: String,
}

impl GroupText {
    pub fn new(text: &str) -> Self {
        GroupText { display_name: text.to_string() }
    }
}

/// Used to decorate a type or type member with display text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayTextAttribute {
    _text: String,
}

impl DisplayTextAttribute {
    /// Throws when `text` is null or empty.
    pub fn new(text: &str) -> Result<Self, Exception> {
        if text.is_empty() {
            return Err(Exception::Argument("\"text\" is required.".into()));
        }
        Ok(DisplayTextAttribute { _text: text.to_string() })
    }

    /// `Text`: the translated text.
    pub fn text(&self) -> String {
        L10NU::get_string(&self._text)
    }
}
