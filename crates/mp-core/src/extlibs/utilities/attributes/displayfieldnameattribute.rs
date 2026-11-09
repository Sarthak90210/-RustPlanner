//! Port of `ExtLibs/Utilities/Attributes/DisplayFieldNameAttribute.cs`

use crate::dotnet::Exception;
use crate::extlibs::utilities::l10nu::L10NU;

/// Used to decorate a field with a custom name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayFieldNameAttribute {
    _translation_key: String,
}

impl DisplayFieldNameAttribute {
    /// Throws when `translationKey` is null or empty.
    pub fn new(translation_key: &str) -> Result<Self, Exception> {
        if translation_key.is_empty() {
            return Err(Exception::Argument("\"translationKey\" is required.".into()));
        }
        Ok(DisplayFieldNameAttribute { _translation_key: translation_key.to_string() })
    }

    /// `TryTranslate(defaultTo)`
    pub fn try_translate(&self, default_to: &str) -> String {
        L10NU::get_string_or(&self._translation_key, default_to)
    }
}
