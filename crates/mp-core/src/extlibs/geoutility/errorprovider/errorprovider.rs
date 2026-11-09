//! Port of `ExtLibs/GeoUtility/ErrorProvider/ErrorProvider.cs`

use crate::dotnet::Exception;
use crate::extlibs::geoutility::localizer::Message;

/// `internal class ErrorMessage`: fetches the localized message for an error ID.
#[derive(Debug, Clone)]
pub struct ErrorMessage {
    id: String,
}

impl Default for ErrorMessage {
    /// `new ErrorMessage()`
    fn default() -> Self {
        ErrorMessage { id: "NO_ERROR_MESSAGE".into() }
    }
}

impl ErrorMessage {
    /// `new ErrorMessage(id)`
    pub fn new(id: &str) -> Self {
        ErrorMessage { id: id.to_string() }
    }

    /// `ID`
    pub fn id(&self) -> &str {
        &self.id
    }

    pub fn set_id(&mut self, value: &str) {
        self.id = value.to_string();
    }

    /// `Text`: `null` when the resource does not exist.
    pub fn text(&self) -> Option<String> {
        Message::new(&self.id).text()
    }
}

/// `new GeoException(message)` / `new GeoException(message, inner)`: `Exception(message.Text)`.
/// A `null` text gives .NET's default exception message. The inner exception and the
/// `Data["ExtraInfo"]` entry are not observable through the ported callers.
pub fn geo_exception(message: ErrorMessage) -> Exception {
    Exception::Geo(
        message
            .text()
            .unwrap_or_else(|| "Exception of type 'GeoUtility.ErrorProvider.GeoException' was thrown.".to_string()),
    )
}

/// `new GeoException()`
pub fn geo_exception_default() -> Exception {
    Exception::Geo("Exception of type 'GeoUtility.ErrorProvider.GeoException' was thrown.".into())
}

/// `string + null` in C# appends nothing: `ErrorMessage(id).Text + "\r\n"`.
pub fn text_or_empty(message: &ErrorMessage) -> String {
    message.text().unwrap_or_default()
}
