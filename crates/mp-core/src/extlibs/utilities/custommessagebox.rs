//! Port of `ExtLibs/Utilities/CustomMessageBox.cs`
//!
//! (namespace `System`)
//!
//! The library-side message box: it only raises `ShowEvent`, which the UI subscribes to (the
//! app's `MissionPlanner.MsgBox.CustomMessageBox`). With no subscriber it throws.

use std::sync::{Arc, RwLock};

use crate::dotnet::Exception;

crate::cs_enum! {
    /// `public enum DialogResult`
    pub DialogResult: i32 {
        None = 0,
        OK = 1,
        Cancel = 2,
        Abort = 3,
        Retry = 4,
        Ignore = 5,
        Yes = 6,
        No = 7,
    }
}

crate::cs_enum! {
    /// `public enum MessageBoxButtons`
    pub MessageBoxButtons: i32 {
        OK = 0,
        OKCancel = 1,
        AbortRetryIgnore = 2,
        YesNoCancel = 3,
        YesNo = 4,
        RetryCancel = 5,
    }
}

crate::cs_enum! {
    /// `public enum MessageBoxIcon`. Several names share a value; `ToString()` returns the
    /// name .NET Framework's binary search lands on.
    pub MessageBoxIcon: i32 {
        None = 0,
        Hand = 16,
        Question = 32,
        Exclamation = 48,
        Asterisk = 64,
        Stop = 16,
        Error = 16,
        Warning = 48,
        Information = 64,
    }
}

impl MessageBoxIcon {
    /// `Enum.ToString()`: the generic binary-search lookup, which picks Hand, Warning and
    /// Asterisk for the shared values.
    pub fn cs_name(self) -> String {
        self.to_string()
    }
}

/// `public delegate DialogResult ShowDelegate(string text, string caption, MessageBoxButtons
/// buttons, MessageBoxIcon icon, string YesText, string NoText)`
pub type ShowDelegate = Arc<dyn Fn(&str, &str, MessageBoxButtons, MessageBoxIcon, &str, &str) -> DialogResult + Send + Sync>;

/// `public static event ShowDelegate ShowEvent`: the subscribers in order.
static SHOW_EVENT: RwLock<Vec<ShowDelegate>> = RwLock::new(Vec::new());

/// `public static class CustomMessageBox`
pub struct CustomMessageBox;

impl CustomMessageBox {
    /// `ShowEvent += handler`
    pub fn add_show_event(handler: ShowDelegate) {
        SHOW_EVENT.write().unwrap_or_else(|e| e.into_inner()).push(handler);
    }

    /// `ShowEvent -= handler` (by identity)
    pub fn remove_show_event(handler: &ShowDelegate) {
        let mut v = SHOW_EVENT.write().unwrap_or_else(|e| e.into_inner());
        // a multicast delegate removes the last matching entry
        if let Some(pos) = v.iter().rposition(|h| Arc::ptr_eq(h, handler)) {
            v.remove(pos);
        }
    }

    /// `Show(string text)`
    pub fn show(text: &str) -> Result<DialogResult, Exception> {
        Self::show_full(text, "", MessageBoxButtons::OK, MessageBoxIcon::None, "Yes", "No")
    }

    /// `Show(string text, string caption)`
    pub fn show_caption(text: &str, caption: &str) -> Result<DialogResult, Exception> {
        Self::show_full(text, caption, MessageBoxButtons::OK, MessageBoxIcon::None, "Yes", "No")
    }

    /// `int Show(string text, string caption, object MessageBoxButtons, object MessageBoxIcon)`:
    /// null arguments mean OK and None; the result as an int.
    pub fn show_object(text: &str, caption: &str, buttons: Option<i32>, icon: Option<i32>) -> Result<i32, Exception> {
        let b = buttons.unwrap_or(MessageBoxButtons::OK.0);
        let i = icon.unwrap_or(MessageBoxIcon::None.0);
        Ok(Self::show_full(text, caption, MessageBoxButtons(b), MessageBoxIcon(i), "Yes", "No")?.0)
    }

    /// `Show(text, caption, buttons, icon, YesText, NoText)`: writes a console line, then
    /// invokes every subscriber and returns the last one's result (multicast delegate).
    pub fn show_full(
        text: &str,
        caption: &str,
        buttons: MessageBoxButtons,
        icon: MessageBoxIcon,
        yes_text: &str,
        no_text: &str,
    ) -> Result<DialogResult, Exception> {
        crate::dotnet::io::console_write_line("CustomMessageBox.Show");

        let handlers = SHOW_EVENT.read().unwrap_or_else(|e| e.into_inner()).clone();
        if !handlers.is_empty() {
            let mut result = DialogResult::None;
            for h in &handlers {
                result = h(text, caption, buttons, icon, yes_text, no_text);
            }
            return Ok(result);
        }

        Err(Exception::Other("ShowEvent Not Set".into()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn icon_names() {
        assert_eq!(MessageBoxIcon::Error.cs_name(), "Hand");
        assert_eq!(MessageBoxIcon::Exclamation.cs_name(), "Warning");
        assert_eq!(MessageBoxIcon::Information.cs_name(), "Asterisk");
    }
}
