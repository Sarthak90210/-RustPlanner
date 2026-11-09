//! Port of `ExtLibs/Utilities/IBrowserOpen.cs`

use crate::dotnet::uri::Uri;

/// `public interface IBrowserOpen`
pub trait IBrowserOpen: Send + Sync {
    /// `OpenURL(Uri uri)`
    fn open_url(&self, uri: &Uri) -> bool;
}
