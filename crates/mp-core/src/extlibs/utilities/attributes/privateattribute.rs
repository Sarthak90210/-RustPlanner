//! Port of `ExtLibs/Utilities/Attributes/PrivateAttribute.cs`

/// `public sealed class PrivateAttribute : Attribute`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PrivateAttribute {
    _is_private: bool,
}

impl PrivateAttribute {
    pub fn new(is_private: bool) -> Self {
        PrivateAttribute { _is_private: is_private }
    }

    /// `IsPrivate`
    pub fn is_private(&self) -> bool {
        self._is_private
    }
}
