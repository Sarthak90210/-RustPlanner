//! Port of `ExtLibs/Interfaces/IActivate.cs`

/// The implementor executes some logic on activation, for e.g when moving from not selected to
/// selected in a tab control.
pub trait IActivate {
    /// Should be idempotent
    fn activate(&mut self);
}

/// The implementor executes some logic on deactivation, for e.g when moving from selected to
/// not selected in a tab control.
pub trait IDeactivate {
    /// Should be idempotent
    fn deactivate(&mut self);
}
