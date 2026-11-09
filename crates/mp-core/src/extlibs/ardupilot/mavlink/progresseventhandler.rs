//! Port of `ExtLibs/ArduPilot/Mavlink/ProgressEventHandler.cs`

/// `public delegate void ProgressEventHandler(int percent, string status)`
pub type ProgressEventHandler = std::sync::Arc<dyn Fn(i32, &str) + Send + Sync>;
