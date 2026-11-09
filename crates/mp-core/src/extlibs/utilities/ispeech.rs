//! Port of `ExtLibs/Utilities/ISpeech.cs`

/// `public interface ISpeech`
pub trait ISpeech: Send + Sync {
    #[allow(non_snake_case)]
    fn speechEnable(&self) -> bool;
    #[allow(non_snake_case)]
    fn set_speechEnable(&self, value: bool);
    /// `IsReady`
    fn is_ready(&self) -> bool;
    /// `SpeakAsync(text)`
    fn speak_async(&self, text: &str);
    /// `SpeakAsyncCancelAll()`
    fn speak_async_cancel_all(&self);
}
