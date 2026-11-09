//! Port of `ExtLibs/Utilities/ICorrections.cs`

/// `public interface ICorrections`: a GNSS correction stream parser.
pub trait ICorrections {
    /// overall packet length (use for sending forward)
    fn length(&self) -> i32;
    /// raw packet data
    fn packet(&self) -> &[u8];
    /// reset the parser to the initial state
    #[allow(non_snake_case)]
    fn resetParser(&mut self) -> bool;
}
