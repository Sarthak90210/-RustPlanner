//! `ExtLibs/Interfaces` (`Interfaces.csproj`).

pub mod iactivate;
pub mod icommsserial;
pub mod servicelocator;

pub use self::iactivate::{IActivate, IDeactivate};
pub use self::icommsserial::ICommsSerial;
pub use self::servicelocator::ServiceLocator;
