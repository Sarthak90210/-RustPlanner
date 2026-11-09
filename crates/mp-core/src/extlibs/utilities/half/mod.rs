//! `ExtLibs/Utilities/Half`: `System.Half` as Mission Planner ships it (the BCL had no
//! half type on .NET Framework).

#[allow(clippy::module_inception)]
pub mod half;
pub mod halfhelper;

pub use self::half::Half;
pub use self::halfhelper::HalfHelper;
