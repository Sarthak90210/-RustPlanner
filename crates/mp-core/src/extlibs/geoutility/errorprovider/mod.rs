//! `ExtLibs/GeoUtility/ErrorProvider`: namespace `GeoUtility.ErrorProvider`.

#[allow(clippy::module_inception)]
pub mod errorprovider;

pub use self::errorprovider::{geo_exception, geo_exception_default, text_or_empty, ErrorMessage};
