//! `ExtLibs/GeoUtility/GeoSystem`: namespace `GeoUtility.GeoSystem`. `GaussKrueger` and
//! `MapService` (with its `MapService/` helpers) are not ported yet.

pub mod base;
pub mod geographic;
pub mod helper;
pub mod mgrs;
pub mod utm;

pub use self::geographic::Geographic;
pub use self::mgrs::MGRS;
pub use self::utm::UTM;

use crate::dotnet::collections::Dictionary;

/// The out parameters of the GeoUtility `TryParse` methods: the return value, the parsed
/// object (or `null`), the error text and the per-field validity list.
#[derive(Debug, Clone)]
pub struct TryParseResult<T> {
    pub ok: bool,
    pub value: Option<T>,
    pub error_message: String,
    pub valid_items: Dictionary<String, bool>,
}
