//! Port of `ExtLibs/GeoUtility/GeoSystem/Base/BaseSystem.cs`

use crate::extlibs::geoutility::transformation::Transformation;

/// `internal static GeoUtility.Transformation Transform`: the shared instance every
/// coordinate class converts through. The abstract `BaseSystem` class itself has no state;
/// in Rust the derived types are plain structs and reach the transformation through this.
pub(crate) static TRANSFORM: Transformation = Transformation::new();
