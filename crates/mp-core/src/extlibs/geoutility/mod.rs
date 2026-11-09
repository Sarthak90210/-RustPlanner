//! `ExtLibs/GeoUtility` (`GeoUtility.csproj`): UTM, MGRS and geographic coordinate
//! conversions. Ported file by file for the parts Mission Planner uses (UTM, MGRS,
//! Geographic); Gauss-Krueger and MapService are not ported yet.

pub mod errorprovider;
pub mod geosystem;
pub mod localizer;
pub mod properties;
pub mod transformation;
