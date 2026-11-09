//! `ExtLibs/GMap.NET.Core/GMap.NET.Projections`: namespace `GMap.NET.Projections`.

pub mod epsg3995projection;
pub mod epsg5936projection;
pub mod mercatorprojection;
pub mod mercatorprojectiongcj;
pub mod mercatorprojectionyandex;
pub mod noprojection;
pub mod platecarreeprojection;
pub mod platecarreeprojectionpergo;
pub mod polarstereographicnorthprojection;
pub mod polarstereographicsouthprojection;

pub use self::mercatorprojection::MercatorProjection;

