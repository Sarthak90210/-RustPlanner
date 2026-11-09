//! Port of `ExtLibs/GeoUtility/Transformation/PODWGS.cs`

use std::f64::consts::PI;

use super::Transformation;
use crate::extlibs::geoutility::geosystem::helper::GeoDatum;
use crate::extlibs::geoutility::geosystem::Geographic;

impl Transformation {
    /// `PODWGS(Geographic geo)`: datum shift Potsdam (Germany only) to WGS84.
    pub(crate) fn podwgs(&self, geo: &Geographic) -> Geographic {
        let laenge_potsdam = geo.longitude();
        let breite_potsdam = geo.latitude();

        // Latitude and longitude (rad)
        let breite_rad = breite_potsdam * (PI / 180.0);
        let laenge_rad = laenge_potsdam * (PI / 180.0);

        // Transverse radius of curvature
        let qkhm = Self::HALBACHSE / (1.0 - Self::EXZENT * breite_rad.sin() * breite_rad.sin()).sqrt();

        // Cartesian coordinates, Potsdam
        let x_potsdam = qkhm * breite_rad.cos() * laenge_rad.cos();
        let y_potsdam = qkhm * breite_rad.cos() * laenge_rad.sin();
        let z_potsdam = (1.0 - Self::EXZENT) * qkhm * breite_rad.sin();

        // Cartesian coordinates, WGS84
        let x = x_potsdam + Self::POTSDAM_DATUM_SHIFT_X;
        let y = y_potsdam + Self::POTSDAM_DATUM_SHIFT_Y;
        let z = z_potsdam + Self::POTSDAM_DATUM_SHIFT_Z;

        // Latitude and longitude, WGS84
        let b = (x * x + y * y).sqrt();
        let breite = (180.0 / PI) * ((z / b) / (1.0 - Self::WGS84_EXZENT)).atan();

        let mut laenge = 0.0;
        if x > 0.0 {
            laenge = (180.0 / PI) * (y / x).atan();
        }
        if x < 0.0 && y > 0.0 {
            laenge = (180.0 / PI) * (y / x).atan() + 180.0;
        }
        if x < 0.0 && y < 0.0 {
            laenge = (180.0 / PI) * (y / x).atan() - 180.0;
        }

        Geographic::with_datum(laenge, breite, GeoDatum::WGS84)
    }
}
