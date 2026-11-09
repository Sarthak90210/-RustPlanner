//! Port of `ExtLibs/GeoUtility/Transformation/WGSPOD.cs`

use std::f64::consts::PI;

use super::Transformation;
use crate::dotnet::Exception;
use crate::extlibs::geoutility::errorprovider::{geo_exception, ErrorMessage};
use crate::extlibs::geoutility::geosystem::helper::GeoDatum;
use crate::extlibs::geoutility::geosystem::Geographic;

impl Transformation {
    /// `WGSPOD(Geographic geo)`: datum shift WGS84 to Potsdam (Germany only). Fails outside
    /// the German Gauss-Krueger area.
    pub(crate) fn wgspod(&self, geo: &Geographic) -> Result<Geographic, Exception> {
        let laenge_wgs84 = geo.longitude();
        let breite_wgs84 = geo.latitude();

        // Latitude and longitude (rad)
        let breite_rad = breite_wgs84 * (PI / 180.0);
        let laenge_rad = laenge_wgs84 * (PI / 180.0);

        // Transverse radius of curvature
        let qkhm = Self::WGS84_HALBACHSE / (1.0 - Self::WGS84_EXZENT * breite_rad.sin() * breite_rad.sin()).sqrt();

        // Cartesian coordinates, WGS84
        let x_wgs84 = qkhm * breite_rad.cos() * laenge_rad.cos();
        let y_wgs84 = qkhm * breite_rad.cos() * laenge_rad.sin();
        let z_wgs84 = (1.0 - Self::WGS84_EXZENT) * qkhm * breite_rad.sin();

        // Cartesian coordinates, Potsdam
        let x = x_wgs84 - Self::POTSDAM_DATUM_SHIFT_X;
        let y = y_wgs84 - Self::POTSDAM_DATUM_SHIFT_Y;
        let z = z_wgs84 - Self::POTSDAM_DATUM_SHIFT_Z;

        // Latitude and longitude, Potsdam
        let b = (x * x + y * y).sqrt();
        let breite = (180.0 / PI) * ((z / b) / (1.0 - Self::EXZENT)).atan();

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

        if laenge < Self::MIN_OST || laenge > Self::MAX_OST || breite < Self::MIN_NORD || breite > Self::MAX_NORD {
            return Err(geo_exception(ErrorMessage::new("ERROR_GK_OUT_OF_RANGE")));
        }

        Ok(Geographic::with_datum(laenge, breite, GeoDatum::Potsdam))
    }
}
