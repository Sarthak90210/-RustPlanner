//! Port of `ExtLibs/GeoUtility/Transformation/WGSUTM.cs`

use std::f64::consts::PI;

use super::Transformation;
use crate::dotnet::{cast_i32, math_round, substring, Exception};
use crate::extlibs::geoutility::errorprovider::{geo_exception, ErrorMessage};
use crate::extlibs::geoutility::geosystem::helper::GeoDatum;
use crate::extlibs::geoutility::geosystem::{Geographic, UTM};

impl Transformation {
    /// `WGSUTM(Geographic geo)`: geographic coordinates to UTM. A NaN coordinate passes the
    /// range check, as in C#, and then fails on the band lookup.
    pub(crate) fn wgsutm(&self, geo: &Geographic) -> Result<UTM, Exception> {
        // if the geo object is not in the WGS84 datum
        let converted;
        let geo = if geo.datum() != GeoDatum::WGS84 {
            converted = self.podwgs(geo);
            &converted
        } else {
            geo
        };

        let laenge = geo.longitude();
        let breite = geo.latitude();

        if laenge <= Self::MIN_LAENGE || laenge > Self::MAX_LAENGE || breite < Self::MIN_BREITE || breite > Self::MAX_BREITE {
            return Err(geo_exception(ErrorMessage::new("ERROR_GEO2UTM")));
        }

        // Coefficients for the meridian arc length
        let koeff0 = Self::WGS84_POL
            * (PI / 180.0)
            * (1.0 - 3.0 * Self::WGS84_EXZENT2 / 4.0 + 45.0 * Self::WGS84_EXZENT4 / 64.0 - 175.0 * Self::WGS84_EXZENT6 / 256.0
                + 11025.0 * Self::WGS84_EXZENT8 / 16384.0);
        let koeff2 = Self::WGS84_POL
            * (-3.0 * Self::WGS84_EXZENT2 / 8.0 + 15.0 * Self::WGS84_EXZENT4 / 32.0 - 525.0 * Self::WGS84_EXZENT6 / 1024.0
                + 2205.0 * Self::WGS84_EXZENT8 / 4096.0);
        let koeff4 = Self::WGS84_POL
            * (15.0 * Self::WGS84_EXZENT4 / 256.0 - 105.0 * Self::WGS84_EXZENT6 / 1024.0 + 2205.0 * Self::WGS84_EXZENT8 / 16384.0);
        let koeff6 = Self::WGS84_POL * (-35.0 * Self::WGS84_EXZENT6 / 3072.0 + 315.0 * Self::WGS84_EXZENT8 / 12288.0);

        // Zone and band
        let mut zone = cast_i32((laenge + 180.0) / 6.0).wrapping_add(1);
        // Handling UTM exception
        if (breite >= 56.0) && (breite < 64.0) && (laenge >= 3.0) && (laenge < 12.0) {
            // South-Norway 31V-32V (32V extends to the west from 3 to 12 degrees, with degree 9 as meridian)
            zone = 32;
        } else if breite >= 72.0 {
            // Arctic region exceptions
            if (laenge >= 0.0) && (laenge < 9.0) {
                zone = 31;
            } else if (laenge >= 9.0) && (laenge < 21.0) {
                zone = 33;
            } else if (laenge >= 21.0) && (laenge < 33.0) {
                zone = 35;
            } else if (laenge >= 33.0) && (laenge < 42.0) {
                zone = 37;
            }
        }

        // string sZone = string.Format("00", zone); (unused)
        let band_index = cast_i32(1.0 + (breite + 80.0) / 8.0);
        let band = substring(Self::UTM_BAND, band_index.wrapping_sub(1), 1)?;

        // Geographic latitude (rad)
        let breite_rad = breite * PI / 180.0;

        let tangens1 = breite_rad.tan();
        let tangens2 = tangens1.powf(2.0);
        let tangens4 = tangens1.powf(4.0);
        let cosinus1 = breite_rad.cos();
        let cosinus2 = cosinus1.powf(2.0);
        let cosinus3 = cosinus1.powf(3.0);
        let cosinus4 = cosinus1.powf(4.0);
        let cosinus5 = cosinus1.powf(5.0);

        let eta = Self::WGS84_EXZENT2 * cosinus2;

        // Transverse radius of curvature
        let qkhm = Self::WGS84_POL / (1.0 + eta).sqrt();

        // Meridian arc length
        let lmbog =
            (koeff0 * breite) + (koeff2 * (2.0 * breite_rad).sin()) + (koeff4 * (4.0 * breite_rad).sin()) + (koeff6 * (6.0 * breite_rad).sin());

        // Difference to the central meridian
        let merid = zone.wrapping_sub(30).wrapping_mul(6).wrapping_sub(3);

        let dlaenge1 = (laenge - merid as f64) * PI / 180.0;
        let dlaenge2 = dlaenge1.powf(2.0);
        let dlaenge3 = dlaenge1.powf(3.0);
        let dlaenge4 = dlaenge1.powf(4.0);
        let dlaenge5 = dlaenge1.powf(5.0);

        // East, North
        let mut north = if breite < 0.0 {
            10E+06
                + Self::UTM_FAKTOR
                    * (lmbog
                        + qkhm * cosinus2 * tangens1 * dlaenge2 / 2.0
                        + qkhm * cosinus4 * tangens1 * (5.0 - tangens2 + 9.0 * eta) * dlaenge4 / 24.0)
        } else {
            Self::UTM_FAKTOR
                * (lmbog
                    + qkhm * cosinus2 * tangens1 * dlaenge2 / 2.0
                    + qkhm * cosinus4 * tangens1 * (5.0 - tangens2 + 9.0 * eta) * dlaenge4 / 24.0)
        };
        let mut east = Self::UTM_FAKTOR
            * (qkhm * cosinus1 * dlaenge1
                + qkhm * cosinus3 * (1.0 - tangens2 + eta) * dlaenge3 / 6.0
                + qkhm * cosinus5 * (5.0 - 18.0 * tangens2 + tangens4) * dlaenge5 / 120.0)
            + Self::UTM_FALSE_EASTING;

        north = math_round(north, 3);
        east = math_round(east, 3);

        UTM::with_band(zone, &band, east, north)
    }
}
