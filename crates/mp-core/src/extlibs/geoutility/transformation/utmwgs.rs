//! Port of `ExtLibs/GeoUtility/Transformation/UTMWGS.cs`

use std::f64::consts::PI;

use super::Transformation;
use crate::dotnet::{index_out_of_range, Exception};
use crate::extlibs::geoutility::geosystem::{Geographic, UTM};

impl Transformation {
    /// `UTMWGS(UTM utm)`: UTM to geographic coordinates (WGS84). Fails like the C# when the
    /// band is empty (`band.ToCharArray()[0]`).
    pub(crate) fn utmwgs(&self, utm: &UTM) -> Result<Geographic, Exception> {
        let zone = utm.zone();
        let band = utm.band();
        let east = utm.east();
        let mut north = utm.north();

        // Coefficients for the latitude from a meridian arc length
        let koeff0 = Self::WGS84_POL
            * (PI / 180.0)
            * (1.0 - 3.0 * Self::WGS84_EXZENT2 / 4.0 + 45.0 * Self::WGS84_EXZENT4 / 64.0 - 175.0 * Self::WGS84_EXZENT6 / 256.0
                + 11025.0 * Self::WGS84_EXZENT8 / 16384.0);
        let koeff2 = (180.0 / PI)
            * (3.0 * Self::WGS84_EXZENT2 / 8.0 - 3.0 * Self::WGS84_EXZENT4 / 16.0 + 213.0 * Self::WGS84_EXZENT6 / 2048.0
                - 255.0 * Self::WGS84_EXZENT8 / 4096.0);
        let koeff4 = (180.0 / PI)
            * (21.0 * Self::WGS84_EXZENT4 / 256.0 - 21.0 * Self::WGS84_EXZENT6 / 256.0 + 533.0 * Self::WGS84_EXZENT8 / 8192.0);
        let koeff6 = (180.0 / PI) * (151.0 * Self::WGS84_EXZENT6 / 6144.0 - 453.0 * Self::WGS84_EXZENT8 / 12288.0);

        // Northern/southern hemisphere
        let b = band.chars().next().ok_or_else(index_out_of_range)?;
        if b < 'N' && !band.is_empty() && north > 0.0 {
            north -= 10E+06;
        }

        // Latitude (rad)
        let sig = (north / Self::UTM_FAKTOR) / koeff0;
        let sig_rad = sig * PI / 180.0;
        let fbreite = sig + koeff2 * (2.0 * sig_rad).sin() + koeff4 * (4.0 * sig_rad).sin() + koeff6 * (6.0 * sig_rad).sin();
        let breite_rad = fbreite * PI / 180.0;

        let tangens1 = breite_rad.tan();
        let tangens2 = tangens1 * tangens1;
        let tangens4 = tangens2 * tangens2;
        let cosinus1 = breite_rad.cos();
        let cosinus2 = cosinus1 * cosinus1;

        let eta = Self::WGS84_EXZENT2 * cosinus2;

        // Transverse radius of curvature
        let qkhm1 = Self::WGS84_POL / (1.0 + eta).sqrt();
        let qkhm2 = qkhm1.powf(2.0);
        let qkhm3 = qkhm1.powf(3.0);
        let qkhm4 = qkhm1.powf(4.0);
        let qkhm5 = qkhm1.powf(5.0);
        let qkhm6 = qkhm1.powf(6.0);

        // Difference to the central meridian
        let merid = ((zone.wrapping_sub(30)).wrapping_mul(6).wrapping_sub(3)) as f64;
        let dlaenge1 = (east - Self::UTM_FALSE_EASTING) / Self::UTM_FAKTOR;
        let dlaenge2 = dlaenge1.powf(2.0);
        let dlaenge3 = dlaenge1.powf(3.0);
        let dlaenge4 = dlaenge1.powf(4.0);
        let dlaenge5 = dlaenge1.powf(5.0);
        let dlaenge6 = dlaenge1.powf(6.0);

        // Latitude factors
        let bfakt2 = -tangens1 * (1.0 + eta) / (2.0 * qkhm2);
        let bfakt4 = tangens1 * (5.0 + 3.0 * tangens2 + 6.0 * eta * (1.0 - tangens2)) / (24.0 * qkhm4);
        let bfakt6 = -tangens1 * (61.0 + 90.0 * tangens2 + 45.0 * tangens4) / (720.0 * qkhm6);

        // Longitude factors
        let lfakt1 = 1.0 / (qkhm1 * cosinus1);
        let lfakt3 = -(1.0 + 2.0 * tangens2 + eta) / (6.0 * qkhm3 * cosinus1);
        let lfakt5 = (5.0 + 28.0 * tangens2 + 24.0 * tangens4) / (120.0 * qkhm5 * cosinus1);

        // Geographic latitude/longitude WGS84
        let breite = fbreite + (180.0 / PI) * (bfakt2 * dlaenge2 + bfakt4 * dlaenge4 + bfakt6 * dlaenge6);
        let laenge = merid + (180.0 / PI) * (lfakt1 * dlaenge1 + lfakt3 * dlaenge3 + lfakt5 * dlaenge5);

        Ok(Geographic::new(laenge, breite))
    }
}
