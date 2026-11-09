//! Port of `ExtLibs/GeoUtility/Transformation/MGRUTM.cs`

use super::Transformation;
use crate::dotnet::{double_parse, index_out_of_range, pad_right, substring, Exception};
use crate::extlibs::geoutility::errorprovider::{geo_exception, ErrorMessage};
use crate::extlibs::geoutility::geosystem::{MGRS, UTM};

/// `string.IndexOf(string)` for the single-letter lookups here (-1 when absent; an empty
/// needle is found at 0).
fn index_of(haystack: &str, needle: &str) -> i32 {
    haystack.find(needle).map(|i| i as i32).unwrap_or(-1)
}

impl Transformation {
    /// `MGRUTM(MGRS mgrs)`: military grid (MGRS/UTMREF) to UTM coordinates.
    pub(crate) fn mgrutm(&self, mgrs: &MGRS) -> Result<UTM, Exception> {
        let zone = mgrs.zone();
        let eastgrid = substring(mgrs.grid(), 0, 1)?;
        let northgrid = substring(mgrs.grid(), 1, 1)?;

        // Leading East digit from the square letter
        let mut i = mgrs.zone() % 3;
        let mut stellenwert = 0i32;
        if i == 0 {
            stellenwert = index_of(Self::MGRS_EAST0, &eastgrid) + 1;
        }
        if i == 1 {
            stellenwert = index_of(Self::MGRS_EAST1, &eastgrid) + 1;
        }
        if i == 2 {
            stellenwert = index_of(Self::MGRS_EAST2, &eastgrid) + 1;
        }
        let s_east = stellenwert.to_string() + &pad_right(mgrs.east_string(), 5, '0');

        // Leading North digits from the square letter
        i = mgrs.zone() % 2;
        stellenwert = if i == 0 { index_of(Self::MGRS_NORTH0, &northgrid) } else { index_of(Self::MGRS_NORTH1, &northgrid) };

        // Leading North digits from the band
        let band = mgrs.band().chars().next().ok_or_else(index_out_of_range)?;

        if zone < Self::MIN_ZONE || zone > Self::MAX_ZONE || band < Self::MIN_BAND || band > Self::MAX_BAND {
            return Err(geo_exception(ErrorMessage::new("ERROR_UTM_ZONE")));
        }

        if band >= 'N' {
            if (band == 'Q') && (stellenwert < 10) {
                stellenwert += 20;
            }
            if band >= 'R' {
                stellenwert += 20;
            }
            if (band == 'S') && (stellenwert < 30) {
                stellenwert += 20;
            }
            if band >= 'T' {
                stellenwert += 20;
            }
            if (band == 'U') && (stellenwert < 50) {
                stellenwert += 20;
            }
            if band >= 'V' {
                stellenwert += 20;
            }
            if (band == 'W') && (stellenwert < 70) {
                stellenwert += 20;
            }
            if band >= 'X' {
                stellenwert += 20;
            }
        } else {
            if (band == 'C') && (stellenwert < 10) {
                stellenwert += 20;
            }
            if band >= 'D' {
                stellenwert += 20;
            }
            if (band == 'F') && (stellenwert < 30) {
                stellenwert += 20;
            }
            if band >= 'G' {
                stellenwert += 20;
            }
            if (band == 'H') && (stellenwert < 50) {
                stellenwert += 20;
            }
            if band >= 'J' {
                stellenwert += 20;
            }
            if (band == 'K') && (stellenwert < 70) {
                stellenwert += 20;
            }
            if band >= 'L' {
                stellenwert += 20;
            }
        }

        // North
        let mut s_north = String::new();
        if stellenwert.to_string().len() == 1 {
            s_north = "0".into();
        }
        s_north += &(stellenwert.to_string() + &pad_right(mgrs.north_string(), 5, '0'));

        UTM::with_band(zone, &band.to_string(), double_parse(&s_east)?, double_parse(&s_north)?)
    }
}
