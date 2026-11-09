//! Port of `ExtLibs/GeoUtility/Transformation/UTMMGR.cs`

use super::Transformation;
use crate::dotnet::{cast_i32, double_parse, index_out_of_range, int_parse, pad_left, substring, Exception};
use crate::extlibs::geoutility::errorprovider::{geo_exception, ErrorMessage};
use crate::extlibs::geoutility::geosystem::{MGRS, UTM};

impl Transformation {
    /// `UTMMGR(UTM utm)`: civil UTM to military grid (MGRS/UTMREF) coordinates.
    pub(crate) fn utmmgr(&self, utm: &UTM) -> Result<MGRS, Exception> {
        let zone = utm.zone();
        let band = utm.band().to_string();
        let cz2 = band.chars().next().ok_or_else(index_out_of_range)?;
        let sep = [',', '.'];

        // The leading digits of East and North pick the 100 km square.
        let e = utm.east_string().split(sep).next().unwrap_or("").to_string();
        let east_plan = int_parse(&substring(&e, 0, 1)?)?; // first digit of East
        let n = utm.north_string().split(sep).next().unwrap_or("").to_string();
        let north_plan = int_parse(&substring(&n, 0, 2)?)?; // first and second digit of North

        // East
        let mut east = cast_i32(utm.east().round_ties_even()).to_string();
        if east.len() > 2 {
            east.remove(0);
        }

        // North
        let mut north = cast_i32(utm.north().round_ties_even()).to_string();
        if north.len() > 5 {
            north = north[north.len() - 5..].to_string();
        }

        // East and North need the same number of digits
        if east.len() < north.len() {
            east = pad_left(&east, north.len(), '0');
        } else if north.len() < east.len() {
            north = pad_left(&north, east.len(), '0');
        }

        if zone < Self::MIN_ZONE || zone > Self::MAX_ZONE || cz2 < Self::MIN_BAND || cz2 > Self::MAX_BAND {
            return Err(geo_exception(ErrorMessage::new("ERROR_UTM_ZONE")));
        }

        // Index of the east letter of the square
        let mut eastgrid = 0i32;
        let mut i = zone % 3;
        if i == 1 {
            eastgrid = east_plan.wrapping_sub(1);
        }
        if i == 2 {
            eastgrid = east_plan.wrapping_add(7);
        }
        if i == 0 {
            eastgrid = east_plan.wrapping_add(15);
        }

        // Index of the north letter of the square
        let mut northgrid = 0i32;
        i = zone % 2;
        if i != 1 {
            northgrid = 5;
        }
        i = north_plan;
        while i - 20 >= 0 {
            i -= 20;
        }
        northgrid += i;
        if northgrid > 19 {
            northgrid -= 20;
        }

        // Square letters from the two indices
        let plan = substring(Self::MGRS_EAST, eastgrid, 1)? + &substring(Self::MGRS_NORTH1, northgrid, 1)?;

        MGRS::new(utm.zone(), utm.band(), &plan, double_parse(&east)?, double_parse(&north)?)
    }
}
