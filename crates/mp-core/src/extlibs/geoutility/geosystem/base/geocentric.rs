//! Port of `ExtLibs/GeoUtility/GeoSystem/Base/Geocentric.cs`

use super::basesystem::TRANSFORM;
use crate::dotnet::{index_out_of_range, math_round, to_upper, Exception};
use crate::extlibs::geoutility::geosystem::{mgrs, utm, UTM};

/// Band selection
const UTM_BAND: &str = "CDEFGHJKLMNPQRSTUVWXX";

/// `public enum Hemisphere`, used by some constructors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Hemisphere {
    North,
    South,
}

/// `public enum Target`: which value `Format` formats.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Target {
    All,
    East,
    North,
    /// Can be used for MGRS.
    Nothing,
}

/// Which derived class the object is, standing in for the virtual `Format` override.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum GeocentricKind {
    Utm,
    Mgrs,
}

/// `public abstract class Geocentric : BaseSystem`: the members shared by UTM and MGRS.
#[derive(Debug, Clone)]
pub struct Geocentric {
    zone: i32,
    band: String,
    north: f64,
    east: f64,
    northstring: String,
    eaststring: String,
    precision: i32,
    kind: GeocentricKind,
}

impl Geocentric {
    /// `protected Geocentric()`
    pub(crate) fn empty(kind: GeocentricKind) -> Self {
        Geocentric {
            zone: 0,
            band: String::new(),
            north: 0.0,
            east: 0.0,
            northstring: String::new(),
            eaststring: String::new(),
            precision: 0,
            kind,
        }
    }

    /// `protected Geocentric(int zone, string band, double east, double north)`
    pub(crate) fn with_band(kind: GeocentricKind, zone: i32, band: &str, east: f64, north: f64) -> Result<Self, Exception> {
        let mut g = Self::empty(kind);
        g.set_east(east);
        g.set_north(north);
        g.set_zone(zone);
        g.set_band(band)?;
        Ok(g)
    }

    /// `protected Geocentric(int zone, double east, double north, Hemisphere hem)`: the band is
    /// worked out by converting to geographic coordinates and back.
    pub(crate) fn with_hemisphere(kind: GeocentricKind, zone: i32, east: f64, north: f64, hem: Hemisphere) -> Result<Self, Exception> {
        let mut band = "N"; // run the transformation on the northern hemisphere
        if hem == Hemisphere::South {
            band = "A"; // run the transformation on the southern hemisphere
        }
        let mut geo = TRANSFORM.utmwgs(&UTM::with_band(zone, band, east, north)?)?;
        geo.set_longitude(math_round(geo.longitude(), 4)); // without rounding the band could come out wrong
        geo.set_latitude(math_round(geo.latitude(), 4)); // without rounding the band could come out wrong
        let utm = TRANSFORM.wgsutm(&geo)?; // second transformation to get the band

        let mut g = Self::empty(kind);
        g.set_east(east);
        g.set_north(north);
        g.set_zone(zone);
        g.set_band(utm.band())?;
        Ok(g)
    }

    /// `East`
    pub fn east(&self) -> f64 {
        self.east
    }

    pub fn set_east(&mut self, value: f64) {
        self.east = value;
        self.format(Target::East);
    }

    /// `North`
    pub fn north(&self) -> f64 {
        self.north
    }

    pub fn set_north(&mut self, value: f64) {
        self.north = value;
        self.format(Target::North);
    }

    /// `Zone`
    pub fn zone(&self) -> i32 {
        self.zone
    }

    pub fn set_zone(&mut self, value: i32) {
        self.zone = value;
    }

    /// `Band`
    pub fn band(&self) -> &str {
        &self.band
    }

    /// `Band` setter: upper-cased in the current culture, then clamped to C..X. An empty
    /// value throws `IndexOutOfRangeException` (`ToCharArray()[0]`).
    pub fn set_band(&mut self, value: &str) -> Result<(), Exception> {
        let mut band = to_upper(value);
        let cband = band.chars().next().ok_or_else(index_out_of_range)?;
        let lband = UTM_BAND.chars().next().unwrap_or('C');
        let uband = UTM_BAND.chars().last().unwrap_or('X');

        if cband < lband {
            band = lband.to_string();
        } else if cband > uband {
            band = uband.to_string();
        }
        self.band = band;
        Ok(())
    }

    /// `Zoneband`: e.g. "32U".
    pub fn zoneband(&self) -> String {
        self.zone.to_string() + &self.band
    }

    /// `EastString`: only used for output.
    pub fn east_string(&self) -> &str {
        &self.eaststring
    }

    pub fn set_east_string(&mut self, value: &str) {
        self.eaststring = value.to_string();
    }

    /// `NorthString`: only used for output.
    pub fn north_string(&self) -> &str {
        &self.northstring
    }

    pub fn set_north_string(&mut self, value: &str) {
        self.northstring = value.to_string();
    }

    /// `Precision`: decimals in the string output.
    pub fn precision(&self) -> i32 {
        self.precision
    }

    pub fn set_precision(&mut self, value: i32) {
        self.precision = value;
        self.format(Target::All);
    }

    /// `internal abstract void Format(Target target)`, dispatched to the derived class.
    pub(crate) fn format(&mut self, target: Target) {
        match self.kind {
            GeocentricKind::Utm => utm::format(self, target),
            GeocentricKind::Mgrs => mgrs::format(self, target),
        }
    }
}
