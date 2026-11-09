//! Port of `ExtLibs/Utilities/SingleLocation.cs`
//!
//! (namespace `MissionPlanner.GeoRef`)

use crate::dotnet::DateTime;

/// `public class SingleLocation : ICloneable`: one position/attitude sample for geotagging.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SingleLocation {
    time: DateTime,
    lat: f64,
    lon: f64,
    alt_amsl: f64,
    rel_alt: f64,
    gpsalt: f64,
    salt: f64,
    roll: f32,
    pitch: f32,
    yaw: f32,
}

macro_rules! prop {
    ($get:ident, $set:ident, $field:ident, $t:ty, $doc:literal) => {
        #[doc = $doc]
        pub fn $get(&self) -> $t {
            self.$field
        }
        pub fn $set(&mut self, value: $t) {
            self.$field = value;
        }
    };
}

impl SingleLocation {
    prop!(time, set_time, time, DateTime, "`Time`");
    prop!(lat, set_lat, lat, f64, "`Lat`");
    prop!(lon, set_lon, lon, f64, "`Lon`");
    prop!(alt_amsl, set_alt_amsl, alt_amsl, f64, "`AltAMSL`");
    prop!(rel_alt, set_rel_alt, rel_alt, f64, "`RelAlt`");
    prop!(gps_alt, set_gps_alt, gpsalt, f64, "`GPSAlt`");
    prop!(s_alt, set_s_alt, salt, f64, "`SAlt`");
    prop!(roll, set_roll, roll, f32, "`Roll`");
    prop!(pitch, set_pitch, pitch, f32, "`Pitch`");
    prop!(yaw, set_yaw, yaw, f32, "`Yaw`");

    /// `getAltitude(bool AMSL, bool gpsalt)`: GPS altitude wins, then AMSL or relative.
    pub fn get_altitude(&self, amsl: bool, gpsalt: bool) -> f64 {
        if gpsalt {
            return self.gps_alt();
        }
        if amsl {
            self.alt_amsl()
        } else {
            self.rel_alt()
        }
    }

    /// `Clone()`: `MemberwiseClone`.
    pub fn clone_object(&self) -> SingleLocation {
        *self
    }
}
