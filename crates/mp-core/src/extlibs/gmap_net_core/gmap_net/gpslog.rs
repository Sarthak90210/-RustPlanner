//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/GpsLog.cs`

#![allow(non_snake_case)]

use super::pointlatlng::PointLatLng;
use crate::dotnet::DateTime;

/// `public struct GpsLog`: one GPS log record. `default()` is C#'s `default(GpsLog)`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GpsLog {
    pub TimeUTC: DateTime,
    pub SessionCounter: i64,
    pub Delta: Option<f64>,
    pub Speed: Option<f64>,
    pub SeaLevelAltitude: Option<f64>,
    pub EllipsoidAltitude: Option<f64>,
    pub SatellitesInView: Option<i16>,
    pub SatelliteCount: Option<i16>,
    pub Position: PointLatLng,
    pub PositionDilutionOfPrecision: Option<f64>,
    pub HorizontalDilutionOfPrecision: Option<f64>,
    pub VerticalDilutionOfPrecision: Option<f64>,
    pub FixQuality: FixQuality,
    pub FixType: FixType,
    pub FixSelection: FixSelection,
}

crate::cs_enum! {
    /// `public enum FixQuality : int`
    pub FixQuality: i32 {
        Unknown = 0,
        Gps = 1,
        DGps = 2,
    }
}

crate::cs_enum! {
    /// `public enum FixType : int`
    pub FixType: i32 {
        Unknown = 0,
        XyD = 1,
        XyzD = 2,
    }
}

crate::cs_enum! {
    /// `public enum FixSelection : int`
    pub FixSelection: i32 {
        Unknown = 0,
        Auto = 1,
        Manual = 2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_record() {
        let g = GpsLog::default();
        assert_eq!(g.TimeUTC.ticks, 0);
        assert!(g.Speed.is_none() && g.SatelliteCount.is_none());
        assert!(g.Position.is_empty());
        assert_eq!(g.FixQuality, FixQuality::Unknown);
        assert_eq!(FixType::XyzD.to_string(), "XyzD");
        assert_eq!(FixSelection(1), FixSelection::Auto);
    }
}
