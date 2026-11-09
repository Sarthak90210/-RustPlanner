//! Port of `ExtLibs/GeoUtility/GeoSystem/Helper/GeoDatum.cs`

/// `public enum GeoDatum`: WGS84 (international) or Potsdam (Germany only).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum GeoDatum {
    #[default]
    WGS84 = 0,
    Potsdam = 1,
}

impl std::fmt::Display for GeoDatum {
    /// `Enum.ToString()`
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            GeoDatum::WGS84 => "WGS84",
            GeoDatum::Potsdam => "Potsdam",
        })
    }
}
