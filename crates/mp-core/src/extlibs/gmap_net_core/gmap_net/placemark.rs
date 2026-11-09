//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/Placemark.cs`

#![allow(non_snake_case)]

/// `public struct Placemark`: represents place info. `default()` is C#'s
/// `default(Placemark)` (every string null); `new` is the internal constructor that sets
/// them to `string.Empty`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Placemark {
    address: Option<String>,
    /// the accuracy of address
    pub Accuracy: i32,
    // parsed values from address
    pub ThoroughfareName: Option<String>,
    pub LocalityName: Option<String>,
    pub PostalCodeNumber: Option<String>,
    pub CountryName: Option<String>,
    pub AdministrativeAreaName: Option<String>,
    pub DistrictName: Option<String>,
    pub SubAdministrativeAreaName: Option<String>,
    pub Neighborhood: Option<String>,
    pub StreetNumber: Option<String>,
    pub CountryNameCode: Option<String>,
    pub HouseNo: Option<String>,
}

impl Placemark {
    /// `internal Placemark(string address)`
    pub fn new(address: Option<&str>) -> Self {
        let e = || Some(String::new());
        Placemark {
            address: address.map(str::to_string),
            Accuracy: 0,
            HouseNo: e(),
            ThoroughfareName: e(),
            DistrictName: e(),
            LocalityName: e(),
            PostalCodeNumber: e(),
            CountryName: e(),
            CountryNameCode: e(),
            AdministrativeAreaName: e(),
            SubAdministrativeAreaName: e(),
            Neighborhood: e(),
            StreetNumber: e(),
        }
    }

    /// the address
    pub fn Address(&self) -> Option<&str> {
        self.address.as_deref()
    }

    /// `Address` internal setter
    pub fn set_Address(&mut self, value: Option<String>) {
        self.address = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constructor_sets_empty_strings() {
        let p = Placemark::new(Some("Vilnius"));
        assert_eq!(p.Address(), Some("Vilnius"));
        assert_eq!(p.HouseNo.as_deref(), Some(""));
        assert_eq!(p.Accuracy, 0);
        let d = Placemark::default();
        assert!(d.Address().is_none() && d.CountryName.is_none());
    }
}
