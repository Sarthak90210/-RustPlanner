//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/GeocodingProvider.cs`
//!
//! Each `out` parameter is the second element of the returned tuple; a null `out` list is
//! `None`.

#![allow(non_snake_case)]

use super::placemark::Placemark;
use super::pointlatlng::PointLatLng;
use super::statuscodes::GeoCoderStatusCode;
use crate::dotnet::Exception;

/// `public interface GeocodingProvider`: geocoding interface
pub trait GeocodingProvider {
    /// `GeoCoderStatusCode GetPoints(string keywords, out List<PointLatLng> pointList)`
    fn GetPoints(&self, keywords: &str) -> Result<(GeoCoderStatusCode, Option<Vec<PointLatLng>>), Exception>;

    /// `PointLatLng? GetPoint(string keywords, out GeoCoderStatusCode status)`
    fn GetPoint(&self, keywords: &str) -> Result<(Option<PointLatLng>, GeoCoderStatusCode), Exception>;

    /// `GeoCoderStatusCode GetPoints(Placemark placemark, out List<PointLatLng> pointList)`
    fn GetPointsByPlacemark(&self, placemark: &Placemark) -> Result<(GeoCoderStatusCode, Option<Vec<PointLatLng>>), Exception>;

    /// `PointLatLng? GetPoint(Placemark placemark, out GeoCoderStatusCode status)`
    fn GetPointByPlacemark(&self, placemark: &Placemark) -> Result<(Option<PointLatLng>, GeoCoderStatusCode), Exception>;

    /// `GeoCoderStatusCode GetPlacemarks(PointLatLng location, out List<Placemark> placemarkList)`
    fn GetPlacemarks(&self, location: PointLatLng) -> Result<(GeoCoderStatusCode, Option<Vec<Placemark>>), Exception>;

    /// `Placemark? GetPlacemark(PointLatLng location, out GeoCoderStatusCode status)`
    fn GetPlacemark(&self, location: PointLatLng) -> Result<(Option<Placemark>, GeoCoderStatusCode), Exception>;
}
