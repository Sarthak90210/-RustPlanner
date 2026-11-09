//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/MapRoute.cs`
//!
//! The `ISerializable` members (binary serialization of a route) are not carried: nothing in
//! Mission Planner serializes routes.

#![allow(non_snake_case)]

use super::pointlatlng::PointLatLng;
use super::pureprojection::PureProjection;
use crate::extlibs::gmap_net_core::gmap_net_projections::MercatorProjection;

/// `public class MapRoute`: represents route of map
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MapRoute {
    /// points of route
    pub Points: Vec<PointLatLng>,
    /// route info
    pub Name: Option<String>,
    /// custom object
    pub Tag: Option<String>,
}

impl MapRoute {
    /// `new MapRoute(name)`
    pub fn new(name: &str) -> Self {
        MapRoute { Points: Vec::new(), Name: Some(name.to_string()), Tag: None }
    }

    /// `new MapRoute(points, name)`
    pub fn with_points(points: impl IntoIterator<Item = PointLatLng>, name: &str) -> Self {
        MapRoute { Points: points.into_iter().collect(), Name: Some(name.to_string()), Tag: None }
    }

    /// route start point
    pub fn From(&self) -> Option<PointLatLng> {
        self.Points.first().copied()
    }

    /// route end point (needs two points)
    pub fn To(&self) -> Option<PointLatLng> {
        if self.Points.len() > 1 {
            return self.Points.last().copied();
        }
        None
    }

    /// route distance (in km), through `GMapProviders.EmptyProvider.Projection` (Mercator)
    pub fn Distance(&self) -> f64 {
        let mut distance = 0.0;
        if self.From().is_some() && self.To().is_some() {
            let p = MercatorProjection::Instance();
            for i in 1..self.Points.len() {
                distance += p.GetDistance(self.Points[i - 1], self.Points[i]);
            }
        }
        distance
    }

    /// clears points and sets tag and name to null
    pub fn Clear(&mut self) {
        self.Points.clear();
        self.Tag = None;
        self.Name = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_needs_two_points() {
        let mut r = MapRoute::new("r");
        r.Points.push(PointLatLng::new(0.0, 0.0));
        assert_eq!(r.To(), None);
        assert_eq!(r.Distance(), 0.0);
        r.Points.push(PointLatLng::new(0.0, 1.0));
        assert!((r.Distance() - 111.319_490_793).abs() < 1e-6);
        r.Clear();
        assert!(r.Points.is_empty() && r.Name.is_none());
    }
}
