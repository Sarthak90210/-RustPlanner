//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/GDirections.cs`
//!
//! Null C# strings are `None`; string concatenation treats them as empty, as `+` does.

#![allow(non_snake_case)]

use super::pointlatlng::PointLatLng;

/// `public class GDirections`
#[derive(Debug, Clone, Default)]
pub struct GDirections {
    /// a short textual description for the route
    pub Summary: Option<String>,
    /// a human-readable representation of the duration
    pub Duration: Option<String>,
    /// a value of the duration
    pub DurationValue: u32,
    /// a human-readable representation of the distance
    pub Distance: Option<String>,
    /// a value of the distance
    pub DistanceValue: u32,
    /// the latitude/longitude coordinates of the origin of this leg
    pub StartLocation: PointLatLng,
    /// the latitude/longitude coordinates of the given destination of this leg
    pub EndLocation: PointLatLng,
    /// the human-readable address reflecting the start_location of this leg
    pub StartAddress: Option<String>,
    /// the human-readable address reflecting the end_location of this leg
    pub EndAddress: Option<String>,
    /// the copyrights text to be displayed for this route
    pub Copyrights: Option<String>,
    /// the steps of the leg of the journey
    pub Steps: Option<Vec<GDirectionStep>>,
    /// all points of the route
    pub Route: Option<Vec<PointLatLng>>,
}

fn s(v: &Option<String>) -> &str {
    v.as_deref().unwrap_or("")
}

impl std::fmt::Display for GDirections {
    /// `Summary + " | " + Distance + " | " + Duration`
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} | {} | {}", s(&self.Summary), s(&self.Distance), s(&self.Duration))
    }
}

/// `public struct GDirectionStep`
#[derive(Debug, Clone, Default)]
pub struct GDirectionStep {
    pub TravelMode: Option<String>,
    /// the location of the starting point of this step
    pub StartLocation: PointLatLng,
    /// the location of the ending point of this step
    pub EndLocation: PointLatLng,
    /// the typical time required to perform the step
    pub Duration: Option<String>,
    /// the distance covered by this step until the next step
    pub Distance: Option<String>,
    /// formatted instructions for this step, as an HTML text string
    pub HtmlInstructions: Option<String>,
    /// points of the step
    pub Points: Option<Vec<PointLatLng>>,
}

impl std::fmt::Display for GDirectionStep {
    /// `TravelMode + " | " + Distance + " | " + Duration + " | " + HtmlInstructions`
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} | {} | {} | {}", s(&self.TravelMode), s(&self.Distance), s(&self.Duration), s(&self.HtmlInstructions))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn to_string_with_nulls() {
        let mut d = GDirections::default();
        assert_eq!(d.to_string(), " |  | ");
        d.Summary = Some("A1".into());
        d.Distance = Some("5 km".into());
        assert_eq!(d.to_string(), "A1 | 5 km | ");
        let st = GDirectionStep { TravelMode: Some("DRIVING".into()), ..Default::default() };
        assert_eq!(st.to_string(), "DRIVING |  |  | ");
    }
}
