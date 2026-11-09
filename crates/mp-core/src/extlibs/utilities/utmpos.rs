//! Port of `ExtLibs/Utilities/utmpos.cs`

use std::any::Any;
use std::sync::Arc;

use super::math::MathHelper;
use super::pointlatlngalt::PointLatLngAlt;
use crate::dotnet::{double_get_hash_code, double_to_string, Exception};
use crate::extlibs::geoutility::geosystem::base::Hemisphere;
use crate::extlibs::geoutility::geosystem::UTM;
use crate::extlibs::projnet::coordinatesystems::projectedcoordinatesystem::ProjectedCoordinateSystem;
use crate::extlibs::projnet::coordinatesystems::transformations::coordinatetransformationfactory;
use crate::extlibs::projnet::coordinatesystems::transformations::mathtransform::MathTransform;

/// `object.ToString()` of a `Tag`: the type's text form. `Tag` is assigned by callers; the
/// values Mission Planner stores are strings and numbers, which carry their own ToString.
pub trait TagToString: Any + Send + Sync {
    fn to_cs_string(&self) -> String;
}

impl TagToString for String {
    fn to_cs_string(&self) -> String {
        self.clone()
    }
}

impl TagToString for i32 {
    fn to_cs_string(&self) -> String {
        self.to_string()
    }
}

impl TagToString for f64 {
    fn to_cs_string(&self) -> String {
        double_to_string(*self)
    }
}

/// `Math.Abs(int)`: throws for `int.MinValue`.
fn math_abs_i32(v: i32) -> Result<i32, Exception> {
    v.checked_abs().ok_or_else(|| Exception::Overflow("Negating the minimum value of a twos complement number is invalid.".into()))
}

/// `public struct utmpos`: a UTM position; a negative zone is the southern hemisphere.
#[allow(non_camel_case_types)]
#[derive(Clone, Default)]
pub struct utmpos {
    pub x: f64,
    pub y: f64,
    pub zone: i32,
    /// `public object Tag`
    pub tag: Option<Arc<dyn TagToString>>,
}

impl std::fmt::Debug for utmpos {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("utmpos")
            .field("x", &self.x)
            .field("y", &self.y)
            .field("zone", &self.zone)
            .field("Tag", &self.tag.as_ref().map(|t| t.to_cs_string()))
            .finish()
    }
}

impl utmpos {
    /// `utmpos.Zero`
    pub fn zero() -> utmpos {
        utmpos::default()
    }

    /// `new utmpos(double x, double y, int zone)`
    pub fn new(x: f64, y: f64, zone: i32) -> Self {
        utmpos { x, y, zone, tag: None }
    }

    /// `new utmpos(utmpos pos)`: the tag is not copied.
    pub fn copy_of(pos: &utmpos) -> Self {
        utmpos { x: pos.x, y: pos.y, zone: pos.zone, tag: None }
    }

    /// `new utmpos(PointLatLngAlt pos)`: in the point's own zone.
    pub fn from_plla(pos: &PointLatLngAlt) -> Result<Self, Exception> {
        let dd = pos.to_utm()?;
        Ok(utmpos { x: dd[0], y: dd[1], zone: pos.get_utm_zone(), tag: None })
    }

    /// `implicit operator double[](utmpos a)`: `{ x, y }`.
    pub fn to_double_array(&self) -> [f64; 2] {
        [self.x, self.y]
    }

    /// `ToLLA2()`: through GeoUtility.
    pub fn to_lla2(&self) -> Result<PointLatLngAlt, Exception> {
        let utm = UTM::with_hemisphere(
            math_abs_i32(self.zone)?,
            self.x,
            self.y,
            if self.zone < 0 { Hemisphere::South } else { Hemisphere::North },
        )?;

        let mut ans: PointLatLngAlt = utm.to_geographic()?.into();
        if let Some(t) = &self.tag {
            ans.Tag = t.to_cs_string();
        }

        Ok(ans)
    }

    /// `ToLLA()` (and `implicit operator PointLatLngAlt(utmpos a)`): through ProjNet's inverse
    /// transverse Mercator. A new transformation is built on every call, as in C#.
    pub fn to_lla(&self) -> Result<PointLatLngAlt, Exception> {
        let utm = ProjectedCoordinateSystem::wgs84_utm(math_abs_i32(self.zone)?, !(self.zone < 0));

        let trans = coordinatetransformationfactory::create_from_wgs84_to_projected(&utm)?;

        // get leader utm coords
        let pll = trans.inverse()?.transform(&self.to_double_array())?;

        let mut ans = PointLatLngAlt::new(pll[1], pll[0]);
        if let Some(t) = &self.tag {
            ans.Tag = t.to_cs_string();
        }

        Ok(ans)
    }

    /// `ToList(List<double[]> input, int zone)`: each array's first two values. A shorter
    /// array throws.
    pub fn to_list(input: &[Vec<f64>], zone: i32) -> Result<Vec<utmpos>, Exception> {
        let mut data = vec![];
        for x in input {
            if x.len() < 2 {
                return Err(crate::dotnet::index_out_of_range());
            }
            data.push(utmpos::new(x[0], x[1], zone));
        }
        Ok(data)
    }

    /// `GetDistance(utmpos b)`: planar distance in meters.
    pub fn get_distance(&self, b: &utmpos) -> f64 {
        ((self.x - b.x).abs().powf(2.0) + (self.y - b.y).abs().powf(2.0)).sqrt()
    }

    /// `GetBearing(utmpos b)`: grid bearing in degrees, 0..360.
    pub fn get_bearing(&self, b: &utmpos) -> f64 {
        let y = b.y - self.y;
        let x = b.x - self.x;

        (MathHelper::RAD2DEG * (x.atan2(y)) + 360.0) % 360.0
    }

    /// `Equals(object obj)`: x and y only (the zone is ignored, unlike `==`).
    pub fn equals(&self, obj: &utmpos) -> bool {
        (obj.x == self.x) && (obj.y == self.y)
    }

    /// `GetHashCode()`
    pub fn get_hash_code(&self) -> i32 {
        let mut hash_code = double_get_hash_code(self.x);
        hash_code = hash_code.wrapping_mul(397) ^ double_get_hash_code(self.y);
        hash_code = hash_code.wrapping_mul(397) ^ self.zone;
        hash_code
    }

    /// `IsZero`: `this == Zero`.
    pub fn is_zero(&self) -> bool {
        *self == utmpos::zero()
    }
}

impl PartialEq for utmpos {
    /// `operator ==`: x, y and zone.
    fn eq(&self, right: &utmpos) -> bool {
        (self.x == right.x) && (self.y == right.y) && (self.zone == right.zone)
    }
}

impl std::fmt::Display for utmpos {
    /// `ToString()`: "utmpos: x,y" in the current culture.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "utmpos: {},{}", double_to_string(self.x), double_to_string(self.y))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_both_ways() {
        let p = PointLatLngAlt::new(-35.36, 149.16);
        let u = utmpos::from_plla(&p).unwrap();
        assert_eq!(u.zone, -55);
        let back = u.to_lla().unwrap();
        assert!((back.Lat - p.Lat).abs() < 1e-7 && (back.Lng - p.Lng).abs() < 1e-7, "{back}");
        let back2 = u.to_lla2().unwrap();
        assert!((back2.Lat - p.Lat).abs() < 1e-5, "{back2}");
        assert!(utmpos::zero().is_zero());
        assert_eq!(utmpos::new(0.0, 0.0, 0).get_bearing(&utmpos::new(1.0, 0.0, 0)), 90.0);
    }
}
