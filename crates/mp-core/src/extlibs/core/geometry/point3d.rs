//! Port of `ExtLibs/Core/Geometry/Point3D.cs`

use crate::dotnet::convert::to_double_str;
use crate::dotnet::{double_to_string_culture, get_culture_info, Exception};
use crate::extlibs::core::utils::stringutils::{list_get, StringUtils};

/// `public class Point3D`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Point3D {
    m_x: f64,
    m_y: f64,
    m_z: f64,
}

pub(crate) fn en_us(x: f64) -> String {
    double_to_string_culture(x, get_culture_info("en-US").expect("en-US culture"))
}

impl Point3D {
    /// `new Point3D(double x, double y, double z)`
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Point3D { m_x: x, m_y: y, m_z: z }
    }

    /// `new Point3D(float x, float y, float z)`
    pub fn new_f32(x: f32, y: f32, z: f32) -> Self {
        Point3D { m_x: x as f64, m_y: y as f64, m_z: z as f64 }
    }

    /// `new Point3D(Point3D xyz)`
    pub fn from_point(xyz: &Point3D) -> Self {
        xyz.clone()
    }

    /// `new Point3D(double x, double y)`
    pub fn new_xy(x: f64, y: f64) -> Self {
        Point3D { m_x: x, m_y: y, m_z: 0.0 }
    }

    /// `X`
    pub fn x(&self) -> f64 {
        self.m_x
    }

    /// `X` setter
    pub fn set_x(&mut self, v: f64) {
        self.m_x = v;
    }

    /// `Y`
    pub fn y(&self) -> f64 {
        self.m_y
    }

    /// `Y` setter
    pub fn set_y(&mut self, v: f64) {
        self.m_y = v;
    }

    /// `Z`
    pub fn z(&self) -> f64 {
        self.m_z
    }

    /// `Z` setter
    pub fn set_z(&mut self, v: f64) {
        self.m_z = v;
    }

    /// `Serialize()`: en-US numbers.
    pub fn serialize(&self) -> String {
        format!("{},{},{}", en_us(self.m_x), en_us(self.m_y), en_us(self.m_z))
    }

    /// `Deserialize(string str)`: parses in the current culture (not en-US as `Serialize`
    /// writes); fields already parsed stay set when a later one fails.
    pub fn deserialize(&mut self, str: Option<&str>) -> Result<(), Exception> {
        let bits = StringUtils::split_to_list(str, ",")?;
        self.m_x = to_double_str(Some(list_get(&bits, 0)?))?;
        self.m_y = to_double_str(Some(list_get(&bits, 1)?))?;
        self.m_z = to_double_str(Some(list_get(&bits, 2)?))?;
        Ok(())
    }

    /// `MakePointFromStr(string str)`
    pub fn make_point_from_str(str: Option<&str>) -> Result<Point3D, Exception> {
        let mut ans = Point3D::default();
        ans.deserialize(str)?;
        Ok(ans)
    }

    /// `SameCoordsAs(Point3D point3D)`
    pub fn same_coords_as(&self, p: &Point3D) -> bool {
        self.m_x == p.m_x && self.m_y == p.m_y && self.m_z == p.m_z
    }
}

impl std::fmt::Display for Point3D {
    /// `ToString()`
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[Point3D: {}]", self.serialize())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serialize_round_trip() {
        let p = Point3D::new(1.5, -2.0, 0.25);
        assert_eq!(p.serialize(), "1.5,-2,0.25");
        assert_eq!(p.to_string(), "[Point3D: 1.5,-2,0.25]");
        let mut q = Point3D::new_xy(9.0, 9.0);
        assert!(q.deserialize(Some("1,2")).is_err());
        // x and y were already assigned before z failed
        assert_eq!((q.x(), q.y(), q.z()), (1.0, 2.0, 0.0));
        assert!(Point3D::make_point_from_str(Some("1,2,3")).unwrap().same_coords_as(&Point3D::new_f32(1.0, 2.0, 3.0)));
    }
}
