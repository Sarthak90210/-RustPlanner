//! Port of `ExtLibs/Utilities/Vector3.cs`
//!
//! `Vector3<T>` is generic over `float`/`double` and does all arithmetic in `double` through
//! `(T)(dynamic)` casts; `CsFloat` gives the same two conversions. `Vector3` and `Vector3f`
//! are the C# subclasses `Vector3 : Vector3<double>` and `Vector3f : Vector3<float>`.
//!
//! The C# types are classes, so `==` is reference equality there; Rust values have no
//! identity, and `PartialEq` here is the C# `Equals(Vector3<T>)`.

use serde::{Deserialize, Serialize};

use super::pointlatlngalt::PointLatLngAlt;
use crate::dotnet::{double_to_string, float_to_string, Exception};
use crate::extlibs::gmap_net_core::gmap_net::PointLatLng;

/// The `T` of `Vector3<T>` (`float` or `double`).
pub trait CsFloat: Copy + Default + std::fmt::Debug + PartialEq + 'static {
    /// `(double)(dynamic)x`
    fn to_f64(self) -> f64;
    /// `(T)(dynamic)d`
    fn from_f64(d: f64) -> Self;
    /// `T.Equals(T)`: NaN equals NaN.
    fn cs_equals(self, o: Self) -> bool;
    /// `T.ToString()` in the current culture.
    fn to_cs_string(self) -> String;
    /// The CLR name of the generic instantiation, for `object.ToString()`.
    const CLR_NAME: &'static str;
}

impl CsFloat for f64 {
    fn to_f64(self) -> f64 {
        self
    }
    fn from_f64(d: f64) -> Self {
        d
    }
    fn cs_equals(self, o: Self) -> bool {
        self == o || (self.is_nan() && o.is_nan())
    }
    fn to_cs_string(self) -> String {
        double_to_string(self)
    }
    const CLR_NAME: &'static str = "MissionPlanner.Utilities.Vector3`1[System.Double]";
}

impl CsFloat for f32 {
    fn to_f64(self) -> f64 {
        self as f64
    }
    fn from_f64(d: f64) -> Self {
        d as f32
    }
    fn cs_equals(self, o: Self) -> bool {
        self == o || (self.is_nan() && o.is_nan())
    }
    fn to_cs_string(self) -> String {
        float_to_string(self)
    }
    const CLR_NAME: &'static str = "MissionPlanner.Utilities.Vector3`1[System.Single]";
}

/// `public class Vector3<T> : IEquatable<Vector3<T>>, IFormattable where T : struct`
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Vector3T<T: CsFloat> {
    pub x: T,
    pub y: T,
    pub z: T,
}

/// `public class Vector3 : Vector3<double>`
pub type Vector3 = Vector3T<f64>;

/// `public class Vector3f : Vector3<float>`
pub type Vector3f = Vector3T<f32>;

impl Vector3 {
    /// `Vector3.Zero`
    pub const ZERO: Vector3 = Vector3T { x: 0.0, y: 0.0, z: 0.0 };
    /// `Vector3.One`
    pub const ONE: Vector3 = Vector3T { x: 1.0, y: 1.0, z: 1.0 };
}

impl Vector3f {
    /// `Vector3f.Zero`
    pub const ZERO: Vector3f = Vector3T { x: 0.0, y: 0.0, z: 0.0 };
    /// `Vector3f.One`
    pub const ONE: Vector3f = Vector3T { x: 1.0, y: 1.0, z: 1.0 };
}

/// `HALF_SQRT_2`
#[allow(clippy::approx_constant)] // Vector3.cs literal
const HALF_SQRT_2: f64 = 0.70710678118654757;

const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;
const DEG2RAD: f64 = 1.0 / RAD2DEG;

crate::cs_enum! {
    /// `public enum Rotation`, from libraries\AP_Math\rotations.h
    pub Rotation: i32 {
        ROTATION_NONE = 0,
        ROTATION_YAW_45 = 1,
        ROTATION_YAW_90 = 2,
        ROTATION_YAW_135 = 3,
        ROTATION_YAW_180 = 4,
        ROTATION_YAW_225 = 5,
        ROTATION_YAW_270 = 6,
        ROTATION_YAW_315 = 7,
        ROTATION_ROLL_180 = 8,
        ROTATION_ROLL_180_YAW_45 = 9,
        ROTATION_ROLL_180_YAW_90 = 10,
        ROTATION_ROLL_180_YAW_135 = 11,
        ROTATION_PITCH_180 = 12,
        ROTATION_ROLL_180_YAW_225 = 13,
        ROTATION_ROLL_180_YAW_270 = 14,
        ROTATION_ROLL_180_YAW_315 = 15,
        ROTATION_ROLL_90 = 16,
        ROTATION_ROLL_90_YAW_45 = 17,
        ROTATION_ROLL_90_YAW_90 = 18,
        ROTATION_ROLL_90_YAW_135 = 19,
        ROTATION_ROLL_270 = 20,
        ROTATION_ROLL_270_YAW_45 = 21,
        ROTATION_ROLL_270_YAW_90 = 22,
        ROTATION_ROLL_270_YAW_135 = 23,
        ROTATION_PITCH_90 = 24,
        ROTATION_PITCH_270 = 25,
        ROTATION_MAX = 26,
    }
}

impl<T: CsFloat> Vector3T<T> {
    /// `new Vector3<T>(T x, T y, T z)`
    pub fn new(x: T, y: T, z: T) -> Self {
        Vector3T { x, y, z }
    }

    /// `new Vector3<T>(Tuple<T, T, T> inp)`
    pub fn from_tuple(inp: (T, T, T)) -> Self {
        Vector3T { x: inp.0, y: inp.1, z: inp.2 }
    }

    /// `private Vector3(double x, double y, double z)`: each value through `(T)(dynamic)`.
    fn from_doubles(x: f64, y: f64, z: f64) -> Self {
        Vector3T { x: T::from_f64(x), y: T::from_f64(y), z: T::from_f64(z) }
    }

    /// `xd`
    pub fn xd(&self) -> f64 {
        self.x.to_f64()
    }

    /// `yd`
    pub fn yd(&self) -> f64 {
        self.y.to_f64()
    }

    /// `zd`
    pub fn zd(&self) -> f64 {
        self.z.to_f64()
    }

    /// `Equals(Vector3<T> other)`: component `Equals`, so NaN equals NaN.
    pub fn equals(&self, other: &Vector3T<T>) -> bool {
        self.x.cs_equals(other.x) && self.y.cs_equals(other.y) && self.z.cs_equals(other.z)
    }

    /// `object.ToString()`: `public new string ToString()` only hides it, so a call through
    /// `object` (string concatenation, `Convert.ToString`) gives the CLR type name.
    pub fn object_to_string(&self) -> &'static str {
        T::CLR_NAME
    }

    /// `implicit operator Vector3<T>(PointLatLngAlt a)`: (Lat, Lng, Alt).
    pub fn from_plla(a: &PointLatLngAlt) -> Self {
        Vector3T { x: T::from_f64(a.Lat), y: T::from_f64(a.Lng), z: T::from_f64(a.Alt) }
    }

    /// `implicit operator Vector3<T>(PointLatLng a)`: (Lat, Lng, 0).
    pub fn from_point_lat_lng(a: PointLatLng) -> Self {
        Vector3T { x: T::from_f64(a.lat()), y: T::from_f64(a.lng()), z: T::from_f64(0.0) }
    }

    /// `implicit operator PointLatLng(Vector3<T> a)`
    pub fn to_point_lat_lng(&self) -> PointLatLng {
        PointLatLng::new(self.xd(), self.yd())
    }

    /// `implicit operator T[](Vector3<T> a)`
    pub fn to_array(&self) -> [T; 3] {
        [self.x, self.y, self.z]
    }

    /// `implicit operator Vector3(Vector3<T> a)`
    pub fn to_vector3(&self) -> Vector3 {
        Vector3T { x: self.xd(), y: self.yd(), z: self.zd() }
    }

    /// `implicit operator Vector3f(Vector3<T> a)`
    pub fn to_vector3f(&self) -> Vector3f {
        Vector3T { x: self.xd() as f32, y: self.yd() as f32, z: self.zd() as f32 }
    }

    /// `this[int index]` getter: `Exception("Bad index")` outside 0..2.
    pub fn get(&self, index: i32) -> Result<T, Exception> {
        match index {
            0 => Ok(self.x),
            1 => Ok(self.y),
            2 => Ok(self.z),
            _ => Err(Exception::Other("Bad index".into())),
        }
    }

    /// `this[int index]` setter
    pub fn set(&mut self, index: i32, value: T) -> Result<(), Exception> {
        match index {
            0 => self.x = value,
            1 => self.y = value,
            2 => self.z = value,
            _ => return Err(Exception::Other("Bad index".into())),
        }
        Ok(())
    }

    /// `operator *(Vector3<T>, Vector3<T>)`: dot product.
    pub fn dot(&self, v: &Vector3T<T>) -> f64 {
        self.xd() * v.xd() + self.yd() * v.yd() + self.zd() * v.zd()
    }

    /// `operator %(Vector3<T>, Vector3<T>)`: cross product.
    pub fn cross(&self, v: &Vector3T<T>) -> Vector3T<T> {
        Self::from_doubles(
            self.yd() * v.zd() - self.zd() * v.yd(),
            self.zd() * v.xd() - self.xd() * v.zd(),
            self.xd() * v.yd() - self.yd() * v.xd(),
        )
    }

    /// `copy()`
    pub fn copy(&self) -> Vector3T<T> {
        Vector3T { x: self.x, y: self.y, z: self.z }
    }

    /// `length()`
    pub fn length(&self) -> T {
        T::from_f64((self.xd() * self.xd() + self.yd() * self.yd() + self.zd() * self.zd()).sqrt())
    }

    /// `zero()`
    pub fn zero(&mut self) {
        let z = T::from_f64(0.0);
        self.x = z;
        self.y = z;
        self.z = z;
    }

    /// `normalized()`
    pub fn normalized(&self) -> Vector3T<T> {
        *self / self.length().to_f64()
    }

    /// `normalize()`
    pub fn normalize(&mut self) {
        let v = self.normalized();
        self.x = v.x;
        self.y = v.y;
        self.z = v.z;
    }

    /// `rotate(double rotation)`: rotates x/y by `rotation` degrees in place and returns this.
    pub fn rotate_degrees(&mut self, mut rotation: f64) -> &mut Self {
        rotation *= DEG2RAD;
        let x1 = self.xd() * rotation.cos() - self.yd() * rotation.sin();
        let y1 = self.xd() * rotation.sin() + self.yd() * rotation.cos();

        self.x = T::from_f64(x1);
        self.y = T::from_f64(y1);

        self
    }

    /// `rotate(Rotation rotation)`: applies an AP_Math board rotation in place and returns
    /// this; an undefined value throws `Exception("Invalid Rotation")`. Each step reads the
    /// fields as they stand after the previous step, as the C# does.
    pub fn rotate(&mut self, rotation: Rotation) -> Result<&mut Self, Exception> {
        let h = HALF_SQRT_2;
        let f = T::from_f64;
        let tmp: T;
        match rotation {
            Rotation::ROTATION_NONE | Rotation::ROTATION_MAX => {}
            Rotation::ROTATION_YAW_45 => {
                tmp = f(h * (self.xd() - self.yd()));
                self.y = f(h * (self.xd() + self.yd()));
                self.x = tmp;
            }
            Rotation::ROTATION_YAW_90 => {
                tmp = self.x;
                self.x = f(-self.yd());
                self.y = tmp;
            }
            Rotation::ROTATION_YAW_135 => {
                tmp = f(-h * (self.xd() + self.yd()));
                self.y = f(h * (self.xd() - self.yd()));
                self.x = tmp;
            }
            Rotation::ROTATION_YAW_180 => {
                self.x = f(-self.xd());
                self.y = f(-self.yd());
            }
            Rotation::ROTATION_YAW_225 => {
                tmp = f(h * (self.yd() - self.xd()));
                self.y = f(-h * (self.xd() + self.yd()));
                self.x = tmp;
            }
            Rotation::ROTATION_YAW_270 => {
                tmp = self.x;
                self.x = self.y;
                self.y = f(-tmp.to_f64());
            }
            Rotation::ROTATION_YAW_315 => {
                tmp = f(h * (self.xd() + self.yd()));
                self.y = f(h * (self.yd() - self.xd()));
                self.x = tmp;
            }
            Rotation::ROTATION_ROLL_180 => {
                self.y = f(-self.yd());
                self.z = f(-self.zd());
            }
            Rotation::ROTATION_ROLL_180_YAW_45 => {
                tmp = f(h * (self.xd() + self.yd()));
                self.y = f(h * (self.xd() - self.yd()));
                self.x = tmp;
                self.z = f(-self.zd());
            }
            Rotation::ROTATION_ROLL_180_YAW_90 => {
                tmp = self.x;
                self.x = self.y;
                self.y = tmp;
                self.z = f(-self.zd());
            }
            Rotation::ROTATION_ROLL_180_YAW_135 => {
                tmp = f(h * (self.yd() - self.xd()));
                self.y = f(h * (self.yd() + self.xd()));
                self.x = tmp;
                self.z = f(-self.zd());
            }
            Rotation::ROTATION_PITCH_180 => {
                self.x = f(-self.xd());
                self.z = f(-self.zd());
            }
            Rotation::ROTATION_ROLL_180_YAW_225 => {
                tmp = f(-h * (self.xd() + self.yd()));
                self.y = f(h * (self.yd() - self.xd()));
                self.x = f(tmp.to_f64());
                self.z = f(-self.zd());
            }
            Rotation::ROTATION_ROLL_180_YAW_270 => {
                tmp = f(self.xd());
                self.x = f(-self.yd());
                self.y = f(-tmp.to_f64());
                self.z = f(-self.zd());
            }
            Rotation::ROTATION_ROLL_180_YAW_315 => {
                tmp = f(h * (self.xd() - self.yd()));
                self.y = f(-h * (self.xd() + self.yd()));
                self.x = f(tmp.to_f64());
                self.z = f(-self.zd());
            }
            Rotation::ROTATION_ROLL_90 => {
                tmp = f(self.zd());
                self.z = f(self.yd());
                self.y = f(-tmp.to_f64());
            }
            Rotation::ROTATION_ROLL_90_YAW_45 => {
                let t1 = f(self.zd());
                self.z = f(self.yd());
                self.y = f(-t1.to_f64());
                tmp = f(h * (self.xd() - self.yd()));
                self.y = f(h * (self.xd() + self.yd()));
                self.x = f(tmp.to_f64());
            }
            Rotation::ROTATION_ROLL_90_YAW_90 => {
                let t1 = f(self.zd());
                self.z = f(self.yd());
                self.y = f(-t1.to_f64());
                tmp = f(self.xd());
                self.x = f(-self.yd());
                self.y = f(tmp.to_f64());
            }
            Rotation::ROTATION_ROLL_90_YAW_135 => {
                let t1 = f(self.zd());
                self.z = f(self.yd());
                self.y = f(-t1.to_f64());
                tmp = f(-h * (self.xd() + self.yd()));
                self.y = f(h * (self.xd() - self.yd()));
                self.x = f(tmp.to_f64());
            }
            Rotation::ROTATION_ROLL_270 => {
                tmp = f(self.zd());
                self.z = f(-self.yd());
                self.y = f(tmp.to_f64());
            }
            Rotation::ROTATION_ROLL_270_YAW_45 => {
                let t1 = f(self.zd());
                self.z = f(-self.yd());
                self.y = f(t1.to_f64());
                tmp = f(h * (self.xd() - self.yd()));
                self.y = f(h * (self.xd() + self.yd()));
                self.x = f(tmp.to_f64());
            }
            Rotation::ROTATION_ROLL_270_YAW_90 => {
                let t1 = f(self.zd());
                self.z = f(-self.yd());
                self.y = f(t1.to_f64());
                tmp = f(self.xd());
                self.x = f(-self.yd());
                self.y = f(tmp.to_f64());
            }
            Rotation::ROTATION_ROLL_270_YAW_135 => {
                let t1 = f(self.zd());
                self.z = f(-self.yd());
                self.y = f(t1.to_f64());
                tmp = f(-h * (self.xd() + self.yd()));
                self.y = f(h * (self.xd() - self.yd()));
                self.x = f(tmp.to_f64());
            }
            Rotation::ROTATION_PITCH_90 => {
                tmp = f(self.zd());
                self.z = f(-self.xd());
                self.x = f(tmp.to_f64());
            }
            Rotation::ROTATION_PITCH_270 => {
                tmp = f(self.zd());
                self.z = f(self.xd());
                self.x = f(-tmp.to_f64());
            }
            _ => return Err(Exception::Other("Invalid Rotation".into())),
        }
        Ok(self)
    }
}

impl<T: CsFloat> PartialEq for Vector3T<T> {
    fn eq(&self, other: &Self) -> bool {
        self.equals(other)
    }
}

impl<T: CsFloat> std::fmt::Display for Vector3T<T> {
    /// `ToString()` / `IFormattable.ToString`: the literal text "Vector3<T>(x, y, z)".
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Vector3<T>({}, {}, {})", self.x.to_cs_string(), self.y.to_cs_string(), self.z.to_cs_string())
    }
}

impl<T: CsFloat> std::ops::Add for Vector3T<T> {
    type Output = Vector3T<T>;
    /// `operator +`
    fn add(self, v: Vector3T<T>) -> Vector3T<T> {
        Vector3T::from_doubles(self.xd() + v.xd(), self.yd() + v.yd(), self.zd() + v.zd())
    }
}

impl<T: CsFloat> std::ops::AddAssign for Vector3T<T> {
    fn add_assign(&mut self, v: Vector3T<T>) {
        *self = *self + v;
    }
}

impl<T: CsFloat> std::ops::Sub for Vector3T<T> {
    type Output = Vector3T<T>;
    /// `operator -(self, v)`
    fn sub(self, v: Vector3T<T>) -> Vector3T<T> {
        Vector3T::from_doubles(self.xd() - v.xd(), self.yd() - v.yd(), self.zd() - v.zd())
    }
}

impl<T: CsFloat> std::ops::SubAssign for Vector3T<T> {
    fn sub_assign(&mut self, v: Vector3T<T>) {
        *self = *self - v;
    }
}

impl<T: CsFloat> std::ops::Neg for Vector3T<T> {
    type Output = Vector3T<T>;
    /// `operator -(self)`
    fn neg(self) -> Vector3T<T> {
        Vector3T::from_doubles(-self.xd(), -self.yd(), -self.zd())
    }
}

impl<T: CsFloat> std::ops::Mul<Vector3T<T>> for Vector3T<T> {
    type Output = f64;
    /// `operator *(Vector3<T>, Vector3<T>)`: dot product.
    fn mul(self, v: Vector3T<T>) -> f64 {
        self.dot(&v)
    }
}

impl<T: CsFloat> std::ops::Mul<f64> for Vector3T<T> {
    type Output = Vector3T<T>;
    /// `operator *(Vector3<T> self, double v)`
    fn mul(self, v: f64) -> Vector3T<T> {
        Vector3T::from_doubles(self.xd() * v, self.yd() * v, self.zd() * v)
    }
}

impl<T: CsFloat> std::ops::MulAssign<f64> for Vector3T<T> {
    fn mul_assign(&mut self, v: f64) {
        *self = *self * v;
    }
}

impl<T: CsFloat> std::ops::Mul<Vector3T<T>> for f64 {
    type Output = Vector3T<T>;
    /// `operator *(double v, Vector3<T> self)`
    fn mul(self, v: Vector3T<T>) -> Vector3T<T> {
        v * self
    }
}

impl<T: CsFloat> std::ops::Div<f64> for Vector3T<T> {
    type Output = Vector3T<T>;
    /// `operator /(Vector3<T> self, double v)`
    fn div(self, v: f64) -> Vector3T<T> {
        Vector3T::from_doubles(self.xd() / v, self.yd() / v, self.zd() / v)
    }
}

impl<T: CsFloat> std::ops::DivAssign<f64> for Vector3T<T> {
    fn div_assign(&mut self, v: f64) {
        *self = *self / v;
    }
}

impl<T: CsFloat> std::ops::Rem for Vector3T<T> {
    type Output = Vector3T<T>;
    /// `operator %`: cross product.
    fn rem(self, v: Vector3T<T>) -> Vector3T<T> {
        self.cross(&v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arithmetic_and_rotation() {
        let a = Vector3::new(1.0, 2.0, 3.0);
        let b = Vector3::new(4.0, 5.0, 6.0);
        assert_eq!(a * b, 32.0);
        assert_eq!(a % b, Vector3::new(-3.0, 6.0, -3.0));
        let mut r = Vector3::new(1.0, 2.0, 3.0);
        r.rotate(Rotation::ROTATION_YAW_90).unwrap();
        assert_eq!(r, Vector3::new(-2.0, 1.0, 3.0));
        let mut r = Vector3::new(1.0, 2.0, 3.0);
        r.rotate(Rotation::ROTATION_ROLL_90_YAW_90).unwrap();
        // roll 90: (1, -3, 2); then yaw 90 reads the new y: (3, 1, 2)
        assert_eq!(r, Vector3::new(3.0, 1.0, 2.0));
        assert!(Vector3::new(0.0, 0.0, 0.0).rotate(Rotation(99)).is_err());
        assert!(Vector3::new(f64::NAN, 0.0, 0.0).equals(&Vector3::new(f64::NAN, 0.0, 0.0)));
        assert_eq!(a.to_string(), "Vector3<T>(1, 2, 3)");
        assert!(a.get(3).is_err());
    }
}
