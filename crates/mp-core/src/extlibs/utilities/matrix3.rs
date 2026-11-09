//! Port of `ExtLibs/Utilities/Matrix3.cs`

use super::utils::Utils;
use super::vector3::Vector3;
use crate::dotnet::{double_to_string, Exception};

/// `public class Matrix3`: a 3x3 matrix, intended as a rotation matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix3 {
    pub a: Vector3,
    pub b: Vector3,
    pub c: Vector3,
}

impl Default for Matrix3 {
    /// `new Matrix3()`: the identity.
    fn default() -> Self {
        let mut m = Matrix3 { a: Vector3::default(), b: Vector3::default(), c: Vector3::default() };
        m.identity();
        m
    }
}

impl Matrix3 {
    /// `new Matrix3()`: the identity.
    pub fn new() -> Self {
        Self::default()
    }

    /// `new Matrix3(Vector3 a, Vector3 b, Vector3 c)`
    pub fn from_rows(a: Vector3, b: Vector3, c: Vector3) -> Self {
        Matrix3 { a, b, c }
    }

    /// `new Matrix3(float a, ..., float i)`: the floats widen to double.
    #[allow(clippy::too_many_arguments)]
    pub fn from_floats(a: f32, b: f32, c: f32, d: f32, e: f32, f: f32, g: f32, h: f32, i: f32) -> Self {
        Matrix3::from_rows(
            Vector3::new(a as f64, b as f64, c as f64),
            Vector3::new(d as f64, e as f64, f as f64),
            Vector3::new(g as f64, h as f64, i as f64),
        )
    }

    /// `item(v1, v2)`
    pub fn item(&self, v1: i32, v2: i32) -> Result<f64, Exception> {
        self.get(v1, v2)
    }

    /// `this[int v1, int v2]` getter: "Invalid Matrix item" for a bad row, "Bad index" for a
    /// bad column.
    pub fn get(&self, v1: i32, v2: i32) -> Result<f64, Exception> {
        match v1 {
            0 => self.a.get(v2),
            1 => self.b.get(v2),
            2 => self.c.get(v2),
            _ => Err(Exception::Other("Invalid Matrix item".into())),
        }
    }

    /// `this[int v1, int v2]` setter
    pub fn set(&mut self, v1: i32, v2: i32, value: f64) -> Result<(), Exception> {
        match v1 {
            0 => self.a.set(v2, value),
            1 => self.b.set(v2, value),
            2 => self.c.set(v2, value),
            _ => Err(Exception::Other("Invalid Matrix item".into())),
        }
    }

    /// `identity()`
    pub fn identity(&mut self) {
        self.a = Vector3::new(1.0, 0.0, 0.0);
        self.b = Vector3::new(0.0, 1.0, 0.0);
        self.c = Vector3::new(0.0, 0.0, 1.0);
    }

    /// `transposed()`
    pub fn transposed(&self) -> Matrix3 {
        Matrix3::from_rows(
            Vector3::new(self.a.x, self.b.x, self.c.x),
            Vector3::new(self.a.y, self.b.y, self.c.y),
            Vector3::new(self.a.z, self.b.z, self.c.z),
        )
    }

    /// `from_euler(roll, pitch, yaw)`: fill the matrix from Euler angles in radians.
    pub fn from_euler(&mut self, roll: f64, pitch: f64, yaw: f64) {
        let cp = Utils::cos(pitch);
        let sp = Utils::sin(pitch);
        let sr = Utils::sin(roll);
        let cr = Utils::cos(roll);
        let sy = Utils::sin(yaw);
        let cy = Utils::cos(yaw);

        self.a.x = cp * cy;
        self.a.y = (sr * sp * cy) - (cr * sy);
        self.a.z = (cr * sp * cy) + (sr * sy);
        self.b.x = cp * sy;
        self.b.y = (sr * sp * sy) + (cr * cy);
        self.b.z = (cr * sp * sy) - (sr * cy);
        self.c.x = -sp;
        self.c.y = sr * cp;
        self.c.z = cr * cp;
    }

    /// `to_euler(ref roll, ref pitch, ref yaw)`: the Euler angles of the matrix as
    /// (roll, pitch, yaw).
    pub fn to_euler(&self) -> (f64, f64, f64) {
        let pitch = if self.c.x >= 1.0 {
            std::f64::consts::PI
        } else if self.c.x <= -1.0 {
            -std::f64::consts::PI
        } else {
            -Utils::asin(self.c.x)
        };
        let roll = Utils::atan2(self.c.y, self.c.z);
        let yaw = Utils::atan2(self.b.x, self.a.x);
        (roll, pitch, yaw)
    }

    /// `to_euler312()`
    pub fn to_euler312(&self) -> Vector3 {
        Vector3::new(Utils::asin(self.c.y), Utils::atan2(-self.c.x, self.c.z), Utils::atan2(-self.a.y, self.b.y))
    }

    /// `from_euler312(roll, pitch, yaw)`: the trig is single precision (`Utils.cosf`/`sinf`).
    pub fn from_euler312(&mut self, roll: f64, pitch: f64, yaw: f64) {
        let cp = Utils::cosf(pitch) as f64;
        let sp = Utils::sinf(pitch) as f64;
        let sr = Utils::sinf(roll) as f64;
        let cr = Utils::cosf(roll) as f64;
        let sy = Utils::sinf(yaw) as f64;
        let cy = Utils::cosf(yaw) as f64;

        self.a.x = cy * cp - sy * sr * sp;
        self.b.y = cy * cr;
        self.c.z = cp * cr;
        self.a.y = -cr * sy;
        self.a.z = sp * cy + cp * sr * sy;
        self.b.x = cp * sy + sp * sr * cy;
        self.b.z = sy * sp - sr * cy * cp;
        self.c.x = -sp * cr;
        self.c.y = sr;
    }

    /// `to_euler_yxz()`
    pub fn to_euler_yxz(&self) -> Vector3 {
        let cos_phi = (1.0 - self.c.y * self.c.y).sqrt();
        let phi = Utils::atan2(self.c.y, cos_phi);
        let omega = Utils::atan2(-self.c.x, self.c.z);
        let kappa = Utils::atan2(-self.a.y, self.b.y);

        Vector3::new(omega, phi, kappa)
    }

    /// `copy()`
    pub fn copy(&self) -> Matrix3 {
        Matrix3::from_rows(self.a.copy(), self.b.copy(), self.c.copy())
    }

    /// `rotate(Vector3 g)`: rotate the matrix by a given amount on 3 axes.
    pub fn rotate(&mut self, g: &Vector3) {
        let mut temp_matrix = Matrix3::new();

        temp_matrix.a.x = self.a.y * g.z - self.a.z * g.y;
        temp_matrix.a.y = self.a.z * g.x - self.a.x * g.z;
        temp_matrix.a.z = self.a.x * g.y - self.a.y * g.x;
        temp_matrix.b.x = self.b.y * g.z - self.b.z * g.y;
        temp_matrix.b.y = self.b.z * g.x - self.b.x * g.z;
        temp_matrix.b.z = self.b.x * g.y - self.b.y * g.x;
        temp_matrix.c.x = self.c.y * g.z - self.c.z * g.y;
        temp_matrix.c.y = self.c.z * g.x - self.c.x * g.z;
        temp_matrix.c.z = self.c.x * g.y - self.c.y * g.x;

        self.a += temp_matrix.a;
        self.b += temp_matrix.b;
        self.c += temp_matrix.c;
    }

    /// `rotateXY(Vector3 g)`
    pub fn rotate_xy(&mut self, g: &Vector3) {
        let mut temp_matrix = Matrix3::new();
        temp_matrix.a.x = -self.a.z * g.y;
        temp_matrix.a.y = self.a.z * g.x;
        temp_matrix.a.z = self.a.x * g.y - self.a.y * g.x;
        temp_matrix.b.x = -self.b.z * g.y;
        temp_matrix.b.y = self.b.z * g.x;
        temp_matrix.b.z = self.b.x * g.y - self.b.y * g.x;
        temp_matrix.c.x = -self.c.z * g.y;
        temp_matrix.c.y = self.c.z * g.x;
        temp_matrix.c.z = self.c.x * g.y - self.c.y * g.x;

        self.a += temp_matrix.a;
        self.b += temp_matrix.b;
        self.c += temp_matrix.c;
    }

    /// `rotateXYinv(Vector3 g)`
    pub fn rotate_xy_inv(&mut self, g: &Vector3) {
        let mut temp_matrix = Matrix3::new();
        temp_matrix.a.x = self.a.z * g.y;
        temp_matrix.a.y = -self.a.z * g.x;
        temp_matrix.a.z = -self.a.x * g.y + self.a.y * g.x;
        temp_matrix.b.x = self.b.z * g.y;
        temp_matrix.b.y = -self.b.z * g.x;
        temp_matrix.b.z = -self.b.x * g.y + self.b.y * g.x;
        temp_matrix.c.x = self.c.z * g.y;
        temp_matrix.c.y = -self.c.z * g.x;
        temp_matrix.c.z = -self.c.x * g.y + self.c.y * g.x;

        self.a += temp_matrix.a;
        self.b += temp_matrix.b;
        self.c += temp_matrix.c;
    }

    /// `normalize()`: re-normalise a rotation matrix.
    pub fn normalize(&mut self) {
        let error = self.a * self.b;
        let t0 = self.a - (self.b * (0.5 * error));
        let t1 = self.b - (self.a * (0.5 * error));
        let t2 = t0 % t1;
        self.a = t0 * (1.0 / t0.length());
        self.b = t1 * (1.0 / t1.length());
        self.c = t2 * (1.0 / t2.length());
    }

    /// `trace()`
    pub fn trace(&self) -> f64 {
        self.a.x + self.b.y + self.c.z
    }
}

impl std::fmt::Display for Matrix3 {
    /// `ToString()`: "Matrix3((ax, ay, az), (bx, by, bz), (cx, cy, cz))" in the current culture.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = |v: f64| double_to_string(v);
        write!(
            f,
            "Matrix3(({}, {}, {}), ({}, {}, {}), ({}, {}, {}))",
            s(self.a.x),
            s(self.a.y),
            s(self.a.z),
            s(self.b.x),
            s(self.b.y),
            s(self.b.z),
            s(self.c.x),
            s(self.c.y),
            s(self.c.z)
        )
    }
}

impl std::ops::Add for Matrix3 {
    type Output = Matrix3;
    fn add(self, m: Matrix3) -> Matrix3 {
        Matrix3::from_rows(self.a + m.a, self.b + m.b, self.c + m.c)
    }
}

impl std::ops::Sub for Matrix3 {
    type Output = Matrix3;
    fn sub(self, m: Matrix3) -> Matrix3 {
        Matrix3::from_rows(self.a - m.a, self.b - m.b, self.c - m.c)
    }
}

impl std::ops::Mul<Vector3> for Matrix3 {
    type Output = Vector3;
    /// `operator *(Matrix3 self, Vector3 v)`
    fn mul(self, v: Vector3) -> Vector3 {
        Vector3::new(
            self.a.x * v.x + self.a.y * v.y + self.a.z * v.z,
            self.b.x * v.x + self.b.y * v.y + self.b.z * v.z,
            self.c.x * v.x + self.c.y * v.y + self.c.z * v.z,
        )
    }
}

impl std::ops::Mul for Matrix3 {
    type Output = Matrix3;
    /// `operator *(Matrix3 self, Matrix3 m)`
    fn mul(self, m: Matrix3) -> Matrix3 {
        let s = self;
        Matrix3::from_rows(
            Vector3::new(
                s.a.x * m.a.x + s.a.y * m.b.x + s.a.z * m.c.x,
                s.a.x * m.a.y + s.a.y * m.b.y + s.a.z * m.c.y,
                s.a.x * m.a.z + s.a.y * m.b.z + s.a.z * m.c.z,
            ),
            Vector3::new(
                s.b.x * m.a.x + s.b.y * m.b.x + s.b.z * m.c.x,
                s.b.x * m.a.y + s.b.y * m.b.y + s.b.z * m.c.y,
                s.b.x * m.a.z + s.b.y * m.b.z + s.b.z * m.c.z,
            ),
            Vector3::new(
                s.c.x * m.a.x + s.c.y * m.b.x + s.c.z * m.c.x,
                s.c.x * m.a.y + s.c.y * m.b.y + s.c.z * m.c.y,
                s.c.x * m.a.z + s.c.y * m.b.z + s.c.z * m.c.z,
            ),
        )
    }
}

impl std::ops::Mul<f64> for Matrix3 {
    type Output = Matrix3;
    fn mul(self, v: f64) -> Matrix3 {
        Matrix3::from_rows(self.a * v, self.b * v, self.c * v)
    }
}

impl std::ops::Div<f64> for Matrix3 {
    type Output = Matrix3;
    fn div(self, v: f64) -> Matrix3 {
        Matrix3::from_rows(self.a / v, self.b / v, self.c / v)
    }
}

impl std::ops::Neg for Matrix3 {
    type Output = Matrix3;
    fn neg(self) -> Matrix3 {
        Matrix3::from_rows(-self.a, -self.b, -self.c)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn euler_round_trip() {
        let mut m = Matrix3::new();
        m.from_euler(0.1, 0.2, 0.3);
        let (r, p, y) = m.to_euler();
        assert!((r - 0.1).abs() < 1e-12 && (p - 0.2).abs() < 1e-12 && (y - 0.3).abs() < 1e-12);
        assert_eq!(Matrix3::new().trace(), 3.0);
        assert!(m.get(3, 0).is_err());
        assert_eq!(Matrix3::new().to_string(), "Matrix3((1, 0, 0), (0, 1, 0), (0, 0, 1))");
    }
}
